from enum import Enum
import sys
import time

import numpy as np

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.action import ActionServer

from audio_interfaces.action import CrazyflieCommands, StateMachine
from audio_interfaces.msg import PoseRaw, SignalsFreq, Distribution
from audio_interfaces_py.messages import (
    read_signals_freq_message,
    read_pose_raw_message,
    create_distribution_message,
)
from audio_interfaces_py.node_with_params import NodeWithParams
from audio_stack.topic_synchronizer import TopicSynchronizer

sys.path.append("python/")
from utils.data_collector import DataCollector
from utils.inference import Inference, get_approach_angle_fft
from utils.estimators import DistanceEstimator, AngleEstimator
from utils.moving_estimators import MovingEstimator


# dslice
FREQ = 3000  # mono frequency signal
N_MAX = 14  # how many distances to use

# flsice
PUBLISH_MOVING = True
PUBLISH_RAW = False

N_WINDOW = 5
WALL_ANGLE_DEG = 90  # for raw distribution only
DISTANCES_CM = DistanceEstimator.DISTANCES_M * 1e2
DIST_RANGE_CM = [min(DISTANCES_CM), max(DISTANCES_CM)]
N_CALIBRATION = 3  # 10
N_MICS = 4
ALGORITHM = "bayes"
DISTANCE_THRESHOLD_CM = 20

# movement stuff
FLYING_HEIGHT_CM = 30
VELOCITY_CMS = 5  # linear constant velocity in cm / s
TIME_BLIND_FLIGHT = 0  # 10 # seconds


class State(Enum):
    GROUND = 0
    HOVER = 1
    WAIT_DISTANCE = 2
    WAIT_ANGLE = 3
    WAIT_CALIB = 4
    AVOID_DISTANCE = 5
    AVOID_ANGLE = 6
    ABORT = 7
    BLIND_FLIGHT = 8


class Mode(Enum):
    FSLICE = 0
    DSLICE = 1


MODE = Mode.FSLICE
DRONE = False


def get_distance_distribution(diff_dict):
    distance_estimator = DistanceEstimator()
    for mic_i, (diff, prob_mic) in diff_dict.items():
        distance_estimator.add_distribution(diff * 1e-2, prob_mic, mic_i)
    __, prob = distance_estimator.get_distance_distribution(
        distances_m=DISTANCES_CM * 1e-2, azimuth_deg=WALL_ANGLE_DEG
    )
    return DISTANCES_CM, prob


def get_angle_distribution(dslices, resolve_side=False):
    angle_estimator = AngleEstimator()
    # d_slices is of shape 4x20, rel_distances_cm of shape 20
    for mic_idx in range(N_MICS):
        angles, proba = get_approach_angle_fft(
            d_slice=d_slices[mic_idx],  # shape 4 x 20
            frequency=FREQ,
            relative_distances_cm=rel_distances_cm,
            bayes=True,
            reduced=True,
        )
        angle_estimator.add_distribution(angles, proba, mic_idx, FREQ)

    if resolve_side:
        # from 0 to 90 or 90 to 180
        angles_augment, prob_augment = angle_estimator.get_angle_distribution(
            mics_left_right=[[1], [3]]
        )
        angles = np.arange(180)
        prob = np.zeros(180)
        prob[angles_augment] = prob_augment
    else:
        angles, prob = angle_estimator.get_angle_distribution()
    return angles, prob


class WallDetection(NodeWithParams):
    PARAMS_DICT = {"mode": Mode.FSLICE.value, "drone": int(DRONE)}

    def __init__(self):
        super().__init__("wall_detection")

        self._action_client = ActionClient(self, CrazyflieCommands, "commands")
        self._action_client_wall = ActionClient(
            self, StateMachine, "state_machine_wall"
        )
        self._action_server = ActionServer(
            self, StateMachine, "state_machine", self.server_callback
        )

        self.pose_synch = TopicSynchronizer(20, self.get_logger(), n_buffer=10)
        self.subscription_pose = self.create_subscription(
            PoseRaw, "geometry/pose_raw", self.pose_synch.listener_callback, 10,
        )
        self.subscription_signals = self.create_subscription(
            SignalsFreq, "audio/signals_f", self.listener_callback, 10
        )

        # self.dist_raw_synch = TopicSynchronizer(10, self.get_logger())
        if PUBLISH_RAW:
            self.publisher_distribution_raw = self.create_publisher(
                Distribution, "results/distribution_raw", 10
            )
        if PUBLISH_MOVING:
            self.publisher_distribution_moving = self.create_publisher(
                Distribution, "results/distribution_moving", 10
            )

        # self.start_time = None # use relative time since start
        self.start_time = 0  # use absolute time
        self.start_timestamp = None

        # initialize wall detection stuff
        self.data_collector = DataCollector()
        self.inf_machine = Inference()
        self.inf_machine.add_geometry(DIST_RANGE_CM, WALL_ANGLE_DEG)
        self.moving_estimator = MovingEstimator(n_window=N_WINDOW)

        self.calibration_count = 0
        self.calibration_data = np.empty((N_MICS, 0, 0))
        self.calibration = np.empty((N_MICS, 0))

        self.new_sample_to_treat = False

        self.state = State.GROUND
        self.state_by_server = None

        # movement stuff
        self.velocity_ms = VELOCITY_CMS * 1e-2

    def add_to_calib(self, magnitudes):
        if self.calibration_count == 0:
            self.calibration_data = magnitudes[:, :, None]
            self.calibration_count += 1
        else:
            self.calibration_data = np.concatenate(
                [self.calibration_data, magnitudes[:, :, None]], axis=2
            )
            self.calibration_count += 1
            self.get_logger().info(
                f"Have added {self.calibration_count} frames to calib"
            )

    def calibrate(self, magnitudes):
        # sometimes there can be a sample missing from the end of the signal...
        if len(self.calibration) != len(magnitudes):
            self.get_logger().warn("length mismatch! {len(magnitudes)}")
            calibration_here = self.calibration[:, : len(magnitudes)]
        else:
            calibration_here = self.calibration
        valid_freqs = np.all(calibration_here > 0, axis=0)
        return magnitudes[:, valid_freqs] / calibration_here[:, valid_freqs]

    def get_raw_distributions(self, magnitudes_calib, freqs):
        diff_dict = {}
        self.inf_machine.add_data(
            magnitudes_calib, freqs,  # 4 x 32
        )
        for mic_i in range(magnitudes_calib.shape[0]):
            __, prob_mic, diff = self.inf_machine.do_inference(ALGORITHM, mic_i)
            diff_dict[mic_i] = (diff, prob_mic)
        return diff_dict

    def listener_callback(self, msg_signals):
        if not self.state in [State.WAIT_DISTANCE, State.WAIT_ANGLE, State.WAIT_CALIB]:
            # self.get_logger().warn(f"ignoring signal because state is {self.state}")
            return

        # use the audio signal timestamp as reference for this measurement.
        timestamp = msg_signals.timestamp

        # Sometimes, this callback is called while the other TopicSynchronizer has not
        # registered the latest messages yet. therefore, we add a little buffer
        msg_pose = None
        i = 0
        timeout = 3
        while msg_pose is None:
            msg_pose = self.pose_synch.get_latest_message(timestamp, verbose=False)
            i += 1
            if i > timeout:
                self.get_logger().warn("did not register valid pose")
                break
        # msg_pose = self.pose_synch.get_latest_message(timestamp, verbose=True)

        if msg_pose is not None:
            self.get_logger().warn(
                f"for audio {timestamp}, using pose {msg_pose.timestamp}. lag: {msg_pose.timestamp - timestamp}ms"
            )
            r_world, v_world, yaw, yaw_rate = read_pose_raw_message(msg_pose)
            if r_world[2] < 0.3:
                self.get_logger().warn("Not flying.")
                return

            __, signals_f, freqs = read_signals_freq_message(msg_signals)
            magnitudes = np.abs(signals_f).T  # 4 x 20

            if self.state == State.WAIT_CALIB:
                self.add_to_calib(magnitudes)
                return None
            elif self.state == State.WAIT_DISTANCE:
                magnitudes_calib = self.calibrate(magnitudes)
                assert magnitudes.shape == magnitudes_calib.shape

                diff_dict = self.get_raw_distributions(magnitudes_calib, freqs)

                if PUBLISH_RAW:
                    distances_cm, probabilities = get_distance_distribution(diff_dict)
                    msg = create_distribution_message(
                        distances_cm, probabilities, timestamp
                    )
                    self.publisher_distribution_raw.publish(msg)
                    self.get_logger().info(
                        f"Published raw distribution with timestamp {msg.timestamp}"
                    )

                if PUBLISH_MOVING:
                    self.moving_estimator.add_distributions(
                        diff_dict, position_cm=r_world * 1e2, rot_deg=yaw,
                    )
                    probabilities, __ = self.moving_estimator.get_distributions(
                        local=True
                    )

                    msg = create_distribution_message(
                        self.moving_estimator.DISTANCES_CM, probabilities, timestamp
                    )
                    self.publisher_distribution_moving.publish(msg)
                    self.get_logger().info(
                        f"Published moving-average distribution with timestamp {timestamp}"
                    )

                self.new_sample_to_treat = True

            elif self.state == State.WAIT_ANGLE:
                data = self.data_collector.get_current_distance_slice(
                    n_max=N_MAX, verbose=False
                )
                if data is not None:
                    d_slices, rel_distances_cm, *_ = data
                else:
                    self.get_logger().info("No measurements yet")
                    return

                if len(rel_distances_cm) < N_MAX:
                    self.get_logger().warn(f"Not ready yet: {len(rel_distances_cm)}")

                angles, prob = get_angle_distribution(dslices)

                # ANGLES_DEG is between 0 and 90, angles between 0 and 180.
                msg = create_distribution_message(
                    angle_estimator.ANGLES_DEG, np.array(prob), timestamp
                )
                self.get_logger().info("Published raw distribution")
                self.publisher_distribution_raw.publish(msg)

                angle_estimate = angles[np.argmax(prob)]
                self.get_logger().info(
                    f"Current angle estimate: {angle_estimate:.1f}deg, probability:{np.max(prob)}"
                )

    def server_callback(self, goal_handle):
        msg = goal_handle.request
        # find which enum the state corresponds to
        self.state_by_server = state(msg.state)
        self.get_logger().info(
            f"Action received: {msg.state} which corresponds to {self.state_by_server}"
        )
        feedback = StateMachine.Feedback()
        goal_handle.publish_feedback(feedback)
        goal_handle.succeed()

        result = StateMachine.Result()
        result.flag = 1
        result.message = f"Changed to {self.state_by_server}"
        return result

    def set_buzzer(self, buzzer_idx):
        if self.current_params["drone"] > 0:
            self.get_logger().warn("setting buzzer...")
            self.send_command("buzzer_idx", buzzer_idx)
            self.get_logger().warn("done")
        else:
            self.get_logger().warn("simulating buzzer")
            rclpy.spin_once(self)

    def take_off(self, height_m=0.4):
        if self.current_params["drone"] > 1:
            self.get_logger().warn("taking off...")
            self.send_command("hover_height", height_m)
            self.get_logger().warn("done")
        else:
            self.get_logger().info(f"simulating takeoff")
            rclpy.spin_once(self)

    def land(self):
        if self.current_params["drone"] > 1:
            self.get_logger().info("landing...")
            future = self.send_command("land_velocity", 0.2)  # 50.0)
            self.get_logger().info("done")
            self.get_logger().warn("exiting...")
        else:
            self.get_logger().info(f"simulating landing")
            rclpy.spin_once(self)

    def move_linear(self):
        if self.current_params["drone"] > 1:
            self.get_logger().info(f"moving with {self.velocity_ms:.2f}m/s...")
            future = self.send_command("move_forward", self.velocity_ms)
            self.get_logger().info("done")
        else:
            self.get_logger().info(f"simulating moving with {self.velocity_ms:.2f}m/s")
            rclpy.spin_once(self)

    def execute_state_machine(self):
        rclpy.spin_once(self)
        if self.state == State.GROUND:
            self.take_off()
            return State.HOVER
            # self.get_logger().warn("abort")
            # return State.ABORT

        elif self.state == State.HOVER:
            if self.current_params["mode"] == Mode.FSLICE.value:
                self.set_buzzer(1)
                self.get_logger().warn("calibrating")
                self.calibration_count = 0
                return State.WAIT_CALIB

            elif self.current_params["mode"] == Mode.DSLICE.value:
                self.set_buzzer(3000)
                return State.WAIT_ANGLE
            else:
                raise ValueError(self.current_params["mode"])

        elif self.state == State.WAIT_ANGLE:
            self.move_linear()
            # if curr_dist is not None:
            timestamp = self.get_timestamp()
            curr_dist_message = self.dist_raw_synch.get_latest_message(timestamp)
            if curr_dist_message is not None:
                # angles, prob, timestamp = curr_dist
                angles = curr_dist_message.values
                prob = curr_dist_message.probabilities
                angle_estimate = angles[np.argmax(prob)]
                self.get_logger().info(f"wall at {angle_estimate} deg, continuing.")

                # TODO(FD): implement stopping criterion
                # return State.AVOID_ANGLE
            return State.WAIT_ANGLE

        elif self.state == State.WAIT_CALIB:
            # wait until calibration is done
            if self.calibration_count < N_CALIBRATION:
                rclpy.spin_once(self)
                return State.WAIT_CALIB

            self.calibration = np.median(self.calibration_data, axis=2)
            self.get_logger().info("done calibrating")
            return State.WAIT_DISTANCE

        elif self.state == State.WAIT_DISTANCE:
            self.move_linear()

            if not self.new_sample_to_treat:
                self.get_logger().info("staying in WAIT_DISTANCE cause no new data")
                return State.WAIT_DISTANCE

            self.new_sample_to_treat = False

            future = self.ask_about_wall()
            self.get_logger().warn(f"Action result: {future.message}")
            return State[future.result().flag]

        elif self.state == State.AVOID_DISTANCE:
            # invert velocity
            self.velocity_ms = -self.velocity_ms

            # start a few seconds of blind flight to not retrigger
            self.get_logger().warn("start blind flight")
            self.start_blind = time.time()
            return State.BLIND_FLIGHT

        elif self.state == State.BLIND_FLIGHT:
            self.move_linear()
            if time.time() - self.start_blind < TIME_BLIND_FLIGHT:
                return State.BLIND_FLIGHT
            self.get_logger().warn("stop blind flight")
            return State.WAIT_DISTANCE

        elif self.state == State.AVOID_ANGLE:
            self.velocity_ms = -self.velocity_ms
            return State.WAIT_ANGLE
        else:
            raise ValueError(self.state)

    def main(self):
        self.get_logger().info("Starting main")
        while self.state != State.ABORT:
            self.state = self.execute_state_machine()

            # check if in the meantime the server was called
            if self.state_by_server is not None:
                self.get_logger().info(
                    f"Overwriting {self.state} with {self.state_by_server}"
                )
                self.state = self.state_by_server
                self.state_by_server = None

        # land and exit
        self.set_buzzer(0)
        self.land()
        return

    def sleep(self, time_s):
        self.get_logger().info(f"sleeping for {time_s}...")
        time.sleep(time_s)

    def get_seconds(self, timestamp_ms):
        if self.start_timestamp is None:
            self.start_timestamp = timestamp_ms * 1e-3
        return round(timestamp_ms * 1e-3 - self.start_timestamp, 3)

    def get_timestamp(self):
        curr_time = int(time.process_time() * 1e3)  # ms
        if self.start_time is None:
            self.start_time = curr_time
        return curr_time - self.start_time

    def send_command(self, command_name, command_value=0.0):
        goal_msg = CrazyflieCommands.Goal()
        goal_msg.command_name = command_name
        goal_msg.command_value = float(command_value)
        timestamp = self.get_timestamp()
        goal_msg.timestamp = timestamp

        if self.current_params["drone"] > 0:
            self._action_client.wait_for_server()  # timeout_sec=0.5)

        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        return future

        # self._send_goal_future = self._action_client.send_goal_async(
        #    goal_msg, feedback_callback=self.feedback_callback
        # )
        # self._send_goal_future.add_done_callback(self.goal_response_callback)
        # return self._send_goal_future

    def ask_about_wall(self):
        self.get_logger().warn(f"Sending current state {self.state} to wall_mapper...")
        goal_msg = StateMachine.Goal()
        goal_msg.state = self.state.value

        future = self._action_client_wall.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().warn("... done!")
        return future

    def feedback_callback(self, feedback):
        self.get_logger().warn(feedback.message)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("goal_response_callback: Command rejected")
            return

        # self.get_logger().info("goal_response_callback: Command accepted")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        # self.get_logger().info(f"get_result_callback: Result: {result.message}")

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info(f"Received feedback: {feedback.message}")


def main(args=None):
    rclpy.init(args=args)
    action_client = WallDetection()
    try:
        action_client.main()
    except Exception as e:
        print(e)
        action_client.set_buzzer(0)
        action_client.land()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
