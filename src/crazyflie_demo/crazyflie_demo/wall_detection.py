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
from data_collector import DataCollector
from inference import Inference, get_approach_angle_fft
from estimators import DistanceEstimator, AngleEstimator
from moving_estimators import MovingEstimator

# dslice
FREQ = 3000  # mono frequency signal
N_MAX = 14  # how many distances to use

# flsice
N_WINDOW = 5
WALL_ANGLE_DEG = 90
DISTANCES_CM = DistanceEstimator.DISTANCES_M * 1e2
DIST_RANGE_CM = [min(DISTANCES_CM), max(DISTANCES_CM)]
N_CALIBRATION = 5
N_MICS = 4
ALGORITHM = "bayes"
DISTANCE_THRESHOLD_CM = 20
FLYING_HEIGHT_CM = 30
VELOCITY_CMS = 5 # cm

# TODO(FD) in the future, this should be done in gateway depending on the window chosen.
WINDOW_CORRECTION = 0.215579  #  flattop

class state(Enum):
    GROUND = 0
    HOVER = 1
    WAIT_DISTANCE = 2
    WAIT_ANGLE = 3
    WAIT_CALIB = 4
    AVOID_DISTANCE = 5
    AVOID_ANGLE = 6
    ABORT = 7

class mode(Enum):
    FSLICE = 0
    DSLICE = 1

MODE = mode.FSLICE
DRONE = False

class WallDetection(NodeWithParams):
    PARAMS_DICT = {
        "mode": mode.FSLICE.value,
        "drone": int(DRONE)
    }

    def __init__(self):
        super().__init__("wall_detection")

        self._action_client = ActionClient(self, CrazyflieCommands, "commands")
        self._action_server = ActionServer(self, StateMachine, "state_machine", self.server_callback)

        self.subscription_signals = self.create_subscription(
            SignalsFreq, "audio/signals_f", self.listener_callback, 10
        )
        self.pose_synch = TopicSynchronizer(10, self.get_logger())
        self.subscription_pose = self.create_subscription(
            PoseRaw, "geometry/pose_raw", self.pose_synch.listener_callback, 10,
        )
        self.dist_raw_synch = TopicSynchronizer(10, self.get_logger())
        self.subscription_dist_raw = self.create_subscription(
            Distribution,
            "results/distribution_raw",
            self.dist_raw_synch.listener_callback,
            10,
        )
        self.publisher_distribution_raw = self.create_publisher(
            Distribution, "results/distribution_raw", 10
        )
        self.publisher_distribution_moving = self.create_publisher(
            Distribution, "results/distribution_moving", 10
        )

        # self.start_time = None # use relative time since start
        self.start_time = 0  # use absolute time

        # initialize wall detection stuff

        self.data_collector = DataCollector()
        self.inf_machine = Inference()
        self.inf_machine.add_geometry(DIST_RANGE_CM, WALL_ANGLE_DEG)
        self.moving_estimator = MovingEstimator(n_window=N_WINDOW)

        self.calibration_count = 0
        self.calibration_data = np.empty((N_MICS, 0, 0))
        self.calibration = np.empty((N_MICS, 0))

        self.new_sample_to_treat = False
        self.distributions = {}

        self.state = state.GROUND
        self.state_by_server = None 

        # movement stuff
        self.velocity_ms = VELOCITY_CMS * 1e-2
        self.get_logger().info("Init okay")


    def add_to_calib(self, magnitudes):
        if self.calibration_count == 0:
            self.calibration_data = magnitudes[:, :, None]
            self.calibration_count += 1
        else:
            self.calibration_data = np.concatenate(
                [self.calibration_data, magnitudes[:, :, None]], axis=2
            )
            self.calibration_count += 1
            self.get_logger().info(f"Have added {self.calibration_count} frames to calib")

    def calibrate(self, magnitudes):
        # sometimes there can be a sample missing from the end of the signal...
        if len(self.calibration) != len(magnitudes):
            self.get_logger().warn("length mismatch! {len(magnitudes)}")
            calibration_here = self.calibration[:, :len(magnitudes)]
        else:
            calibration_here = self.calibration
        valid_freqs = np.all(calibration_here > 0, axis=0)
        return magnitudes[:, valid_freqs] / calibration_here[:, valid_freqs]

    def get_distance_distribution(self, magnitudes_calib, freqs):
        #self.get_logger().info(f"Adding to inf_machine: {magnitudes_calib.shape}, {freqs.shape}")
        self.inf_machine.add_data(
            magnitudes_calib, freqs,  # 4 x 32
        )
        distance_estimator = DistanceEstimator()
        for mic_i in range(magnitudes_calib.shape[0]):
            __, prob_mic, diff = self.inf_machine.do_inference(ALGORITHM, mic_i)
            distance_estimator.add_distribution(diff * 1e-2, prob_mic, mic_i)

        __, prob = distance_estimator.get_distance_distribution(
            distances_m=DISTANCES_CM * 1e-2, azimuth_deg=WALL_ANGLE_DEG
        )
        return DISTANCES_CM, np.array(prob)

    def get_angle_distribution(self, dslices, resolve_side=False):
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

    def listener_callback(self, msg_signals):
        if not self.state in [state.WAIT_DISTANCE, state.WAIT_ANGLE, state.WAIT_CALIB]:
            self.get_logger().warn("ignoring signal because state is {self.state}")
            return

        msg_pose = self.pose_synch.get_latest_message(msg_signals.timestamp)
        if msg_pose is not None:

            self.get_logger().info(f"treating new signal at time {msg_signals.timestamp}")
            timestamp = self.get_timestamp()
            r_world, v_world, yaw, yaw_rate = read_pose_raw_message(msg_pose)

            __, signals_f, freqs = read_signals_freq_message(msg_signals)
            magnitudes = np.abs(signals_f).T / WINDOW_CORRECTION  # 4 x 20

            if self.state == state.WAIT_CALIB:
                self.add_to_calib(magnitudes)
                return None

            elif self.state == state.WAIT_DISTANCE:
                magnitudes_calib = self.calibrate(magnitudes)
                assert magnitudes.shape == magnitudes_calib.shape
                distances_cm, probabilities = self.get_distance_distribution(magnitudes_calib, freqs)

                msg = create_distribution_message(distances_cm, probabilities, timestamp)
                self.publisher_distribution_raw.publish(msg)

                # combine multiple
                self.moving_estimator.add_distributions(
                    dist_cm=distances_cm,
                    dist_p=probabilities,
                    position_cm=r_world*1e2,
                    rot_deg=yaw,
                )
                if not self.moving_estimator.enough_measurements():
                    return None

                __, prob_moving = self.moving_estimator.get_distance_distribution(
                    distances_cm=distances_cm
                )
                msg = create_distribution_message(
                    distances_cm, np.array(prob_moving), timestamp
                )
                self.publisher_distribution_moving.publish(msg)
                self.get_logger().info("Published moving-average distribution")

                # TODO: can delete below if we work with subscribers instead
                self.new_sample_to_treat = True
                if np.any(prob_moving > 0):
                    self.distributions["wall_distance"] = (
                        DISTANCES_CM,
                        prob_moving,
                        timestamp,
                    )
                else:
                    self.get_logger().warn(f"not adding cause all-zero")

                distance_estimate = DISTANCES_CM[np.argmax(prob_moving)]
                self.get_logger().info(
                    f"Current distance estimate: {distance_estimate:.1f}cm"
                )
            elif self.state == state.WAIT_ANGLE:
                data = self.data_collector.get_current_distance_slice(
                    n_max=N_MAX, verbose=True
                )
                if data is not None:
                    d_slices, rel_distances_cm, *_ = data
                else:
                    self.get_logger().info("no measurements yet")
                    return

                if len(rel_distances_cm) < N_MAX:
                    self.get_logger().info(f"Not ready yet: {len(rel_distances_cm)}")
                    # return

                angles, prob = self.get_angle_distribution(dslices)


                # ANGLES_DEG is between 0 and 90, angles between 0 and 180. 
                msg = create_distribution_message(
                    angle_estimator.ANGLES_DEG, np.array(prob), timestamp
                )
                self.get_logger().info("Published raw distribution")
                self.publisher_distribution_raw.publish(msg)

                angle_estimate = angles[np.argmax(prob)]
                self.get_logger().info(f"Current angle estimate: {angle_estimate:.1f}deg")

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
            future = self.send_command("buzzer_idx", buzzer_idx)
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().warn("done")
        else:
            self.get_logger().warn("simulating buzzer")
            rclpy.spin_once(self)

    def take_off(self, height_m=0.4):
        if self.current_params["drone"] > 1:
            self.get_logger().info("taking off...")
            future = self.send_command("hover_height", height_m)
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info("done")
        else:
            self.get_logger().info(f"simulating takeoff")
            rclpy.spin_once(self)

    def land(self):
        if self.current_params["drone"] > 1:
            self.get_logger().info("landing...")
            future = self.send_command("land_velocity", 0.2)  # 50.0)
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info("done")
            self.get_logger().warn("exiting...")
        else:
            self.get_logger().info(f"simulating landing")
            rclpy.spin_once(self)

    def move_linear(self):
        if self.current_params["drone"] > 1:
            self.get_logger().info(f"moving with {self.velocity_ms:.2f}m/s...")
            future = self.send_command("move_forward", self.velocity_ms)
            rclpy.spin_until_future_complete(self, future)
            self.get_logger().info("done")
        else:
            self.get_logger().info(f"simulating moving with {self.velocity_ms:.2f}m/s")
            rclpy.spin_once(self)

    def execute_state_machine(self):
        if self.state == state.GROUND:
            self.take_off()
            return state.HOVER

        elif self.state == state.HOVER:
            if self.current_params["mode"] == mode.FSLICE.value:
                self.set_buzzer(1)
                self.get_logger().warn("calibrating")
                self.calibration_count = 0
                return state.WAIT_CALIB

            elif self.current_params["mode"] == mode.DSLICE.value:
                self.set_buzzer(3000)
                return state.WAIT_ANGLE
            else:
                raise ValueError(self.current_params["mode"])


        elif self.state == state.WAIT_ANGLE:
            self.move_linear()
            #if curr_dist is not None:
            curr_dist_message = self.dist_raw_synch.get_latest_message(timestamp)
            if curr_dist_message is not None:
                #angles, prob, timestamp = curr_dist
                angles = curr_dist_message.values
                prob = curr_dist_message.probabilities
                angle_estimate = angles[np.argmax(prob)]
                self.get_logger().info(f"wall at {angle_estimate} deg, continuing.")

                #TODO(FD): implement stopping criterion
                return state.WAIT_ANGLE
            return state.AVOID_ANGLE

        elif self.state == state.WAIT_CALIB:
            # wait until calibration is done
            if self.calibration_count < N_CALIBRATION:
                rclpy.spin_once(self)
                return state.WAIT_CALIB

            self.calibration = np.median(self.calibration_data, axis=2)
            self.get_logger().warn("done calibrating")
            return state.WAIT_DISTANCE

        elif self.state == state.WAIT_DISTANCE:
            self.move_linear()

            if not self.new_sample_to_treat:
                return state.WAIT_DISTANCE
            self.new_sample_to_treat = False

            timestamp = self.get_timestamp()
            # curr_dist = self.distributions.get("wall_distance", None)
            curr_dist_message = self.dist_raw_synch.get_latest_message(timestamp)
            if curr_dist_message is not None:
                dist = curr_dist_message.values
                prob = curr_dist_message.probabilities
                timestamp = curr_dist_message.timestamp
                distance_estimate = dist[np.argmax(prob)]
                if distance_estimate < DISTANCE_THRESHOLD_CM:
                    self.get_logger().info(
                        f"WALL AT {distance_estimate}, TURNING AROUND!"
                    )
                    return state.AVOID_DISTANCE
                else:
                    self.get_logger().info(f"wall at {distance_estimate}, continuing.")
            else:
                self.get_logger().info("no distribution yet")
            return state.WAIT_DISTANCE

        elif self.state == state.AVOID_DISTANCE:
            # invert velocity
            self.velocity_ms = -self.velocity_ms
            return state.WAIT_DISTANCE

        elif self.state == state.AVOID_ANGLE:
            self.velocity_ms = -self.velocity_ms
            return state.WAIT_ANGLE
        else:
            raise ValueError(self.state)

    def main(self):
        self.get_logger().info("Starting main")
        while self.state != state.ABORT:
            self.state = self.execute_state_machine()

            # check if in the meantime the server was called
            if self.state_by_server is not None: 
                self.get_logger().info(f"Overwriting {self.state} with {self.state_by_server}")
                self.state = self.state_by_server
                self.state_by_server = None

        # land and exit
        self.set_buzzer(0)
        self.land()
        return

    def sleep(self, time_s):
        self.get_logger().info(f"sleeping for {time_s}...")
        time.sleep(time_s)

    def get_timestamp(self):
        curr_time = int(time.process_time() * 1e3)  # ms
        if self.start_time is None:
            self.start_time = curr_time
        return curr_time - self.start_time

    def send_command(self, command_name, command_value=0.0):
        goal_msg = CrazyflieCommands.Goal()
        goal_msg.command_name = command_name
        goal_msg.command_value = float(command_value)
        timestamp = int(time.time() * 1e-3)
        goal_msg.timestamp = timestamp

        if self.current_params["drone"] > 0:
            self._action_client.wait_for_server(timeout_sec=5)

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return self._send_goal_future

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
    except:
        action_client.set_buzzer(0)
        action_client.land()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
