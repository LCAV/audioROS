import sys
import time

import numpy as np

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from audio_interfaces.action import CrazyflieCommands
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

DRYRUN = False

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

# TODO(FD) in the future, this should be done in gateway depending on the window chosen.
WINDOW_CORRECTION = 0.215579  #  flattop

SCHEME = "dslice_debug"
# SCHEME = "fslice"
# SCHEME = "fslice_hover"
# SCHEME = "fslice_debug"


class WallDetection(NodeWithParams):
    PARAMS_DICT = {}
    #    "mode": "user-input"  #
    # "mode": "automatic"
    # }

    def __init__(self):
        super().__init__("wall_detection")

        self._action_client = ActionClient(self, CrazyflieCommands, "commands")

        if "fslice" in SCHEME:
            self.subscription_signals = self.create_subscription(
                SignalsFreq, "audio/signals_f", self.listener_callback_fslice, 10
            )
        else:
            self.subscription_signals = self.create_subscription(
                SignalsFreq, "audio/signals_f", self.listener_callback_dslice, 10
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
        self.calibration = np.empty((N_MICS, 0, 0))

        self.new_sample_to_treat = False
        self.distributions = {}

    def listener_callback_fslice(self, msg_signals):
        msg_pose = self.pose_synch.get_latest_message(msg_signals.timestamp)
        if msg_pose is not None:

            timestamp = self.get_timestamp()
            r_world, v_world, yaw, yaw_rate = read_pose_raw_message(msg_pose)

            __, signals_f, freqs = read_signals_freq_message(msg_signals)
            magnitudes = np.abs(signals_f).T / WINDOW_CORRECTION  # 4 x 20

            if self.calibration_count == 0:
                self.calibration = magnitudes[:, :, None]
                self.calibration_count += 1
                return None
            elif self.calibration_count < N_CALIBRATION:
                self.calibration = np.concatenate(
                    [self.calibration, magnitudes[:, :, None]], axis=2
                )
                self.calibration_count += 1
                return None

            calibration = np.median(self.calibration, axis=2)
            magnitudes_calib = magnitudes

            # sometimes there can be a sample missing from the end of the signal...
            if len(calibration) != len(magnitudes_calib):
                self.get_logger().warn("length mismatch! {len(magnitudes_calib)}")
                calibration_here = calibration[:, : len(magnitudes_calib)]
            else:
                calibration_here = calibration

            magnitudes_calib[calibration_here > 0] /= calibration_here[
                calibration_here > 0
            ]

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
            msg = create_distribution_message(DISTANCES_CM, np.array(prob), timestamp)
            self.publisher_distribution_raw.publish(msg)
            # self.get_logger().info("Published raw distribution")

            self.moving_estimator.add_distributions(
                dist_cm=DISTANCES_CM,
                dist_p=prob,
                position_cm=r_world * 1e2,
                rot_deg=yaw,
            )
            if not self.moving_estimator.enough_measurements():
                return None

            __, prob_moving = self.moving_estimator.get_distance_distribution(
                distances_cm=DISTANCES_CM
            )
            msg = create_distribution_message(
                DISTANCES_CM, np.array(prob_moving), timestamp
            )
            self.publisher_distribution_moving.publish(msg)
            # self.get_logger().info("Published moving-average distribution")

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
            self.new_sample_to_treat = True

    def listener_callback_dslice(self, msg_signals):
        msg_pose = self.pose_synch.get_latest_message(msg_signals.timestamp)
        if msg_pose is not None:
            timestamp = self.get_timestamp()
            r_world, v_world, yaw, yaw_rate = read_pose_raw_message(msg_pose)

            __, signals_f, freqs = read_signals_freq_message(msg_signals)
            signals_f = signals_f / WINDOW_CORRECTION  # 4 x 20

            position_cm = r_world * 1e2
            # self.get_logger().info(
            #        f"Filling {position_cm[1]} {np.abs(signals_f[15:19, 0])} {freqs[15:19]}"
            # )
            self.data_collector.fill_from_signal(
                signals_f.T,
                freqs,
                distance_cm=position_cm[1],
                time=timestamp,
                mode=FREQ,
            )
            # self.get_logger().info(
            #    f"Number of measurements: {self.data_collector.get_n_measurement_times()}"
            # )

            data = self.data_collector.get_current_distance_slice(n_max=N_MAX)
            if data is not None:
                d_slices, rel_distances_cm, *_ = data
            else:
                self.get_logger().info("no measurements yet")
                return

            if len(rel_distances_cm) < N_MAX:
                self.get_logger().info(f"Not ready yet: {len(rel_distances_cm)}")
                return

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

            angles, prob = angle_estimator.get_angle_distribution(
                mics_left_right=[[1], [3]]
            )
            msg = create_distribution_message(
                angle_estimator.ANGLES_DEG, np.array(prob), timestamp
            )
            self.get_logger().info("Published raw distribution")
            self.publisher_distribution_raw.publish(msg)

            angle_estimate = angles[np.argmax(prob)]
            self.get_logger().info(f"Current angle estimate: {angle_estimate:.1f}deg")

    def do_fslices(self):
        self.get_logger().info("taking off...")
        future = self.send_command("hover_height", 0.4)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("done")

        self.get_logger().info("buzzer...")
        future = self.send_command("buzzer_idx", 1)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("done")

        # wait until calibration is done
        self.get_logger().info("calibrating...")
        self.n_samples = 0
        t1 = time.time()
        while self.n_samples < (N_CALIBRATION + N_WINDOW):
            if time.time() - t1 > 30:
                self.get_logger().info("calibration timeout")
                break
            rclpy.spin_once(self)
            if self.new_sample_to_treat:
                self.n_samples += 1
        self.get_logger().info("done")

        self.get_logger().info("forward...")
        future = self.send_command("move_forward", 0.04)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("done")

        t1 = time.time()
        while 1:
            if time.time() - t1 > 30:
                self.get_logger().info("wall detection timeout")
                break

            rclpy.spin_once(self)
            if not self.new_sample_to_treat:
                continue
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
                    break
                else:
                    self.get_logger().info(f"wall at {distance_estimate}, continuing.")
            else:
                self.get_logger().info("no distribution yet")

        self.get_logger().info("buzzer...")
        future = self.send_command("buzzer_idx", 0)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("done")

        self.get_logger().info("landing...")
        future = self.send_command("land_velocity", 0.2)  # 50.0)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("done")
        self.get_logger().warn("exiting...")
        return

    def sleep(self, time_s):
        self.get_logger().info(f"sleeping for {time_s}...")
        time.sleep(time_s)

    def get_timestamp(self):
        curr_time = int(time.process_time() * 1e3)  # ms
        if self.start_time is None:
            self.start_time = curr_time
        return curr_time - self.start_time

    def do_fslices_process(self):
        while 1:
            rclpy.spin_once(self)
            if not self.new_sample_to_treat:
                continue
            self.new_sample_to_treat = False

            timestamp = self.get_timestamp()
            curr_dist = self.distributions.get("wall_distance", None)
            curr_dist_message = self.dist_raw_synch.get_latest_message(timestamp)
            if curr_dist_message is not None:
                dist = curr_dist_message.values
                prob = curr_dist_message.probabilities
                self.get_logger()
                dist2, prob2, timestamp2 = curr_dist
                self.get_logger().info(
                    f"from message:{timestamp}, from variable:{timestamp2}"
                )
                distance_estimate = dist[np.argmax(prob)]
                if distance_estimate < DISTANCE_THRESHOLD_CM:
                    self.get_logger().info(
                        f"WALL AT {distance_estimate}, TURNING AROUND!"
                    )
                else:
                    self.get_logger().info(f"wall at {distance_estimate}, continuing.")
            else:
                self.get_logger().info("no distribution yet")

    def do_fslices_hover(self):
        self.get_logger().info("taking off...")
        future = self.send_command("hover_height", 0.4)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("done")

        self.get_logger().info("buzzer...")
        future = self.send_command("buzzer_idx", 1)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("done")

        # wait until calibration is done
        self.get_logger().info("calibrating...")
        self.n_samples = 0
        t1 = time.time()
        while self.n_samples < (N_CALIBRATION + N_WINDOW):
            if time.time() - t1 > 30:
                self.get_logger().info("calibration timeout")
                break
            rclpy.spin_once(self)
            if self.new_sample_to_treat:
                self.n_samples += 1
        self.get_logger().info("done")

        while 1:
            self.get_logger().info("moving forward")
            future = self.send_command("move_distance", 0.05)  # 5cm
            rclpy.spin_until_future_complete(self, future)

            future = self.send_command("buzzer_idx", 0)
            rclpy.spin_until_future_complete(self, future)
            time.sleep(0.5)
            future = self.send_command("buzzer_idx", 3)
            rclpy.spin_until_future_complete(self, future)

            n_samples = 0
            t1 = time.time()
            while n_samples < 3:
                if time.time() - t1 > 30:
                    self.get_logger().info("wall detection timeout")
                    break

                rclpy.spin_once(self)

                if not self.new_sample_to_treat:
                    continue

                self.new_sample_to_treat = False

                n_samples += 1
                # self.get_logger().info(f"sample ({n_samples}/3)")

                curr_dist = self.distributions.get("wall_distance", None)
                if curr_dist is not None:
                    dist, prob = curr_dist
                    distance_estimate = dist[np.argmax(prob)]
                    if distance_estimate < DISTANCE_THRESHOLD_CM:
                        self.get_logger().info(
                            f"WALL AT {distance_estimate}, TURNING AROUND!"
                        )
                    else:
                        self.get_logger().info(
                            f"wall at {distance_estimate}, continuing."
                        )
                else:
                    self.get_logger().info("no distribution yet")
                    continue

        self.get_logger().info("landing...")
        future = self.send_command("land_velocity", 0.2)  # 50.0)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("done")
        return

    # detect wall using mono frequency
    def do_dslices(self):
        self.get_logger().info("taking off...")
        future = self.send_command("hover_height", 0.4)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("done")

        self.get_logger().info("buzzer...")
        future = self.send_command("buzzer_idx", 3000)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("done")

        self.get_logger().info("forward...")
        future = self.send_command("move_forward", 0.05)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("done")
        # wait until calibration is done

        t1 = time.time()
        while 1:
            if time.time() - t1 > 20:
                self.get_logger().info("wall detection timeout")
                break

            rclpy.spin_once(self)
            if not self.new_sample_to_treat:
                continue

            self.new_sample_to_treat = False

            curr_dist = self.distributions.get("wall_approach", None)
            if curr_dist is not None:
                angles, prob = curr_dist
                angle_estimate = angles[np.argmax(prob)]
                self.get_logger().info(f"wall at {angle_estimate} deg, continuing.")

        future = self.send_command("buzzer_idx", 0)
        rclpy.spin_until_future_complete(self, future)

        future = self.send_command("land_velocity", 0.2)  # 50.0)
        rclpy.spin_until_future_complete(self, future)
        return

    def do_dslices_process(self):
        while 1:
            rclpy.spin_once(self)
            if not self.new_sample_to_treat:
                continue

            self.new_sample_to_treat = False

            curr_dist = self.distributions.get("wall_approach", None)
            curr_dist_message = self.dist_raw_synch.get_latest_message(timestamp)
            if curr_dist_message is not None:
                angles, prob, timestamp = curr_dist
                angle_estimate = angles[np.argmax(prob)]
                self.get_logger().info(f"wall at {angle_estimate} deg, continuing.")

        future = self.send_command("buzzer_idx", 0)
        rclpy.spin_until_future_complete(self, future)

        future = self.send_command("land_velocity", 0.2)  # 50.0)
        rclpy.spin_until_future_complete(self, future)
        return

    def send_command(self, command_name, command_value=0.0):
        goal_msg = CrazyflieCommands.Goal()
        goal_msg.command_name = command_name
        goal_msg.command_value = float(command_value)
        timestamp = int(time.time() * 1e-3)
        goal_msg.timestamp = timestamp

        if not DRYRUN:
            self._action_client.wait_for_server()

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
    if SCHEME == "fslice_debug":
        action_client.do_fslices_process()
    elif SCHEME == "dslice_debug":
        action_client.do_dslices_process()
    # action_client.do_fslices()
    # action_client.do_fslices_hover()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
