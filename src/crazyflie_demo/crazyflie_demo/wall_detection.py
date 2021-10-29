from enum import Enum
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
from inference import Inference
from estimators import DistanceEstimator
from moving_estimators import MovingEstimator

DRYRUN = True

N_WINDOW = 3
WALL_ANGLE_DEG = 90
DISTANCES_CM = DistanceEstimator.DISTANCES_M * 1e2
DIST_RANGE_CM = [min(DISTANCES_CM), max(DISTANCES_CM)]
N_CALIBRATION = 5
N_MICS = 4
ALGORITHM = "bayes"
DISTANCE_THRESHOLD_CM = 20
FLYING_HEIGHT_CM = 30


class states(Enum):
    CALIBRATE = 0
    INFERENCE = 1


class WallDetection(NodeWithParams):
    PARAMS_DICT = {}
    #    "mode": "user-input"  #
    # "mode": "automatic"
    # }

    def __init__(self):
        super().__init__("wall_detection")

        self._action_client = ActionClient(self, CrazyflieCommands, "commands")

        self.subscription_signals = self.create_subscription(
            SignalsFreq, "audio/signals_f", self.listener_callback, 10
        )
        self.pose_synch = TopicSynchronizer(10)
        self.subscription_pose = self.create_subscription(
            PoseRaw, "geometry/pose_raw", self.pose_synch.listener_callback, 10,
        )
        self.publisher_distribution_raw = self.create_publisher(
            Distribution, "results/distribution_raw", 10
        )
        self.publisher_distribution_moving = self.create_publisher(
            Distribution, "results/distribution_moving", 10
        )

        # initialize wall detection stuff
        self.data_collector = DataCollector()
        self.inf_machine = Inference()
        self.distance_estimator = DistanceEstimator()
        self.inf_machine.add_geometry(DIST_RANGE_CM, WALL_ANGLE_DEG)
        self.moving_estimator = MovingEstimator(n_window=N_WINDOW)

        self.calibration_count = 0
        self.calibration = np.empty((N_MICS, 0, 0))

        self.distributions = {}

    def listener_callback(self, msg_signals):
        timestamp = msg_signals.timestamp
        msg_pose = self.pose_synch.get_latest_message(timestamp, self.get_logger())
        if msg_pose is not None:
            r_world, v_world, yaw, yaw_rate = read_pose_raw_message(msg_pose)
            if r_world[2] < FLYING_HEIGHT_CM * 1e-2:
                self.get_logger().info(f"Not flying: {r_world}")
                return
            self.get_logger().info(
                f"Processing signals at time {timestamp}, position {r_world}"
            )

            __, signals_f, freqs = read_signals_freq_message(msg_signals)
            magnitudes = np.abs(signals_f)

            if self.calibration_count == 0:
                self.calibration = magnitudes[:, :, None]
                self.calibration_count += 1
                return
            elif self.calibration_count <= N_CALIBRATION:
                self.calibration = np.concatenate(
                    [self.calibration, magnitudes[:, :, None]], axis=2
                )
                self.calibration_count += 1

            calibration = np.median(self.calibration, axis=2)
            magnitudes_calib = magnitudes
            magnitudes_calib[calibration > 0] /= calibration[calibration > 0]

            # calibrate
            self.inf_machine.add_data(
                magnitudes_calib.T,  # 4 x 32
                freqs,
                # distances
            )

            for mic_i in range(magnitudes_calib.shape[1]):
                __, prob_mic, diff = self.inf_machine.do_inference(ALGORITHM, mic_i)
                self.distance_estimator.add_distribution(diff * 1e-2, prob_mic, mic_i)

            __, prob = self.distance_estimator.get_distance_distribution(
                distances_m=DISTANCES_CM * 1e-2, azimuth_deg=WALL_ANGLE_DEG
            )
            msg = create_distribution_message(DISTANCES_CM, prob, timestamp)
            self.publisher_distribution_raw.publish(msg)
            self.get_logger().info("Published raw distribution")

            self.moving_estimator.add_distributions(
                dist_cm=DISTANCES_CM,
                dist_p=prob,
                position_cm=r_world * 1e2,
                rot_deg=yaw,
            )
            __, prob_moving = self.moving_estimator.get_distance_distribution(
                distances_cm=DISTANCES_CM
            )
            msg = create_distribution_message(DISTANCES_CM, prob_moving, timestamp)
            self.publisher_distribution_moving.publish(msg)
            self.get_logger().info("Published moving-average distribution")

            if np.any(prob_moving > 0):
                self.distributions["wall"] = (DISTANCES_CM, prob_moving)
            else:
                self.get_logger().warn(f"not adding cause all-zero")

            distance_estimate = DISTANCES_CM[np.argmax(prob_moving)]
            self.get_logger().info(
                f"Current distance estimate: {distance_estimate:.1f}cm"
            )

    # detect wall using continuous sweeps
    def do_fslices(self):
        self.get_logger().info("taking off...")
        future = self.send_command("takeoff")
        if not DRYRUN:
            rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("done")

        self.get_logger().info("forward...")
        future = self.send_command("forward")
        if not DRYRUN:
            rclpy.spin_until_future_complete(self, future)

        while 1:
            rclpy.spin_once(self)
            curr_dist = self.distributions.get("wall", None)
            if curr_dist is not None:
                dist, prob = curr_dist
                distance_estimate = dist[np.argmax(prob)]

                # if distance_estimate < DISTANCE_THRESHOLD_CM:
                #    self.get_logger().info(f'WALL AT {distance_estimate}, TURNING AROUND!')
                # else:
                #    self.get_logger().info(f'wall at {distance_estimate}, continuing.')

            else:
                # self.get_logger().info('no distribution yet')
                continue
        future = self.send_command("turn", 180.0)
        if not DRYRUN:
            rclpy.spin_until_future_complete(self, future)

        future = self.send_command("forward", 50.0)
        if not DRYRUN:
            rclpy.spin_until_future_complete(self, future)
        rclpy.shutdown()

    # detect wall using mono frequency
    def do_dslice(self):
        pass

    def send_command(self, command_name, command_value=0.0):
        goal_msg = CrazyflieCommands.Goal()
        goal_msg.command_name = command_name
        goal_msg.command_value = command_value
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
            self.get_logger().info("Command rejected")
            return

        self.get_logger().info("Command accepted")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result.message}")
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Received feedback: {feedback.message}")


def main(args=None):
    rclpy.init(args=args)
    action_client = WallDetection()
    action_client.do_fslices()
    # rclpy.spin(action_client)


if __name__ == "__main__":
    main()
