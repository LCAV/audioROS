from copy import deepcopy
from enum import Enum
import sys
import time

import numpy as np

import rclpy
from rclpy.action import ActionClient
from rclpy.action import ActionServer

from audio_interfaces.srv import CrazyflieCommands, StateMachine
from audio_interfaces.msg import PoseRaw, SignalsFreq, Distribution
from audio_interfaces_py.messages import (
    read_signals_freq_message,
    read_pose_raw_message,
    create_distribution_message,
)
from audio_interfaces_py.node_with_params import NodeWithParams
from audio_stack.topic_synchronizer import TopicSynchronizer

from crazyflie_description_py.parameters import FLYING_HEIGHT_CM

sys.path.append("python/")
from utils.data_collector import DataCollector
from utils.inference import Inference, get_approach_angle_fft
from utils.inference import get_angle_distribution as get_beamform_distribution

from utils.estimators import DistanceEstimator, AngleEstimator
from utils.moving_estimators import MovingEstimator
from utils.particle_estimators import ParticleEstimator
from utils.split_particle_estimators import SplitParticleEstimator
from utils.histogram_estimators import HistogramEstimator

# dslice
FREQ = 3000  # mono frequency signal
N_MAX = 14  # how many distances to use

PUBLISH_MOVING = True
PUBLISH_RAW = False

# corresponds to discretization "fine":
DISTANCES_CM = np.arange(7, 80, step=2)
ANGLES_DEG = np.arange(360, step=10)

WALL_ANGLE_DEG = 90  # for raw distribution only
N_MICS = 4
ALGORITHM = "bayes"
DISTANCE_THRESHOLD_CM = 20

# movement stuff
VELOCITY_CMS = 4  # linear constant velocity in cm / s
TIME_BLIND_FLIGHT = 0  # seconds, set to 0 for no effect
TIME_FORWARD = 60  # seconds, time to move forward in BLIND mode

MAG_THRESH = 1e-3


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
    BLIND = 2


MODE = Mode.FSLICE
DRONE = 0  # 0: no drone, 1: buzzer only, 2: flying


def get_distance_distribution(diff_dict):
    distance_estimator = DistanceEstimator()
    for mic_i, (diff, prob_mic) in diff_dict.items():
        distance_estimator.add_distribution(diff * 1e-2, prob_mic, mic_i)
    dists_cm, prob = distance_estimator.get_distance_distribution(
        angle_deg=WALL_ANGLE_DEG
    )
    return dists_cm, prob


def get_angle_distribution(dslices, resolve_side=False):
    angle_estimator = AngleEstimator()
    # d_slices is of shape 4x20, rel_distances_cm of shape 20
    for mic_idx in range(dslices.shape[0]):
        angles, proba = get_approach_angle_fft(
            d_slice=d_slices[mic_idx],  # shape 4 x 20
            frequency=FREQ,
            relative_distances_cm=rel_distances_cm,
            bayes=True,
            reduced=True,
        )
        angle_estimator.add_distribution(angles, proba, mic_idx, FREQ)

    if resolve_side:
        # output is from from 0 to 90 or 90 to 180
        angles_augment, prob_augment = angle_estimator.get_angle_distribution(
            mics_left_right=[[1], [3]]
        )
        angles = np.arange(180)
        prob = np.zeros(180)
        prob[angles_augment] = prob_augment
    else:
        angles, prob = angle_estimator.get_angle_distribution()
    return angles, prob


class PythonLogger(object):
    """Simple wrapper for ROS2 logger that doesn't require ROS"""

    def warn(self, msg):
        print("Warn:", msg)

    def info(self, msg):
        print("Info:", msg)

    def error(self, msg):
        print("Error:", msg)


class WallDetection(NodeWithParams):
    PARAMS_DICT = {"mode": Mode.FSLICE.value, "drone": int(DRONE)}

    BEAMFORM = False

    MASK_BAD = "fixed"
    # MASK_BAD = "adaptive"
    # MASK_BAD = None

    BAD_FREQ_RANGES = [[0, 3100], [3600, 3800]]  # removes two frequencies
    OUTLIER_FACTOR = None  # reject values outside of OUTLIER_FACTOR * std window

    # need at least two, otherwise end up with all-ones magnitudes_calib.
    N_CALIBRATION = 2
    CALIBRATION = "iir"
    ALPHA_IIR = 0.3  # 1: overwrite with new data, 1: ignore new data. The higher, the shorter the window.
    # N_CALIBRATION = 10
    # CALIBRATION = "window"

    # N_CALIBRATION = 10
    # CALIBRATION= "fixed"

    # if set to true, we use simplify the angles, meaning that we use
    # the forward direction as our angle estimate. If we do not simplify
    # angles, we simply use a uniform prior on the angle estimate.
    SIMPLIFY_ANGLES = False

    # estimator variables
    N_WINDOW = 5
    RELATIVE_MOVEMENT_STD = 0.0
    N_PARTICLES = 100

    def __init__(self, python_only=False, estimator="moving", angles_deg=ANGLES_DEG):
        if not python_only:
            print("initializing ros stuff")
            super().__init__("wall_detection")
            self.logger = self.get_logger()

            self._client_command = self.create_client(CrazyflieCommands, "send_command")
            self._client_wall = self.create_client(StateMachine, "check_wall")
            self.already_asking = (
                False  # semaphore to make sure we only ask once at a time
            )
            self._server_state = self.create_service(
                StateMachine, "change_state_manual", self.change_state_manual_callback
            )

            # Note that for this usecase,  n_buffer>1 doesn't work because the pose synchronizer
            # creates exactly one matching message.
            self.signals_synch = TopicSynchronizer(20, self.logger, n_buffer=1)
            self.subscription_signals = self.create_subscription(
                SignalsFreq, "audio/signals_f", self.signals_synch.listener_callback, 10
            )
            self.subscription_pose = self.create_subscription(
                PoseRaw,
                "geometry/pose_raw_synch",
                self.listener_callback,
                10,
            )

            if PUBLISH_RAW:
                self.publisher_distribution_raw = self.create_publisher(
                    Distribution, "results/distribution_raw", 10
                )
            if PUBLISH_MOVING:
                self.publisher_distribution_moving = self.create_publisher(
                    Distribution, "results/distribution_moving", 10
                )
            timer_period = 0.1  # seconds
            self.timer = self.create_timer(timer_period, self.main)

        else:
            self.current_params = self.PARAMS_DICT
            self.logger = PythonLogger()

        # self.start_time = None # use relative time since start
        self.start_time = 0  # use absolute time
        self.start_timestamp = None

        if estimator == "moving":
            self.estimator = MovingEstimator(
                n_window=self.N_WINDOW,
                distances_cm=DISTANCES_CM,
                angles_deg=angles_deg,
                relative_movement_std=self.RELATIVE_MOVEMENT_STD,
            )
        elif estimator == "histogram":
            self.estimator = HistogramEstimator(
                distances_cm=DISTANCES_CM,
                angles_deg=angles_deg,
            )
        elif estimator == "particle":
            self.estimator = ParticleEstimator(
                n_particles=self.N_PARTICLES, 
                angles_deg=angles_deg, 
                distances_cm=DISTANCES_CM
            )
        elif estimator == "split_particle":
            if not self.BEAMFORM:
                raise ValueError("Need beamforming to use split particle filter")
            self.estimator = SplitParticleEstimator(
                n_particles=self.N_PARTICLES, 
                angles_deg=angles_deg, 
                distances_cm=DISTANCES_CM
            )
        elif type(estimator) in [MovingEstimator, ParticleEstimator, HistogramEstimator, SplitParticleEstimator]:
            if type(estimator) == SplitParticleEstimator:
                assert self.BEAMFORM, "Need beamforming to use split particle filter"
            self.estimator = estimator
        else:
            raise ValueError(estimator)

        # initialize wall detection stuff
        self.data_collector = DataCollector()
        self.inf_machine = Inference()
        distance_range_cm = [
            self.estimator.distances_cm[0],
            self.estimator.distances_cm[-1],
        ]
        self.inf_machine.add_geometry(distance_range_cm, WALL_ANGLE_DEG)

        self.calibration_count = 0

        self.calibration = None
        self.calibrationsq = None
        self.calibration_data = None

        self.new_sample_to_treat = False

        self.state = State.GROUND
        self.state_by_server = None

        # movement stuff
        self.velocity_ms = VELOCITY_CMS * 1e-2

        # start main timer
        self.start_forward = None

    def flight_check(self, position_cm=None):
        """
        drone=0: analyzing bag file, should be "flying" to do analysis
        drone=1: only buzzer, for debugging, don't mind if not flying
        drone=2: full experiment, has to be flying.
        """
        if self.current_params["drone"] == 1:
            return True
        if (position_cm is not None) and (position_cm[2] < FLYING_HEIGHT_CM):
            return False
        elif any(np.isnan(position_cm)):
            return False
        return True

    def add_to_calib(self, magnitudes):
        if self.CALIBRATION == "fixed":
            if self.calibration_data is None:
                self.add_to_calib_data(magnitudes)
            elif self.calibration_data.shape[2] < self.N_CALIBRATION:
                self.add_to_calib_data(magnitudes)
            return
        elif self.CALIBRATION == "window":
            self.add_to_calib_data(magnitudes, verbose=False)
        elif self.CALIBRATION == "iir":
            self.add_to_calib_iir(magnitudes)

    def add_to_calib_data(self, magnitudes, verbose=False):
        if self.calibration_data is None:
            if verbose:
                print("adding column 0")
            self.calibration_data = magnitudes[:, :, None]
        elif self.calibration_count >= self.calibration_data.shape[2]:
            if verbose:
                print(f"adding column {self.calibration_data.shape[2]}")
            self.calibration_data = np.concatenate(
                [self.calibration_data, magnitudes[:, :, None]], axis=2
            )
        else:
            assert (
                self.CALIBRATION != "fixed"
            ), f"Trying to fill more than requested {self.N_CALIBRATION}!"
            if verbose:
                print(f"filling at {self.calibration_count}")
            self.calibration_data[:, :, self.calibration_count] = magnitudes
        self.calibration_count = (self.calibration_count + 1) % self.N_CALIBRATION

    def add_to_calib_iir(self, magnitudes):
        if self.calibration is None:
            self.calibration = magnitudes
            self.calibrationsq = magnitudes**2
        else:
            valid = ~np.isnan(self.calibration) 
            self.calibration[valid] = (1 - self.ALPHA_IIR) * self.calibration[
                valid
            ] + self.ALPHA_IIR * magnitudes[valid]
            self.calibrationsq[valid] = (1 - self.ALPHA_IIR) * self.calibrationsq[
                valid
            ] + self.ALPHA_IIR * (magnitudes[valid] ** 2)

            self.calibration[~valid] = magnitudes[~valid]
            self.calibrationsq[~valid] = magnitudes[~valid] ** 2
        self.calibration_count += 1

    def calibrate(self, magnitudes):
        """Note that invalid values are masked (set to zero)."""
        if self.CALIBRATION in ("fixed", "window"):
            if self.calibration is None:
                if self.calibration_data.shape[2] <= 1:
                    # self.logger.error(f"No calibration data yet. Returning raw magnitudes. ")
                    return magnitudes

                if (
                    self.CALIBRATION == "fixed"
                    and self.calibration_data.shape[2] != self.N_CALIBRATION
                ):
                    # self.logger.warn(f"Not enough calibration data yet: {self.calibration_data.shape[2]} != {self.N_CALIBRATION}")
                    pass

            # treat invalid frequencies (nan for all mics and all times) especially, because otherwise
            # nanmedian raises suspicious warning.
            freqs_all_nan = np.all(np.isnan(self.calibration_data), axis=(0, 2))
            self.calibration = np.full(self.calibration_data.shape[:2], np.nan)
            self.calibration_std = np.full(self.calibration_data.shape[:2], np.nan)
            self.calibration[:, ~freqs_all_nan] = np.nanmedian(
                self.calibration_data[:, ~freqs_all_nan], axis=2
            )
            self.calibration_std[:, ~freqs_all_nan] = np.nanstd(
                self.calibration_data[:, ~freqs_all_nan], axis=2
            )
            self.calibration_std[self.calibration_std == 0] = np.nan

        elif self.CALIBRATION == "iir":
            if self.calibration is None:
                self.logger.error(
                    f"No calibration data yet. Returning raw magnitudes. "
                )
                return magnitudes

            var = self.calibrationsq - self.calibration**2
            self.calibration_std = np.full(var.shape, np.nan)
            self.calibration_std[var > 0] = np.sqrt(var[var > 0])
            self.calibration_std[self.calibration_std == 0] = np.nan

        if self.calibration.shape[1] != magnitudes.shape[1]:
            self.logger.error("length mismatch in calibration")
            return None

        valid = ~np.isnan(self.calibration)
        magnitudes_calib = deepcopy(magnitudes)
        magnitudes_calib[valid] /= self.calibration[valid]
        magnitudes_calib[~valid] = np.nan
        return magnitudes_calib

    def get_raw_distributions(self, magnitudes_calib, freqs):
        diff_dict = {}
        self.inf_machine.add_data(
            magnitudes_calib,
            freqs,  # 4 x 32
        )
        for mic_i in range(magnitudes_calib.shape[0]):
            __, prob_mic, diff = self.inf_machine.do_inference(ALGORITHM, mic_i)
            diff_dict[mic_i] = (diff, prob_mic)
        return diff_dict

    def mask_bad_measurements(self, magnitudes, freqs, verbose=False):
        magnitudes[magnitudes <= MAG_THRESH] = np.nan
        if self.MASK_BAD == "fixed":
            remove = np.zeros(magnitudes.shape[1], dtype=bool)
            for freq_range in self.BAD_FREQ_RANGES:
                remove |= (freq_range[0] <= freqs) & (freqs <= freq_range[1])
            magnitudes[:, remove] = np.nan
            return magnitudes
        elif self.MASK_BAD == "adaptive":
            if self.calibration is None:
                return magnitudes
            elif not np.any(self.calibration_std > 0):
                return magnitudes
            else:
                remove = np.abs(magnitudes - self.calibration) >= (
                    self.calibration_std * self.OUTLIER_FACTOR
                )
                remove[np.isnan(self.calibration)] = False
                if np.sum(remove) == np.size(remove):
                    print("Not removing all measurements.")
                elif np.sum(remove):
                    if verbose:
                        print(
                            f"Removing {np.sum(remove)} of {np.size(remove)} elements"
                        )
                        xx, yy = np.where(remove.T)
                        freqs_i = np.unique(xx)
                        mics_per_freq = np.split(
                            yy, np.unique(xx, return_index=True)[1][1:]
                        )
                        removed = "\n".join(
                            [
                                f"  freq {freqs[f]} for mics {m}"
                                for f, m in zip(freqs_i, mics_per_freq)
                            ]
                        )
                        print(f"Removed elements: \n{removed}")
                    magnitudes[remove] = np.nan
                return magnitudes
        elif self.MASK_BAD is None:
            return magnitudes
        else:
            raise ValueError(self.MASK_BAD)

    def listener_callback_offline(
        self, signals_f, freqs, position_cm, yaw_deg, timestamp=0
    ):
        if not self.flight_check(position_cm):
            # print("did not pass flight check:", position_cm)
            return
        from audio_stack.parameters import WINDOW_CORRECTION

        magnitudes = np.abs(signals_f).T  # 4 x 20
        # just a formality to compare bag file to csv file.
        magnitudes *= WINDOW_CORRECTION[2]

        magnitudes = self.mask_bad_measurements(magnitudes, freqs)

        self.add_to_calib(magnitudes)

        magnitudes_calib = self.calibrate(deepcopy(magnitudes))
        if magnitudes_calib is None:
            print("could not calibrate")
            return

        try:
            diff_dict = self.get_raw_distributions(magnitudes_calib, freqs)
        except Exception as e:
            print("could not process calibrated magnitudes:", e)
            raise e
            return None

        return_dict = {}

        # get the distance probabilities without considering movement
        dist_static, prob_dist_static = get_distance_distribution(diff_dict)
        return_dict["dist_static"] = dist_static
        return_dict["prob_dist_static"] = prob_dist_static

        # get distance and angle probabilities, considering movement.
        # diff_dict is in centimeters
        self.estimator.add_distributions(
            diff_dict,
            position_cm=position_cm,
            rot_deg=yaw_deg,
        )
        if self.BEAMFORM:
            angle_static, prob_angle_static = get_beamform_distribution(
                signals_f, freqs, self.estimator.context.mics.T
            )
            self.estimator.add_angle_distribution(angle_static, prob_angle_static)
            return_dict["angle_static"] = angle_static
            return_dict["prob_angle_static"] = prob_angle_static

        (
            dist_moving,
            prob_dist_moving,
            angle_moving,
            prob_angle_moving,
        ) = self.estimator.get_distributions(simplify_angles=self.SIMPLIFY_ANGLES)
        return_dict["angle_moving"] = angle_moving
        return_dict["prob_angle_moving"] = prob_angle_moving
        return_dict["dist_moving"] = dist_moving
        return_dict["prob_dist_moving"] = prob_dist_moving

        return return_dict
        # return (
        #    dist_static,
        #    prob_dist_static,
        #    prob_dist_moving,
        #    prob_angle_moving,
        # )

    def listener_callback(self, msg_pose):
        if not self.state in [State.WAIT_DISTANCE, State.WAIT_ANGLE, State.WAIT_CALIB]:
            # self.logger.warn(f"ignoring signal because state is {self.state}")
            return
        timestamp = msg_pose.timestamp

        msg_signals = self.signals_synch.get_latest_message(timestamp, verbose=False)
        if msg_signals is not None:
            # self.logger.warn(
            #    f"for pose {timestamp}, using audio {msg_signals.timestamp}. lag: {msg_signals.timestamp - timestamp}ms"
            # )
            r_world, v_world, yaw, yaw_rate = read_pose_raw_message(msg_pose)
            position_cm = r_world * 1e2

            if not self.flight_check(position_cm):
                return

            __, signals_f, freqs = read_signals_freq_message(msg_signals)
            magnitudes = np.abs(signals_f).T  # 4 x 20

            if self.state == State.WAIT_ANGLE:
                data = self.data_collector.get_current_distance_slice(
                    n_max=N_MAX, verbose=False
                )
                if data is not None:
                    d_slices, rel_distances_cm, *_ = data
                else:
                    self.logger.info("No measurements yet")
                    return

                if len(rel_distances_cm) < N_MAX:
                    self.logger.warn(f"Not ready yet: {len(rel_distances_cm)}")

                angles, prob = get_angle_distribution(dslices)

                # ANGLES_DEG is between 0 and 90, angles between 0 and 180.
                msg = create_distribution_message(
                    angle_estimator.ANGLES_DEG, np.array(prob), timestamp
                )
                self.logger.info("Published raw distribution")
                self.publisher_distribution_raw.publish(msg)

                angle_estimate = angles[np.argmax(prob)]
                self.logger.info(
                    f"Current angle estimate: {angle_estimate:.1f}deg, probability:{np.max(prob)}"
                )
                return

            self.add_to_calib(magnitudes, self.state == State.WAIT_CALIB)

            magnitudes_calib = self.calibrate(magnitudes)

            diff_dict = self.get_raw_distributions(magnitudes_calib, freqs)

            if PUBLISH_RAW:
                distances_cm, probabilities = get_distance_distribution(diff_dict)
                msg = create_distribution_message(
                    distances_cm, probabilities, timestamp
                )
                self.publisher_distribution_raw.publish(msg)
                self.logger.info(
                    f"Published raw distribution with timestamp {msg.timestamp}"
                )

            if PUBLISH_MOVING:
                self.estimator.add_distributions(
                    diff_dict,
                    position_cm=r_world * 1e2,
                    rot_deg=yaw,
                )
                (
                    distances_cm,
                    probabilities,
                    *_,
                ) = self.estimator.get_distributions(simplify_angles=SIMPLIFY_ANGLES)
                # self.logger.warn(f"{timestamp}, after adding {r_world[0] * 1e2:.2f}, {yaw:.1f}: {probabilities[2]}")

                # TODO(FD) to save time, we could consider only publishing the distribution
                # if it contains viable distance estimates.
                msg = create_distribution_message(
                    distances_cm, probabilities, timestamp
                )
                self.publisher_distribution_moving.publish(msg)
                self.logger.info(
                    f"Published moving-average distribution with timestamp {timestamp}"
                )
            self.new_sample_to_treat = True

        else:
            self.logger.error(f"Did not find valid audio for pose {timestamp}")

    def change_state_manual_callback(self, request, response):
        # find which enum the state corresponds to
        self.state_by_server = State(request.state)
        self.logger.info(
            f"Action received: {request.state} which corresponds to {self.state_by_server}"
        )
        response.flag = 1
        response.message = f"Manually changed to {self.state_by_server}"
        return response

    def set_buzzer(self, buzzer_idx):
        if self.current_params["drone"] > 0:
            self.logger.warn("setting buzzer...")
            self.send_command("buzzer_idx", buzzer_idx)
            self.logger.warn("...done")
        else:
            self.logger.warn("simulating buzzer")

    def take_off(self, height_m=0.4):
        if self.current_params["drone"] > 1:
            self.logger.warn("Taking off...")
            self.send_command("hover_height", height_m)
            self.logger.warn("...done")
        else:
            self.logger.info(f"simulating takeoff")

    def land(self):
        if self.current_params["drone"] > 1:
            self.logger.info("landing...")
            future = self.send_command("land_velocity", 0.2)  # 50.0)
            self.logger.info("done")
            self.logger.warn("exiting...")
        else:
            self.logger.info(f"simulating landing")

    def move_linear(self):
        if self.current_params["drone"] > 1:
            self.logger.info(f"moving with {self.velocity_ms:.2f}m/s...")
            try:
                future = self.send_command("move_forward", self.velocity_ms)
            except Exception as e:
                self.logger.warn(f"Error when trying to move forward: {e}")
            else:
                self.logger.info("...done")
        else:
            self.logger.info(f"simulating moving with {self.velocity_ms:.2f}m/s")

    def execute_state_machine(self):
        if self.state == State.GROUND:
            self.take_off()
            return State.HOVER
            # self.logger.warn("abort")
            # return State.ABORT

        elif self.state == State.HOVER:
            if self.current_params["mode"] in (Mode.FSLICE.value, Mode.BLIND.value):
                self.set_buzzer(1)
                self.calibration_count = 0
                return State.WAIT_CALIB
            elif self.current_params["mode"] == Mode.DSLICE.value:
                self.set_buzzer(3000)
                return State.WAIT_ANGLE
            else:
                raise ValueError(self.current_params["mode"])

        elif self.state == State.WAIT_ANGLE:
            self.logger.error("wait_angle is not implemented yet")
            return State.ABORT
            self.move_linear()
            # if curr_dist is not None:
            timestamp = self.get_timestamp()
            curr_dist_message = self.dist_raw_synch.get_latest_message(timestamp)
            if curr_dist_message is not None:
                # angles, prob, timestamp = curr_dist
                angles = curr_dist_message.values
                prob = curr_dist_message.probabilities
                angle_estimate = angles[np.argmax(prob)]
                self.logger.info(f"wall at {angle_estimate} deg, continuing.")

                # TODO(FD): implement stopping criterion
                # return State.AVOID_ANGLE
            return State.WAIT_ANGLE

        elif self.state == State.WAIT_CALIB:
            # wait until calibration is done
            if self.calibration_count < self.N_CALIBRATION:
                return State.WAIT_CALIB

            self.calibration = np.nanmedian(self.calibration_data, axis=2)
            self.calibration_std = np.nanstd(self.calibration_data, axis=2)
            self.calibration_std[self.calibration_std == 0] = np.nan

            self.logger.info("done calibrating")
            self.start_forward = time.time()
            return State.WAIT_DISTANCE

        elif self.state == State.WAIT_DISTANCE:
            self.move_linear()

            if self.current_params["mode"] == Mode.BLIND.value:
                if (time.time() - self.start_forward) > TIME_FORWARD:
                    return State.AVOID_DISTANCE
                else:
                    return State.WAIT_DISTANCE

            if not self.new_sample_to_treat:
                self.logger.info("staying in WAIT_DISTANCE cause no new data")
                return State.WAIT_DISTANCE
            self.new_sample_to_treat = False

            if not self.already_asking:
                self.ask_about_wall()
            # if there is a wall in sight, the attribute state_by_server
            # is changed, and below will be overwritten in main.
            return State.WAIT_DISTANCE

        elif self.state == State.AVOID_DISTANCE:
            # invert velocity
            self.velocity_ms = -self.velocity_ms

            # start a few seconds of blind flight to not retrigger immediately
            self.start_blind = time.time()
            return State.BLIND_FLIGHT

        elif self.state == State.BLIND_FLIGHT:
            self.move_linear()
            if time.time() - self.start_blind < TIME_BLIND_FLIGHT:
                return State.BLIND_FLIGHT

            self.start_forward = time.time()
            return State.WAIT_DISTANCE

        elif self.state == State.AVOID_ANGLE:
            self.velocity_ms = -self.velocity_ms
            return State.WAIT_ANGLE
        else:
            raise ValueError(self.state)

    def main(self):
        # land and exit
        if self.state == State.ABORT:
            self.logger.error(f"Received ABORT state. Setting buzzer to 0 and landing.")
            self.set_buzzer(0)
            self.land()
            self.destroy_node()
            rclpy.shutdown()
            return

        state = self.execute_state_machine()
        if state != self.state:
            self.logger.warn(f"Change state from {self.state} to {state}")
            self.state = state

        # check if in the meantime the server was called
        if self.state_by_server is not None:
            self.logger.info(f"Overwriting {self.state} with {self.state_by_server}")
            self.state = self.state_by_server
            self.state_by_server = None

    def sleep(self, time_s):
        self.logger.info(f"sleeping for {time_s}...")
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
        request = CrazyflieCommands.Request()
        request.command_name = command_name
        request.command_value = float(command_value)
        timestamp = self.get_timestamp()
        request.timestamp = timestamp

        if self.current_params["drone"] > 0:
            ok = self._client_command.wait_for_service(timeout_sec=3)
            if not ok:
                self.logger.warn(f"timeout while waiting for commands service")

        self.future_command = self._client_command.call_async(request)
        self.future_command.add_done_callback(self.get_result_command)

    def get_result_command(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.logger.error(f"get_result_command failed: {e}")
        else:
            value = response.value
            msg = response.message
            self.logger.info(
                f"get_result_command received response message:{msg}, value:{value}"
            )

    def ask_about_wall(self):
        self.already_asking = True
        request = StateMachine.Request()
        request.state = self.state.value

        ok = self._client_wall.wait_for_service(timeout_sec=3)
        if not ok:
            self.logger.warn(f"timeout while waiting for wall service")

        self.future_wall = self._client_wall.call_async(request)
        self.future_wall.add_done_callback(self.get_result_wall)

    def get_result_wall(self, future):  # state_machine
        try:
            response = future.result()
        except Exception as e:
            self.logger.error(f"get_result_wall failed: {e}")
        else:
            new_state_int = response.flag
            if new_state_int < 0:
                self.logger.warn(f"get_result_wall received -1, doing nothing.")
            self.state_by_server = State(new_state_int)
            if self.state_by_server == State.AVOID_DISTANCE:
                self.logger.warn(f"Avoiding detected wall!")
            self.already_asking = False

    def print_params(self):
        print("calibration:", self.CALIBRATION)
        print("n_calibration:", self.N_CALIBRATION)
        print("alpha_iir:", self.ALPHA_IIR)
        print("mask bad:", self.MASK_BAD )
        print("outlier factor:", self.OUTLIER_FACTOR )
        print("n_window:", self.N_WINDOW)
        print("relative_movement_std:", self.RELATIVE_MOVEMENT_STD)
        print("simplify_angles:", self.SIMPLIFY_ANGLES )
        print("beamform:", self.BEAMFORM)
        print("bad freq ranges:", self.BAD_FREQ_RANGES)
        print("n_particles:", self.N_PARTICLES)

def main(args=None):
    rclpy.init(args=args)
    action_client = WallDetection()
    try:
        rclpy.spin(action_client)
    except Exception as e:
        print(e)
        raise e
        action_client.set_buzzer(0)
        action_client.land()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
