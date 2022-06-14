#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
geometry.py: Functions to convert between path differences and angles/distances for different mic-speaker geometries.
"""
import warnings

import numpy as np

from audio_stack.beam_former import rotate_mics

from .constants import PLATFORM

DIM = 2


# standalone functions
def get_deltas_from_global(azimuth_deg, distances_cm, mic_idx, ax=None):
    """ Return path differences in m """
    context = Context.get_platform_setup()
    delta = (
        context.get_delta(
            azimuth_deg=azimuth_deg, distances_cm=distances_cm, mic_idx=mic_idx
        )
        * 1e-2
    )
    d0 = context.get_direct_path(mic_idx)
    return delta, d0


def get_orthogonal_distance_from_global(azimuth_deg, deltas_cm, mic_idx, ax=None):
    """ Return distances in cm """
    context = Context.get_platform_setup()
    distances_cm = context.get_distance(deltas_cm, azimuth_deg, mic_idx)
    return distances_cm


# any dimension
def convert_angle(theta):
    theta = np.round(theta * 180 / np.pi, 2)
    if theta >= 360:
        theta -= 360
    if theta < 0:
        theta += 360
    return theta


def get_normal(distance, azimuth_deg, elevation_deg=0):
    """
    both distance and azimuth_deg can be vectors, in which case they have to be of equal length.
    """
    assert np.ndim(elevation_deg) == 0
    azimuth = azimuth_deg / 180 * np.pi
    elevation = elevation_deg / 180 * np.pi

    if (np.ndim(azimuth) > 0) and (np.ndim(distance) > 0):
        assert len(azimuth) == len(distance), (azimuth, distance)

    if np.ndim(azimuth) > 0:
        dir_vector = np.c_[
            np.cos(azimuth) * np.cos(elevation),
            np.sin(azimuth) * np.cos(elevation),
            np.full(azimuth.shape, np.sin(elevation)),
        ]
    else:
        dir_vector = np.array(
            [
                np.cos(azimuth) * np.cos(elevation),
                np.sin(azimuth) * np.cos(elevation),
                np.sin(elevation),
            ]
        )

    if np.ndim(distance) > 0 and np.ndim(azimuth) == 0:
        n = distance[:, None] * dir_vector[None, :]
    elif np.ndim(distance) > 0 and np.ndim(azimuth) > 0:
        n = distance[:, None] * dir_vector
    else:
        n = distance * dir_vector
    return n


def get_source_image(normal, source):
    d = np.linalg.norm(normal)
    l = 2 * (1 - source.dot(normal) / (d ** 2))
    return source + l * normal


def get_delta_from_normal(mic, source, normal):
    """ 
    :param normal: can be of shape (n_distances x dim)
    """
    vec = source - mic
    r0 = np.linalg.norm(vec)

    if normal.ndim > 1:
        d = np.linalg.norm(normal, axis=1)
    else:
        d = np.linalg.norm(normal)

    l = 2 * (1 - np.inner(source, normal) / (d ** 2))
    r1_sq = r0 ** 2 + 2 * l * np.inner(vec, normal) + (l * d) ** 2
    delta = np.sqrt(r1_sq) - r0
    return delta


def get_angles(mic, source, delta_m, distance_m):
    vec = source - mic
    theta0 = np.arctan2(vec[1], vec[0])
    r0 = np.linalg.norm(vec)
    r1 = delta_m + r0
    # import pudb; pudb.set_trace()

    cos = (r1 ** 2 - r0 ** 2 - 4 * distance_m ** 2) / (4 * distance_m * r0)

    EPS = 1e-10
    if np.abs(cos) > 1 + EPS:
        # warnings.warn(f"cos too big ({cos}) for delta, distance: {delta_m, distance_m}")
        return None
    elif np.abs(cos) > 1:  # round cos to 1 or -1.
        cos = np.sign(cos)

    beta = np.arccos(cos)  # between 0 and pi
    theta1 = theta0 - beta
    theta2 = theta0 + beta
    thetas_deg = [convert_angle(theta) for theta in [theta1, theta2]]
    return thetas_deg


class Context(object):
    def __init__(self, dim=DIM, mics=None, source=None):
        assert dim in (2, 3), dim
        self.dim = dim
        if mics is not None:
            assert mics.shape[1] == dim
        if source is not None:
            assert source.shape[0] == dim
        self.mics = mics
        self.source = source

    @staticmethod
    def get_platform_setup(platform=PLATFORM):
        if platform == "crazyflie":
            #print("using crazyflie in get_platform_setup")
            return Context.get_crazyflie_setup()
        elif platform == "epuck":
            #print("using epuck in get_platform_setup")
            return Context.get_epuck_setup()
        else:
            raise ValueError(platform)

    @staticmethod
    def get_crazyflie_setup(dim=DIM):
        from crazyflie_description_py.parameters import MIC_POSITIONS, BUZZER_POSITION

        mics = np.array(MIC_POSITIONS)[:, :dim]
        source = np.array(BUZZER_POSITION)[0, :dim]
        return Context(dim, mics, source)

    @staticmethod
    def get_epuck_setup(dim=DIM):
        from epuck_description_py.parameters import MIC_POSITIONS, BUZZER_POSITION

        mics = np.array(MIC_POSITIONS)[:, :dim]
        source = np.array(BUZZER_POSITION)[0, :dim]
        return Context(dim, mics, source)

    @staticmethod
    def get_random_setup(n_mics=4, dim=DIM):
        mics = np.random.uniform(low=-1.0, high=1.0, size=(n_mics, dim))
        source = np.random.uniform(low=-1.0, high=1.0, size=(dim,))
        return Context(dim, mics, source)

    @staticmethod
    def get_standard_setup(dim=DIM):
        Z = 1.0
        X = 1 / np.sqrt(2)
        if dim == 2:
            mics = np.array([[X, X], [-X, X], [-X, -X], [X, -X]])
            source = np.array([2 * X, 2 * X])
        elif dim == 3:
            mics = np.array([[X, X, Z], [-X, X, Z], [-X, -X, Z], [X, -X, Z]])
            source = np.array([2 * X, 2 * X, Z])
        return Context(dim, mics, source)

    def get_source_image(self, normal):
        d = np.linalg.norm(normal)
        l = 2 * (1 - self.source.dot(normal) / (d ** 2))
        return self.source + l * normal

    def get_normal(self, distances_cm, azimuth_deg):
        distances_m = np.array(distances_cm) * 1e-2
        normal = get_normal(distances_m, azimuth_deg)
        normal = normal[..., : self.dim]
        return normal

    def get_delta_from_normal(self, azimuth_deg, distances_cm, mic_idx=0):
        normal = self.get_normal(distances_cm, azimuth_deg)
        return get_delta_from_normal(self.mics[mic_idx], self.source, normal)

    def get_delta(self, azimuth_deg, distances_cm, mic_idx=0):
        """ 
        :return: path difference in cm.
        """
        r0_cm = self.get_direct_path(mic_idx) * 1e2
        theta0 = self.get_theta0(mic_idx)  # in rad
        theta = azimuth_deg / 180 * np.pi
        cos = np.cos(theta - theta0)
        return (
            np.sqrt(r0_cm ** 2 + 4 * distances_cm ** 2 - 4 * distances_cm * r0_cm * cos)
            - r0_cm
        )

    def get_distance(self, delta_cm, azimuth_deg, mic_idx=0):
        """ 
        :return: distance to wall in cm.
        """
        r0_cm = self.get_direct_path(mic_idx) * 1e2
        theta0 = self.get_theta0(mic_idx)  # in rad
        theta = azimuth_deg / 180 * np.pi
        cos = np.cos(theta - theta0)
        return 0.5 * (
            r0_cm * cos
            + np.sqrt(delta_cm * (delta_cm + 2 * r0_cm) + r0_cm ** 2 * cos ** 2)
        )

    def get_delta_gradient(self, azimuth_deg, distances_cm, mic_idx):
        """ gradient of delta differentiated by angle """
        r0_cm = self.get_direct_path(mic_idx) * 1e2
        theta0 = self.get_theta0(mic_idx)  # in rad
        theta = azimuth_deg / 180 * np.pi
        return np.abs(
            (4 * distances_cm - 2 * r0_cm * np.cos(theta - theta0))
            / np.sqrt(
                r0_cm ** 2
                + 4 * distances_cm ** 2
                - 4 * distances_cm * r0_cm * np.cos(theta - theta0)
            )
        )

    # TODO(FD) test below function
    def get_delta_gradient_angle(self, distance_m, azimuths_deg, mic_idx):
        """ gradient of delta differentiated by angle """
        r0_m = self.get_direct_path(mic_idx)
        theta0 = self.get_theta0(mic_idx)  # in rad
        azimuths = azimuths_deg / 180 * np.pi
        return np.abs(
            2
            * distance_m
            * r0_m
            * np.sin(azimuths - theta0)
            / np.sqrt(
                r0_m ** 2
                + 4 * distance_m ** 2
                - 4 * distance_m * r0_m * np.cos(azimuths - theta0)
            )
        )

    def get_total_distance(self, delta_m, azimuth_deg, mic_idx):
        """ 
        :return: distance of centre to wall in m.
        """
        warnings.warn(
            "Do not use this function anymore, replace by get_distance (delta_cm input!)"
        )
        if self.dim == 3:
            raise NotImplementedError(
                "distance retrieval only implemented for 2D setups"
            )
        return self.get_distance(delta_m * 1e2, azimuth_deg, mic_idx) * 1e-2

    def get_angles(self, delta_m, distance_m, mic_idx):
        if self.dim == 3:
            raise NotImplementedError("angle retrieval only implemented for 2D setups")
        return get_angles(self.mics[mic_idx], self.source, delta_m, distance_m)

    def get_direct_path(self, mic_idx):
        """
        :return: direct path in m
        """
        return np.linalg.norm(self.source - self.mics[mic_idx])

    def get_theta0(self, mic_idx):
        """
        :return: direct angle in rad
        """
        vec = self.mics[mic_idx, :] - self.source
        return np.arctan2(vec[1], vec[0])

    def plot(self, normal=None, azimuth_deg=None, distance=None, ax=None):
        import matplotlib.pylab as plt

        if ax is None:
            fig, ax = plt.subplots()

        ax.scatter(0, 0, color="black", marker="x")
        [ax.scatter(*mic[:2], label=f"mic{i}") for i, mic in enumerate(self.mics)]
        ax.scatter(*self.source[:2], label="source")

        if azimuth_deg is not None and distance is not None:
            if normal is not None:
                warnings.warn("overwriting normal!")
            normal = get_normal(distance, azimuth_deg)[:2]

        if normal is not None:
            s = self.get_source_image(normal)
            ax.plot([0, normal[0]], [0, normal[1]], label="wall normal")
            ax.scatter(*s[:2], label="image source")

        ax.axis("equal")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.legend()
        return ax

    def get_possible_distances(self, range_m=0.5, step_m=0.01):
        # minimum distance for the wall not touching the mics or source:
        min_distance = max(
            [*np.linalg.norm(self.mics, axis=1), np.linalg.norm(self.source)]
        )
        return np.arange(min_distance, min_distance + range_m, step=step_m)
