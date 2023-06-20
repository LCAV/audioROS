#!/usr/bin/env python
# coding: utf-8
"""
algos_basics.py: basics for audio algorithms.
"""

import numpy as np

from .constants import SPEED_OF_SOUND
from scipy.spatial.transform import Rotation


def rotate_mics(mics, orientation_deg=0):
    """
    :param mics: mic positions (n_mics, 2)
    :return mics_rotated: (n_mics, 2)
    """
    rot = Rotation.from_euler("z", orientation_deg, degrees=True)
    R = rot.as_matrix()  # 3 x 3
    mics_aug = np.c_[mics, np.ones(mics.shape[0])].T  # 3 x 4
    mics_rotated = R.dot(mics_aug)[:2, :]  # 2 x 4
    return mics_rotated.T


def low_rank_inverse(low_rank_matrix, rank=1):
    u, s, vt = np.linalg.svd(low_rank_matrix, full_matrices=True)
    # np.testing.assert_allclose(u.dot(np.diag(s)).dot(vt), low_rank_matrix)
    s[rank:] = 0
    s[:rank] = 1 / s[:rank]
    s_mat = np.diag(s)
    inverse = vt.T @ s_mat @ u.T
    return inverse


def get_mic_delays(mic_positions, azimuth, elevation=None):
    r0 = mic_positions[0]
    return np.array(
        [
            get_mic_delta(r0, r1, azimuth, elevation) / SPEED_OF_SOUND
            for r1 in mic_positions
        ]
    )


def get_mic_delays_near(mic_positions, source):
    """
    :param mic_positions: mic coordinates of shape (n_mics, 2)
    :param source: source coordinates of shape (2,)
    """
    r0 = mic_positions[0]
    return np.array(
        [get_mic_delta_near(r0, r1, source) / SPEED_OF_SOUND for r1 in mic_positions]
    )


def get_mic_delta(r0, r1, azimuth, elevation=None):
    """
    Get difference in travel distance for r1 w.r.t. r0 for source coming from specified angle.
    """
    assert r0.shape == r1.shape

    # 3d source
    if len(r0) == 3:
        assert len(r0) == 3
        if elevation is None:
            raise ValueError("need to specify elevation for 3D doa.")
        s = np.array(
            (
                np.cos(azimuth) * np.cos(elevation),
                np.sin(azimuth) * np.cos(elevation),
                np.sin(elevation),
            )
        )
    # 2d source
    elif len(r0) == 2:
        assert len(r0) == 2
        s = np.array((np.cos(azimuth), np.sin(azimuth)))
    else:
        raise ValueError(f"r0 has to be of length 2 or 3, but is {len(r0)}")
    return np.inner(r0 - r1, s)


def get_mic_delta_near(r0, r1, source):
    """
    Get difference in travel distance for r1 w.r.t. r0 for source coming
    from near-field location `source`.
    """
    assert r0.shape == r1.shape
    assert source.shape == r0.shape

    return np.linalg.norm(r1 - source) - np.linalg.norm(r0 - source)


def get_autocorrelation(signals, frequency_bins=None):
    """Compute autocorrelations (in frequency domain) from time signals

    :param signals: num_mics x num_samples signal samples.

    :returns: num_frequencies x num_mics x num_mics autocorrelation matrix.
    """
    num_mics = signals.shape[0]
    if signals.shape[0] > signals.shape[1]:
        print(
            "Warning: probably shape mismatch for signals (expecting num_mics in frist dimension)",
            signals.shape,
        )

    signals_f = np.fft.rfft(
        signals, n=signals.shape[1], axis=1
    ).T  # num_samples x num_mics
    if frequency_bins is None:
        Rx = (
            1
            / num_mics
            * signals_f[:, :, np.newaxis]
            @ signals_f[:, np.newaxis, :].conj()
        )
    else:
        Rx = (
            1
            / num_mics
            * signals_f[frequency_bins, :, np.newaxis]
            @ signals_f[frequency_bins, np.newaxis, :].conj()
        )
    return Rx


def get_responses_DAS_old(Rx, mic_positions, omega=None, num_angles=1000):
    """Apply DAS algorithm."""

    dimension = mic_positions.shape[1]
    if dimension == 3 and num_angles > 100:
        print(f"Warning: number of angles {num_angles} quite high for 3 dimensions.")

    azimuth_array = np.linspace(0, 2 * np.pi, num_angles)
    if dimension == 3:
        # eps = 1e-3
        eps = 0
        # doing -1 here to not confuse dimensions.
        elevation_array = np.linspace(-np.pi + eps, np.pi - eps, num_angles - 1)
        # elevation_array = [0]
    else:
        elevation_array = np.array([0])

    # can use anything as reference
    # ref_mic = np.array([10, 10])
    ref_mic = mic_positions[0]

    responses = []
    for az in azimuth_array:
        for el in el_array:
            h = [
                np.exp(
                    -1j
                    * omega
                    * get_mic_delta(ref_mic, mic_pos, az, el)
                    / SPEED_OF_SOUND
                )
                for mic_pos in mic_positions
            ]

            h = np.array(h).reshape((-1, 1))
            power_theta = np.real(h.conjugate().T.dot(Rx).dot(h)).flatten()[0]
            responses.append(power_theta)

    if dimension == 3:
        responses = np.array(responses)
        responses = responses.reshape((len(azimuth_array), len(elevation_array)))
    return azimuth_array, elevation_array, responses


def get_responses_DAS_old(Rx, mic_positions, omega=None, num_angles=1000):
    """Apply DAS algorithm."""

    dimension = mic_positions.shape[1]
    if dimension == 3 and num_angles > 100:
        print(f"Warning: number of angles {num_angles} quite high for 3 dimensions.")

    azimuth_array = np.linspace(0, 2 * np.pi, num_angles)
    if dimension == 3:
        # doing -1 here to not confuse dimensions.
        elevation_array = np.linspace(-np.pi, np.pi, num_angles - 1)
    else:
        elevation_array = np.array([0])

    # can use anything as reference
    ref_mic = mic_positions[0]

    responses = []
    for az in azimuth_array:
        for elevation in elevation_array:
            h = [
                np.exp(
                    -1j
                    * omega
                    * get_mic_delta(ref_mic, mic_pos, az, elevation)
                    / SPEED_OF_SOUND
                )
                for mic_pos in mic_positions
            ]

            h = np.array(h).reshape((-1, 1))
            power_theta = np.real(h.conjugate().T.dot(Rx).dot(h)).flatten()[0]
            responses.append(power_theta)

    if dimension == 3:
        responses = np.array(responses)
        responses = responses.reshape((len(azimuth_array), len(elevation_array)))
    return azimuth_array, elevation_array, responses


def get_doa_DAS(azimuth_array, responses, elevation_array=np.array([0])):
    """Return array of pairs of angles of max response.

    :param azimuth_array: length N array of azimuths
    :param elevation_array: length M array of azimuths
    :param responses: matrix of responses (N x M)

    :return: list of maximum directions
        [(azimuth1, elevation1),
         (azimuth2, elevation2),
         ...]
    """
    if len(elevation_array) == 1:
        assert isinstance(responses, list) or responses.ndim == 1
        return azimuth_array[np.where(responses == max(responses))], elevation_array[0]
    indices = np.where(responses == np.max(responses))
    max_azimuths = azimuth_array[indices[0]]
    max_elevation = elevation_array[indices[1]]
    return max_azimuths, max_elevation


if __name__ == "__main__":
    from mic_array import get_uniform_array

    # uniform linear array
    baseline = 1  # in m
    mic_number = 5
    dimension = 2
    mic_positions_2d = get_uniform_array(
        mic_number=mic_number, baseline=baseline, dimension=dimension
    )

    # testing delays for pi_2 and 0
    azimuth_array = np.linspace(0, 2 * np.pi, 360)
    deltas = np.array(
        [
            get_mic_delta(mic_positions_2d[0], mic_positions_2d[1], az) / SPEED_OF_SOUND
            for az in azimuth_array
        ]
    )
    assert np.allclose(deltas[azimuth_array == np.pi / 2.0], 0), deltas[
        azimuth_array == np.pi / 2.0
    ]
    assert np.allclose(deltas[azimuth_array == -np.pi / 2.0], 0), deltas[
        azimuth_array == np.pi / 2.0
    ]
    assert np.allclose(deltas[azimuth_array == 0], np.max(deltas)), deltas[
        azimuth_array == 0
    ]
    assert np.allclose(deltas[azimuth_array == np.pi], np.min(deltas)), deltas[
        azimuth_array == np.pi
    ]

    # uniform linear array
    dimension = 3
    mic_positions_2d = get_uniform_array(
        mic_number=mic_number, baseline=baseline, dimension=dimension
    )

    # testing delays 0, -pi and pi
    az = 1
    elevation_array = np.linspace(-np.pi, np.pi, 360)
    deltas = np.array(
        [
            get_mic_delta(mic_positions_2d[0], mic_positions_2d[1], az, el)
            / SPEED_OF_SOUND
            for el in elevation_array
        ]
    )
    assert np.allclose(deltas[elevation_array == np.pi / 2.0], 0), deltas[
        elevation_array == np.pi / 2.0
    ]
    assert np.allclose(deltas[elevation_array == -np.pi / 2.0], 0), deltas[
        elevation_array == np.pi / 2.0
    ]
    assert np.allclose(deltas[elevation_array == 0], np.max(deltas)), deltas[
        elevation_array == 0
    ]
    assert np.allclose(deltas[elevation_array == np.pi], np.min(deltas)), deltas[
        elevation_array == np.pi
    ]

    # make sure far and near field converge to the same delays for very far source.
    baseline = 1
    distance = 1e10 * baseline
    theta = 0.3
    mic_positions_2d = get_uniform_array(mic_number=2, baseline=baseline, dimension=2)
    source = distance * np.array([np.cos(theta), np.sin(theta)])
    delta_far = get_mic_delta(mic_positions_2d[0], mic_positions_2d[1], theta)
    delta_near = get_mic_delta_near(mic_positions_2d[0], mic_positions_2d[1], source)
    assert abs(delta_far - delta_near) < 1e-5, (delta_far, delta_near)

    print("all tests ok.")
