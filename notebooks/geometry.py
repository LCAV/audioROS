#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
geometry.py: 
"""
import numpy as np

from audio_stack.beam_former import rotate_mics

DIM = 2

# this corresponds to the setup in BC325 with stepper motor:
D_OFFSET = 0.08  # actual distance at zero-distance, in meters
YAW_OFFSET = -42  # in degrees

# old functions, kept for compatibility.
def get_deltas_from_global(yaw_deg, distances_cm, mic_idx, ax=None):
    context = Context.get_crazyflie_setup()
    delta = context.get_delta(yaw_deg=yaw_deg, distances_cm=distances_cm, mic_idx=mic_idx)
    d0 = context.get_direct_path(mic_idx)
    return delta, d0


def get_orthogonal_distance_from_global(yaw_deg, deltas_cm, mic_idx, ax=None):
    context = Context.get_crazyflie_setup()
    distances_m = context.get_total_distance(deltas_cm * 1e-2, yaw_deg, mic_idx)
    distances_cm = distances_m * 1e2
    return distances_cm


# any dimension
def convert_angle(theta):
    theta = np.round(theta * 180 / np.pi, 2)
    if theta >= 360:
        theta -= 360
    if theta < 0:
        theta += 360 
    return theta


def get_normal(distance, yaw_deg, elevation_deg=0): 
    """
    both distance and yaw_deg can be vectors, in which case they hav eto be of equal length.
    """
    assert np.ndim(elevation_deg) == 0
    yaw = yaw_deg / 180 * np.pi
    elevation = elevation_deg / 180 * np.pi

    if (np.ndim(yaw) > 0) and (np.ndim(distance) > 0):
        assert len(yaw) == len(distance), (yaw, distance)

    if np.ndim(yaw) > 0:
        dir_vector = np.c_[
                np.cos(yaw)*np.cos(elevation),
                np.sin(yaw) * np.cos(elevation),
                np.full(yaw.shape, np.sin(elevation))
        ]
    else:
        dir_vector = np.array([
                np.cos(yaw) * np.cos(elevation),
                np.sin(yaw) * np.cos(elevation),
                np.sin(elevation)
        ])

    if np.ndim(distance) > 0 and np.ndim(yaw) == 0:
        n = distance[:, None] * dir_vector[None, :]
    elif np.ndim(distance) > 0 and np.ndim(yaw) > 0:
        n = distance[:, None] * dir_vector
    else:
        n = distance * dir_vector
    return n


def get_source_image(normal, source):
    d = np.linalg.norm(normal)
    l = 2 * (1 - source.dot(normal) / (d**2))
    return source + l*normal


def get_delta_from_normal(mic, source, normal): 
    """ 
    :param normal: kcan be of shape (n_distances x dim)
    """
    vec = source - mic
    r0 = np.linalg.norm(vec)

    if normal.ndim > 1:
        d = np.linalg.norm(normal, axis=1)
    else:
        d = np.linalg.norm(normal)
    
    l = 2 * (1 - np.inner(source, normal) / (d**2))
    r1_sq = r0**2 + 2*l*np.inner(vec, normal) + (l*d)**2
    delta = np.sqrt(r1_sq) - r0
    return delta


# only two dimensions
def get_source_distance(mic, source, delta, yaw_deg): 
    vec = source - mic
    r0 = np.linalg.norm(vec)
    theta0 = np.arctan2(vec[1], vec[0])
    r1 = delta + r0

    theta = yaw_deg / 180 * np.pi

    cos = np.cos(theta - theta0)
    distance = 0.5 * (-r0*cos + np.sqrt(
            r1**2 - r0**2 * (1 - cos**2)
    ))
    return distance


def get_total_distance(source, source_distance, yaw_deg): 
    """ 
    source_distance is the distance measured from the source to the wall.
    To get the full distance we need to add the projection
    of the source to the wall normal. 
    """
    normal = get_normal(source_distance, yaw_deg)
    normal = normal[..., :2]
    if np.ndim(source_distance) > 0:
        total_distance = source_distance
        valid = source_distance > 0
        total_distance[valid] += np.inner(source, normal[valid]) / source_distance[valid]
    else:
        total_distance = source_distance + np.inner(source, normal) / source_distance
    return total_distance
    

def get_angles(mic, source, delta, source_distance): 
    vec = source - mic
    theta0 = np.arctan2(vec[1], vec[0])
    r0 = np.linalg.norm(vec)
    r1 = delta + r0

    cos = (r1**2 - r0**2 - 4*source_distance**2) / (4 * source_distance * r0)

    EPS = 1e-10
    if np.abs(cos) > 1 + EPS: 
        #print(f"Warning: cos way to big ({cos}) for distance {source_distance}")
        #print(r0, r1, source_distance)
        return None
    elif np.abs(cos) > 1: # round cos to 1 or -1.
        cos = np.sign(cos)

    beta = np.arccos(cos) # between 0 and pi
    theta1 = theta0 - beta
    theta2 = theta0 + beta
    thetas_deg = [convert_angle(theta) for theta in [theta1, theta2]]
    return thetas_deg


class Context(object):
    def __init__(self, dim=DIM, mics=None, source=None, d_offset=0):
        assert dim in (2, 3), dim
        self.dim = dim
        if mics is not None:
            assert mics.shape[1] == dim
        if source is not None:
            assert source.shape[0] == dim
        self.mics = mics
        self.source = source
        self.d_offset = d_offset

    @staticmethod
    def get_crazyflie_setup(dim=DIM, yaw_offset=None):
        from crazyflie_description_py.parameters import MIC_POSITIONS, BUZZER_POSITION
        mics = np.array(MIC_POSITIONS)[:, :dim]
        if yaw_offset is None:
            yaw_offset = YAW_OFFSET
        mics = rotate_mics(mics, yaw_offset)
        source = np.array(BUZZER_POSITION)[0, :dim]
        return Context(dim, mics, source, d_offset=D_OFFSET)

    @staticmethod
    def get_random_setup(n_mics=4, dim=DIM):
        mics = np.random.uniform(low=-1.0, high=1.0, size=(n_mics, dim))
        source = np.random.uniform(low=-1.0, high=1.0, size=(dim,))
        return Context(dim, mics, source)

    @staticmethod
    def get_standard_setup(dim=DIM):
        Z = 1.0
        X = 1/np.sqrt(2)
        if dim == 2:
            mics = np.array([[X, X], [-X, X], [-X, -X], [X, -X]])
            source = np.array([2*X, 2*X])
        elif dim == 3:
            mics = np.array([[X, X, Z], [-X, X, Z], [-X, -X, Z], [X, -X, Z]])
            source = np.array([2*X, 2*X, Z])
        return Context(dim, mics, source)

    def get_source_image(self, normal):
        d = np.linalg.norm(normal)
        l = 2 * (1 - self.source.dot(normal) / (d**2))
        return self.source + l*normal

    def get_delta_from_normal(self, normal, mic_idx):
        mic = self.mics[mic_idx, :]
        return get_delta_from_normal(mic, self.source, normal)

    def get_delta(self, yaw_deg, distances_cm, mic_idx=0):
        distances_here = np.array(distances_cm)*1e-2 + self.d_offset
        mic = self.mics[mic_idx]
        normal = get_normal(distances_here, yaw_deg)
        if np.ndim(normal) > 1:
            normal = normal[:, :2]
        else:
            normal = normal[:2]
        return get_delta_from_normal(mic, self.source, normal)

    def get_source_distance(self, delta, yaw_deg, mic_idx):
        if self.dim == 3:
            raise NotImplementedError("distance retrieval only implemented for 2D setups")
        mic = self.mics[mic_idx, :]
        return get_source_distance(mic, self.source, delta, yaw_deg)

    def get_total_distance(self, delta, yaw_deg, mic_idx):
        if self.dim == 3:
            raise NotImplementedError("distance retrieval only implemented for 2D setups")
        source_distance = self.get_source_distance(delta, yaw_deg, mic_idx)
        total_distance = get_total_distance(self.source, source_distance, yaw_deg)
        return total_distance - self.d_offset

    def get_angles(self, delta, source_distance, mic_idx): 
        if self.dim == 3:
            raise NotImplementedError("angle retrieval only implemented for 2D setups")
        mic = self.mics[mic_idx, :]
        return get_angles(mic, self.source, delta, source_distance)

    def get_direct_path(self, mic_idx):
        return np.linalg.norm(self.source - self.mics[mic_idx])

    def get_theta0(self, mic_idx):
        mic = self.mics[mic_idx, :]
        vec = self.source - mic
        return np.arctan2(vec[1], vec[0])

    def plot(self, normal=None, yaw_deg=None, distance=None, ax=None):
        import matplotlib.pylab as plt
        if ax is None:
            fig, ax = plt.subplots()

        ax.scatter(0, 0, color='black', marker='x')
        [ax.scatter(*mic[:2], label=f'mic{i}') for i, mic in enumerate(self.mics)]
        ax.scatter(*self.source[:2], label='source')

        if normal is not None:
            s = self.get_source_image(normal)
            ax.plot([0, normal[0]], [0, normal[1]])
            ax.scatter(*s[:2], label='image source')

        if yaw_deg is not None and distance is not None:
            distance += self.d_offset
            normal = get_normal(distance, yaw_deg)[:2]
            s = self.get_source_image(normal)
            ax.plot([0, normal[0]], [0, normal[1]])
            ax.scatter(*s[:2], label='image source')

        ax.axis('equal')
        ax.legend()
        return ax
        
