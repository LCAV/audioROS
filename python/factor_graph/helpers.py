
import numpy as np
import matplotlib.pylab as plt

import gtsam


def rad(deg):
    return deg / 180 * np.pi

def deg(rad):
    return rad * 180 / np.pi

def get_angles(unit_vector, verbose=False):
    azimuth = np.arctan2(unit_vector[1], unit_vector[0]) * 180 / np.pi
    elevation = np.arcsin(unit_vector[2]) * 180 / np.pi
    if verbose:
        print(f"angles: azimuth={azimuth:.0f}deg, elevation={elevation:.0f}deg")
    return azimuth, elevation

def get_vector(azimuth, elevation):
    return np.r_[
        np.cos(azimuth) * np.cos(elevation), 
        np.sin(azimuth) * np.cos(elevation),
        np.sin(elevation)
    ]

def print_plane(label, plane):
    print(f"{label} distance: {plane.distance():.2f}", end="\t")
    normal = plane.normal().point3()
    #print(f"{label} normal:", normal)
    get_angles(normal, verbose=True)

def plot_all(estimate, axis_length=0.2, top=True, side=True):
    import plot
    fig = plt.figure(0)
    fig.set_size_inches(10, 10)
    plot.plot_trajectory(0, estimate, axis_length=axis_length)
    plot.set_axes_equal(0)

    if side:
        fig = plt.figure(1)
        fig.set_size_inches(10, 10)
        plot.plot_trajectory(1, estimate, axis_length=axis_length)
        plot.set_axes_equal(1)
        plt.gca().view_init(elev=0., azim=0)
        plt.title("side view")

    if top:
        fig = plt.figure(2)
        fig.set_size_inches(10, 10)
        plot.plot_trajectory(2, estimate, axis_length=axis_length)
        plot.set_axes_equal(2)
        plt.gca().view_init(elev=90., azim=0)
        plt.title("top view")

# TODO: need functions to
# - calculate bearing to a plane
# - calculate range to a plane
# just like Pose3.bearing(Pose3) or Pose3.range(pose3)

class WallSimulation(object):
    """
    Wall simulation: 
    - we have a fixed velocity (linear and angular)
    - we start from a given pose and plane
    - 
    """
    def __init__(self):
        self.v = np.array([0, 0.1, 0]); 
        self.noise_v = gtsam.noiseModel.Isotropic.Sigma(3, 0.01) # m/s, linear velocity
        self.w = np.array([0.1, 0, 0]); 
        self.noise_w = gtsam.noiseModel.Isotropic.Sigma(3, rad(1)) # rad/s (yaw, pitch, roll)
        self.time = 0
        
        # xyz
        self.noise_p = gtsam.noiseModel.Diagonal.Sigmas([0.02, 0.02, 0.001])
        # ypr
        self.noise_r = gtsam.noiseModel.Diagonal.Sigmas([rad(10), rad(1), rad(1)]) 
        # distance, azimuth, elevation
        self.noise_plane = gtsam.noiseModel.Diagonal.Sigmas([0.1, rad(10), rad(1)])
        self.seed = 0
        
    def initialize(self, plane, pose_0):
        self.plane = plane
        self.pose_0 = pose_0
        self.pose_t = self.pose_0
        
    def move_until_time(self, time=0):
        """ move with noisy velocities until given time 
        """
        delta_t = time - self.time
        v_noisy = delta_t * (self.v + gtsam.Sampler(self.noise_v, self.seed).sample())
        w_noisy = delta_t * (self.w + gtsam.Sampler(self.noise_w, self.seed).sample())
        pose_delta = gtsam.Pose3(r=gtsam.Rot3.Ypr(*w_noisy), t=gtsam.Point3(*v_noisy))
        self.seed += 1
        self.pose_t = pose_delta * self.pose_t
        self.time = time
        
    def real_pose(self):
        return self.pose_t
    
    def expected_pose(self, time=None):
        """ currently expected pose, given the velocities """
        if time is None:
            time = self.time
        pose_delta = gtsam.Pose3(r=gtsam.Rot3.Ypr(*(self.w * time)), t=gtsam.Point3(*(self.v * time)))
        return pose_delta * self.pose_0
    
    def measure_pose(self):
        pose_delta = gtsam.Pose3(t=gtsam.Sampler(self.noise_p, self.seed).sample(),
                                 r=gtsam.Rot3.Ypr(*gtsam.Sampler(self.noise_r, self.seed).sample()))
        return pose_delta * self.pose_t
    
    def measure_plane(self):
        plane_gt = self.plane.transform(self.pose_t)
        azimuth_deg, elevation_deg = get_angles(plane_gt.normal().point3())
        distance = plane_gt.distance()
        azimuth = rad(azimuth_deg)
        elevation = rad(elevation_deg)
        
        noise = gtsam.Sampler(self.noise_plane, self.seed).sample()
        distance += noise[0]
        azimuth += noise[1]
        elevation += noise[2]
        
        plane_measured = gtsam.OrientedPlane3() 
        normal_vec = get_vector(azimuth, elevation) 
        plane_meas = gtsam.OrientedPlane3(n=gtsam.Unit3(-normal_vec), d=distance)
        return plane_meas, azimuth, elevation, distance
