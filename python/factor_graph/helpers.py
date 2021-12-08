
import numpy as np
import matplotlib.pylab as plt

import gtsam


def rad(deg):
    return deg / 180 * np.pi

def deg(rad):
    return rad * 180 / np.pi

def get_distribution(estimate, range_=[0, 1], n_outliers=1, std=0.01):
    import scipy.stats
    # add noise to distance
    estimate = np.random.normal(estimate, scale=std)
    values = np.arange(*range_, step=0.01)
    prob = np.zeros(len(values))
    prob += scipy.stats.norm(estimate, std).pdf(values)
    for _ in range(n_outliers):
        outlier = np.random.uniform(*range_)
        prob += scipy.stats.norm(outlier, 3*std).pdf(values)
    prob /= np.sum(prob)
    return values, prob

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
        #self.noise_v = gtsam.noiseModel.Isotropic.Sigma(3, 0.01) # m/s, linear velocity
        self.noise_v = gtsam.noiseModel.Isotropic.Sigma(3, 1e-10) # m/s, linear velocity
        self.w = np.array([0.1, 0, 0]); 
        #self.noise_w = gtsam.noiseModel.Isotropic.Sigma(3, rad(1)) # rad/s (yaw, pitch, roll)
        self.noise_w = gtsam.noiseModel.Isotropic.Sigma(3, 1e-10) # rad/s (yaw, pitch, roll)
        self.time = 0
        
        self.pose_noise = gtsam.noiseModel.Diagonal.Sigmas([
            rad(1), rad(1), rad(10), # roll, pitch, yaw
            0.02, 0.02, 0.001, # x, y, z
        ])

        # distance, azimuth, elevation
        self.plane_noise = gtsam.noiseModel.Diagonal.Sigmas([1e-2, 1e-10, 1e-10])
        self.seed = 0


    def set_velocities(self, linear_m_s=0.1, yaw_deg_s=0, noise_linear=1e-3, noise_yaw=1):
        self.v = np.array([0, linear_m_s, 0])
        self.w = np.array([rad(yaw_deg_s), 0, 0])
        self.noise_v = gtsam.noiseModel.Isotropic.Sigma(3, noise_linear) 
        self.noise_w = gtsam.noiseModel.Isotropic.Sigma(3, rad(noise_yaw)) 

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

        # concatenate poses in increments of 1e-3. 
        pose_t = self.pose_0
        delta_s = 0.1  # time steps

        # like mod, but mod is unstable
        assert (int(time / delta_s) - time/delta_s) < 1e-10, time / delta_s
        for t in np.arange(0, time, step=delta_s):
            pose_delta = gtsam.Pose3(r=gtsam.Rot3.Ypr(*(self.w * delta_s)), t=gtsam.Point3(*(self.v * delta_s)))
            pose_t = pose_delta * pose_t 
        return pose_t
    
    def measure_pose(self):
        # TODO(FD) this needs to be done differently
        noise = gtsam.Sampler(self.pose_noise, self.seed).sample()
        rpy = noise[:3]
        ypr = rpy[::-1]
        pose_delta = gtsam.Pose3(t=noise[3:],
                                 r=gtsam.Rot3.Ypr(*ypr))
        return pose_delta * self.pose_t
    
    def measure_plane(self):
        plane_gt = self.plane.transform(self.pose_t)
        azimuth_deg, elevation_deg = get_angles(plane_gt.normal().point3())
        distance = plane_gt.distance()
        azimuth = rad(azimuth_deg)
        elevation = rad(elevation_deg)
        
        noise = gtsam.Sampler(self.plane_noise, self.seed).sample()
        distance += noise[0]
        azimuth += noise[1]
        elevation += noise[2]
        
        plane_measured = gtsam.OrientedPlane3() 
        normal_vec = get_vector(azimuth, elevation) 
        plane_meas = gtsam.OrientedPlane3(n=gtsam.Unit3(-normal_vec), d=distance)
        return plane_meas, azimuth, elevation, distance
