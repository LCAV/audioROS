""" Various plotting utlities.

Copied from gtsam.utils and adapted so that it can be used without having to recompile gtsam each time. 
"""

from typing import Iterable, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import patches
from mpl_toolkits.mplot3d import Axes3D

import gtsam
from gtsam import Marginals, Point3, Pose2, Pose3, Values


def set_axes_equal(fignum: int) -> None:
    """
    Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Args:
      fignum: An integer representing the figure number for Matplotlib.
    """
    fig = plt.figure(fignum)
    if not fig.axes:
        ax = fig.add_subplot(projection="3d")
    else:
        ax = fig.axes[0]

    limits = np.array([ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d(),])

    origin = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))

    ax.set_xlim3d([origin[0] - radius, origin[0] + radius])
    ax.set_ylim3d([origin[1] - radius, origin[1] + radius])
    ax.set_zlim3d([origin[2] - radius, origin[2] + radius])


def ellipsoid(
    rx: float, ry: float, rz: float, n: int
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Numpy equivalent of Matlab's ellipsoid function.

    Args:
        rx: Radius of ellipsoid in X-axis.
        ry: Radius of ellipsoid in Y-axis.
        rz: Radius of ellipsoid in Z-axis.
        n: The granularity of the ellipsoid plotted.

    Returns:
        The points in the x, y and z axes to use for the surface plot.
    """
    u = np.linspace(0, 2 * np.pi, n + 1)
    v = np.linspace(0, np.pi, n + 1)
    x = -rx * np.outer(np.cos(u), np.sin(v)).T
    y = -ry * np.outer(np.sin(u), np.sin(v)).T
    z = -rz * np.outer(np.ones_like(u), np.cos(v)).T

    return x, y, z


def plot_covariance_ellipse_3d(
    axes,
    origin: Point3,
    P: np.ndarray,
    cov_scale: float = 0.1,
    n: int = 8,
    alpha: float = 0.5,
) -> None:
    """
    Plots a Gaussian as an uncertainty ellipse

    Based on Maybeck Vol 1, page 366
    k=2.296 corresponds to 1 std, 68.26% of all probability
    k=11.82 corresponds to 3 std, 99.74% of all probability

    Args:
        axes (matplotlib.axes.Axes): Matplotlib axes.
        origin: The origin in the world frame.
        P: The marginal covariance matrix of the 3D point
            which will be represented as an ellipse.
        cov_scale: Scaling factor of the radii of the covariance ellipse.
        n: Defines the granularity of the ellipse. Higher values indicate finer ellipses.
        alpha: Transparency value for the plotted surface in the range [0, 1].
    """
    k = 11.82
    U, S, _ = np.linalg.svd(P)

    radii = k * np.sqrt(S)
    radii = radii * cov_scale
    rx, ry, rz = radii

    # generate data for "unrotated" ellipsoid
    xc, yc, zc = ellipsoid(rx, ry, rz, n)

    # rotate data with orientation matrix U and center c
    data = np.kron(U[:, 0:1], xc) + np.kron(U[:, 1:2], yc) + np.kron(U[:, 2:3], zc)
    n = data.shape[1]
    x = data[0:n, :] + origin[0]
    y = data[n : 2 * n, :] + origin[1]
    z = data[2 * n :, :] + origin[2]

    axes.plot_surface(x, y, z, alpha=alpha, cmap="hot")


def plot_pose2_on_axes(
    axes,
    pose: Pose2,
    axis_length: float = 0.1,
    cov_scale: float = 0.1,
    covariance: np.ndarray = None,
    ls="-",
) -> None:
    """
    Plot a 2D pose on given axis `axes` with given `axis_length`.

    Args:
        axes (matplotlib.axes.Axes): Matplotlib axes.
        pose: The pose to be plotted.
        axis_length: The length of the camera axes.
        covariance (numpy.ndarray): Marginal covariance matrix to plot
            the uncertainty of the estimation.
    """
    # get rotation and translation (center)
    gRp = pose.rotation().matrix()  # rotation from pose to global
    t = pose.translation()
    origin = t

    # draw the camera axes
    x_axis = origin + gRp[:, 0] * axis_length
    line = np.append(origin[np.newaxis], x_axis[np.newaxis], axis=0)
    axes.plot(line[:, 0], line[:, 1], "r" + ls)

    y_axis = origin + gRp[:, 1] * axis_length
    line = np.append(origin[np.newaxis], y_axis[np.newaxis], axis=0)
    axes.plot(line[:, 0], line[:, 1], "g" + ls)

    if covariance is not None:
        pPp = covariance[0:2, 0:2]
        gPp = np.matmul(np.matmul(gRp, pPp), gRp.T)

        w, v = np.linalg.eig(gPp)

        # k = 2.296
        k = 5.0

        angle = np.arctan2(v[1, 0], v[0, 0])
        e1 = patches.Ellipse(
            origin, np.sqrt(w[0] * k), np.sqrt(w[1] * k), np.rad2deg(angle), fill=False
        )
        axes.add_patch(e1)


def plot_pose2(
    fignum: int,
    pose: Pose2,
    axis_length: float = 0.1,
    cov_scale: float = 0.1,
    covariance: np.ndarray = None,
    axis_labels=("X axis", "Y axis", "Z axis"),
    ls="-",
) -> plt.Figure:
    """
    Plot a 2D pose on given figure with given `axis_length`.

    Args:
        fignum: Integer representing the figure number to use for plotting.
        pose: The pose to be plotted.
        axis_length: The length of the camera axes.
        covariance: Marginal covariance matrix to plot
            the uncertainty of the estimation.
        axis_labels (iterable[string]): List of axis labels to set.
    """
    # get figure object
    fig = plt.figure(fignum)
    axes = fig.gca()
    plot_pose2_on_axes(
        axes,
        pose,
        axis_length=axis_length,
        cov_scale=cov_scale,
        covariance=covariance,
        ls=ls,
    )
    axes.set_xlabel(axis_labels[0])
    axes.set_ylabel(axis_labels[1])

    return fig


def plot_point3_on_axes(
    axes,
    point: Point3,
    linespec: str,
    P: Optional[np.ndarray] = None,
    cov_scale: float = 1.0,
) -> None:
    """
    Plot a 3D point on given axis `axes` with given `linespec`.

    Args:
        axes (matplotlib.axes.Axes): Matplotlib axes.
        point: The point to be plotted.
        linespec: String representing formatting options for Matplotlib.
        P: Marginal covariance matrix to plot the uncertainty of the estimation.
    """
    axes.plot([point[0]], [point[1]], [point[2]], linespec)
    if P is not None:
        plot_covariance_ellipse_3d(axes, point, P, cov_scale=cov_scale)


def plot_point3(
    fignum: int,
    point: Point3,
    linespec: str,
    P: np.ndarray = None,
    axis_labels: Iterable[str] = ("X axis", "Y axis", "Z axis"),
    cov_scale: float = 1.0,
) -> plt.Figure:
    """
    Plot a 3D point on given figure with given `linespec`.

    Args:
        fignum: Integer representing the figure number to use for plotting.
        point: The point to be plotted.
        linespec: String representing formatting options for Matplotlib.
        P: Marginal covariance matrix to plot the uncertainty of the estimation.
        axis_labels: List of axis labels to set.

    Returns:
        fig: The matplotlib figure.

    """
    fig = plt.figure(fignum)
    axes = fig.gca(projection="3d")
    plot_point3_on_axes(axes, point, linespec, P, cov_scale=cov_scale)

    axes.set_xlabel(axis_labels[0])
    axes.set_ylabel(axis_labels[1])
    axes.set_zlabel(axis_labels[2])

    return fig


def plot_3d_points(
    fignum,
    values,
    linespec="g*",
    marginals=None,
    title="3D Points",
    axis_labels=("X axis", "Y axis", "Z axis"),
    cov_scale=1.0,
):
    """
    Plots the Point3s in `values`, with optional covariances.
    Finds all the Point3 objects in the given Values object and plots them.
    If a Marginals object is given, this function will also plot marginal
    covariance ellipses for each point.

    Args:
        fignum (int): Integer representing the figure number to use for plotting.
        values (gtsam.Values): Values dictionary consisting of points to be plotted.
        linespec (string): String representing formatting options for Matplotlib.
        marginals (numpy.ndarray): Marginal covariance matrix to plot the
            uncertainty of the estimation.
        title (string): The title of the plot.
        axis_labels (iterable[string]): List of axis labels to set.
    """

    keys = values.keys()

    # Plot points and covariance matrices
    for key in keys:
        try:
            point = values.atPoint3(key)
            if marginals is not None:
                covariance = marginals.marginalCovariance(key)
            else:
                covariance = None

            fig = plot_point3(
                fignum,
                point,
                linespec,
                covariance,
                axis_labels=axis_labels,
                cov_scale=cov_scale,
            )

        except RuntimeError:
            continue

    fig = plt.figure(fignum)
    # fig.suptitle(title)
    fig.canvas.set_window_title(title.lower())


def plot_plane3_on_axes(axes, plane, ls="-", axis_length=0.2):
    wall_origin = -plane.normal().point3() * plane.distance()
    wall_end = wall_origin * (1 - axis_length)
    axes.plot(
        [wall_origin[0], wall_end[0]],
        [wall_origin[1], wall_end[1]],
        [wall_origin[2], wall_end[2]],
        color="k",
        ls=ls,
    )
    axes.scatter(*wall_origin, color="k", marker="o")
    axes.scatter(*wall_end, color="k", marker=">")


def plot_pose3_on_axes(axes, pose, axis_length=0.1, P=None, cov_scale=1, ls="-"):
    """
    Plot a 3D pose on given axis `axes` with given `axis_length`.

    Args:
        axes (matplotlib.axes.Axes): Matplotlib axes.
        point (gtsam.Point3): The point to be plotted.
        linespec (string): String representing formatting options for Matplotlib.
        P (numpy.ndarray): Marginal covariance matrix to plot the uncertainty of the estimation.
    """
    # get rotation and translation (center)
    gRp = pose.rotation().matrix()  # rotation from pose to global
    origin = pose.translation()

    # draw the camera axes
    x_axis = origin + gRp[:, 0] * axis_length
    line = np.append(origin[np.newaxis], x_axis[np.newaxis], axis=0)
    axes.plot(line[:, 0], line[:, 1], line[:, 2], "r" + ls)

    y_axis = origin + gRp[:, 1] * axis_length
    line = np.append(origin[np.newaxis], y_axis[np.newaxis], axis=0)
    axes.plot(line[:, 0], line[:, 1], line[:, 2], "g" + ls)

    z_axis = origin + gRp[:, 2] * axis_length
    line = np.append(origin[np.newaxis], z_axis[np.newaxis], axis=0)
    axes.plot(line[:, 0], line[:, 1], line[:, 2], "b" + ls)

    # plot the covariance
    if P is not None:
        # covariance matrix in pose coordinate frame
        pPp = P[3:6, 3:6]
        # convert the covariance matrix to global coordinate frame
        gPp = gRp @ pPp @ gRp.T
        plot_covariance_ellipse_3d(axes, origin, gPp, cov_scale=cov_scale)


def plot_pose3(
    fignum: int,
    pose: Pose3,
    axis_length: float = 0.1,
    P: np.ndarray = None,
    axis_labels: Iterable[str] = ("X axis", "Y axis", "Z axis"),
    cov_scale: float = 1.0,
) -> plt.Figure:
    """
    Plot a 3D pose on given figure with given `axis_length`.

    Args:
        fignum: Integer representing the figure number to use for plotting.
        pose (gtsam.Pose3): 3D pose to be plotted.
        axis_length: The length of the camera axes.
        P: Marginal covariance matrix to plot the uncertainty of the estimation.
        axis_labels: List of axis labels to set.

    Returns:
        fig: The matplotlib figure.
    """
    # get figure object
    fig = plt.figure(fignum)
    if not fig.axes:
        axes = fig.add_subplot(projection="3d")
    else:
        axes = fig.axes[0]

    plot_pose3_on_axes(axes, pose, P=P, axis_length=axis_length, cov_scale=cov_scale)

    axes.set_xlabel(axis_labels[0])
    axes.set_ylabel(axis_labels[1])
    axes.set_zlabel(axis_labels[2])

    return fig


def plot_trajectory(
    fignum: int,
    values: Values,
    cov_scale: float = 1,
    axis_length: float = 1,
    marginals: Marginals = None,
    title: str = "Plot Trajectory",
    axis_labels: Iterable[str] = ("X axis", "Y axis", "Z axis"),
    ls="-",
) -> None:
    """
    Plot a complete 2D/3D trajectory using poses in `values`.

    Args:
        fignum: Integer representing the figure number to use for plotting.
        values: Values containing some Pose2 and/or Pose3 values.
        cov_scale: Value to cov_scale the poses by.
        marginals: Marginalized probability values of the estimation.
            Used to plot uncertainty bounds.
        title: The title of the plot.
        axis_labels (iterable[string]): List of axis labels to set.
    """
    fig = plt.figure(fignum)
    axes = fig.gca(projection="3d")

    axes.set_xlabel(axis_labels[0])
    axes.set_ylabel(axis_labels[1])
    axes.set_zlabel(axis_labels[2])

    # Plot 2D poses, if any
    poses = gtsam.utilities.allPose2s(values)
    for key in poses.keys():
        pose = poses.atPose2(key)
        if marginals:
            covariance = marginals.marginalCovariance(key)
        else:
            covariance = None

        plot_pose2_on_axes(
            axes,
            pose,
            covariance=covariance,
            axis_length=axis_length,
            cov_scale=cov_scale,
            ls=ls,
        )

    # Then 3D poses, if any
    poses = gtsam.utilities.allPose3s(values)
    for key in poses.keys():
        pose = poses.atPose3(key)
        if marginals:
            covariance = marginals.marginalCovariance(key)
        else:
            covariance = None

        plot_pose3_on_axes(
            axes,
            pose,
            P=covariance,
            axis_length=axis_length,
            cov_scale=cov_scale,
            ls=ls,
        )

    # Then planes, if any
    planes = gtsam.utilities.allOrientedPlane3s(values)
    for key in planes.keys():
        plane = planes.atOrientedPlane3(key)
        plot_plane3_on_axes(axes, plane, ls=ls, axis_length=axis_length)

    # fig.suptitle(title)
    fig.canvas.set_window_title(title.lower())


def plot_incremental_trajectory(
    fignum: int,
    values: Values,
    start: int = 0,
    cov_scale: float = 1,
    marginals: Optional[Marginals] = None,
    time_interval: float = 0.0,
) -> None:
    """
    Incrementally plot a complete 3D trajectory using poses in `values`.

    Args:
        fignum: Integer representing the figure number to use for plotting.
        values: Values dict containing the poses.
        start: Starting index to start plotting from.
        cov_scale: Value to cov_scale the poses by.
        marginals: Marginalized probability values of the estimation.
            Used to plot uncertainty bounds.
        time_interval: Time in seconds to pause between each rendering.
            Used to create animation effect.
    """
    fig = plt.figure(fignum)
    axes = fig.gca(projection="3d")

    poses = gtsam.utilities.allPose3s(values)
    keys = gtsam.KeyVector(poses.keys())

    for key in keys[start:]:
        if values.exists(key):
            pose_i = values.atPose3(key)
            plot_pose3(fignum, pose_i, cov_scale)

    # Update the plot space to encompass all plotted points
    axes.auto_scale()

    # Set the 3 axes equal
    set_axes_equal(fignum)

    # Pause for a fixed amount of seconds
    plt.pause(time_interval)


def plot_projections(
    estimate, axis_length=0.2, perspective=True, top=True, side=True, ls="-"
):
    if perspective:
        fig = plt.figure(0)
        fig.set_size_inches(10, 10)
        plot_trajectory(0, estimate, axis_length=axis_length, ls=ls)
        set_axes_equal(0)

    if side:
        fig = plt.figure(1)
        fig.set_size_inches(10, 10)
        plot_trajectory(1, estimate, axis_length=axis_length, ls=ls)
        set_axes_equal(1)
        plt.gca().view_init(elev=0.0, azim=0)
        plt.title("side view", y=0.9)

    if top:
        fig = plt.figure(2)
        fig.set_size_inches(10, 10)
        plot_trajectory(2, estimate, axis_length=axis_length, ls=ls)
        set_axes_equal(2)
        plt.gca().view_init(elev=90.0, azim=0)  # x down, y to right, looking from top
        plt.title("top view", y=0.9)
