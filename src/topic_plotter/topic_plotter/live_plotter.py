#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
live_plotter.py: live data plotting using matplotlib.
"""

import math

import matplotlib
import matplotlib.pylab as plt
import numpy as np

matplotlib.use("TkAgg")
matplotlib.interactive(False)

MAX_YLIM = math.inf  # set to inf for no effect.
MIN_YLIM = -math.inf  # set to -inf for no effect.

MAX_XLIM = math.inf  # set to inf for no effect.
MIN_XLIM = -math.inf  # set to -inf for no effect.


class LivePlotter(object):
    def __init__(
        self,
        max_ylim=MAX_YLIM,
        min_ylim=MIN_YLIM,
        log=True,
        label="",
        max_xlim=MAX_XLIM,
        min_xlim=MIN_XLIM,
        ax=None,
        fig=None,
    ):
        self.max_ylim = max_ylim
        self.min_ylim = min_ylim
        self.max_xlim = max_xlim
        self.min_xlim = min_xlim
        self.log = log

        if (fig is None) and (ax is None):
            self.fig, self.ax = plt.subplots()
        else:
            self.fig = fig
            self.ax = ax

        self.fig.canvas.set_window_title(label)

        # containers for continuously updated data
        self.lines = {}
        self.axvlines = {}
        self.arrows = {}
        self.scatter = {}

        self.fig.canvas.mpl_connect("close_event", self.handle_close)
        self.fig.canvas.mpl_connect("resize_event", self.handle_resize)

        # Need the block argument to make sure the script continues after
        # plotting the figure.
        plt.show(block=False)

    def handle_close(self, evt):
        plt.close("all")
        print("closed all figures")

    def handle_resize(self, evt):
        # TODO(FD) this is not called when we resize
        # the figure, need to figure out why.
        self.fig.canvas.draw()

    def clear(self):
        for i in range(len(self.ax.lines)):
            self.ax.lines.pop()
        self.lines = {}
        self.axvlines = {}

    def update_arrow(self, origin, angle_deg, label="orientation"):
        """ Update arrow coordinates. 
        """
        xmin, xmax = self.ax.get_xlim()
        arrow_length = (xmax - xmin) / 5

        origin_x, origin_y = origin
        dx = arrow_length * math.cos(angle_deg * math.pi / 180)
        dy = arrow_length * math.sin(angle_deg * math.pi / 180)

        if label in self.arrows.keys():  #
            self.arrows[label]["pointer"].set_data(origin_x + dx, origin_y + dy)
            self.arrows[label]["line"].set_data(
                [origin_x, origin_x + dx], [origin_y, origin_y + dy]
            )
        else:
            self.arrows[label] = {}
            (line,) = self.ax.plot(
                origin_x + dx, origin_y + dy, marker=">", color=f"C{len(self.arrows)}"
            )
            self.arrows[label]["pointer"] = line
            (line,) = self.ax.plot(
                [origin_x, origin_x + dx],
                [origin_y, origin_y + dy],
                label=label,
                color=f"C{len(self.arrows)}",
            )
            self.arrows[label]["line"] = line

    def update_lines(self, data_matrix, x_data=None, labels=None, **kwargs):
        """ Plot each row of data_matrix as one line.
        """
        for i in range(data_matrix.shape[0]):
            if i in self.lines.keys():
                if x_data is not None:
                    self.lines[i].set_data(x_data, data_matrix[i, :])
                    self.ax.set_xlim(min(x_data), max(x_data))
                else:
                    self.lines[i].set_ydata(data_matrix[i, :])

                if labels is not None:
                    self.lines[i].set_label(labels[i])
            else:
                x_data = range(data_matrix.shape[1]) if x_data is None else x_data
                label = labels[i] if labels is not None else None
                if self.log:
                    (line,) = self.ax.semilogy(
                        x_data,
                        data_matrix[i, :],
                        color=f"C{i % 10}",
                        label=label,
                        **kwargs,
                    )
                else:
                    (line,) = self.ax.plot(
                        x_data,
                        data_matrix[i, :],
                        color=f"C{i % 10}",
                        label=label,
                        **kwargs,
                    )
                self.lines[i] = line

        self.reset_xlim()
        self.reset_ylim()

    def update_mesh(
        self, data_matrix, y_labels=None, x_labels=None, log=False, n_labels=5
    ):
        """ Plot each row of data_matrix in an image. """
        mesh = self.ax.pcolorfast(data_matrix)

        if y_labels is not None:
            if n_labels is None:
                step = 1
            else:
                step = max(len(y_labels) // n_labels, 1)
            self.ax.set_yticks(step / 2 + np.arange(len(y_labels), step=step))
            self.ax.set_yticklabels(y_labels[::step])
        if x_labels is not None:
            if n_labels is None:
                step = 1
            else:
                step = max(len(x_labels) // n_labels, 1)
            self.ax.set_xticks(step / 2 + np.arange(len(x_labels), step=step))
            self.ax.set_xticklabels(x_labels[::step])
            # xticks = self.ax.get_xticks()
            # new_xticks = np.array(x_labels)[xticks[:-1].astype(int)]
            # self.ax.set_xticks(xticks.astype(int))
            # self.ax.set_xticklabels(new_xticks)

    def update_axvlines(self, data_vector, **kwargs):
        for i, xcoord in enumerate(data_vector):
            if i in self.axvlines.keys():
                self.axvlines[i].set_xdata(xcoord)
            else:
                color = kwargs.get("color", None)
                if color is None:
                    color = f"C{i % 10}"
                axvline = self.ax.axvline(xcoord, ls=":", **kwargs)
                self.axvlines[i] = axvline

    def update_scatter(self, x_data, y_data, label="position", **kwargs):
        """ Plot x_data and y_data as scattered points.
        """
        if label in self.scatter.keys():
            self.scatter[label].set_data(x_data, y_data)

        else:
            (line,) = self.ax.plot(
                x_data, y_data, label=label, linestyle="-", marker="o", **kwargs
            )
            self.scatter[label] = line

        self.reset_xlim()
        self.reset_ylim()

    def reset_ylim(self):
        # recompute the ax.dataLim
        self.ax.relim()

        # limit ymin and ymax to chosen window.
        ymin, ymax = self.ax.get_ylim()
        ymax_new = min(max(self.ax.dataLim.y1, ymax), self.max_ylim)
        ymin_new = max(min(self.ax.dataLim.y0, ymin), self.min_ylim)

        # update ax.viewLim using new ax.dataLim
        self.ax.set_ylim(ymin_new, ymax_new)

    def reset_xlim(self):
        # recompute the ax.dataLim
        self.ax.relim()

        # limit xmin and xmax to chosen window.
        xmin, xmax = self.ax.get_xlim()
        xmax_new = min(max(self.ax.dataLim.x1, xmax), self.max_xlim)
        xmin_new = max(min(self.ax.dataLim.x0, xmin), self.min_xlim)

        # update ax.viewLim using new ax.dataLim
        self.ax.set_xlim(xmin_new, xmax_new)


if __name__ == "__main__":
    test = LivePlotter()
