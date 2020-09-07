#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
live_plotter.py: live data plotting using matplotlib.
"""

import math

import matplotlib
import matplotlib.pylab as plt

matplotlib.use("TkAgg")

MAX_YLIM = math.inf  # set to inf for no effect.
MIN_YLIM = -math.inf  # set to -inf for no effect.

MAX_XLIM = math.inf  # set to inf for no effect.
MIN_XLIM = -math.inf  # set to -inf for no effect.


class LivePlotter(object):
    def __init__(self, max_ylim=MAX_YLIM, min_ylim=MIN_YLIM, log=True, label='', max_xlim=MAX_XLIM, min_xlim=MIN_XLIM):
        self.max_ylim = max_ylim
        self.min_ylim = min_ylim
        self.max_xlim = max_xlim
        self.min_xlim = min_xlim
        self.log = log

        self.fig, self.ax = plt.subplots()
        self.fig.canvas.set_window_title(label)

        # containers for continuously updated data
        self.lines = {}
        self.axvlines = {}
        self.arrows = {}
        self.scatter = {}

        self.fig.canvas.mpl_connect("close_event", self.handle_close)

        # works with Tk backend: remove the frame so that user does not
        # wrongly close window.
        # win = plt.gcf().canvas.manager.window
        # win.overrideredirect(1)

        # Need the block argument to make sure the script continues after
        # plotting the figure.
        plt.show(block=False)

    def handle_close(self, evt):
        plt.close("all")
        print("closed all figures")

    def clear(self):
        for i in range(len(self.ax.lines)):
            self.ax.lines.pop()
        self.lines = {}
        self.axvlines = {}

    def update_arrow(self, origin_x, origin_y, dx, dy):
        """ Update arrow coordinates. 
        """
        if self.arrows: # 
            self.arrows['pointer'].set_data(origin_x + dx, origin_y + dy)
            self.arrows['line'].set_data([origin_x, origin_x + dx], [origin_y, origin_y + dy])
        else:
            (line,) = self.ax.plot(
                    origin_x+dx, origin_y+dy, color="black", marker=">"
                )
            self.arrows['pointer'] = line
            (line,) = self.ax.plot(
                    [origin_x, origin_x + dx], [origin_y, origin_y + dy], 
                    color="black" 
            )
            self.arrows['line'] = line
        # without this, the plot does not get updated live.
        self.fig.canvas.draw()

    def update_lines(self, data_matrix, x_data=None, labels=None):
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
                        x_data, data_matrix[i, :], color=f"C{i % 10}", label=label
                    )
                else:
                    (line,) = self.ax.plot(
                        x_data, data_matrix[i, :], color=f"C{i % 10}", label=label
                    )
                self.lines[i] = line

        self.ax.legend(loc="upper right")
        self.reset_ylim()

        # without this, the plot does not get updated live.
        self.fig.canvas.draw()

    def update_axvlines(self, data_vector):
        for i, xcoord in enumerate(data_vector):
            if i in self.axvlines.keys():
                self.axvlines[i].set_xdata(xcoord)
            else:
                axvline = self.ax.axvline(xcoord, color=f"C{i % 10}", ls=":")
                self.axvlines[i] = axvline

        self.fig.canvas.draw()

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


    def update_scatter(self, x_data, y_data):
        """ Plot x_data and y_data as scattered points.
        """
        if self.scatter:
            self.scatter["line"].set_data(x_data, y_data)

        else:
            (line,) = self.ax.plot(
                x_data, y_data, linestyle='-', marker='o'
            )
            self.scatter["line"] = line
        # without this, the plot does not get updated live.
        self.reset_xlim()
        self.reset_ylim()
        self.fig.canvas.draw()


if __name__ == "__main__":
    test = LivePlotter()
