#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
live_plotter.py: live data plotting using matplotlib.
"""

import math

import matplotlib.pylab as plt

MAX_YLIM = math.inf # set to inf for no effect.
MIN_YLIM = -math.inf # set to -inf for no effect.

class LivePlotter(object):
    def __init__(self, max_ylim=MAX_YLIM, min_ylim=MIN_YLIM):

        self.max_ylim = max_ylim
        self.min_ylim = min_ylim

        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel('frequency [Hz]')
        self.ax.set_ylabel('magnitude [-]')
        self.lines = {}
        # Need the block argument to make sure the script continues after
        # plotting the figure. 
        plt.show(block=False)

    def update_animation(self, data_matrix, x_data=None, labels=None):
        """ Plot each row of data_matrix as one line.
        """
        for i in range(data_matrix.shape[0]):
            if (i in self.lines.keys()):
                self.lines[i].set_ydata(data_matrix[i, :])
            else:
                x_data = range(data_matrix.shape[1]) if x_data is not None else x_data
                label = labels[i] if labels is not None else None
                line, = self.ax.semilogy(x_data, data_matrix[i, :], color=f"C{i % 10}", 
                                         label=label)
                self.lines[i] = line

        self.ax.legend(loc='upper right')
        self.reset_ylim()

        # without this, the plot does not get updated live.
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

if __name__ == "__main__":
    test = LivePlotter()
