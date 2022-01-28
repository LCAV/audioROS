#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_get_estimate.py:  Test get_estimate function for simple Gaussian.
"""
import sys, os

import numpy as np
from scipy import stats

sys.path.append(os.path.abspath(os.path.join(os.getcwd(), "../")))
from utils.moving_estimators import get_estimate
import matplotlib.pylab as plt


PLOT = False


def get_gaussian(stds, means, step=0.1):
    mins = means - 3 * stds
    maxs = means + 3 * stds
    min_ = min(mins)
    max_ = max(maxs)
    values = np.arange(min_, max_ + step, step=step)

    probs = np.zeros_like(values)
    for mean, std in zip(means, stds):
        norm = stats.norm(loc=mean, scale=std)
        probs += norm.pdf(values)
    return values, probs


def test_multimodal_gaussian():
    mean_min = 0
    mean_max = 5
    std_min = 2
    std_max = 1
    stds = np.array([std_min, std_max])
    means = np.array([mean_min, mean_max])
    values, probs = get_gaussian(stds, means)
    plt.figure()
    plt.plot(values, probs)

    mean_est, std_est = get_estimate(values, probs, method="peak")
    print(mean_est, std_est)
    assert np.abs(mean_est - mean_max) < 1e-10
    assert np.abs(std_est - std_max) < 0.1
    mean_est, std_est = get_estimate(values, probs, method="max")
    print(mean_est, std_est)
    assert np.abs(mean_est - mean_max) < 1e-10
    # assert np.abs(std_est - std) < 0.1


def test_single_gaussian():
    mean = 0
    std = 2
    stds = np.array([std])
    means = np.array([mean])
    values, probs = get_gaussian(stds, means)
    plt.figure()
    plt.plot(values, probs)

    mean_est, std_est = get_estimate(values, probs, method="peak")
    print(mean_est, std_est)
    assert np.abs(mean_est - mean) < 1e-10
    assert np.abs(std_est - std) < 0.1
    mean_est, std_est = get_estimate(values, probs, method="max")
    print(mean_est, std_est)
    assert np.abs(mean_est - mean) < 1e-10
    assert np.abs(std_est - std) < 0.1
    mean_est, std_est = get_estimate(values, probs, method="mean")
    print(mean_est, std_est)
    assert np.abs(mean_est - mean) < 1e-10
    assert np.abs(std_est - std) < 0.1
    mean_est, std_est = get_estimate(values, probs, method="mean", unbiased=False)
    print(mean_est, std_est)
    assert np.abs(mean_est - mean) < 1e-10
    assert np.abs(std_est - std) < 0.1


test_single_gaussian()
test_multimodal_gaussian()
