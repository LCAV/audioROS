#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
parameters.py: Parameters for audio processing pipeline.
"""

TUKEY_ALPHA = 0.2  # alpha parameter for tukey window

WINDOW_NAMES = {0: "", 1: "hann", 2: "flattop", 3: "tukey"}

# below is found by increasing n_buffer and finding to what sum(window)/n_buffer converges.
WINDOW_CORRECTION = {
    0: 1.0,
    1: 0.5,
    2: 0.215579,
    3: 0.9,
}
