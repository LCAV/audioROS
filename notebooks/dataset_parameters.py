#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
dataset_parameters.py: Parameters for parsing different datasets.
"""

kwargs_standard = {
    "slope": 4000 / 30,
    "offset": 400,
    "delta": 6,
    "min_freq": 900,
    "max_freq": 5000,
}

kwargs_datasets = {
    "2020_12_9_rotating": {
        "audio_deck": {**kwargs_standard, "min_time": 9, "max_time": 36,},
        "measurement": {
            **kwargs_standard,
            "min_time": 2,
            "max_time": 35,
            #'min_time': 7,
            #'max_time': 34,
        },
    },
    "2020_11_26_wall": {
        "audio_deck": {**kwargs_standard, "min_time": 9, "max_time": 36,},
        "measurement": {**kwargs_standard, "min_time": 6, "max_time": 31.5,},
    },
    "2021_02_09_wall": {
        "audio_deck": {**kwargs_standard, "min_time": 5, "max_time": 37,},
        "measurement": {**kwargs_standard, "min_time": 3, "max_time": 35,},
    },
    "2021_02_09_wall_tukey": {
        "audio_deck": {**kwargs_standard, "min_time": 5, "max_time": 37,},
        "measurement": {**kwargs_standard, "min_time": 2, "max_time": 34,},
    },
    "2021_02_19_windows": {
        "audio_deck": {
            **kwargs_standard,
            "offset": 1400,
            "min_freq": 2000,
            "max_freq": 6000,
            "min_time": 5,
            "max_time": 37,
        },
        "measurement": {
            **kwargs_standard,
            "offset": 1400,
            "min_freq": 2000,
            "max_freq": 6000,
            "min_time": 3,
            "max_time": 35,
        },
    },
    "2021_02_23_wall": {
        "audio_deck": {
            **kwargs_standard,
            "offset": 2400,
            "min_freq": 2900,
            "max_freq": 5000,
            "min_time": 5,
            "max_time": 22,
            "mag_thresh": 1e-3,  # important
        },
        "measurement": {
            **kwargs_standard,
            "offset": 2400,
            "min_freq": 2900,
            "max_freq": 5000,
            "min_time": 2,
            "max_time": 19,
        },
    },
    "2021_02_25_wall": {
        "audio_deck": {
            **kwargs_standard,
            "offset": 2400,
            "min_freq": 2900,
            "max_freq": 5000,
            "min_time": 5,
            "max_time": 22,
            "mag_thresh": 1,  # important
        },
        "measurement": {
            **kwargs_standard,
            "offset": 2400,
            "min_freq": 2900,
            "max_freq": 5000,
            "min_time": 2,
            "max_time": 19,
        },
    },
    "2021_03_01_flying": {
        "audio_deck": {
            "offset": None,
            "slope": None,
            "min_freq": 2900,
            "max_freq": 5000,
            "min_time": 10,
            "max_time": 27,
        },
    },
    "2021_04_30_hover": {
        "audio_deck": {
            "offset": None,
            "slope": None,
            "min_freq": 2000,
            "max_freq": 6000,
            "min_time": 0,
            "max_time": 12,  # not active
            "n_spurious": 1,  # important, because only one measurement per frequency.
        },
    },
    "2021_04_30_stepper": {
        "audio_deck": {
            "offset": None,
            "slope": None,
            "min_freq": 2000,
            "max_freq": 6000,
            "min_time": 0,
            "max_time": 30,  # not active
            "mag_thresh": 1,
            "n_spurious": 1,  # important, because only one measurement per frequency.
        },
    },
    "2021_05_04_flying": {
        "audio_deck": {
            "min_freq": 2000,
            "max_freq": 6000,
            "min_time": 0,
            "max_time": 60,  # not active
            "n_spurious": 1,  # important, because only one measurement per frequency.
        },
    },
    "2021_05_04_linear": {
        "audio_deck": {
            "min_freq": 2000,
            "max_freq": 6000,
            "min_time": 0,
            "max_time": 60,  # not active
            "n_spurious": 1,  # important, because only one measurement per frequency.
        },
    },
    "2021_06_08_epuck_stepper": {
        "audio_deck": {
            "min_freq": 2000,
            "max_freq": 6000,
            "min_time": 0,
            "max_time": 60,  # not active
            "n_spurious": 1,  # important, because only one measurement per frequency.
        },
        "measurement": {
            "min_freq": 2000,
            "max_freq": 6000,
            "min_time": 0,
            "max_time": 60,  # not active
            "n_spurious": 1,  # important, because only one measurement per frequency.
        },
    },
    "2021_06_08_epuck_stepper": {
        "audio_deck": {
            "min_freq": 2000,
            "max_freq": 6000,
            "min_time": 0,
            "max_time": 60,  # not active
            "n_spurious": 1,  # important, because only one measurement per frequency.
        },
        "measurement": {
            "min_freq": 2000,
            "max_freq": 6000,
            "min_time": 0,
            "max_time": 4,
            "n_spurious": 1,  # important, because only one measurement per frequency.
            "std_thresh": 1e10,  # remove std bigger than this
            "mag_thresh": 0,  # remove mag smaller than this
        },
    },
}