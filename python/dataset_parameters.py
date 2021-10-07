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
            "min_freq": 2900,
            "max_freq": 5000,
            "min_time": 10,
            "max_time": 27,
        },
    },
    "2021_04_30_hover": {
        "audio_deck": {
            "max_time": 12,
            "n_spurious": 1,  # important, because only one measurement per frequency.
        },
    },
    "2021_04_30_stepper": {
        "audio_deck": {
            "mag_thresh": 1,
            "n_spurious": 1,  # important, because only one measurement per frequency.
        },
    },
    "2021_05_04_flying": {
        "audio_deck": {
            "n_spurious": 1,  # important, because only one measurement per frequency.
        },
        "appendix": [f"_{i}" for i in range(22, 25)],
    },
    "2021_05_04_linear": {
        "audio_deck": {
            "n_spurious": 1,  # important, because only one measurement per frequency.
        },
        "appendix": [f"_{i}" for i in range(1, 6)] + [f"_fast{i}" for i in range(1, 6)],
    },
    "2021_06_09_stepper": {
        "audio_deck": {
            "mag_thresh": 1e-2,
            "std_thresh": 100,
            "n_spurious": 1,  # important, because only one measurement per frequency.
        },
    },
    "2021_06_17_stepper": {
        "audio_deck": {
            "mag_thresh": 1e-2,
            "std_thresh": 100,
            "n_spurious": 1,  # important, because only one measurement per frequency.
        },
    },
    "2021_06_19_stepper": {
        "audio_deck": {
            "factor_outliers": 1e3,
            "mag_thresh": 1e-2,
            "std_thresh": 100,
            "n_spurious": 1,  # important, because only one measurement per frequency.
        },
    },
    "2021_07_07_stepper": {
        "audio_deck": {
            "factor_outliers": 1e3,
            "mag_thresh": 1e-2,
            "std_thresh": 100,
            "n_spurious": 1,  # important, because only one measurement per frequency.
        },
    },
    "2021_07_08_stepper": {
        "audio_deck": {
            "factor_outliers": 1e3,
            "mag_thresh": 1e-2,
            "std_thresh": 100,
            "n_spurious": 1,  # important, because only one measurement per frequency.
        },
    },
    "2021_07_08_stepper_slow": {
        "audio_deck": {
            "factor_outliers": 1e3,
            "mag_thresh": 1e-2,
            "std_thresh": 100,
            "n_spurious": 1,  # important, because only one measurement per frequency.
        },
    },
    "2021_07_08_stepper_fast": {
        "audio_deck": {
            "factor_outliers": 1e3,
            "mag_thresh": 1e-2,
            "std_thresh": 100,
            "n_spurious": 1,  # important, because only one measurement per frequency.
        },
    },
    "2021_07_14_propsweep": {"appendix": ["", "_thirdtry"]},
    "2021_07_14_flying": {
        "audio_deck": {
            "factor_outliers": 1e3,
            "mag_thresh": 1e-2,
            "std_thresh": 100,
            "n_spurious": 1,  # important, because only one measurement per frequency.
            "min_freq": 3000,
            "max_freq": 5000,
        },
        "appendix": [f"_{i}" for i in range(11, 20)],
    },
    "2021_07_14_flying_hover": {
        "audio_deck": {
            "factor_outliers": 1e3,
            "mag_thresh": 1e-2,
            "std_thresh": 100,
            "n_spurious": 1,  # important, because only one measurement per frequency.
            "min_freq": 3000,
            "max_freq": 5000,
        },
        "appendix": ["", "_2"],
    },
    "2021_07_27_hover": {
        "audio_deck": {
            "factor_outliers": 1e3,
            "mag_thresh": 1e-2,
            "std_thresh": 100,
            "n_spurious": 1,  # important, because only one measurement per frequency.
            "min_freq": 3000,
            "max_freq": 5000,
        },
        "appendix": ["", "_30", "_50"],
    },
    "2021_07_27_manual": {
        "audio_deck": {
            "factor_outliers": 1e3,
            "mag_thresh": 1e-2,
            "std_thresh": 100,
            "n_spurious": 1,  # important, because only one measurement per frequency.
            "min_freq": 3000,
            "max_freq": 5000,
        },
        "appendix": ["", "_2", "_3", "_4"],
    },
    "2021_07_27_epuck_wall": {
        "audio_deck": {
            "factor_outliers": 1e3,
            "mag_thresh": 1e4,
            "std_thresh": 100,
            "n_spurious": 1,  # important, because only one measurement per frequency.
            "min_freq": 1000,
            "max_freq": 6000,
        },
        "appendix": [""],
    },
    "2021_09_23_polar_measurement": {
        "audio_deck": {
            "factor_outliers": 1e3,
            "mag_thresh": 0,
            "std_thresh": 1e10,
            "n_spurious": 1,  # important, because only one measurement per frequency.
            "min_freq": 100,
            "max_freq": 6000,
        },
        "appendix": [""],
    },
    "2021_09_30_polar_measurement": {
        "audio_deck": {
            "factor_outliers": 1e3,
            "mag_thresh": 0,
            "std_thresh": 1e10,
            "n_spurious": 1,  # important, because only one measurement per frequency.
            "min_freq": 100,
            "max_freq": 6000,
        },
        "appendix": ["", "_loud"],
    },
    "2021_10_05_polar_measurement": {
        "audio_deck": {
            "factor_outliers": 1e3,
            "mag_thresh": 0,
            "std_thresh": 1e10,
            "n_spurious": 1,  # important, because only one measurement per frequency.
            "min_freq": 100,
            "max_freq": 6000,
        },
        "appendix": [""],
    },
    "2021_10_07_stepper": {
        "audio_deck": {
            "factor_outliers": 1e3,
            "mag_thresh": 0,
            "std_thresh": 1e10,
            "n_spurious": 1,  # important, because only one measurement per frequency.
            "min_freq": 100,
            "max_freq": 6000,
        },
        "appendix": [""],
    },
}
