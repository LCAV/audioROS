#!/usr/bin/env python
# coding: utf-8
"""
signals.py: Functions to generate different sound signals.
"""

from abc import ABC, abstractmethod
from math import ceil
import os

import matplotlib.pylab as plt
import numpy as np

from .constants import AUDIO_SAMPLING_RATE

MIN_FREQ = 200
MAX_FREQ = 7000


def generate_signal_sweep(
    Fs, duration_sec, min_freq=MIN_FREQ, max_freq=MAX_FREQ, step=False, **kwargs
):
    num_samples = int(ceil(Fs * duration_sec))

    if step:
        # stay at each frequency at least 10 buffers at audio_deck sampling rate.
        duration_per_freq = 2048.0 * 5 / 48000  # seconds
        max_samples_per_frequency = int(ceil(Fs * duration_per_freq))
        num_frequencies = int(ceil(num_samples / max_samples_per_frequency))
        frequencies = np.exp(
            np.linspace(np.log(min_freq), np.log(max_freq), num_frequencies)
        )
        frequencies_repeat = np.repeat(frequencies, max_samples_per_frequency)[
            :num_samples
        ]
        times = np.tile(
            np.linspace(0, duration_per_freq, max_samples_per_frequency),
            num_frequencies,
        )[:num_samples]
    else:
        frequencies_repeat = np.exp(
            np.linspace(np.log(min_freq), np.log(max_freq), num_samples)
        )
        times = np.linspace(0, duration_sec, num_samples)
    return np.sin(2 * np.pi * np.multiply(frequencies_repeat, times))


def generate_signal_mono(Fs, duration_sec, frequency_hz=1000, **kwargs):
    num_samples = int(ceil(Fs * duration_sec))
    times = np.linspace(0, duration_sec, num_samples)
    phase_offset = kwargs.get("phase_offset", 0)
    return np.sin(2 * np.pi * frequency_hz * times + phase_offset)


def generate_signal_random(Fs, duration_sec, **kwargs):
    """
    :returns: ndarray, uniform white noise between -1 and 1
    """
    num_samples = int(ceil(Fs * duration_sec))
    return 2.0 * (np.random.rand(num_samples) - 0.5)


def generate_signal_real(Fs, duration_sec, fname, **kwargs):
    from scipy.io.wavfile import read

    if not os.path.isfile(fname):
        raise ValueError(f"{fname} does not exist.")

    print("reading", fname, end="")
    Fs_file, real_signal = read(fname)

    if Fs != Fs_file:
        q = Fs_file / Fs
        if q < 1:
            raise NotImplementedError("Cannot upsample file.")
        from scipy.signal import resample

        old_length = len(real_signal)
        num = int(old_length / q)
        print(
            f"Warning: specified Fs({Fs}) mismatches file ({Fs_file}). Upsampling by {q}"
        )
        real_signal = resample(real_signal, num=num)
        assert len(real_signal) * q == old_length, (old_length, q * len(real_signal))

    if duration_sec is not None:
        num_samples = int(ceil(Fs * duration_sec))
        if num_samples >= len(real_signal):
            print(
                f"Warning: requested more samples than available ({num_samples}>{len(real_signal)})"
            )
            return real_signal

        start_idx = np.random.choice(np.arange(len(real_signal) - num_samples))
        print(", starting at", start_idx)
        return real_signal[start_idx : start_idx + num_samples]
    print("\n")
    return real_signal


def generate_signal_zero(Fs, duration_sec, **kwargs):
    num_samples = int(ceil(Fs * duration_sec))
    return np.zeros(num_samples)


def generate_signal(
    Fs, duration_sec, signal_type="mono", min_dB=-50, max_dB=0, **kwargs
):
    if signal_type == "mono":
        signal = generate_signal_mono(Fs, duration_sec, **kwargs)
        signal = amplify_signal(signal, target_dB=max_dB)

    elif signal_type == "random":
        signal = generate_signal_random(Fs, duration_sec, **kwargs)
        signal = amplify_signal(signal, target_dB=max_dB)

    elif signal_type == "real":
        signal = generate_signal_real(Fs, duration_sec, **kwargs)
        signal = amplify_signal(signal, target_dB=max_dB)

    elif signal_type == "sweep":
        signal = generate_signal_sweep(Fs, duration_sec, **kwargs)
        signal = amplify_signal(signal, target_dB=max_dB)

    elif signal_type == "random_linear":
        signal = generate_signal_random(Fs, duration_sec, **kwargs)
        signal = linear_increase(signal, min_dB, max_dB)

    elif signal_type == "mono_linear":
        signal = generate_signal_mono(Fs, duration_sec, **kwargs)
        signal = linear_increase(signal, min_dB, max_dB)

    elif signal_type in [None, "none"]:
        signal = generate_signal_zero(Fs, duration_sec, **kwargs)

    else:
        raise ValueError(signal_type)

    # if np.any(signal > 1.0):
    #    print(f"Warning: signal is higher than 1.0. This could lead to clipping at the soundcard! Range: {np.min(signal)} to {np.max(signal)}")

    return signal


def get_power(signal, dB=True):
    """Get power density of signal (normalized by the length of the signal)

    :param signal: either 1-dim array or matrix of signals in rows. If signals has multiple rows,
    the average energy is calculated.

    """
    power = np.linalg.norm(signal) ** 2 / signal.size
    if dB:
        return 10 * np.log10(power)
    return power


def amplify_signal(signal, change_dB=None, target_dB=None, verbose=False):
    power_dB = get_power(signal)

    if change_dB is not None:
        target_dB = power_dB + change_dB

    elif target_dB is not None:
        change_dB = target_dB - power_dB

    ratio_amplitudes = np.sqrt(10 ** (change_dB / 10.0))
    if verbose:
        print("current power", power_dB)
        print("target power", target_dB)
        print("amplitudes ratio:", ratio_amplitudes)
    new_signal = ratio_amplitudes * signal

    new_dB = get_power(new_signal)
    assert abs(new_dB - target_dB) < 1e-3, f"new {new_dB} != target {target_dB}"
    return new_signal


def linear_increase(signal, min_dB=-50, max_dB=0):
    curr_dB = get_power(signal)
    gains_dB = np.linspace(min_dB, max_dB, len(signal)) - curr_dB
    gains = 10 ** (gains_dB / 20)
    return np.multiply(gains, signal)


# TODO(FD) simplify below using above functions.


class SourceSignal(ABC):
    def __init__(self, **params):
        self.params = params

    @abstractmethod
    def evaluate(self, times, noise=0):
        """
        :param t: array or list of time instances.
        """
        raise NotImplementedError("needs to be implemented by inheriting classes")

    def create_audio_sample(self, sound_duration=1, sampling_rate=AUDIO_SAMPLING_RATE):
        time_samples = np.arange(0, sound_duration, 1 / sampling_rate)
        assert len(time_samples) == sound_duration * sampling_rate
        return np.real(self.evaluate(time_samples, noise=0))


class MonoSignal(SourceSignal):
    def __init__(self, f=440, amplitude=1):
        omega = 2 * np.pi * f
        super(MonoSignal, self).__init__(f=f, amplitude=amplitude, omega=omega)

    def evaluate(self, times, noise=0):
        """
        :param t: array or list of time instances.
        """
        signal = self.params["amplitude"] * np.sin(
            self.params["omega"] * np.array(times)
        )
        if noise > 0:
            signal += np.random.normal(scale=noise, loc=0, size=signal.shape)
        return signal.flatten()


class WhiteSignal(SourceSignal):
    def __init__(self, sigma, amplitude=1):
        super(WhiteSignal, self).__init__(amplitude=amplitude, sigma=sigma)

    def evaluate(self, times, noise=0):
        """
        :param t: array or list of time instances.
        """
        signal = np.random.normal(scale=self.params["sigma"], loc=0, size=len(times))
        return signal.flatten()
