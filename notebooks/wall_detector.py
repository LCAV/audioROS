#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
wall_detector.py: Create and exploit distance-frequency matrix.
"""

import time

import pandas as pd
import scipy.interpolate

from frequency_analysis import *

from dataset_parameters import kwargs_datasets

ANGLE = 0
DISTANCE = 0
METHOD = np.nanmedian

YAW_DEG = 0  # drone angle used for when nothing else is given.

# parameters for cleaning signals
N_SPURIOUS = 2
MAG_THRESH = 1e-3
STD_THRESH = 2.0  # TODO(FD) this doesn't behave as expected
DELTA_MERGE_FREQ = 50
RATIO_MISSING_ALLOWED = 0.2


def normalize_df_matrix(df_matrix, freqs, method="calibration-offline"):
    df_matrix_normalized = np.zeros_like(df_matrix)
    # we already pass the interpolation function (more efficient)
    if type(method) == scipy.interpolate.interpolate.interp1d:
        calib_values = method(list(freqs))[:, :, None]
        for i in range(df_matrix.shape[0]):
            df_matrix_normalized[i] = df_matrix[i] / calib_values[i]

    elif callable(method):
        calib_values = method(list(freqs))[:, :, None]
        for i in range(df_matrix.shape[0]):
            df_matrix_normalized[i] = df_matrix[i] / calib_values[i]

    elif type(method) == np.ndarray:
        calib_values = method
        for i in range(df_matrix.shape[0]):
            df_matrix_normalized[i] = df_matrix[i] / calib_values[i]

    elif method == "calibration-offline":
        from calibration import get_calibration_function

        calib_function = get_calibration_function(plot=False)
        calib_values = calib_function(list(freqs))[i, :, None]

        for i in range(df_matrix.shape[0]):
            df_matrix_normalized[i] = df_matrix[i] / calib_values[i]

    elif method == "calibration-online":
        calib_values = np.nanmedian(df_matrix, axis=2)[:, :, None]
        for i in range(df_matrix.shape[0]):
            df_matrix_normalized[i] = df_matrix[i] / calib_values[i]

        # sanity check
        medians = np.nanmedian(df_matrix_normalized, axis=2)
        np.testing.assert_allclose(medians[~np.isnan(medians)], 1.0)

    elif method == "zero_to_one":
        calib_values = np.nanmin(df_matrix, axis=2)[:, :, None]
        for i in range(df_matrix.shape[0]):
            min_ = np.nanmin(df_matrix[i], axis=-1)
            max_ = np.nanmax(df_matrix[i], axis=-1)
            df_matrix_normalized[i] = (df_matrix[i] - min_) / (max_ - min_)

        # sanity check
        np.testing.assert_allclose(np.nanmax(df_matrix_normalized, axis=2), 1.0)
        np.testing.assert_allclose(np.nanmin(df_matrix_normalized, axis=2), 0.0)

    elif method == "zero_median":
        calib_values = np.nanmedian(df_matrix, axis=2)[:, :, None]
        for i in range(df_matrix.shape[0]):
            df_matrix_normalized[i] = df_matrix[i] - calib_values[i]

    elif method == "zero_mean":
        calib_values = np.nanmean(df_matrix, axis=2)[:, :, None]
        for i in range(df_matrix.shape[0]):
            df_matrix_normalized[i] = df_matrix[i] - calib_values[i]

    elif method == "standardize":
        calib_median = np.nanmedian(df_matrix, axis=2)[:, :, None]
        calib_std = np.nanstd(df_matrix, axis=2)[:, :, None]
        for i in range(df_matrix.shape[0]):
            df_matrix_normalized[i] = (df_matrix[i] - calib_median[i]) / calib_std[i]
        calib_values = calib_std

        # sanity check
        new_std = np.nanstd(df_matrix_normalized, axis=2)
        np.testing.assert_allclose(new_std[~np.isnan(new_std)], 1.0)
        new_mean = np.nanmedian(df_matrix_normalized, axis=2)
        np.testing.assert_allclose(
            new_mean[~np.isnan(new_mean)], 0.0, rtol=1, atol=1e-10
        )

    elif method == "normalize":
        # print(calib_values.shape)
        calib_values = np.sqrt(np.nansum(df_matrix ** 2, axis=2))[:, :, None]
        for i in range(df_matrix.shape[0]):
            df_matrix_normalized[i] = df_matrix[i] / calib_values[i] * 3
    else:
        raise ValueError(method)

    return df_matrix_normalized, calib_values


def prune_df_matrix(
    df_matrix, frequencies, ratio_missing_allowed=RATIO_MISSING_ALLOWED, verbose=False
):
    """ Remove all rows with more than a certain percentage of missing values """

    count_missing = np.sum(np.isnan(df_matrix), axis=(2))
    freq_i = np.where(
        np.all(count_missing / df_matrix.shape[2] <= ratio_missing_allowed, axis=0)
    )[0]
    if verbose:
        print(f"keeping {len(freq_i)}/{len(frequencies)} frequencies")
    return df_matrix[:, freq_i, :], frequencies[freq_i], freq_i


def clip(df, min_val, max_val, name="frequency"):
    if min_val is None:
        min_val = min(df[name])
    if max_val is None:
        max_val = max(df[name])
    return df.loc[(df[name] >= min_val) & (df[name] <= max_val)]


def clip_both(df, min_freq, max_freq, min_dist, max_dist):
    df = clip(df, min_freq, max_freq, name="frequency")
    return clip(df, min_dist, max_dist, name="distance")


def sort_and_clip(df, min_val, max_val, name="frequency"):
    df = df.sort_values(by=name, axis=0, inplace=False)
    if min_val is None:
        min_val = min(df[name])
    if max_val is None:
        max_val = max(df[name])
    return df.loc[(df[name] >= min_val) & (df[name] <= max_val)]


def normalized_std(values, method=METHOD):
    method_values = method(values)
    if method_values != 0:
        return np.nanstd(values) / method_values
    else:
        return 0


def get_abs_fft(f_slice, window=None, n_max=1000 ):
    import scipy.signal.windows
    f_slice_norm = f_slice - np.mean(f_slice)
    n = max(len(f_slice), n_max)
    if window is not None:
        w = scipy.signal.windows.get_window(window, len(f_slice_norm))
        f_slice_norm *= w
    return np.abs(np.fft.rfft(f_slice_norm, n=n))

def get_interference_distances(frequencies, mic_idx=1, distance_range=None, n_max=1000):
    from constants import SPEED_OF_SOUND
    from simulation import get_orthogonal_distance_from_global

    n = max(len(frequencies), n_max)
    df = np.mean(frequencies[1:] - frequencies[:-1])
    deltas_cm = np.fft.rfftfreq(n, df) * SPEED_OF_SOUND * 100
    distances = get_orthogonal_distance_from_global(
        yaw_deg=YAW_DEG, deltas_cm=deltas_cm, mic_idx=mic_idx
    )
    if distance_range is not None:
        mask = (distances >= distance_range[0]) & (distances <= distance_range[1])
        distances = distances[mask]
    else:
        mask = None
    return distances, mask


def get_probability_fft(
    f_slice, frequencies, window=None, mic_idx=1, distance_range=None, n_max=1000
):
    abs_fft = get_abs_fft(f_slice, window, n_max)
    distances, mask = get_interference_distances(frequencies, mic_idx, distance_range, n_max=n_max)
    if mask is not None:
        abs_fft = abs_fft[mask]
    prob = abs_fft / np.sum(abs_fft)
    return distances, prob


def get_posterior(abs_fft, sigma=None):
    N = len(abs_fft)
    periodogram = 1/N * abs_fft**2
    #print('periodogram:', np.min(periodogram), np.max(periodogram))

    if sigma is not None:
        periodogram /= sigma**2

        # TODO(FD) we do below for numerical reasons. its effect 
        # is undone by later exponentiation anyways. Make sure
        # this really as no effect on the result.
        periodogram -= np.max(periodogram) 
        #print('exponent:', np.min(periodogram), np.max(periodogram))
        posterior = np.exp(periodogram)
    else:
        d_bar = 1/N * np.sum(abs_fft**2)
        posterior = (1 - 2*periodogram/(N*d_bar))**((2-N)/2)
        #posterior = np.exp(periodogram)

    posterior /= np.sum(posterior)
    return posterior


def get_probability_bayes(
    f_slice, frequencies, window=None, mic_idx=1, distance_range=None, n_max=1000, sigma=None
):
    abs_fft = get_abs_fft(f_slice, window, n_max=n_max) # magnitude
    distances, mask = get_interference_distances(frequencies, mic_idx, distance_range, n_max=n_max)
    if mask is not None:
        abs_fft = abs_fft[mask]

    posterior = get_posterior(abs_fft, sigma)
    return distances, posterior


def get_probability_cost(
    f_slice,
    frequencies,
    distances,
    ax=None,
    mic_idx=1,
    relative_ds=None,
    absolute_yaws=None,
):
    from simulation import get_freq_slice_theory

    if absolute_yaws is not None:
        yaw_deg = absolute_yaws
    else:
        yaw_deg = YAW_DEG

    f_slice_norm = f_slice - np.mean(f_slice)
    f_slice_norm /= np.std(f_slice_norm)

    probs = []
    for d in distances:
        if relative_ds is not None:
            d += relative_ds
        f_slice_theory = get_freq_slice_theory(frequencies, d, yaw_deg)[:, mic_idx]
        f_slice_theory -= np.mean(f_slice_theory)
        f_slice_theory /= np.std(f_slice_theory)
        probs.append(np.exp(-np.linalg.norm(f_slice_theory - f_slice_norm)))

        if ax is not None:
            ax.plot(frequencies, f_slice_theory, color="black")

    if ax is not None:
        ax.plot(frequencies, f_slice_norm, color="green")

    probs /= np.sum(probs)
    return probs


def get_approach_angle_fft(
    d_slice, frequency, relative_distances_cm, window=None, n_max=1000, bayes=False, sigma=None
):
    import scipy.signal.windows
    from constants import SPEED_OF_SOUND
    from math import floor

    n = max(len(d_slice), n_max)
    d_slice_norm = d_slice - np.mean(d_slice)

    if window is not None:
        w = scipy.signal.windows.get_window(window, len(d_slice_norm))
        d_slice_norm *= w

    fft = np.fft.rfft(d_slice_norm, n=n)
    d_m = np.mean(relative_distances_cm[1:] - relative_distances_cm[:-1]) * 1e-2

    # TODO(FD) fix below approximation. 
    period_90 = (2*frequency) / SPEED_OF_SOUND # 1/m in terms of orthogonal distance (approximation)
    periods_k = (np.arange(0, n//2+1)) / (d_m * n) # 1/m
    sines_gamma = periods_k / period_90
    #if np.any(sines_gamma>1):
    #    print(f'Values bigger than 1: {np.sum(sines_gamma>1)}/{len(sines_gamma)}')
    abs_fft = np.abs(fft)[sines_gamma <= 1]
    sines_gamma = sines_gamma[sines_gamma <= 1]

    if bayes: 
        prob = get_posterior(abs_fft, sigma)
    else:
        prob = abs_fft / np.sum(abs_fft)
    return sines_gamma, prob


def get_approach_angle_cost(
        d_slice, frequency, relative_distances_cm, 
        start_distances_grid_cm, gammas_grid_deg, mic_idx=1, ax=None
    ): 
    from simulation import get_dist_slice_theory
    yaw_deg = YAW_DEG

    d_slice_norm = d_slice - np.mean(d_slice)
    d_slice_norm /= np.std(d_slice_norm)

    probs = np.zeros((len(start_distances_grid_cm), len(gammas_grid_deg))) 
    for i, start_distance_cm in enumerate(start_distances_grid_cm):
        for j, gamma_deg in enumerate(gammas_grid_deg):
            distances_cm = start_distance_cm - relative_distances_cm * np.sin(gamma_deg/180*np.pi)
            assert np.all(distances_cm >= 0)
            d_slice_theory = get_dist_slice_theory(frequency, distances_cm, yaw_deg)[:, mic_idx]
            d_slice_theory -= np.nanmean(d_slice_theory)
            std = np.nanstd(d_slice_theory)
            if std > 0:
                d_slice_theory /= std
            assert d_slice_theory.shape == d_slice_norm.shape
            loss = np.linalg.norm(d_slice_theory - d_slice_norm)
            probs[i, j] = np.exp(-loss)

            if ax is not None:
                ax.plot(distances_cm, d_slice_theory, label=f"{start_distance_cm}cm, {gamma_deg}deg")
    probs /= np.nansum(probs)
    return probs


class WallDetector(object):
    def __init__(
        self, params={}, exp_name=None, mic_type="audio_deck", interpolation=""
    ):
        self.df = pd.DataFrame(
            columns=[
                "time",
                "counter",
                "mic",
                "frequency",
                "distance",
                "angle",
                "magnitude",
            ]
        )
        self.params = params
        self.current_spectrogram = self.current_freqs = None
        self.interpolation = interpolation
        self.n_mics = 4 if (mic_type == "audio_deck") else 1
        # self.n_spurious = 1 if (mic_type == 'audio_deck') else 10
        self.mic_indices = range(4) if (mic_type == "audio_deck") else [1]
        if exp_name is not None:
            self.params.update(kwargs_datasets[exp_name][mic_type])

    def init_from_row(exp_name, row, interpolation="", verbose=False):
        wall_detector = WallDetector(
            exp_name=exp_name, mic_type=row.mic_type, interpolation=interpolation
        )
        try:
            wall_detector.fill_from_row(row, verbose=verbose)
        except:
            print("skipping", row)
            raise
            return None
        wall_detector.remove_bad_freqs(verbose=verbose)
        wall_detector.merge_close_freqs(verbose=verbose)
        wall_detector.remove_spurious_freqs(verbose=verbose)
        return wall_detector

    def get_linear_kwargs(self):
        kwargs = {
            key: self.params.get(key, None) for key in ["delta", "offset", "slope"]
        }
        if any([v is None for v in kwargs.values()]):
            return None
        else:
            return kwargs

    def get_box_kwargs(self):
        kwargs = {
            key: self.params.get(key, None)
            for key in ["min_freq", "max_freq", "min_time", "max_time"]
        }
        if any([v is None for v in kwargs.values()]):
            return None
        else:
            return kwargs

    def fill_from_row(self, row, verbose=False, mask=True):
        distance = row.get("distance", DISTANCE)
        if distance is None:  # otherwise these rows get excluded in groupby operations
            distance = DISTANCE
        angle = row.get("angle", ANGLE)
        return self.fill_from_data(
            row.frequencies_matrix,
            row.stft,
            distance=distance,
            angle=angle,
            times=row.seconds,
            verbose=verbose,
            mask=mask,
        )

    def fill_from_data(
        self,
        frequencies_matrix,
        stft,
        distance=DISTANCE,
        angle=ANGLE,
        times=None,
        verbose=False,
        mask=True,
    ):
        spec, freqs = get_spectrogram_raw(frequencies_matrix, stft)

        if mask:
            linear_kwargs = self.get_linear_kwargs()
            if linear_kwargs is not None:
                spec, freqs = apply_linear_mask(
                    spec, freqs, times=times, **linear_kwargs
                )
            box_kwargs = self.get_box_kwargs()
            if box_kwargs is not None:
                spec, freqs = apply_box_mask(spec, freqs, times=times, **box_kwargs)

        self.fill_from_spec(spec, freqs, distance, angle, verbose)
        self.current_spectrogram = spec
        self.current_freqs = freqs
        return spec, freqs

    def fill_from_spec(
        self, spec, freqs, distance=DISTANCE, angle=ANGLE, verbose=False
    ):
        df = psd_df_from_spec(
            spec, freqs, interpolation=self.interpolation, verbose=verbose
        )
        assert np.all(df.magnitude.values[~np.isnan(df.magnitude.values)] >= 0)
        df.loc[:, "distance"] = distance
        df.loc[:, "angle"] = angle
        # need ignore_index here to make sure that the final index is unique.
        self.df = pd.concat([self.df, df], ignore_index=True)

    def get_frequency_slice(
        self, distance=None, min_freq=None, max_freq=None, method=METHOD
    ):
        """ 
        :return: slice along one distance, of shape (mic x frequencies), frequencies
        """
        df = sort_and_clip(self.df, min_freq, max_freq, name="frequency")

        distances = df.distance.unique()
        if distance is None:
            if len(distances) > 1:
                print(
                    f"Warning: taking frequency slice over multiple distances: {distances}"
                )
        elif distance not in distances:
            print(f"Warning: did not find distance {distance}cm")
            bin_ = np.argmin(np.abs(distances - distance))
            distance = distances[bin_]
            print(f"Closest match: {distance}cm")
            df = df.loc[df.distance == distance]
        else:
            df = df.loc[df.distance == distance]

        slice_f = df.pivot_table(
            index="mic", columns="frequency", values="magnitude", aggfunc=method
        ).values
        freqs = df.frequency.unique()
        return slice_f, freqs

    def get_frequency_slice_with_std(
        self, distance=None, min_freq=None, max_freq=None, method=METHOD
    ):
        """ 
        :return: slice along one distance, of shape (mic x frequencies), std dev (mic x frequencies), frequencies
        """
        slice_f, freqs = self.get_frequency_slice(
            distance, min_freq, max_freq, method=METHOD
        )
        std_f, std_freqs = self.get_frequency_slice(
            distance,
            min_freq,
            max_freq,
            method=lambda x: normalized_std(x, method=METHOD),
        )
        np.testing.assert_equal(freqs, std_freqs)
        return slice_f, std_f, freqs

    def get_distance_slice(
        self, frequency, min_dist=None, max_dist=None, method=METHOD
    ):
        """ 
        :return: slice along one frequency, of shape (mic x distances), distances
        """
        df = sort_and_clip(self.df, min_dist, max_dist, name="distance")

        frequencies = df.frequency.unique()
        if frequency is None:
            if len(frequencies) > 1:
                print(
                    f"Warning: taking frequency slice over multiple frequencies: {frequencies}"
                )
        elif frequency not in frequencies:
            print(f"Warning: did not find frequency {frequency}Hz")
            bin_ = np.argmin(np.abs(frequencies - frequency))
            frequency = frequencies[bin_]
            print(f"Closest match: {frequency}Hz")
            df = df.loc[df.frequency == frequency]
        else:
            df = df.loc[df.frequency == frequency]

        slice_d = df.pivot_table(
            index="mic", columns="distance", values="magnitude", aggfunc=method
        ).values
        return slice_d, df.distance.unique()

    def get_df_matrix(
        self, max_freq=None, min_freq=None, min_dist=None, max_dist=None, method=METHOD
    ):
        df = clip_both(self.df, min_dist, max_dist, min_freq, max_freq)

        frequencies = np.sort(np.array(df.frequency.unique(), dtype=np.float))
        distances = np.sort(np.array(df.distance.unique(), dtype=np.float))

        # Attention: below only works if ferquency and distance "masks" are the same for each mic.
        # This is currently the case.
        # Otherwise, get_df_matrix_old can be used.
        df_matrix = np.full((self.n_mics, len(frequencies), len(distances)), np.nan)
        for i_mic, df_mic in df.groupby("mic", sort=True):
            pt = df_mic.pivot_table(
                index="frequency",
                columns="distance",
                values="magnitude",
                aggfunc=method,
            )
            frequencies_pt = pt.index.values

            distances_pt = pt.columns.values
            np.testing.assert_allclose(distances_pt, distances)

            # find frequencies mask
            if (len(frequencies_pt) != len(frequencies)) or not np.allclose(
                frequencies, frequencies_pt
            ):
                print(f"Warning: mismatch between freqs for mic{i_mic}")
                f_indices = np.where((frequencies[None, :] == frequencies_pt[:, None]))[
                    0
                ]
                np.testing.assert_allclose(frequencies[f_indices], frequencies_pt)
                df_matrix[i_mic, f_indices, :] = pt.values
            else:
                df_matrix[i_mic, :, :] = pt.values

        return df_matrix, distances, frequencies

    def get_df_matrix_old(
        self, max_freq=None, min_freq=None, min_dist=None, max_dist=None, method=METHOD
    ):
        df = clip_both(self.df, min_dist, max_dist, min_freq, max_freq)

        frequencies = np.sort(np.array(df.frequency.unique(), dtype=np.float))
        distances = np.sort(np.array(df.distance.unique(), dtype=np.float))

        df_matrix = np.full((self.n_mics, len(frequencies), len(distances)), np.nan)
        for (d, f, m), df in self.df.groupby(["distance", "frequency", "mic"]):
            try:
                d_i = np.where(distances == d)[0][0]
                f_i = np.where(frequencies == f)[0][0]
            except:
                continue
            df_matrix[m, f_i, d_i] = method(df.magnitude.values)
        return df_matrix, distances, frequencies

    def merge_close_freqs(
        self, delta_merge_freq=DELTA_MERGE_FREQ, verbose=False, dryrun=False
    ):
        """ Merge frequency bins that are closer than delta. """
        unique_frequencies = np.sort(self.df.frequency.unique())
        indices = np.where(
            np.abs(unique_frequencies[1:] - unique_frequencies[:-1]) > delta_merge_freq
        )[0]
        new_frequencies = unique_frequencies[[0] + list(indices + 1)]
        if verbose:
            print(
                f"merge_close_freqs: removing {len(unique_frequencies)-len(new_frequencies)} rows."
            )

        if dryrun:
            return unique_frequencies, new_frequencies

        for f in new_frequencies:
            self.df.loc[
                np.abs(self.df.frequency - f) < delta_merge_freq, "frequency"
            ] = f
        return unique_frequencies, new_frequencies

    def remove_spurious_freqs(self, n_spurious=N_SPURIOUS, verbose=False, dryrun=False):
        """ Remove the frequencies for which we only have less than n_min measurements. """
        remove_rows = []

        # average number of measurements per frequency and distance.
        # choose any column for 'magnitude', it doesn't matter.
        values = (
            self.df.loc[self.df.mic == 0]
            .groupby(["frequency", "distance"])
            .agg("count")["magnitude"]
            .values
        )
        if n_spurious is None:
            n_spurious = np.quantile(values, 0.7)

        if verbose:
            print(
                f"remove frequencies with less than {n_spurious} measurements. {np.min(values), np.max(values), np.median(values)}"
            )

        for (freq, mic, distance), df in self.df.groupby(
            ["frequency", "mic", "distance"]
        ):
            if len(df) < n_spurious:
                # if verbose:
                #   print('removing', df)
                remove_rows += list(df.index.values)
        if verbose:
            print(f"remove_spurious_freqs: removing {len(remove_rows)} rows.")
        if not dryrun:
            self.df = self.df.drop(index=remove_rows, inplace=False)
        if len(self.df) == 0:
            print("Warning: remove_spurious_freqs removed all rows.")

        # if there are no nans left in certain columns, we can
        # convert them to numeric.
        self.df = self.df.apply(pd.to_numeric, axis=0, downcast="integer")
        return len(remove_rows)

    def remove_bad_freqs(
        self, mag_thresh=MAG_THRESH, std_thresh=STD_THRESH, verbose=False, dryrun=False
    ):
        """ Remove the frequencies for which we only have less than n_min measurements. """
        remove_rows = []

        for freq, df in self.df.groupby("frequency", sort=True):
            vals = df.magnitude.values
            if np.nanmedian(vals) < mag_thresh:
                if verbose:
                    print(f"removing {freq} with median {np.nanmedian(vals)}")
                remove_rows += list(df.index.values)
            elif normalized_std(vals) > std_thresh:
                if verbose:
                    print(f"removing {freq} with std {normalized_std(vals)}")
                remove_rows += list(df.index.values)
            # else:
            #    print(f"keeping {freq}: {np.nanmedian(vals):.2e}, {normalized_std(vals):.2e}")
        if verbose:
            print(f"remove_bad_freqs: removing {len(remove_rows)} rows.")
        if not dryrun:
            self.df = self.df.drop(index=remove_rows, inplace=False)

        if len(self.df) == 0:
            print("Warning: remove_bad_freqs removed all rows.")

        # if there are no nans left in certain columns, we can
        # convert them to numeric.
        self.df = self.df.apply(pd.to_numeric, axis=0, downcast="integer")
        return len(remove_rows)

    def fill_from_backup(self, exp_name, mic_type=""):
        fname = f"results/backup_{exp_name}_{mic_type}.pkl"
        self.df = pd.read_pickle(fname)
        print("read", fname)

    def backup(self, exp_name, mic_type=""):
        fname = f"results/backup_{exp_name}_{mic_type}.pkl"
        pd.to_pickle(self.df, fname)
        print("saved", fname)
