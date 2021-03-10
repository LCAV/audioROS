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


def get_probability_fft(
    f_slice, frequencies, window=None, mic_idx=1, distance_range=None
):
    import scipy.signal.windows
    from constants import SPEED_OF_SOUND
    from simulation import get_orthogonal_distance_from_global

    n = max(len(f_slice), 1000)

    f_slice_norm = f_slice - np.mean(f_slice)
    fft = np.abs(np.fft.rfft(f_slice_norm, n=n))
    if window is not None:
        w = scipy.signal.windows.get_window(window, len(fft))
        fft *= w

    df = np.mean(frequencies[1:] - frequencies[:-1])
    deltas = np.fft.rfftfreq(n, df) * SPEED_OF_SOUND * 100

    distances = get_orthogonal_distance_from_global(
        yaw_deg=YAW_DEG, deltas_cm=deltas, mic_idx=mic_idx
    )
    if distance_range is not None:
        mask = (distances >= distance_range[0]) & (distances <= distance_range[1])
        distances = distances[mask]
        fft = fft[mask]
    prob = fft / np.sum(fft)
    return distances, prob


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


class WallDetector(object):
    def __init__(self, params={}, exp_name=None, mic_type="audio_deck"):
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
        self.n_mics = 4 if (mic_type == "audio_deck") else 1
        # self.n_spurious = 1 if (mic_type == 'audio_deck') else 10
        self.mic_indices = range(4) if (mic_type == "audio_deck") else [1]
        if exp_name is not None:
            self.params.update(kwargs_datasets[exp_name][mic_type])

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
        if verbose:
            t1 = time.time()
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

        if verbose:
            print(f"after masking: found {len(freqs)} bins.")
        index_matrix = get_index_matrix(spec)
        self.fill_from_spec(spec, freqs, index_matrix, distance, angle, verbose)
        if verbose:
            print("fill_from_data time:", time.time() - t1)
            t1 = time.time()
        return spec, freqs

    def fill_from_spec(
        self, spec, freqs, index_matrix, distance=DISTANCE, angle=ANGLE, verbose=False
    ):
        df = psd_df_from_spec(spec, freqs, index_matrix)
        assert np.all(df.magnitude.values[~np.isnan(df.magnitude.values)] >= 0)
        if verbose:
            print(f"filling with {len(df)} new rows")

        df.loc[:, "distance"] = distance
        df.loc[:, "angle"] = angle

        df = df.apply(pd.to_numeric, axis=0, downcast="integer")
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
