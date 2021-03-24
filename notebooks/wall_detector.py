#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
wall_detector.py: Collect distance-frequency measurements in one class. 
"""

import time

import pandas as pd
import scipy.interpolate


from calibration import fit_distance_slice
from dataset_parameters import kwargs_datasets
from frequency_analysis import *
from pandas_utils import fill_nans

ANGLE = 0
DISTANCE = 0
METHOD = np.nanmedian

YAW_DEG = 0  # drone angle used for when nothing else is given.

# parameters for cleaning signals
N_SPURIOUS = 2  # number of samples per frequency
MAG_THRESH = 1e-3  # minimum magnitude
STD_THRESH = 2.0  # maximum std deviation
DELTA_MERGE_FREQ = 100  # frequencies spaced by less than this are considered one
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


def find_indices(values_here, values):
    """ find the closest indices of an array (values_here) inside another array (values) """
    values_here = np.array(values_here)
    values = np.array(values)
    if (len(values_here) != len(values)) or not np.allclose(values, values_here):
        d_indices = np.where((values[:, None] == values_here[None, :]))[0]
        np.testing.assert_allclose(values[d_indices], values_here)
    else:
        d_indices = np.arange(len(values))
    return d_indices


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


class WallDetector(object):
    def __init__(self, exp_name=None, mic_type="audio_deck", interpolation=""):
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
        self.current_spectrogram = self.current_freqs = None
        self.interpolation = interpolation

        self.params = {}
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
        wall_detector.cleanup(verbose=verbose)
        return wall_detector

    def filter_by_freq(self, frequency):
        frequencies = self.df.frequency.unique()
        if frequency is None:
            if len(frequencies) > 1:
                print(
                    f"Warning: taking distance slice over multiple frequencies: {frequencies}"
                )
        elif frequency not in frequencies:
            print(f"Warning: did not find frequency {frequency}Hz")
            bin_ = np.argmin(np.abs(frequencies - frequency))
            frequency = frequencies[bin_]
            print(f"Closest match: {frequency}Hz")
            df = self.df.loc[self.df.frequency == frequency]
        else:
            df = self.df.loc[self.df.frequency == frequency]
        return df

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
        self,
        distance=None,
        min_freq=None,
        max_freq=None,
        method=METHOD,
        normalize_method="",
        mics=None
    ):
        """ 
        :return: slice along one distance, of shape (mic x frequencies), frequencies
        """
        df = sort_and_clip(self.df, min_freq, max_freq, name="frequency")
        if mics is not None:
            df = df.loc[df.mic.isin(mics)]

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

        # it can happen that not all mics measure
        # at the given frequency. 

        pt = df.pivot_table(
            index="mic", columns="frequency", values="magnitude", aggfunc=method
        )
        slice_f = pt.values
        freqs = pt.columns.values
        all_mics = np.sort(self.df.mic.unique())
        mics_here = pt.index.values
        indices = find_indices(mics_here, all_mics)
        if normalize_method != "":
            gains_f = normalize_method(freqs)
            gains_f = gains_f[indices, :]
            slice_f /= gains_f  # n_mics x n_freqs

            # TODO(FD): below is expensive and might not be necessary.
            # taking the std of slice_f instead of raw values gives similar results.
            stds = []
            for mic, df_mic in df.groupby("mic"):
                pt = df_mic.pivot_table(
                    index="frequency",
                    columns="counter",
                    values="magnitude",
                    aggfunc="median",
                )
                freqs_mic = pt.index.values
                raw_vals = pt.values  # n_freqs x n_counter
                raw_vals /= normalize_method(freqs_mic)[mic][:, None]
                stds.append(np.nanstd(raw_vals))
            # stds_median = np.nanstd(slice_f, axis=1)
            # print("raw vs. median std:", np.round(stds, 2), np.round(stds_median))
            return slice_f, freqs, stds
        else:
            stds = df.groupby("mic").magnitude.std().values
            return slice_f, freqs, stds

    def get_distance_slice(
        self, frequency, min_dist=None, max_dist=None, method=METHOD, mics=None,
    ):
        """ 
        :return: slice along one frequency, of shape (mic x distances), distances
        """
        df = sort_and_clip(self.df, min_dist, max_dist, name="distance")
        df = self.filter_by_freq(frequency)

        if mics is not None:
            df = df.loc[df.mic.isin(mics)]

        pt = df.pivot_table(
            index="mic", columns="distance", values="magnitude", aggfunc=method
        )
        distances = pt.columns.values
        mics = pt.index.values
        slice_d = pt.values

        stds = df.groupby("mic").magnitude.std()
        return slice_d, distances, mics, stds

    def get_df_matrix(self, max_freq=None, min_freq=None, min_dist=None, max_dist=None):
        df = clip_both(self.df, min_dist, max_dist, min_freq, max_freq)
        mics = np.sort(np.array(df.mic.unique(), dtype=np.int))
        n_mics = len(mics)
        frequencies = np.sort(np.array(df.frequency.unique(), dtype=np.float))
        distances = np.sort(np.array(df.distance.unique(), dtype=np.float))

        # Attention: below only works if distance "masks" are the same for each mic.
        # This is currently the case.
        # Otherwise, get_df_matrix_old can be used.
        df_matrix = np.full((n_mics, len(frequencies), len(distances)), np.nan)
        for i_f, frequency in enumerate(frequencies):
            distance_slice, distances_here, mics_here, std = self.get_distance_slice(
                frequency
            )
            d_indices = find_indices(distances_here, distances)
            mic_indices = find_indices(mics_here, mics)
            df_matrix[mic_indices[:, None], i_f, d_indices[None, :]] = distance_slice
        return df_matrix, distances, frequencies

    def get_df_matrix_old(
        self, max_freq=None, min_freq=None, min_dist=None, max_dist=None, method=METHOD
    ):
        df = clip_both(self.df, min_dist, max_dist, min_freq, max_freq)
        n_mics = len(df.mic.unique())

        frequencies = np.sort(np.array(df.frequency.unique(), dtype=np.float))
        distances = np.sort(np.array(df.distance.unique(), dtype=np.float))

        df_matrix = np.full((n_mics, len(frequencies), len(distances)), np.nan)
        for (d, f, m), df in self.df.groupby(["distance", "frequency", "mic"]):
            try:
                d_i = np.where(distances == d)[0][0]
                f_i = np.where(frequencies == f)[0][0]
            except:
                continue
            df_matrix[m, f_i, d_i] = method(df.magnitude.values)
        return df_matrix, distances, frequencies

    def remove_nan_rows(self):
        self.df.dropna(axis=0, subset=["magnitude"], inplace=True)

    def merge_close_freqs(
        self, delta_merge_freq=DELTA_MERGE_FREQ, verbose=False, dryrun=False
    ):
        """ Merge frequency bins that are closer than delta. """
        curr_f = curr_unique = self.df.frequency.min()
        merge_dict = {curr_unique: []}
        # for f in unique_frequencies[1:]:
        for f, df in self.df.groupby("frequency", sort=True):
            if f - curr_unique < delta_merge_freq:
                merge_dict[curr_unique] += list(df.index)
            else:
                merge_dict[f] = list(df.index)
                curr_uniqe = f
            curr_f = f

        if verbose:
            print(
                f"merging {len(self.df.frequency.unique())} frequencies to {len(merge_dict)}"
            )
        for start_f, index in merge_dict.items():
            average_f = np.round(self.df.loc[index].frequency.mean())
            self.df.loc[index].frequency = average_f

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
        """ Remove the frequencies with too low medians or too high standard deviation. """
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
        return len(remove_rows)

    def remove_bad_measurements(self, mag_thresh=MAG_THRESH):
        self.df = self.df[self.df.magnitude > mag_thresh]

    def remove_outliers(self, factor=3):
        median_magnitude = self.df.magnitude.median()
        window = self.df.magnitude.std() * factor
        mask = np.bitwise_and(
            self.df.magnitude > median_magnitude - window,
            self.df.magnitude < median_magnitude + window,
        )
        self.df = self.df[mask]

    def cleanup(self, mag_thresh=MAG_THRESH, verbose=False):
        self.remove_bad_measurements(mag_thresh)
        self.remove_bad_freqs(mag_thresh, verbose=verbose)
        self.remove_outliers()
        self.merge_close_freqs(verbose=verbose)
        self.remove_spurious_freqs(verbose=verbose)
        self.remove_nan_rows()
        self.df = self.df.apply(pd.to_numeric, axis=0, downcast="integer")

    def fill_from_backup(self, exp_name, mic_type="audio_deck", motors="0"):
        fname = f"results/backup_{exp_name}_{mic_type}_{motors}.pkl"
        try:
            self.df = pd.read_pickle(fname)
            print("read", fname)
            return True
        except FileNotFoundError:
            print(f"did not find {fname}")
            return False

    def backup(self, exp_name, mic_type="audio_deck", motors="0"):
        fname = f"results/backup_{exp_name}_{mic_type}_{motors}.pkl"
        pd.to_pickle(self.df, fname)
        print("saved", fname)

    def get_calib_function(self, method="median", ax=None):
        from scipy.interpolate import interp1d
        from calibration import plot_calibration

        """ 
        Return calibration function of form
        calib_function(freqs) => (n_mics x n_freqs) calibration gains. 

        """
        self.cleanup()

        mics = np.sort(self.df.mic.unique())
        freqs = np.sort(self.df.frequency.unique())
        gains = np.zeros((len(mics), len(freqs)))
        n_dist = len(self.df.distance.unique())

        for i_freq, freq in enumerate(freqs): 
            df = self.df.loc[self.df.frequency==freq]

            # do not have enough data to fit distance slice.
            if len(df.distance.unique())/n_dist <= RATIO_MISSING_ALLOWED:
                continue
            if method == "fit-one":
                coeffs_one, *_ = self.fit_to_median(freq, fit_one_gain=True)
                gains[:, i_freq] = coeffs_one[2:]
            else:
                for i_mic, mic in enumerate(mics):
                    df_here = df.loc[df.mic==mic]
                    if len(df_here.distance.unique())/n_dist <= RATIO_MISSING_ALLOWED:
                        continue
                    if method == "median":
                        gains[i_mic, i_freq] = df_here.magnitude.median()
                    elif method == "fit":
                        coeffs, *_ = self.fit_to_median(freq, mic_idx=mic)
                        if coeffs is not None:
                            gains[i_mic, i_freq] = coeffs[2]

        # TODO(FD) figure out more elegant way to do this. 
        missing = np.any(gains<1, axis=0)
        freqs = freqs[~missing]
        gains = gains[:, ~missing]

        calib_function = interp1d(
            freqs, gains, kind="cubic", fill_value="extrapolate", assume_sorted=True
        )
        if ax is not None:
            plot_calibration(freqs, gains, calib_function, ax=ax)
        return calib_function

    def fit_to_raw(self, frequency, mic_idx=None, fit_one_gain=True):
        from calibration import fit_distance_slice
        df_here = self.filter_by_freq(frequency)
        frequency_here = df_here.frequency.unique()[0]
        if mic_idx is not None:
            df_here = df_here.loc[df_here.mic == mic_idx]
            chosen_mics = [mic_idx]
        else:
            chosen_mics = np.sort(df_here.mic.unique())

        distances_raw = np.sort(df_here.distance.unique())
        mags_pt = pd.pivot_table(
            df_here, values="magnitude", index=["mic", "distance"], columns="counter", fill_value=0.0)
        counters = mags_pt.columns.values
        raw_values = np.empty((len(distances_raw), len(chosen_mics), len(counters)))
        for i, mic in enumerate(chosen_mics): 
            # get table corresponding to this mic
            table = mags_pt[(mags_pt.index.get_level_values("mic")==mic)]

            # will raw values with corresponding magnitudes
            distances_here = table.index.get_level_values("distance")
            indices = find_indices(distances_here, distances_raw)
            new_values = table.values
            raw_values[indices, i, :new_values.shape[-1]] = new_values 

        coeffs_raw, d_slice_raw, cost_raw = fit_distance_slice(
            raw_values,
            distances_raw,
            method="minimize",
            yaw_deg=YAW_DEG,
            frequency=frequency_here,
            chosen_mics=chosen_mics,
            optimize_absorption=True,
            fit_one_gain=fit_one_gain,
        )
        return coeffs_raw, distances_raw, d_slice_raw, cost_raw

    def fit_to_median(self, frequency, mic_idx=None, fit_one_gain=True):
        from calibration import fit_distance_slice
        df_here = self.filter_by_freq(frequency)
        frequency_here = df_here.frequency.unique()[0]
        if mic_idx is not None:
            df_here = df_here.loc[df_here.mic == mic_idx]
            chosen_mics = [mic_idx]
        else:
            chosen_mics = np.sort(df_here.mic.unique())

        distance_slices, distances_median, mics_here, *_ = self.get_distance_slice(frequency)
        if not len(set(chosen_mics).intersection(mics_here)):
            return None, None, None, None
        indices = find_indices(chosen_mics, mics_here)
        distance_slices = distance_slices[indices, :]
        
        coeffs_median, d_slice_median, cost_median = fit_distance_slice(
            distance_slices.T,
            distances_median,
            method="minimize",
            yaw_deg=YAW_DEG,
            frequency=frequency_here,
            chosen_mics=chosen_mics,
            optimize_absorption=True,
            fit_one_gain=fit_one_gain,
        )
        return coeffs_median, d_slice_median, distances_median, cost_median
