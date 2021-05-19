#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
data_collector.py: Collect distance-frequency measurements in one class.
"""

import scipy.interpolate

from dataset_parameters import kwargs_datasets
from frequency_analysis import *

ANGLE = 0
DISTANCE = 0
METHOD = np.nanmedian

YAW_DEG = 0  # drone angle used for when nothing else is given.

# parameters for cleaning signals
N_SPURIOUS = 2  # number of samples per frequency
MAG_THRESH = 1e-3  # minimum magnitude
STD_THRESH = 0.5  # maximum normalized std deviation
DELTA_MERGE_FREQ = 50  # frequencies spaced by less are merged. sweep spacing: ca. 125
RATIO_MISSING_ALLOWED = 0.2
OUTLIER_FACTOR = 3

SWEEP_DELTA = 1000  # jumps by more than this downwards are detected as end of sweep.
F_DELTA = 100  # jumps by more than this in frequency mark end of mono
D_DELTA_MAX = 3  # jumps by more than this in cm mark endf mono
D_DELTA_MIN = 1  # need at least movement by this to register meas.
MONO_FREQ = 3000  # mono frequency.


def get_peak_freq(signals_f, frequencies):
    avg_magnitude = np.nanmedian(np.abs(signals_f), axis=0)
    idx = np.argmax(avg_magnitude)
    magnitude = avg_magnitude[idx]
    f = frequencies[idx]
    return f, magnitude


def get_frequency_slice(df, mics=None):
    if mics is not None:
        df = df.loc[df.mic.isin(mics)]

    # it can happen that not all mics measure
    # at the given frequency.
    pt = df.pivot_table(
        index="mic", columns="frequency", values="magnitude", aggfunc=METHOD
    )
    pt_distance = df.pivot_table(
        index="mic", columns="frequency", values="distance", aggfunc=METHOD
    )
    distances = pt_distance.values[0, :]

    slice_f = pt.values
    freqs = pt.columns.values
    stds = df.groupby("mic").magnitude.std().values
    return slice_f, freqs, stds, distances


def get_distance_slice(df, mics=None):
    if mics is not None:
        df = df.loc[df.mic.isin(mics)]

    pt = df.pivot_table(
        index="mic", columns="distance", values="magnitude", aggfunc=METHOD
    )
    pt_frequency = df.pivot_table(
        index="mic", columns="distance", values="frequency", aggfunc=METHOD
    )
    freqs = pt_frequency.values[0, :]

    distances = pt.columns.values
    slice_d = pt.values
    stds = df.groupby("mic").magnitude.std().values
    return slice_d, distances, stds, freqs


def sorted_and_unique(df, name):
    return np.sort(df[name].dropna().unique())


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

        calib_function = get_calibration_function()
        calib_values = calib_function(list(freqs))[:, :, None]

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
    """ find the indices of an array (values_here) inside another array (values) """
    values_here = np.array(values_here)
    values = np.array(values)
    if (len(values_here) != len(values)) or not np.allclose(values, values_here):
        d_indices = np.where((values[:, None] == values_here[None, :]))[0]
        try:
            np.testing.assert_allclose(values[d_indices], values_here)
        except:
            print("error in find_indices.")
            return np.arange(len(values))
    else:
        d_indices = np.arange(len(values))
    return d_indices


def find_closest_indices(values_here, values):
    """ Find the closest indices of an array (values_here) inside another array (values) """
    values_here = np.array(values_here)
    values = np.array(values)
    if (len(values_here) != len(values)) or not np.allclose(values, values_here):
        d_indices = np.argmin(np.abs(values[:, None] - values_here[None, :]), axis=0)
    else:
        d_indices = np.arange(len(values))
    return d_indices


def prune_df_matrix(
    df_matrix, frequencies, ratio_missing_allowed=RATIO_MISSING_ALLOWED, verbose=False,
):
    """ Remove all rows with more than a certain percentage of missing values """
    df_matrix[np.isnan(df_matrix)] = 0

    # we also consider very small values as missing.
    eps = 1e-10
    count_missing = np.sum(df_matrix < eps, axis=2)
    freq_i = np.where(
        np.all(count_missing / df_matrix.shape[2] <= ratio_missing_allowed, axis=0)
    )[0]
    if verbose:
        print(f"keeping {len(freq_i)}/{len(frequencies)} frequencies")
    df_matrix[df_matrix < eps] = np.nan
    return df_matrix[:, freq_i, :], frequencies[freq_i], freq_i


def merge_close_freqs(df, delta_merge_freq=DELTA_MERGE_FREQ, verbose=False):
    """ Merge frequency bins that are closer than delta. """
    curr_unique = df.frequency.dropna().min()
    merge_dict = {curr_unique: []}
    for f, df_here in df.groupby("frequency", sort=True):
        # print(f'delta for {f}: {f - curr_unique}')
        if f - curr_unique < delta_merge_freq:
            merge_dict[curr_unique] += list(df_here.index)
        else:
            if verbose:
                print(
                    f"will be merged to {curr_unique}: {df.loc[merge_dict[curr_unique]].frequency.unique()}"
                )
            curr_unique = f
            merge_dict[curr_unique] = list(df_here.index)

    if verbose:
        print(f"merging {len(df.frequency.unique())} frequencies to {len(merge_dict)}")
    for start_f, index in merge_dict.items():
        average_f = np.round(df.loc[index].frequency.mean())
        df.loc[index, "frequency"] = average_f
    assert len(df.frequency.unique()) == len(merge_dict)


def remove_spurious_freqs(df, n_spurious, verbose=False, dryrun=False):
    remove_rows = []

    # average number of measurements per frequency and distance.
    # choose any column for 'magnitude', it doesn't matter.
    values = (
        df.loc[df.mic == 0]
        .groupby(["frequency", "distance"])
        .agg("count")["magnitude"]
        .values
    )
    if n_spurious is None:
        n_spurious = np.quantile(values, 0.7)

    if verbose:
        print(f"remove frequencies with less than {n_spurious} measurements.")
        print(f"min, max, median: {np.min(values), np.max(values), np.median(values)}")

    remove_names = []
    for (freq, mic, distance), df_here in df.groupby(["frequency", "mic", "distance"]):
        if len(df) < n_spurious:
            # if verbose:
            #   print('removing', df)
            remove_rows += list(df_here.index.values)
            remove_names += [(freq, mic, distance)]
    if verbose:
        print(
            f"remove_spurious_freqs: removing {len(remove_rows)} rows: {remove_names}"
        )
    if not dryrun:
        df.drop(index=remove_rows, inplace=True)
    if len(df) == 0:
        print("Warning: remove_spurious_freqs removed all rows.")


def remove_bad_freqs(df, mag_thresh, std_thresh, verbose=False, dryrun=False):
    """ Remove the frequencies with too low medians or too high standard deviation. """
    remove_rows = []
    for freq, df_here in df.groupby("frequency", sort=True):
        vals = df_here.magnitude.values
        if np.nanmedian(vals) < mag_thresh:
            if verbose:
                print(f"removing {freq} with median {np.nanmedian(vals)}")
            remove_rows += list(df_here.index.values)
        elif normalized_std(vals) > std_thresh:
            if verbose:
                print(f"removing {freq} with std {normalized_std(vals)}")
            remove_rows += list(df_here.index.values)
        elif verbose:
            print(
                f"keeping {freq}: {np.nanmedian(vals):.2e}, {normalized_std(vals):.2e}"
            )
    if verbose:
        print(f"remove_bad_freqs: removing {len(remove_rows)} rows.")
    if not dryrun:
        df.drop(index=remove_rows, inplace=True)
    if len(df) == 0:
        print(
            f"Warning: remove_bad_freqs with {mag_thresh, std_thresh} removed all rows."
        )
    return len(remove_rows)


def remove_bad_measurements(df, mag_thresh, verbose=False):
    index_remove = df[df.magnitude < mag_thresh].index
    if verbose:
        print(f"removing {len(index_remove)} rows")
    df.drop(index=index_remove, inplace=True)


def remove_outliers(df, factor=OUTLIER_FACTOR, normalize=False, verbose=False):
    index_remove = []
    for (mic, d, f), df_dist in df.groupby(["mic", "distance", "frequency"]):
        median_magnitude = df_dist.magnitude.median()
        if normalize:
            std = normalized_std(df_dist.magnitude.values)
        else:
            std = df_dist.magnitude.std()
        window = std * factor
        index_here = df_dist[
            (df_dist.magnitude < median_magnitude - window)
            | (df_dist.magnitude > median_magnitude + window)
        ].index
        index_remove += list(index_here)
        if verbose and len(index_here) > 0:
            print(
                f"{mic, d, f}: removing {len(index_here)} measurements outside of {median_magnitude - window:.1f}, {median_magnitude + window:.1f}"
            )
    df.drop(index=index_remove, inplace=True)


def remove_nan_rows(df, verbose=False):
    if verbose:
        print(f"dropping {len(df.loc[df.magnitude.isnull()])} rows")
    len_before = len(df)
    df.dropna(axis=0, subset=["magnitude"], inplace=True)
    len_after = len(df)
    if verbose:
        print("dropped", len_before - len_after)


def to_numeric(df):
    return df.apply(pd.to_numeric, axis=0, downcast="integer")


def cleanup(df, params, verbose=False):
    """ cleanup df inplace. """

    mag_thresh = params.get("mag_thresh", MAG_THRESH)
    std_thresh = params.get("std_thresh", STD_THRESH)
    n_spurious = params.get("n_spurious", N_SPURIOUS)

    df = to_numeric(df)

    # operate in-place:
    remove_nan_rows(df, verbose)
    remove_bad_measurements(df, mag_thresh, verbose)
    remove_bad_freqs(df, mag_thresh, std_thresh, verbose=verbose)
    merge_close_freqs(df, verbose=verbose)
    remove_outliers(df, verbose)
    remove_spurious_freqs(df, n_spurious, verbose=verbose)
    remove_nan_rows(df, verbose)

    # doesn't operate in-place:
    return to_numeric(df)


def normalized_std(values, method=METHOD):
    method_values = method(values)
    if method_values != 0:
        return np.nanstd(values) / method_values
    else:
        return 0


class DataCollector(object):
    def __init__(self, exp_name=None, mic_type="audio_deck", interpolation=""):
        self.latest_fslice_time = None
        self.latest_dslice_time = None
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

    @staticmethod
    def init_from_row(exp_name, row, interpolation="", verbose=False):
        data_collector = DataCollector(
            exp_name=exp_name, mic_type=row.mic_type, interpolation=interpolation,
        )
        try:
            data_collector.fill_from_row(row, verbose=verbose)
        except:
            print("skipping", row)
            raise
            return None
        data_collector.cleanup(verbose=verbose)
        return data_collector

    def filter_by_column(self, column_value, column_name="frequency"):
        values = self.df[column_name].unique()

        if column_value is None:
            if len(values) > 1:
                print(f"Warning: taking slice over multiple values: {values}")
            return self.df
        elif column_value not in values:
            print(f"Warning: did not find {column_name} {column_value}")
            bin_ = np.argmin(np.abs(values - column_value))
            column_value = values[bin_]
            print(f"Closest match: {column_value}")

        df = self.df.loc[self.df[column_name] == column_value]
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

    def get_mics(self):
        return sorted_and_unique(self.df, "mic")

    def get_distances(self):
        return sorted_and_unique(self.df, "distance")

    def get_frequencies(self):
        return sorted_and_unique(self.df, "frequency")

    def next_fslice_ready(self, signals_f, frequencies):
        """ find big frequency jump, meaning end of sweep. """
        fslice_ready = False

        f, magnitude = get_peak_freq(signals_f, frequencies)

        if len(self.df) == 0:
            return False

        latest_frequency = self.df.loc[len(self.df) - 1, "frequency"]

        if (magnitude > self.params.get("mag_thresh", MAG_THRESH)) and (
            f < latest_frequency - SWEEP_DELTA
        ):
            return True
        return False

    def valid_dslice_measurement(self, position_cm, signals_f, frequencies):
        if position_cm[2] < 35:
            return False

        if MONO_FREQ is not None:
            f, magnitude = get_peak_freq(signals_f, frequencies)
            if abs(MONO_FREQ - f) > F_DELTA:
                return False

        if len(self.df) == 0:
            return True

        # detect too small distance change
        new_d = position_cm[1]
        latest_d = self.df.iloc[-1].distance
        if new_d - latest_d < D_DELTA_MIN:
            return False

        return True

    def next_dslice_ready(self, signals_f, frequencies, position_cm, n_max=100):
        """ find if we have a new dslice ready, if either:
            - we have more than n_slice values, or
            - we have a jump in relative_distance measurements.
            - if we changed frequency significantly since last time.
        """
        dslice_ready = False

        if self.latest_dslice_time is not None:
            df_current = self.df[self.df.time > self.latest_dslice_time]
        else:
            df_current = self.df

        if len(df_current) == 0:
            return False

        # detect more than n_max values
        if len(df_current) >= n_max:
            print(f"more than {n_max} values since last")
            return True

        # detect distance change
        df_mic = df_current.loc[df_current.mic == 0]
        average_delta_d = df_mic.distance.diff().mean()
        new_d = position_cm[1]
        latest_d = df_current.iloc[-1].distance
        predicted_d = latest_d + average_delta_d
        if np.abs(new_d - predicted_d) > D_DELTA_MAX:
            print(
                f"new position different from predicted: {latest_d} + {average_delta_d} + {D_DELTA_MAX}, {new_d}"
            )
            return True

        # detect frequency change
        f, magnitude = get_peak_freq(signals_f, frequencies)
        latest_frequency = df_current.iloc[-1].frequency
        if (magnitude > self.params.get("mag_thresh", MAG_THRESH)) and (
            np.abs(f - latest_frequency) > F_DELTA
        ):

            print(f"big frequency change: {f}, {latest_frequency}")
            return False  # True
        return False

    def fill_from_signal(self, signals_f, frequencies, distance_cm=0, angle=0, time=0):
        """
        :param signals_f: shape (n_mics, n_freqs), complex
        :param frequencies: shape (n_freqs,)

        """
        sweep_complete = False
        for i_mic in range(signals_f.shape[0]):
            idx = np.argmax(np.abs(signals_f[i_mic, :]))
            f = frequencies[idx]
            counter = len(
                self.df.loc[
                    (self.df.mic == i_mic)
                    & (self.df.distance == distance_cm)
                    & (self.df.angle == angle)
                    & (self.df.frequency == f)
                ]
            )
            magnitude = np.abs(signals_f[i_mic, idx])

            self.df.loc[len(self.df), :] = {
                "time": time,
                "counter": counter,
                "mic": i_mic,
                "frequency": f,
                "distance": distance_cm,
                "angle": angle,
                "magnitude": magnitude,
            }

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

        self.fill_from_spec(spec, freqs, distance, angle, verbose, times=times)
        self.current_spectrogram = spec
        self.current_freqs = freqs
        return spec, freqs

    def fill_from_spec(
        self, spec, freqs, distance=DISTANCE, angle=ANGLE, verbose=False, times=None
    ):
        df = psd_df_from_spec(
            spec, freqs, interpolation=self.interpolation, verbose=verbose, times=times,
        )
        assert np.all(df.magnitude.values[~np.isnan(df.magnitude.values)] >= 0)
        df.loc[:, "distance"] = distance
        df.loc[:, "angle"] = angle
        # need ignore_index here to make sure that the final index is unique.
        self.df = pd.concat([self.df, df], ignore_index=True)

    def get_frequency_slice(self, distance=None, mics=None):
        """ 
        :return: slice along one distance, of shape (mic x frequencies), frequencies, stds
        """
        df = self.filter_by_column(distance, "distance")
        return get_frequency_slice(df, mics)[:2]

    def get_frequency_slice_fixed(
        self,
        frequencies,
        distance=None,
        normalize_method="",
        mics=None,
        verbose=False,
        allowed_delta=50,
    ):
        """ Give frequency slice for fixed frequencies.

        :param frequencies: list of frequencies we want to evaluate at. 
        :param allowed_delta: if absolute difference between measured frequency and given frequency
        is smaller than this, we still use it in the frequency slice. 

        :return: slice along one distance, of shape (mic x frequencies), used frequencies, stds.
        """
        df = self.filter_by_column(distance, "distance")

        # return get_frequency_slice(df, mics, frequencies=frequencies, normalize_method)

        if mics is not None:
            df = df.loc[df.mic.isin(mics)]

        # it can happen that not all mics measure
        # at the given frequency.
        pt = df.pivot_table(
            index="mic", columns="frequency", values="magnitude", aggfunc=METHOD
        )
        slice_f = pt.values
        freqs_here = pt.columns.values
        f_indices = find_closest_indices(frequencies, freqs_here)

        if verbose:
            diff = freqs_here[f_indices] - frequencies
            print(
                "evaluating at", freqs_here[f_indices], "instead of", frequencies,
            )
            print("difference:", diff)
            f_indices = f_indices[np.abs(diff) < allowed_delta]
        slice_f = slice_f[:, f_indices]
        freqs = freqs_here[f_indices]

        if normalize_method != "":
            all_mics = self.get_mics()
            mics_here = pt.index.values
            m_indices = find_indices(all_mics, mics_here)

            gains_f = normalize_method(freqs)

            valid = np.all(gains_f > 0, axis=0)
            gains_f = gains_f[m_indices, valid]
            slice_f[:, valid] /= gains_f  # n_mics x n_freqs

            stds = np.nanstd(slice_f, axis=1)
            return slice_f, freqs, stds
        else:
            stds = df.groupby("mic").magnitude.std().values
            return slice_f, freqs, stds

    def get_current_frequency_slice(self, verbose=False):
        if self.latest_fslice_time is None:
            self.latest_fslice_time = self.df.iloc[0].time
            if verbose:
                print("set latest time to", self.latest_fslice_time)

        latest_df = self.df[self.df.time > self.latest_fslice_time]
        latest_df_clean = cleanup(latest_df, self.params, verbose=verbose)

        f_slice, freqs, stds, d_slice = get_frequency_slice(latest_df_clean)

        self.latest_fslice_time = latest_df.iloc[-1].time
        return f_slice, freqs, stds, d_slice

    def get_distance_slice(self, frequency=None, mics=None):
        """ 
        :return: slice along one frequency, of shape (mic x distances), distances
        """
        df = self.filter_by_column(frequency, "frequency")
        return get_distance_slice(df, mics=mics)

    def get_current_distance_slice(self, verbose=False):
        if self.latest_dslice_time is None:
            self.latest_dslice_time = self.df.iloc[0].time
            if verbose:
                print("set latest time to", self.latest_dslice_time)

        latest_df = self.df[self.df.time > self.latest_dslice_time]
        latest_df_clean = cleanup(latest_df, self.params, verbose=verbose)

        d_slice, distances, stds, freqs = get_distance_slice(latest_df_clean)

        self.latest_dslice_time = latest_df.iloc[-1].time
        return d_slice, distances, stds, freqs

    def get_df_matrix(self):
        mics = sorted_and_unique(self.df, "mic").astype(np.int)
        frequencies = sorted_and_unique(self.df, "frequency").astype(np.float)
        distances = sorted_and_unique(self.df, "distance").astype(np.float)
        n_mics = len(mics)

        df_matrix = np.full((n_mics, len(frequencies), len(distances)), np.nan)
        for i_f, frequency in enumerate(frequencies):
            df = self.filter_by_column(frequency, "frequency")
            distance_slice, distances_here, freqs, stds = get_distance_slice(df)
            mics_here = df.mic.unique()
            d_indices = find_indices(distances_here, distances)
            mic_indices = find_indices(mics, mics_here)
            df_matrix[mic_indices[:, None], i_f, d_indices[None, :]] = distance_slice
        return df_matrix, distances, frequencies

    def remove_nan_rows(self, verbose=False):
        remove_nan_rows(self.df, verbose)

    def merge_close_freqs(self, delta_merge_freq=DELTA_MERGE_FREQ, verbose=False):
        merge_close_freqs(self.df, delta_merge_freq, verbose=verbose)

    def remove_spurious_freqs(self, verbose=False, dryrun=False):
        """ Remove the frequencies for which we only have less than n_min measurements. """
        n_spurious = self.params.get("n_spurious", N_SPURIOUS)
        remove_spurious_freqs(self.df, n_spurious, verbose=verbose, dryrun=dryrun)

    def to_numeric(self):
        self.df = to_numeric(self.df)

    def remove_bad_freqs(self, verbose=False, dryrun=False):
        mag_thresh = self.params.get("mag_thresh", MAG_THRESH)
        std_thresh = self.params.get("std_thresh", STD_THRESH)
        return remove_bad_freqs(self.df, mag_thresh, std_thresh, verbose, dryrun)

    def remove_bad_measurements(self, verbose=False):
        mag_thresh = self.params.get("mag_thresh", MAG_THRESH)
        remove_bad_measurements(self.df, mag_thresh, verbose)

    def remove_outliers(self, factor=OUTLIER_FACTOR, normalize=True, verbose=False):
        return remove_outliers(self.df, factor, normalize, verbose)

    def cleanup(self, verbose=False):
        self.remove_nan_rows()
        self.remove_bad_measurements()
        self.remove_bad_freqs(verbose=verbose)
        self.merge_close_freqs(verbose=verbose)
        self.remove_outliers()
        self.remove_spurious_freqs(verbose=verbose)
        self.remove_nan_rows()
        self.to_numeric()

    def fill_from_backup(
        self, exp_name, mic_type="audio_deck", motors="0", appendix=""
    ):
        fname = f"results/backup_{exp_name}_{mic_type}_{motors}{appendix}.pkl"
        try:
            self.df = pd.read_pickle(fname)
            print("read", fname)
            return True
        except FileNotFoundError:
            print(f"did not find {fname}")
            return False

    def backup(self, exp_name, mic_type="audio_deck", motors="0", appendix=""):
        fname = f"results/backup_{exp_name}_{mic_type}_{motors}{appendix}.pkl"
        pd.to_pickle(self.df, fname)
        print("saved", fname)

    # TODO(FD): below are potentially deprecated.
    def get_calib_function(self, method="median", ax=None):
        from scipy.interpolate import interp1d
        from calibration import plot_calibration

        """ 
        Return calibration function of form
        calib_function(freqs) => (n_mics x n_freqs) calibration gains. 

        """
        mics = self.get_mics()
        freqs = self.get_frequencies()
        distances = self.get_distances()

        n_dist = len(distances)
        gains = np.zeros((len(mics), len(freqs)))

        for i_freq, (freq, df) in enumerate(self.df.groupby("frequency")):
            # don't have enough data to fit distance slice.
            if len(sorted_and_unique(df, "distance")) / n_dist <= RATIO_MISSING_ALLOWED:
                continue
            if method == "fit-one":
                coeffs_one, *_ = self.fit_to_median(freq, fit_one_gain=True)
                gains[:, i_freq] = coeffs_one[2:]
            else:
                for i_mic, (mic, df_here) in enumerate(df.groupby("mic")):
                    if (
                        len(sorted_and_unique(df_here, "distance")) / n_dist
                        <= RATIO_MISSING_ALLOWED
                    ):
                        continue
                    if method == "median":
                        gains[i_mic, i_freq] = df_here.magnitude.median()

                    elif method == "fit":
                        coeffs, *_ = self.fit_to_median(freq, mic_idx=mic)
                        if coeffs is not None:
                            gains[i_mic, i_freq] = coeffs[2]

        # TODO(FD) figure out more elegant way to do this.
        not_all_missing = np.any(gains > 0, axis=0)
        freqs = freqs[not_all_missing]
        gains = gains[:, not_all_missing]

        calib_function = interp1d(
            freqs, gains, kind="linear", fill_value="extrapolate", assume_sorted=True,
        )
        if ax is not None:
            plot_calibration(freqs, gains, calib_function, ax=ax)
        return calib_function

    def fit_to_raw(self, frequency, mic_idx=None, fit_one_gain=True):
        from calibration import fit_distance_slice

        df_here = self.filter_by_column(frequency, "frequency")
        frequency_here = df_here.frequency.unique()[0]
        if mic_idx is not None:
            df_here = df_here.loc[df_here.mic == mic_idx]
            chosen_mics = [mic_idx]
        else:
            chosen_mics = sorted_and_unique(df_here, "mic")

        distances_raw = sorted_and_unique(df_here, "distance")
        mags_pt = pd.pivot_table(
            df_here,
            values="magnitude",
            index=["mic", "distance"],
            columns="counter",
            fill_value=0.0,
        )
        counters = mags_pt.columns.values
        raw_values = np.empty((len(distances_raw), len(chosen_mics), len(counters)))
        for i, mic in enumerate(chosen_mics):
            # get table corresponding to this mic
            table = mags_pt[(mags_pt.index.get_level_values("mic") == mic)]

            # will raw values with corresponding magnitudes
            distances_here = table.index.get_level_values("distance")
            indices = find_indices(distances_here, distances_raw)
            new_values = table.values
            raw_values[indices, i, : new_values.shape[-1]] = new_values

        coeffs_raw, d_slice_raw, cost_raw = fit_distance_slice(
            raw_values,
            distances_raw,
            method="minimize",
            azimuth_deg=YAW_DEG,
            frequency=frequency_here,
            chosen_mics=chosen_mics,
            optimize_absorption=True,
            fit_one_gain=fit_one_gain,
        )
        return coeffs_raw, distances_raw, d_slice_raw, cost_raw

    def fit_to_median(self, frequency, mic_idx=None, fit_one_gain=True):
        from calibration import fit_distance_slice

        df_here = self.filter_by_column(frequency, "frequency")
        frequency_here = df_here.frequency.unique()[0]

        (distance_slices, distances_median, mics_here, *_,) = self.get_distance_slice(
            frequency, mics=[mic_idx]
        )
        coeffs_median, d_slice_median, cost_median = fit_distance_slice(
            distance_slices.T,
            distances_median,
            method="minimize",
            azimuth_deg=YAW_DEG,
            frequency=frequency_here,
            chosen_mics=mics_here,
            optimize_absorption=True,
            fit_one_gain=fit_one_gain,
        )
        return coeffs_median, d_slice_median, distances_median, cost_median

    def get_df_matrix_old(self):
        df = self.df
        n_mics = len(df.mic.unique())

        frequencies = sorted_and_unique(df, "frequency")
        distances = sorted_and_unique(df, "distance")

        df_matrix = np.full((n_mics, len(frequencies), len(distances)), np.nan)
        for (d, f, m), df in self.df.groupby(["distance", "frequency", "mic"]):
            try:
                d_i = np.where(distances == d)[0][0]
                f_i = np.where(frequencies == f)[0][0]
            except IndexError:
                continue
            df_matrix[m, f_i, d_i] = METHOD(df.magnitude.values)
        return df_matrix, distances, frequencies
