import itertools
import sys

import numpy as np
import pandas as pd
import progressbar

from crazyflie_description_py.parameters import N_BUFFER, FS 
from evaluate_data import read_df, read_df_from_wav, get_fname
from evaluate_data import get_positions
from dynamic_analysis import add_pose_to_df

def clean_signals_f(signals_f, max_value=N_BUFFER):
    """
    The values in signals_f are normally between -N_BUFFER and N_BUFFER, 
    so values outside of this range are due to communication errors.
    """
    signals_f[np.isnan(signals_f)] = 0.0
    signals_f[np.abs(signals_f) > max_value] = 0.0
    return signals_f


def filter_by_dicts(df, dicts):
    mask = np.zeros(len(df), dtype=bool)
    for dict_ in dicts:
        this_mask = np.ones(len(df), dtype=bool)
        for key, val in dict_.items():
            this_mask = this_mask & (df.loc[:, key] == val)
        mask = np.bitwise_or(mask, this_mask)
    if not np.any(mask):
        return []
    else:
        return df.loc[mask, :]


def extract_linear_psd(signals_f, frequencies, slope, offset, delta=50, ax=None, times=None):
    eps = 1e-2
    times_window = (frequencies - offset) / slope
    # freqs_window = offset + slope * times
    
    if ax is not None:
        ax.plot(times_window-delta, frequencies, color='red')
        ax.plot(times_window+delta, frequencies, color='red')

    psd = np.zeros(signals_f.shape[1:]) # 4 x 32
    if times is None:
        times = np.arange(signals_f.shape[0])
    for i, t in enumerate(times_window):

        # reduce the signals to a valid window given by offset and slope
        valid_times = (times <= t + delta) & (times >= t - delta)
        signals_window = np.abs(signals_f[valid_times, : , i]) # n_times x n_mics

        # within the window, choose only nonzero indices for average
        signals_nonzero = signals_window[np.mean(signals_window, axis=1) > eps, :]
        #print('reduced from:', signals_window.shape[0], signals_nonzero.shape[0])
        psd[:, i] = np.mean(signals_nonzero**2, axis=0)
    return psd


def extract_psd_dict(signals_f, frequencies_matrix, min_t=0, max_t=None, n_freq=1, ax=None):
    """
    Extract a hash table from the signals and frequencies information, one for 
    each microphone.
    
    structure of output: 
    
    [{ #mic0
        f0: [val0, val1],
        f1: [val0, val1, val2], 
        f2: [...]
     },
     { #mic1
        f0: 
        ...
     }
     ...
    ]
    """
    n_mics = signals_f.shape[1]
    all_frequencies = np.unique(frequencies_matrix.flatten())

    psd_dict = [{f:[] for f in all_frequencies} for i in range(n_mics)]

    if max_t is None:
        max_t = frequencies_matrix.shape[0]
        
    if ax is not None:
        ax.plot(frequencies_matrix[:, 0])
        ax.axvline(min_t, color='k')
        ax.axvline(max_t, color='k')

    for i_t in range(min_t, max_t): # n_times x n_freqs 
        # save strongest n_freq frequencies
        fs = frequencies_matrix[i_t, :n_freq]
        for i_mic in range(n_mics):
            for f_idx, f in enumerate(fs):
                psd_dict[i_mic][f].append(np.abs(signals_f[i_t, i_mic, f_idx])**2)
    return psd_dict
                

def extract_psd(psd_dict_list, verbose=False, method='median'):
    """
    Combine hash tables in given list into one big hashtable, and return the unique keys
    and statistics of its values.
    """
    # extract all the different frequencies from psd_dict_list.
    n_mics = len(psd_dict_list[0])
    frequencies = set().union(*(psd_dict[i].keys() for psd_dict in psd_dict_list for i in range(n_mics)))
    frequencies = np.sort(list(frequencies))
    
    psd = np.zeros((n_mics, len(frequencies)))
    psd_std = np.zeros((n_mics, len(frequencies)))
    for i_mic in range(n_mics):
        for j, f in enumerate(frequencies):
            
            # combine all values at this f and mic
            vals = []
            for psd_dict in psd_dict_list:
                if f in psd_dict[i_mic].keys():
                    vals += psd_dict[i_mic][f]
                    
            if verbose:
                print(f'for frequency {f}, mic{i_mic}, found {vals}')
            if len(vals):
                
                if ('reject' in method) and (len(vals) > 4):
                    vals = vals[2:-2]
                if 'median' in method: 
                    psd[i_mic, j] = np.median(vals) 
                if 'mean' in method: 
                    psd[i_mic, j] = np.mean(vals) 
                    
                psd_std[i_mic, j] = np.std(vals) 
                
    # remove the frequencies for which we have no data
    mask = np.any(psd > 0, axis=0)
    return psd[:, mask], frequencies[mask], psd_std[:, mask]


def get_psd(signals_f, frequencies, ax=None, fname='real'):
    """ 
    :param signals_f: tensor of signals of shape n_times x n_mics x n_frequencies
    :param frequencies: frequencies vector in Hz.
    """

    # read off from plot: 
    if 'simulated' in fname:
        slope = (4000 - 1000) / (200 - 50)
        offset = -500
    elif 'real' in fname:
        # old dataset (2020_11_23_wall2)
        #slope = (4000 - 1000) / (285 - 90)
        #offset = -500
        slope = (4000 - 1000) / (250 - 50)
        offset = 200
    else:
        raise ValueError(exp_name)

    return extract_linear_psd(signals_f, frequencies, slope, offset, delta=50, ax=ax)


def add_distance_estimates(row, ax=None, min_z=300):
    if row.distance == 51:
        d = np.full(len(row.seconds), np.nan)
        d[row.seconds>=0] = 0.1 + row.seconds[row.seconds>=0] * 0.5 / 165  # takes 165 seconds for 50 cm
    elif row.distance == -51:
        d = np.full(len(row.seconds), np.nan)
        d[row.seconds>=0] = 0.6 - row.seconds[row.seconds>=0] * 0.5 / 165  # takes 165 seconds for 50 cm
    else:
        #TODO(FD): to be improved once we have better flow estimates. 
        # read from plot in FlyingAnalysis.ipynb:
        min_time = 5 
        max_time = 25 
        
        d = np.full(len(row.seconds), np.nan)
        row.z[np.isnan(row.z)] = 0
        start_idx = np.where(row.seconds > min_time)[0][0]
        end_idx = np.where(row.seconds < max_time)[0][-1]

        assert end_idx > start_idx, f"{start_idx}<={end_idx}"
        valid_z = row.z[start_idx:end_idx]
        max_idx = start_idx + np.argmax(valid_z)
        try:
            min_idx = start_idx + np.where(valid_z > min_z)[0][-1]
        except:
            min_idx = 0
        duration = row.seconds[min_idx] - row.seconds[max_idx]

        times = row.seconds[max_idx:min_idx] - row.seconds[max_idx]
        d[max_idx:min_idx] = D_START - times * VELOCITY
        
        if ax is not None:
            ax.axvline(row.seconds[start_idx], color='r')
            ax.axvline(row.seconds[max_idx], color='k', label='start (max)')
            ax.axvline(row.seconds[min_idx], color='k', label=f'end (above {min_z})')
            ax.axvline(row.seconds[end_idx], color='b')
            ax.set_xlim(0, 35)
            ax.set_title(f"duration: {duration:.1f}s")
        
    row['d_estimate'] = d

    if ax is not None:
        ax.scatter(row.seconds, d * 1000, color='k', label='distance')
        ax.legend()
    return row


def add_spectrogram(row):
    """  Add snr-based or normal spectrogram to row.

    Usage: 
    df = df.apply(add_spectrogram, axis=1)

    new spectrogram column is of shape: n_times x n_mics x n_frequencies
    """

    # TODO(FD) remove this sanity check 
    assert row.signals_f.shape[1] in (1, 4)
    assert row.signals_f.shape[2] in (32, 1025)

    if row.frequencies_matrix is None:
        row.spectrogram = np.abs(row.signals_f)
    else:
        all_frequencies = np.fft.rfftfreq(N_BUFFER, 1/FS) 
        
        # spectrogram is of shape 1025 x n_times x n_mics
        spectrogram = np.zeros((row.signals_f.shape[0], row.signals_f.shape[1], len(all_frequencies)), dtype=float) 
        for t_idx in range(row.signals_f.shape[0]):
            freqs = row.frequencies_matrix[t_idx, :]
            for i, f in enumerate(freqs):
                f_idx =  np.argmin(np.abs(f - all_frequencies))
                if abs(all_frequencies[f_idx] - f) > 1: 
                    print(f'Warning: frequency {f} is far from {all_frequencies}')

                if 0: #np.any(spectrogram[f_idx, t_idx, :] > 0):
                    print('overwriting', t_idx, f_idx, f, freqs[0])
                    err = np.max(np.abs(np.abs(row.signals_f[t_idx, :, i]) - spectrogram[t_idx, :, f_idx]))
                    # TODO(FD) find out why there is a difference between the
                    # first freq. bin (forced) and the one selected by snr scheme. 
                    if err > 0.1:
                        print('big error:', err)
                spectrogram[t_idx, :, f_idx] = np.abs(row.signals_f[t_idx, :, i])
        row.spectrogram = spectrogram
    return row


def parse_experiments(exp_name='2020_12_9_moving', wav=True):
    if exp_name == '2020_12_7_moving':
        appendix_list = ["", "_new"]; snr_list = [0, 1]; props_list=[0]
    elif exp_name == '2020_12_9_rotating':
        appendix_list = ["", "_new"]; snr_list = [0, 1]; props_list = [0, 1]
    elif exp_name == '2020_12_18_flying':
        appendix_list = ["", "_new"]; snr_list = [2]; props_list = [0, 1]; wav = False
    elif exp_name == '2020_12_18_stepper':
        appendix_list = ["", "_new"]; snr_list = [2]; props_list = [0, 1]

    sys.path = [p for p in sys.path if not 'experiments' in p]
    sys.path.insert(0, f'../experiments/{exp_name}')
    from params import SOURCE_LIST, DISTANCE_LIST, DEGREE_LIST, MOTORS_LIST
    from crazyflie_description_py.parameters import N_BUFFER

    # TODO(FD) remove this when we use more angles again.
    DEGREE_LIST = [0]

    pos_columns = ['dx', 'dy', 'z', 'yaw_deg']

    df_total = pd.DataFrame(columns=[
        'appendix', 'degree', 'distance', 'motors', 'mic_type', 'source', 'snr', 'props', # categories
        'seconds', 'frequencies', 'frequencies_matrix', 'signals_f', 'positions'] + pos_columns # data
    )

    params = {
        'props': False,
        'exp_name': exp_name
    }
    for degree, distance, source, motors, appendix, snr, props in itertools.product(
        DEGREE_LIST, DISTANCE_LIST, SOURCE_LIST, MOTORS_LIST, appendix_list, snr_list, props_list): 

        mic_dfs = {'audio_deck': None, 'measurement': None}
        try:
            params['appendix'] = appendix
            params['degree'] = degree
            params['distance'] = distance
            params['motors'] = motors
            params['props'] = props
            params['source'] = source
            params['snr'] = snr

            df_csv, df_pos = read_df(**params)

            positions = get_positions(df_pos)
            add_pose_to_df(df_csv, df_pos)
            mic_dfs['audio_deck'] = df_csv
            
            fname = get_fname(**params)

            if wav: 

                try:
                    wav_fname = f'../experiments/{exp_name}/export/{fname}.wav'
                    df_wav = read_df_from_wav(wav_fname, n_buffer=N_BUFFER)
                    mic_dfs['measurement'] = df_wav
                except:
                    print('could not read', wav_fname)
            
        except FileNotFoundError as e:
            continue 
            
        for mic_type, df in mic_dfs.items():
            if df is None:
                continue
            if not 'signals_f' in df.columns:
                continue
            
            signals_f = np.array([*df.signals_f.values]) # n_times x n_mics x n_freqs
            signals_f = clean_signals_f(signals_f)

            seconds = (df.timestamp.values - df.iloc[0].timestamp) / 1000
            frequencies_matrix = np.array([*df.loc[:,'frequencies']])
            frequencies = frequencies_matrix[0, :]

            # save frequency matrix only if frequencies vary.
            if not np.any(np.any(frequencies_matrix - frequencies[None, :], axis=0)):
                frequencies_matrix = None
            
            params['source'] = str(params['source'])
            all_items = dict(
                mic_type=mic_type,
                seconds=seconds, 
                frequencies=frequencies,
                frequencies_matrix=frequencies_matrix,
                signals_f=signals_f,
                positions=positions)
            if 'dx' in df.columns: 
                pos_dict = {
                    col:df[col].values for col in pos_columns
                }
            else:
                pos_dict = {
                    col:None for col in pos_columns
                }
            all_items.update(pos_dict)
            all_items.update(params)
            df_total.loc[len(df_total), :] = all_items
    return df_total


def parse_calibration_experiments():
    df_total = pd.DataFrame(columns=['source', 'snr', 'motors', 'exp_name', 'appendix', 
                                     'seconds', 'frequencies', 'frequencies_matrix', 'signals_f'])

    exp_name_list = [
        '2020_12_11_calibration',
    ]
    source_list = ['sweep', 'sweep_buzzer']
    appendix_list = ['', '_BC329', '_HALL', '_HALL2', '_HALL3']
    filter_list = [0, 1] # snr and props
    motors_list = [0, 'all43000']

    for exp_name, source, motors, appendix, filter_ in itertools.product(exp_name_list, source_list, motors_list, appendix_list, filter_list):
        params = dict(
            exp_name=exp_name,
            motors=motors,
            degree=0 ,
            distance=0 ,
            source=source,
            appendix=appendix,
            snr=filter_,
            props=filter_,
        )
        try:
            df, __ = read_df(**params)
        except FileNotFoundError as e:
            print('could not read', params)
            continue 
            
        signals_f = np.array([*df.signals_f.values]) # n_times x n_mics x n_freqs
        signals_f = clean_signals_f(signals_f)

        seconds = (df.timestamp.values - df.iloc[0].timestamp) / 1000
        frequencies_matrix = np.array([*df.loc[:,'frequencies']])
        frequencies = frequencies_matrix[0, :]

        # save frequency matrix only if frequencies vary.
        if not np.any(np.any(frequencies_matrix - frequencies[None, :], axis=0)):
            frequencies_matrix = None
        
        df_total.loc[len(df_total), :] = dict(
            source=str(source), 
            snr=filter_, 
            motors=motors, 
            exp_name=exp_name, 
            appendix=appendix, 
            seconds=seconds, 
            frequencies=frequencies, 
            frequencies_matrix=frequencies_matrix,
            signals_f=signals_f, 
        )
    return df_total


if __name__ == "__main__":
    import os 

    #exp_name = '2020_12_9_rotating'
    #exp_name = '2020_12_18_flying'
    exp_name = '2020_12_18_stepper'
    fname = f'results/{exp_name}_real.pkl'

    dirname = os.path.dirname(fname)
    if not os.path.isdir(dirname):
        os.makedirs(dirname)
        print('created directory', dirname)

    try:
        raise 
        df_total = pd.read_pickle(fname)
        print('read', fname)
    except:
        print('could not read', fname)
        df_total = parse_experiments(exp_name=exp_name)
        pd.to_pickle(df_total, fname)
        print('saved as', fname)
