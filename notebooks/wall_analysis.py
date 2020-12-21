import itertools
import sys

import numpy as np
import pandas as pd

from evaluate_data import read_df, read_df_from_wav, get_fname


def filter_by_dicts(df, dicts):
    mask = np.zeros(len(df), dtype=bool)
    for dict_ in dicts:
        this_mask = np.ones(len(df), dtype=bool)
        for key, val in dict_.items():
            this_mask = this_mask & (df.loc[:, key] == val)
        mask = np.bitwise_or(mask, this_mask)
        assert np.any(mask)
    return df.loc[mask, :]


def extract_linear_psd(signals_f, frequencies, slope, offset, delta=50, ax=None):
    # freqs_window = offset + slope * times
    times_window = (frequencies - offset) / slope
    
    if ax is not None:
        ax.plot(times_window-delta, frequencies, color='red')
        ax.plot(times_window+delta, frequencies, color='red')

    psd = np.zeros(signals_f.shape[1:]) # 4 x 32
    times = np.arange(signals_f.shape[0])
    for i, t in enumerate(times_window):
        signals_window = signals_f[(times <= t + delta) & (times >= t - delta), : , i]
        psd[:, i] = np.sum(np.abs(signals_window)**2 / signals_window.shape[0], axis=0)
    return psd


def extract_psd_dict(signals_f, frequencies_matrix, min_t=0, max_t=None, n_freq=1, ax=None):
    """
    Extract a hash table from the signals and frequencies information, one for 
    each microphone.
    
    structure of output: 
    
    mic0: {
        f0: [val0, val1],
        f1: [val0, val1, val2], 
        f2: [...]
    }
    mic1: ...
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
                psd_dict[i_mic][f].append(np.abs(signals_f[i_t, i_mic, f_idx]))
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


def parse_experiments(exp_name='2020_12_9_moving', wav=True):
    if exp_name == '2020_12_7_moving':
        appendix_list = ["", "_new"]; snr_list = [0, 1]; props_list=[0]
    elif exp_name == '2020_12_9_rotating':
        appendix_list = ["", "_new"]; snr_list = [0, 1]; props_list = [0, 1]
    elif exp_name == '2020_12_18_flying':
        appendix_list = ["", "_new"]; snr_list = [2]; props_list = [0, 1]; wav = False
    elif exp_name == '2020_12_18_stepper':
        appendix_list = ["", "_new"]; snr_list = [2]; props_list = [0, 1]

    max_value = 100 # all signals received are actually between 0 and 2. 
    sys.path = [p for p in sys.path if not 'experiments' in p]
    sys.path.insert(0, f'../experiments/{exp_name}')
    from params import SOURCE_LIST, DISTANCE_LIST, DEGREE_LIST, MOTORS_LIST
    N_BUFFER = 2048

    DEGREE_LIST = [0]

    df_total = pd.DataFrame(columns=['signals_f', 'degree', 'distance', 'source', 'snr', 
                                     'motors', 'psd', 'spec', 'frequencies', 'appendix', 'seconds', 'mic_type', 'frequencies_matrix'])

    params = dict(
        props = False,
        exp_name = exp_name
    )
    for degree, distance, source, motors, appendix, snr, props in itertools.product(
        DEGREE_LIST, DISTANCE_LIST, SOURCE_LIST, MOTORS_LIST, appendix_list, snr_list, props_list): 

        mic_dfs = {'audio_deck': None, 'measurement': None}
        try:
            params['motors'] = motors
            params['degree'] = degree
            params['distance'] = distance
            params['source'] = source
            params['appendix'] = appendix
            params['snr'] = snr
            params['props'] = props

            df_csv, __ = read_df(**params)
            mic_dfs['audio_deck'] = df_csv
            
            fname = get_fname(**params)

            if wav: 
                df_wav = read_df_from_wav(f'../experiments/{exp_name}/export/{fname}.wav', n_buffer=N_BUFFER)
                mic_dfs['measurement'] = df_wav
            
        except FileNotFoundError as e:
            continue 
            
        for mic_type, df in mic_dfs.items():
            if df is None:
                continue
            if not 'signals_f' in df.columns:
                continue
            
            signals_f = np.array([*df.signals_f.values]) # n_times x n_mics x n_freqs
            if np.any(np.abs(signals_f) > max_value):
                signals_f[np.where(np.abs(signals_f) > max_value)] = 0

            seconds = (df.timestamp.values - df.iloc[0].timestamp) / 1000
            frequencies_matrix = np.array([*df.loc[:,'frequencies']])
            frequencies = frequencies_matrix[0, :]
            # save frequency matrix only if frequencies vary.
            if not np.any(np.any(frequencies_matrix - frequencies[None, :], axis=0)):
                frequencies_matrix = None
            else:
                print('saving frequencies matrix')
            
            spec = np.sum(np.abs(signals_f), axis=1) # average over mics
            psd = get_psd(signals_f, frequencies, fname='real')

            params['source'] = str(params['source'])
            all_items = dict(
                mic_type=mic_type,
                signals_f=signals_f,
                frequencies=frequencies,
                frequencies_matrix=frequencies_matrix,
                spec=spec,
                psd=psd,
                seconds=seconds
            )
            all_items.update(params)
            df_total.loc[len(df_total), :] = all_items
    return df_total


def parse_calibration_experiments():
    max_value = 100 # all signals received are actually between 0 and 2. 
    df_total = pd.DataFrame(columns=['signals_f', 'source', 'snr', 'motors', 'exp_name', 'appendix', 
                                     'seconds', 'frequencies', 'frequencies_matrix'])

    exp_name_list = [
        '2020_12_11_calibration',
    ]
    source_list = ['sweep', 'sweep_buzzer']
    appendix_list = ['', '_BC329', '_HALL', '_HALL2', '_HALL3', '_HALL3ok']
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
        if np.any(np.abs(signals_f) > max_value):
            signals_f[np.where(np.abs(signals_f) > max_value)] = 0

        seconds = (df.timestamp.values - df.iloc[0].timestamp) / 1000
        frequencies_matrix = np.array([*df.loc[:,'frequencies']])
        frequencies = frequencies_matrix[0, :]
        # save frequency matrix only if frequencies vary.
        if not np.any(np.any(frequencies_matrix - frequencies[None, :], axis=0)):
            frequencies_matrix = None
        else:
            print('saving frequencies matrix')
        
        df_total.loc[len(df_total), :] = dict(
            exp_name=exp_name, 
            appendix=appendix, 
            source=str(source), 
            snr=filter_, 
            motors=motors, 
            signals_f=signals_f, 
            seconds=seconds, 
            frequencies=frequencies, 
            frequencies_matrix=frequencies_matrix
        )
    return df_total


if __name__ == "__main__":
    #exp_name = '2020_12_9_rotating'
    exp_name = '2020_12_18_flying'
    fname = f'results/{exp_name}_real.pkl'
    try:
        raise 
        df_total = pd.read_pickle(fname)
        print('read', fname)
    except:
        print('could not read', fname)
        df_total = parse_experiments(exp_name=exp_name)
        pd.to_pickle(df_total, fname)
        print('saved intermediate as', fname)
