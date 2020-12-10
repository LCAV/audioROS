import itertools
import sys

import numpy as np
import pandas as pd

from evaluate_data import read_df, read_df_from_wav, get_fname

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


def get_psd(signals_f, frequencies, ax=None, fname='real'):
    # signals_f shape: times, 4, 32
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


def parse_experiments(exp_name='2020_12_9_moving'):
    if exp_name == '2020_12_7_moving':
        appendix_list = ["", "_new"]; snr_list = [0, 1]; props_list=[0]
    elif exp_name == '2020_12_9_rotating':
        appendix_list = ["", "_new"]; snr_list = [0, 1]; props_list = [0, 1]

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
        try:
            params['motors'] = motors
            params['degree'] = degree
            params['distance'] = distance
            params['source'] = source
            params['appendix'] = appendix
            params['snr'] = snr
            params['props'] = props
            df_csv, __ = read_df(**params)
            
            fname = get_fname(**params)
            df_wav = read_df_from_wav(f'../experiments/{exp_name}/export/{fname}.wav', n_buffer=N_BUFFER)
            
        except FileNotFoundError as e:
            continue 
            
        for mic_type, df in zip(['audio_deck', 'measurement'], [df_csv, df_wav]):
            
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
            
            spec = np.sum(np.abs(signals_f), axis=1)
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


if __name__ == "__main__":
    exp_name = '2020_12_9_rotating'
    fname = f'results/{exp_name}_real.pkl'
    try:
        df_total = pd.read_pickle(fname)
        print('read', fname)
    except:
        print('could not read', fname)
        df_total = parse_experiments(exp_name=exp_name)
        pd.to_pickle(df_total, fname)
        print('saved intermediate as', fname)
