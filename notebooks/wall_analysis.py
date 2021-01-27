import itertools
import sys

import numpy as np
import pandas as pd
import progressbar

from crazyflie_description_py.parameters import N_BUFFER, FS 
from evaluate_data import read_df, read_df_from_wav, get_fname
from evaluate_data import get_positions
from dynamic_analysis import add_pose_to_df

VELOCITY = 0.05 # [m/s], parameter in crazyflie
D_START = 0.6 # [m], starting distance 

def load_params(exp_name):
    """ load parameters module at the experiment of interest """
    import importlib.util
    spec = importlib.util.spec_from_file_location("params", f"../experiments/{exp_name}/params.py")
    params = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(params)
    return params


def clean_stft(stft, max_value=N_BUFFER):
    """
    The values in stft are normally between -N_BUFFER and N_BUFFER, 
    so values outside of this range are due to communication errors.
    """
    stft[np.isnan(stft)] = 0.0
    stft[np.abs(stft) > max_value] = 0.0
    return stft


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


def parse_experiments(exp_name='2020_12_9_moving', wav=True):
    if exp_name == '2020_12_7_moving':
        appendix_list = ["", "_new"]; snr_list = [0, 1]; props_list=[0]
    elif exp_name == '2020_12_9_rotating':
        appendix_list = ["", "_new"]; snr_list = [0, 1]; props_list = [0, 1]
    elif exp_name == '2020_12_18_flying':
        appendix_list = ["", "_new"]; snr_list = [2]; props_list = [0, 1]; wav = False
    elif exp_name == '2020_12_18_stepper':
        appendix_list = ["", "_new"]; snr_list = [2]; props_list = [0, 1]
    elif exp_name == '2020_11_26_wall':
        appendix_list = [""]; snr_list = [0]; props_list = [0]; wav = True

    from crazyflie_description_py.parameters import N_BUFFER

    params_file = load_params(exp_name)

    # TODO(FD) remove this when we use more angles again.
    params_file.DEGREE_LIST = [0]

    pos_columns = ['dx', 'dy', 'z', 'yaw_deg']

    df_total = pd.DataFrame(columns=[
        'appendix', 'degree', 'distance', 'motors', 'mic_type', 'source', 'snr', 'props', # categories
        'seconds', 'frequencies', 'frequencies_matrix', 'stft', 'positions'] + pos_columns # data
    )

    params = {
        'props': False,
        'exp_name': exp_name
    }
    for degree, distance, source, motors, appendix, snr, props in itertools.product(
        params_file.DEGREE_LIST, 
        params_file.DISTANCE_LIST, 
        params_file.SOURCE_LIST, 
        params_file.MOTORS_LIST, 
        appendix_list, 
        snr_list, 
        props_list): 

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
            print('skipping', params)
            continue 
            
        for mic_type, df in mic_dfs.items():
            if df is None:
                continue
            if not 'signals_f' in df.columns:
                continue
            
            stft = np.array([*df.signals_f.values]) # n_times x n_mics x n_freqs
            stft = clean_stft(stft)

            seconds = (df.timestamp.values - df.iloc[0].timestamp) / 1000
            frequencies_matrix = np.array([*df.loc[:,'frequencies']])
            frequencies = frequencies_matrix[0, :]

            params['source'] = str(params['source'])
            all_items = dict(
                mic_type=mic_type,
                seconds=seconds, 
                frequencies=frequencies,
                frequencies_matrix=frequencies_matrix,
                stft=stft,
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


def parse_old_experiments(exp_name='2020_11_26_wall'):
    from evaluate_data import read_df, integrate_yaw
    from dynamic_analysis import add_pose_to_df
    from frequency_analysis import get_psd, get_spectrogram_raw, psd_df_from_spec, extract_psd, apply_linear_mask, get_index_matrix

    kwargs_all = {
        '2020_11_26_wall': {
            'slope': (4000 - 1000) / 200,
            'offset': 200,
            'delta': 50
        }
    }

    kwargs = kwargs_all[exp_name]

#fname = f'results/{exp_name}_simulated.pkl'
    fname = f'results/{exp_name}_real.pkl'

    SOURCE_LIST = ['mono4125', 'mono3500', None, 'sweep', 'sweep_low', 'sweep_high'] # 
    DEGREE_LIST = [0, 27, 54, 81, 360]
    DISTANCE_LIST = np.arange(50)

    try:
        raise
        df_total = pd.read_pickle(fname)
        frequencies = df_total.iloc[0].frequencies
        print('read', fname)
    except:
        print('could not read', fname)
        df_total = pd.DataFrame(columns=['stft', 'degree', 'yaw', 'distance', 'source', 'psd', 'freqs'])
        params = dict(
          props = False,
          snr = False,
          motors = False,
          exp_name = exp_name
        )
        
        for degree in DEGREE_LIST:
            for distance in DISTANCE_LIST:
                for source in SOURCE_LIST:
                    try:
                        params['degree'] = degree
                        params['distance'] = distance
                        params['source'] = source
                        params['appendix'] = ""
                        df, df_pos = read_df(**params)
                    except Exception as e:
                        continue 

                    # detect index decrease (happens when two csv files are concatenated)
                    sign = np.sign(df['index'].values[1:] - df['index'].values[:-1])
                    if np.any(sign < 0):
                        index = np.where(sign<0)[0][-1]
                        print('Warning: found multiple start indices, start at', index)
                        df = df.iloc[index:]
                        index_start = df.iloc[0]['index']
                        df_pos = df_pos.loc[df_pos.index >= index_start]

                    stft = np.array([*df.signals_f.values]) # n_times x n_mics x n_freqs
                    frequencies_matrix = np.array([*df.loc[:,'frequencies']])

                    if degree == 360:
                        add_pose_to_df(df, df_pos, max_allowed_lag_ms=50)
                        yaw = integrate_yaw(df.timestamp.values, df.yaw_rate_deg.values)
                    else:
                        yaw = np.full(len(df), -degree)

                    spec, freqs = get_spectrogram_raw(frequencies_matrix, stft)
                    spec_masked, freqs = apply_linear_mask(spec, freqs, 
                                                           kwargs['slope'], kwargs['offset'], kwargs['delta'])
                    index_matrix = get_index_matrix(spec_masked)
                    psd_df = psd_df_from_spec(spec_masked, freqs, index_matrix)
                    psd, psd_freqs, freqs, std = extract_psd(psd_df)
                    
                    psd_old = get_psd(stft, freqs, fname='real')

                    df_total.loc[len(df_total), :] = dict(
                        degree=degree,
                        yaw=yaw,
                        distance=distance,
                        source=str(source),
                        stft=stft,
                        freqs=freqs,
                        spec=spec,
                        psd=psd,
                        psd_freqs=psd_freqs,
                    )
        pd.to_pickle(df_total, fname)
        print('saved as', fname)


def parse_calibration_experiments():
    df_total = pd.DataFrame(columns=['source', 'snr', 'motors', 'exp_name', 'appendix', 
                                     'seconds', 'frequencies', 'frequencies_matrix', 'stft'])

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
            
        stft = np.array([*df.signals_f.values]) # n_times x n_mics x n_freqs
        stft = clean_stft(stft)

        seconds = (df.timestamp.values - df.iloc[0].timestamp) / 1000
        frequencies_matrix = np.array([*df.loc[:,'frequencies']])
        frequencies = frequencies_matrix[0, :]

        df_total.loc[len(df_total), :] = dict(
            source=str(source), 
            snr=filter_, 
            motors=motors, 
            exp_name=exp_name, 
            appendix=appendix, 
            seconds=seconds, 
            frequencies=frequencies, 
            frequencies_matrix=frequencies_matrix,
            stft=stft, 
        )
    return df_total


if __name__ == "__main__":
    import os

    all_exp_names = ['2020_12_9_rotating',
                     '2020_12_18_flying',
                     '2020_12_18_stepper']
    #exp_name = '2020_12_9_rotating'
    #exp_name = '2020_12_18_flying'
    #exp_name = '2020_12_18_stepper'
    for exp_name in ['2020_11_26_wall']:

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
