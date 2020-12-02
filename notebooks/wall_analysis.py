import numpy as np

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
    
    delta = 50

    # freqs_window = offset + slope * times
    times_window = (frequencies - offset) / slope
    
    if ax is not None:
        ax.plot(times_window-delta, frequencies, color='red')
        ax.plot(times_window+delta, frequencies, color='red')

    psd = np.zeros(signals_f.shape[1:]) # 4 x 32
    times = np.arange(signals_f.shape[0])
    for i, (t, f) in enumerate(zip(times_window, frequencies)):
        signals_window = signals_f[(times <= t + delta) & (times >= t - delta),:,i]
        psd[:, i] = np.sum(np.abs(signals_window)**2 / signals_window.shape[0], axis=0)
    return psd
