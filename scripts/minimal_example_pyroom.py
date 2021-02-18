import pyroomacoustics as pra
import numpy as np
from math import ceil

plotting = True

fs = 32000  # sampling frequency [Hz]
duration_sec = 5  # duration of simulated audio signal [s]
mic_locs = np.c_[[1, 4, 1], [6, 4, 1]].T  # mics positions [m]

source_pos = np.c_[
    [2, 3, 1], [3, 3, 1], [4, 3, 1], [5, 3, 1],
].T  # source's consecutive positions [m]
room_dim = np.array([9, 7.5, 3.5])  # room dimensions [m]

# source_signal = 2.0 * (np.random.rand(int(ceil(fs * duration_sec))) - 0.5) # uniform white noise between -1 and 1
f = 100  # frequency of source signal [Hz]
source_signal = np.sin(2 * np.pi * f * np.arange(duration_sec, step=1 / fs))

start_idx = 1000  # starting index of current buffer
n_buffer = 2048  # number of samples per buffer

if plotting:
    import matplotlib.pylab as plt

    mic_idx = 0  # index of mic to plot
    fig, axs = plt.subplots(
        source_pos.shape[0], 1, sharex=True, sharey=True, squeeze=False
    )
    fig_pos, ax_pos = plt.subplots()
    [
        ax_pos.scatter(*mic[:2], marker="x", label=f"mic {j}")
        for j, mic in enumerate(mic_locs)
    ]
    ax_pos.plot([0, room_dim[0]], [0, 0], color="k")
    ax_pos.plot([room_dim[0], room_dim[0]], [0, room_dim[1]], color="k")
    ax_pos.plot([0, room_dim[0]], [room_dim[1], room_dim[1]], color="k")
    ax_pos.plot([0, 0], [0, room_dim[1]], color="k")

for i, position in enumerate(source_pos):
    pyroom = pra.ShoeBox(room_dim, fs=fs)

    # TODO(FD): find the correct range of source signal here.
    pyroom.add_source(position, signal=source_signal[: start_idx + n_buffer])
    pyroom.add_microphone_array(mic_locs.T)
    pyroom.simulate()

    simulated_signal = pyroom.mic_array.signals[:, start_idx : start_idx + n_buffer]
    start_idx += n_buffer

    if plotting:
        axs[i, 0].plot(simulated_signal[mic_idx], color=f"C{i}")
        ax_pos.scatter(*position[:2], label=f"position {i}", color=f"C{i}")
        axs[i, 0].set_ylabel(f"position {i}")

if plotting:
    axs[0, 0].set_title(f"signal of mic {mic_idx} at different positions")
    axs[-1, 0].set_xlabel("time idx [-]")
    ax_pos.set_xlabel("x [m]")
    ax_pos.set_ylabel("y [m]")
    ax_pos.legend(loc="upper right")

    try:
        plt.show()
    except KeyboardInterrupt:
        print("closing")
