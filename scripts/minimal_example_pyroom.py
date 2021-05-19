import time

import matplotlib.pylab as plt
import numpy as np

import pyroomacoustics as pra
from audio_simulation.pyroom_helpers import simulate_truncated

if __name__ == "__main__":
    verbose = True

    fs = 32000  # sampling frequency [Hz]
    duration_sec = 5  # duration of simulated audio signal [s]
    mic_locs = np.c_[[1, 4, 1], [6, 4, 1]].T  # mics positions [m]
    plot_mic_idx = 0  # index of mic to plot

    # TODO(FD) add also a variable source_times, with the times at which the source is at below 
    # positions. Currently the times are implicitly assumed to be exactly n_buffer*fs appart. 
    source_pos = np.c_[
        [2, 3, 1], [3, 3, 1], [4, 3, 1], [5, 3, 1],
    ].T  # source's consecutive positions [m]
    room_dim = np.array([9, 7.5, 3.5])  # room dimensions [m]

    # source_signal = 2.0 * (np.random.rand(int(ceil(fs * duration_sec))) - 0.5) # uniform white noise between -1 and 1
    f = 100  # frequency of source signal [Hz]
    source_signal = np.sin(2 * np.pi * f * np.arange(duration_sec, step=1 / fs))

    n_buffer = 2048  # number of samples per buffer

    # TODO(FD) find a general formula for the required number of extra samples
    n_extra_samples = 1000  # add this number of extra samples to avoid boundary effects

    start_idx = 5000  # starting index of buffer

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
        # full
        pyroom = pra.ShoeBox(room_dim, fs=fs)

        pyroom.add_source(position, signal=source_signal[: start_idx + n_buffer])
        pyroom.add_source([0, 0, 0], signal=np.zeros(start_idx+n_buffer)) # add zero source to make sure it works with multiple sources too.
        pyroom.add_microphone_array(mic_locs.T)
        t1 = time.time()
        pyroom.simulate()
        if verbose:
            print(f"length {start_idx + n_buffer}: took {(time.time() - t1) * 1000:.0f}ms")

        simulated_signal_full = pyroom.mic_array.signals[
            :, start_idx : start_idx + n_buffer
        ]

        # truncated
        simulated_signal = simulate_truncated(pyroom, start_idx, n_buffer, verbose=verbose)

        start_idx += n_buffer

        # plotting
        axs[i, 0].plot(
            simulated_signal[plot_mic_idx], color=f"C{i}", label="truncated simulation"
        )
        axs[i, 0].plot(
            simulated_signal_full[plot_mic_idx],
            color=f"C{i}",
            ls=":",
            label="full simulation",
        )
        ax_pos.scatter(*position[:2], label=f"position {i}", color=f"C{i}")
        axs[i, 0].set_ylabel(f"position {i}")
        axs[i, 0].legend(loc="upper right")

    axs[0, 0].set_title(
        f"signal of mic {plot_mic_idx} with source at different positions"
    )
    axs[-1, 0].set_xlabel("time idx [-]")
    ax_pos.set_xlabel("x [m]")
    ax_pos.set_ylabel("y [m]")
    ax_pos.legend(loc="upper right")

    plt.show()
