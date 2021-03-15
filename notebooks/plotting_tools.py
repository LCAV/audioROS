import matplotlib.pylab as plt
import numpy as np


def make_dirs(fname):
    import os

    dirname = os.path.dirname(fname)
    if not os.path.exists(dirname):
        os.makedirs(dirname)


def save_fig(fig, fname, extension="pdf"):
    extension_current = fname.split(".")[-1]
    fname = fname.replace("." + extension_current, "." + extension)
    make_dirs(fname)
    fig.savefig(fname, bbox_inches="tight")
    print("saved as", fname)


def plot_square(x_indices, y_indices, ax):
    ax.plot(
        [x_indices[0], x_indices[-1]],
        [y_indices[-1], y_indices[-1]],
        color="red",
        linewidth=1,
    )
    ax.plot(
        [x_indices[0], x_indices[-1]],
        [y_indices[0], y_indices[0]],
        color="red",
        linewidth=1,
    )
    ax.plot(
        [x_indices[0], x_indices[0]],
        [y_indices[0], y_indices[-1]],
        color="red",
        linewidth=1,
    )
    ax.plot(
        [x_indices[-1], x_indices[-1]],
        [y_indices[0], y_indices[-1]],
        color="red",
        linewidth=1,
    )


def plot_df_matrix(
    dist, freq, df_matrix, ax=None, min_freq=None, max_freq=None, **kwargs
):
    if ax is None:
        fig, ax = plt.subplots()

    mask = np.ones(freq.shape, dtype=np.bool)
    if min_freq is not None:
        mask &= freq > min_freq
    if max_freq is not None:
        mask &= freq < max_freq
    im = ax.pcolorfast(dist, freq[mask], df_matrix[mask][:-1, :-1], **kwargs)
    return ax, im


def add_colorbar(fig, ax, im, title=None):
    from mpl_toolkits.axes_grid1 import make_axes_locatable

    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="5%", pad=0.05)
    fig.colorbar(im, cax=cax, orientation="vertical")
    if title is not None:
        cax.set_ylabel(title)


def plot_spectrograms(df_freq):
    from frequency_analysis import get_spectrogram_raw

    mic_type = "measurement"
    cut_x = range(0, 20)
    cut_y = range(1000, 3000)

    filters = ["mic_type", "snr", "source", "degree", "distance"]
    for chosen_tuple, df_source in df_freq.groupby(filters):

        fig, axs = plt.subplots(1, 2, squeeze=False)
        fig.set_size_inches(15, 5)
        fig.suptitle(dict(zip(filters, chosen_tuple)))

        for j, motors in enumerate([0, "all43000"]):
            df_this = df_source[df_source.motors == motors]
            if len(df_this) != 1:
                print(df_this)
                continue
            row = df_this.iloc[0]

            spec_all, freqs = get_spectrogram_raw(row.frequencies_matrix, row.stft)
            spec = np.mean(spec_all, axis=1)  # n_times x n_mics x n_freqs

            axs[0, j].pcolorfast(row.seconds, freqs, np.log10(spec[:-1, :-1]))
            axs[0, j].set_title(f"motors {motors}")


def plot_raw_signals(spec_masked_all, freqs_masked, mic_idx=0, delta=50):
    """ 
    :param delta: consider frequencies more than delta Hz appart as new frequency.
    """
    spec_mic = spec_masked_all[:, mic_idx, :]

    start_idx = None
    counter = 0
    indices = []

    fig, ax = plt.subplots()
    fig.set_size_inches(15, 5)
    for i in range(spec_mic.shape[1]):
        new_idx = np.argmax(spec_mic[:, i])
        if (start_idx is None) or (
            abs(freqs_masked[new_idx] - freqs_masked[start_idx]) > delta
        ):

            # plot old frequencies.
            # some frequencies have only one or two realizations, they are typically outliers.
            if len(indices) > 2:
                label = f"{counter}:{freqs_masked[new_idx]}Hz"
                for idx in indices:
                    freq_indices = np.where(spec_mic[:, idx] > 0)[0]
                    ax.plot(
                        freqs_masked[freq_indices],
                        spec_mic[freq_indices, idx],
                        color=f"C{counter}",
                        label=label,
                    )
                    label = None
                counter += 1

            # initialize new frequencies
            start_idx = new_idx
            indices = []

        indices.append(i)

    ax.set_yscale("log")
    ax.grid(which="both")
    ax.set_xlabel("frequency [Hz]")
    ax.set_ylabel("amplitude")
    # ax.legend(loc='upper left', bbox_to_anchor=[1, 1])
    return fig, ax
