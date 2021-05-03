import matplotlib
import matplotlib.pylab as plt
import numpy as np
import pandas as pd

labels = {
    "fft": "FFT method",
    "cost": "optimization method",
    "sigmadelta": "delta noise $\sigma_\\Delta$ [cm]",
    "sigmay": "amplitude noise $\sigma_y$ [-]",
    "distance": "distance $d$ [cm]",
    np.nanstd: "error std",
    np.nanmedian: "median error",
    np.nanmean: "mean error",
}


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


def plot_performance(err_dict, xs=None, xlabel="", ylabel="error"):
    """ Plot error evolution over xs and as cdf. """
    if xs is None:
        xs = range(len(err_dict.values()[0]))

    from matplotlib.lines import Line2D

    markers = [m for m in list(Line2D.markers.keys()) if m not in [".", "", ","]]

    i = 0
    fig, axs = plt.subplots(1, 2, squeeze=False)
    fig.set_size_inches(10, 5)
    max_abs = 0
    for method, err_list in err_dict.items():
        markersize = 8 - i

        axs[0, 0].plot(
            xs,
            err_list,
            label=method,
            marker=markers[i],
            ls=":",
            markersize=markersize,
        )

        xvals = sorted(np.abs(err_list))
        yvals = np.linspace(0, 1, len(xvals))
        axs[0, 1].plot(
            xvals,
            yvals,
            label=method,
            marker=markers[i],
            ls=":",
            markersize=markersize,
        )
        i += 1

        max_abs = max(max_abs, max(xvals))

    # axs[0, 0].set_yscale('log')
    axs[0, 0].set_ylim(-max_abs, max_abs)
    axs[0, 0].set_ylabel(ylabel)
    axs[0, 0].set_xlabel(xlabel)
    axs[0, 0].legend(loc="upper right")

    axs[0, 1].set_ylabel("cdf")
    axs[0, 1].set_xlabel("absolute " + ylabel)
    axs[0, 1].grid(which="both")
    axs[0, 1].legend(loc="lower right")
    return fig, axs


def pcolorfast_custom(ax, xs, ys, values, verbose=False, **kwargs):
    """ pcolorfast with gray for nan and centered xticks and yticks. 
    """
    current_cmap = matplotlib.cm.get_cmap()
    current_cmap.set_bad(color="gray")

    assert values.shape == (len(ys), len(xs))

    dx = xs[1] - xs[0]  # assumes uniform samples
    dy = ys[1] - ys[0]
    try:
        im = ax.pcolorfast(xs, ys, values, **kwargs)
        yticks = ys + dy / 2
        xticks = xs + dx / 2
        ax.set_xticks(xticks)
        ax.set_xticklabels(xs)
        ax.set_yticks(yticks)
        ax.set_yticklabels(ys)
        extent = [xs[0], xs[-1] + dx, ys[0], ys[-1] + dy]
        im.set_extent(extent)
    except:
        # print("Warning: problem with dimensions in pcolorfast (bug by matplotlib)")
        im = ax.pcolorfast(
            list(xs) + [xs[-1] + dx], list(ys) + [ys[-1] + dy], values, **kwargs
        )
    return im


def plot_error_distance(
    sub_df, column, name, log=False, aggfunc=np.nanmedian, vmin=None, vmax=None
):
    table = pd.pivot_table(
        sub_df,
        values="error",
        index=["method", column],
        columns="distance",
        aggfunc=aggfunc,
    )
    nonzero_values = table.values[table.values > 0]
    if vmin is None and len(nonzero_values):
        vmin = np.min(nonzero_values)
    if vmax is None and len(nonzero_values):
        vmax = np.max(nonzero_values)

    fig, axs = plt.subplots(1, len(sub_df.method.unique()), sharey=True, squeeze=False)
    fig.set_size_inches(5 * len(sub_df.method.unique()), 5)
    for i, (method, df) in enumerate(table.groupby("method")):
        index = df.index.get_level_values(column).values
        distances = df.columns.values
        if log:
            im = pcolorfast_custom(
                axs[0, i],
                distances,
                index,
                np.log10(df.values),
                vmin=np.log10(vmin),
                vmax=np.log10(vmax),
            )
        else:
            im = pcolorfast_custom(
                axs[0, i], distances, index, df.values, vmin=vmin, vmax=vmax
            )
        axs[0, i].set_xlabel("distance $d$ [cm]")
        axs[0, i].set_title(labels[method])
    add_colorbar(fig, axs[0, -1], im, title=f"{labels[aggfunc]} [cm]")
    axs[0, 0].set_ylabel(name.replace("_", " "))
    return fig, axs


def plot_error_gamma(
    sub_df,
    column,
    name,
    log=False,
    aggfunc=np.nanmedian,
    vmin=None,
    vmax=None,
    logy=False,
    ax=None,
    fig=None,
    colorbar=True,
):
    table = pd.pivot_table(
        sub_df,
        values="error",
        index=["method", column],
        columns="gamma",
        aggfunc=aggfunc,
    )
    nonzero_values = table.values[table.values > 0]
    if vmin is None and len(nonzero_values):
        vmin = np.min(nonzero_values)
    if vmax is None and len(nonzero_values):
        vmax = np.max(nonzero_values)

    if ax is None:
        fig, ax = plt.subplots()
        fig.set_size_inches(5, 5)

    ys = table.index.get_level_values(column).values
    # print(column, ys)
    if logy:
        ys = np.log10(ys)
    gammas = table.columns.values
    if log:
        im = pcolorfast_custom(
            ax,
            gammas,
            ys,
            np.log10(table.values),
            vmin=np.log10(vmin),
            vmax=np.log10(vmax),
        )
    else:
        im = pcolorfast_custom(ax, gammas, ys, table.values, vmin=vmin, vmax=vmax)
    ax.set_xlabel("approach angle $\\gamma$ [deg]")
    if colorbar:
        from plotting_tools import add_colorbar

        add_colorbar(fig, ax, im, title=f"{labels[aggfunc]} [deg]")
    ax.set_ylabel(name.replace("_", " "))
    return fig, ax
