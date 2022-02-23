color = {
    "theoretical": "black",
    "calibrated": "C2",
    # "raw": "C3",
    "random": "C4",
    "fixed": "C5",
}
ls = {
    "histogram 1": ":",
    "histogram 3": "-.",
    "histogram 5": "--",
    "particle": "-",
    "single": (0, (1, 4)),
    "": "-"  # random
    # "": (0, (5, 7)), # random
}
labels = {f"histogram {i}": f"hist. $N_w={i}$" for i in [1, 3, 5]}
labels["particle"] = "particle"
units = {
    "bayes angle": "deg",
    "bayes distance": "cm",
}


def add_double_legend(ax, exclude=[], color=color, title="filter"):
    leg = ax.legend(loc="lower left", bbox_to_anchor=[1.0, 0.5], title="filter")
    lines = []
    for name, c in color.items():
        if name in exclude:
            continue
        name = "measured" if name == "calibrated" else name
        lines.append(ax.plot([], [], color=c)[0])
    ax.legend(
        lines,
        color.keys(),
        loc="upper left",
        bbox_to_anchor=[1.0, 0.5],
        title="used data",
    )
    ax.add_artist(leg)
