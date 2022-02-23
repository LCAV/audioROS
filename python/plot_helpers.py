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
labels = {f"histogram {i}": f"histogram $N_w={i}$" for i in [1, 3, 5]}
labels["particle"] = "particle"
units = {
    "bayes angle": "deg",
    "bayes distance": "cm",
}
