#! /usr/bin/env python3
# -*- coding: utf-8 -*-
"""
generate_classifier_results.py: Detect if there is a wall or not based on distributions.
"""

import matplotlib.pylab as plt
import numpy as np
import pandas as pd
from progressbar import ProgressBar
from sklearn.metrics import auc

from crazyflie_demo.wall_detection import DISTANCES_CM
from utils.moving_estimators import get_estimate

PLOT = False

DIST_THRESH = 20
TAIL_THRESH = -5
STD_THRESH = 2

# if we didn't see a wall and there was one at < D_FALSE_NEG, this is a false negative.
# if we saw a wall and there was none at < D_FALSE_NEG this is a false positive.
# used only if USE_FIXED_THRESH is True
D_FALSE_NEG = 20

USE_FIXED_THRESH = True

# walls location per dataset, [distance_cm, angle_deg]
WALLS_DICT = {
    "2021_10_12_flying": [[100, 90]],
    "2022_01_27_demo": [[100, 90], [100, -90]],
}

THRESHOLDS_DICT = {
    "distance-mean": np.arange(40, step=1)[::-1],
    "distance-max": np.arange(40, step=1)[::-1],
    "std-mean": np.arange(10, step=0.5)[::-1],
    "std-max": np.arange(10, step=0.5)[::-1],
    "tail": np.arange(-7, 0, step=0.5)[::-1],
}

METHODS = [
    "distance-mean",
    # "distance-max",
    "std-mean",
    # "std-max",
    # "tail",
]


def detect_wall(
    distances_cm, prob_moving_dist, method="distance-mean", thresh=DIST_THRESH
):
    if "distance" in method:
        d_estimate, __ = get_estimate(
            distances_cm, prob_moving_dist, method=method.lstrip("distance-")
        )
        # print(f"estimate: {d_estimate:.0f}")
        if d_estimate is None:
            return None
        return d_estimate < thresh
    elif "std" in method:
        __, std = get_estimate(
            distances_cm, prob_moving_dist, method=method.lstrip("std-")
        )
        if std is None:
            return None
        return std < thresh
    elif method == "tail":
        mean_tail = np.mean(prob_moving_dist[-3:])
        if mean_tail == 0:
            return None
        return np.log10(mean_tail) < thresh


def get_groundtruth_distances(
    exp_name, appendix, walls=None, flying=False, angles=False, correct=False
):
    def normal(angle_deg):
        return np.r_[
            [np.cos(angle_deg / 180 * np.pi)], [np.sin(angle_deg / 180 * np.pi)]
        ]

    data_df = pd.read_pickle(f"../datasets/{exp_name}/all_data.pkl")
    row = data_df.loc[data_df.appendix == appendix].iloc[0]
    if flying:
        from crazyflie_description_py.parameters import FLYING_HEIGHT_CM

        mask_flying = row.positions[:, 2] > FLYING_HEIGHT_CM * 1e-2
        positions_cm = row.positions[mask_flying, :2] * 1e2
        yaws_deg = row.positions[mask_flying, 3]
    else:
        positions_cm = row.positions[:, :2] * 1e2
        yaws_deg = row.positions[:, 3]

    if walls is None:
        walls = WALLS_DICT[exp_name]
    distances_wall = None  # np.full(positions_cm.shape[0], np.nan)
    angles_wall = None  # np.full(positions_cm.shape[0], np.nan)
    for wall in walls:
        distances_here = wall[0] - positions_cm.dot(normal(wall[1]))
        angles_here = wall[1] - yaws_deg
        if distances_wall is None:
            distances_wall = distances_here
        else:
            distances_wall = np.min(np.c_[distances_here, distances_wall], axis=1)
        if angles_wall is None:
            angles_wall = angles_here
        else:
            angles_wall = np.min(np.c_[angles_here, angles_wall], axis=1)

    # we correct the distances using the fact that the last distance
    # should be equal to 7 i.e. hitting the wall
    if correct:
        distances_wall -= distances_wall[-1] - 10

    if not angles:
        return distances_wall
    else:
        return distances_wall, angles_wall


def get_precision_recall(matrix, distances_wall, method, verbose=False, sort=True):
    thresholds = THRESHOLDS_DICT[method]
    false_positives = np.zeros_like(thresholds)
    false_negatives = np.zeros_like(thresholds)
    true_positives = np.zeros_like(thresholds)

    # for each chosen threshold...
    for j, thresh in enumerate(thresholds):
        # ...loop through dataset and aggregate false positives, false negatives, true positives.
        for i, prob_moving_dist in enumerate(matrix.T):
            if i >= len(distances_wall):
                # print("Warning: length mismatch. This is cause distances_wall1 is defined for flying indices only")
                continue

            wall_estimate = detect_wall(
                DISTANCES_CM, prob_moving_dist, method, thresh=thresh
            )
            if wall_estimate is None:
                continue

            if USE_FIXED_THRESH:
                wall_gt = distances_wall[i] < D_FALSE_NEG
            else:
                wall_gt = distances_wall[i] < thresh

            # false positive
            if wall_estimate and (not wall_gt):
                false_positives[j] += 1
            # false negative
            elif (not wall_estimate) and wall_gt:
                false_negatives[j] += 1
            # true positives
            elif wall_estimate and wall_gt:
                true_positives[j] += 1
        if verbose:
            print(
                f"For threshold {thresh}: Fp",
                false_positives[j],
                "Fn",
                false_negatives[j],
                "Tp",
                true_positives[j],
            )

    precision = np.full(len(thresholds), np.nan)
    recall = np.full(len(thresholds), np.nan)

    denom = true_positives + false_positives
    precision[denom > 0] = true_positives[denom > 0] / denom[denom > 0]
    denom = true_positives + false_negatives
    recall[denom > 0] = true_positives[denom > 0] / denom[denom > 0]

    if sort:
        sort_idx = np.argsort(precision)
        sort_idx = sort_idx[~np.isnan(precision[sort_idx])]
        precision = precision[sort_idx]
        recall = recall[sort_idx]
    return precision, recall


def generate_classifier_results(matrix_df, fname="", verbose=False):
    distances_wall = get_groundtruth_distances("2022_01_27_demo", appendix="test4")

    categories = matrix_df.columns.drop(["matrix distances", "matrix angles"]).values

    scores = pd.DataFrame(columns=["method", "auc"] + list(categories))

    progress_count = 0
    progressbar = ProgressBar(maxval=len(matrix_df))
    progressbar.start()
    for k, row in matrix_df.iterrows():
        matrix = row["matrix distances"][0]

        if PLOT:
            fig, ax = plt.subplots()
            fig.set_size_inches(5, 5)

        for method in METHODS:

            precision, recall = get_precision_recall(
                matrix, distances_wall, method, verbose=False
            )
            try:
                metric = auc(precision, recall)
            except:
                print(method, matrix.shape)
                print(precision, recall)
                raise

            fill_dict = {
                "method": method,
                "auc": metric,
            }
            fill_dict.update({c: row[c] for c in categories})
            scores.loc[len(scores), :] = fill_dict

            # ax.scatter(precision, recall, label=method)
            if PLOT:
                ax.plot(precision, recall, label=f"{method}: {metric:.1f}")

        progress_count += 1
        progressbar.update(progress_count)
        if PLOT:
            ax.legend(loc="upper right")
            ax.set_xlabel("precision")
            ax.set_ylabel("recall")
            ax.set_xlim(0, 1.1)
            ax.set_ylim(0, 1.1)
            ax.grid(True, which="both")
            ax.set_title(title)

        if fname != "":
            scores.to_pickle(fname)
            # print(f"Saved intermediate as {fname}")
    return scores


if __name__ == "__main__":
    for estimator in ["particle", "moving"]:
        try:
            matrix_df = pd.read_pickle(f"results/demo_results_matrices_{estimator}.pkl")
        except FileNotFoundError:
            print("Run generate_flying_results.py to generate results.")
            raise

        fname = f"results/demo_results_classifier_{estimator}.pkl"
        generate_classifier_results(matrix_df, fname, verbose=False)
