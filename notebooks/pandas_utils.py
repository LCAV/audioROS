#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
pandas_utils.py: Commonly used pandas functions
"""

import pandas as pd
import numpy as np


def filter_by_dicts(df, dicts):
    """ Return the rows of df for which any of the dicts applies. """
    if len(dicts) == 1:
        print("Warning: filter_by_dicts is deprecated, replace with filter_by_dict.")
        return filter_by_dict(df, dicts[0])

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


def filter_by_dict(df, dict_):
    """ Return the rows of df for which dict_ applies. """
    mask = np.ones(len(df), dtype=bool)

    for key, vals in dict_.items():
        if np.ndim(vals) == 0:
            mask = mask & (df.loc[:, key] == vals)
        else:
            this_mask = np.zeros(len(df), dtype=bool)
            for val in vals:
                this_mask = this_mask | (df.loc[:, key] == val)
            mask = mask & this_mask

    if not np.any(mask):
        return []
    else:
        return df.loc[mask, :]


def fill_df(df, filter_dict, fill_dict, verbose=False):
    """
    Fill rows for which filter_dict applies with fill_dict. 
    """
    filter_v = list(filter_dict.values())
    filter_k = list(filter_dict.keys())

    # add missing columns
    missing_keys = set(fill_dict.keys()).difference(set(df.columns))
    if verbose:
        print("missing keys", missing_keys)
    df = df.assign(**dict(zip(missing_keys, [None] * len(missing_keys))))

    series = pd.Series(index=filter_k, data=filter_v)
    mask = (df[filter_k] == series).all(axis=1)
    if verbose:
        print("filling:\n", mask.values, np.any(mask.values))

    rows_to_fill = np.where(mask.values)[0]
    if not len(rows_to_fill):
        print(f"did not find {filter_dict} in df")
        data_dict = fill_dict
        data_dict.update(filter_dict)
        df.loc[len(df), data_dict.keys()] = list(data_dict.values())
    else:
        print(f"overwriting rows {rows_to_fill}")
        for r in rows_to_fill:
            df.loc[r, fill_dict.keys()] = list(fill_dict.values())

    df = df.apply(pd.to_numeric, downcast="integer", errors="ignore")
    return df


if __name__ == "__main__":
    df = pd.DataFrame({"a": [1, 2, 3], "b": [1, 3, 4], "c": [3, 4, 5]})
    df_new = fill_df(df, {"a": 1, "b": 1}, {"d": 5})
    df_new = fill_df(df_new, {"a": 1, "b": 1}, {"d": 6}, verbose=True)
    df_new = fill_df(df_new, {"a": 3, "b": 1}, {"d": 6})
    print(df_new, end="\n\n")

    df = pd.DataFrame({"a": [1, 2, 3], "b": [1, 3, 4], "c": [3, 4, 5]})
    df_sub = filter_by_dicts(df, [{"a": 1}, {"b": 1}])
    print(df_sub)
