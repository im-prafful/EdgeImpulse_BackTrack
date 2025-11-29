"""
Desktop feature extractor (mirrors on-device extractor)
Takes CSV raw IMU logs and produces feature vectors (numpy .npy)
"""

import numpy as np
import pandas as pd
import os, sys

WINDOW_MS = 500
SAMPLE_PERIOD_MS = 5
WINDOW_SAMPLES = WINDOW_MS // SAMPLE_PERIOD_MS
FEATURE_SIZE = 64


def sliding_windows(data, window=WINDOW_SAMPLES):
    samples = []
    n = len(data)
    for start in range(0, n - window + 1, window // 2):  # 50% overlap
        samples.append(data[start : start + window])
    return samples


def extract_stats(window):
    # window: (N,9)
    feats = []
    for ch in range(9):
        col = window[:, ch]
        feats += [col.mean(), col.std(), col.min(), col.max()]
    # energy per channel
    for ch in range(9):
        col = window[:, ch]
        feats.append((col**2).sum())
    # pad / trim to FEATURE_SIZE
    feats = np.array(feats, dtype=np.float32)
    if feats.size < FEATURE_SIZE:
        feats = np.pad(feats, (0, FEATURE_SIZE - feats.size), mode="constant")
    else:
        feats = feats[:FEATURE_SIZE]
    return feats


def process_csv(csvpath, out_features="features.npy", out_labels="labels.npy"):
    df = pd.read_csv(csvpath, header=None)
    arr = df.values.astype(np.float32)  # shape (T,9)
    windows = sliding_windows(arr)
    feats = []
    for w in windows:
        feats.append(extract_stats(w))
    feats = np.stack(feats)
    np.save(out_features, feats)
    print("Saved", out_features, feats.shape)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python extract_features.py data/raw/session.csv")
        sys.exit(1)
    process_csv(sys.argv[1])
