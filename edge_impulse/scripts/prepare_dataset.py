"""
prepare_dataset.py
------------------
Builds the full training-ready dataset from raw IMU CSV logs.

Features:
- Loads raw CSV logs
- Cleans NaNs
- Applies 500 ms sliding window (same as ESP32 firmware)
- Extracts EI-compatible feature vector (same as feature_extractor.cpp)
- Encodes labels from folder names
- Generates train/test split
- Saves .npy datasets + CSV previews

Expected dataset structure:
data/
 ├── raw/
 │     ├── good/
 │     │     ├── session1.csv
 │     │     ├── session2.csv
 │     ├── slight/
 │     │     ├── ...
 │     ├── bad/
 │           ├── ...
"""

import os
import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split

# CONFIG
WINDOW_MS = 500
SAMPLE_PERIOD_MS = 5
WINDOW_SAMPLES = WINDOW_MS // SAMPLE_PERIOD_MS
FEATURE_SIZE = 64  # must match firmware FEATURE_VECTOR_SIZE
CHANNELS = 9  # ax, ay, az, gx, gy, gz, mx, my, mz

DATA_RAW = "data/raw"
OUT_DIR = "data/processed"

os.makedirs(OUT_DIR, exist_ok=True)


# Feature extraction (same as firmware)
def extract_stats(window):
    feats = []

    # window shape: (N,9)
    for ch in range(CHANNELS):
        col = window[:, ch]
        feats += [
            float(col.mean()),
            float(col.std()),
            float(col.min()),
            float(col.max()),
        ]

    # energy per channel
    for ch in range(CHANNELS):
        col = window[:, ch]
        feats.append(float(np.sum(col**2)))

    # pad or trim to FEATURE_SIZE
    feats = np.array(feats, dtype=np.float32)

    if len(feats) < FEATURE_SIZE:
        feats = np.pad(feats, (0, FEATURE_SIZE - len(feats)))
    else:
        feats = feats[:FEATURE_SIZE]

    return feats


def sliding_windows(arr):
    windows = []
    # 50% overlap
    step = WINDOW_SAMPLES // 2
    for start in range(0, len(arr) - WINDOW_SAMPLES + 1, step):
        win = arr[start : start + WINDOW_SAMPLES]
        windows.append(win)
    return windows


# Load one CSV file
def load_csv(path):
    df = pd.read_csv(path, header=None)
    df = df.dropna()
    arr = df.values.astype(np.float32)
    if arr.shape[1] != CHANNELS:
        raise ValueError(f"{path} has {arr.shape[1]} channels, expected {CHANNELS}")
    return arr


# Process folder into features + labels
def process_class_folder(folder_path, label):
    X_list, y_list = [], []
    for fname in os.listdir(folder_path):
        if not fname.endswith(".csv"):
            continue

        full = os.path.join(folder_path, fname)
        print("Loading", full)
        arr = load_csv(full)

        wins = sliding_windows(arr)
        for w in wins:
            feats = extract_stats(w)
            X_list.append(feats)
            y_list.append(label)

    return np.array(X_list, dtype=np.float32), np.array(y_list, dtype=np.int32)


# Main builder
def build_dataset():
    classes = sorted(os.listdir(DATA_RAW))
    label_map = {cls_name: i for i, cls_name in enumerate(classes)}

    print("Label map:", label_map)

    X_all = []
    y_all = []

    for cls_name, lbl in label_map.items():
        path = os.path.join(DATA_RAW, cls_name)
        Xi, yi = process_class_folder(path, lbl)
        X_all.append(Xi)
        y_all.append(yi)

    X_all = np.vstack(X_all)
    y_all = np.concatenate(y_all)

    print("Dataset size:", X_all.shape, y_all.shape)

    # Train/Test split
    X_train, X_test, y_train, y_test = train_test_split(
        X_all, y_all, test_size=0.20, random_state=42, stratify=y_all
    )

    np.save(os.path.join(OUT_DIR, "X_train.npy"), X_train)
    np.save(os.path.join(OUT_DIR, "X_test.npy"), X_test)
    np.save(os.path.join(OUT_DIR, "y_train.npy"), y_train)
    np.save(os.path.join(OUT_DIR, "y_test.npy"), y_test)

    print("Saved dataset to:", OUT_DIR)
    print("Train:", X_train.shape, "Test:", X_test.shape)


if __name__ == "__main__":
    build_dataset()
