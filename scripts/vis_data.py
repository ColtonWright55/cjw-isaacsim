#!/usr/bin/env python3

import json
import numpy as np
import matplotlib.pyplot as plt
import os

def load_data(data_dir="_lidar_data"):
    """Load LIDAR data from JSON files"""
    metadata_file = os.path.join(data_dir, "lidar_metadata.json")
    data_file = os.path.join(data_dir, "lidar_data.json")

    if not os.path.exists(data_file):
        print(f"Data file not found: {data_file}")
        return None, None

    with open(metadata_file, 'r') as f:
        metadata = json.load(f)

    with open(data_file, 'r') as f:
        data = json.load(f)

    return metadata, data

def plot_frame(data, frame_idx=0):
    """Plot a single frame of LIDAR data"""
    if frame_idx >= len(data):
        print(f"Frame {frame_idx} not available")
        return

    frame = data[frame_idx]
    x_coords = frame['x']
    z_coords = frame['z']

    plt.figure(figsize=(10, 6))
    plt.scatter(x_coords, z_coords, s=5, alpha=0.7)
    plt.xlabel('X (mm)')
    plt.ylabel('Z (mm)')
    plt.title(f'LIDAR Data - Frame {frame_idx}')
    plt.grid(True)
    plt.axis('equal')
    plt.show()

def plot_all_frames(data, max_frames=10):
    """Plot multiple frames"""
    frames_to_plot = min(max_frames, len(data))

    fig, axes = plt.subplots(2, 5, figsize=(20, 8))
    axes = axes.flatten()

    for i in range(frames_to_plot):
        frame = data[i]
        x_coords = frame['x']
        z_coords = frame['z']

        axes[i].scatter(x_coords, z_coords, s=3, alpha=0.7)
        axes[i].set_title(f'Frame {i}')
        axes[i].grid(True)
        axes[i].set_aspect('equal')

    plt.tight_layout()
    plt.show()

def main():
    metadata, data = load_data()

    plot_frame(data, 0)

    plot_all_frames(data)


if __name__ == "__main__":
    main()