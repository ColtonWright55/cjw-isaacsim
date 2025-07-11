#!/usr/bin/env python3
"""
Simple LIDAR data visualization from JSON
No NVIDIA dependencies
"""

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

def plot_time_series(data):
    """Plot statistics over time"""
    timestamps = [frame['timestamp'] for frame in data]
    x_ranges = [max(frame['x']) - min(frame['x']) for frame in data]
    z_ranges = [max(frame['z']) - min(frame['z']) for frame in data]
    point_counts = [len(frame['x']) for frame in data]

    plt.figure(figsize=(15, 5))

    plt.subplot(1, 3, 1)
    plt.plot(timestamps, point_counts)
    plt.xlabel('Frame')
    plt.ylabel('Number of Points')
    plt.title('Point Count Over Time')
    plt.grid(True)

    plt.subplot(1, 3, 2)
    plt.plot(timestamps, x_ranges)
    plt.xlabel('Frame')
    plt.ylabel('X Range (mm)')
    plt.title('X Range Over Time')
    plt.grid(True)

    plt.subplot(1, 3, 3)
    plt.plot(timestamps, z_ranges)
    plt.xlabel('Frame')
    plt.ylabel('Z Range (mm)')
    plt.title('Z Range Over Time')
    plt.grid(True)

    plt.tight_layout()
    plt.show()

def main():
    """Main function"""
    print("Loading LIDAR data...")
    metadata, data = load_data()

    if data is None:
        return

    print(f"Loaded {len(data)} frames")

    # Plot first frame
    plot_frame(data, 0)

    # Plot multiple frames
    plot_all_frames(data)

    # Plot time series
    plot_time_series(data)

if __name__ == "__main__":
    main()