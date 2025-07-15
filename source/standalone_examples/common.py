import os

import carb.tokens
import omni
from pxr import PhysxSchema, UsdGeom, UsdPhysics
import numpy as np


def set_drive_parameters(drive, target_type, target_value, stiffness=None, damping=None, max_force=None):
    """Enable velocity drive for a given joint"""

    if target_type == "position":
        if not drive.GetTargetPositionAttr():
            drive.CreateTargetPositionAttr(target_value)
        else:
            drive.GetTargetPositionAttr().Set(target_value)
    elif target_type == "velocity":
        if not drive.GetTargetVelocityAttr():
            drive.CreateTargetVelocityAttr(target_value)
        else:
            drive.GetTargetVelocityAttr().Set(target_value)

    if stiffness is not None:
        if not drive.GetStiffnessAttr():
            drive.CreateStiffnessAttr(stiffness)
        else:
            drive.GetStiffnessAttr().Set(stiffness)

    if damping is not None:
        if not drive.GetDampingAttr():
            drive.CreateDampingAttr(damping)
        else:
            drive.GetDampingAttr().Set(damping)

    if max_force is not None:
        if not drive.GetMaxForceAttr():
            drive.CreateMaxForceAttr(max_force)
        else:
            drive.GetMaxForceAttr().Set(max_force)



def convert_lidar_to_xz(lidar, depth, zenith, azimuth, horizontal_resolution=1280):
    """

    """
    depth = np.array(depth)
    zenith = np.array(zenith)
    azimuth = np.array(azimuth)

    maxdepth = lidar.GetMaxRangeAttr().Get()

    depth = depth*maxdepth/65535.0*1000.0 # Depth in mm

    # print(f"Depth: {depth.shape}")
    # print(f"Depth col 0: {depth[0:10, 0]}")
    # print(f"Depth col 1: {depth[0:10, 1]}")
    # print(f"Depth col 2: {depth[0:10, 2]}")

    # print(f"Zenith: {zenith.shape}")
    # print(f"Azimuth: {azimuth.shape}")

    x_coords = depth[:, 0] * np.sin(azimuth)  # Left-right distance in mm
    z_coords = depth[:, 0] * np.cos(azimuth)  # Z-axis distance in mm

    # Stack x and z coordinates
    data = (x_coords, z_coords)
    return data

def _get_info_function(lidar, _li, lidarPath, val=False):
    if not lidar:
        return
    depth = _li.get_depth_data(lidarPath)
    zenith = _li.get_zenith_data(lidarPath)
    azimuth = _li.get_azimuth_data(lidarPath)

    data = convert_lidar_to_xz(lidar, depth, zenith, azimuth)
    return data
