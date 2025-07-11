# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


import math
import os
import json
from datetime import datetime

from isaacsim import SimulationApp

# The most basic usage for creating a simulation app
kit = SimulationApp({"headless": False})

import omni
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.api.robots import Robot
from isaacsim.sensors.physx import _range_sensor
import omni.isaac.RangeSensorSchema as RangeSensorSchema
from omni.kit.viewport.utility.camera_state import ViewportCameraState
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.viewports import set_camera_view

from common import set_drive_parameters, _get_info_function

# Create data directory for LIDAR recordings
data_dir = "_lidar_data"
os.makedirs(data_dir, exist_ok=True)

# LIDAR data storage
lidar_data_buffer = []
lidar_metadata = {
    "horizontal_fov": 45.0,
    "vertical_fov": 0.5,
    "horizontal_resolution": 1280,
    "min_range": 0.0,
    "max_range": 1.0,
    "lidar_position": [0.19, -0.17, 1.246],
    "lidar_rotation": 90.0,
    "recording_start_time": datetime.now().isoformat()
}


# Create simulation world
world = World()
world.scene.add_default_ground_plane()
assets_root_path = get_assets_root_path()


# Setup LIDAR
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))
lidarPath = "/World/Lidar"
lidar = RangeSensorSchema.Lidar.Define(stage, Sdf.Path(lidarPath))
HOZ_FOV = 45.0
HOZ_RES = 1280
lidar.CreateHorizontalFovAttr().Set(HOZ_FOV)
lidar.CreateVerticalFovAttr().Set(0.5)
lidar.CreateHorizontalResolutionAttr().Set(HOZ_FOV/HOZ_RES) # Make sure you get 1280 points
lidar.CreateVerticalResolutionAttr().Set(0) # So there is only one beam
# Rotation rate in Hz, set to 0 for static lidar
lidar.CreateRotationRateAttr().Set(0.0)
# Min and max range for the LIDAR.  This defines the starting and stopping locations for the linetrace
lidar.CreateMinRangeAttr().Set(0.0)
lidar.CreateMaxRangeAttr().Set(0.4)  # Increased range to make beam more visible
lidar.CreateHighLodAttr().Set(True) # High level of detail
lidar.CreateDrawPointsAttr().Set(True)  # Enable point visualization
lidar.CreateDrawLinesAttr().Set(True)   # Enable line visualization
lidar.GetDrawLinesAttr().Set(True)

lidar.AddTranslateOp().Set(Gf.Vec3f(0.19, -0.17, 1.246))
lidar.AddRotateZOp().Set(90.0)
_li = _range_sensor.acquire_lidar_sensor_interface()



# Import robot
status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
import_config.merge_fixed_joints = False
import_config.fix_base = True
import_config.make_default_prim = True
import_config.create_physics_scene = True
import_config.set_collision_from_visuals(True)
import_config.set_convex_decomp(False)
# Show all attributes and their current values
for attr in dir(import_config):
    if not attr.startswith("_"):
        try:
            print(f"{attr}: {getattr(import_config, attr)}")
        except Exception as e:
            print(f"{attr}: <error> {e}")
omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path= "my_assets/agf1/urdf/agf1.urdf",
    import_config=import_config,
)



# Below here is trying to shutoff gravity and make workpiece kinematic

scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
scene.CreateGravityMagnitudeAttr().Set(0)

stage = omni.usd.get_context().get_stage()
workpiece_prim = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/agf1/workpiece"), "angular")
set_drive_parameters(workpiece_prim, "velocity", math.degrees(1), 0, math.radians(1))

# Debug: Inspect the collision geometry that was actually created
workpiece_collision_prim = stage.GetPrimAtPath("/agf1/workpiece")
print(f"Workpiece collision prim: {workpiece_collision_prim.GetPath()}")
UsdPhysics.CollisionAPI.Apply(workpiece_collision_prim)


# Find collision meshes
def find_collision_meshes(prim):
    collision_meshes = []
    for child in prim.GetChildren():
        if child.HasAPI(UsdPhysics.CollisionAPI):
            collision_meshes.append(child)
        collision_meshes.extend(find_collision_meshes(child))
    return collision_meshes

collision_meshes = find_collision_meshes(workpiece_collision_prim)
print(f"Found {len(collision_meshes)} collision meshes")



distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
distantLight.CreateIntensityAttr(500)
set_camera_view(eye=[.4, -3.00, 3.00], target=[0.0, 0.0, 0.0], camera_prim_path="/OmniverseKit_Persp")






# Reset the simulation
world.reset()

time_step = 0
recording_duration = 10000  # Record for 1000 simulation steps

while time_step < recording_duration or True:
    world.step(render=True)

    # Get LIDAR data for debugging and recording
        # Get the latest LIDAR data
    lidar_data = _get_info_function(lidar, _li, lidarPath)
    if lidar_data is not None:
        # Store data with timestamp
        frame_data = {
            "timestamp": time_step,
            "x": lidar_data[0],
            "z": lidar_data[1]
        }
        lidar_data_buffer.append(frame_data)

        # Print some debug info every 10 steps
        if time_step % 10 == 0:
            print(f"Time step: {time_step}")

    _get_info_function(lidar, _li, lidarPath)
    time_step += 1
    # kit.update()

# Convert all numpy arrays to lists for JSON serialization
for frame in lidar_data_buffer:
    frame["x"] = frame["x"].tolist() if hasattr(frame["x"], 'tolist') else frame["x"]
    frame["z"] = frame["z"].tolist() if hasattr(frame["z"], 'tolist') else frame["z"]

# Save LIDAR data to disk
print(f"\nSaving {len(lidar_data_buffer)} frames of LIDAR data...")

# Save metadata
metadata_file = os.path.join(data_dir, "lidar_metadata.json")
with open(metadata_file, 'w') as f:
    json.dump(lidar_metadata, f, indent=2)

# Save data
data_file = os.path.join(data_dir, "lidar_data.json")
with open(data_file, 'w') as f:
    json.dump(lidar_data_buffer, f, indent=2)

print(f"LIDAR data saved to: {data_file}")
print(f"Metadata saved to: {metadata_file}")
print("Run 'python plot_lidar_data.py' to visualize the data")

omni.kit.app.get_app().print_and_log("Hello World!")

kit.close()  # Cleanup application
