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
from omni.physx.scripts.physicsUtils import set_or_add_translate_op

from common import set_drive_parameters, _get_info_function
import asyncio
from omni.isaac.core.utils.nucleus import find_nucleus_server
from omni.isaac.core.utils.stage import add_reference_to_stage
import carb
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units


import asyncio
import omni.kit.asset_converter

def progress_callback(current_step: int, total: int):
    # Show progress
    print(f"{current_step} of {total}")


# Convert obj to us
input_asset_path = "/home/cjw/GithubOther/cjw-isaacsim/my_assets/agf1/meshes/part_afp.obj"
output_asset_path = "/home/cjw/GithubOther/cjw-isaacsim/my_assets/agf1/meshes/part_afp.usd"
converter = omni.kit.asset_converter.get_instance()
def progress_callback(current_step: int, total: int):
    # Show progress
    print(f"{current_step} of {total}")
async def convert(input_asset_path, output_asset_path):
    task_manager = converter
    print("cjw start task")
    task = task_manager.create_converter_task(input_asset_path, output_asset_path, progress_callback)
    success = await task.wait_until_finished()
    print("cjw done task")
    if not success:
        detailed_status_code = task.get_status()
        detailed_status_error_string = task.get_error_message()
asyncio.ensure_future(convert(input_asset_path, output_asset_path))
# Create simulation world
world = World()
world.scene.add_default_ground_plane()
assets_root_path = get_assets_root_path()
mesh_prim = "/World/part_afp"

# Import usd
add_reference_to_stage(usd_path=output_asset_path, prim_path="/World/part_afp")

# Get the stage and find the imported mesh
stage = omni.usd.get_context().get_stage()
part_prim = stage.GetPrimAtPath("/World/part_afp")
UsdPhysics.CollisionAPI.Apply(part_prim)
col_mesh_api = UsdPhysics.MeshCollisionAPI.Apply(part_prim)
col_mesh_api.GetApproximationAttr().Set("none")  # Use exact mesh, no approximation
xform = UsdGeom.Xform(part_prim)
for op in xform.GetOrderedXformOps():
    print("Existing XformOp:", op.GetOpName(), op.GetOpType())
    if op.GetOpType() == UsdGeom.XformOp.TypeScale:
        op.Set(Gf.Vec3f(0.001, 0.001, 0.001))
        break


time_step = 0
recording_duration = 10000  # Record for 1000 simulation steps

while time_step < recording_duration or True:
    world.step(render=True)
