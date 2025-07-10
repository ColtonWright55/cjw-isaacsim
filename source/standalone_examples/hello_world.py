# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import math


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

# from omni.importer.mesh_importer import MeshImporter
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade

from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.viewports import set_camera_view

from common import set_drive_parameters



# Create simulation world
world = World()
world.scene.add_default_ground_plane()
assets_root_path = get_assets_root_path()
# asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
# add_reference_to_stage(usd_path=asset_path, prim_path="/World/Fancy_Robot")
# jetbot_robot = world.scene.add(Robot(prim_path="/World/Fancy_Robot", name="fancy_robot"))
# Add a cube
# cube = world.scene.add(
#     DynamicCuboid(
#         prim_path="/World/random_cube",
#         name="fancy_cube",
#         position=np.array([0, 1, 0.25]),
#         scale=np.array([0.5, 0.5, 0.5]),
#         color=np.array([0.0, 0.0, 1.0])
#     )
# )
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
set_camera_view(eye=[.4, -3.00, 3.00], target=[0.0, 0.0, 0.0], camera_prim_path="/OmniverseKit_Persp")



status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
import_config.merge_fixed_joints = False
import_config.fix_base = True
import_config.make_default_prim = True
import_config.create_physics_scene = True
omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path= "my_assets/agf1/urdf/agf1.urdf",
    import_config=import_config,
)
# stage = omni.usd.get_context().get_stage()
# agf1_prim = stage.GetPrimAtPath("/agf1")









# Configuration for dynamic mesh loading
import random
import os

# List of available mesh files (you can modify this list)
available_meshes = [
    "my_assets/agf1/meshes/Agility_Forge_Frame.obj",
    "my_assets/agf1/meshes/blenderAgF.obj",
    "my_assets/agf1/meshes/scanCONTROL LLT29xx-10.obj"
]

# Randomly select a mesh for this run (or you can set a specific one)
selected_mesh = random.choice(available_meshes)
print(f"Loading mesh: {selected_mesh}")

# Add a dynamic mesh that can rotate about any axis
# Import the selected mesh file using USD reference
add_reference_to_stage(
    usd_path=selected_mesh,
    prim_path="/World/dynamic_mesh"
)

# Get the mesh prim for manipulation
stage = omni.usd.get_context().get_stage()
dynamic_mesh_prim = stage.GetPrimAtPath("/World/dynamic_mesh")
add_reference_to_stage(
    usd_path=selected_mesh,
    prim_path="/World/dynamic_mesh"
)
# Reset the simulation
world.reset()


while True:
    world.step(render=True)
    # kit.update()

omni.kit.app.get_app().print_and_log("Hello World!")

kit.close()  # Cleanup application
