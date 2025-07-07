# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

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


from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics

from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.viewports import set_camera_view

# Create simulation world
world = World()
world.scene.add_default_ground_plane()
assets_root_path = get_assets_root_path()
asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/Fancy_Robot")
jetbot_robot = world.scene.add(Robot(prim_path="/World/Fancy_Robot", name="fancy_robot"))
# Add a cube
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/random_cube",
        name="fancy_cube",
        position=np.array([0, 0, 1.0]),
        scale=np.array([0.5, 0.5, 0.5]),
        color=np.array([0.0, 0.0, 1.0])
    )
)
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))
lidarPath = "/World/Lidar"
lidar = RangeSensorSchema.Lidar.Define(stage, Sdf.Path(lidarPath))
lidar.CreateHorizontalFovAttr().Set(360.0)
lidar.CreateVerticalFovAttr().Set(10)
lidar.CreateRotationRateAttr().Set(20.0)
lidar.CreateHorizontalResolutionAttr().Set(1.0)
lidar.CreateVerticalResolutionAttr().Set(1.0)
lidar.CreateMinRangeAttr().Set(0.4)
lidar.CreateMaxRangeAttr().Set(100.0)
lidar.CreateHighLodAttr().Set(True)
lidar.CreateDrawPointsAttr().Set(False)
lidar.CreateDrawLinesAttr().Set(False)
lidar.GetRotationRateAttr().Set(0.5)
lidar.GetDrawLinesAttr().Set(True)
lidar.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 0.250))
set_camera_view(eye=[5.00, 5.00, 5.00], target=[0.0, 0.0, 0.0], camera_prim_path="/OmniverseKit_Persp")

# Reset the simulation
world.reset()


for i in range(1000):
    world.step(render=True)
    # kit.update()

omni.kit.app.get_app().print_and_log("Hello World!")

kit.close()  # Cleanup application
