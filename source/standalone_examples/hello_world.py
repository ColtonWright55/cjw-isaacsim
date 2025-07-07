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
asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/Fancy_Robot")
jetbot_robot = world.scene.add(Robot(prim_path="/World/Fancy_Robot", name="fancy_robot"))
# Add a cube
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/random_cube",
        name="fancy_cube",
        position=np.array([0, 1, 0.25]),
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





status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
import_config.merge_fixed_joints = False
import_config.fix_base = True
import_config.make_default_prim = True
import_config.create_physics_scene = True
omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path= "my_assets/ur10/urdf/ur10.urdf",
    import_config=import_config,
)
base_prim = stage.GetPrimAtPath("/ur10/base_link")  # adjust for your URDF
# UsdGeom.Xformable(base_prim).AddTranslateOp().Set(Gf.Vec3f(2.0, 0.0, 0.5))
UsdGeom.Xformable(base_prim).MakeMatrixXform().SetTranslate(Gf.Vec3f(2.0, 0.0, 0.5))
# camera_state = ViewportCameraState("/OmniverseKit_Persp")
# camera_state.set_position_world(Gf.Vec3d(2.0, -2.0, 0.5), True)
# camera_state.set_target_world(Gf.Vec3d(0.0, 0.0, 0.0), True)

# stage = omni.usd.get_context().get_stage()
# scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
# scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
# scene.CreateGravityMagnitudeAttr().Set(9.81)

# distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
# distantLight.CreateIntensityAttr(500)

# stage = omni.usd.get_context().get_stage()


# PhysxSchema.PhysxArticulationAPI.Get(stage, "/ur10").CreateSolverPositionIterationCountAttr(64)
# PhysxSchema.PhysxArticulationAPI.Get(stage, "/ur10").CreateSolverVelocityIterationCountAttr(64)

# joint_1 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/ur10/joints/shoulder_pan_joint"), "angular")
# joint_2 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/ur10/joints/shoulder_lift_joint"), "angular")
# joint_3 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/ur10/joints/elbow_joint"), "angular")
# joint_4 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/ur10/joints/wrist_1_joint"), "angular")
# joint_5 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/ur10/joints/wrist_2_joint"), "angular")
# joint_6 = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/ur10/joints/wrist_3_joint"), "angular")

# # Set the drive mode, target, stiffness, damping and max force for each joint
# set_drive_parameters(joint_1, "position", math.degrees(0), math.radians(1e8), math.radians(5e7))
# set_drive_parameters(joint_2, "position", math.degrees(0), math.radians(1e8), math.radians(5e7))
# set_drive_parameters(joint_3, "position", math.degrees(0), math.radians(1e8), math.radians(5e7))
# set_drive_parameters(joint_4, "position", math.degrees(0), math.radians(1e8), math.radians(5e7))
# set_drive_parameters(joint_5, "position", math.degrees(0), math.radians(1e8), math.radians(5e7))
# set_drive_parameters(joint_6, "position", math.degrees(0), math.radians(1e8), math.radians(5e7))





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









# Reset the simulation
world.reset()


for i in range(1000):
    world.step(render=True)
    # kit.update()

omni.kit.app.get_app().print_and_log("Hello World!")

kit.close()  # Cleanup application
