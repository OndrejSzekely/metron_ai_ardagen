"""
Defines *Physics Synthesizer* class which is responsible for ground synthesis.
"""


import omni
from pxr import Gf, Sdf, UsdPhysics, PhysxSchema, PhysicsSchemaTools
from .base_synthesizer import BaseSynthesizer


class PhysicsSynthesizer(BaseSynthesizer):  # pylint: disable=too-few-public-methods
    """
    Defines *Physics Synthesizer* class which is responsible for physics synthesis.
    """

    def __init__(self) -> None:
        stage = omni.usd.get_context().get_stage()
        physics_scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))

        # set gravity vector
        physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        physics_scene.CreateGravityMagnitudeAttr().Set(981.0)

        # set CPU physics and TGS solver
        PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/World/physicsScene"))
        physx_scene_api = PhysxSchema.PhysxSceneAPI.Get(stage, "/World/physicsScene")
        physx_scene_api.CreateEnableCCDAttr(True)
        physx_scene_api.CreateEnableStabilizationAttr(True)
        physx_scene_api.CreateEnableGPUDynamicsAttr(False)
        physx_scene_api.CreateBroadphaseTypeAttr("MBP")
        physx_scene_api.CreateSolverTypeAttr("TGS")

        # add ground plane
        PhysicsSchemaTools.addGroundPlane(stage, "/World/groundPlane", "Z", 400, Gf.Vec3f(0, 0, 0), Gf.Vec3f(1.0))

    def __call__(self) -> None:
        """
        Nothing to do.
        """
        ...