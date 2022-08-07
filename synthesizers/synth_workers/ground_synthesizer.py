"""
Defines *Ground Synthesizer* class which is responsible for ground plane synthesis.
"""

from typing import List, Dict
from .base_synthesizer import BaseSynthesizer
from metron_shared import param_validators as param_val


class GroundSynthesizer(BaseSynthesizer):  # pylint: disable=too-few-public-methods
    """
    Defines *Ground Synthesizer* class which is responsible for ground plane synthesis.
    """

    __name__ = "ground_synthesizer"

    def __init__(self, position: List[int], semantics: str, materials: Dict[str, List[str]], scale: float) -> None:
        # Isaac Sim app has to be created before modules can be imported, so called in here.
        import omni.replicator.core as rep  # pylint: disable=import-outside-toplevel
        import omni.usd  # pylint: disable=import-outside-toplevel
        import omni.kit.material.library as mat_lib  # pylint: disable=import-outside-toplevel

        param_val.check_type(position, List[int])
        param_val.check_type(semantics, str)
        param_val.check_type(materials, Dict[str, List[str]])
        param_val.check_type(scale, float)
        param_val.check_length_of_list(position, 3)

        plane_node = rep.create.plane(position, semantics=[("class", semantics)], scale=scale)
        self.stage = omni.usd.get_context().get_stage()
        self.stage_plane_path = (
            self.stage.GetPrimAtPath(plane_node.node.get_prim_path())
            .GetRelationship("inputs:prims")
            .GetTargets()[0]
            .pathString
        )
        self.plane_node = plane_node.node
        self.materials_list = []
        for material_group in materials.values():
            self.materials_list.extend(material_group)

    def __call__(self, camera_setup: List[str]) -> None:
        """
        With this magic function, a command is executed.

        Args:
            camera_setup (List[str]): List of camera primitive paths in for the camera setup. It can contain more than
                one camera, e.g. stereo camera or more complicated camera rigs.
        """
        import omni.replicator.core as rep  # pylint: disable=import-outside-toplevel

        rep.randomizer.materials(
            self.materials_list,
            input_prims=[self.stage_plane_path],
        )
        return self.plane_node
