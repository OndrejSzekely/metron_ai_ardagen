"""
Defines *Ground Synthesizer* class which is responsible for ground plane synthesis.
"""

from typing import List, Dict
from .base_synthesizer import BaseSynthesizer
from metron_shared import param_validators as param_val


class GroundSynthesizer(BaseSynthesizer):  # pylint: disable=too-few-public-methods
    """
    Defines *Ground Synthesizer* class which is responsible for ground plane synthesis.

    Attributes:
        stage (Any): Current Isaac Sim stage.
        stage_plane_path (str): Prim path of the created ground.
        plane_node (og.Node): Plane primitive as a OmniGraph Node representation.
        materials_list (List[str]): List of material Nucelus paths.
    """

    def __init__(
        self,
        class_name: str,
        scenario_owner: str,
        position: List[int],
        semantics: str,
        materials: Dict[str, List[str]],
        scale: List[float],
    ) -> None:
        """

        Args:
            class_name (str): Synthesizer name given by user in the config.
            scenario_owner (str): Name of owning Scenario.
            position (List[int]): Location of the ground - X, Y, Z coordinates.
            semantics (str): Semantic class for the Synthesizer's primitives.
            materials (Dict[str, List[str]]): Dictionary of materials pool, from which a ramdom selection is taken.
            scale (List[float]): X, Y, Z ground scale.
        """
        # Isaac Sim app has to be created before modules can be imported, so called in here.
        import omni.replicator.core as rep  # pylint: disable=import-outside-toplevel
        import omni.usd  # pylint: disable=import-outside-toplevel

        param_val.check_type(class_name, str)
        param_val.check_type(scenario_owner, str)
        param_val.check_type(position, List[float])
        param_val.check_type(semantics, str)
        param_val.check_type(materials, Dict[str, List[str]])
        param_val.check_type(scale, List[float])
        param_val.check_length_of_list(position, 3)
        param_val.check_length_of_list(scale, 3)

        super(GroundSynthesizer, self).__init__(class_name, scenario_owner)

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

    def get_prims(self) -> List[str]:
        """
        Returns paths in stage to `Synthesizer's` created prims.

        Returns:
            List[str]: List of stage prim paths.
        """
        return [self.stage_plane_path]

    def register_synthesizers_prims(self, synthesizer_workers: Dict[str, BaseSynthesizer]) -> None:
        """
        Allows an access to other `Synthesizer's` prims if needed.

        Args:
            synthesizer_workers (Dict[str, BaseSynthesizer]): Dict of all Synthesizers.
        Returns (None):
        """
