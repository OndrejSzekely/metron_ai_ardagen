"""
Defines *Scene Synthesizer* class which is responsible for scene loading.
"""

from typing import Any, List, Dict
from omni.isaac.core.utils.nucleus import get_assets_root_path
from metron_shared import param_validators as param_val
from .base_synthesizer import BaseSynthesizer


class SceneSynthesizer(BaseSynthesizer):  # pylint: disable=too-few-public-methods
    """
    Defines *Scene Synthesizer* class which is responsible for scene loading.

    Attributes:
            _scene_node (og.Node, but can't be used as annotation): Scene stage node.
            scene_path (str): Path of a USD scene to be loaded inside OV Nucleus.
    """

    def __init__(self, class_name: str, scenario_owner: str, scene_path: str) -> None:
        """

        Args:
            class_name (str): Synthesizer name given by user in the config.
            scenario_owner (str): Name of owning Scenario.
            scene_path (str): Path of a USD scene to be loaded inside OV Nucleus.
        """
        param_val.check_type(scene_path, str)
        param_val.check_type(class_name, str)
        param_val.check_type(scenario_owner, str)

        super().__init__(class_name, scenario_owner)

        self.scene_path = scene_path
        self._load_from_nucleus()

    def _load_from_nucleus(self) -> Any:
        """
        Loads the scene from OV Nucleus server.

        Returns:
            og.Node, but can't be used as annotation: Reference to the scene node.
        """
        # Isaac Sim app has to be created before modules can be imported, so called in here.
        import omni.replicator.core as rep  # pylint: disable=import-outside-toplevel

        nucleus_root_path = get_assets_root_path()
        param_val.check_type(nucleus_root_path, str)

        self._scene_node = rep.create.from_usd(nucleus_root_path + self.scene_path)

    def __call__(self, camera_setup: List[str]) -> None:
        """
        With this magic function, a command is executed.

        Args:
            camera_setup (List[str]): List of camera primitive paths in for the camera setup. It can contain more than
                one camera, e.g. stereo camera or more complicated camera rigs.
        """

    def get_prims(self) -> List[str]:
        """
        Returns paths in stage to `Synthesizer's` created prims.

        Returns:
            List[str]: List of stage prim paths.
        """
        return []

    def register_synthesizers_prims(self, synthesizer_workers: Dict[str, BaseSynthesizer]) -> None:
        """
        Allows an access to other `Synthesizer's` prims if needed.

        Args:
            synthesizer_workers (Dict[str, BaseSynthesizer]): Dict of all Synthesizers.
        Returns (None):
        """
        param_val.check_type(synthesizer_workers, Dict[str, BaseSynthesizer])
