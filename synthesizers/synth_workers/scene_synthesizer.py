"""
Defines *Scene Synthesizer* class which is responsible for scene loading.
"""

from typing import Any, List
from omni.isaac.core.utils.nucleus import get_assets_root_path
from metron_shared import param_validators as param_val
from .base_synthesizer import BaseSynthesizer


class SceneSynthesizer(BaseSynthesizer):  # pylint: disable=too-few-public-methods
    """
    Defines *Scene Synthesizer* class which is responsible for scene loading.

    Attributes:
            __name__(str): Defines name of the <__call__> magic method, which Omniverse Replicator
                uses to register a method. It returns `Synthesizer's` name as defined in the config.
            scene_path (str): Path of a USD scene to be loaded inside OV Nucleus.
            scene_node (og.Node, but can't be used as annotation): Scene stage node.
    """

    __name__ = "scene_synthesizer"

    def __init__(self, scene_path: str) -> None:
        """
        Init.

        Args:
            scene_path (str): Path of a USD scene to be loaded inside OV Nucleus.
        """
        param_val.check_type(scene_path, str)

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

        self.scene_node = rep.create.from_usd(nucleus_root_path + self.scene_path)

    def __call__(self, camera_setup: List[str]) -> None:
        """
        Called by Replicator to make changes in the scene.

        Args:
            camera_setup (List[str]): List of camera primitive paths in for the camera setup. It can contain more than
                one camera, e.g. stereo camera or more complicated camera rigs.
        """
