# This file is part of the Metron AI ArDaGen (https://github.com/OndrejSzekely/metron_ai_ardagen).
# Copyright (c) 2023-2024 Ondrej Szekely.
#
# This program is free software: you can redistribute it and/or modify it under the terms of the
# GNU General Public License as published by the Free Software Foundation, version 3. This program
# is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
# Public License for more details. You should have received a copy of the GNU General Public
# License along with this program. If not, see <http://www.gnu.org/licenses/>.

"""
Defines *NVIDIA Scene Synthesizer* class which is responsible for scene loading from NVIDIA assets.
"""

from typing import Any, List, Dict
from omni.isaac.core.utils.nucleus import get_nvidia_asset_root_path
from metron_shared import param_validators as param_val
from .base_synthesizer import BaseSynthesizer


class NVIDIASceneSynthesizer(BaseSynthesizer):  # pylint: disable=too-few-public-methods
    """
    Defines *NVIDIA Scene Synthesizer* class which is responsible for scene loading from NVIDIA assets.

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

        nucleus_root_path = get_nvidia_asset_root_path()
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
