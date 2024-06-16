# This file is part of the Metron AI ArDaGen (https://github.com/OndrejSzekely/metron_ai_ardagen).
# Copyright (c) 2023 Ondrej Szekely.
#
# This program is free software: you can redistribute it and/or modify it under the terms of the
# GNU General Public License as published by the Free Software Foundation, version 3. This program
# is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
# Public License for more details. You should have received a copy of the GNU General Public
# License along with this program. If not, see <http://www.gnu.org/licenses/>.

"""
Defines *Light Synthesizer* class which is responsible for one light source synthesis.
"""

from typing import Dict, List
from metron_shared import param_validators as param_val
from .base_synthesizer import BaseSynthesizer


class LightSynthesizer(BaseSynthesizer):  # pylint: disable=too-few-public-methods
    """
    Defines *Light Synthesizer* class which is responsible for one light source synthesis.

    Attributes:
        _stage_light_path (str): Prim path of the created light.
        _light_node (og.Node): Light primitive as a OmniGraph Node representation.
        position (List[float]): Light position in the scene given by XYZ coordinates.
        rotation (List[float]): Euler angles in degrees in XYZ order.
        scale (List[float]): Scaling factors for XYZ axes.
        light_type (str): Light type. Select from "cylinder", "disk", "distant", "dome", "rect", "sphere".
    """

    def __init__(  # pylint: disable=too-many-arguments
        self,
        class_name: str,
        scenario_owner: str,
        position: List[float],
        rotation: List[float],
        scale: List[float],
        light_type: str,
    ) -> None:
        """

        Args:
            class_name (str): Synthesizer name given by user in the config.
            scenario_owner (str): Name of owning Scenario.
            position (List[float]): Light position in the scene given by XYZ coordinates.
            rotation (List[float]): Euler angles in degrees in XYZ order.
            scale (List[float]): Scaling factors for XYZ axes.
            light_type (str): Light type. Select from "cylinder", "disk", "distant", "dome", "rect", "sphere".
        """
        import omni.usd  # pylint: disable=import-outside-toplevel
        import omni.replicator.core as rep  # pylint: disable=import-outside-toplevel

        param_val.check_type(class_name, str)
        param_val.check_type(scenario_owner, str)
        param_val.check_type(position, List[float])
        param_val.check_type(rotation, List[float])
        param_val.check_type(scale, List[float])
        param_val.check_type(light_type, str)

        super().__init__(class_name, scenario_owner)

        self.position = position
        self.rotation = rotation
        self.scale = scale
        self.light_type = light_type

        light_node = rep.create.light(
            position=self.position, rotation=self.rotation, scale=self.scale, light_type=self.light_type
        )
        self._light_node = light_node.node.get_prim_path()
        stage = omni.usd.get_context().get_stage()
        self._stage_light_path = (
            stage.GetPrimAtPath(self._light_node).GetRelationship("inputs:primsIn").GetTargets()[0].pathString
        )

    def __call__(self, camera_setup: List[str]) -> None:
        """
        Called by Replicator to make changes in the scene.

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
        return [self._stage_light_path]

    def register_synthesizers_prims(self, synthesizer_workers: Dict[str, BaseSynthesizer]) -> None:
        """
        Allows an access to other `Synthesizer's` prims if needed.

        Returns (None):
        """
