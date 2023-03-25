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
Defines *OV Assets Synthesizer* class which is responsible for providing Omniverse 3D assets.
"""

from typing import List, Dict
import numpy as np
import metron_shared.param_validators as param_val
from ..base_synthesizer import BaseSynthesizer


class OVAssetsSynthesizer(BaseSynthesizer):  # pylint: disable=too-few-public-methods
    """
    Defines *OV Assets Synthesizer* class which is responsible for providing Omniverse 3D assets.

    !!!!! EACH ASSET SYNTHESIZER HAS TO IMPLEMENT <assets_num_to_generate> INPUT ARGUMENT IN <__init__> interface!!!

    Attributes:
        assets_num_to_generate (int): Number of assets to add to be added to the scene.
        _stage (Any): Current Isaac Sim stage.
        _created_assets (List[og.Node]): List of added prims to the scene from the USD paths pool.
        _created_assets_paths (List[str]): List of assets prim paths.
    """

    def __init__(self, class_name: str, scenario_owner: str, usds: List[str], assets_num_to_generate: int) -> None:
        """

        Args:
            class_name (str): Synthesizer name given by user in the config.
            scenario_owner (str): Name of owning Scenario.
            usds (List[str]): List of USD assets pool, from which assets added to the scene are sampled.
            assets_num_to_generate (int): Number of assets to add to the scene from the pool.
        """
        # Isaac Sim app has to be created before modules can be imported, so called in here.
        import omni.replicator.core as rep  # pylint: disable=import-outside-toplevel
        import omni.usd  # pylint: disable=import-outside-toplevel

        param_val.check_type(class_name, str)
        param_val.check_type(scenario_owner, str)
        param_val.check_type(usds, List[str])
        param_val.check_type(assets_num_to_generate, int)
        param_val.check_parameter_value_in_range(
            assets_num_to_generate, 1, 1e4
        )  # hardcoded value, no reason to go over 1e4. FIXME later.

        super().__init__(class_name, scenario_owner)

        usds = np.array(usds)
        self.assets_num_to_generate = assets_num_to_generate

        self._stage = omni.usd.get_context().get_stage()
        self._created_assets = []
        self._created_assets_paths = []
        taken_assets_inds = np.random.randint(0, len(usds), self.assets_num_to_generate, np.int16)
        for taken_asset_ind in taken_assets_inds:
            asset_node = rep.create.from_usd(usds[taken_asset_ind])
            self._created_assets.append(asset_node)
            stage_asset_path = (
                self._stage.GetPrimAtPath(asset_node.node.get_prim_path())
                .GetRelationship("inputs:prims")
                .GetTargets()[0]
                .pathString
            )
            self._created_assets_paths.append(stage_asset_path)

    def __call__(self, camera_setup: List[str]) -> None:
        """
        Called by Replicator to make changes in the scene.

        Args:
            camera_setup (List[str]): List of camera primitive paths in for the camera setup. It can contain more than
                one camera, e.g. stereo camera or more complicated camera rigs.
        """
        param_val.check_type(camera_setup, List[str])

    def get_prims(self) -> List[str]:
        """
        Returns paths in stage to `Synthesizer's` created prims.

        Returns:
            List[str]: List of stage prim paths.
        """
        return self._created_assets_paths

    def register_synthesizers_prims(self, synthesizer_workers: Dict[str, BaseSynthesizer]) -> None:
        """
        Allows an access to other `Synthesizer's` prims if needed.

        Args:
            synthesizer_workers (Dict[str, BaseSynthesizer]): Dict of all Synthesizers.
        Returns (None):
        """
        param_val.check_type(synthesizer_workers, Dict[str, BaseSynthesizer])
