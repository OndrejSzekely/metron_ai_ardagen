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
Defines *Dummy Synthesizer* class which is responsible for scene loading.
"""

from typing import List, Dict
from .base_synthesizer import BaseSynthesizer


class DummySynthesizer(BaseSynthesizer):  # pylint: disable=too-few-public-methods
    """
    Defines *Dummy Synthesizer* class which is responsible for scene loading.

    Attributes:
            __name__(str): Defines name of the <__call__> magic method, which Omniverse Replicator
                uses to register a method. It returns `Synthesizer's` name as defined in the config.
    """

    _box_primitive_path = "SM_CardBoxA_3"

    def __init__(self, class_name: str, scenario_owner: str) -> None:  # pylint: disable=useless-super-delegation
        super().__init__(class_name, scenario_owner)

    def __call__(self, camera_setup: List[str]) -> None:
        """
        Called by Replicator to make changes in the scene.

        Args:
            camera_setup (List[str]): List of camera primitive paths in for the camera setup. It can contain more than
                one camera, e.g. stereo camera or more complicated camera rigs.
        """
        # Isaac Sim app has to be created before modules can be imported, so called in here.
        import omni.replicator.core as rep  # pylint: disable=import-outside-toplevel

        box = rep.get.prims(path_pattern=self._box_primitive_path)
        with box:
            rep.modify.semantics([("class", "box")])
            rep.modify.visibility(rep.distribution.choice([True, False]))

    def get_prims(self) -> List[str]:
        """
        Returns paths in stage to `Synthesizer's` created prims.

        Returns:
            List[str]: List of stage prim paths.
        """
        return [self._box_primitive_path]

    def register_synthesizers_prims(self, synthesizer_workers: Dict[str, BaseSynthesizer]) -> None:
        """
        Allows an access to other `Synthesizer's` prims if needed.

        Returns (None):
        """
