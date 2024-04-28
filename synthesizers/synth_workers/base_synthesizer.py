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
Implements `Base Synthesizer` which defines an interface across all `Synthesizer Workers`.
"""

from __future__ import (
    annotations,
)  # allowing future references -> return class under which return value is returned
from abc import abstractmethod
from typing import List, Dict


class BaseSynthesizer:  # pylint: disable=too-few-public-methods
    """
    Implements `Base Synthesizer` which defines an interface across all `Synthesizer Workers`.

    Attributes:
        __name__(str): Defines name of the <__call__> magic method, which Omniverse Replicator
            uses to register a method. It has to return synthesizer name, the same name as is used in the config.
        scenario_owner (str): Defines the name of owning `Scenario`.
    """

    def __init__(self, class_name: str, scenario_owner: str) -> None:
        self.__name__ = class_name
        self.scenario_owner = scenario_owner

    @abstractmethod
    def __call__(self, camera_setup: List[str]) -> None:
        """
        With this magic function, a command is executed.

        Args:
            camera_setup (List[str]): List of camera primitive paths in for the camera setup. It can contain more than
                one camera, e.g. stereo camera or more complicated camera rigs.
        """

    @abstractmethod
    def get_prims(self) -> List[str]:
        """
        Returns paths in stage to `Synthesizer's` created prims.

        Returns:
            List[str]: List of stage prim paths.
        """

    @abstractmethod
    def register_synthesizers_prims(self, synthesizer_workers: Dict[str, BaseSynthesizer]) -> None:
        """
        Allows an access to other `Synthesizer's` prims if needed.

        Args:
            synthesizer_workers (Dict[str, BaseSynthesizer]): Dict of all Synthesizers.
        Returns (None):
        """
