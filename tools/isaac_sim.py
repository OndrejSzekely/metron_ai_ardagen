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
Implements Isaac App wrapper class.
"""


from omni.isaac.kit import SimulationApp
from metron_shared import param_validators as param_val


class IsaacSimApp:
    """
    Implements Isaac App wrapper class.

    Attributes:
        app (SimulationApp): Isaac Sim app.
        debug (bool): If `True` OV Replicator won't be executed and allows the user to inspect the generated scene.
    """

    def __init__(self, debug: bool) -> None:
        """

        Args:
            debug (bool): Switch whether to run ArDaGen in debug mode (headful).
        """
        param_val.check_type(debug, bool)

        self.debug = debug

        self.app = SimulationApp({"headless": True ^ debug})  # Non-headless mode for debug only.

    def update(self) -> None:
        """
        Updates Isaac Sim.

        Returns (None):
        """
        self.app.update()  # Render a single frame

    def close(self) -> None:
        """
        Closes Isaac Sim.

        Returns (None):
        """
        self.app.close()
