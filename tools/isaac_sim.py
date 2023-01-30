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
