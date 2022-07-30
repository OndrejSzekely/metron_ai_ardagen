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
    """

    def __init__(self, headless: bool) -> None:
        param_val.check_type(headless, bool)

        self.app = SimulationApp({"headless": headless})

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
        self.close()
