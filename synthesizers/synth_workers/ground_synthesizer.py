"""
Defines *Ground Synthesizer* class which is responsible for ground synthesis.
"""

from typing import List
import numpy as np
from omni.isaac.core.scenes import Scene
from .base_synthesizer import BaseSynthesizer


class GroundSynthesizer(BaseSynthesizer):  # pylint: disable=too-few-public-methods
    """
    Defines *Ground Synthesizer* class which is responsible for ground synthesis.
    """

    def __init__(self) -> None:
        scene = Scene()
        scene.add_ground_plane(
            color=np.array(
                [
                    255,
                    0,
                    0,
                ]
            )
        )

    def __call__(self, camera_setup: List[str]) -> None:
        """
        With this magic function, a command is executed.

        Args:
            camera_setup (List[str]): List of camera primitive paths in for the camera setup. It can contain more than
                one camera, e.g. stereo camera or more complicated camera rigs.
        """
