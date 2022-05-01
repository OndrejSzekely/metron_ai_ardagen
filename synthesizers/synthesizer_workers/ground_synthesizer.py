"""
Defines *Ground Synthesizer* class which is responsible for ground synthesis.
"""

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

    def __call__(self) -> None:
        """ """
