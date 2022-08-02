"""
Defines *OV Assets Synthesizer* class which is responsible for providing Omniverse 3D assets.
"""

from typing import List
from ..base_synthesizer import BaseSynthesizer


class OVAssetsSynthesizer(BaseSynthesizer):  # pylint: disable=too-few-public-methods
    """
    Defines *OV Assets Synthesizer* class which is responsible for providing Omniverse 3D assets.
    """

    def __init__(self) -> None:
        ...

    def __call__(self, camera_setup: List[str]) -> None:
        """
        Called by Replicator to make changes in the scene.

        Args:
            camera_setup (List[str]): List of camera primitive paths in for the camera setup. It can contain more than
                one camera, e.g. stereo camera or more complicated camera rigs.
        """
