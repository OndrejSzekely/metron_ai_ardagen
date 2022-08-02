"""
Implements `Base Synthesizer` which defines an interface across all `Synthesizer Workers`.
"""


from abc import ABC, abstractmethod
from typing import List


class BaseSynthesizer(ABC):  # pylint: disable=too-few-public-methods
    """
    Implements `Base Synthesizer` which defines an interface across all `Synthesizer Workers`.

    Attributes:
        __name__(str): Defines name of the <__call__> magic method, which Omniverse Replicator
            uses to register a method. It has to return synthesizer name, the same name as is used in the config.
    """

    @abstractmethod
    def __call__(self, camera_setup: List[str]) -> None:
        """
        With this magic function, a command is executed.

        Args:
            camera_setup (List[str]): List of camera primitive paths in for the camera setup. It can contain more than
                one camera, e.g. stereo camera or more complicated camera rigs.
        """

    @property
    def __name__(self) -> str:
        """
        Defines name of the <__call__> magic method, which Omniverse Replicator uses to register a method.
        It has to return synthesizer name, the same name as is used in the config.

        Returns (str):
        """
        raise NotImplementedError
