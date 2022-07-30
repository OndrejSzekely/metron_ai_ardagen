"""
Implements `Base Synthesizer` which defines an interface across all `Synthesizer Workers`.
"""


from abc import ABC, abstractmethod


class BaseSynthesizer(ABC):  # pylint: disable=too-few-public-methods
    """
    Implements `Base Synthesizer` which defines an interface across all `Synthesizer Workers`.
    """

    @abstractmethod
    def __call__(self) -> None:
        """
        With this magic function, a command is executed.
        """
