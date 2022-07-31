"""
Defines *Development Synthesizer* class which is responsible for scene loading.
"""

from omni.graph.core import Node


class DevelopmentSynthesizer:
    """
    Defines *Development Synthesizer* class which is responsible for scene loading.

    Attributes:
            __name__(str): Defines name of the <__call__> magic method, which Omniverse Replicator
                uses to register a method. It returns `Synthesizer's` name as defined in the config.
    """

    def __init__(self) -> None:
        """
        Init.
        """
        self.__name__: str = "development_synthesizer"

    def __call__(self) -> Node:
        # Isaac Sim app has to be created before modules can be imported, so called in here.
        import omni.replicator.core as rep

        box = rep.get.prims(path_pattern="/Replicator/Ref/SM_CardBoxA_3")
        with box:
            rep.modify.semantics([("class", "box")])
            rep.modify.visibility(rep.distribution.choice([True, False]))
        return box.node
