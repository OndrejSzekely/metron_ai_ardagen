"""
The module provides *Structured Config Schema* to validate Hydra configs - config parameters and their type.
It does not validate parameter values.
"""


from dataclasses import dataclass
from hydra.core.config_store import ConfigStore


@dataclass
class IsaacSimSchema:
    """
    Main Hydra Config Schema for Isaac Sim.

    Attributes:
        headless (bool):  How to run Omniverse Isaac Sim.
        _target_ (str): Loacation of the corresponding class.
    """

    headless: bool
    _target_: str = "tools.isaac_sim.IsaacSimApp"


@dataclass
class BaseSynthesizerSchema:
    """
    Main Hydra Config Schema for `Base Synthesizer`.

    Attributes:
    """


@dataclass
class PhysicsSynthesizerSchema(BaseSynthesizerSchema):
    """
    Main Hydra Config Schema for `Physics Synthesizer`.

    Attributes:
        enabled (bool): Whether the synthesizer is enabled.
        _target_ (str): Loacation of the corresponding class.
    """

    enabled: bool
    _target_: str = "synthesizers.synthesizer_workers.physics_synthesizer.PhysicsSynthesizer"


@dataclass
class GroundSynthesizerSchema(BaseSynthesizerSchema):
    """
    Main Hydra Config Schema for `Ground Synthesizer`.

    Attributes:
        enabled (bool): Whether the synthesizer is enabled.
        _target_ (str): Loacation of the corresponding class.
    """

    enabled: bool
    _target_: str = "synthesizers.synthesizer_workers.ground_synthesizer.GroundSynthesizer"


@dataclass
class SynthesizerWorkersSchema:
    """
    Main Hydra Config Schema for `Synthesizer Workers`.

    Attributes:
        physics_synthesizer (PhysicsSynthesizerSchema): Physics Synthesizer.
        ground_synthesizer (GroundSynthesizerSchema): Gound Synthesizer.
    """

    physics_synthesizer: PhysicsSynthesizerSchema
    ground_synthesizer: GroundSynthesizerSchema


@dataclass
class MasterSynthesizerSchema:
    """
    Main Hydra Config Schema for `Master Synthesizer`.

    Attributes:
        _target_ (str): Loacation of the corresponding class.
        synthesizer_workers (SynthesizerWorkersSchema): Synthesizers.
    """

    synthesizer_workers: SynthesizerWorkersSchema


@dataclass
class MetronAIArDaGenConfigSchema:
    """
    Main Hydra Config Schema for `Metron AI ArDaGen`.

    Attributes:
        master_synthesizer (MasterSynthesizerSchema):
        isaac_sim (IsaacSimSchema):
    """

    master_synthesizer: MasterSynthesizerSchema
    isaac_sim: IsaacSimSchema


def metron_ai_ardagen_config_schema_registration(cf_instance: ConfigStore) -> None:
    """
    Registers Hydra Config Schema for *Metron AI ArDaGen*.

    Args:
        cf_instance (ConfigStore): ConfigStore instance.

    Returns (None):
    """

    cf_instance.store(name="metron_ai_ardagen_config_schema", node=MetronAIArDaGenConfigSchema)
    cf_instance.store(name="master_synthesizer_schema", node=MasterSynthesizerSchema)
    cf_instance.store(name="isaac_sim_schema", node=IsaacSimSchema)
    cf_instance.store(group="synthesizer_workers", name="physics_synthesizer_schema", node=PhysicsSynthesizerSchema)
    cf_instance.store(group="synthesizer_workers", name="ground_synthesizer_schema", node=GroundSynthesizerSchema)
