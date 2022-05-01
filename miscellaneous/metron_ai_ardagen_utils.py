"""
This file contains small domain-free functions specific to GRE2G.
"""


import hydra
from metron_shared.config.config import GetHydraConfig
from miscellaneous.metron_ai_argagen_config_schema import MetronAIArDaGenConfigSchema
from tools.isaac_sim import IsaacSimApp


@GetHydraConfig
def instantiate_isaac_sim(hydra_config: MetronAIArDaGenConfigSchema) -> IsaacSimApp:
    """
    Instantiates `Isaac Sim App`.

    Args:
        hydra_config (GRE2GConfigSchema): GRE2G configuration parameters provided by Hydra's config.

    Returns (IsaacSimApp): Instantiated `IsaacSimApp`.
    """
    return hydra.utils.instantiate(hydra_config.isaac_sim)
