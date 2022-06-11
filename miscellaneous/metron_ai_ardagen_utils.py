"""
This file contains small domain-free functions specific to GRE2G.
"""


import hydra
from omegaconf import DictConfig
from metron_shared.config.config import GetHydraConfig
from tools.isaac_sim import IsaacSimApp


@GetHydraConfig
def instantiate_isaac_sim(hydra_config: DictConfig) -> IsaacSimApp:
    """
    Instantiates `Isaac Sim App`.

    Args:
        hydra_config (DictConfig): GRE2G configuration parameters provided by Hydra's config.

    Returns (IsaacSimApp): Instantiated `IsaacSimApp`.
    """
    return hydra.utils.instantiate(hydra_config.isaac_sim)
