"""
This file contains small domain-free functions specific to GRE2G.
"""


from typing import Any
import hydra
from omegaconf import DictConfig


def instantiate_from_hydra_config(hydra_object_config: DictConfig) -> Any:
    """
    Instantiates object from Hydra object config <hydra_object_config>. It has to contain <_target_> attribute.

    Args:
        hydra_object_config (DictConfig): Object's Hydra config DictConfig.

    Returns (Any): Instantiated object.
    """
    return hydra.utils.instantiate(hydra_object_config)
