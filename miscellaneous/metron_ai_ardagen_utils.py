"""
This file contains small domain-free functions specific to Metron AI ArDaGen.
"""


from enum import Enum
from typing import Any
import hydra
from omegaconf import DictConfig


class HydraInstantiateConversion(Enum):
    """
    Defines Hydra's instantiation conversion options. Basically how to handle list and dict like objects,
    whether they are represented via OmegaConf structs or Python structs.
    """

    NO_CONVERSION = "none"
    PARTIAL = "partial"
    ALL = "all"


def instantiate_from_hydra_config(
    hydra_object_config: DictConfig, conversion: HydraInstantiateConversion = HydraInstantiateConversion.NO_CONVERSION
) -> Any:
    """
    Instantiates object from Hydra object config <hydra_object_config>. It has to contain <_target_> attribute.

    Args:
        hydra_object_config (DictConfig): Object's Hydra config DictConfig.
        conversion
        conversion (HydraInstantiateConversion): Defined how non-primitive values in OmegaConf are handled.
            See https://hydra.cc/docs/advanced/instantiate_objects/overview/#parameter-conversion-strategies.

    Returns (Any): Instantiated object.
    """
    return hydra.utils.instantiate(hydra_object_config, _convert_=conversion.value)
