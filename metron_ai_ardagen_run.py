"""
The script runs *Metron AI ArDaGen*.
"""


import os
import hydra
from synthesizers.master_synthesizer import MasterSynthesizer
from miscellaneous.metron_ai_ardagen_utils import instantiate_isaac_sim
from miscellaneous.metron_ai_argagen_config_schema import (
    metron_ai_ardagen_config_schema_registration,
    MetronAIArDaGenConfigSchema,
)
from metron_shared.config.config_schema import create_structured_config_schema
from metron_shared.config.config import set_hydra_config


@create_structured_config_schema(metron_ai_ardagen_config_schema_registration)
@hydra.main(  # type: ignore[misc]
    config_path=f"{os.path.join(os.getcwd(), 'metron_ai_ardagen', 'conf')}", config_name="metron_ai_ardagen_config"
)
@set_hydra_config
def main(hydra_config: MetronAIArDaGenConfigSchema) -> None:  # pylint: disable=unused-argument
    """
    Main function of *Metron AI ArDaGen* component.

    Args:
        hydra_config (MetronAiArDaGenConfigSchema): *Metron AI ArDaGen* configuration parameters
            provided by Hydra's config.

    Returns (None):
    """
    isaac_sim = instantiate_isaac_sim()  # pylint: disable=no-value-for-parameter
    MasterSynthesizer(isaac_sim)

    while True:
        isaac_sim.update()


if __name__ == "__main__":
    main()  # pylint: disable=no-value-for-parameter
