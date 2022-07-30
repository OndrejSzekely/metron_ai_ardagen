"""
The script runs *Metron AI ArDaGen*.
"""


import os
import hydra
from omegaconf import DictConfig
from synthesizers.master_synthesizer import MasterSynthesizer
from tools.isaac_sim import IsaacSimApp
from miscellaneous.metron_ai_ardagen_utils import instantiate_from_hydra_config
from metron_shared.config.config import set_hydra_config


@hydra.main(  # type: ignore[misc]
    config_path=f"{os.path.join(os.getcwd(), 'metron_ai_ardagen', 'conf')}", config_name="metron_ai_ardagen_config"
)
@set_hydra_config
def main(hydra_config: DictConfig) -> None:  # pylint: disable=unused-argument
    """
    Main function of *Metron AI ArDaGen* component.

    Args:
        hydra_config (DictConfig): *Metron AI ArDaGen* configuration parameters
            provided by Hydra's config.

    Returns (None):
    """
    isaac_sim: IsaacSimApp = instantiate_from_hydra_config(
        hydra_config.isaac_sim
    )  # pylint: disable=no-value-for-parameter
    MasterSynthesizer(isaac_sim)

    while True:
        isaac_sim.update()


if __name__ == "__main__":
    main()  # pylint: disable=no-value-for-parameter
