# This file is part of the Metron AI ArDaGen (https://github.com/OndrejSzekely/metron_ai_ardagen).
# Copyright (c) 2023 Ondrej Szekely.
#
# This program is free software: you can redistribute it and/or modify it under the terms of the
# GNU General Public License as published by the Free Software Foundation, version 3. This program
# is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
# Public License for more details. You should have received a copy of the GNU General Public
# License along with this program. If not, see <http://www.gnu.org/licenses/>.

"""
The script runs *Metron AI ArDaGen*.
"""


import os
import sys
import hydra
from omegaconf import DictConfig
from tools.isaac_sim import IsaacSimApp
from tools.scenarios_manager import ScenariosManager
from tools.replicator import OVReplicator
from tools.writer import OVWriter
from miscellaneous import metron_ai_ardagen_utils as ardagen_utils
from metron_shared.config.instantiate import instantiate_from_hydra_config
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
    # Has to be called after loading Isaac Sim app.
    ardagen_utils.load_ov_extension(
        hydra_config.settings.ardagen_extension.fs_path, hydra_config.settings.ardagen_extension.name
    )
    scenarios_manager = ScenariosManager(hydra_config.scenarios, isaac_sim)
    ov_writer: OVWriter = instantiate_from_hydra_config(hydra_config.writer)  # pylint: disable=no-value-for-parameter
    ov_replicator = OVReplicator(isaac_sim, scenarios_manager, ov_writer)
    ov_replicator()
    isaac_sim.close()


if __name__ == "__main__":
    main()  # pylint: disable=no-value-for-parameter
    sys.exit(os.EX_OK)
