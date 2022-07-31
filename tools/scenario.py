"""
Implements `Scenario` class.
"""

from typing import Any, Generator, Tuple, List
from omegaconf import DictConfig
from synthesizers.master_synthesizer import MasterSynthesizer
from tools.isaac_sim import IsaacSimApp
from metron_shared import param_validators as param_val
from tools.single_camera import SingleCamera
from miscellaneous.metron_ai_ardagen_utils import instantiate_from_hydra_config, HydraInstantiateConversion


class Scenario:
    """
    Implements `Scenario` class which is a configuration description what and how should be rendered.

    Attributes:
        master_synthesizer (Optinal[MasterSynthesizer]): `Master Synthesizer` instance containing all `Synthesizer`
            instances to be executed in the given scenario.
        isaac_sim (IsaacSimApp): Reference to Isaac Sim wrapper.
        scenario_name (str): Name of the scenario.
        scenario_dict_config (DictConfig): Hydra's configuration of given scenario's Synthesizers.
        frames_number (int): Number of images which are generated for given scenario.
    """

    def __init__(self, isaac_sim: IsaacSimApp, scenario_name: str, scenario_dict_config: DictConfig) -> None:
        """
        Init.

        Args:
            isaac_sim (IsaacSimApp): Reference to Isaac Sim wrapper.
            scenario_name (str): Name of the scenario.
            scenario_dict_config (DictConfig): Hydra's configuration of given scenario's `Synthesizers`.
        """
        param_val.check_type(isaac_sim, IsaacSimApp)
        param_val.check_type(scenario_name, str)
        param_val.check_type(scenario_dict_config, DictConfig)
        param_val.check_type(scenario_dict_config.frames_number, int)
        param_val.check_parameter_value_in_range(scenario_dict_config.frames_number, 1, 1e10)  # hardcoded value

        self.master_synthesizer = None
        self.isaac_sim = isaac_sim
        self.scenario_name = scenario_name
        self.scenario_dict_config = scenario_dict_config
        self.frames_number = scenario_dict_config.frames_number

    def prepare(self) -> None:
        """
        Prepares the scenario, which means to instantiate all `Synthesizers` encapsulated in `Master Synthesizer`.
        """
        self.master_synthesizer = MasterSynthesizer(self.isaac_sim, self.scenario_dict_config.synthesizer_workers)

    def get_cameras(self) -> Generator[Tuple[List[Any], List[Any]], None, None]:
        """
        Collects all camera setups defined for the scenario. Camera setup means, that in one setup there could be more
        cameras (e.g. stereo camera setup)

        Yields:
            Generator[Tuple[List[Any], List[Any]], None, None]: Camera setup given by list of camera stage paths and
                list of corresponding camera render products.
        """
        for camera_conf_name in self.scenario_dict_config.cameras.keys():
            camera: SingleCamera = instantiate_from_hydra_config(
                self.scenario_dict_config.cameras[camera_conf_name], HydraInstantiateConversion.partial
            )
            yield from camera.get_cameras()
