"""
Implements `Scenario` class.
"""

from typing import Any, Generator, Tuple, List, Union
from omegaconf import DictConfig
from synthesizers.master_synthesizer import MasterSynthesizer, NullMasterSynthesizer
from tools.isaac_sim import IsaacSimApp
from tools.single_camera import SingleCamera
from metron_shared import param_validators as param_val
from metron_shared.config.instantiate import instantiate_from_hydra_config, HydraInstantiateConversion


class Scenario:
    """
    Implements `Scenario` class which is a configuration description what and how should be rendered.

    Attributes:
        master_synthesizer (Union[NullMasterSynthesizer, MasterSynthesizer]): `Master Synthesizer` instance
            containing all `Synthesizer` instances to be executed in the given scenario.
        isaac_sim (IsaacSimApp): Reference to Isaac Sim wrapper. Till MasterSynthesizer is not instantiated, it stores
            NullMasterSynthesizer class instance as a Null Object duck-type instance.
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
        param_val.check_type(scenario_dict_config.frames_readout_offset, int)
        param_val.check_parameter_value_in_range(scenario_dict_config.frames_number, 1, 1e10)  # hardcoded value
        param_val.check_parameter_value_in_range(scenario_dict_config.frames_readout_offset, 1, 1e10)  # hardcoded value

        self.master_synthesizer: Union[NullMasterSynthesizer, MasterSynthesizer] = NullMasterSynthesizer()
        self.isaac_sim = isaac_sim
        self.scenario_name = scenario_name
        self.scenario_dict_config = scenario_dict_config
        self.frames_number = scenario_dict_config.frames_number
        self.frames_readout_offset = scenario_dict_config.frames_readout_offset

    def prepare(self, scenario_name: str) -> None:
        """
        Prepares the scenario, which means to instantiate all `Synthesizers` encapsulated in `Master Synthesizer`.

        Args:
            scenario_name (str): Corresponding scenario name, as defined in the Hydra config.
        """
        param_val.check_type(scenario_name, str)

        self.master_synthesizer = MasterSynthesizer(
            self.isaac_sim, self.scenario_dict_config.synthesizer_workers, scenario_name
        )

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
                self.scenario_dict_config.cameras[camera_conf_name], HydraInstantiateConversion.PARTIAL
            )
            yield from camera.get_cameras()
