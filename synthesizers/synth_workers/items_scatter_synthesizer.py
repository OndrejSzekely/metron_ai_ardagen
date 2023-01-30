"""
Defines *Items Scatter Synthesizer* class which is responsible for items scatter placement.
"""

from typing import Dict, List
import numpy as np
from metron_shared import param_validators as param_val
from metron_shared.config.instantiate import instantiate_from_hydra_config, HydraInstantiateConversion
from .base_synthesizer import BaseSynthesizer


class ItemsScatterSynthesizer(BaseSynthesizer):  # pylint: disable=too-few-public-methods
    """
    Defines *Items Scatter Synthesizer* class which is responsible for items scatter placement.

    Attributes:
        placement_synths (List[str]): Target Synthesizers. The assets are placed on their prims.
        semantics (str): Semantic class for the Synthesizer's primitives.
        number_of_assets_displayed_at_once (int): Number of assets displayed at once. The corresponding number of
                assets are sampled.
        assets_pool_size (int): Number of assets in the pool.
        placement_prims (List[str]): List of placement prims of the corresponding Synthesizers, defined by
            <placement_synths> in constructor.
        scattered_prims (List[str]): List of prims of the assets added to the scene from the pool.
    """

    def __init__(  # pylint: disable=too-many-arguments
        self,
        class_name: str,
        scenario_owner: str,
        assets: Dict[str, Dict],
        number_of_assets_displayed_at_once: int,
        assets_pool_size: int,
        placement_synths: List[str],
        semantics: str,
    ) -> None:
        """

        Args:
            class_name (str): Synthesizer name given by user in the config.
            scenario_owner (str): Name of owning Scenario.
            assets (Dict[str, Dict]): Assets pool. From the objects pool, <number_of_assets_displayed_at_once> objects
                are chosen randomly.
            number_of_assets_displayed_at_once (int): Number of assets displayed at once. The corresponding number of
                assets are sampled.
            assets_pool_size (int): Number of assets in the pool.
            placement_synths (List[str]): Target Synthesizer's names. The assets are placed on their prims.
            semantics (str): Semantic class for the Synthesizer's primitives.
        """
        param_val.check_type(assets, Dict[str, Dict])
        param_val.check_type(number_of_assets_displayed_at_once, int)
        param_val.check_type(assets_pool_size, int)
        param_val.check_type(placement_synths, List[str])
        param_val.check_type(semantics, str)
        param_val.check_parameter_value_in_range(
            number_of_assets_displayed_at_once, 1, 1e4
        )  # hardcoded value, no reason to go over 1e4. FIXME later.
        param_val.check_parameter_value_in_range(
            assets_pool_size, 1, 1e4
        )  # hardcoded value, no reason to go over 1e4. FIXME later.

        super().__init__(class_name, scenario_owner)

        self.placement_synths = placement_synths
        self.semantics = semantics
        self.number_of_assets_displayed_at_once = number_of_assets_displayed_at_once
        self.assets_pool_size = assets_pool_size
        self.placement_prims: List[str] = []

        assets_keys = list(assets.keys())
        assets_distribution = np.random.randint(0, len(assets_keys), self.assets_pool_size, np.int32)
        unique_assets_stats = np.unique(assets_distribution, return_counts=True)

        self.fg_assets_synthesizers = []
        self.scattered_prims = []
        # Only used `Assets Synthesizers` are instantiated.
        for ind, (fg_asset_ind, assets_num_per_synth) in enumerate(zip(*unique_assets_stats)):
            assets_synth = instantiate_from_hydra_config(
                assets[assets_keys[fg_asset_ind]],
                HydraInstantiateConversion.PARTIAL,
                assets_num_to_generate=assets_num_per_synth.item(),
                class_name=f"{self.__name__}_{assets_keys[ind]}",
                scenario_owner=self.scenario_owner,
            )
            self.fg_assets_synthesizers.append(assets_synth)
            self.scattered_prims.extend(assets_synth.get_prims())

    def register_synthesizers_prims(self, synthesizer_workers: Dict[str, BaseSynthesizer]) -> None:
        """
        Allows an access to other `Synthesizer's` prims if needed.

        Args:
            synthesizer_workers (Dict[str, BaseSynthesizer]): Dict of all Synthesizers.
        Returns (None):
        """
        for placement_synth in self.placement_synths:
            self.placement_prims.extend(synthesizer_workers[placement_synth].get_prims())

    def __call__(self, camera_setup: List[str]) -> None:
        """
        With this magic function, a command is executed.

        Args:
            camera_setup (List[str]): List of camera primitive paths in for the camera setup. It can contain more than
                one camera, e.g. stereo camera or more complicated camera rigs.
        """
        # Isaac Sim app has to be created before modules can be imported, so called in here.
        import omni.replicator.core as rep  # pylint: disable=import-outside-toplevel

        visibility_choice = [False] * (self.assets_pool_size - self.number_of_assets_displayed_at_once)
        visibility_choice.extend([True] * self.number_of_assets_displayed_at_once)
        rep.randomizer.scatter_2d(surface_prims=self.placement_prims, input_prims=self.scattered_prims)

        with rep.create.group(self.scattered_prims):
            rep.modify.visibility(
                # Calling registered ArDaGen Extension function.
                rep.distribution.shuffle(choices=visibility_choice)
            )

    def get_prims(self) -> List[str]:
        """
        Returns paths in stage to `Synthesizer's` created prims.

        Returns:
            List[str]: List of stage prim paths.
        """
        return []
