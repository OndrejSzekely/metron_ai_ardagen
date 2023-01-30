"""
`Master Synthesizer` does virtual world orchestration and manages `Synthesizer Workers`.
"""


import importlib
from typing import Iterator, List, Any
from omegaconf import OmegaConf, DictConfig
from tools.isaac_sim import IsaacSimApp
from metron_shared import param_validators as param_val
from .synth_workers.base_synthesizer import BaseSynthesizer


class NullMasterSynthesizer:  # pylint: disable=too-few-public-methods
    """
    Represents a non-instantiated `Master Synthesizer` in `Scenario`. Follows Null Object design pattern.

    Attributes:
        synthesizers_worker_names (list[str]): Empty list of `Synthesizer Worker` names.
    """

    def __init__(self) -> None:
        """ """
        self.synthesizers_worker_names: List[Any] = []

    def __iter__(self) -> Iterator[BaseSynthesizer]:
        """
        Duck-type interface of null object for `Scenario`.

        Returns:
            iter(Any): Returns empty iterator.
        """
        return iter([])


class MasterSynthesizer:  # pylint: disable=too-few-public-methods
    """
    `Master Synthesizer` does virtual world orchestration and manages `Synthesizer Workers`.

    Attributes:
        isaac_sim_app (IsaacSimApp): `Isaac Sim App` instance.
        synthesizers_workers (List[BaseSynthesizer]): List of `Synthesizer Workers`.
        synthesizers_worker_names (list[str]): List of `Synthesizer Worker` names.
    """

    def __init__(self, isaac_sim: IsaacSimApp, synthesizer_workers: DictConfig, scenario_name: str) -> None:
        """
        Args:
            isaac_sim (IsaacSimApp): `Isaac Sim App` instance.
            synthesizer_workers (DictConfig): `Synthesizer Workers` configuration.
            scenario_name (str): `Scenario` owner name.
        """
        param_val.check_type(isaac_sim, IsaacSimApp)
        param_val.check_type(synthesizer_workers, DictConfig)
        param_val.check_type(scenario_name, str)

        self.isaac_sim_app = isaac_sim
        self._instantiate_synthesizer_workers(
            synthesizer_workers, scenario_name
        )  # pylint: disable=no-value-for-parameter

    def _instantiate_synthesizer_workers(self, synthesizer_workers: DictConfig, scenario_name: str) -> None:
        """
        Instantiates all enabled `Synthesizer Workers`. Objects can't be instantiated directly using Hydra's
        `initialize`, because it has to be get rid of each `Synthesizer Worker` config's `enabled` attribute
        which is not meant to be used during classes initialization. It is used only to state whether it's meant to
        be used or not.

        Args:
            synthesizer_workers (DictConfig): `Synthesizer Workers` dict.
            scenario_name (str): Defines the name of owning `Scenario`.
        """
        param_val.check_type(synthesizer_workers, DictConfig)
        param_val.check_type(scenario_name, str)

        self.synthesizers_workers = []
        self.synthesizers_worker_names = []
        for synth_worker in synthesizer_workers:
            synth_worker_module_path, _, synth_worker_class_name = synthesizer_workers[
                synth_worker
            ].target_class.rpartition(".")
            synth_worker_module = importlib.import_module(synth_worker_module_path)
            synth_worker_conf_dict = OmegaConf.to_container(synthesizer_workers[synth_worker], resolve=True)
            synth_worker_conf_dict.pop("target_class")
            synth = getattr(synth_worker_module, synth_worker_class_name)(
                class_name=synth_worker, scenario_owner=scenario_name, **synth_worker_conf_dict
            )
            self.synthesizers_workers.append(synth)
            self.isaac_sim_app.update()
            self.synthesizers_worker_names.append(synth_worker)

        kw_synthesizers_workers = dict(zip(self.synthesizers_worker_names, self.synthesizers_workers))
        for synth_worker in self.synthesizers_workers:
            synth_worker.register_synthesizers_prims(kw_synthesizers_workers)

    def __iter__(self) -> Iterator[BaseSynthesizer]:
        """
        Returns iterator over the `Synthesizer Workers`.

        Returns:
            Iterator[BaseSynthesizer]: Iterator over the `Synthesizer Workers`.
        """
        return iter(self.synthesizers_workers)
