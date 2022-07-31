"""
`Master Synthesizer` does virtual world orchestration and manages `Synthesizer Workers`.
"""


import importlib
from typing import Dict, Iterable
from omegaconf import OmegaConf
from tools.isaac_sim import IsaacSimApp
from .synth_workers.base_synthesizer import BaseSynthesizer
from metron_shared import param_validators as param_val


class MasterSynthesizer:  # pylint: disable=too-few-public-methods
    """
    `Master Synthesizer` does virtual world orchestration and manages `Synthesizer Workers`.

    Attributes:
        isaac_sim_app (IsaacSimApp): `Isaac Sim App` instance.
        synthesizers_workers (List[BaseSynthesizer]): List of `Synthesizer Workers`.
        synthesizers_worker_names (list[str]): List of `Synthesizer Worker` names.
    """

    def __init__(self, isaac_sim: IsaacSimApp, synthesizer_workers: Dict) -> None:
        """
        Args:
            isaac_sim (IsaacSimApp): `Isaac Sim App` instance.

        """
        param_val.check_type(isaac_sim, IsaacSimApp)
        param_val.check_type(synthesizer_workers, Dict)

        self.isaac_sim_app = isaac_sim
        self._instantiate_synthesizer_workers(synthesizer_workers)  # pylint: disable=no-value-for-parameter

    def _instantiate_synthesizer_workers(self, synthesizer_workers: Dict) -> None:
        """
        Instantiates all enabled `Synthesizer Workers`. Objects can't be instantiated directly using Hydra's
        `initialize`, because it has to be get rid of each `Synthesizer Worker` config's `enabled` attribute
        which is not meant to be used during classes initialization. It is used only to state whether it's meant to
        be used or not.

        Args:
            synthesizer_workers (Dict): `Synthesizer Workers` dict.
        """
        self.synthesizers_workers = []
        self.synthesizers_worker_names = []
        for synth_worker in synthesizer_workers:
            synth_worker_module_path, _, synth_worker_class_name = synthesizer_workers[
                synth_worker
            ].target_class.rpartition(".")
            synth_worker_module = importlib.import_module(synth_worker_module_path)
            synth_worker_conf_dict = OmegaConf.to_container(synthesizer_workers[synth_worker])
            synth_worker_conf_dict.pop("target_class")
            self.synthesizers_workers.append(
                getattr(synth_worker_module, synth_worker_class_name)(rep, **synth_worker_conf_dict)
            )
            self.isaac_sim_app.update()
            self.synthesizers_worker_names.append(synth_worker)

    def __iter__(self) -> Iterable[BaseSynthesizer]:
        """
        Returns iterator over the `Synthesizer Workers`.

        Returns:
            Iterable[BaseSynthesizer]: Iterator over the `Synthesizer Workers`.
        """
        return iter(self.synthesizers_workers)
