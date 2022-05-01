"""
`Master Synthesizer` does virtual world orchestration and manages `Synthesizer Workers`.
"""


import importlib
from omegaconf import OmegaConf
from tools.isaac_sim import IsaacSimApp
from metron_shared.config.config import GetHydraConfig
from metron_shared import param_validators as param_val
from miscellaneous.metron_ai_argagen_config_schema import MetronAIArDaGenConfigSchema


class MasterSynthesizer:  # pylint: disable=too-few-public-methods
    """
    `Master Synthesizer` does virtual world orchestration and manages `Synthesizer Workers`.

    Attributes:
        isaac_sim_app (IsaacSimApp): `Isaac Sim App` instance.
        worker_synthesizers (List[BaseSynthesizer]): List of `Worker Synthesizers`.
    """

    def __init__(self, isaac_sim: IsaacSimApp) -> None:
        """
        Args:
            isaac_sim (IsaacSimApp): `Isaac Sim App` instance.

        Returns (None):
        """
        param_val.type_check(isaac_sim, IsaacSimApp)

        self.isaac_sim_app = isaac_sim
        self._instantiate_synthesizer_workers()  # pylint: disable=no-value-for-parameter

    @GetHydraConfig
    def _instantiate_synthesizer_workers(self, hydra_config: MetronAIArDaGenConfigSchema) -> None:
        """
        Instantiates all enabled `Synthesizer Workers`. Objects can't be instantiated directly using Hydra's
        `initialize`, because it has to be get rid of each `Synthesizer Worker` config's `enabled` attribute
        which is not meant to be used during classes initialization. It is used only to state whether it's meant to
        be used or not.

        Args:
            hydra_config (GRE2GConfigSchema): GRE2G configuration parameters provided by Hydra's config.
        """
        ms_synth: OmegaConf = hydra_config.master_synthesizer

        self.worker_synthesizers = []
        for synth_worker in ms_synth.synthesizer_workers:
            if ms_synth.synthesizer_workers[synth_worker].enabled:
                synth_worker_module_path, _, synth_worker_class_name = ms_synth.synthesizer_workers[
                    synth_worker
                ].target_class.rpartition(".")
                synth_worker_module = importlib.import_module(synth_worker_module_path)
                synth_worker_conf_dict = OmegaConf.to_container(ms_synth.synthesizer_workers[synth_worker])
                synth_worker_conf_dict.pop("_target_")
                synth_worker_conf_dict.pop("enabled")
                self.worker_synthesizers.append(
                    getattr(synth_worker_module, synth_worker_class_name)(**synth_worker_conf_dict)
                )
