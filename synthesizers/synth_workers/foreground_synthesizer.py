"""
Defines *Foreground Synthesizer* class which is responsible for scene foreground synthesis.
"""

from typing import Dict
from .base_synthesizer import BaseSynthesizer
from miscellaneous.metron_ai_ardagen_utils import instantiate_from_hydra_config
from metron_shared import param_validators as param_val


class ForegroundSynthesizer(BaseSynthesizer):  # pylint: disable=too-few-public-methods
    """
    Defines *Foreground Synthesizer* class which is responsible for scene foreground synthesis.
    """

    def __init__(self, assets: Dict[str, Dict]) -> None:
        param_val.check_type(assets, Dict[str, Dict])

        fg_assets_synthesizers = []
        for asset_type_config in assets:
            fg_assets_synthesizers.append(instantiate_from_hydra_config(assets[asset_type_config]))

        self.fg_assets_synthesizers = fg_assets_synthesizers

    def __call__(self) -> None:
        """
        Nothing to do.
        """
