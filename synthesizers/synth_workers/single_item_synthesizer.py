"""
Defines *Single Item Synthesizer* class which is responsible for single item synthesis.
"""

from typing import Dict, List
from metron_shared import param_validators as param_val
from .base_synthesizer import BaseSynthesizer


class SingleItemSynthesizer(BaseSynthesizer):  # pylint: disable=too-few-public-methods
    """
    Defines *Single Item Synthesizer* class which is responsible for single item synthesis.

    Attributes:
        position (List[float]): Asset placement position in the scene. Given by X, Y, Z coordinates.
        usd_path (str): Nucleus path of the USD asset to be added.
        semantics (str): Semantic class for the asset.
        _stage_prim_path (str): Path of the loaded asset in the stage.
    """

    def __init__(  # pylint: disable=too-many-arguments
        self, class_name: str, scenario_owner: str, usd_path: str, position: List[float], semantics: str
    ) -> None:
        """

        Args:
            class_name (str): Synthesizer name given by user in the config.
            scenario_owner (str): Name of owning Scenario.
            usd_path (str): Nucleus path of the USD asset to be added.
            position (List[float]): Asset placement position - X, Y, Z.
            semantics (str): Semantic class for the asset.
        """
        import omni.usd  # pylint: disable=import-outside-toplevel
        from pxr import Semantics, UsdGeom  # pylint: disable=import-outside-toplevel

        param_val.check_type(usd_path, str)
        param_val.check_type(position, List[float])
        param_val.check_type(semantics, str)

        super().__init__(class_name, scenario_owner)

        self.position = position
        self.usd_path = usd_path
        self.semantics = semantics

        stage = omni.usd.get_context().get_stage()
        prim_path = omni.usd.get_stage_next_free_path(stage, "SingleItemSynthesizer/Ref", False)
        prim = stage.DefinePrim(prim_path)
        prim.GetReferences().AddReference(usd_path)
        UsdGeom.XformCommonAPI(prim).SetTranslate(self.position)
        semantics_str = f"class_{semantics}"
        sem = Semantics.SemanticsAPI.Apply(prim, semantics_str)
        sem.CreateSemanticTypeAttr()
        sem.CreateSemanticDataAttr()
        sem.GetSemanticTypeAttr().Set("class")
        sem.GetSemanticDataAttr().Set(semantics)

        self._stage_prim_path = prim_path

    def __call__(self, camera_setup: List[str]) -> None:
        """
        Called by Replicator to make changes in the scene.

        Args:
            camera_setup (List[str]): List of camera primitive paths in for the camera setup. It can contain more than
                one camera, e.g. stereo camera or more complicated camera rigs.
        """

    def get_prims(self) -> List[str]:
        """
        Returns paths in stage to `Synthesizer's` created prims.

        Returns:
            List[str]: List of stage prim paths.
        """
        return [self._stage_prim_path]

    def register_synthesizers_prims(self, synthesizer_workers: Dict[str, BaseSynthesizer]) -> None:
        """
        Allows an access to other `Synthesizer's` prims if needed.

        Args:
            synthesizer_workers (Dict[str, BaseSynthesizer]): Dict of all Synthesizers.
        Returns (None):
        """
        param_val.check_type(synthesizer_workers, Dict[str, BaseSynthesizer])
