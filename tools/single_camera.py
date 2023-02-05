"""
Defines `Single Camera` class.
"""

from typing import Tuple, Any, Generator, List
from metron_shared import param_validators as param_val


class SingleCamera:  # pylint: disable=too-few-public-methods
    """
    Defines `Single Camera` camera setup.

    Attributes:
        cam_name (str): Camera's name.
        position (List[float]): Camera's position given by 3 numbers in 3D.
        rotation (List[float]): Camera's rotation given by 3 numbers in 3D.
        clipping_range (List[float]): Clipping range of 3D objects given by tupple (clipping_min, clipping_max).
        cam_resolution (List[int]): Camera resolution.
    """

    def __init__(  # pylint: disable = too-many-arguments
        self,
        cam_name: str,
        position: List[float],
        rotation: List[float],
        clipping_range: List[float],
        resolution: List[int],
    ) -> None:
        """
        Init.

        Args:
            cam_name (str): Camera's name.
            position (List[float]): Camera's position given by 3 numbers in 3D.
            rotation (List[float]): Camera's rotation given by 3 numbers in 3D.
            clipping_range (List[float]): Clipping range of 3D objects given by tupple (clipping_min, clipping_max).
            resolution (List[int]): Camera resolution.
        """
        param_val.check_type(cam_name, str)
        param_val.check_type(position, List[float])
        param_val.check_type(rotation, List[float])
        param_val.check_type(clipping_range, List[float])
        param_val.check_type(resolution, List[int])
        param_val.check_length_of_list(position, 3)
        param_val.check_length_of_list(rotation, 3)
        param_val.check_length_of_list(clipping_range, 2)
        param_val.check_length_of_list(resolution, 2)
        param_val.check_parameter_value_in_range(resolution[0], 1, 7680)
        param_val.check_parameter_value_in_range(resolution[1], 1, 4320)

        self.cam_name = cam_name
        self.position = position
        self.rotation = rotation
        self.clipping_range = clipping_range
        self.cam_resolution = resolution

    def get_cameras(self) -> Generator[Tuple[str, List[Any], List[Any]], None, None]:
        """
        Returns camera setup for single camera, which means one camera.

        Yields:
            Generator[str, Tuple[List[Any], List[Any]], None, None]: Returns three values. The first one is camera name.
                The second and the third values are lists of one item. The first one contains a stage path to the camera
                and the second list containes a corresponding camera render product.
        """
        # Isaac Sim app has to be created before modules can be imported, so called in here.
        import omni.replicator.core as rep  # pylint: disable=import-outside-toplevel
        import omni.usd  # pylint: disable=import-outside-toplevel

        camera = rep.create.camera(
            position=self.position, rotation=self.rotation, clipping_range=tuple(self.clipping_range)
        )
        render_product = rep.create.render_product(camera, self.cam_resolution)
        stage = omni.usd.get_context().get_stage()
        yield self.cam_name, [
            stage.GetPrimAtPath(camera.node.get_prim_path()).GetRelationship("inputs:prims").GetTargets()[0].pathString
        ], [render_product]
