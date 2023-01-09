"""
Defines `OVWriter` class to write generated data on a file system.
"""

from os import path
from typing import Any, List
from metron_shared import param_validators as param_val
from metron_shared import io_utils


class NullWriter:
    """
    Represents a non-instantiated OV Writer. Follows Null Object design pattern.
    """

    def create(self, scenario_name: str) -> str:  # pylint: disable=unused-argument no-self-use
        """
        Duck-type interface of null object for OV Writer.

        Args:
            scenario_name (str): scenario name.

        Returns:
            str: Returns the name name of created writer. This name is used later on to delete the writer.
        """
        return "NullWriter"

    def attach(self, camera_renders: List[Any]) -> None:
        """
        Duck-type interface of null object for OV Writer.

        Args:
            camera_render (List[Any]): List of camera renders belonging to the camera setup.
        """

    def initialize(*args: Any, **kwargs: Any) -> None:  # pylint: disable=no-method-argument
        """
        Duck-type interface of null object for OV Writer.
        """


class OVWriter:  # pylint: disable=too-many-instance-attributes), too-many-arguments
    """
    Defines `OVWriter` class to write generated data on file system.

    Attributes:
        fs_store_path (str): Defines `Writer` output folder path.
        writer (Union[NullWriter, WriterTemplate]): Stores actual OV writer used to save data. NullWriter is used before
            OV Writer is instantiated.
        writer_name (str): OV Writer type name.
        write_rgb (bool): `True` will generate RGB images.
        write_bounding_box_2d_tight (bool): `True` will generate 2D bboxes, not covering obscured object's parts.
        write_bounding_box_2d_loose (bool): `True` will generate 2D bboxes, covering also obscured object's parts.
        write_semantic_segmentation (bool): `True` will generate semantic segmentaion annotations.
        write_instance_segmentation (bool): `True` will generate instance segmentation annotations.
        write_distance_to_camera (bool): `True` will generate inverse depth map to camera.
        write_distance_to_image_plane (bool): `True` will generate inverse depth map to image plane.
        write_bounding_box_3d (bool): `True` will generate 3D bounding boxes.
        write_occlusion (bool): TODO add the description.
        write_normals (bool):  `True` will generate normals for each pixels in the image.
        write_motion_vectors (bool): `True` will generate motion vector of the pixels inside the image.
        write_camera_params (bool): `True` will store camera's parameters (intrinsics, extrinsics).
    """

    def __init__(  # pylint: disable=too-many-function-args
        self,
        fs_store_path: str,
        write_rgb: bool,
        write_bounding_box_2d_tight: bool,
        write_bounding_box_2d_loose: bool,
        write_semantic_segmentation: bool,
        write_instance_segmentation: bool,
        write_distance_to_camera: bool,
        write_distance_to_image_plane: bool,
        write_bounding_box_3d: bool,
        write_occlusion: bool,
        write_normals: bool,
        write_motion_vectors: bool,
        write_camera_params: bool,
    ) -> None:
        """
        Init.

        Args:
            fs_store_path (str): Defines `Writer` output folder path.
            write_rgb (bool): `True` will generate RGB images.
            write_bounding_box_2d_tight (bool): `True` will generate 2D bboxes, not covering obscured object's parts.
            write_bounding_box_2d_loose (bool): `True` will generate 2D bboxes, covering also obscured object's parts.
            write_semantic_segmentation (bool): `True` will generate semantic segmentaion annotations.
            write_instance_segmentation (bool): `True` will generate instance segmentation annotations.
            write_distance_to_camera (bool): `True` will generate inverse depth map to camera.
            write_distance_to_image_plane (bool): `True` will generate inverse depth map to image plane.
            write_bounding_box_3d (bool): `True` will generate 3D bounding boxes.
            write_occlusion (bool): TODO add the description.
            write_normals (bool):  `True` will generate normals for each pixels in the image.
            write_motion_vectors (bool): `True` will generate motion vector of the pixels inside the image.
            write_camera_params (bool): `True` will store camera's parameters (intrinsics, extrinsics).
        """

        param_val.check_type(fs_store_path, str)
        param_val.check_type(write_rgb, bool)
        param_val.check_type(write_bounding_box_2d_tight, bool)
        param_val.check_type(write_bounding_box_2d_loose, bool)
        param_val.check_type(write_semantic_segmentation, bool)
        param_val.check_type(write_instance_segmentation, bool)
        param_val.check_type(write_distance_to_camera, bool)
        param_val.check_type(write_distance_to_image_plane, bool)
        param_val.check_type(write_bounding_box_3d, bool)
        param_val.check_type(write_occlusion, bool)
        param_val.check_type(write_normals, bool)
        param_val.check_type(write_motion_vectors, bool)
        param_val.check_type(write_camera_params, bool)

        self.fs_store_path = fs_store_path
        self.write_rgb = write_rgb
        self.write_bounding_box_2d_tight = write_bounding_box_2d_tight
        self.write_bounding_box_2d_loose = write_bounding_box_2d_loose
        self.write_semantic_segmentation = write_semantic_segmentation
        self.write_instance_segmentation = write_instance_segmentation
        self.write_distance_to_camera = write_distance_to_camera
        self.write_distance_to_image_plane = write_distance_to_image_plane
        self.write_bounding_box_3d = write_bounding_box_3d
        self.write_occlusion = write_occlusion
        self.write_normals = write_normals
        self.write_motion_vectors = write_motion_vectors
        self.write_camera_params = write_camera_params

        self.writer = NullWriter()
        self.writer_name = "OffsetWriter"

    def create(self, scenario_name: str, frames_readout_offset: int) -> str:
        """
        Creates a writer for given <scenario_name> scenario. It creates a new folder with <scenario_name> name on
        <fs_store_path> path to sepearte data for each scenario.

        Args:
            scenario_name (str): scenario name.
            frames_readout_offset (int): Scenario's frames readout offset before saving a frame.

        Returns:
            str: Returns the name name of created writer. This name is used later on to delete the writer.
        """
        # Isaac Sim app has to be created before modules can be imported, so called in here.
        import omni.replicator.core as rep  # pylint: disable=import-outside-toplevel

        # The module with custom writer has to be imported to be visible for the regitering process.
        import miscellaneous.offset_writer  # noqa: F401 pylint: disable=import-outside-toplevel, unused-import

        scenario_folder_path = path.join(self.fs_store_path, scenario_name)
        io_utils.force_folder_create(scenario_folder_path)
        self.writer = rep.WriterRegistry.get(self.writer_name)
        self.writer.initialize(
            output_dir=scenario_folder_path,
            rgb=self.write_rgb,
            bounding_box_2d_tight=self.write_bounding_box_2d_tight,
            bounding_box_2d_loose=self.write_bounding_box_2d_loose,
            semantic_segmentation=self.write_semantic_segmentation,
            instance_segmentation=self.write_instance_segmentation,
            distance_to_camera=self.write_distance_to_camera,
            distance_to_image_plane=self.write_distance_to_image_plane,
            bounding_box_3d=self.write_bounding_box_3d,
            occlusion=self.write_occlusion,
            normals=self.write_normals,
            motion_vectors=self.write_motion_vectors,
            camera_params=self.write_camera_params,
            frame_content_lifespan=frames_readout_offset + 1,  # To match with Replicator, exclusive lifespan
        )

        return self.writer_name

    def attach(self, camera_renders: List[Any]) -> None:
        """
        Attaches <camera_renders> camera renders to the writer to capture images from that cameras in the camera setup.

        Args:
            camera_render (List[Any]): List of camera renders belonging to the camera setup.
        """
        self.writer.attach(camera_renders)
