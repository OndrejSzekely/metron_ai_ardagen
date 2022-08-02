"""
Defines `OVWriter` class to write generated data on file system.
"""

from os import path
from typing import Any, List
from metron_shared import param_validators as param_val


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


class OVWriter:
    """
    Defines `OVWriter` class to write generated data on file system.

    Attributes:
        fs_store_path (str): Defines `Writer` output folder path.
        writer (Union[NullWriter, WriterTemplate]): Stores actual OV writer used to save data. NullWriter is used before
            OV Writer is instantiated.
        writer_name (str): OV Writer type name.
    """

    def __init__(self, fs_store_path: str) -> None:
        """
        Init.

        Args:
            fs_store_path (str): Defines `Writer` output folder path.
        """
        param_val.check_type(fs_store_path, str)

        self.fs_store_path = fs_store_path
        self.writer = NullWriter()
        self.writer_name = "BasicWriter"

    def create(self, scenario_name: str) -> str:
        """
        Creates a writer for given <scenario_name> scenario. It creates a new folder with <scenario_name> name on
        <fs_store_path> path to sepearte data for each scenario.

        Args:
            scenario_name (str): scenario name.

        Returns:
            str: Returns the name name of created writer. This name is used later on to delete the writer.
        """
        # Isaac Sim app has to be created before modules can be imported, so called in here.
        import omni.replicator.core as rep  # pylint: disable=import-outside-toplevel

        self.writer = rep.WriterRegistry.get(self.writer_name)
        self.writer.initialize(
            output_dir=path.join(self.fs_store_path, scenario_name),
            rgb=True,
            bounding_box_2d_tight=True,
            bounding_box_2d_loose=False,
            semantic_segmentation=False,
            instance_segmentation=False,
            distance_to_camera=False,
            distance_to_image_plane=False,
            bounding_box_3d=False,
            occlusion=False,
            normals=False,
            motion_vectors=False,
        )

        return self.writer_name

    def attach(self, camera_renders: List[Any]) -> None:
        """
        Attaches <camera_renders> camera renders to the writer to capture images from that cameras in the camera setup.

        Args:
            camera_render (List[Any]): List of camera renders belonging to the camera setup.
        """
        self.writer.attach(camera_renders)
