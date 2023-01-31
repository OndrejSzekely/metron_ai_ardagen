"""
Implements OV Writer with offset.
"""

import io
from os import path
import json
from typing import List, Optional, Any
import numpy as np
from omni.replicator.core import WriterRegistry, Writer, AnnotatorRegistry
from omni.replicator.core import BackendDispatch
from omni.replicator.core.scripts.writers_default.tools import colorize_normals
from metron_shared import param_validators as param_val


# TODO: Refactor the class. Too many SCA exceptions. pylint: disable=fixme
class OffsetWriter(Writer):  # pylint: disable=too-many-instance-attributes, too-few-public-methods
    """Offset writer which is built upon <BasicWriter>, but has a frames offset before readout the frame. It assumes
    that stage content holds still for number of frames given by <frame_content_lifespan> before some change. This
    offset allows renderer to render everything properly and stabilize quality of RTX and DLSS. It has to be empirically
    determined what is a good offset for given scenario.

    Attributes:
        __name__ (str): Class name.
        _MAX_FRAMES_OFFSET (int): Defines the maximal allowed frames offset.
        _NUMBERING_PADDING (int): Defines the leading zeros for saved files naming.
        _output_dir (str): Output directory string that indicates the directory to save the results.
        _backend(BackendDispatch): OV backend dispatcher.
        _frame_id (int): Frame index.
        _image_output_format (str): Output image format.
        _content_lifespan (int): Internal aux variable for the loop. Indicates how many frames passed since the last
            read out.
        cam_setup_name (str): Camera setup name.
        annotators (List[Any]): List of annotators.
        frame_read_out_num (int): Defines frame number after which the frame is read out and saved.
            For <frame_content_lifespan> = 1 it is every frame. For <frame_content_lifespan> > 1, the value is
             <frame_content_lifespan> - 1. This is because we want to read the frame every <frame_content_lifespan>th
             frame.
        content_lifespan_init_val (int): Init value for the helper loop, to decide when to perform a new read out.
            Depends on <content_lifespan_init_val>. For single frame read out, it's set to 0, otherwise to -1.
    """

    __name__ = "OffsetWriter"
    _MAX_FRAMES_OFFSET: int = 60 * 3  # Final is missing in this Python version.
    _NUMBERING_PADDING = 3

    def __init__(  # pylint: disable=too-many-arguments, too-many-locals, too-many-branches, too-many-statements
        self,
        cam_setup_name: str,
        output_dir: str,
        semantic_types: Optional[List[str]] = None,
        rgb: bool = False,
        bounding_box_2d_tight: bool = False,
        bounding_box_2d_loose: bool = False,
        semantic_segmentation: bool = False,
        instance_segmentation: bool = False,
        distance_to_camera: bool = False,
        distance_to_image_plane: bool = False,
        bounding_box_3d: bool = False,
        occlusion: bool = False,
        normals: bool = False,
        motion_vectors: bool = False,
        camera_params: bool = False,
        # TODO: Create enum for file format.
        image_output_format: str = "png",
        colorize_semantic_segmentation: bool = True,
        colorize_instance_segmentation: bool = True,
        frame_content_lifespan: int = 1,
    ):
        """
        cam_setup_name (str): Camera setup name.
        output_dir (str): Output FS folder where to save data.
        semantic_types (Optional[List[str]], optional): Semantic classes to be kept. Defaults to None.
        rgb (bool, optional): `True` will generate RGB images. Defaults to False.
        bounding_box_2d_tight (bool, optional): `True` will generate 2D bboxes, not covering obscured object's
            parts. Defaults to False.
        bounding_box_2d_loose (bool, optional): `True` will generate 2D bboxes, covering also obscured object's
            parts. Defaults to False.
        semantic_segmentation (bool, optional): `True` will generate semantic segmentaion annotations.
            Defaults to False.
        instance_segmentation (bool, optional): `True` will generate instance segmentation annotations.
            Defaults to False.
        distance_to_camera (bool, optional): `True` will generate inverse depth map to camera. Defaults to False.
        distance_to_image_plane (bool, optional): `True` will generate inverse depth map to image plane.
            This means, it's not exact distance, but Z component of the distance. Defaults to False.
        bounding_box_3d (bool, optional): `True` will generate 3D bounding boxes. Defaults to False.
        occlusion (bool, optional): TODO add the description. Defaults to False.
        normals (bool, optional): `True` will generate normals for each pixels in the image. Defaults to False.
        motion_vectors (bool, optional): `True` will generate motion vector of the pixels inside the image.
            Defaults to False.
        camera_params (bool, optional): `True` will store camera's parameters (intrinsics, extrinsics).
            Defaults to False.
        image_output_format (str, optional): String that indicates the format of saved RGB images.
            Defaults to "png".
        colorize_semantic_segmentation (bool, optional): If ``True``, semantic segmentation is converted to an image
            where semantic IDs are mapped to colors and saved as a uint8 4 channel PNG image. If ``False``,
            the output is saved as a uint32 PNG image. Defaults to ``True``
        colorize_instance_segmentation (bool, optional): If ``True``, semantic segmentation is converted to an image
            where semantic IDs are mapped to colors and saved as a uint8 4 channel PNG image. If ``False``,
            the output is saved as a uint32 PNG image. Defaults to ``True``.
        frame_content_lifespan (int, optional): Number of frames for which the scene content holds still and
            no frames readout is performed. Defaults to 1.
        semantic_types (List[str], optional): List of semantic types to consider when filtering annotator data.
            Default: ["class"]
        writer_name (str): OV Writer type name.
        """
        param_val.check_type(cam_setup_name, str)
        param_val.check_type(output_dir, str)
        param_val.check_type(semantic_types, Optional[List[str]])
        param_val.check_type(rgb, bool)
        param_val.check_type(bounding_box_2d_tight, bool)
        param_val.check_type(bounding_box_2d_loose, bool)
        param_val.check_type(semantic_segmentation, bool)
        param_val.check_type(instance_segmentation, bool)
        param_val.check_type(distance_to_camera, bool)
        param_val.check_type(distance_to_image_plane, bool)
        param_val.check_type(bounding_box_3d, bool)
        param_val.check_type(occlusion, bool)
        param_val.check_type(normals, bool)
        param_val.check_type(motion_vectors, bool)
        param_val.check_type(camera_params, bool)
        param_val.check_type(image_output_format, str)
        param_val.check_type(colorize_semantic_segmentation, bool)
        param_val.check_type(colorize_instance_segmentation, bool)
        param_val.check_type(frame_content_lifespan, int)
        param_val.check_parameter_value_in_range(frame_content_lifespan, 1, self._MAX_FRAMES_OFFSET)

        self.cam_setup_name = cam_setup_name
        self._output_dir = output_dir
        self._backend = BackendDispatch({"paths": {"out_dir": output_dir}})
        self._frame_id = 0
        self._image_output_format = image_output_format
        self.annotators = []
        self.frame_read_out_num = 1 if frame_content_lifespan == 1 else frame_content_lifespan - 1
        self._content_lifespan = 1
        # To have correct `difference` for no-offset and offset writing
        self.content_lifespan_init_val = 0 if frame_content_lifespan == 1 else -1

        # Specify the semantic types that will be included in output
        if semantic_types is None:
            semantic_types = ["class"]

        # RGB
        if rgb:
            self.annotators.append(AnnotatorRegistry.get_annotator("rgb"))

        # Bounding Box 2D
        if bounding_box_2d_tight:
            self.annotators.append(AnnotatorRegistry.get_annotator("bounding_box_2d_tight_fast"))

        if bounding_box_2d_loose:
            self.annotators.append(AnnotatorRegistry.get_annotator("bounding_box_2d_loose_fast"))

        # Semantic Segmentation
        if semantic_segmentation:
            self.annotators.append(
                AnnotatorRegistry.get_annotator(
                    "semantic_segmentation",
                    init_params={"semanticTypes": semantic_types, "colorize": colorize_semantic_segmentation},
                )
            )

        # Instance Segmentation
        if instance_segmentation:
            self.annotators.append(
                AnnotatorRegistry.get_annotator(
                    "instance_segmentation_fast", init_params={"colorize": colorize_instance_segmentation}
                )
            )

        # Depth
        if distance_to_camera:
            self.annotators.append(AnnotatorRegistry.get_annotator("distance_to_camera"))

        if distance_to_image_plane:
            self.annotators.append(AnnotatorRegistry.get_annotator("distance_to_image_plane"))

        # Bounding Box 3D
        if bounding_box_3d:
            self.annotators.append(AnnotatorRegistry.get_annotator("bounding_box_3d_fast"))

        # Motion Vectors
        if motion_vectors:
            self.annotators.append(AnnotatorRegistry.get_annotator("motion_vectors"))

        # Occlusion
        if occlusion:
            self.annotators.append(AnnotatorRegistry.get_annotator("occlusion"))

        # Normals
        if normals:
            self.annotators.append(AnnotatorRegistry.get_annotator("normals"))

        # Camera_params:
        if camera_params:
            self.annotators.append(AnnotatorRegistry.get_annotator("camera_params"))

    def write(self, data: dict) -> None:  # pylint: disable=too-many-locals, too-many-branches, too-many-statements
        """Write function called from the OgnWriter node on every frame to process annotator output.

        Args:
            data: A dictionary containing the annotator data for the current frame.
        """
        param_val.check_type(data, dict)

        if self._content_lifespan == self.frame_read_out_num:
            for annotator in data.keys():
                if annotator.startswith("rgb"):
                    fs_path = "rgb/"
                    self._write_rgb(fs_path, data[annotator])

                if annotator.startswith("normals"):
                    fs_path = "normals/"
                    self._write_normals(fs_path, data[annotator])

                if annotator.startswith("distance_to_camera"):
                    fs_path = "dist_to_cam/"
                    self._write_distance_to_camera(fs_path, data[annotator])

                if annotator.startswith("distance_to_image_plane"):
                    fs_path = "dist_to_img_plane/"
                    self._write_distance_to_image_plane(fs_path, data[annotator])

                if annotator.startswith("semantic_segmentation"):
                    fs_path = "sem_seg/"
                    self._write_semantic_segmentation(fs_path, data[annotator])

                if annotator.startswith("instance_segmentation"):
                    fs_path = "instance_seg/"
                    self._write_instance_segmentation(fs_path, data[annotator])

                if annotator.startswith("motion_vectors"):
                    fs_path = "motion_vector/"
                    self._write_motion_vectors(fs_path, data[annotator])

                if annotator.startswith("occlusion"):
                    fs_path = "occlusion/"
                    self._write_occlusion(fs_path, data[annotator])

                if annotator.startswith("bounding_box_3d"):
                    fs_path = "bbox_3d/"
                    self._write_bounding_box_data("3d", fs_path, data[annotator])

                if annotator.startswith("bounding_box_2d_loose"):
                    fs_path = "bbox_2d_loose/"
                    self._write_bounding_box_data("2d_loose", fs_path, data[annotator])

                if annotator.startswith("bounding_box_2d_tight"):
                    fs_path = "bbox_2d_tight/"
                    self._write_bounding_box_data("2d_tight", fs_path, data[annotator])

                if annotator.startswith("camera_params"):
                    fs_path = "camera_params/"
                    self._write_camera_params(fs_path, data[annotator])

            self._frame_id += 1
            self._content_lifespan = self.content_lifespan_init_val
        self._content_lifespan += 1

    def _write_rgb(self, fs_relative_path: str, annotator_data: Any) -> None:
        """
        Writes RGB image data.

        Args:
            fs_relative_path (str): FS relative path where to save RGB images.
            annotator_data (str): Annotator data.
        """
        param_val.check_type(fs_relative_path, str)

        rel_file_path = path.join(
            fs_relative_path,
            f"{self.cam_setup_name}_rgb_{self._frame_id:0{self._NUMBERING_PADDING}d}.{self._image_output_format}",
        )
        self._backend.write_image(rel_file_path, annotator_data)

    def _write_bounding_box_data(self, bbox_type: str, fs_relative_path: str, annotator_data: Any) -> None:
        """
        Writes bounding box data.

        Args:
            bbox_type (str): Specifies bbox type.
            fs_relative_path (str): FS relative path where to save bounding box data.
            annotator_data (str): Annotator data.
        """
        param_val.check_type(fs_relative_path, str)
        param_val.check_type(bbox_type, str)

        bbox_data = annotator_data["data"]

        file_path = path.join(
            fs_relative_path,
            f"{self.cam_setup_name}_bbox_{bbox_type}_{self._frame_id:0{self._NUMBERING_PADDING}d}.npy",
        )
        buf = io.BytesIO()
        np.save(buf, bbox_data)
        self._backend.write_blob(file_path, buf.getvalue())

    def _write_camera_params(self, fs_relative_path: str, annotator_data: Any) -> None:
        """
        Writes camera parameters data.

        Args:
            fs_relative_path (str): FS relative path where to save camera parameters data.
            annotator_data (str): Annotator data.
        """
        param_val.check_type(fs_relative_path, str)

        serializable_data = {}

        for key, val in annotator_data.items():
            if isinstance(val, np.ndarray):
                serializable_data[key] = val.tolist()
            else:
                serializable_data[key] = val

        file_path = path.join(
            fs_relative_path,
            f"{self.cam_setup_name}_camera_params_{self._frame_id:0{self._NUMBERING_PADDING}d}.json",
        )
        buf = io.BytesIO()
        buf.write(json.dumps(serializable_data).encode())
        self._backend.write_blob(file_path, buf.getvalue())

    def _write_instance_segmentation(self, fs_relative_path: str, annotator_data: Any) -> None:
        """
        Writes instance segmentation data.

        Args:
            fs_relative_path (str): FS relative path where to save instance segmentatino data.
            annotator_data (str): Annotator data.
        """
        param_val.check_type(fs_relative_path, str)

        instant_seg_data = annotator_data["data"]
        height, width = instant_seg_data.shape[:2]

        file_path = path.join(fs_relative_path, f"instance_seg_{self._frame_id:0{self._NUMBERING_PADDING}d}.png")
        if instant_seg_data.shape[-1] == 1:
            instance_seg_data = instant_seg_data.view(np.uint32).reshape(height, width)
            self._backend.write_image(file_path, instance_seg_data)
        else:
            instance_seg_data = instant_seg_data.view(np.uint8).reshape(height, width, -1)
            self._backend.write_image(file_path, instance_seg_data)

        id_to_labels = annotator_data["info"]["idToLabels"]
        file_path = path.join(
            fs_relative_path,
            f"{self.cam_setup_name}_instance_seg_mapping_{self._frame_id:0{self._NUMBERING_PADDING}d}.json",
        )
        buf = io.BytesIO()
        buf.write(json.dumps({str(k): v for k, v in id_to_labels.items()}).encode())
        self._backend.write_blob(file_path, buf.getvalue())

    def _write_semantic_segmentation(self, fs_relative_path: str, annotator_data: Any) -> None:
        """
        Writes semantic segmentation data.

        Args:
            fs_relative_path (str): FS relative path where to save semantic segmentatino data.
            annotator_data (str): Annotator data.
        """
        param_val.check_type(fs_relative_path, str)

        sem_seg_data = annotator_data["data"]
        height, width = sem_seg_data.shape[:2]

        file_path = path.join(
            fs_relative_path,
            f"{self.cam_setup_name}_sem_seg_{self._frame_id:0{self._NUMBERING_PADDING}d}.png",
        )
        if sem_seg_data.shape[-1] == 1:
            semantic_seg_data = sem_seg_data.view(np.uint32).reshape(height, width)
            self._backend.write_image(file_path, semantic_seg_data)
        else:
            semantic_seg_data = sem_seg_data.view(np.uint8).reshape(height, width, -1)
            self._backend.write_image(file_path, semantic_seg_data)

        id_to_labels = annotator_data["info"]["idToLabels"]
        file_path = path.join(
            fs_relative_path,
            f"{self.cam_setup_name}_sem_seg_labels_{self._frame_id:0{self._NUMBERING_PADDING}d}.json",
        )
        buf = io.BytesIO()
        buf.write(json.dumps({str(k): v for k, v in id_to_labels.items()}).encode())
        self._backend.write_blob(file_path, buf.getvalue())

    def _write_normals(self, fs_relative_path: str, annotator_data: Any) -> None:
        """
        Writes normals data.

        Args:
            fs_relative_path (str): FS relative path where to save normals data.
            annotator_data (str): Annotator data.
        """
        param_val.check_type(fs_relative_path, str)

        file_path = path.join(
            fs_relative_path,
            f"{self.cam_setup_name}_normals_{self._frame_id:0{self._NUMBERING_PADDING}d}.png",
        )
        colorized_normals_data = colorize_normals(annotator_data)
        self._backend.write_image(file_path, colorized_normals_data)

    def _write_distance_to_camera(self, fs_relative_path: str, annotator_data: Any) -> None:
        """
        Writes distance to camera data.

        Args:
            fs_relative_path (str): FS relative path where to save distance to camera data.
            annotator_data (str): Annotator data.
        """
        param_val.check_type(fs_relative_path, str)

        file_path = path.join(
            fs_relative_path,
            f"{self.cam_setup_name}_dist_to_cam_{self._frame_id:0{self._NUMBERING_PADDING}d}.npy",
        )
        buf = io.BytesIO()
        np.save(buf, annotator_data)
        self._backend.write_blob(file_path, buf.getvalue())

    def _write_distance_to_image_plane(self, fs_relative_path: str, annotator_data: Any) -> None:
        """
        Writes distance to image plane data.

        Args:
            fs_relative_path (str): FS relative path where to save distance to image plane data.
            annotator_data (str): Annotator data.
        """
        param_val.check_type(fs_relative_path, str)

        file_path = path.join(
            fs_relative_path,
            f"{self.cam_setup_name}_dist_to_img_plane_{self._frame_id:0{self._NUMBERING_PADDING}d}.npy",
        )
        buf = io.BytesIO()
        np.save(buf, annotator_data)
        self._backend.write_blob(file_path, buf.getvalue())

    def _write_occlusion(self, fs_relative_path: str, annotator_data: Any) -> None:
        """
        Writes occlusion data.

        Args:
            fs_relative_path (str): FS relative path where to save occlusion data.
            annotator_data (str): Annotator data.
        """
        file_path = path.join(
            fs_relative_path,
            f"{self.cam_setup_name}_occlusion_{self._frame_id:0{self._NUMBERING_PADDING}d}.npy",
        )
        buf = io.BytesIO()
        np.save(buf, annotator_data)
        self._backend.write_blob(file_path, buf.getvalue())

    def _write_motion_vectors(self, fs_relative_path: str, annotator_data: Any) -> None:
        """
        Writes motion vectors data.

        Args:
            fs_relative_path (str): FS relative path where to save motion vectors data.
            annotator_data (str): Annotator data.
        """
        file_path = path.join(
            fs_relative_path,
            f"{self.cam_setup_name}_motion_vector_{self._frame_id:0{self._NUMBERING_PADDING}d}.npy",
        )
        buf = io.BytesIO()
        np.save(buf, annotator_data)
        self._backend.write_blob(file_path, buf.getvalue())


WriterRegistry.register(OffsetWriter)
