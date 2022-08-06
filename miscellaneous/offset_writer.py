__copyright__ = "Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved."
__license__ = """
NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

import io
import json
from typing import List

from omni.replicator.core import WriterRegistry, Writer, AnnotatorRegistry
from omni.replicator.core import BackendDispatch
import numpy as np

from omni.replicator.core.scripts.writers.tools import colorize_segmentation, colorize_normals


class OffsetWriter(Writer):
    """Offset writer which is built upon <BasicWriter>, but has a frames offset before readout the frame. It assumes
    that stage content holds still for number of frames given by <frame_content_lifespan> before some change. This
    offset allows renderer to render everything properly and stabilize quality of RTX and DLSS. It has to be empirically
    determined what is a good offset for given scenario.

    Attributes:
        output_dir:
            Output directory string that indicates the directory to save the results.
        semantic_types:
            List of semantic types to consider when filtering annotator data. Default: ["class"]
        rgb:
            Boolean value that indicates whether the rgb annotator will be activated
            and the data will be written or not. Default: False.
        bounding_box_2d_tight:
            Boolean value that indicates whether the bounding_box_2d_tight annotator will be activated
            and the data will be written or not. Default: False.
        bounding_box_2d_loose:
            Boolean value that indicates whether the bounding_box_2d_loose annotator will be activated
            and the data will be written or not. Default: False.
        semantic_segmentation:
            Boolean value that indicates whether the semantic_segmentation annotator will be activated
            and the data will be written or not. Default: False.
        instance_segmentation:
            Boolean value that indicates whether the instance_segmentation annotator will be activated
            and the data will be written or not. Default: False.
        distance_to_camera:
            Boolean value that indicates whether the distance_to_camera annotator will be activated
            and the data will be written or not. Default: False.
        distance_to_image_plane:
            Boolean value that indicates whether the distance_to_image_plane annotator will be activated
            and the data will be written or not. Default: False.
        bounding_box_3d:
            Boolean value that indicates whether the bounding_box_3d annotator will be activated
            and the data will be written or not. Default: False.
        occlusion:
            Boolean value that indicates whether the occlusion annotator will be activated
            and the data will be written or not. Default: False.
        normals:
            Boolean value that indicates whether the normals annotator will be activated
            and the data will be written or not. Default: False.
        motion_vectors:
            Boolean value that indicates whether the motion_vectors annotator will be activated
            and the data will be written or not. Default: False.
        camera_params:
            Boolean value that indicates whether the camera_params annotator will be activated
            and the data will be written or not. Default: False.
        image_output_format:
            String that indicates the format of saved RGB images. Default: "png"
        colorize_semantic_segmentation:
            If ``True``, semantic segmentation is converted to an image where semantic IDs are mapped to colors
            and saved as a uint8 4 channel PNG image. If ``False``, the output is saved as a uint32 PNG image.
            Defaults to ``True``.
        colorize_instance_segmentation:
            If True, semantic segmentation is converted to an image where semantic IDs are mapped to colors.
            and saved as a uint8 4 channel PNG image. If ``False``, the output is saved as a uint32 PNG image.
            Defaults to ``True``.
    """

    __name__ = "OffsetWriter"

    def __init__(
        self,
        output_dir: str,
        semantic_types: List[str] = None,
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
        image_output_format: str = "png",
        colorize_semantic_segmentation: bool = True,
        colorize_instance_segmentation: bool = True,
        frame_content_lifespan: int = 1,
    ):
        self._output_dir = output_dir
        self._backend = BackendDispatch({"paths": {"out_dir": output_dir}})
        self._frame_id = 0
        self._image_output_format = image_output_format
        self._output_data_format = {}
        self.annotators = []
        self.frame_read_out_num = 1 if frame_content_lifespan == 1 else frame_content_lifespan - 1
        self.content_lifespan = 1

        # Specify the semantic types that will be included in output
        if semantic_types is None:
            semantic_types = ["class"]

        # RGB
        if rgb:
            self.annotators.append(AnnotatorRegistry.get_annotator("rgb"))

        # Bounding Box 2D
        if bounding_box_2d_tight:
            self.annotators.append(
                AnnotatorRegistry.get_annotator("bounding_box_2d_tight", init_params={"semanticTypes": semantic_types})
            )

        if bounding_box_2d_loose:
            self.annotators.append(
                AnnotatorRegistry.get_annotator("bounding_box_2d_loose", init_params={"semanticTypes": semantic_types})
            )

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
                    "instance_segmentation", init_params={"colorize": colorize_instance_segmentation}
                )
            )

        # Depth
        if distance_to_camera:
            self.annotators.append(AnnotatorRegistry.get_annotator("distance_to_camera"))

        if distance_to_image_plane:
            self.annotators.append(AnnotatorRegistry.get_annotator("distance_to_image_plane"))

        # Bounding Box 3D
        if bounding_box_3d:
            self.annotators.append(
                AnnotatorRegistry.get_annotator("bounding_box_3d", init_params={"semanticTypes": semantic_types})
            )

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

    def write(self, data: dict):
        """Write function called from the OgnWriter node on every frame to process annotator output.

        Args:
            data: A dictionary containing the annotator data for the current frame.
        """

        if self.content_lifespan == self.frame_read_out_num:

            if "rgb" in data:
                file_path = f"rgb_{self._frame_id}.{self._image_output_format}"
                self._backend.write_image(file_path, data["rgb"])

            if "normals" in data:
                normals_data = data["normals"]

                file_path = f"normals_{self._frame_id}.png"
                colorized_normals_data = colorize_normals(normals_data)
                self._backend.write_image(file_path, colorized_normals_data)

            if "distance_to_camera" in data:
                dis_to_cam_data = data["distance_to_camera"]

                file_path = f"distance_to_camera_{self._frame_id}.npy"
                buf = io.BytesIO()
                np.save(buf, dis_to_cam_data)
                self._backend.write_blob(file_path, buf.getvalue())

            if "distance_to_image_plane" in data:
                dis_to_img_plane_data = data["distance_to_image_plane"]

                file_path = f"distance_to_image_plane_{self._frame_id}.npy"
                buf = io.BytesIO()
                np.save(buf, dis_to_img_plane_data)
                self._backend.write_blob(file_path, buf.getvalue())

            if "semantic_segmentation" in data:
                semantic_seg_data = data["semantic_segmentation"]["data"]
                height, width = semantic_seg_data.shape[:2]

                file_path = f"semantic_segmentation_{self._frame_id}.png"
                if semantic_seg_data.shape[-1] == 1:
                    semantic_seg_data = semantic_seg_data.view(np.uint32).reshape(height, width)
                    self._backend.write_image(file_path, semantic_seg_data)
                else:
                    semantic_seg_data = semantic_seg_data.view(np.uint8).reshape(height, width, -1)
                    self._backend.write_image(file_path, semantic_seg_data)

                id_to_labels = data["semantic_segmentation"]["info"]["idToLabels"]
                file_path = f"semantic_segmentation_labels_{self._frame_id}.json"
                buf = io.BytesIO()
                buf.write(json.dumps({str(k): v for k, v in id_to_labels.items()}).encode())
                self._backend.write_blob(file_path, buf.getvalue())

            if "instance_segmentation" in data:
                instance_seg_data = data["instance_segmentation"]["data"]
                height, width = instance_seg_data.shape[:2]

                file_path = f"instance_segmentation_{self._frame_id}.png"
                if instance_seg_data.shape[-1] == 1:
                    instance_seg_data = instance_seg_data.view(np.uint32).reshape(height, width)
                    self._backend.write_image(file_path, instance_seg_data)
                else:
                    instance_seg_data = instance_seg_data.view(np.uint8).reshape(height, width, -1)
                    self._backend.write_image(file_path, instance_seg_data)

                id_to_labels = data["instance_segmentation"]["info"]["idToLabels"]
                file_path = f"instance_segmentation_mapping_{self._frame_id}.json"
                buf = io.BytesIO()
                buf.write(json.dumps({str(k): v for k, v in id_to_labels.items()}).encode())
                self._backend.write_blob(file_path, buf.getvalue())

            if "motion_vectors" in data:
                motion_vec_data = data["motion_vectors"]

                file_path = f"motion_vectors_{self._frame_id}.npy"
                buf = io.BytesIO()
                np.save(buf, motion_vec_data)
                self._backend.write_blob(file_path, buf.getvalue())

            if "occlusion" in data:
                occlusion_data = data["occlusion"]

                file_path = f"occlusion_{self._frame_id}.npy"
                buf = io.BytesIO()
                np.save(buf, occlusion_data)
                self._backend.write_blob(file_path, buf.getvalue())

            if "bounding_box_3d" in data:
                bbox_3d_data = data["bounding_box_3d"]["data"]

                id_to_labels = data["bounding_box_3d"]["info"]["idToLabels"]

                file_path = f"bounding_box_3d_{self._frame_id}.npy"
                buf = io.BytesIO()
                np.save(buf, bbox_3d_data)
                self._backend.write_blob(file_path, buf.getvalue())

                labels_file_path = f"bounding_box_3d_labels_{self._frame_id}.json"
                buf = io.BytesIO()
                buf.write(json.dumps(id_to_labels).encode())
                self._backend.write_blob(labels_file_path, buf.getvalue())

            if "bounding_box_2d_loose" in data:
                bbox_2d_loose_data = data["bounding_box_2d_loose"]["data"]

                id_to_labels = data["bounding_box_2d_loose"]["info"]["idToLabels"]

                file_path = f"bounding_box_2d_loose_{self._frame_id}.npy"
                buf = io.BytesIO()
                np.save(buf, bbox_2d_loose_data)
                self._backend.write_blob(file_path, buf.getvalue())

                labels_file_path = f"bounding_box_2d_loose_labels_{self._frame_id}.json"
                buf = io.BytesIO()
                buf.write(json.dumps(id_to_labels).encode())
                self._backend.write_blob(labels_file_path, buf.getvalue())

            if "bounding_box_2d_tight" in data:
                bbox_2d_tight_data = data["bounding_box_2d_tight"]["data"]

                id_to_labels = data["bounding_box_2d_tight"]["info"]["idToLabels"]

                file_path = f"bounding_box_2d_tight_{self._frame_id}.npy"
                buf = io.BytesIO()
                np.save(buf, bbox_2d_tight_data)
                self._backend.write_blob(file_path, buf.getvalue())

                labels_file_path = f"bounding_box_2d_tight_labels_{self._frame_id}.json"
                buf = io.BytesIO()
                buf.write(json.dumps(id_to_labels).encode())
                self._backend.write_blob(labels_file_path, buf.getvalue())

            if "camera_params" in data:
                camera_data = data["camera_params"]

                serializable_data = {}

                for key, val in camera_data.items():
                    if isinstance(val, np.ndarray):
                        serializable_data[key] = val.tolist()
                    else:
                        serializable_data[key] = val

                file_path = f"camera_params_{self._frame_id}.json"

                buf = io.BytesIO()
                buf.write(json.dumps(serializable_data).encode())
                self._backend.write_blob(file_path, buf.getvalue())

            self._frame_id += 1
            self.content_lifespan = 0
        self.content_lifespan += 1


WriterRegistry.register(OffsetWriter)
