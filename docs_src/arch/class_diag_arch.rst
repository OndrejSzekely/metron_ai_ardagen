Class Diagram Architecture
==========================

***************************
Software Class Architecture
***************************

.. mermaid::
    :align: center

    classDiagram

        ScenariosManager o-- IsaacSimApp
        ScenariosManager *-- Scenario
        Scenario o-- IsaacSimApp
        Scenario *-- NullMasterSynthesizer
        Scenario *-- MasterSynthesizer
        Scenario *-- SingleCamera
        MasterSynthesizer o-- IsaacSimApp
        MasterSynthesizer *-- BaseSynthesizer
        OVWriter *-- NullWriter
        Writer <|-- OffsetWriter
        OVWriter *-- OffsetWriter
        OVReplicator o-- IsaacSimApp
        OVReplicator o-- OVWriter
        OVReplicator o-- ScenariosManager

        class IsaacSimApp{
            +bool debug
            +app SimulationApp
            +update() None
            +close() None
        }

        class ScenariosManager{
            +DictConfig scenarios
            +Iterator[Any] scenario_names
            +IsaacSimApp isaac_sim
            +__iter__() ScenariosManager
            +__next__() Scenario
        }
    
        class Scenario{
            +IsaacSimApp isaac_sim
            +Union[NullMasterSynthesizer, MasterSynthesizer] master_synthesizer
            +str scenario_name
            +DictConfig scenario_dict_config
            +int frames_number
            +int frames_readout_offset
            +prepare(scenario_name) None
            +get_cameras() Generator[Tuple[List[Any], List[Any]], None, None]
        }

        class NullMasterSynthesizer{
            +List[Any] synthesizers_worker_names
            +__iter__() Iterator[BaseSynthesizer]
        }

        class MasterSynthesizer{
            +IsaacSimApp isaac_sim_app
            +List[BaseSynthesizer] synthesizers_workers
            +List[str] synthesizers_worker_names
            +__iter__() Iterator[BaseSynthesizer]
            -_instantiate_synthesizer_workers(synthesizer_workers, scenario_name) None
        }

        class BaseSynthesizer{
            +str scenario_owner
            -str __name__
            +__call__(camera_setup) None
            +get_prims() List[str]
            +register_synthesizers_prims(synthesizer_workers) None
        }

        class OVWriter{
            +str fs_store_path
            +Union[NullWriter, Writer]
            +str writer_name
            +bool write_rgb
            +bool write_bounding_box_2d_tight
            +bool write_bounding_box_2d_loose
            +bool write_semantic_segmentation
            +bool write_instance_segmentation
            +bool write_distance_to_camera
            +bool write_distance_to_image_plane
            +bool write_bounding_box_3d
            +bool write_occlusion
            +bool write_normals
            +bool write_motion_vectors
            +bool write_camera_params
            +create(scenario_name, frames_readout_offset) str
            +attach(camera_renders) None
        }

        class NullWriter{
            +create(scenario_name) str
            +attach(camera_renders) None
            +initialize(*args, **kwargs) None
        }

        class OffsetWriter{
            -str __name__
            -int _MAX_FRAMES_OFFSET
            -int _WRITER_NUMBERING_PADDING
            -str _output_dir
            -BackendDispatch _backend
            -int _frame_id
            -str _image_output_format
            -int _content_lifespan
            +List[Any] annotators
            +int frame_read_out_num
            +int content_lifespan_init_val
            -_write_rgb(fs_relative_path, annotator_data) None
            -_write_bounding_box_data(bbox_type, fs_relative_path, annotator_data) None
            -_write_camera_params(fs_relative_path, annotator_data) None
            -_write_instance_segmentation(fs_relative_path, annotator_data) None
            -_write_semantic_segmentation(fs_relative_path, annotator_data) None
            -_write_normals(fs_relative_path, annotator_data) None
            -_write_distance_to_camera(fs_relative_path, annotator_data) None
            -_write_distance_to_image_plane(fs_relative_path, annotator_data) None
            -_write_occlusion(fs_relative_path, annotator_data) None
            -_write_motion_vectors(fs_relative_path, annotator_data) None
            +write(data) None
        }

        class OVReplicator{
            +omni.replicator.core ov_replicator
            +ScenariosManager scenarios_manager
            +OVWriter ov_writer
            +IsaacSimApp isaac_sim
            -_remove_camera(camera_setup, writer_name) None
            -_run_orchestration() None
            +__call__() None
        }

        class SingleCamera{
            +List[float] position
            +List[float] rotation
            +List[float] clipping_range
            +List[int] cam_resolution
            +get_cameras() Generator[Tuple[List[Any], List[Any]], None, None]
        }

**************************************
Synthesizer Workers Class Architecture
**************************************

.. mermaid::
    :align: center

    classDiagram

    BaseSynthesizer <|-- DummySynthesizer
    BaseSynthesizer <|-- GroundSynthesizer
    BaseSynthesizer <|-- ItemsScatterSynthesizer
    BaseSynthesizer <|-- LightSynthesizer
    BaseSynthesizer <|-- SceneSynthesizer
    BaseSynthesizer <|-- SingleItemSynthesizer
    BaseSynthesizer <|-- OVAssetsSynthesizer

    class BaseSynthesizer{
        +str scenario_owner
        -str __name__
        +__call__(camera_setup) None
        +get_prims() List[str]
        +register_synthesizers_prims(synthesizer_workers) None
    }

    class DummySynthesizer{
        +str scenario_owner
        -str __name__
        -str _box_primitive_path
        +__call__(camera_setup) None
        +get_prims() List[str]
        +register_synthesizers_prims(synthesizer_workers) None
    }

    class GroundSynthesizer{
        +str scenario_owner
        +List[str] materials_list
        -str __name__
        -Any _stage
        -str _stage_plane_path
        -og.Node _plane_node
        +__call__(camera_setup) None
        +get_prims() List[str]
        +register_synthesizers_prims(synthesizer_workers) None
    }

    class ItemsScatterSynthesizer{
        +str scenario_owner
        +List[str] placement_synths
        +str semantics
        +int number_of_assets_displayed_at_once
        +int assets_pool_size
        -str __name__
        -List[str] _placement_prims
        -List[str] _scattered_prims
        +__call__(camera_setup) None
        +get_prims() List[str]
        +register_synthesizers_prims(synthesizer_workers) None
    }

    class LightSynthesizer{
        +str scenario_owner
        +List[float] position
        +List[float] rotation
        +List[float] scale
        +str light_type
        -str __name__
        -str _stage_light_path
        -og.node _light_node
        +__call__(camera_setup) None
        +get_prims() List[str]
        +register_synthesizers_prims(synthesizer_workers) None
    }

    class SceneSynthesizer{
        +str scenario_owner
        +str scene_path
        -str __name__
        -og.Node _scene_node
        +__call__(camera_setup) None
        +get_prims() List[str]
        +register_synthesizers_prims(synthesizer_workers) None
        -_load_from_nucleus() Any
    }

    class SingleItemSynthesizer{
        +str scenario_owner
        +List[float] position
        +str usd_path
        +str semantics
        -str __name__
        -str _stage_prim_path
        +__call__(camera_setup) None
        +get_prims() List[str]
        +register_synthesizers_prims(synthesizer_workers) None
    }

    class OVAssetsSynthesizer{
        +str scenario_owner
        +int assets_num_to_generate
        -str __name__
        -Any _stage
        -List[og.Node] _created_assets
        -List[str] _created_assets_paths
        +__call__(camera_setup) None
        +get_prims() List[str]
        +register_synthesizers_prims(synthesizer_workers) None
    }
