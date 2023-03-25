Search.setIndex({"docnames": ["api/metron_ai_ardagen_run", "api/metron_shared", "api/metron_shared.config", "api/metron_shared.config.config", "api/metron_shared.config.config_schema", "api/metron_shared.config.instantiate", "api/metron_shared.io_utils", "api/metron_shared.param_validators", "api/metron_shared.structures", "api/miscellaneous", "api/miscellaneous.custom_exceptions", "api/miscellaneous.metron_ai_ardagen_utils", "api/miscellaneous.offset_writer", "api/synthesizers", "api/synthesizers.master_synthesizer", "api/synthesizers.synth_workers", "api/synthesizers.synth_workers.assets_synths", "api/synthesizers.synth_workers.assets_synths.ov_assets_synthesizer", "api/synthesizers.synth_workers.base_synthesizer", "api/synthesizers.synth_workers.dummy_synthesizer", "api/synthesizers.synth_workers.ground_synthesizer", "api/synthesizers.synth_workers.items_scatter_synthesizer", "api/synthesizers.synth_workers.light_synthesizer", "api/synthesizers.synth_workers.physics_synthesizer", "api/synthesizers.synth_workers.scene_synthesizer", "api/synthesizers.synth_workers.single_item_synthesizer", "api/tools", "api/tools.isaac_sim", "api/tools.replicator", "api/tools.scenario", "api/tools.scenarios_manager", "api/tools.single_camera", "api/tools.writer", "api_docs", "arch/docs_arch", "arch/high_level_arch", "architecture", "development_notes", "foo", "guides", "guides/docs_generation", "index", "project_installation", "scenarios", "synthesizer_workers"], "filenames": ["api/metron_ai_ardagen_run.rst", "api/metron_shared.rst", "api/metron_shared.config.rst", "api/metron_shared.config.config.rst", "api/metron_shared.config.config_schema.rst", "api/metron_shared.config.instantiate.rst", "api/metron_shared.io_utils.rst", "api/metron_shared.param_validators.rst", "api/metron_shared.structures.rst", "api/miscellaneous.rst", "api/miscellaneous.custom_exceptions.rst", "api/miscellaneous.metron_ai_ardagen_utils.rst", "api/miscellaneous.offset_writer.rst", "api/synthesizers.rst", "api/synthesizers.master_synthesizer.rst", "api/synthesizers.synth_workers.rst", "api/synthesizers.synth_workers.assets_synths.rst", "api/synthesizers.synth_workers.assets_synths.ov_assets_synthesizer.rst", "api/synthesizers.synth_workers.base_synthesizer.rst", "api/synthesizers.synth_workers.dummy_synthesizer.rst", "api/synthesizers.synth_workers.ground_synthesizer.rst", "api/synthesizers.synth_workers.items_scatter_synthesizer.rst", "api/synthesizers.synth_workers.light_synthesizer.rst", "api/synthesizers.synth_workers.physics_synthesizer.rst", "api/synthesizers.synth_workers.scene_synthesizer.rst", "api/synthesizers.synth_workers.single_item_synthesizer.rst", "api/tools.rst", "api/tools.isaac_sim.rst", "api/tools.replicator.rst", "api/tools.scenario.rst", "api/tools.scenarios_manager.rst", "api/tools.single_camera.rst", "api/tools.writer.rst", "api_docs.rst", "arch/docs_arch.md", "arch/high_level_arch.md", "architecture.rst", "development_notes.md", "foo.md", "guides.rst", "guides/docs_generation.md", "index.rst", "project_installation.md", "scenarios.md", "synthesizer_workers.md"], "titles": ["metron_ai_ardagen_run module", "metron_shared package", "metron_shared.config package", "metron_shared.config.config module", "metron_shared.config.config_schema module", "metron_shared.config.instantiate module", "metron_shared.io_utils module", "metron_shared.param_validators module", "metron_shared.structures module", "miscellaneous package", "miscellaneous.custom_exceptions module", "miscellaneous.metron_ai_ardagen_utils module", "miscellaneous.offset_writer module", "synthesizers package", "synthesizers.master_synthesizer module", "synthesizers.synth_workers package", "synthesizers.synth_workers.assets_synths package", "synthesizers.synth_workers.assets_synths.ov_assets_synthesizer module", "synthesizers.synth_workers.base_synthesizer module", "synthesizers.synth_workers.dummy_synthesizer module", "synthesizers.synth_workers.ground_synthesizer module", "synthesizers.synth_workers.items_scatter_synthesizer module", "synthesizers.synth_workers.light_synthesizer module", "synthesizers.synth_workers.physics_synthesizer module", "synthesizers.synth_workers.scene_synthesizer module", "synthesizers.synth_workers.single_item_synthesizer module", "tools package", "tools.isaac_sim module", "tools.replicator module", "tools.scenario module", "tools.scenarios_manager module", "tools.single_camera module", "tools.writer module", "API Docs", "Documentation Architecture", "High Level Architecture", "Architecture", "Development Notes", "Heading 1", "Guides", "Documentation Generation Guide", "Metron AI - ArDaGen Docs", "Project Installation", "Scenarios", "Scenarios"], "terms": {"The": [0, 3, 4, 7, 8, 9, 13, 15, 16, 21, 26, 31, 34, 35, 42], "script": [0, 42], "run": [0, 11, 27, 35, 40, 42], "metron": [0, 3, 7, 11, 26, 34, 35, 37], "ai": [0, 11, 26, 34, 35], "ardagen": [0, 11, 26, 27, 34, 35, 42, 43], "main": [0, 3, 4, 34], "hydra_config": [0, 3], "omegaconf": [0, 4, 5, 14, 29, 30], "dictconfig": [0, 3, 4, 5, 14, 29, 30], "none": [0, 3, 4, 5, 6, 7, 11, 12, 14, 17, 18, 19, 20, 21, 22, 24, 25, 27, 29, 30, 31, 32], "function": [0, 3, 4, 7, 8, 11, 12, 18, 20, 21, 24], "compon": [0, 3, 7, 12, 35, 40], "paramet": [0, 3, 4, 5, 7, 8, 10, 11, 12, 14, 17, 18, 19, 20, 21, 22, 24, 25, 27, 29, 30, 31, 32, 35], "configur": [0, 3, 14, 29, 34, 35], "provid": [0, 3, 4, 7, 8, 17, 35, 40], "hydra": [0, 3, 4, 5, 14, 29, 35], "s": [0, 3, 4, 5, 7, 11, 12, 14, 17, 18, 19, 20, 21, 22, 24, 25, 29, 31, 32, 34, 35, 40, 42, 43], "config": [0, 7, 14, 17, 18, 19, 20, 21, 22, 24, 25, 29, 30], "return": [0, 3, 4, 5, 7, 8, 10, 14, 17, 18, 19, 20, 21, 22, 24, 25, 27, 30, 31, 32], "config_schema": [], "instanti": [8, 14, 29, 30, 32], "util": [6, 9], "i": 6, "o": [6, 38, 40], "force_folder_cr": 6, "fs_folder_path": 6, "str": [5, 6, 7, 10, 11, 12, 14, 17, 18, 19, 20, 21, 22, 24, 25, 29, 30, 32], "valid": [4, 7], "common": 7, "across": [7, 8, 18], "scope": 7, "all": [5, 7, 13, 14, 15, 17, 18, 20, 21, 24, 25, 29, 30, 34, 35, 40, 43], "except": [7, 8, 10, 42], "yaml": 7, "check_file_exist": 7, "file_path": 7, "file": [4, 7, 10, 11, 12, 32, 36, 42], "path": [7, 11, 12, 17, 18, 19, 20, 21, 22, 24, 25, 29, 31, 32, 42], "exist": [7, 35], "oserror": [7, 10], "If": [7, 8, 12, 27], "doe": [4, 7, 8, 14, 35], "check_folder_exist": 7, "folder_path": 7, "folder": [7, 11, 12, 32, 34, 40, 42], "check_length_of_list": 7, "list_inst": 7, "list": [5, 7, 12, 14, 17, 18, 19, 20, 21, 22, 24, 25, 29, 30, 31, 32, 34, 42], "ani": [3, 4, 5, 7, 8, 12, 14, 17, 20, 24, 29, 31, 32, 42], "expected_list_len": 7, "int": [7, 12, 17, 20, 21, 29, 31, 32], "check": [7, 8, 43], "given": [3, 7, 11, 12, 17, 20, 21, 22, 24, 25, 29, 31, 32, 34], "ha": [3, 4, 5, 7, 11, 12, 14, 17, 18, 35, 42], "expect": [4, 7], "number": [7, 12, 17, 21, 29, 31], "item": [7, 21, 25, 31, 38, 43], "contain": [5, 7, 9, 11, 12, 13, 15, 16, 17, 18, 19, 20, 21, 22, 24, 25, 26, 29, 30, 31, 34], "valueerror": 7, "mismatch": 7, "check_parameter_value_in_rang": 7, "param_valu": 7, "union": [5, 7, 29, 32], "float": [7, 20, 22, 25, 31], "lower_bound": 7, "upper_bound": 7, "valu": [4, 5, 7, 12, 31], "rang": [7, 31], "lower": 7, "bound": [7, 12, 32, 43], "allow": [7, 8, 12, 17, 18, 19, 20, 21, 22, 24, 25, 27, 34, 42], "upper": 7, "check_typ": 7, "variabl": [3, 7, 12], "expected_typ": 7, "type": [3, 4, 7, 8, 10, 12, 14, 17, 18, 19, 20, 21, 22, 24, 25, 27, 29, 30, 31, 32, 35], "typeerror": 7, "rais": [7, 11], "thi": [3, 4, 8, 10, 11, 12, 18, 20, 21, 24, 32, 34, 37, 43, 44], "data": [4, 8, 12, 32, 35], "pattern": [8, 14, 32], "class": [3, 5, 8, 12, 14, 17, 18, 19, 20, 21, 22, 23, 24, 25, 27, 28, 29, 30, 31, 32], "decor": [3, 4, 8], "singleton": 8, "decorated_object": 8, "base": [3, 5, 8, 10, 12, 14, 17, 18, 19, 20, 21, 22, 24, 25, 27, 29, 30, 31, 32, 34, 35, 42], "object": [3, 5, 8, 12, 14, 18, 21, 27, 29, 30, 31, 32, 35, 42], "repres": [5, 8, 14, 32, 35], "applic": [8, 35, 42], "method": [3, 8, 18, 19, 30], "when": [3, 8, 11, 12], "appli": [3, 8], "onli": [4, 8, 14, 35, 42], "one": [4, 8, 17, 18, 19, 20, 21, 22, 24, 25, 29, 31, 35, 43], "instanc": [3, 8, 12, 14, 29, 30, 32, 35], "whole": [8, 35, 42], "program": 8, "lifespan": 8, "call": [3, 4, 8, 12, 17, 19, 22, 25, 30, 38], "can": [8, 11, 14, 17, 18, 19, 20, 21, 22, 24, 25, 34, 35], "more": [8, 17, 18, 19, 20, 21, 22, 24, 25, 29, 35], "It": [3, 4, 5, 8, 12, 14, 17, 18, 19, 20, 21, 22, 24, 25, 32, 34, 35, 40, 42, 43], "chang": [8, 12, 17, 19, 22, 25, 42, 43], "_call": 8, "set": [8, 12, 42], "which": [3, 4, 5, 8, 12, 14, 17, 18, 19, 20, 21, 22, 23, 24, 25, 29, 30, 31, 35, 40, 42, 43], "keep": 8, "us": [3, 4, 8, 10, 14, 18, 19, 24, 32, 35, 36, 40, 42, 43], "_function": 8, "set_hydra_config": [3, 8], "defin": [3, 5, 10, 12, 14, 17, 18, 19, 20, 21, 22, 23, 24, 25, 29, 30, 31, 32, 34, 35], "store": [3, 12, 29, 32, 34, 37], "obtain": 3, "framework": [3, 4, 34, 35, 40], "initi": [3, 14, 32, 35], "right": 3, "after": [3, 11, 12, 43], "see": [3, 4, 5, 34, 35, 40, 42], "sampl": [3, 4, 17, 21, 42], "code": [3, 4, 34, 35, 42, 43], "config_path": [3, 4], "config_nam": [3, 4], "def": [3, 4], "cfg": [3, 4], "gethydraconfig": 3, "pass": [3, 4, 5, 12, 30], "first": [3, 31, 38, 40], "argument": [3, 4, 5, 8, 17], "do_someth": 3, "decorated_func": 3, "get": [3, 10, 14, 40], "global": 3, "might": [3, 35, 42], "_decorated_func": 3, "create_structured_config_schema": 4, "creat": [4, 5, 8, 17, 18, 19, 20, 21, 22, 24, 25, 32, 35, 40], "structur": [4, 34], "schema": 4, "presenc": 4, "registr": 4, "an": [4, 12, 17, 18, 19, 20, 21, 22, 24, 25, 38, 43, 44], "befor": [4, 12, 32], "schema_registration_function_to_be_cal": 4, "other_decor": 4, "schema_registering_funct": 4, "callabl": 4, "core": [4, 12, 42], "config_stor": 4, "configstor": 4, "actual": [4, 32], "regist": [4, 18, 19, 42], "done": 4, "advantag": 4, "rather": 4, "than": [4, 17, 18, 19, 20, 21, 22, 24, 25], "transpar": 4, "from": [4, 5, 12, 17, 20, 21, 22, 24, 32, 35, 38, 42], "usag": 4, "perspect": 4, "have": [4, 42], "input": [3, 4, 17], "hydrainstantiateconvers": 5, "enum": 5, "convers": 5, "option": [5, 12, 34, 42], "basic": [5, 35, 43], "how": [5, 12, 29, 34, 40, 42], "handl": [5, 35], "dict": [5, 12, 14, 17, 18, 19, 20, 21, 22, 24, 25, 30, 42], "like": 5, "whether": [5, 14, 27], "thei": [5, 34], "ar": [5, 12, 17, 21, 29, 31, 34, 37, 40, 42], "via": [5, 34, 35], "struct": 5, "python": [5, 40, 42], "no_convers": 5, "partial": 5, "instantiate_from_hydra_config": 5, "hydra_object_config": 5, "kwarg": [3, 5, 8, 12, 32], "_target_": 5, "attribut": [3, 5, 8, 14, 42], "non": [5, 14, 32, 40], "primit": [5, 17, 18, 19, 20, 21, 22, 24, 25, 35], "http": [5, 40, 42], "cc": 5, "doc": [5, 36], "advanc": 5, "instantiate_object": 5, "overview": [5, 35, 43, 44], "strategi": 5, "kei": [3, 5, 8], "word": [3, 5, 8], "constructor": [5, 21], "custom": [10, 34], "ovextnotload": [10, 11], "ext_nam": 10, "omnivers": [10, 17, 18, 19, 26, 28, 35, 42], "extens": [10, 11, 36], "wa": [8, 10, 42], "load": [10, 11, 19, 24, 25, 34], "messag": 10, "small": 11, "domain": 11, "free": 11, "specif": [11, 26, 35], "load_ov_extens": 11, "ext_path": 11, "extension_nam": 11, "invok": 11, "isaac": [11, 14, 17, 20, 27, 29, 42, 43], "sim": [11, 14, 17, 20, 27, 29, 30, 42, 43], "app": [11, 14, 27, 30], "root": [11, 40, 42], "name": [10, 11, 12, 14, 17, 18, 19, 20, 21, 22, 24, 25, 29, 30, 32, 43], "ov": [10, 11, 12, 17, 24, 27, 32], "t": [11, 14, 24, 27, 38, 40], "implement": [12, 17, 18, 27, 28, 29, 30, 43, 44], "writer": 12, "offset": [12, 32], "offsetwrit": 12, "arg": [3, 8, 12], "built": 12, "upon": 12, "basicwrit": 12, "frame": [12, 32], "readout": [12, 32], "assum": 12, "stage": [12, 16, 17, 18, 19, 20, 21, 22, 24, 25, 29, 31], "content": 12, "hold": 12, "still": [12, 43], "frame_content_lifespan": 12, "some": 12, "render": [12, 29, 31, 32], "everyth": [12, 42], "properli": 12, "stabil": 12, "qualiti": 12, "rtx": 12, "dlss": 12, "empir": 12, "determin": 12, "what": [12, 29, 30], "good": [12, 40], "scenario": [12, 14, 17, 18, 20, 21, 22, 24, 25, 30, 32], "__name__": [12, 18, 19], "_max_frames_offset": 12, "maxim": 12, "_writer_numbering_pad": 12, "lead": 12, "zero": 12, "save": [3, 8, 12, 32, 35], "_output_dir": 12, "output": [12, 32], "directori": [12, 40], "string": [10, 12, 38], "indic": 12, "result": 12, "_backend": 12, "backend": 12, "dispatch": 12, "backenddispatch": 12, "_frame_id": 12, "index": [12, 34], "_image_output_format": 12, "imag": [12, 29, 32, 42], "format": [12, 34], "_content_lifespan": 12, "intern": 12, "aux": 12, "loop": 12, "mani": 12, "sinc": 12, "last": 12, "read": 12, "out": [12, 42], "annot": [12, 24, 32, 35, 43], "frame_read_out_num": 12, "For": [12, 35], "1": [12, 40], "everi": [12, 34], "becaus": [12, 14], "we": 12, "want": 12, "th": 12, "content_lifespan_init_v": 12, "init": [12, 29, 30, 31, 32], "helper": 12, "decid": 12, "perform": [12, 35, 42], "new": [12, 32, 40], "depend": [12, 40], "singl": [12, 25, 31, 43], "0": [12, 40, 42], "otherwis": 12, "180": 12, "3": [12, 31, 38, 40], "_write_bounding_box_data": 12, "bbox_typ": 12, "fs_relative_path": 12, "annotator_data": 12, "write": [12, 32], "box": [12, 32, 42, 43], "specifi": 12, "bbox": [12, 32], "fs": 12, "rel": 12, "where": 12, "_write_camera_param": 12, "camera": [12, 17, 18, 19, 20, 21, 22, 24, 25, 29, 31, 32, 43], "_write_distance_to_camera": 12, "distanc": 12, "_write_distance_to_image_plan": 12, "plane": [12, 20, 32, 35], "_write_instance_segment": 12, "segment": [12, 32], "segmentatino": 12, "_write_motion_vector": 12, "motion": [12, 32], "vector": [12, 32], "_write_norm": 12, "normal": [12, 32], "_write_occlus": 12, "occlus": 12, "_write_rgb": 12, "rgb": [12, 32], "_write_semantic_segment": 12, "semant": [12, 20, 21, 25, 32], "ognwrit": 12, "node": [12, 17, 20, 22, 24], "process": 12, "A": [12, 34], "dictionari": [12, 20], "current": [12, 17, 20, 42], "metron_ai_ardagen_run": [], "modul": [35, 38, 40, 43], "metron_shar": [], "packag": [35, 38], "subpackag": 41, "submodul": [41, 42], "io_util": [], "param_valid": [], "miscellan": [], "custom_except": [], "metron_ai_ardagen_util": [], "offset_writ": [], "synthes": [29, 44], "synth_work": [], "base_synthes": [], "dummy_synthes": [], "ground_synthes": [], "items_scatter_synthes": [], "light_synthes": [], "physics_synthes": [], "scene_synthes": [], "single_item_synthes": [], "master_synthes": 29, "tool": [9, 34, 40], "isaac_sim": [14, 29, 30, 42], "replic": [12, 17, 18, 19, 22, 25, 27], "scenarios_manag": [], "single_camera": [], "assets_synth": [], "ov_assets_synthes": [], "master": [13, 14, 29], "virtual": [14, 40], "world": [14, 38], "orchestr": [14, 35], "manag": [14, 30, 40], "worker": [14, 18, 44], "mastersynthes": [14, 29], "isaacsimapp": [14, 27, 29, 30], "synthesizer_work": [14, 17, 18, 19, 20, 21, 22, 24, 25], "scenario_nam": [14, 29, 30, 32], "isaac_sim_app": 14, "synthesizers_work": 14, "basesynthes": [14, 17, 18, 19, 20, 21, 22, 24, 25], "synthesizers_worker_nam": 14, "_instantiate_synthesizer_work": 14, "enabl": 14, "directli": 14, "rid": 14, "each": [12, 14, 17, 32, 35], "meant": 14, "dure": 14, "state": [14, 34], "own": [14, 17, 18, 20, 21, 22, 24, 25], "nullmastersynthes": [14, 29], "follow": [14, 32, 34, 35, 40, 42], "null": [14, 29, 32], "design": [14, 32, 35, 40], "empti": [14, 30], "interfac": [14, 17, 18, 32, 42], "class_nam": [17, 18, 19, 20, 21, 22, 24, 25], "scenario_own": [17, 18, 19, 20, 21, 22, 24, 25], "__call__": [3, 8, 17, 18, 19, 20, 21, 22, 24, 25], "magic": [18, 19, 20, 21, 24, 30], "same": 18, "abstract": 18, "get_prim": [17, 18, 19, 20, 21, 22, 24, 25], "prim": [17, 18, 19, 20, 21, 22, 24, 25], "register_synthesizers_prim": [17, 18, 19, 20, 21, 22, 24, 25], "access": [17, 18, 19, 20, 21, 22, 24, 25], "other": [17, 18, 19, 20, 21, 22, 24, 25, 34, 40, 42], "need": [3, 8, 17, 18, 19, 20, 21, 22, 24, 25], "dummi": 19, "respons": [17, 19, 20, 21, 22, 23, 24, 25], "scene": [12, 17, 19, 21, 22, 24, 25, 27, 35, 43], "dummysynthes": 19, "_box_primitive_path": 19, "sm_cardboxa_3": 19, "ground": [20, 23, 35, 43], "synthesi": [20, 22, 23, 25], "groundsynthes": 20, "posit": [3, 8, 20, 22, 25, 31], "materi": [20, 35], "scale": [20, 22], "_stage": [17, 20], "_stage_plane_path": 20, "_plane_nod": 20, "omnigraph": [20, 22], "represent": [10, 20, 22], "og": [17, 20, 22, 24], "materials_list": 20, "nucelu": 20, "scatter": 21, "placement": [21, 25, 35], "itemsscattersynthes": 21, "asset": [16, 17, 21, 25, 35], "number_of_assets_displayed_at_onc": 21, "assets_pool_s": 21, "placement_synth": 21, "_placement_prim": 21, "correspond": [21, 29, 31, 34], "_scattered_prim": 21, "ad": [17, 21, 25], "pool": [17, 20, 21], "target": 21, "place": [21, 42], "displai": 21, "onc": 21, "light": [22, 35], "sourc": [22, 36], "lightsynthes": 22, "rotat": [22, 31], "light_typ": 22, "_stage_light_path": 22, "_light_nod": 22, "xyz": 22, "coordin": [20, 22, 25], "euler": 22, "angl": 22, "degre": [22, 35], "order": 22, "factor": 22, "ax": 22, "select": [20, 22], "cylind": 22, "disk": 22, "distant": 22, "dome": 22, "rect": 22, "sphere": 22, "physic": [23, 35], "scenesynthes": 24, "scene_path": 24, "_scene_nod": 24, "usd": [17, 24, 25, 43], "insid": [12, 24, 32, 35], "nucleu": [24, 25], "_load_from_nucleu": 24, "server": [24, 40], "refer": [24, 29], "singleitemsynthes": 25, "usd_path": 25, "x": [20, 25], "y": [20, 25, 40], "z": [12, 20, 25], "_stage_prim_path": 25, "3d": [12, 17, 31, 32, 35], "ovassetssynthes": 17, "assets_num_to_gener": 17, "TO": 17, "IN": 17, "__init__": [3, 8, 10, 12, 14, 17, 18, 19, 20, 21, 22, 24, 25, 27, 29, 30, 31, 32], "add": [12, 17, 32], "_created_asset": 17, "_created_assets_path": 17, "wrapper": [26, 27, 29, 30, 35], "debug": [27, 42], "bool": [12, 27, 32], "simulationapp": 27, "true": [12, 27, 32], "won": 27, "execut": [18, 20, 21, 24, 27, 29, 30, 35, 40], "user": [17, 20, 21, 22, 24, 25, 27, 41], "inspect": 27, "gener": [12, 16, 27, 29, 30, 31, 32, 35, 36, 39, 41], "close": 27, "updat": [27, 34], "handler": [28, 35], "scenario_dict_config": 29, "descript": [12, 29, 32, 34, 36, 43, 44], "should": [29, 30], "till": [29, 30], "duck": [14, 29, 32], "frames_numb": 29, "get_camera": [29, 31], "tupl": [12, 29, 31], "collect": 29, "setup": [17, 18, 19, 20, 21, 22, 24, 25, 29, 31, 32, 35, 42], "mean": [12, 29, 31, 35], "could": [29, 34, 35], "e": [17, 18, 19, 20, 21, 22, 24, 25, 29, 35, 40], "g": [17, 18, 19, 20, 21, 22, 24, 25, 29, 35, 40], "stereo": [17, 18, 19, 20, 21, 22, 24, 25, 29, 35], "yield": [29, 31], "product": [29, 31], "prepar": 29, "encapsul": [29, 40], "iter": [14, 30], "over": [14, 30], "scenariosmanag": 30, "__iter__": [14, 30], "issac": 30, "singlecamera": 31, "clipping_rang": 31, "resolut": 31, "clip": 31, "tuppl": 31, "clipping_min": 31, "clipping_max": 31, "cam_resolut": 31, "two": [31, 42], "both": 31, "second": 31, "ovwrit": 32, "system": 32, "nullwrit": 32, "attach": [32, 40], "camera_rend": 32, "belong": 32, "later": [3, 8, 32], "delet": 32, "fs_store_path": 32, "write_rgb": 32, "write_bounding_box_2d_tight": 32, "write_bounding_box_2d_loos": 32, "write_semantic_segment": 32, "write_instance_segment": 32, "write_distance_to_camera": 32, "write_distance_to_image_plan": 32, "write_bounding_box_3d": 32, "write_occlus": 32, "write_norm": 32, "write_motion_vector": 32, "write_camera_param": 32, "writertempl": 32, "writer_nam": [12, 32, 42], "2d": [12, 32], "cover": [12, 32], "obscur": [12, 32], "part": [12, 32], "also": [12, 32, 35, 40, 42], "segmentaion": [12, 32], "invers": [12, 32], "depth": [12, 32], "map": [12, 32], "todo": [12, 32, 42], "pixel": [12, 32], "intrins": [12, 32, 35], "extrins": [12, 32], "captur": 32, "frames_readout_offset": 32, "sepeart": 32, "nvidia": [35, 42], "work": [3, 8, 35, 42, 43], "just": 35, "On": 35, "top": 35, "meta": 35, "reus": 35, "share": 35, "adress": 35, "easi": 35, "flexibl": 35, "standard": 35, "linux": [35, 42], "os": 35, "ubuntu": [35, 42], "gpu": [35, 42], "power": 35, "machin": 35, "solut": [35, 42], "consist": 35, "eight": 35, "around": 35, "platform": 35, "oper": 35, "destruct": 35, "simplest": 35, "case": 35, "complex": 35, "360": 35, "view": 35, "pinhol": 35, "fishey": 35, "layer": 35, "contribut": 35, "complet": 35, "foreground": 35, "etc": [35, 40], "random": [35, 43], "so": 35, "artifici": 35, "record": 35, "differ": [35, 42], "multipl": 35, "seri": 35, "variou": [26, 35], "optic": 35, "In": [35, 40, 42], "therefor": 35, "filesystem": 35, "sice": 35, "page": [37, 42], "project": [34, 37, 40, 41], "high": [36, 41], "hallo": 38, "admonit": [34, 38], "note": [38, 41], "foo": 38, "link": [38, 42], "import": 38, "term": 38, "pi": 38, "14159": 38, "caption": 38, "header": 38, "2": [38, 40, 42], "b": [38, 40], "sphinx": [36, 38, 40], "apidoc": [34, 38, 40], "docs_src": [34, 38, 40], "api": [36, 38, 41], "f": [38, 40], "_templat": [34, 38, 40], "privat": [38, 40], "architectur": 41, "instal": 41, "end": 41, "docker": [], "compos": [], "develop": 41, "bug": 41, "fix": 41, "detach": [], "fail": [], "There": [42, 43], "wai": 42, "guid": [34, 41, 42], "step": 42, "recommend": [40, 42], "requir": [34, 42], "To": 42, "abl": 42, "pull": 42, "instruct": 42, "21": 42, "04": 42, "test": 42, "window": 42, "download": 42, "launcher": 42, "here": [34, 42], "microsoft": 42, "visual": 42, "studio": 42, "id": [12, 42], "support": [34, 42], "nor": 42, "repositori": [40, 42], "git": [34, 42], "clone": 42, "recurs": 42, "github": 42, "com": 42, "ondrejszek": 42, "metron_ai_ardagen": 42, "move": 42, "command": [18, 20, 21, 24, 40, 42], "your": [40, 42], "mv": 42, "ssd_crucial": 42, "2022": 42, "shapenet": 42, "shapenetcor": 42, "v2": 42, "dataset": 42, "you": [40, 42], "arbitrati": 42, "locat": [20, 34, 42], "fast": 42, "storag": 42, "start": 42, "go": [40, 42], "open": [40, 42], "predefin": 42, "explor": 42, "tab": 42, "present": 42, "els": 42, "metron_ai_garden": 42, "irrelev": 42, "Be": 42, "awar": 42, "append": 42, "analysi": 42, "extrapath": 42, "isaac_sim_root_folder_path": 42, "vscode": 42, "json": 42, "miss": 42, "sh": 42, "m": [40, 42], "pip": [40, 42], "r": [40, 42], "txt": [40, 42], "discard": 42, "librari": 42, "ext": 42, "omni": [12, 42], "cp37": 42, "writer_registri": 42, "py": [34, 42], "line": 42, "line_num": 42, "version": 42, "cl": 42, "_render_product_writer_map": 42, "render_product": 42, "pop": 42, "section": [34, 43, 44], "show": [43, 44], "its": 43, "suggest": 43, "veri": 43, "fine": 43, "full_warehous": 43, "depict": 43, "fraction": 43, "focu": 43, "spred": 43, "element": 43, "hide": 43, "closest": 43, "time": 43, "produc": 43, "tight": 43, "level": [36, 41], "decis": 36, "rig": [17, 18, 19, 20, 21, 22, 24, 25], "detail": 35, "sdf": [], "document": [36, 39, 41], "descib": 34, "approach": 34, "markedli": 34, "text": 34, "myst": [34, 40], "www": [], "org": [], "en": [], "html": [34, 40], "pars": 34, "rich": 34, "flavour": 34, "markdown": 34, "rst": 34, "md": 34, "extend": 34, "sf": [], "arch": 34, "diagram_project": 34, "img": 34, "conf": 34, "templat": 34, "autogener": 34, "resid": 34, "draw": 34, "io": 34, "diagram": 34, "export": 34, "pictur": 34, "parser": [34, 40], "webpag": 34, "guidelin": 34, "automat": 36, "commit": 34, "hook": 34, "pre": 34, "manual": 34, "toctre": [], "maxdepth": [], "docs_gener": [], "documen": [], "maintain": 34, "jinja": 34, "engin": 34, "environ": 40, "dev": 40, "softwar": 40, "7": 40, "conda": 40, "pipenv": 40, "anaconda": 40, "channel": [12, 40], "n": 40, "metron_dev_37": 40, "further": 40, "activ": 40, "point_right": [], "found": 34, "short": 34, "individu": [], "shell": [], "tip": [], "sphinxemoji": [34, 40], "syntax": 34, "emoticon_nam": 34, "wrap": 34, "special": 34, "eval": 34, "emoticon": 36, "clap": [], "_": [], "bulb": [], "5": 40, "book": 40, "theme": 40, "phinx": 40, "togglebutton": 40, "c": 40, "forg": 40, "requirements_doc": 40, "4444": 40, "Then": 40, "browser": 40, "localhost": 40, "job": 40, "latest": 34, "cli": 34, "copybutton": 40, "tocfil": [], "api_doc": [], "_summary_": [], "foooooo": [], "spreadsheet": [], "column": [], "printer": [], "print": [], "consol": [], "row": [], "accept": [], "comma": [], "separ": [], "csv": [], "well": [], "excel": [], "xl": [], "xlsx": [], "panda": [], "within": [], "get_spreadsheet_col": [], "d": 40, "toc": 40, "take": 3, "__dict__": [], "mappingproxi": [], "__module__": [], "__doc__": [], "__get__": [3, 8], "__weakref__": [], "__annotations__": [8, 12], "owner": [3, 8, 14], "lookup": [3, 8], "weak": [], "alreadi": 8, "itself": [8, 30], "without": 8, "violenc": 8, "paradigm": 8, "__str__": 10, "error": 10, "__sphinx_decorator_args__": 12, "output_dir": 12, "semantic_typ": 12, "fals": 12, "bounding_box_2d_tight": 12, "bounding_box_2d_loos": 12, "semantic_segment": 12, "instance_segment": 12, "distance_to_camera": 12, "distance_to_image_plan": 12, "bounding_box_3d": 12, "motion_vector": 12, "camera_param": 12, "image_output_format": 12, "png": 12, "colorize_semantic_segment": 12, "colorize_instance_segment": 12, "kept": 12, "default": 12, "exact": 12, "convert": 12, "color": 12, "uint8": 12, "4": 12, "uint32": 12, "consid": 12, "filter": 12, "__orig_bases__": 12, "camera_setup": [17, 18, 19, 20, 21, 22, 24, 25], "make": [17, 19, 22, 25], "complic": [17, 18, 19, 20, 21, 22, 24, 25], "With": [18, 20, 21, 24], "ramdom": 20, "taken": 20, "chosen": 21, "randomli": 21, "switch": 27, "mode": 27, "head": 27, "__next__": 30, "next": 30, "describ": 30, "blah": []}, "objects": {"": [[0, 0, 0, "-", "metron_ai_ardagen_run"], [1, 0, 0, "-", "metron_shared"], [9, 0, 0, "-", "miscellaneous"], [13, 0, 0, "-", "synthesizers"], [26, 0, 0, "-", "tools"]], "metron_ai_ardagen_run": [[0, 1, 1, "", "main"]], "metron_shared": [[2, 0, 0, "-", "config"], [6, 0, 0, "-", "io_utils"], [7, 0, 0, "-", "param_validators"], [8, 0, 0, "-", "structures"]], "metron_shared.config": [[3, 0, 0, "-", "config"], [4, 0, 0, "-", "config_schema"], [5, 0, 0, "-", "instantiate"]], "metron_shared.config.config": [[3, 2, 1, "", "GetHydraConfig"]], "metron_shared.config.config.GetHydraConfig": [[3, 3, 1, "", "__call__"], [3, 3, 1, "", "__get__"], [3, 3, 1, "", "__init__"], [3, 4, 1, "", "_decorated_func"]], "metron_shared.config.config_schema": [[4, 1, 1, "", "create_structured_config_schema"]], "metron_shared.config.instantiate": [[5, 2, 1, "", "HydraInstantiateConversion"], [5, 1, 1, "", "instantiate_from_hydra_config"]], "metron_shared.config.instantiate.HydraInstantiateConversion": [[5, 4, 1, "", "ALL"], [5, 4, 1, "", "NO_CONVERSION"], [5, 4, 1, "", "PARTIAL"]], "metron_shared.io_utils": [[6, 1, 1, "", "force_folder_create"]], "metron_shared.param_validators": [[7, 1, 1, "", "check_file_existence"], [7, 1, 1, "", "check_folder_existence"], [7, 1, 1, "", "check_length_of_list"], [7, 1, 1, "", "check_parameter_value_in_range"], [7, 1, 1, "", "check_type"]], "metron_shared.structures": [[8, 2, 1, "", "Singleton"]], "metron_shared.structures.Singleton": [[8, 4, 1, "", "__annotations__"], [8, 3, 1, "", "__call__"], [8, 3, 1, "", "__get__"], [8, 3, 1, "", "__init__"], [8, 4, 1, "id0", "_called"], [8, 4, 1, "", "_function"]], "miscellaneous": [[10, 0, 0, "-", "custom_exceptions"], [11, 0, 0, "-", "metron_ai_ardagen_utils"], [12, 0, 0, "-", "offset_writer"]], "miscellaneous.custom_exceptions": [[10, 5, 1, "", "OVExtNotLoaded"]], "miscellaneous.custom_exceptions.OVExtNotLoaded": [[10, 3, 1, "", "__init__"], [10, 3, 1, "", "__str__"], [10, 4, 1, "", "message"]], "miscellaneous.metron_ai_ardagen_utils": [[11, 1, 1, "", "load_ov_extension"]], "miscellaneous.offset_writer": [[12, 2, 1, "", "OffsetWriter"]], "miscellaneous.offset_writer.OffsetWriter": [[12, 4, 1, "id0", "_MAX_FRAMES_OFFSET"], [12, 4, 1, "id1", "_WRITER_NUMBERING_PADDING"], [12, 4, 1, "", "__annotations__"], [12, 3, 1, "", "__init__"], [12, 4, 1, "id2", "__name__"], [12, 4, 1, "", "__orig_bases__"], [12, 4, 1, "", "_backend"], [12, 4, 1, "", "_content_lifespan"], [12, 4, 1, "", "_frame_id"], [12, 4, 1, "", "_image_output_format"], [12, 4, 1, "", "_output_dir"], [12, 3, 1, "", "_write_bounding_box_data"], [12, 3, 1, "", "_write_camera_params"], [12, 3, 1, "", "_write_distance_to_camera"], [12, 3, 1, "", "_write_distance_to_image_plane"], [12, 3, 1, "", "_write_instance_segmentation"], [12, 3, 1, "", "_write_motion_vectors"], [12, 3, 1, "", "_write_normals"], [12, 3, 1, "", "_write_occlusion"], [12, 3, 1, "", "_write_rgb"], [12, 3, 1, "", "_write_semantic_segmentation"], [12, 4, 1, "", "annotators"], [12, 4, 1, "", "content_lifespan_init_val"], [12, 4, 1, "", "frame_read_out_num"], [12, 3, 1, "", "write"]], "synthesizers": [[14, 0, 0, "-", "master_synthesizer"], [15, 0, 0, "-", "synth_workers"]], "synthesizers.master_synthesizer": [[14, 2, 1, "", "MasterSynthesizer"], [14, 2, 1, "", "NullMasterSynthesizer"]], "synthesizers.master_synthesizer.MasterSynthesizer": [[14, 3, 1, "", "__init__"], [14, 3, 1, "", "__iter__"], [14, 3, 1, "", "_instantiate_synthesizer_workers"], [14, 4, 1, "", "isaac_sim_app"], [14, 4, 1, "", "synthesizers_worker_names"], [14, 4, 1, "", "synthesizers_workers"]], "synthesizers.master_synthesizer.NullMasterSynthesizer": [[14, 3, 1, "", "__init__"], [14, 3, 1, "", "__iter__"], [14, 4, 1, "", "synthesizers_worker_names"]], "synthesizers.synth_workers": [[16, 0, 0, "-", "assets_synths"], [18, 0, 0, "-", "base_synthesizer"], [19, 0, 0, "-", "dummy_synthesizer"], [20, 0, 0, "-", "ground_synthesizer"], [21, 0, 0, "-", "items_scatter_synthesizer"], [22, 0, 0, "-", "light_synthesizer"], [23, 0, 0, "-", "physics_synthesizer"], [24, 0, 0, "-", "scene_synthesizer"], [25, 0, 0, "-", "single_item_synthesizer"]], "synthesizers.synth_workers.assets_synths": [[17, 0, 0, "-", "ov_assets_synthesizer"]], "synthesizers.synth_workers.assets_synths.ov_assets_synthesizer": [[17, 2, 1, "", "OVAssetsSynthesizer"]], "synthesizers.synth_workers.assets_synths.ov_assets_synthesizer.OVAssetsSynthesizer": [[17, 3, 1, "", "__call__"], [17, 3, 1, "", "__init__"], [17, 4, 1, "", "_created_assets"], [17, 4, 1, "", "_created_assets_paths"], [17, 4, 1, "", "_stage"], [17, 4, 1, "", "assets_num_to_generate"], [17, 3, 1, "", "get_prims"], [17, 3, 1, "", "register_synthesizers_prims"]], "synthesizers.synth_workers.base_synthesizer": [[18, 2, 1, "", "BaseSynthesizer"]], "synthesizers.synth_workers.base_synthesizer.BaseSynthesizer": [[18, 3, 1, "", "__call__"], [18, 3, 1, "", "__init__"], [18, 4, 1, "", "__name__"], [18, 3, 1, "", "get_prims"], [18, 3, 1, "", "register_synthesizers_prims"], [18, 4, 1, "", "scenario_owner"]], "synthesizers.synth_workers.dummy_synthesizer": [[19, 2, 1, "", "DummySynthesizer"]], "synthesizers.synth_workers.dummy_synthesizer.DummySynthesizer": [[19, 3, 1, "", "__call__"], [19, 3, 1, "", "__init__"], [19, 4, 1, "", "__name__"], [19, 4, 1, "", "_box_primitive_path"], [19, 3, 1, "", "get_prims"], [19, 3, 1, "", "register_synthesizers_prims"]], "synthesizers.synth_workers.ground_synthesizer": [[20, 2, 1, "", "GroundSynthesizer"]], "synthesizers.synth_workers.ground_synthesizer.GroundSynthesizer": [[20, 3, 1, "", "__call__"], [20, 3, 1, "", "__init__"], [20, 4, 1, "", "_plane_node"], [20, 4, 1, "", "_stage"], [20, 4, 1, "", "_stage_plane_path"], [20, 3, 1, "", "get_prims"], [20, 4, 1, "", "materials_list"], [20, 3, 1, "", "register_synthesizers_prims"]], "synthesizers.synth_workers.items_scatter_synthesizer": [[21, 2, 1, "", "ItemsScatterSynthesizer"]], "synthesizers.synth_workers.items_scatter_synthesizer.ItemsScatterSynthesizer": [[21, 3, 1, "", "__call__"], [21, 3, 1, "", "__init__"], [21, 4, 1, "", "_placement_prims"], [21, 4, 1, "", "_scattered_prims"], [21, 4, 1, "", "assets_pool_size"], [21, 3, 1, "", "get_prims"], [21, 4, 1, "", "number_of_assets_displayed_at_once"], [21, 4, 1, "", "placement_synths"], [21, 3, 1, "", "register_synthesizers_prims"], [21, 4, 1, "", "semantics"]], "synthesizers.synth_workers.light_synthesizer": [[22, 2, 1, "", "LightSynthesizer"]], "synthesizers.synth_workers.light_synthesizer.LightSynthesizer": [[22, 3, 1, "", "__call__"], [22, 3, 1, "", "__init__"], [22, 4, 1, "", "_light_node"], [22, 4, 1, "", "_stage_light_path"], [22, 3, 1, "", "get_prims"], [22, 4, 1, "", "light_type"], [22, 4, 1, "", "position"], [22, 3, 1, "", "register_synthesizers_prims"], [22, 4, 1, "", "rotation"], [22, 4, 1, "", "scale"]], "synthesizers.synth_workers.scene_synthesizer": [[24, 2, 1, "", "SceneSynthesizer"]], "synthesizers.synth_workers.scene_synthesizer.SceneSynthesizer": [[24, 3, 1, "", "__call__"], [24, 3, 1, "", "__init__"], [24, 3, 1, "", "_load_from_nucleus"], [24, 4, 1, "", "_scene_node"], [24, 3, 1, "", "get_prims"], [24, 3, 1, "", "register_synthesizers_prims"], [24, 4, 1, "", "scene_path"]], "synthesizers.synth_workers.single_item_synthesizer": [[25, 2, 1, "", "SingleItemSynthesizer"]], "synthesizers.synth_workers.single_item_synthesizer.SingleItemSynthesizer": [[25, 3, 1, "", "__call__"], [25, 3, 1, "", "__init__"], [25, 4, 1, "", "_stage_prim_path"], [25, 3, 1, "", "get_prims"], [25, 4, 1, "", "position"], [25, 3, 1, "", "register_synthesizers_prims"], [25, 4, 1, "", "semantics"], [25, 4, 1, "", "usd_path"]], "tools": [[27, 0, 0, "-", "isaac_sim"], [28, 0, 0, "-", "replicator"], [29, 0, 0, "-", "scenario"], [30, 0, 0, "-", "scenarios_manager"], [31, 0, 0, "-", "single_camera"], [32, 0, 0, "-", "writer"]], "tools.isaac_sim": [[27, 2, 1, "", "IsaacSimApp"]], "tools.isaac_sim.IsaacSimApp": [[27, 3, 1, "", "__init__"], [27, 4, 1, "", "app"], [27, 3, 1, "", "close"], [27, 4, 1, "", "debug"], [27, 3, 1, "", "update"]], "tools.scenario": [[29, 2, 1, "", "Scenario"]], "tools.scenario.Scenario": [[29, 3, 1, "", "__init__"], [29, 4, 1, "", "frames_number"], [29, 3, 1, "", "get_cameras"], [29, 4, 1, "", "isaac_sim"], [29, 4, 1, "", "master_synthesizer"], [29, 3, 1, "", "prepare"], [29, 4, 1, "", "scenario_dict_config"], [29, 4, 1, "", "scenario_name"]], "tools.scenarios_manager": [[30, 2, 1, "", "ScenariosManager"]], "tools.scenarios_manager.ScenariosManager": [[30, 3, 1, "", "__init__"], [30, 3, 1, "", "__iter__"], [30, 3, 1, "", "__next__"], [30, 4, 1, "", "isaac_sim"], [30, 4, 1, "", "scenario_names"], [30, 4, 1, "", "scenarios"]], "tools.single_camera": [[31, 2, 1, "", "SingleCamera"]], "tools.single_camera.SingleCamera": [[31, 3, 1, "", "__init__"], [31, 4, 1, "", "cam_resolution"], [31, 4, 1, "", "clipping_range"], [31, 3, 1, "", "get_cameras"], [31, 4, 1, "", "position"], [31, 4, 1, "", "rotation"]], "tools.writer": [[32, 2, 1, "", "NullWriter"], [32, 2, 1, "", "OVWriter"]], "tools.writer.NullWriter": [[32, 3, 1, "", "attach"], [32, 3, 1, "", "create"], [32, 3, 1, "", "initialize"]], "tools.writer.OVWriter": [[32, 3, 1, "", "__init__"], [32, 3, 1, "", "attach"], [32, 3, 1, "", "create"], [32, 4, 1, "", "fs_store_path"], [32, 4, 1, "", "write_bounding_box_2d_loose"], [32, 4, 1, "", "write_bounding_box_2d_tight"], [32, 4, 1, "", "write_bounding_box_3d"], [32, 4, 1, "", "write_camera_params"], [32, 4, 1, "", "write_distance_to_camera"], [32, 4, 1, "", "write_distance_to_image_plane"], [32, 4, 1, "", "write_instance_segmentation"], [32, 4, 1, "", "write_motion_vectors"], [32, 4, 1, "", "write_normals"], [32, 4, 1, "", "write_occlusion"], [32, 4, 1, "", "write_rgb"], [32, 4, 1, "", "write_semantic_segmentation"], [32, 4, 1, "", "writer"], [32, 4, 1, "", "writer_name"]]}, "objtypes": {"0": "py:module", "1": "py:function", "2": "py:class", "3": "py:method", "4": "py:attribute", "5": "py:exception"}, "objnames": {"0": ["py", "module", "Python module"], "1": ["py", "function", "Python function"], "2": ["py", "class", "Python class"], "3": ["py", "method", "Python method"], "4": ["py", "attribute", "Python attribute"], "5": ["py", "exception", "Python exception"]}, "titleterms": {"metron_ai_ardagen_run": 0, "modul": [0, 3, 4, 5, 6, 7, 8, 10, 11, 12, 14, 17, 18, 19, 20, 21, 22, 23, 24, 25, 27, 28, 29, 30, 31, 32], "metron_shar": [1, 2, 3, 4, 5, 6, 7, 8], "packag": [1, 2, 9, 13, 15, 16, 26], "subpackag": [1, 13, 15, 33], "submodul": [1, 2, 9, 13, 15, 16, 26, 33], "io_util": 6, "param_valid": 7, "structur": 8, "config": [2, 3, 4, 5], "config_schema": 4, "instanti": 5, "miscellan": [9, 10, 11, 12], "custom_except": 10, "metron_ai_ardagen_util": 11, "offset_writ": 12, "metron_ai_ardagen": [], "synthes": [13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 35], "master_synthes": 14, "synth_work": [15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25], "base_synthes": 18, "dummy_synthes": 19, "ground_synthes": 20, "items_scatter_synthes": 21, "light_synthes": 22, "physics_synthes": 23, "scene_synthes": 24, "single_item_synthes": 25, "assets_synth": [16, 17], "ov_assets_synthes": 17, "tool": [26, 27, 28, 29, 30, 31, 32], "isaac_sim": 27, "replic": [28, 35, 42], "scenario": [29, 35, 43, 44], "scenarios_manag": 30, "single_camera": 31, "writer": [32, 35], "api": [33, 34, 40], "doc": [33, 34, 40, 41], "high": 35, "level": 35, "architectur": [34, 35, 36], "decis": 35, "descript": 35, "isaac": 35, "sim": 35, "app": 35, "camera": [35, 42], "rig": 35, "worker": 35, "master": 35, "manag": 35, "implement": [], "detail": [], "develop": [37, 42], "note": 37, "head": 38, "1": [38, 42], "math": 38, "definit": 38, "list": 38, "figur": 38, "tabl": 38, "metron": 41, "ai": 41, "ardagen": 41, "content": [36, 39, 41], "project": 42, "instal": [40, 42], "end": 42, "user": 42, "docker": 42, "compos": 42, "attent": 42, "bug": 42, "fix": 42, "ov": 42, "detach": 42, "fail": 42, "data": 42, "gener": [34, 40, 42], "dummi": 43, "document": [34, 40], "guid": [39, 40], "automat": 34, "build": 40, "refer": 40, "step": 40, "us": 34, "sphinx": 34, "extens": 34, "emoticon": 34, "sourc": [34, 40], "file": [34, 40], "manual": 40, "goo": []}, "envversion": {"sphinx.domains.c": 2, "sphinx.domains.changeset": 1, "sphinx.domains.citation": 1, "sphinx.domains.cpp": 6, "sphinx.domains.index": 1, "sphinx.domains.javascript": 2, "sphinx.domains.math": 2, "sphinx.domains.python": 3, "sphinx.domains.rst": 2, "sphinx.domains.std": 2, "sphinx": 56}})