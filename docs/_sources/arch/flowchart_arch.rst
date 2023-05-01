Flowchart Architecture Diagram
===============================

In `High Level Architecture <high_level_arch.html>`_ components high level overview was performed.
This chapter depicts flow interactions inside ArDaGen.

.. mermaid::
    :align: center

    ---
    Flow Chart Architecture Diagram
    ---
    %%{init: { "theme": "base" } }%%
    flowchart TD
        subgraph Metron Shared
            hydra["Configs parsing through Hydra"]
        end
        subgraph NVIDIA Omniverse
            nucleus[("Nucleus")]
            isaac_sim_start[["Start Isaac Sim"]]
            load_ardagen_ext["Load ArDaGen Extension"]
            start_replicator[["Start OV Replicator"]]
            all_frames_generated{"All dataset <br> frames generated?"}
            render_new_frame["Render new frame"]
        end
        start_generation(["Run ArDaGen"])
        end_generation(["Stop ArDaGen"])
        hydra_config[("Hydra scenarios configs")]
        filesystem[("Filesystem")]
        hydra_omegaconf[/"Config as OmegaConf"/]
        has_scenario{"Has scenario?"}
        prepare_scenario["Prepare Scenario"]
        has_synth_worker{"Has Synthesizer Worker?"}
        prepare_synth_worker["Prepare Synthesizer Worker"]
        gather_synth_worker["Gather the worker into Master Synthesizer"]
        has_camera_setup{"Has Camera Rig?"}
        prepare_camera_rig["Prepare Camera Rig"]
        attach_writer["Attach synthetic data Writer"]
        app_synth_workers["Apply all Synthesizer Workers <br> from the Master Synthesizer"]
        write_synthetic_data["Save generated synthetic data <br> on filesystem"]
        hydra_config --> hydra
        start_generation --> hydra
        hydra --> hydra_omegaconf
        hydra_omegaconf -.-> isaac_sim_start
        hydra --> isaac_sim_start
        isaac_sim_start --> load_ardagen_ext
        hydra_omegaconf -.-> load_ardagen_ext
        load_ardagen_ext --> has_scenario
        has_scenario -- No --> end_generation
        has_scenario -- Yes, get Scenario --> prepare_scenario
        hydra_omegaconf -.-> prepare_scenario
        prepare_scenario --> has_synth_worker
        has_synth_worker -- Yes, get Synthesizer Worker --> prepare_synth_worker
        hydra_omegaconf -.-> prepare_synth_worker
        nucleus -.-> prepare_synth_worker
        prepare_synth_worker --> gather_synth_worker
        gather_synth_worker --> has_synth_worker
        has_synth_worker -- No --> has_camera_setup
        has_camera_setup -- Yes, get Camera Rig --> prepare_camera_rig
        has_camera_setup -- No --> has_scenario
        hydra_omegaconf -.-> prepare_camera_rig
        prepare_camera_rig --> attach_writer
        hydra_omegaconf -.-> attach_writer
        attach_writer --> start_replicator
        start_replicator --> all_frames_generated
        all_frames_generated -- No --> app_synth_workers
        app_synth_workers --> render_new_frame
        render_new_frame --> write_synthetic_data
        write_synthetic_data --> filesystem
        write_synthetic_data --> all_frames_generated
        all_frames_generated -- Yes --> has_camera_setup

.. mermaid::
    :align: center

    ---
    Legend_Part1
    ---
    %%{init: { "theme": "base" } }%%
    flowchart LR
        subgraph Legend
            db_legend[("Database")]
            action_legend["Action performed by a function or method"]
            subrutine_legend[["Process/subprogram"]]
            start_end_legent(["Program start/end"])
            subgraph Component/Module/Package
                ...
            end
        end
        style Legend fill:#65BCD8,stroke:#545A5E,stroke-width:2px,color:#fff

.. mermaid::
    :align: center

    ---
    Legend_Part2
    ---
    %%{init: { "theme": "base" } }%%
    flowchart LR
        subgraph Legend
            start_legend(["Flow starting action"])
            io_legend[/"Input / Output(JSON, data structure, etc...)"/]
            A-- interaction flow -->B
            C-. configs flow .->D
            legend_decision{"Decision"}
        end
        style Legend fill:#65BCD8,stroke:#545A5E,stroke-width:2px,color:#fff