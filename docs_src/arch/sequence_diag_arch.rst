Sequence Diagram Architecture
=============================

.. mermaid::
    :align: center

    %%{init: {"sequence": {"useMaxWidth": false}}}%%
    sequenceDiagram
        actor User
        participant Hydra
        participant ArDaGen Runtime
        participant OV Replicator
        participant Scenarios Manager
        participant Scenario
        participant Master Synthesizer
        participant Synthesizer Worker
        participant Single Camera
        participant OV Isaac Sim
        participant OV Writer
        participant File System
        User->>ArDaGen Runtime: Start ArDaGen
        activate ArDaGen Runtime
        ArDaGen Runtime->>Hydra: Load ArDaGen Hydra configuration
        activate Hydra
        Hydra-->>ArDaGen Runtime: Get ArDaGen Hydra config
        ArDaGen Runtime->>OV Isaac Sim: Run OV Isaac Sim
        activate OV Isaac Sim
        OV Isaac Sim-->>ArDaGen Runtime: Isaac Sim instance reference
        ArDaGen Runtime->>OV Isaac Sim: Load ArDaGen OV Extension
        ArDaGen Runtime->>Scenarios Manager: Initialize Scenarios Manager
        activate Scenarios Manager
        ArDaGen Runtime->>OV Replicator: Start synthetic data generation
        activate OV Replicator
        loop For each Scenario
            OV Replicator->>OV Replicator: Create new layer
            OV Replicator->>Scenarios Manager: Get scenario
            Scenarios Manager->>Scenario: Initialize Scenario
            activate Scenario
            Scenarios Manager-->>OV Replicator: Scenario
            OV Replicator->>Master Synthesizer: Prepare Scenario
            activate Master Synthesizer
            loop For each Synthesizer Worker
                Master Synthesizer->>Synthesizer Worker: Initialize Synthesizer Worker
                activate Synthesizer Worker
                Synthesizer Worker-->>Master Synthesizer: Synthesizer Worker
            end
            loop For each instantiated Synthesizer Worker
                Master Synthesizer->>Synthesizer Worker: Register Synthesizer Worker's Prims
            end
            Master Synthesizer-->>OV Replicator: Giving back the control
            loop For each Synthesizer Worker
                OV Replicator->>Master Synthesizer: Get Synthesizer Worker
                Master Synthesizer-->>OV Replicator: Synthesizer Worker
                OV Replicator->>OV Replicator: Register the Synthesizer Worker into OV Randomizer
            end
            loop For each camera rig
                OV Replicator->>Scenario: Get camera rig
                Scenario-->>OV Replicator: Camera rig
                loop For each single camera in the camera rig
                    OV Replicator->>Single Camera: Initialize the camera in OV Replicator
                    activate Single Camera
                    Single Camera-->>OV Replicator: Single Camera reference
                end
                OV Replicator->>OV Writer: Instantiate OV Replicator Writer
                activate OV Writer
                OV Replicator->>OV Writer: Attach camera rig
                OV Replicator-->>+OV Replicator: Start simulation
                loop For each frame in the frames range
                OV Replicator->>OV Replicator: Update the simulation
                opt frame_index mod frame_capture_interval == 0
                OV Replicator->>OV Writer: Capture camera images and the labels
                OV Writer->>File System: Save images and labels on the file system
                end
                opt frame_index mod (frame_capture_interval + 1) == 0
                loop For each Synthesizer Worker
                    OV Replicator->>Synthesizer Worker: Randomize
                end
                end
                end
                OV Replicator->>-OV Replicator: Stop the simulation
                OV Replicator->>OV Replicator: Remove camera rig
                deactivate OV Writer
                deactivate Single Camera
            end
            deactivate Synthesizer Worker
            deactivate Scenario
            deactivate Master Synthesizer
            OV Replicator->>OV Replicator: Remove the layer
        end
        ArDaGen Runtime->>OV Isaac Sim: Stop OV Isaac Sim
        deactivate OV Replicator
        deactivate Scenarios Manager
        deactivate OV Isaac Sim
        deactivate Hydra
        deactivate ArDaGen Runtime
