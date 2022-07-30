# Project Installation

Solution is based on [NVIDIA Omniverse](https://developer.nvidia.com/nvidia-omniverse-platform) with Python interface.
There are two ways of installation. End user installation and development installation. *The guide follows Linux based
installation steps*, which is also the recommended option.

## End User Installation

### Docker Compose

> ##### :warning: Attention :exclamation: :raised_hands: :exclamation: <!--markdownlint-disable header-increment no-duplicate-header blanks-around-headers-->
> NVIDIA GPU is required.

To be able to run ArDaGen using Docker Compose, perform following steps:

1. Install Docker ([how to install](https://docs.docker.com/get-docker/)).
2. Install Docker Compose ([how to install](https://docs.docker.com/compose/install/)).
3. Pull [NVIDIA Omniverse Isaac Sim Docker image](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/isaac-sim).

TODO

## Development Installation

Follow the instruction steps.
**Linux (Ubuntu 20.04) installation was tested only. The installation might not work on Windows.**

> ##### :warning: Attention :exclamation: :raised_hands: :exclamation: <!--markdownlint-disable header-increment no-duplicate-header blanks-around-headers-->
> NVIDIA GPU is required.

1. Download and install NVIDIA Omniverse Launcher from [here](https://developer.nvidia.com/nvidia-omniverse-platform).

2. Run Omniverse Launcher and install Isaac Sim application.
[See the link.](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_basic.html)

3. Install [Microsoft Visual Studio Code](https://code.visualstudio.com/).
**Other IDEs are not supported out of the box, nor tested.**

4. Download the repository using
`git clone --recurse-submodules https://github.com/OndrejSzekely/metron_ai_ardagen.git`.

5. Move ArDaGen repository into Isaac Sim's root folder.

    ```shell
    # a sample command, your paths will be different
    mv  /ssd_crucial/projects/metron_ai_ardagen /ssd_crucial/omniverse/isaac_sim-2021.2.1/
    ```

6. Download [Shapenet's](https://shapenet.org/) ShapeNetCore v2 dataset.
You have to register on the page to be allowed to download the dataset. Place the
downloaded dataset into any arbitraty location, but a fast storage is recommended.

7. To start Visual Studio project, go into Isaac Sim's root folder and run `code .`. It will open a setup Visual
Studio project with everything predefined.

    In the *Explorer* tab whole Isaac Sim folder is present, but everything else, except *metron_ai_garden* folder,
    is irrelevant for the development.

    > ##### :clipboard: Remark :raised_hand:
    > Be aware that debuging is performed using the *Python: Current File* option in *Run & Debug*.

8. Append

    ```yaml
    "./metron_ai_ardagen",
    "./exts/omni.replicator.core-1.2.0+cp37",
    ```

    into `python.analysis.extraPaths` list in 
    `<ISAAC_SIM_ROOT_FOLDER_PATH>/.vscode/settings.json` file.

9. Install missing Python requirements. Go into Isaac Sim's root folder and run
`./python.sh -m pip install -r metron_ai_ardagen/requirements.txt`.
