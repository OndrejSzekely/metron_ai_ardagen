# Configuration

Configuration is done via [Hydra](https://hydra.cc/) configuration framework. The configuration is based on
configuration *.yaml* files, but also allows override option via command line arguments.
See [Hydra's overriding docs](https://hydra.cc/docs/1.3/tutorials/basic/your_first_app/config_file/).

## Configuration Logic
*ArDaGen* configuration setups are structured into *Scenarios*. Each *Scenario* has its own `.yaml` file in
`/conf/scenarios` folder. *Scenario* config file defines an interface to be populated. Along mandatory config
parameters, it is required to define *Cameras* setup for the scenario. One *Scenario* can contain multiple camera
setups. The scenario goes one by one *Camera* setup and for each generate all the data. Supported camera type `.yaml`
files are located in `/conf/cameras`. *Omniverse* stage is composed and for each frame randomized by
*Synthesizer Workers* which have to be set up in the *Scenario* config too. *Synthesizer Workers* `.yaml` config files
are located in `/conf/synthesizer_workers`.

## Config Files Structure

*ArDaGen's* configuration files are located in `conf` folder. The configuration folder structure is following:
```shell
/conf
    /cameras # Contains camera template config file. 
             # User can add here custom camera definition config files.
    /scenarios # Contains scenario template config file. 
               # User can add here custom scenario definition config files.
    /synthesizer_workers # Contains all defined Synthesizer Worker config files.
        /assets # Contain config files of Synthesizer Workers responsible
                # for stage assets.
        /materials # Contains config files defining material groups.
                   # Each config file defines a list of the materials of the group.
    isaac_sim.yaml # Contains Isaac Sim app specific parameters.
    metron_ai_ardagen_config.yaml # Root config file which puts all config files together.
                                  # This is the root config file for Hydra.
    writer.yaml # Config file defining what label outputs are produced.
```

## Path to ArDaGen OV Extension

Path to *Metron AI ArDaGen Omniverse Extension* **must be set either for existing scenarios or new ones**.
Set `fs_path` option for `settings.ardagen_extension` in `metron_ai_ardagen_config.yaml` to a valid path. The path is supposed to be absolute and should point to `metron_ai_ardagen_omni_ext` folder of git cloned extension.

## New Scenario

This section guides through a new *Scenario* configuration setup. *The whole configuration process follows [Hydra's](https://hydra.cc/) paradigm. All Hydra's features can be used.*

New *Scenario* `.yaml` file has to be put into `conf/scenarios` folder. New *Scenario's* has to include *Common Scenario*:
```yaml
defaults:
  - common_scenario@dummy_scenario # do not touch
```

:::{admonition} Sample Config
:class: tip, dropdown

```yaml
defaults:
  - common_scenario@dummy_scenario # do not touch
```

:::

Then *Scenario's* name has to be defined:

```yaml
<SCENARIO_NAME>:
```

:::{admonition} Sample Config
:class: tip, dropdown

```yaml
defaults:
  - common_scenario@dummy_scenario # do not touch
sample_scenario:
```

:::

By default a new *Scenario* will generate `100` frames with `20` offset frames between each generated frame. Offset frames improves visual quality, because it gives time to denoisers to stabilize ray-trayced effects/images.
Use higher `<FRAMES_OFFSET>` value for more complex scenes and higher output resolution.
To override the values use following parameters:

```yaml
  frames_number: <GENERATED_FRAMES_NUM>
  frames_readout_offset: <FRAMES_OFFSET>
```

:::{admonition} Sample Config
:class: tip, dropdown

```yaml
defaults:
  - common_scenario@dummy_scenario # do not touch
sample_scenario:
  frames_number: 5
  frames_readout_offset: 60
```

:::

## Cameras

Each *Scenario* must have *at least one* camera setup (rig) in `cameras` *Config Group*. `common_scenario` defines a
default camera `camera` which is of `comon_single_camera` config option.

:::{admonition} Note
:class: note

Default camera's name and config option can be overridden by `<CAMERA_TYPE>` type and `<CAMERA_NAME>` camera name
using *Hydra's* override *defaults* syntax at the very beginning of a *Scenario* file.

```yaml
defaults:
  - /cameras/<CAMERA_TYPE>@cameras.camera
  - cameras.camera@cameras.<CAMERA_NAME>
```

:::

:::{admonition} Sample Config
:class: tip, dropdown

```yaml
defaults:
  - common_scenario@dummy_scenario # do not touch
sample_scenario:
  frames_number: 5
  frames_readout_offset: 60
cameras:
  camera:
    position:
      - 0 #X
      - 0 #Y
      - 10 #Z
    rotation:
      - 0 #X
      - -90 #Y
      - 0 #Z
    clipping_range:
      - 0.01
      - 100.0
    resolution:
      - 1280
      - 720
```

:::

Following `cameras` *Config Group* options are defined:

### Single Camera

Defined in `cameras/common_single_camera.yaml` file. It provides following config options:

- `position` (**required**)
  - World coordinate of camera defined by 3 float values. Metric system depends on OV stage setup. Default metric unit is cm.
- `rotation` (**required**)
  - Camera rotation given by 3 float values - roll, pitch, yaw.
- `clipping_range` (**required**)
  - Clipping range from/up to which distance objects are captured in the camera. Metric system depends on OV stage setup. Default metric unit is cm. The first float value sets the minimal distance from camera at which objects are captured. The second float value sets the maximal distance up to which are objects captured.
- `resolution` (**required**)
  - Camera pixel resolution given by two int values - width and height.

## Synthetic Workers

The most important part of each *Scenario* is *Synthesizer Workers* which defines the whole *Scenario*.

*Synthesizer Workers* are specified under `synthesizer_workers` config group option.

```yaml
<SCENARIO_NAME>:
  synthesizer_workers:
```

There can be only config group value (`<SYNTHESIZER_WORKER_TYPE>`) under the same name. It is allowed to have more instances of the same group config option for a *Scenario* (`<SCENARIO_NAME>`), but it's name has to be overridden (`<INSTANCE_NAME>`) during default import.

```yaml
defaults:
  - /synthesizer_workers/<SYNTHESIZER_WORKER_TYPE>@<SCENARIO_NAME>.synthesizer_workers.<INSTANCE_NAME>
```

Then you have to define missing values or override existing ones for the imported *Synthesizer Workers*.

:::{admonition} Sample Config
:class: tip, dropdown

```yaml
defaults:
  - common_scenario@dummy_scenario # do not touch
  - /synthesizer_workers/single_item_synthesizer@dummy_scenario.synthesizer_workers.box1
  - /synthesizer_workers/single_item_synthesizer@dummy_scenario.synthesizer_workers.box2
sample_scenario:
  frames_number: 5
  frames_readout_offset: 60
  synthesizer_workers:
    box1:
      position:
        - 0.5 #X
        - 0.5 #Y
        - 0 #Z
      usd_path: omniverse://${settings.ov_nucleus_ip}/NVIDIA/Assets/Isaac/2022.1/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxB_01_1344.usd
      semantics: foreground
    box2:
      position:
        - -0.5 #X
        - -0.5 #Y
        - 0 #Z
      usd_path: omniverse://${settings.ov_nucleus_ip}/NVIDIA/Assets/Isaac/2022.1/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxD_05_947.usd
      semantics: foreground
cameras:
  camera:
    position:
      - 0 #X
      - 0 #Y
      - 10 #Z
    rotation:
      - 0 #X
      - -90 #Y
      - 0 #Z
    clipping_range:
      - 0.01
      - 100.0
    resolution:
      - 1280
      - 720
```

:::

### Dummy Synthesizer

Defined in `synthesizer_workers/dummy_synthesizer.yaml` file. It does not provide any config options.

### Ground Synthesizer

Defined in `synthesizer_workers/ground_synthesizer.yaml` file. It provides following config options:

- `materials`
  - List of `yaml` files from `materials` config group.
- `semantics` (**required**)
  - String name of a semantic class which is given to the object. Based on that, particular class is written into label records.
- `position` (**required**)
  - X, Y, Z metric float coordinate in the scene.
- `scale` (**required**)
  - X, Y, Z float scale of the object size.

### Light Synthesizer

Defined in `synthesizer_workers/light_synthesizer.yaml` file. It provides following config options:

- `position` (**required**)
  - X, Y, Z metric float coordinate in the scene.
- `rotation`
  - X, Y, Z float of rotation angle. Default is 0 along all axis.
- `scale`
  - X, Y, Z float scale. Default value is 1.
- `light_type`
  - Light type. See supported light types in *Omniverse* documentation. Default value is `distant`.

### Items Scatter Synthesizer

Defined in `synthesizer_workers/items_scatter_synthesizer.yaml` file. It provides following config options:

- `assets` (**required**)
  - List of config group options which provide assets. Selected from `assets` *Config Group* representing
   *Assets Synthesizers*.
- `number_of_assets_displayed_at_once` (**required**)
  - Number of assets which are displayed at once. It has to be lower equal than `assets_pool_size`.
- `assets_pool_size` (**required**)
  - Number of assets which are deployed to the stage. The number has to be be lower equal than the total number
    of provided assets by `assets`. It is good practice to limit a number of assets in the stage because of memory reasons.
- `placement_synths` (**required**)
  - Name of *Synthesizer Worker* instance in the `synthesizer_workers` section whose prims are used for placement.
- `semantics` (**required**)
  - String name of a semantic class which is given to the object. Based on that, particular class is written into label records.

### Scene Synthesizer

Defined in `synthesizer_workers/scene_synthesizer.yaml` file. It provides following config options:

- `scene_path` (**required**)
  - *Omniverse Nucleus* path to scene which is loaded during the initialization stage of *ArDaGen*.

### Single Item Synthesizer

- `position` (**required**)
  - X, Y, Z metric float coordinate in the scene.
- `usd_path` (**required**)
  - *Nucleus* asset path.
- `semantics` (**required**)
  - String name of semantic class of the asset.

## Assets Synthesizers

*Assets Synthesizers Workers* are special group of *Synthesizers* which are responsible for assets list provisioning to
caller *Synthesizer Workers*.

## OV Assets Synthesizer

Defined in `synthesizer_workers/assets/ov_card_box_assets.yaml` file.

It provides list box-like assets.

## Execute New Scenario

Once the *Scenario* is ready the only thing needed is to add it to the the root *ArDaGen* config `metron_ai_ardagen_config.yaml`
into `defaults.scenarios` package. It can be added more *Scenarios* into the list. *Scenarios* are then executed one by
one in a sequence.

  :::{admonition} Tip
  :class: tip

  Check if used *Omniverse* version matches the one set in `settings.issac_sim_version`.
  :::

```yaml
defaults:
  - scenarios:
    - new_scenario_1
    - new_scenario_2
  - isaac_sim@isaac_sim # do not touch
  - writer@writer

settings:
  ov_nucleus_ip: localhost
  issac_sim_version: 2022.2.0
  ardagen_extension:
    name: metron.ai.ardagen
    fs_path: /ssd_crucial/projects/metron_ai_ardagen_omni_ext
```

The root config is automatically loaded when *ArDaGen* is started.