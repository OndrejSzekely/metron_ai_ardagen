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
using *Hydra's* override *defaults* synthax at the very beggining of a *Scenario* file.

```yaml
defaults:
  - /cameras/<CAMERA_TYPE>@cameras.camera
  - cameras.camera@cameras.<CAMERA_NAME>
```

:::

Following `cameras` *Config Group* options are defined:

### Single Camera

Defined in `cameras/common_single_camera.yaml` file. It provides following config options:

- `position` (**required**)
  - World coordinate of camera defined by 3 float values. Metric system depends on OV stage setup. Defualt metric unit is cm.
- `rotation` (**required**)
  - Camera rotation given by 3 float values - roll, pitch, yaw.
- `clipping_range` (**required**)
  - Clipping range from/up to which distance objects are captured in the camera. Metric system depends on OV stage setup. Defualt metric unit is cm. The frist float value sets the minimal distance from camera at which objects are captured. The second float value sets the maximal distance up to which are objects captured.
- `resolution` (**required**)
  - Camera pixel resolution given by two int values - width and height.
