<h1 align="center">Metron AI - Artificial Data Generator (ArDaGen)</h1>

<p align="center">
  <kbd><img src="docs_src/imgs/title_img.jpg" alt="Title Illustative Image" width="700"></kbd>
</p>

Metron AI ArDaGen is a synthetic data generator build on
[NVIDIA Omniverse](https://developer.nvidia.com/nvidia-omniverse-platform) platform to generate physically accurate
images and annotations to train Metron AI.

<p align="center">
    <a href="https://ondrejszekely.github.io/metron_ai_ardagen"><strong>Explore Metron AI ArDaGen docs »</strong></a>
</p>

## Key Features <!-- omit in toc -->

:boom: photorealistic physical based rendering via NVIDIA Omniverse
:boom: vast image content variations
:boom: 100% accurate annotations
:boom: uses SOTA configuration framework - Meta's Hydra
:boom: modular architecture with custom design scenarios

## Table of Contents <!-- omit in toc -->
- [How to run](#how-to-run)
- [Status](#status)
- [Documentation](#documentation)
## How to run

After you install the solution, go into *Isaac Sim* root folder in a terminal and run

```shell
./python.sh metron_ai_ardagen/metron_ai_ardagen_run.py
```

It loads ArDaGen's configuration via *Hydra* framework. See [configuration section](https://ondrejszekely.github.io/metron_ai_ardagen/configuration.html) for mode details.

## Status
[![License: GPL v3](https://img.shields.io/github/license/ondrejszekely/metron_ai_ardagen)](https://www.gnu.org/licenses/gpl-3.0) [![License: GPL v3](https://img.shields.io/github/v/release/ondrejszekely/metron_ai_ardagen)]()

## Documentation