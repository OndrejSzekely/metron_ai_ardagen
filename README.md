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
  - [Building \& running documentation locally](#building--running-documentation-locally)
## How to run

After you install the solution, go into *Isaac Sim* root folder in a terminal and run

```shell
./python.sh metron_ai_ardagen/metron_ai_ardagen_run.py
```

It loads ArDaGen's configuration via *Hydra* framework. See [configuration section](https://ondrejszekely.github.io/metron_ai_ardagen/configuration.html) for mode details.

## Status
[![License: GPL v3](https://img.shields.io/github/license/ondrejszekely/metron_ai_ardagen)](https://www.gnu.org/licenses/gpl-3.0) [![Release](https://img.shields.io/github/v/release/ondrejszekely/metron_ai_ardagen)](https://github.com/OndrejSzekely/metron_ai_ardagen/releases) [![Last commit](https://img.shields.io/github/last-commit/ondrejszekely/metron_ai_ardagen/develop)](https://github.com/OndrejSzekely/metron_ai_ardagen/compare/main...develop)

## Documentation
Online documentation of the latest commit on the *development* branch can be found [here](https://ondrejszekely.github.io/metron_ai_ardagen). *If you don't want to build a new documentation and use the build in the repository, go to step 5 directly*.

### Building & running documentation locally
1. Download the repository using `git clone https://github.com/OndrejSzekely/metron_ai_ardagen.git`
2. It is recommended to use virtual environment, to encapsulate the dev tools (Python frameworks and other software components). Create Python **3.7** virtual environment using Python dependency management tool you are using (e.g. Conda, Pipenv, etc...).
   
    ##### :bulb: Reference Installation Steps :point_down: <!-- omit in toc -->
    It is recommended to use Anaconda channel ([how to get Anaconda](https://www.anaconda.com/products/individual)),
    which also provides installation management of non-Python software components, and Python. Run following
    command to create a new virtual environment:
    ```shell
    conda create -n metron_dev_37 python=3.7
    ```
    Run following command to attach created virtual environment in which all further steps are executed:
    ```shell
    conda activate metron_dev_37
    ```

3. Install following frameworks in the environment.
    ```text
    sphinx (~5.0.2)
    myst-parser (=1.0.0)
    sphinxemoji (=0.2.0)
    sphinx-design (=0.3.0)
    sphinx-book-theme (=1.0.0)
    ```

    ##### :bulb: Reference Installation Steps :point_down: <!-- omit in toc -->
    In the activated environment run following commands:
    ```shell
    conda install -c anaconda sphinx=5.0.2
    conda install -c conda-forge myst-parser=1.0.0
    pip install sphinxemoji=0.2.0
    pip install sphinx-design=0.3.0
    pip install sphinx-book-theme=1.0.0
    ```

4. Go into repository's root folder and in the activated environment build the documentation:
   ```shell
   sphinx-build -b html docs_src docs
   ```

   ##### :bulb: Reference Installation Steps :point_down: <!-- omit in toc -->
   In the activated environment run following command:
   ```shell
   sphinx-build -b html docs_src docs
   ```

5. In repository's root folder run a HTTP server with the documentation:
   ```shell
   python -m http.server --directory docs 4444
   ```
   Then open your browser `http://localhost:4444` and see the documentation.

   GOOD JOB! :raised_hands: :rocket: :dizzy:

   ##### :bulb: Reference Installation Steps :point_down: <!-- omit in toc -->
   In repository's root folder and activated environment run a HTTP server with the documentation:
   ```shell
   python -m http.server --directory docs 4444
   ```
   Then open your browser `http://localhost:4444` and see the documentation.
