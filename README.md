<h1 align="center">Metron AI - Artificial Data Generator (ArDaGen)</h1> <!-- markdownlint-disable MD033-->

<p align="center">
  <kbd><img src="docs_src/imgs/title_img.jpg" alt="Title Illustative Image" width="700"></kbd>
</p>

Metron AI ArDaGen is a synthetic data generator build on
[NVIDIA Omniverse](https://developer.nvidia.com/nvidia-omniverse-platform) platform to generate physically accurate
images and annotations to train Metron AI.

<p align="center"> <!-- markdownlint-disable MD033-->
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

[![License: MIT](https://img.shields.io/github/license/ondrejszekely/metron_ai_ardagen)](https://opensource.org/licenses/MIT) [![Last commit](https://img.shields.io/github/last-commit/ondrejszekely/metron_ai_ardagen/main)](https://github.com/OndrejSzekely/metron_ai_deepforge/main)

## Documentation

Online documentation of the latest commit on the *main* branch can be found [on this link](https://ondrejszekely.github.io/metron_ai_ardagen). *If you don't want to build a new documentation and use the build in the repository, go to step 5 directly*.

### Building & running documentation locally

1. Download the repository using `git clone https://github.com/OndrejSzekely/metron_ai_ardagen.git`
2. It is recommended to use virtual environment managed by [*uv*](https://docs.astral.sh/uv), to encapsulate the dev tools
   (Python frameworks and other software components) from the system. Create Python **3.11** virtual environment using Python
   dependency management tool you are using (e.g. Conda, Pipenv, etc...).

    ##### :bulb: Reference Installation Steps :point_down:  <!-- markdownlint-disable MD001 MD023--> <!-- omit in toc -->

    Reference installation steps use [*uv*](https://docs.astral.sh/uv) management tool. Run following
    command to create a new virtual environment:

    ```shell
    uv venv
    ```

3. Install following frameworks in the environment.

    ```text
    sphinx (~5.0.2)
    myst-parser (=1.0.0)
    sphinxemoji (=0.2.0)
    sphinx-design (=0.3.0)
    sphinx-book-theme (=1.0.0)
    sphinx-copybutton (=0.5.1)
    ```

    ##### :bulb: Reference Installation Steps :point_down:  <!-- markdownlint-disable MD023 MD024--> <!-- omit in toc -->

    Run following command:

    ```shell
    uv sync --all-extras --frozen
    ```

4. Go into repository's root folder and in the activated environment build the documentation:

   ```shell
   sphinx-build -b html docs_src docs
   ```

   ##### :bulb: Reference Installation Steps :point_down:  <!-- markdownlint-disable MD024--> <!-- omit in toc -->

   Run following command:

   ```shell
   uv run sphinx-build -b html docs_src docs
   ```

5. In repository's root folder run a HTTP server with the documentation:

   ```shell
   python -m http.server --directory docs 4444
   ```

   Then open your browser `http://localhost:4444` and see the documentation.

   GOOD JOB! :raised_hands: :rocket: :dizzy:

   ##### :bulb: Reference Installation Steps :point_down: <!-- markdownlint-disable MD024--> <!-- omit in toc -->

   In repository's root folder and activated environment run a HTTP server with the documentation:

   ```shell
   python -m http.server --directory docs 4444
   ```
  
   Then open your browser `http://localhost:4444` and see the documentation.
