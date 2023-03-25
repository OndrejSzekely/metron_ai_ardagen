# Documentation Generation Guide

## Documentation Building

1. It is recommended to use virtual environment, to encapsulate the dev tools (Python frameworks and other software components). Create Python **3.7** virtual environment using Python dependency management tool you are using (e.g. Conda, Pipenv, etc...).

    ::::{admonition} Reference Installation Steps
    :class: tip, dropdown

    :::{eval-rst}
    |:point_right:| It is recommended to use Anaconda channel `(how to get Anaconda <https://www.anaconda.com/products/individual>`_),
    which also provides installation management of non-Python software components, and Python. Run following
    command to create a new virtual environment:
    :::
    ```shell
    conda create -n metron_dev_37 python=3.7
    ```
    Run following command to attach created virtual environment in which all further steps are executed:
    ```shell
    conda activate metron_dev_37
    ```
    ::::

2. Install following frameworks in the environment.
    ```text
    sphinx (~5.0.2)
    myst-parser (=1.0.0)
    sphinxemoji (=0.2.0)
    sphinx-design (=0.3.0)
    sphinx-book-theme (=1.0.0)
    phinx-togglebutton (=0.3.2)
    ```

    ::::{admonition} Reference Installation Steps
    :class: tip, dropdown

    :::{eval-rst}
    |:point_right:| In the activated environment run following commands:
    :::
    ```shell
    conda install -c anaconda sphinx=5.0.2
    conda install -c conda-forge myst-parser=1.0.0
    pip install -r requirements_docs.txt
    ```
    ::::

3. Go into repository's root folder and in the activated environment build the documentation:
   ```shell
   sphinx-build -b html docs_src docs
   ```
    ::::{admonition} Reference Installation Steps
    :class: tip, dropdown

    :::{eval-rst}
    |:point_right:| In the activated environment run following command:
    :::
    ```shell
    sphinx-build -b html docs_src docs
    ```
    ::::

4. In repository's root folder run a HTTP server with the documentation:
   ```shell
   python -m http.server --directory docs 4444
   ```
   Then open your browser `http://localhost:4444` and see the documentation.
    
   :::{eval-rst}
   GOOD JOB! |:raised_hands:| |:rocket:| |:dizzy:|
   :::

    ::::{admonition} Reference Installation Steps
    :class: tip, dropdown

    :::{eval-rst}
    |:point_right:| In repository's root folder and activated environment run a HTTP server with the documentation:
    :::
    ```shell
    python -m http.server --directory docs 4444
    ```
    Then open your browser `http://localhost:4444` and see the documentation.
    ::::