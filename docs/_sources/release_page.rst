Release Page
============

.. mermaid::
    :align: center

    gitGraph
        commit id: "init"
        branch develop
        checkout develop
        commit id: "0.1.0 dev"
        checkout main
        merge develop tag: "v0.1.0"

v0.1.0
******

* Defined ArDaGen architecture.
* Setting-up *pre-commit* hooks.
* Documentation setting-up.
* First set of basic synthesizers.
* Integration of *Meta's Hydra* framework.
