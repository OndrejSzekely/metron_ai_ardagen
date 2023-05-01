Flow Chart Architecture Diagram
===============================

In `High Level Architecture <high_level_arch.html>`_ components high level overview was performed.
This chapter depicts flow interactions inside ArDaGen.

.. mermaid::
    :align: center

    ---
    Legend
    ---
    flowchart LR
        subgraph Legend
            db_legend[("Database")]
            action_legend["Action performed by a function or class"]
            subgraph Component/Module/Package
                ...
            end
        end
        style Legend fill:#65BCD8,stroke:#545A5E,stroke-width:2px,color:#fff


.. mermaid::
    :align: center

    ---
    Flow Chart Architecture Diagram
    ---
    flowchart TD
        subgraph Metron Shared
            hydra["Configs parsing through Hydra"]
        end
        hydra_config[("Hydra scenarios configs")]
        hydra_config --> hydra

