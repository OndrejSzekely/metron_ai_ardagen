# Synthesizers

Synthesizers, more specifically Synthesizer Workers, are responsible for a stage composition and randomization.
You can find a list of existing Synthesizers in here.

## Dummy Synthesizer

It serves as a toy Synthesizer for the integration test of the chain with Demo Scenario. Beyond Demo Scenario there is
no other practical use in other scenarios. It controls a visibility of *SM_CardBoxA_3* box in the stage. With every
frame it randomly shows/hides the box.

```{eval-rst}
.. subfigure:: AB
    :subcaptions: below
    :gap: 20px

    .. image:: imgs/dummy_scenario_w.jpg
        :alt: Visible box
    
    .. image:: imgs/dummy_scenario_w_o.jpg
        :alt: Hidden box

    Demo Scenario output based on Dummy Synthesizer.
```

## Scene Synthesizer

Loads a scene into the stage. Scene is assumed to be stored in *OV Nucleus* in a USD file. It doesn't perform any
randomizations.

```{eval-rst}
.. subfigure:: A

    .. image:: imgs/dummy_scenario_w.jpg

    Loaded *full_warehouse* scene into stage.
```

## Ground Synthesizer

Creates a plain ground in the stage at given position with given dimensions and semantics. User made list of material
options are assigned to the ground instance. Materials are randomly chosen from the list by the Synthesizer and applied on the ground during the simulation.

```{eval-rst}
.. subfigure:: AB
    :subcaptions: below
    :gap: 20px

    .. image:: imgs/synthesizers/ground_synthesizer_1.jpg
    
    .. image:: imgs/synthesizers/ground_synthesizer_2.jpg

    Different textures for the generated ground.
```

## Light Synthesizer

Creates a light at given position with given rotation and scale. The supported light types are *cylinder*, *disk*, *distant*, *dome*, *rectangle* and *sphere*.

```{eval-rst}
.. subfigure:: AB
    :subcaptions: below
    :gap: 20px

    .. image:: imgs/synthesizers/light_synthesizer_1.jpg
    
    .. image:: imgs/synthesizers/light_synthesizer_2.jpg

    Different light types - *cylinder* vs *disk*.
```

## Single Item Synthesizer

Adds a prim, given by an USD file, into the stage at given position and semantics.

```{eval-rst}
.. subfigure:: AB
    :subcaptions: below
    :gap: 20px

    .. image:: imgs/synthesizers/single_item_synthesizer_1.jpg
    
    .. image:: imgs/synthesizers/single_item_synthesizer_2.jpg

    Added box placed on different locations.
```

## Item Scatter Synthesizer

It selects randomly given number asset paths from given list of USD asset paths. These assets are scatered in the stage on given Synthesizer's prims randomly each frame and only given number of assets is randomly made visible. Given semantic class is assigned to all assets.

```{eval-rst}
.. subfigure:: AB
    :subcaptions: below
    :gap: 20px

    .. image:: imgs/synthesizers/item_scatter_synthesizer_1.jpg
    
    .. image:: imgs/synthesizers/item_scatter_synthesizer_2.jpg

    Randomly placed boxes on the ground each frame and random portion of placed boxes are made visible.
``` 
