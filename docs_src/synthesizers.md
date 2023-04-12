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

    Demo Scenario output based on Dummy Synthesizer
```

## Scene Synthesizer

Loads a scene into the stage. Scene is assumed to be stored in *OV Nucleus* in a USD file. It doesn't perform any
randomizations.

```{eval-rst}
.. subfigure:: A

    .. image:: imgs/dummy_scenario_w.jpg

    Loaded *full_warehouse* scene into stage
```

## Ground Synthesizer

Creates a plain ground in the stage at given position with given dimensions and semantics. User made list of material
options are assigned to the ground instance. Materials are randomly chosen from the list by the Synthesizer and applied
on the ground during the simulation.

```{eval-rst}
.. subfigure:: AB
    :subcaptions: below
    :gap: 20px

    .. image:: imgs/ground_synthesizer_1.jpg
    
    .. image:: imgs/ground_synthesizer_2.jpg

    Ground synthesis in generated frames (you can notice added distant light reflection)
```

## Light Synthesizer
