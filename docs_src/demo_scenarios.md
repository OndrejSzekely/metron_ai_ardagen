# Demo Scenarios

The repository contains demo Scenarios which work as integration test and could be taken as examples for custom
Scenarios creation.

## Dummy Scenario

Opens *full_warehouse* scene. Camera captures the dense central part with a forklift and boxes on the ground. One box is randomly hidden. The scene produces five images in HD resolution. The scenario is based on [Dummy Synthesizer](synthesizers.md#dummy-synthesizer).

```{eval-rst}
.. subfigure:: AB
    :subcaptions: below
    :gap: 20px

    .. image:: imgs/dummy_scenario_w.jpg
        :alt: Visible box
    
    .. image:: imgs/dummy_scenario_w_o.jpg
        :alt: Hidden box

    The box is randomly disappearing.
```

## Demo Scenario

Creates a ground with random texture. On the ground there are placed two boxes at given positions. Additional ten box assets are randomly sampled and scattered on the ground randomly each frame. Additonally, only five scattered boxes are shown at once. Distant light is addded over the ground. Camera is placed to capture top view. Ten images are generated with all annotations at HD resolution.

```{eval-rst}
.. subfigure:: AB
    :subcaptions: below
    :gap: 20px

    .. image:: imgs/demo_scenario_1.jpg
    
    .. image:: imgs/demo_scenario_2.jpg

```