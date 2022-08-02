"""
Defines `Scenarios Manager` class which iterates over all scenarios.
"""


from __future__ import (
    annotations,
)
from typing import Iterator, Any  # allowing future references -> return class under which return value is returned
from omegaconf import DictConfig
from tools.scenario import Scenario
from tools.isaac_sim import IsaacSimApp
from metron_shared import param_validators as param_val


class ScenariosManager:
    """
    Defines `Scenarios Manager` class which iterates over all scenarios.

    Attributes:
        scenarios (DictConfig): Dict Config of all passed in scenarios.
        scenario_names (Iterable[str]): List of scenario names. Instantiated on <__iter__> call, till that contains
            empty iterator.
        isaac_sim (IsaacSimApp): Issac Sim App wrapper instance.
    """

    def __init__(self, scenarios: DictConfig, isaac_sim: IsaacSimApp) -> None:
        """
        Init.

        Args:
            scenarios (DictConfig): Dict Config of all passed in scenarios.
            isaac_sim (IsaacSimApp): Issac Sim App wrapper instance.
        """
        param_val.check_type(scenarios, DictConfig)
        param_val.check_type(isaac_sim, IsaacSimApp)

        self.scenarios = scenarios
        self.scenario_names: Iterator[Any] = iter([])
        self.isaac_sim = isaac_sim

    def __iter__(self) -> ScenariosManager:
        """
        Implements iterator magic method.

        Returns:
            ScenariosManager: Instance of itself.
        """
        self.scenario_names = iter(self.scenarios.keys())
        return self

    def __next__(self) -> Scenario:
        """
        Returns next scenario to be executed.

        Returns:
            Scenario: Scenario describing the what should be generated.
        """
        scenario_name = next(self.scenario_names)
        return Scenario(self.isaac_sim, scenario_name, self.scenarios[scenario_name])
