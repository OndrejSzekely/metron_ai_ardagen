"""
Implements Omniverse Replicator handler class.
"""

from typing import List
from metron_shared import param_validators as param_val
from metron_shared.structures import Singleton
from tools.isaac_sim import IsaacSimApp
from tools.scenarios_manager import ScenariosManager
from tools.writer import OVWriter


@Singleton
class OVReplicator:  # pylint: disable=too-few-public-methods
    """
    Implements Omniverse Replicator handler class.

    Attributes:
        ov_replicator (OV omni.replicator.core module): Omniverse Replicator module.
        scenarios_manager (ScenariosManager): Scenarios manager containing scenarios
            for data generation to be performed.
        ov_writer (OVWriter): Writer which saved generated data produced by Isaac Sim Omniverse app.
    """

    def __init__(self, isaac_sim: IsaacSimApp, scenarios_manager: ScenariosManager, ov_writer: OVWriter) -> None:
        """
        Init method.

        Args:
            isaac_sim (IsaacSimApp): Isaac Sim app wrapper instance.
            scenarios_manager (ScenariosManager): Scenarios manager containing all scenarios which are gonna be
                executed and data generated from.
            ov_writer (OVWriter): `Omniverse Writer` to save generated artificial data on the disk.
        """
        param_val.check_type(isaac_sim, IsaacSimApp)
        param_val.check_type(scenarios_manager, ScenariosManager)
        param_val.check_type(ov_writer, OVWriter)

        self.scenarios_manager = scenarios_manager
        self.isaac_sim = isaac_sim
        self.ov_writer = ov_writer

    def __call__(self) -> None:
        """
        Runs the whole OV Replicator data generation.
        """
        # Isaac Sim app has to be created before modules can be imported, so called in here.
        import omni.replicator.core as rep  # pylint: disable=import-outside-toplevel

        for scenario in self.scenarios_manager:
            with rep.new_layer():
                # We do this such that all of the randomization and changes to the scene are self contained within
                # the new layer. Without adding the new layer to the USD file, the changes made via the script will
                # alter the scene permanently.

                scenario.prepare()

                for synthesizer in scenario.master_synthesizer:
                    rep.randomizer.register(synthesizer)

                for camera_setup, render_product_setup in scenario.get_cameras():
                    scenario_writer_name = self.ov_writer.create(scenario.scenario_name, scenario.frames_readout_offset)
                    self.ov_writer.attach(render_product_setup)

                    with rep.trigger.on_frame(
                        interval=scenario.frames_readout_offset, num_frames=scenario.frames_number
                    ):
                        for synthesizer_worker_name in scenario.master_synthesizer.synthesizers_worker_names:
                            getattr(rep.randomizer, synthesizer_worker_name)(camera_setup)

                    self._run_orchestration()
                    self._remove_camera(camera_setup, scenario_writer_name)

    def _remove_camera(self, camera_setup: List[str], writer_name: str) -> None:  # pylint: disable=no-self-use
        """
        Unloads a camera given by <camera> primitive path from the scene and removes writer given by <writer_name>.

        Args:
            camera (List[str]): Camera setup paths in the stage.
            writer_name (str): Writer which is removed.
        """
        # Isaac Sim app has to be created before modules can be imported, so called in here.
        import omni.replicator.core as rep  # pylint: disable=import-outside-toplevel
        import omni.usd  # pylint: disable=import-outside-toplevel

        param_val.check_type(camera_setup, List[str])
        param_val.check_type(writer_name, str)

        stage = omni.usd.get_context().get_stage()
        for camera_setup_item in camera_setup:
            stage.Unload(camera_setup_item)
        rep.WriterRegistry.detach(writer_name)

    def _run_orchestration(self) -> None:
        """
        Orchestrates Omniverse Replicator exection.
        """
        # Isaac Sim app has to be created before modules can be imported, so called in here.
        import omni.replicator.core as rep  # pylint: disable=import-outside-toplevel

        rep.orchestrator.run()

        # Wait until started
        while not rep.orchestrator.get_is_started():
            self.isaac_sim.update()

        # Wait until stopped
        while rep.orchestrator.get_is_started():
            self.isaac_sim.update()

        rep.BackendDispatch.wait_until_done()
