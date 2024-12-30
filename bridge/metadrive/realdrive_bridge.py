import math
from multiprocessing import Queue

from metadrive.component.sensors.base_camera import _cuda_enable
from metadrive.component.map.pg_map import MapGenerateMethod

from openpilot.tools.sim.bridge.common import SimulatorBridge
from openpilot.tools.sim.bridge.metadrive.realdrive_common import RGBCameraRoad, RGBCameraWide
from openpilot.tools.sim.bridge.metadrive.realdrive_world import RealDriveWorld
from openpilot.tools.sim.lib.camerad import W, H



class RealDriveBridge(SimulatorBridge):
    def __init__(self, dual_camera, high_quality, test_duration=math.inf, test_run=False):
        super().__init__(dual_camera, high_quality)

        self.should_render = False
        self.test_run = test_run
        self.test_duration = test_duration if self.test_run else math.inf

    def spawn_world(self, queue: Queue):
        sensors = {
        "rgb_road": (RGBCameraRoad, W, H, )
        }

        if self.dual_camera:
            sensors["rgb_wide"] = (RGBCameraWide, W, H)

        config = dict(
        use_render=self.should_render,
        vehicle_config=dict(
            enable_reverse=False,
            # render_vehicle=False,
            image_source="rgb_road",
        ),
        sensors=sensors,
        image_on_cuda=_cuda_enable,
        image_observation=True,
        interface_panel=[],
        out_of_route_done=False,
        on_continuous_line_done=False,
        crash_vehicle_done=False,
        crash_object_done=False,
        arrive_dest_done=False,
        traffic_density=0.0, # traffic is incredibly expensive
        decision_repeat=1,
        physics_world_step_size=self.TICKS_PER_FRAME/100,
        preload_models=False,
        show_logo=False,
        # anisotropic_filtering=False
        )


        return RealDriveWorld(queue, config, self.test_duration, self.test_run, self.dual_camera)