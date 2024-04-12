#!/usr/bin/env python

# Imports to start Isaac Sim from this script
import carb
from omni.isaac.kit import SimulationApp

# Start Isaac Sim's simulation environment
simulation_app = SimulationApp({"headless": True})

# -----------------------------------
# The actual script should start here
# -----------------------------------

import omni.timeline
from omni.isaac.core.world import World

import omni.isaac.core.utils.prims as prim_utils

# Import the Pegasus API for simulating drones
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.sensors.rgb_camera import RGBCamera

from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.backends.mavlink_backend import MavlinkBackend, MavlinkBackendConfig
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import XFormPrim

# Auxiliary scipy and numpy modules
import numpy as np
from scipy.spatial.transform import Rotation
from pathlib import Path

import os
os.environ["ROS_DISTRO"] = "humble"
os.environ["ROS_DOMAIN_ID"] = "0"
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend

PX4_PATH = "~/PX4-Autopilot"

class PegasusApp:
    """
    A Template class that serves as an example on how to build a simple Isaac Sim standalone App.
    """

    def __init__(self):
        """
        Method that initializes the PegasusApp and is used to setup the simulation environment.
        """

        print("PegasusApp Simulation App is initializing.", flush=True)

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        print("PegasusApp Simulation App is loading inteface", flush=True)

        # Start the Pegasus Interface
        self.pg = PegasusInterface()

        print("PegasusApp Simulation App has loaded interface.", flush=True)
        self.pg.set_px4_path(PX4_PATH)

        # Acquire the World, .i.e, the singleton that controls that is a one stop shop for setting up physics, 
        # spawning asset primitives, etc.
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Launch one of the worlds provided by NVIDIA
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Warehouse with Shelves"])

        # Add 
        add_reference_to_stage(usd_path="/workspaces/ar_challenge_isaac/Gate.usd", prim_path="/World/Gate1")
        add_reference_to_stage(usd_path="/workspaces/ar_challenge_isaac/Gate.usd", prim_path="/World/Gate2")
        add_reference_to_stage(usd_path="/workspaces/ar_challenge_isaac/Gate.usd", prim_path="/World/Gate3")
        add_reference_to_stage(usd_path="/workspaces/ar_challenge_isaac/Gate.usd", prim_path="/World/Gate4")
        add_reference_to_stage(usd_path="/workspaces/ar_challenge_isaac/Gate.usd", prim_path="/World/Gate5")
        add_reference_to_stage(usd_path="/workspaces/ar_challenge_isaac/Gate.usd", prim_path="/World/Gate6")
        add_reference_to_stage(usd_path="/workspaces/ar_challenge_isaac/Gate.usd", prim_path="/World/Gate7")
        add_reference_to_stage(usd_path="/workspaces/ar_challenge_isaac/Gate.usd", prim_path="/World/Gate8")
        add_reference_to_stage(usd_path="/workspaces/ar_challenge_isaac/Gate.usd", prim_path="/World/Gate9")
        add_reference_to_stage(usd_path="/workspaces/ar_challenge_isaac/Gate.usd", prim_path="/World/Gate10")

        self.world.scene.add(XFormPrim(prim_path="/World/Gate1", name="Gate1", position=[0,-7,0.5], scale=[0.005, 0.005, 0.005], orientation=[0.5,0.5,0.5,0.5]))
        self.world.scene.add(XFormPrim(prim_path="/World/Gate2", name="Gate2", position=[2.5,-5,1], scale=[0.005, 0.005, 0.005], orientation=[0.27076,0.27076,0.65322,0.65322]))

        self.world.scene.add(XFormPrim(prim_path="/World/Gate3", name="Gate3", position=[5, 0,1.5], scale=[0.005, 0.005, 0.005], orientation=[0,0,-0.70711,-0.70711]))
        self.world.scene.add(XFormPrim(prim_path="/World/Gate4", name="Gate4", position=[5, 5,2], scale=[0.005, 0.005, 0.005], orientation=[0,0,-0.70711,-0.70711]))

        self.world.scene.add(XFormPrim(prim_path="/World/Gate5", name="Gate5", position=[2.5,11,2.5], scale=[0.005, 0.005, 0.005], orientation=[0.33197,0.32102,-0.63004,-0.62434]))
        self.world.scene.add(XFormPrim(prim_path="/World/Gate6", name="Gate6", position=[0,14,2], scale=[0.005, 0.005, 0.005], orientation=[0.5,0.5,-0.5,-0.5]))
        self.world.scene.add(XFormPrim(prim_path="/World/Gate7", name="Gate7", position=[-4,11,1.5], scale=[0.005, 0.005, 0.005], orientation=[0.65328,0.65328,-0.2706,-0.2706]))

        self.world.scene.add(XFormPrim(prim_path="/World/Gate8", name="Gate8", position=[-5,5,1], scale=[0.005, 0.005, 0.005], orientation=[0.70711,0.70711,0,0]))
        self.world.scene.add(XFormPrim(prim_path="/World/Gate9", name="Gate9", position=[-5,0,0.5], scale=[0.005, 0.005, 0.005], orientation=[0.70711,0.70711,0,0]))
        self.world.scene.add(XFormPrim(prim_path="/World/Gate10", name="Gate10", position=[-2.5,-5,0], scale=[0.005, 0.005, 0.005], orientation=[0.6532,0.6532,0.2706,0.2706]))      
        # Get the current directory used to read trajectories and save results

        # Turn on the light [ToDo]
        # stage = omni.usd.get_context().get_stage()
        # prim = stage.GetPrimAtPath("/World/layout/SM_LampCeilingA_43/RectLight")
        # visibility_attribute = prim.GetAttribute("visibility")
        # visibility_attribute.Set("inherited")

        self.curr_dir = str(Path(os.path.dirname(os.path.realpath(__file__))).resolve())

        # Create the drone with PX4-Autopilot control (ID0)
        config_multirotor = MultirotorConfig()
        mavlink_config = MavlinkBackendConfig({
            "vehicle_id": 0,
            "px4_autolaunch": False,
            "px4_dir": PegasusInterface().px4_path,
            "px4_vehicle_model": 'iris'
        })
        config_multirotor.backends = [MavlinkBackend(mavlink_config), ROS2Backend(0, 4)]

        # Create the camera sensor
        camera = RGBCamera(
            id=0, 
            config={
                "position": [0.0, 0.0, 0.0], 
                "rotation": [0.0, 0.0, 0.0, 1.0],
                "focal_length": 250.0,
                "resolution": [640, 480],
                "update_rate": 30.0
            }
        )

        # Add the custom camera to the vehicle sensors
        config_multirotor.sensors += [camera]

        # Create UAV object
        self.UAV = Multirotor(
            "/World/quadrotor",
            ROBOTS['Iris'],
            0,
            [5.0, -8.0, 0.0],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor,
        )

        # Set the camera to a nice position
        # First argument - camera position
        # 2nd argument - target that the camera should point 
        self.pg.set_viewport_camera([8, -8, 0.1], [5.0, -8.0, 0.5])

        self.world.reset()

    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """
        print("PegasusApp Simulation App is in run mode.", flush=True)

        # Start the simulation
        self.timeline.play()

        while simulation_app.is_running():
            self.world.step(render=True)
        
        # Cleanup and stop
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()

def main():

    pg_app = PegasusApp()
    print("PegasusApp Simulation App is loaded.", flush=True)
    # Run the application loop
    pg_app.run()

if __name__ == "__main__":
    main()