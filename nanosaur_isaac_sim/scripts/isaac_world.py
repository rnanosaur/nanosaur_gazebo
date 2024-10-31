# Copyright (C) 2024, Raffaello Bonghi <raffaello@rnext.it>
# All rights reserved
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import carb
import rclpy
from isaacsim import SimulationApp
from omni.isaac.core import World, SimulationContext
from omni.isaac.core.utils import stage, nucleus
from omni.kit import commands
from rclpy.node import Node
from std_srvs.srv import Empty
from std_msgs.msg import String

BACKGROUND_STAGE_PATH = "/background"

class IsaacWorldError(Exception):

    def __init__(self, value, message):
        self.value = value
        self.message = message
        super().__init__(self.message)


class IsaacRobotSpawner(Node):
    
    def __init__(self, topic_description: str = "nanosaur/robot_description"):
        super().__init__('isaac_robot_spawner')
        # setup the ROS2 subscriber to load a robot_description
        self._sub_robot_description = self.create_subscription(String, topic_description, self._callback_description, 1)
        self._sub_robot_description  # prevent unused variable warning

    def _callback_description(self, msg):
        self.get_logger().info("Load robot")
        robot_urdf = msg.data
        # Load URDF from file
        temp_path_local_urdf_file="/tmp/robot.urdf"
        # Save robot urdf
        text_file = open(temp_path_local_urdf_file, "w")
        n = text_file.write(robot_urdf)
        text_file.close()
        # Load robot
        status, import_config = commands.execute(
            "URDFCreateImportConfig")
        import_config.merge_fixed_joints = False
        import_config.convex_decomp = False
        import_config.import_inertia_tensor = False
        import_config.fix_base = False
        import_config.distance_scale = 1
        # Import URDF, stage_path contains the path the path to the usd prim in the stage.
        # extension_path = get_extension_path_from_name("omni.isaac.urdf")
        status, stage_path = commands.execute(
            "URDFParseAndImportFile",
            urdf_path=temp_path_local_urdf_file,
            import_config=import_config,
        )
        # TODO Check if is better to destroy this node
        # self.destroy_node()


class IsaacWorld(Node):

    def __init__(self, simulation_app: SimulationApp, stage_path: str = ""):
        super().__init__('isaac_world')
        self._simulation_app = simulation_app
        # List of all robot spawner
        self._robot_spawner = []
        # Setting up scene
        if stage_path:
            self._simulation_context = SimulationContext(stage_units_in_meters=1.0)
            # Locate assets root folder to load sample
            assets_root_path = nucleus.get_assets_root_path()
            if assets_root_path is None:
                carb.log_error("Could not find Isaac Sim assets folder")
                raise IsaacWorldError (-1, "Could not find Isaac Sim assets folder")
            # Loading the simple_room environment
            stage.add_reference_to_stage(assets_root_path + stage_path, BACKGROUND_STAGE_PATH)
        else:
            self._simulation_context = World(stage_units_in_meters=1.0)
            self._simulation_context.scene.add_default_ground_plane()
            # need to initialize physics getting any articulation..etc
            self._simulation_context.initialize_physics()
        # Wait two frames so that stage starts loading
        simulation_app.update()
        simulation_app.update()
        # create service to load robot spawners
        self._srv = self.create_service(Empty, '/isaac_sim_status', self._status_isaac_word)
        self._srv  # prevent unused variable warning
        # Node started
        self.get_logger().info("Isaac World initialized")

    def __enter__(self):
        # Start simulation
        self._simulation_context.play()
        self.get_logger().info("Isaac World playing")
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        # Cleanup
        self._simulation_context.stop()
        # Destroy the node explicitly
        self.destroy_node()
        # Switch off
        self.get_logger().info("Isaac World stop")

    def simulate(self):
        while self._simulation_app.is_running():
            self._simulation_context.step(render=True)
            # Spin all robot spawner
            for robot_spawner in self._robot_spawner:
                self.get_logger().info("Request status Isaac Sim")
                rclpy.spin_once(robot_spawner, timeout_sec=0.0)
            # Spin this node
            rclpy.spin_once(self, timeout_sec=0.0)
            # Fix time simulation
            if self._simulation_context.is_playing():
                if self._simulation_context.current_time_step_index == 0:
                    self._simulation_context.reset()

    def _status_isaac_word(self, _, response):
        self.get_logger().info("Request status Isaac Sim")
        # Add a robot spawner in list
        self._robot_spawner += [IsaacRobotSpawner()]
        return response


def ros_bridge_main(simulation_app: SimulationApp):
    rclpy.init()
    # Start Isaac World controller
    with IsaacWorld(simulation_app) as isaac_world:
        try:
            isaac_world.simulate()
        except (KeyboardInterrupt, SystemExit):
            pass
    rclpy.shutdown()
# EOF