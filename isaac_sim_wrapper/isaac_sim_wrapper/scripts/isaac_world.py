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

import re
import os
import carb
import rclpy
from isaacsim import SimulationApp
from omni.isaac.core import World, SimulationContext
from omni.isaac.core.utils import stage, nucleus
from omni.isaac.core.utils.stage import is_stage_loading
from omni.kit import commands
from omni.graph.core import Controller, GraphPipelineStage
from rclpy.node import Node
import xml.etree.ElementTree as ET
from isaac_sim_wrapper_msgs.srv import RobotSpawner
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from camera_graph import CameraGraph
from plugin_joint_state_publisher import PluginJointStatePublisher


PACKAGE_RE = re.compile(r'package://([^/]+)/')
BACKGROUND_STAGE_PATH = "/background"


def build_clock_graph():
    Controller.edit(
        {
            "graph_path": f"/Clock",
            "evaluator_name": "execution",
            "pipeline_stage": GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION
        },
        {
            Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("IsaacReadSimulationTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("ROS2PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
            ],
            Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ROS2PublishClock.inputs:execIn"),
                ("IsaacReadSimulationTime.outputs:simulationTime", "ROS2PublishClock.inputs:timeStamp"),
            ]
        },
    )


class IsaacWorldError(Exception):

    def __init__(self, value, message):
        self.value = value
        self.message = message
        super().__init__(self.message)


class IsaacRobotSpawner(Node):
    
    def __init__(self, simulation_app: SimulationApp, simulation_context: SimulationContext, topic_description: str):
        super().__init__('isaac_robot_spawner')
        self._simulation_app = simulation_app
        self._simulation_context = simulation_context
        self.get_logger().info(f"Topic robot description: {topic_description}")
        # setup the ROS2 subscriber to load a robot_description
        self._sub_robot_description = self.create_subscription(String, topic_description, self._callback_description, 1)
        self._sub_robot_description  # prevent unused variable warning
        # report if robot is loaded
        self._robot_loaded = False
        # Default robot name
        self._robot_name = ""

    def _callback_description(self, msg):
        self.get_logger().info("Loading robot")
        robot_urdf = msg.data
        # Replace all package with the right path
        for package_name in re.findall(PACKAGE_RE, robot_urdf):
            package_directory = get_package_share_directory(package_name)
            # self.get_logger().info(f"Change Package name: {package_name} to path: {package_directory}")
            robot_urdf = robot_urdf.replace(f'package://{package_name}', package_directory)
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
        # Update simulation
        self._simulation_context.step(render=True)
        while is_stage_loading():
            self._simulation_app.update()
        # Load all controllers
        root = ET.fromstring(robot_urdf)
        # Find robot name
        # Extract the content of the 'name' attribute
        self._robot_name = root.get("name")
        # Find all <isaacsim> elements where contain definitions of sensors and controllers
        isaacsim_sections = root.findall("isaacsim")
        # Print each <robot> section
        camera_counter = 1
        for index, isaacsim in enumerate(isaacsim_sections, start=1):
            # Extract the reference attribute from isaacsim
            isaacsim_reference = isaacsim.attrib.get("reference", None)
            self.get_logger().info(f"IsaacSim reference: {isaacsim_reference}")
            # Find the sensor tag
            sensor_tags = isaacsim.findall("sensor")
            # Check if the sensor tag exists and extract its type attribute and content
            for sensor_tag in sensor_tags:
                sensor_name = sensor_tag.attrib.get("name", None)
                sensor_type = sensor_tag.attrib.get("type", None)
                # Load sensors
                if sensor_type == 'camera':
                    camera = CameraGraph.from_urdf(self, self._simulation_app, self._robot_name, camera_counter, sensor_tag)
                    camera.load_camera()
                    # Increase camera counter
                    camera_counter += 1
                else:
                    self.get_logger().info(f"Sensor name: {sensor_name} - type: {sensor_type}")
                    self.get_logger().info("Sensor content:")
                    sensor_content = ET.tostring(sensor_tag, encoding="unicode")
                    self.get_logger().info(sensor_content)
            # Find all plugins
            plugins = isaacsim.findall("plugin")
            for plugin in plugins:
                plugin_name = plugin.attrib.get("name", None)
                # Load all plugins
                if plugin_name == "JointStatePublisher":
                    joint_state = PluginJointStatePublisher.from_urdf(self, self._simulation_app, self._robot_name, root, plugin)
                    joint_state.load_joint_state()
                else:
                    self.get_logger().info(f"Plugin: {plugin_name}")
        # Remove temporary urdf file
        if os.path.exists(temp_path_local_urdf_file):
            os.remove(temp_path_local_urdf_file)
        # Set robot_loaded to true if robot is spawned on Isaac Sim
        self._robot_loaded = True

    def robot_name(self):
        return self._robot_name

    def isLoaded(self):
        return self._robot_loaded


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
        # Build clock graph
        build_clock_graph()
        # Wait two frames so that stage starts loading
        simulation_app.update()
        simulation_app.update()
        # create service to load robot spawners
        self._srv = self.create_service(RobotSpawner, '/isaac_sim_status', self._status_isaac_word)
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
            # Spin this node
            rclpy.spin_once(self, timeout_sec=0.0)
            # Spin all robot spawner
            # self.get_logger().info(f"Number of robot spawner: {len(self._robot_spawner)}")
            for idx in range(len(self._robot_spawner)):
                if self._robot_spawner[idx].isLoaded():
                    self._robot_spawner[idx].destroy_node()
                    robot_name = self._robot_spawner[idx].robot_name()
                    self.get_logger().info(f"Robot {robot_name} spawner done!")
                    del self._robot_spawner[idx]
                    continue
                rclpy.spin_once(self._robot_spawner[idx], timeout_sec=0.0)
            # Fix time simulation
            if self._simulation_context.is_playing():
                if self._simulation_context.current_time_step_index == 0:
                    self._simulation_context.reset()

    def _status_isaac_word(self, request, response):
        self.get_logger().info("Request status Isaac Sim")
        robot_description = request.robot_description
        # Add a robot spawner in list
        self._robot_spawner += [IsaacRobotSpawner(self._simulation_app, self._simulation_context, robot_description)]
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