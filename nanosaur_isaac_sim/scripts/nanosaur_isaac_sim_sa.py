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

import os
import sys
import carb
from omni.isaac.kit import SimulationApp


CONFIG = {"renderer": "RayTracedLighting", "headless": False}
NANOSAUR_WS="nanosaur_ws"
NANOSAUR_DESCRIPTION_NAME="nanosaur_description"

def get_workspace_path(nanosaur_ws_name):
    """
    Checks if a workspace folder exists in the user's home directory.
    :param folder_name: The name of the workspace folder to check.
    :return: The full path to the workspace if it exists, or None if it doesn't.
    """
    # Get the user's home directory
    user_home_dir = os.path.expanduser("~")
    
    # Create the full path for the workspace folder in the user's home directory
    workspace_path = os.path.join(user_home_dir, nanosaur_ws_name)
    
    # Check if the workspace folder exists
    if os.path.exists(workspace_path) and os.path.isdir(workspace_path):
        return workspace_path
    else:
        return None

def get_description_meshes(nanosaur_ws_name, package_description):
    workspace_path = get_workspace_path(nanosaur_ws_name)
    if workspace_path is None:
        print("There are no nanosaur workspaces")
        return None
    path_meshes = os.path.join(workspace_path, "install", "share", package_description, "meshes")
    if os.path.exists(path_meshes) and os.path.isdir(path_meshes):
        return path_meshes
    print("Nanosaur workspace is not yet build")
    return None

path_meshes = get_description_meshes(NANOSAUR_WS, NANOSAUR_DESCRIPTION_NAME)
if path_meshes is None:
    exit(1)

# Example ROS2 bridge sample demonstrating the manual loading of Multiple Robot Navigation scenario
simulation_app = SimulationApp(CONFIG)

from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.stage import is_stage_loading
from omni.isaac.core import World, SimulationContext
from omni.isaac.core.utils import nucleus
from omni.kit import commands
from omni import usd
import omni
from pxr import Sdf
from omni.kit.viewport.window import get_viewport_window_instances

from nanosaur_action_graphs import build_clock_graph, publish_joint_state_graph, build_mecanum_controller_graph
from camera_graphs import build_realsense_camera_graph

# enable ROS2 bridge extension
enable_extension("omni.isaac.ros2_bridge")

simulation_app.update()

# Note that this is not the system level rclpy, but one compiled for omniverse
from std_srvs.srv import Empty
from std_msgs.msg import String
from rclpy.node import Node
import rclpy
import xml.etree.ElementTree as ET

PATH_LOCAL_URDF_FOLDER="/tmp/robot.urdf"


class IsaacWorld():
    
    def __init__(self, stage_path=""):
        self.commands=[]
        # Setting up scene
        if stage_path:
            self.simulation_context = SimulationContext(stage_units_in_meters=1.0)
            # Locate assets root folder to load sample
            assets_root_path = nucleus.get_assets_root_path()
            if assets_root_path is None:
                carb.log_error("Could not find Isaac Sim assets folder")
                simulation_app.close()
                sys.exit()
            # Loading the simple_room environment
            usd_path = assets_root_path + stage_path
            usd.get_context().open_stage(usd_path, None)
        else:
            self.simulation_context = World(stage_units_in_meters=1.0)
            self.simulation_context.scene.add_default_ground_plane()
            # need to initialize physics getting any articulation..etc
            self.simulation_context.initialize_physics()
        # Build clock graph
        build_clock_graph()
        # Wait two frames so that stage starts loading
        simulation_app.update()
        simulation_app.update()
    
    def add_tick(self, function):
        self.commands += [function]
    
    def wait_step_reload(self):
        self.simulation_context.step(render=True)
        print("Loading stage...")
        while is_stage_loading():
            simulation_app.update()
        print("Loading Complete")

    def start_simulation(self):
        self.simulation_context.play()

    def run_simulation(self, node):
        while simulation_app.is_running():
            self.simulation_context.step(render=True)
            # Tick the Publish/Subscribe JointState, Publish TF and Publish Clock nodes each frame
            for command in self.commands:
                command()
            rclpy.spin_once(node, timeout_sec=0.0)
            if self.simulation_context.is_playing():
                if self.simulation_context.current_time_step_index == 0:
                    self.simulation_context.reset()
        # Cleanup
        self.simulation_context.stop()
        simulation_app.close()


class RobotLoader(Node):
    
    def __init__(self, isaac_world, namespace="nanosaur"):
        super().__init__("robot_loader", namespace=namespace)
        # Load isaac_world
        self.namespace = namespace
        self.isaac_world = isaac_world
        # setup the ROS2 subscriber here
        self.ros_sub = self.create_subscription(String, "robot_description", self.callback_description, 10)
        self.ros_sub  # prevent unused variable warning
        self.srv = self.create_service(Empty, '/isaac_sim_status', self.status_isaac_sim_loader)
        # Node started
        self.get_logger().info("Robot loader start")

    def status_isaac_sim_loader(self, _, response):
        self.get_logger().info("Request status Isaac Sim")
        return response

    def load_robot(self, robot_name):
        # Setting up import configuration:
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
            urdf_path=PATH_LOCAL_URDF_FOLDER,
            import_config=import_config,
        )
        # Wait a step
        self.isaac_world.wait_step_reload()
        # Build camera graph
        build_realsense_camera_graph(robot_name)
        # Build Joint State publisher
        links = ["front_left_mecanum_link", "front_right_mecanum_link", "rear_left_mecanum_link", "rear_right_mecanum_link"]
        publish_joint_state_graph(robot_name, links)
        # Build mecanum controller
        build_mecanum_controller_graph(robot_name)
        
        # Change stiffness and damping
        nanosaur_stage_path="/nanosaur/nanosaur_base"
        for joint in ["front_left_mecanum", "front_right_mecanum", "rear_left_mecanum", "rear_right_mecanum"]:
            omni.kit.commands.execute('ChangeProperty', prop_path=Sdf.Path(f"{nanosaur_stage_path}/{joint}_joint.drive:angular:physics:damping"), value=17453.0, prev=0.0)
            omni.kit.commands.execute('ChangeProperty', prop_path=Sdf.Path(f"{nanosaur_stage_path}/{joint}_joint.drive:angular:physics:stiffness"), value=0.0, prev=0.0)
        # Update simulation
        simulation_app.update()

        for window in get_viewport_window_instances(None):
            if window.title in ["Viewport1", "Viewport2", "Viewport3"]:
                window.visible = False

    def callback_description(self, msg):
        # callback function to set the cube position to a new one upon receiving a (empty) ROS2 message
        robot_name = "nanosaur"
        self.get_logger().info(f"Load robot")
        robot_urdf = msg.data
        # Get the current working directory
        path_meshes = get_description_meshes(NANOSAUR_WS, NANOSAUR_DESCRIPTION_NAME)
        # Replace the old_text with new_text
        robot_urdf = robot_urdf.replace(f'package://{NANOSAUR_DESCRIPTION_NAME}/meshes', path_meshes)
        # Save robot urdf
        text_file = open(PATH_LOCAL_URDF_FOLDER, "w")
        n = text_file.write(robot_urdf)
        text_file.close()
        
        #self.get_logger().info(robot_urdf)
        # Parse the XML content
        root = ET.fromstring(robot_urdf)

        # Find all <robot> elements
        robot_sections = root.findall("isaacsim")
        self.get_logger().info("--------------------------------------------------")
        #self.get_logger().info(robot_sections)
        #self.get_logger().info("--------------------------------------------------")
        # Print each <robot> section
        for index, robot in enumerate(robot_sections, start=1):
            self.get_logger().info(f"Isaac Sim Section {index}:")
            self.get_logger().info(ET.tostring(robot, encoding="unicode", method="xml"))
            self.get_logger().info("-" * 40)
        self.get_logger().info("--------------------------------------------------")
        # Load robot
        self.load_robot(robot_name)


if __name__ == "__main__":
    rclpy.init()
    # Isaac SIM world
    isaac_world = IsaacWorld()
    # Start simulation
    isaac_world.start_simulation()
    # Initialize robot loader
    robot_loader = RobotLoader(isaac_world)
    
    # Run simulation
    isaac_world.run_simulation(robot_loader)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot_loader.destroy_node()
    rclpy.shutdown()
# EOF