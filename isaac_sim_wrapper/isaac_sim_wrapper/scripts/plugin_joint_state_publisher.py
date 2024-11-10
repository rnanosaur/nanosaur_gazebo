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


from rclpy.node import Node
from isaacsim import SimulationApp
from omni.isaac.core_nodes.scripts.utils import set_target_prims
from omni.graph.core import Controller, GraphPipelineStage
from omni.kit import commands
from pxr import Sdf
import xml.etree.ElementTree as ET


def find_continuous_joint_childs(parent_link, robot_urdf):
    # Find all joints with type "continuous"
    joints = []
    child_links = []
    for joint in robot_urdf.findall('joint'):
        if joint.get('type') == 'continuous':
            joint_name = joint.get('name')
            if parent_link != joint.find('parent').get('link'):
                 continue
            child_link = joint.find('child').get('link')
            child_links.append(child_link)
            joints.append(joint_name)
    return joints, child_links


class PluginJointStatePublisher:
    
    def __init__(self,
                 node : Node, 
                 simulation_app : SimulationApp,
                 domain_id: int,
                 robot_name: str,
                 robot_urdf: str,
                 base_link: str = "base_link",
                 fix_joint_physic: bool = True):
        self._node = node
        self._simulation_app = simulation_app
        self._domain_id = domain_id
        self._robot_name = robot_name
        self._base_link = base_link
        self._fix_joint_physic = fix_joint_physic
        # Find all not fixed joint
        self._joints, self._continuous_joints_links = find_continuous_joint_childs(base_link, robot_urdf)
        # Graph path
        self._graph_path = f"/{self._robot_name}/ROS_JointStateGraph"
        # Loading camera
        node.get_logger().info(f"JointState: {self._robot_name} - Graph: {self._graph_path}")
        node.get_logger().info(f"JointState: base_link: {base_link} - Joints: {self._continuous_joints_links}")
        node.get_logger().info(f"JointState: Fix joint physic: {self._fix_joint_physic}")

    @classmethod
    def from_urdf(cls,
                 node : Node, 
                 simulation_app : SimulationApp,
                 domain_id: int,
                 robot_name: str,
                 robot_urdf: str,
                 plugin_data: str):
        # Extract all values from urdf data
        class_data = {
            'base_link': plugin_data.findtext("base_link", "base_link"),
            'fix_joint_physic': plugin_data.findtext("fix_joint_physic", "true").lower() == "true"
        }
        # Pass the required parameters along with the extracted optional data to the class constructor
        return cls(node, simulation_app, domain_id, robot_name, robot_urdf, **class_data)

    def load_joint_state(self):
        # FIX Isaac Sim - Change stiffness and damping
        if self._fix_joint_physic:
            for joint in self._joints:
                commands.execute('ChangeProperty', prop_path=Sdf.Path(f"/{self._robot_name}/{self._base_link}/{joint}.drive:angular:physics:damping"), value=17453.0, prev=0.0)
                commands.execute('ChangeProperty', prop_path=Sdf.Path(f"/{self._robot_name}/{self._base_link}/{joint}.drive:angular:physics:stiffness"), value=0.0, prev=0.0)
        # Build action graph
        try:
            self._load_og()
        except Exception as e:
            self._node.get_logger().error(e)
        # Update simulation
        self._simulation_app.update()

    def _load_og(self):
        Controller.edit(
            {
                "graph_path": self._graph_path,
                "evaluator_name": "execution",
                "pipeline_stage": GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
            },
            {
                Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ROS2Context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("ROS2PublishTransformTree", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                    ("IsaacReadSimulationTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ],
                Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ROS2PublishTransformTree.inputs:execIn"),
                    ("ROS2Context.outputs:context", "ROS2PublishTransformTree.inputs:context"),
                    ("IsaacReadSimulationTime.outputs:simulationTime", "ROS2PublishTransformTree.inputs:timeStamp"),
                    ],
                Controller.Keys.SET_VALUES: [
                    ("ROS2Context.inputs:domain_id", self._domain_id),
                    ],
            }
        )
        
        # Set all joint targets
        set_target_prims(
            primPath=f"{self._graph_path}/ROS2PublishTransformTree",
            inputName="inputs:parentPrim",
            targetPrimPaths=[f"/{self._robot_name}/{self._base_link}"],
        )
        set_target_prims(
            primPath=f"{self._graph_path}/ROS2PublishTransformTree",
            inputName="inputs:targetPrims",
            targetPrimPaths=[f"/{self._robot_name}/{link_name}" for link_name in self._continuous_joints_links],
        )
# EOF
