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
from omni.graph.core import Controller, GraphPipelineStage
import xml.etree.ElementTree as ET
import numpy as np
import usdrt.Sdf


class PluginMecanumDrive:
    
    def __init__(self,
                 node : Node, 
                 simulation_app : SimulationApp,
                 domain_id: int,
                 robot_name: str,
                 child_frame_id: str = "child_frame_id",
                 front_left_joint: str = "front_left_joint",
                 front_right_joint: str = "front_right_joint",
                 back_left_joint: str = "back_left_joint",
                 back_right_joint: str = "back_right_joint",
                 wheelbase: float = 1.0,
                 wheel_separation: float = 1.0,
                 wheel_radius: float = 1.0,
                 wheel_offset_z: float = 0.0,
                 mecanum_angles: float = 45.0,
                 linear_gain: float = 1.0,
                 angular_gain: float = 1.0,
                 max_wheel_speed: float = 1000,
                 topic_name: str = "cmd_vel"
                 ):
        self._node = node
        self._simulation_app = simulation_app
        self._domain_id = domain_id
        self._robot_name = robot_name
        # https://github.com/Road-Balance/RB_WheeledRobotExample/blob/main/RBWheeledRobotExample_python/WheeledRobotSummitO3WheelROS2/robotnik_summit.py
        # https://www.youtube.com/watch?v=XEri32NaLYk

        self._wheel_radius = np.array([ wheel_radius, wheel_radius, wheel_radius, wheel_radius ])
        self._wheel_positions = np.array([
            [wheelbase / 2, wheel_separation / 2, wheel_offset_z],
            [wheelbase / 2, -wheel_separation / 2, wheel_offset_z],
            [-wheelbase / 2, wheel_separation / 2, wheel_offset_z],
            [-wheelbase / 2, -wheel_separation / 2, wheel_offset_z],
        ])
        # Quaternion: [0.7071068, 0, 0, 0.7071068]
        # Roll: 0deg - Pith: 0deg - Yaw: 90deg
        # Represents a 90-degree rotation about the Z-axis.
        self._wheel_orientations = np.array([
            [0.7071068, 0, 0, 0.7071068],
            [0.7071068, 0, 0, 0.7071068],
            [0.7071068, 0, 0, 0.7071068],
            [0.7071068, 0, 0, 0.7071068],
        ])
        self._mecanum_angles = np.array([-mecanum_angles, -mecanum_angles, -mecanum_angles, -mecanum_angles])
        self._wheel_axis = np.array([1, 0, 0])
        self._up_axis = np.array([0, 0, 1])
        # joints
        self._child_frame_id = child_frame_id
        self._front_left_joint = front_left_joint
        self._front_right_joint = front_right_joint
        self._back_left_joint = back_left_joint
        self._back_right_joint = back_right_joint
        # Graph path
        self._graph_path = f"/{self._robot_name}/ROS_MecanumDriveGraph"
        self._targetPrim = f"/{self._robot_name}/{self._child_frame_id}"
        # Control values
        self._linear_gain = linear_gain
        self._angular_gain = angular_gain
        self._max_wheel_speed = max_wheel_speed
        # Topic speed name
        self._topic_name = topic_name
        # Loading camera
        node.get_logger().info(f"MecanumDrive: {self._robot_name} - Graph: {self._graph_path}")
        node.get_logger().info(f"MecanumDrive: wheelbase: {wheelbase} - wheel_separation: {wheel_separation} - wheel_radius: {wheel_radius}")
        node.get_logger().info(f"MecanumDrive: Target Prim {self._targetPrim}")

    @classmethod
    def from_urdf(cls,
                 node : Node, 
                 simulation_app : SimulationApp,
                 domain_id: int,
                 robot_name: str,
                 plugin_data: str):
        # Extract all values from urdf data
        class_data = {
            'child_frame_id': plugin_data.findtext("child_frame_id", "base_link"),
            'front_left_joint': plugin_data.findtext("front_left_joint", "front_left_joint"),
            'front_right_joint': plugin_data.findtext("front_right_joint", "front_right_joint"),
            'back_left_joint': plugin_data.findtext("back_left_joint", "back_left_joint"),
            'back_right_joint': plugin_data.findtext("back_right_joint", "back_right_joint"),
            'wheelbase': float(plugin_data.findtext("wheelbase", 1.0)),
            'wheel_separation': float(plugin_data.findtext("wheel_separation", 1.0)),
            'wheel_radius': float(plugin_data.findtext("wheel_radius", 1.0)),
            'wheel_offset_z': float(plugin_data.findtext("wheel_offset_z", 0.0)),
            'mecanum_angles': float(plugin_data.findtext("mecanum_angles", 45.0)),
            'linear_gain': float(plugin_data.findtext("linear_gain", 1.0)),
            'angular_gain': float(plugin_data.findtext("angular_gain", 1.0)),
            'max_wheel_speed': float(plugin_data.findtext("max_wheel_speed", 1000.0)),
            'topic_name': plugin_data.findtext("topic_name", "cmd_vel"),
        }
        # Pass the required parameters along with the extracted optional data to the class constructor
        return cls(node, simulation_app, domain_id, robot_name, **class_data)

    def load_mecanum_drive(self):
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
                "pipeline_stage": GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION
            },
            {
                Controller.Keys.CREATE_NODES: [
                    ("onPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("subscribeTwist", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
                    ("scaleToFromStage", "omni.isaac.core_nodes.OgnIsaacScaleToFromStageUnit"),
                    ("breakAngVel", "omni.graph.nodes.BreakVector3"),
                    ("breakLinVel", "omni.graph.nodes.BreakVector3"),
                    ("angvelGain", "omni.graph.nodes.ConstantDouble"),
                    ("angvelMult", "omni.graph.nodes.Multiply"),
                    ("linXGain", "omni.graph.nodes.ConstantDouble"),
                    ("linXMult", "omni.graph.nodes.Multiply"),
                    ("linYGain", "omni.graph.nodes.ConstantDouble"),
                    ("linYMult", "omni.graph.nodes.Multiply"),
                    ("velVec3", "omni.graph.nodes.MakeVector3"),
                    ("mecanumAng", "omni.graph.nodes.ConstructArray"),
                    ("holonomicCtrl", "omni.isaac.wheeled_robots.HolonomicController"),

                    ("upAxis", "omni.graph.nodes.ConstantDouble3"),
                    ("wheelAxis", "omni.graph.nodes.ConstantDouble3"),
                    ("wheelOrientation", "omni.graph.nodes.ConstructArray"),
                    ("wheelPosition", "omni.graph.nodes.ConstructArray"),   
                    ("wheelRadius", "omni.graph.nodes.ConstructArray"),   
                    ("jointNames", "omni.graph.nodes.ConstructArray"),   
                    ("articulation", "omni.isaac.core_nodes.IsaacArticulationController"),
                    ],
                Controller.Keys.CONNECT: [
                    ("onPlaybackTick.outputs:tick", "subscribeTwist.inputs:execIn"),
                    ("context.outputs:context", "subscribeTwist.inputs:context"),
                    ("subscribeTwist.outputs:angularVelocity", "breakAngVel.inputs:tuple"),
                    ("subscribeTwist.outputs:linearVelocity", "scaleToFromStage.inputs:value"),
                    ("scaleToFromStage.outputs:result", "breakLinVel.inputs:tuple"),

                    ("breakAngVel.outputs:z", "angvelMult.inputs:a"),
                    ("angvelGain.inputs:value", "angvelMult.inputs:b"),
                    ("breakLinVel.outputs:x", "linXMult.inputs:a"),
                    ("linXGain.inputs:value", "linXMult.inputs:b"),
                    ("breakLinVel.outputs:y", "linYMult.inputs:a"),
                    ("linYGain.inputs:value", "linYMult.inputs:b"),

                    ("angvelMult.outputs:product", "velVec3.inputs:z"),
                    ("linXMult.outputs:product", "velVec3.inputs:x"),
                    ("linYMult.outputs:product", "velVec3.inputs:y"),

                    ("onPlaybackTick.outputs:tick", "holonomicCtrl.inputs:execIn"),
                    ("velVec3.outputs:tuple", "holonomicCtrl.inputs:inputVelocity"),
                    ("mecanumAng.outputs:array", "holonomicCtrl.inputs:mecanumAngles"),
                    ("onPlaybackTick.outputs:tick", "holonomicCtrl.inputs:execIn"),

                    ("upAxis.inputs:value", "holonomicCtrl.inputs:upAxis"),
                    ("wheelAxis.inputs:value", "holonomicCtrl.inputs:wheelAxis"),
                    ("wheelOrientation.outputs:array", "holonomicCtrl.inputs:wheelOrientations"),
                    ("wheelPosition.outputs:array", "holonomicCtrl.inputs:wheelPositions"),
                    ("wheelRadius.outputs:array", "holonomicCtrl.inputs:wheelRadius"),

                    ("onPlaybackTick.outputs:tick", "articulation.inputs:execIn"),
                    ("holonomicCtrl.outputs:jointVelocityCommand", "articulation.inputs:velocityCommand"),
                    ("jointNames.outputs:array", "articulation.inputs:jointNames"),
                    ],
                Controller.Keys.CREATE_ATTRIBUTES: [
                    ("mecanumAng.inputs:input1", "double"),
                    ("mecanumAng.inputs:input2", "double"),
                    ("mecanumAng.inputs:input3", "double"),
                    ("wheelOrientation.inputs:input1", "double[4]"),
                    ("wheelOrientation.inputs:input2", "double[4]"),
                    ("wheelOrientation.inputs:input3", "double[4]"),
                    ("wheelPosition.inputs:input1", "double[3]"),
                    ("wheelPosition.inputs:input2", "double[3]"),
                    ("wheelPosition.inputs:input3", "double[3]"),
                    ("wheelRadius.inputs:input1", "double"),
                    ("wheelRadius.inputs:input2", "double"),
                    ("wheelRadius.inputs:input3", "double"),
                    ("jointNames.inputs:input1", "token"),
                    ("jointNames.inputs:input2", "token"),
                    ("jointNames.inputs:input3", "token"),
                ],
                Controller.Keys.SET_VALUES: [
                    ("context.inputs:domain_id", self._domain_id),
                    # Assigning topic name to clock publisher
                    ("subscribeTwist.inputs:topicName", self._topic_name),
                    ("angvelGain.inputs:value", 1.0),
                    ("linXGain.inputs:value", 1.0),
                    ("linYGain.inputs:value", 1.0),

                    ("mecanumAng.inputs:arraySize", 4),
                    ("mecanumAng.inputs:arrayType", "double[]"),
                    ("mecanumAng.inputs:input0", self._mecanum_angles[0]),
                    ("mecanumAng.inputs:input1", self._mecanum_angles[1]),
                    ("mecanumAng.inputs:input2", self._mecanum_angles[2]),
                    ("mecanumAng.inputs:input3", self._mecanum_angles[3]),
                    ("holonomicCtrl.inputs:angularGain", self._angular_gain),
                    ("holonomicCtrl.inputs:linearGain", self._linear_gain),
                    ("holonomicCtrl.inputs:maxWheelSpeed", self._max_wheel_speed),

                    ("upAxis.inputs:value", self._up_axis),
                    ("wheelAxis.inputs:value", self._wheel_axis),

                    ("wheelOrientation.inputs:arraySize", 4),
                    ("wheelOrientation.inputs:arrayType", "double[4][]"),
                    ("wheelOrientation.inputs:input0", self._wheel_orientations[0]),
                    ("wheelOrientation.inputs:input1", self._wheel_orientations[1]),
                    ("wheelOrientation.inputs:input2", self._wheel_orientations[2]),
                    ("wheelOrientation.inputs:input3", self._wheel_orientations[3]),

                    ("wheelPosition.inputs:arraySize", 4),
                    ("wheelPosition.inputs:arrayType", "double[3][]"),
                    ("wheelPosition.inputs:input0", self._wheel_positions[0]),
                    ("wheelPosition.inputs:input1", self._wheel_positions[1]),
                    ("wheelPosition.inputs:input2", self._wheel_positions[2]),
                    ("wheelPosition.inputs:input3", self._wheel_positions[3]),

                    ("wheelRadius.inputs:arraySize", 4),
                    ("wheelRadius.inputs:arrayType", "double[]"),
                    ("wheelRadius.inputs:input0", self._wheel_radius[0]),
                    ("wheelRadius.inputs:input1", self._wheel_radius[1]),
                    ("wheelRadius.inputs:input2", self._wheel_radius[2]),
                    ("wheelRadius.inputs:input3", self._wheel_radius[3]),

                    ("jointNames.inputs:arraySize", 4),
                    ("jointNames.inputs:arrayType", "token[]"),
                    ("jointNames.inputs:input0", self._front_left_joint),
                    ("jointNames.inputs:input1", self._front_right_joint),
                    ("jointNames.inputs:input2", self._back_left_joint),
                    ("jointNames.inputs:input3", self._back_right_joint),

                    ("articulation.inputs:targetPrim", [usdrt.Sdf.Path(self._targetPrim)]),
                    ("articulation.inputs:robotPath", self._targetPrim),
                    #("articulation.inputs:usePath", False),
                ]
            },
        )
# EOF
