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

import omni
import omni.graph.core as og
from omni.isaac.core.utils import stage
from omni.isaac.core_nodes.scripts.utils import set_target_prims
from pxr import Gf, UsdGeom
from omni.kit import commands
from pxr import Sdf, Usd
from omni.isaac.core.utils.prims import set_targets

import numpy as np
import usdrt.Sdf


def build_mecanum_controller_graph(robot_name, base_link_name="nanosaur_base"):
    # https://github.com/Road-Balance/RB_WheeledRobotExample/blob/main/RBWheeledRobotExample_python/WheeledRobotSummitO3WheelROS2/robotnik_summit.py
    # https://www.youtube.com/watch?v=XEri32NaLYk
    mecanum_graph_name = f"/{robot_name}/ROS_HolonomicControllerGraph"
    robot_stage_path=f"/{robot_name}/{base_link_name}"
    wheel_radius = 0.037 / 2
    wheelbase = 0.1
    wheel_separation = 0.101
    wheel_offset_z = 0 #0.007
    wheel_radius = np.array([ wheel_radius, wheel_radius, wheel_radius, wheel_radius ])
    wheel_positions = np.array([
        [wheelbase / 2, wheel_separation / 2, wheel_offset_z],
        [wheelbase / 2, -wheel_separation / 2, wheel_offset_z],
        [-wheelbase / 2, wheel_separation / 2, wheel_offset_z],
        [-wheelbase / 2, -wheel_separation / 2, wheel_offset_z],
    ])

    wheel_orientations = np.array([
        [0.7071068, 0, 0, 0.7071068],
        [0.7071068, 0, 0, 0.7071068],
        [0.7071068, 0, 0, 0.7071068],
        [0.7071068, 0, 0, 0.7071068],
    ])
    mecanum_angles = np.array([
        -45.0,
        -45.0,
        -45.0,
        -45.0,
    ])
    wheel_axis = np.array([1, 0, 0])
    up_axis = np.array([0, 0, 1])

    targetPrim = robot_stage_path
    # Creating a action graph with ROS component nodes
    try:
        og.Controller.edit(
            {
                "graph_path": mecanum_graph_name,
                "evaluator_name": "execution",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION
            },
            {
                og.Controller.Keys.CREATE_NODES: [
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
                og.Controller.Keys.CONNECT: [
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
                og.Controller.Keys.CREATE_ATTRIBUTES: [
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
                og.Controller.Keys.SET_VALUES: [
                    # Assigning a Domain ID of 1 to Context node
                    ("context.inputs:domain_id", 0),
                    # Assigning topic name to clock publisher
                    ("subscribeTwist.inputs:topicName", "cmd_vel"),
                    ("angvelGain.inputs:value", 1.0),
                    ("linXGain.inputs:value", 1.0),
                    ("linYGain.inputs:value", 1.0),

                    ("mecanumAng.inputs:arraySize", 4),
                    ("mecanumAng.inputs:arrayType", "double[]"),
                    ("mecanumAng.inputs:input0", mecanum_angles[0]),
                    ("mecanumAng.inputs:input1", mecanum_angles[1]),
                    ("mecanumAng.inputs:input2", mecanum_angles[2]),
                    ("mecanumAng.inputs:input3", mecanum_angles[3]),
                    ("holonomicCtrl.inputs:angularGain", 1.0),
                    ("holonomicCtrl.inputs:linearGain", 1.0),
                    ("holonomicCtrl.inputs:maxWheelSpeed", 1200.0),

                    ("upAxis.inputs:value", up_axis),
                    ("wheelAxis.inputs:value", wheel_axis),

                    ("wheelOrientation.inputs:arraySize", 4),
                    ("wheelOrientation.inputs:arrayType", "double[4][]"),
                    ("wheelOrientation.inputs:input0", wheel_orientations[0]),
                    ("wheelOrientation.inputs:input1", wheel_orientations[1]),
                    ("wheelOrientation.inputs:input2", wheel_orientations[2]),
                    ("wheelOrientation.inputs:input3", wheel_orientations[3]),

                    ("wheelPosition.inputs:arraySize", 4),
                    ("wheelPosition.inputs:arrayType", "double[3][]"),
                    ("wheelPosition.inputs:input0", wheel_positions[0]),
                    ("wheelPosition.inputs:input1", wheel_positions[1]),
                    ("wheelPosition.inputs:input2", wheel_positions[2]),
                    ("wheelPosition.inputs:input3", wheel_positions[3]),

                    ("wheelRadius.inputs:arraySize", 4),
                    ("wheelRadius.inputs:arrayType", "double[]"),
                    ("wheelRadius.inputs:input0", wheel_radius[0]),
                    ("wheelRadius.inputs:input1", wheel_radius[1]),
                    ("wheelRadius.inputs:input2", wheel_radius[2]),
                    ("wheelRadius.inputs:input3", wheel_radius[3]),

                    ("jointNames.inputs:arraySize", 4),
                    ("jointNames.inputs:arrayType", "token[]"),
                    ("jointNames.inputs:input0", "front_left_mecanum_joint"),
                    ("jointNames.inputs:input1", "front_right_mecanum_joint"),
                    ("jointNames.inputs:input2", "rear_left_mecanum_joint"),
                    ("jointNames.inputs:input3", "rear_right_mecanum_joint"),

                    ("articulation.inputs:targetPrim", [usdrt.Sdf.Path(targetPrim)]),
                    ("articulation.inputs:robotPath", targetPrim),
                    ("articulation.inputs:usePath", False),
                ]
            },
        )
    except Exception as e:
        print(e)


def publish_joint_state_graph(robot_name, links, base_link_name="base_link"):
    joint_state_graph_graph_name = f"/{robot_name}/ROS_JointStateGraph"
    # Creating a action graph with ROS component nodes
    try:
        og.Controller.edit(
            {
                "graph_path": joint_state_graph_graph_name,
                "evaluator_name": "execution",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ROS2Context", "omni.isaac.ros2_bridge.ROS2Context"),
                    ("ROS2PublishTransformTree", "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),
                    ("IsaacReadSimulationTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ROS2PublishTransformTree.inputs:execIn"),
                    ("ROS2Context.outputs:context", "ROS2PublishTransformTree.inputs:context"),
                    ("IsaacReadSimulationTime.outputs:simulationTime", "ROS2PublishTransformTree.inputs:timeStamp"),
                    ],
                og.Controller.Keys.SET_VALUES: [
                    # Assigning a Domain ID of 1 to Context node
                    ("ROS2Context.inputs:domain_id", 0),
                ]
            },
        )
    except Exception as e:
        print(e)

    # Set all joint targets
    set_target_prims(
        primPath=f"{joint_state_graph_graph_name}/ROS2PublishTransformTree",
        inputName="inputs:parentPrim",
        targetPrimPaths=[f"/{robot_name}/{base_link_name}"],
    )
    set_target_prims(
        primPath=f"{joint_state_graph_graph_name}/ROS2PublishTransformTree",
        inputName="inputs:targetPrims",
        targetPrimPaths=[f"/{robot_name}/{link_name}" for link_name in links],
    )


def build_clock_graph():
    try:
        (_, _, _, _) = og.Controller.edit(
            {
                "graph_path": f"/Clock",
                "evaluator_name": "execution",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION
            },
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("IsaacReadSimulationTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                    ("ROS2PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ROS2PublishClock.inputs:execIn"),
                    ("IsaacReadSimulationTime.outputs:simulationTime", "ROS2PublishClock.inputs:timeStamp"),
                ]
            },
        )
    except Exception as e:
        print(e)
# EOF
