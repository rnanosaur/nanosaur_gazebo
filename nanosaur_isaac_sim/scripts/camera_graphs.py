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
from pxr import Gf, UsdGeom
from omni.isaac.core.utils.prims import set_targets
from omni.kit.viewport.window import get_viewport_window_instances


def create_camera(robot_name, number_camera, camera_frame, camera_stage_path, camera_name, stereo_offset=[0.0, 0.0]):
    
    ros_camera_graph_path = f"/{robot_name}/ROS_CameraGraph_{camera_name}"
    viewport_name = f"Viewport{number_camera}"
    
    # Creating an on-demand push graph with cameraHelper nodes to generate ROS image publishers
    keys = og.Controller.Keys
    og.Controller.edit(
        {
            "graph_path": ros_camera_graph_path,
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            keys.CREATE_NODES: [
                (f"{camera_name}_OnTick", "omni.graph.action.OnTick"),
                (f"{camera_name}_createViewport", "omni.isaac.core_nodes.IsaacCreateViewport"),
                (f"{camera_name}_getRenderProduct", "omni.isaac.core_nodes.IsaacGetViewportRenderProduct"),
                (f"{camera_name}_setViewportResolution", "omni.isaac.core_nodes.IsaacSetViewportResolution"),
                (f"{camera_name}_setCamera", "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct"),
                (f"{camera_name}_cameraHelper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                (f"{camera_name}_cameraHelperInfo", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                #(f"{camera_name}_cameraHelperInfo", "omni.isaac.ros2_bridge.ROS2CameraInfoHelper"),
            ],
            keys.CONNECT: [
                (f"{camera_name}_OnTick.outputs:tick", f"{camera_name}_createViewport.inputs:execIn"),
                (f"{camera_name}_createViewport.outputs:execOut", f"{camera_name}_getRenderProduct.inputs:execIn"),
                (f"{camera_name}_createViewport.outputs:execOut", f"{camera_name}_setViewportResolution.inputs:execIn"),
                (f"{camera_name}_createViewport.outputs:viewport", f"{camera_name}_getRenderProduct.inputs:viewport"),
                (f"{camera_name}_createViewport.outputs:viewport", f"{camera_name}_setViewportResolution.inputs:viewport"),
                (f"{camera_name}_setViewportResolution.outputs:execOut", f"{camera_name}_setCamera.inputs:execIn"),
                (f"{camera_name}_getRenderProduct.outputs:renderProductPath", f"{camera_name}_setCamera.inputs:renderProductPath"),
                (f"{camera_name}_setCamera.outputs:execOut", f"{camera_name}_cameraHelper.inputs:execIn"),
                (f"{camera_name}_setCamera.outputs:execOut", f"{camera_name}_cameraHelperInfo.inputs:execIn"),
                (f"{camera_name}_getRenderProduct.outputs:renderProductPath", f"{camera_name}_cameraHelper.inputs:renderProductPath"),
                (f"{camera_name}_getRenderProduct.outputs:renderProductPath", f"{camera_name}_cameraHelperInfo.inputs:renderProductPath"),
            ],
            keys.SET_VALUES: [
                (f"{camera_name}_createViewport.inputs:name", viewport_name),
                (f"{camera_name}_createViewport.inputs:viewportId", number_camera),
                (f"{camera_name}_setViewportResolution.inputs:width", 640),
                (f"{camera_name}_setViewportResolution.inputs:height", 480),
                (f"{camera_name}_cameraHelper.inputs:frameId", f"{camera_frame}"),
                (f"{camera_name}_cameraHelper.inputs:topicName", f"/{robot_name}/{camera_name}/rgb"),
                (f"{camera_name}_cameraHelper.inputs:type", f"rgb"),
                (f"{camera_name}_cameraHelperInfo.inputs:frameId", f"{camera_frame}"),
                (f"{camera_name}_cameraHelperInfo.inputs:topicName", f"/{robot_name}/{camera_name}/camera_info"),
                (f"{camera_name}_cameraHelperInfo.inputs:type", "camera_info"),
                (f"{camera_name}_cameraHelperInfo.inputs:stereoOffset", stereo_offset),
            ],
        },
    )

    set_targets(
        prim=stage.get_current_stage().GetPrimAtPath(ros_camera_graph_path + f"/{camera_name}_setCamera"),
        attribute="inputs:cameraPrim",
        target_prim_paths=[camera_stage_path],
    )
    return viewport_name


def create_camera_rgb_depth(robot_name, number_camera, camera_frame, camera_depth_frame, camera_stage_path, camera_name, stereo_offset=[0.0, 0.0]):
    
    ros_camera_graph_path = f"/{robot_name}/ROS_CameraGraph_{camera_name}"
    viewport_name = f"Viewport{number_camera}"
    
    # Creating an on-demand push graph with cameraHelper nodes to generate ROS image publishers
    keys = og.Controller.Keys
    og.Controller.edit(
        {
            "graph_path": ros_camera_graph_path,
            "evaluator_name": "push",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnTick"),
                ("createViewport", "omni.isaac.core_nodes.IsaacCreateViewport"),
                ("getRenderProduct", "omni.isaac.core_nodes.IsaacGetViewportRenderProduct"),
                ("setViewportResolution", "omni.isaac.core_nodes.IsaacSetViewportResolution"),
                ("setCamera", "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct"),
                ("cameraHelper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ("cameraDepth", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ("cameraHelperInfo", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", "createViewport.inputs:execIn"),
                ("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"),
                ("createViewport.outputs:execOut", "setViewportResolution.inputs:execIn"),
                ("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"),
                ("createViewport.outputs:viewport", "setViewportResolution.inputs:viewport"),
                ("setViewportResolution.outputs:execOut", "setCamera.inputs:execIn"),
                ("getRenderProduct.outputs:renderProductPath", "setCamera.inputs:renderProductPath"),
                ("setCamera.outputs:execOut", "cameraHelper.inputs:execIn"),
                ("setCamera.outputs:execOut", "cameraDepth.inputs:execIn"),
                ("setCamera.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
                ("getRenderProduct.outputs:renderProductPath", "cameraHelper.inputs:renderProductPath"),
                ("getRenderProduct.outputs:renderProductPath", "cameraDepth.inputs:renderProductPath"),
                ("getRenderProduct.outputs:renderProductPath", "cameraHelperInfo.inputs:renderProductPath"),
            ],
            keys.SET_VALUES: [
                ("createViewport.inputs:name", viewport_name),
                ("createViewport.inputs:viewportId", number_camera),
                ("setViewportResolution.inputs:width", 640),
                ("setViewportResolution.inputs:height", 480),
                ("cameraHelper.inputs:frameId", f"{camera_frame}"),
                ("cameraHelper.inputs:topicName", f"/{robot_name}/{camera_name}/rgb"),
                ("cameraHelper.inputs:type", f"rgb"),
                ("cameraDepth.inputs:frameId", f"{camera_depth_frame}"),
                ("cameraDepth.inputs:topicName", f"/{robot_name}/{camera_name}/depth"),
                ("cameraDepth.inputs:type", f"depth"),
                ("cameraHelperInfo.inputs:frameId", f"{camera_frame}"),
                ("cameraHelperInfo.inputs:topicName", f"/{robot_name}/{camera_name}/camera_info"),
                ("cameraHelperInfo.inputs:type", "camera_info"),
                ("cameraHelperInfo.inputs:stereoOffset", stereo_offset),
            ],
        },
    )

    set_targets(
        prim=stage.get_current_stage().GetPrimAtPath(ros_camera_graph_path + "/setCamera"),
        attribute="inputs:cameraPrim",
        target_prim_paths=[camera_stage_path],
    )
    return viewport_name


def build_realsense_camera_graph(robot_name,
                                baseline=0.05,
                                camera_name="camera",
                                camera_rgb_frame="color_frame",
                                camera_left_frame="infra1_frame",
                                camera_right_frame="infra2_frame",
                                camera_rgb_optical_frame="color_optical_frame",
                                camera_left_optical_frame="infra1_optical_frame",
                                camera_right_optical_frame="infra2_optical_frame",
                                show_camera=True):
    # Creating a Camera prim
    camera_rgb_stage_path = f"/{robot_name}/{camera_name}_{camera_rgb_optical_frame}/camera_rgb"
    camera_rgb_prim = UsdGeom.Camera(omni.usd.get_context().get_stage().DefinePrim(camera_rgb_stage_path, "Camera"))
    xform_api = UsdGeom.XformCommonAPI(camera_rgb_prim)
    xform_api.SetTranslate(Gf.Vec3d(0.0, 0.0, 0.0))
    xform_api.SetRotate((180, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    camera_rgb_prim.GetHorizontalApertureAttr().Set(2.0955)
    camera_rgb_prim.GetVerticalApertureAttr().Set(1.1769)
    camera_rgb_prim.GetClippingRangeAttr().Set((0.01, 10000))
    camera_rgb_prim.GetProjectionAttr().Set("perspective")
    if not show_camera:
        camera_rgb_prim.GetVisibilityAttr().Set("invisible")
    camera_rgb_prim.GetFocalLengthAttr().Set(2.4)
    camera_rgb_prim.GetFocusDistanceAttr().Set(4)
    # Build left infra/rgb camera
    viewport1 = create_camera(robot_name, 1, f"{camera_name}_{camera_rgb_frame}", camera_rgb_stage_path, "rgb")
    # Creating a Camera prim
    camera_left_stage_path = f"/{robot_name}/{camera_name}_{camera_left_optical_frame}/camera_left"
    camera_left_prim = UsdGeom.Camera(omni.usd.get_context().get_stage().DefinePrim(camera_left_stage_path, "Camera"))
    xform_api = UsdGeom.XformCommonAPI(camera_left_prim)
    xform_api.SetTranslate(Gf.Vec3d(0.0, 0.0, 0.0))
    xform_api.SetRotate((180, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    camera_left_prim.GetHorizontalApertureAttr().Set(2.0955)
    camera_left_prim.GetVerticalApertureAttr().Set(1.1769)
    camera_left_prim.GetClippingRangeAttr().Set((0.01, 10000))
    camera_left_prim.GetProjectionAttr().Set("perspective")
    if not show_camera:
        camera_left_prim.GetVisibilityAttr().Set("invisible")
    camera_left_prim.GetFocalLengthAttr().Set(2.4)
    camera_left_prim.GetFocusDistanceAttr().Set(4)
    # Build left infra/rgb camera
    viewport2 = create_camera(robot_name, 2, f"{camera_name}_{camera_left_frame}", camera_left_stage_path, "left")
    # Creating a Camera prim
    camera_right_stage_path = f"/{robot_name}/{camera_name}_{camera_right_optical_frame}/camera_right"
    camera_right_prim = UsdGeom.Camera(omni.usd.get_context().get_stage().DefinePrim(camera_right_stage_path, "Camera"))
    xform_api = UsdGeom.XformCommonAPI(camera_right_prim)
    xform_api.SetTranslate(Gf.Vec3d(0.0, 0.0, 0.0))
    xform_api.SetRotate((180, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
    camera_right_prim.GetHorizontalApertureAttr().Set(2.0955)
    camera_right_prim.GetVerticalApertureAttr().Set(1.1769)
    camera_right_prim.GetClippingRangeAttr().Set((0.01, 10000))
    camera_right_prim.GetProjectionAttr().Set("perspective")
    if not show_camera:
        camera_right_prim.GetVisibilityAttr().Set("invisible")
    camera_right_prim.GetFocalLengthAttr().Set(2.4)
    camera_right_prim.GetFocusDistanceAttr().Set(4)
    # Build right infra/rgb camera
    viewport3 = create_camera(robot_name, 3, f"{camera_name}_{camera_right_frame}", camera_right_stage_path, "right")
# EOF
