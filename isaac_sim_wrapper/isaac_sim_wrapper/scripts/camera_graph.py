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


import yaml
import omni.usd as usd

from rclpy.node import Node
from isaacsim import SimulationApp
from omni.isaac.core.utils import stage
from omni.isaac.core.utils.prims import set_targets
from omni.graph.core import Controller, GraphPipelineStage
from omni.kit.viewport.window import get_viewport_window_instances
from pxr import Gf, UsdGeom

class CameraGraph:

    def __init__(self,
                 node : Node, 
                 simulation_app : SimulationApp,
                 robot_name: str,
                 number_camera: int,
                 namespace: str = "",
                 camera_name: str = "camera",
                 camera_frame: str = "frame",
                 camera_optical_frame: str = "optical_frame",
                 resolution: tuple[int, int] = [640, 480],
                 visible: bool = True):
        self._node = node
        self._simulation_app = simulation_app
        self._robot_name = robot_name
        self._camera_name = camera_name
        # status camera on Isaac Sim
        self._visible = visible
        # viewport camera name and resolution
        self._number_camera = number_camera
        self._viewport_name = f"Viewport{number_camera}"
        self._resolution = resolution
        # Path camera
        self._camera_frame = f"{camera_name}_{camera_frame}"
        self._camera_optical_frame = f"{camera_name}_{camera_optical_frame}"
        self._camera_stage_path = f"/{robot_name}/{self._camera_optical_frame}/camera_rgb"
        # If namespace is not empty add a slash
        root_topic = f"/{namespace}" if namespace else namespace
        self._camera_topic = f"{root_topic}/{camera_name}"
        # Graph path
        self._graph_path = f"/{self._robot_name}/ROS_CameraGraph_{self._camera_name}"
        # Loading camera
        node.get_logger().info(f"Camera name: {self._camera_name} - camera topic:{self._camera_topic} - Graph: {self._graph_path}")

    @classmethod
    def from_yaml(cls,
                  node : Node, 
                  simulation_app: SimulationApp,
                  robot_name: str,
                  number_camera: int,
                  file_path: str):
        with open(file_path, 'r') as file:
            config_data = yaml.safe_load(file)
        # Extract the data for the class using its name as the key, defaulting to an empty dictionary
        class_data = config_data.get(cls.__name__, {})
        # Pass the required parameters along with the extracted optional data to the class constructor
        return cls(node, simulation_app, robot_name, number_camera, **class_data)

    @classmethod
    def from_urdf(cls,
                  node : Node, 
                  simulation_app: SimulationApp,
                  robot_name: str,
                  number_camera: int,
                  urdf_sensor: str):
        # Parse the nested <camera> elements
        camera_elem = urdf_sensor.find("camera")
        horizontal_fov = float(camera_elem.findtext("horizontal_fov", 1.57))
        # Parse <image> settings
        image_elem = camera_elem.find("image")
        width = int(image_elem.findtext("width", 640))
        height = int(image_elem.findtext("height", 480))
        # Extract all values from urdf data
        class_data = {
            'namespace': urdf_sensor.findtext("name_space", ""),
            'camera_name': urdf_sensor.get("name", "camera"),
            'camera_frame': urdf_sensor.findtext("camera_frame", "frame"),
            'camera_optical_frame': urdf_sensor.findtext("camera_optical_frame", "optical_frame"),
            'resolution': [width, height],
            'visible': urdf_sensor.findtext("visualize", "true").lower() == "true"
        }
        # Pass the required parameters along with the extracted optional data to the class constructor
        return cls(node, simulation_app, robot_name, number_camera, **class_data)

    def load_camera(self):
        # Creating a Camera prim
        camera_rgb_prim = UsdGeom.Camera(usd.get_context().get_stage().DefinePrim(self._camera_stage_path, "Camera"))
        xform_api = UsdGeom.XformCommonAPI(camera_rgb_prim)
        xform_api.SetTranslate(Gf.Vec3d(0.0, 0.0, 0.0))
        xform_api.SetRotate((180, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        camera_rgb_prim.GetHorizontalApertureAttr().Set(2.0955)
        camera_rgb_prim.GetVerticalApertureAttr().Set(1.1769)
        camera_rgb_prim.GetClippingRangeAttr().Set((0.01, 10000))
        camera_rgb_prim.GetProjectionAttr().Set("perspective")
        if not self._visible:
            camera_rgb_prim.GetVisibilityAttr().Set("invisible")
        camera_rgb_prim.GetFocalLengthAttr().Set(2.4)
        camera_rgb_prim.GetFocusDistanceAttr().Set(4)
        # Build action graph
        try:
            self._load_og()
        except Exception as e:
            self._node.get_logger().error(e)
        # Update simulation
        self._simulation_app.update()
        # Set status visibility camera
        for window in get_viewport_window_instances(None):
            if window.title == self._viewport_name:
                window.visible = self._visible

    def _load_og(self):
        Controller.edit(
            {
                "graph_path": self._graph_path,
                "evaluator_name": "push",
                "pipeline_stage": GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
            },
            {
                Controller.Keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("createViewport", "omni.isaac.core_nodes.IsaacCreateViewport"),
                    ("getRenderProduct", "omni.isaac.core_nodes.IsaacGetViewportRenderProduct"),
                    ("setViewportResolution", "omni.isaac.core_nodes.IsaacSetViewportResolution"),
                    ("setCamera", "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct"),
                    ("cameraHelper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ("cameraHelperInfo", "omni.isaac.ros2_bridge.ROS2CameraInfoHelper"),
                    ],
                Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "createViewport.inputs:execIn"),
                    ("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"),
                    ("createViewport.outputs:execOut", "setViewportResolution.inputs:execIn"),
                    ("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"),
                    ("createViewport.outputs:viewport", "setViewportResolution.inputs:viewport"),
                    ("setViewportResolution.outputs:execOut", "setCamera.inputs:execIn"),
                    ("getRenderProduct.outputs:renderProductPath", "setCamera.inputs:renderProductPath"),
                    ("setCamera.outputs:execOut", "cameraHelper.inputs:execIn"),
                    ("setCamera.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
                    ("getRenderProduct.outputs:renderProductPath", "cameraHelper.inputs:renderProductPath"),
                    ("getRenderProduct.outputs:renderProductPath", "cameraHelperInfo.inputs:renderProductPath"),
                    ],
                Controller.Keys.SET_VALUES: [
                    ("createViewport.inputs:name", self._viewport_name),
                    ("createViewport.inputs:viewportId", self._number_camera),
                    ("setViewportResolution.inputs:width", self._resolution[0]),
                    ("setViewportResolution.inputs:height", self._resolution[1]),
                    ("cameraHelper.inputs:frameId", self._camera_frame),
                    ("cameraHelper.inputs:topicName", f"{self._camera_topic}/rgb"),
                    ("cameraHelper.inputs:type", "rgb"),
                    ("cameraHelperInfo.inputs:frameId", self._camera_frame),
                    ("cameraHelperInfo.inputs:topicName", f"{self._camera_topic}/camera_info"),
                ]
            }
        )

        set_targets(
            prim=stage.get_current_stage().GetPrimAtPath(f"{self._graph_path}/setCamera"),
            attribute="inputs:cameraPrim",
            target_prim_paths=[self._camera_stage_path],
        )
# EOF
