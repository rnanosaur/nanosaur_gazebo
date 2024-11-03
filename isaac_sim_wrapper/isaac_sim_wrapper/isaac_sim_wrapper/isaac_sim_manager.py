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

# https://docs.ros.org/en/noetic/api/std_srvs/html/srv/Empty.html
from isaac_sim_wrapper_msgs.srv import RobotSpawner

import rclpy
from rclpy.node import Node

class IsaacSimMananger(Node):

    def __init__(self):
        super().__init__('isaac_sim_manager')
        self.cli = self.create_client(RobotSpawner, '/isaac_sim_status')
        # Get interval parameter, default 2.0
        self.declare_parameter('timeout_sec', 2.0)
        timeout_sec = self.get_parameter('timeout_sec')._value
        self.get_logger().info(f"Isaac Sim manager started, check every {timeout_sec}s")
        self.declare_parameter('robot_description', 'robot_description')
        robot_description = self.get_parameter('robot_description')._value
        self.get_logger().info(f"Topic robot description: {robot_description}")
        # Wait until Isaac Sim service starter
        while not self.cli.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().debug("Isaac Sim not yet started, waiting again...")
        self.req = RobotSpawner.Request()
        self.req.robot_description = robot_description
    
    def send_request(self):
        return self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    # Start Isaac Sim manager
    manager = IsaacSimMananger()
    future = manager.send_request()
    try:
        rclpy.spin_until_future_complete(manager, future)
    except (KeyboardInterrupt, SystemExit):
        pass
    # Destroy the node explicitly
    manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
# EOF