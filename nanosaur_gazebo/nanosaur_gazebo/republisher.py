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

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist

class Republisher(Node):
    
    def __init__(self):
        super().__init__('nanosaur_cmd')
        # Node started
        self.get_logger().info("Node republisher started!")
        
        qos_profile = QoSProfile(depth=10)
        self.repub = self.create_publisher(Twist, 'diff_drive_base_controller/cmd_vel_unstamped', qos_profile)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.drive_callback,
            10)
        self.subscription  # prevent unused variable warning
    
    def drive_callback(self, msg):
        self.repub.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    # Start Nanosaur
    republisher = Republisher()
    try:
        rclpy.spin(republisher)
    except (KeyboardInterrupt, SystemExit):
        pass
    # Destroy the node explicitly
    republisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
# EOF