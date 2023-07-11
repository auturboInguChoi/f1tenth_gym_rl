#!/usr/bin/env python3 
 
import rclpy as rp
from rclpy.node import Node 
 
from tf2_ros import TransformException  
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener 

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped

import math 
 
class F1TenthRosIntegration(Node):

    def __init__(self):
        # Initiate the Node class's constructor and give it a name
        super().__init__('f1tenth_rl_integration_node')

        ################################################################
        ### Subscriber
        ################################################################
        self.subscription = self.create_subscription(
            LaserScan,                      # 메시지 타입
            'scan',                         # 토픽 이름
            self.listener_scan_callback,    # 콜백 함수
            10)                             # 큐 사이즈

        self.subscription = self.create_subscription(
            PoseStamped,                      # 메시지 타입
            'goal_pose',                         # 토픽 이름
            self.listener_goal_callback,    # 콜백 함수
            1)                             # 큐 사이즈

        self.scan_msg = LaserScan()
        self.goal_msg = PoseStamped()
        
        ################################################################
        ### Publisher
        ################################################################
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.init_publisher = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

        ################################################################
        ### TF
        ################################################################
        # Declare and acquire `target_frame` parameter
        self.declare_parameter('target_frame', 'ego_racecar/base_link')
        self.target_frame = self.get_parameter(
        'target_frame').get_parameter_value().string_value
    
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
    
        # Call on_timer function on a set interval
        tf_timer_period = 0.01
        self.timer = self.create_timer(tf_timer_period, self.on_timer)
        
        self.current_x = 0.0
        self.current_y = 0.0 
        self.current_yaw = 0.0


    def listener_scan_callback(self, msg = LaserScan()):
        
        self.scan_msg = msg
        
        # print("listener_scan_callback")

        # self.get_logger().info('I heard: "%s"' % msg.header)
        # self.get_logger().info('I heard: "%s"' % msg)

    def listener_goal_callback(self, msg = PoseStamped()):
        
        self.goal_msg = msg
        
        self.get_logger().info('I heard: "%s"' % msg.header)
        self.get_logger().info('I heard: "%s"' % msg)    
    
    def on_timer(self):

        from_frame_rel = self.target_frame
        to_frame_rel = 'map'
    
        trans = None
     
        try:
            now = rp.time.Time()
            trans = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        now)
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
        
            return
       
    # Publish the 2D pose
        self.current_x = trans.transform.translation.x
        self.current_y = trans.transform.translation.y    
        roll, pitch, yaw = self.euler_from_quaternion(
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w)
        self.current_yaw = yaw    

        # print("=== TF ===")
        # print("x = ", trans.transform.translation.x)
        # print("y = ", trans.transform.translation.y)
    
   
    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
            
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
            
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
            
        return roll_x, pitch_y, yaw_z # in radians
 
def main(args=None):
    
    rp.init(args=args)
  
    node = F1TenthRosIntegration()
    rp.spin(node)
    node.destroy_node()
  
  # Shutdown the ROS client library for Python
    rp.shutdown()
  
if __name__ == '__main__':
    main()