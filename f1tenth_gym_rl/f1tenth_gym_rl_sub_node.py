import rclpy as rp
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped

class SubNode(Node):

    def __init__(self):
        super().__init__('f1tenth_rl_sub_node')
        
        
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
                

    def listener_scan_callback(self, msg = LaserScan()):
        
        self.scan_msg = msg
        
        # self.get_logger().info('I heard: "%s"' % msg.header)
        # self.get_logger().info('I heard: "%s"' % msg)

    def listener_goal_callback(self, msg = PoseStamped()):
        
        self.goal_msg = msg
        
        self.get_logger().info('I heard: "%s"' % msg.header)
        self.get_logger().info('I heard: "%s"' % msg)
        
        
def main(args=None):
    print('Sub')
    
    rp.init(args=args)
    node = SubNode()
    rp.spin(node)
    node.destroy_node()
    rp.shutdown()    


if __name__ == '__main__':
    main()
