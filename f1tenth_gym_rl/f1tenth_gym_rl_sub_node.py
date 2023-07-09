import rclpy as rp
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

class SubNode(Node):

    def __init__(self):
        super().__init__('f1tenth_sub_node')
        
        
        self.subscription = self.create_subscription(
            LaserScan,                      # 메시지 타입
            'scan',                         # 토픽 이름
            self.listener_scan_callback,    # 콜백 함수
            10)                             # 큐 사이즈

    def listener_scan_callback(self, msg):
        # 여기에 레이저 스캔 데이터 처리 로직을 넣으세요.
        self.get_logger().info('I heard: "%s"' % msg.header)
        # self.get_logger().info('I heard: "%s"' % msg)

def main(args=None):
    print('Sub')
    
    rp.init(args=args)
    node = SubNode()
    rp.spin(node)
    node.destroy_node()
    rp.shutdown()    


if __name__ == '__main__':
    main()
