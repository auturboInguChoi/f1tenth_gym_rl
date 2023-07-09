import rclpy as rp
from rclpy.node import Node
from geometry_msgs.msg import Twist

class PubNode(Node):

    def __init__(self):
        super().__init__('f1tenth_pub_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # 초
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()

        # Twist 메시지 설정. 이 예제에서는 선형 속도 x를 0.5로, 각속도 z를 0.1로 설정합니다.
        msg.linear.x = 0.5
        msg.angular.z = 0.00

        # 메시지 게시
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    print('Pub')    
    
    rp.init(args=args)
    node = PubNode()
    rp.spin(node)
    node.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()