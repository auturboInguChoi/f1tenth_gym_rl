import rclpy as rp
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped


class PubNode(Node):

    def __init__(self):
        super().__init__('f1tenth_pub_node')
        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.init_publisher = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

        self.count = 0

        timer_period = 0.1  # 초
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()

        # Twist 메시지 설정. 이 예제에서는 선형 속도 x를 0.5로, 각속도 z를 0.1로 설정합니다.
        msg.linear.x = 0.5
        msg.angular.z = 0.00

        # 메시지 게시
        self.cmd_publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)
        
        ######################################
        
        self.count = self.count + 1
        
        if(self.count >= 20):
            init_msg = PoseWithCovarianceStamped()
            t = self.get_clock().now()
            init_msg.header.stamp = t.to_msg()
            init_msg.header.frame_id = "map"
            
            init_msg.pose.pose.position.x = 0.14596080780029297
            init_msg.pose.pose.position.y = -0.1318468600511551
            init_msg.pose.pose.position.z = 0.0
            
            init_msg.pose.pose.orientation.w = 0.0
            init_msg.pose.pose.orientation.x = 0.0
            init_msg.pose.pose.orientation.y = 0.0
            init_msg.pose.pose.orientation.z = 0.0
            
            self.init_publisher.publish(init_msg)
            
            self.count = 0
            

def main(args=None):
    print('Pub')    
    
    rp.init(args=args)
    node = PubNode()
    rp.spin(node)
    node.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()