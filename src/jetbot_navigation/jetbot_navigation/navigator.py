import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
 
class Navigator(Node):
    def __init__(self):
        super().__init__('navigator')
        self.keyboard_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.keyboard_callback,
            10)
        self.obstacle_sub = self.create_subscription(
            Bool,
            '/obstacle',
            self.obstacle_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/jetbot/cmd_vel', 10)
        self.last_cmd = Twist()
        self.obstacle_detected = False
 
    def keyboard_callback(self, msg):
        self.last_cmd = msg
        self.evaluate_command()
 
    def obstacle_callback(self, msg):
        self.obstacle_detected = msg.data
        self.evaluate_command()
 
    def evaluate_command(self):
        if not self.obstacle_detected:
            self.publisher_.publish(self.last_cmd)
        else:
            # Stop or navigate around the obstacle
            stop_cmd = Twist()
            stop_cmd.linear.x = 0
            stop_cmd.angular.z = 0
            self.publisher_.publish(stop_cmd)
 
def main(args=None):
    rclpy.init(args=args)
    navigator = Navigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()

