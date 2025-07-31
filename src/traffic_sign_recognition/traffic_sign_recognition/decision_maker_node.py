import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DecisionMakerNode(Node):
    def __init__(self):
        super().__init__('decision_maker_node')
        # Create a subscriber
        self.subscription = self.create_subscription(
            String,
            'traffic_signs',  # Topic where traffic signs are published
            self.sign_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Create a publisher
        self.publisher_ = self.create_publisher(String, 'jetbot_commands', 10)
        
        self.current_sign = None

    def sign_callback(self, msg):
        self.current_sign = msg.data
        self.process_sign()

    def process_sign(self):
        if self.current_sign == "No Left Turn":
            self.get_logger().info('Prohibited from turning left')
            self.avoid_left_turn()
        elif self.current_sign == "No Right Turn":
            self.get_logger().info('Prohibited from turning right')
            self.avoid_right_turn()
        # Add more conditions based on different signs

    def avoid_left_turn(self):
        command = String()
        command.data = 'avoid_left'
        self.publisher_.publish(command)

    def avoid_right_turn(self):
        command = String()
        command.data = 'avoid_right'
        self.publisher_.publish(command)


def main(args=None):
    rclpy.init(args=args)
    decision_maker_node = DecisionMakerNode()
    rclpy.spin(decision_maker_node)
    decision_maker_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
