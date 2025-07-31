import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput.keyboard import Key, Listener

class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist = Twist()
        self.listener_thread = threading.Thread(target=self.listen_to_keyboard)
        self.listener_thread.start()

    def on_press(self, key):
        try:
            if key.char == 'w':  # forward
                self.twist.linear.x = 0.1
            elif key.char == 's':  # backward
                self.twist.linear.x = -0.1
            elif key.char == 'a':  # left
                self.twist.angular.z = 0.1
            elif key.char == 'd':  # right
                self.twist.angular.z = -0.1
            self.publisher_.publish(self.twist)
        except AttributeError:
            pass

    def on_release(self, key):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.publisher_.publish(self.twist)
        if key == Key.esc:
            # Stop listener
            return False

    def listen_to_keyboard(self):
        with Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            listener.join()

    def __del__(self):
        # Ensure all threads are cleaned up when the node is destroyed
        self.listener_thread.join()

def main(args=None):
    rclpy.init(args=args)
    keyboard_control = KeyboardControl()
    rclpy.spin(keyboard_control)
    keyboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
