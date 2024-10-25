import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster

import serial

uart = serial.Serial("/dev/ttyACM0", 115200)
current_left = 0
current_right = 1

def set_pwm(left, right):
    left = min(max(left, 0), 1)
    right = min(max(right, 0), 1)
    left_byte = left * 255
    right_byte = right * 255
    uart.write(b"\x00\x00\x00\x01")
    uart.write(bytes([left_byte, right_byte]))
    uart.flush()

def stop_motors():
    uart.write("\x00\x00\x00\x02\x02\x02\x00\x00\x00")
    uart.flush()

WHEEL_DIST = 0.125

def compute_pwm(msg):
    angle = msg.angular.z
    speed = msg.linear.x
    left = speed - angle * WHEEL_DIST / 2
    right = speed + angle * WHEEL_DIST / 2
    return (left, right)

class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__("mc_subscribe_cmd_vel")
        self.subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        set_pwm(*compute_pwm(msg))

class OdomPublisher(Node):
    def __init__(self):
        super().__init__("mc_publish_odom")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)
    
    def broadcast_timer_callback(self):
       t = TransformStamped()

       t.header.stamp = self.get_clock().now().to_msg()
       t.header.frame_id = "odom"
       t.child_frame_id = "base_link"
       t.transform.translation.x = 0.0
       t.transform.translation.y = 0.0
       t.transform.translation.z = 0.0
       t.transform.rotation.x = 0.0
       t.transform.rotation.y = 0.0
       t.transform.rotation.z = 0.0
       t.transform.rotation.w = 1.0

       self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)

    cmd_vel_subscriber = CmdVelSubscriber()
    odom_publisher = OdomPublisher()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(cmd_vel_subscriber)
    executor.add_node(odom_publisher)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
        
    uart.write(b"\x01\x00\x00")
    cmd_vel_subscriber.destroy_node()
    odom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()