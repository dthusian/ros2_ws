import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
from threading import Thread
import time
import math
import serial

uart = serial.Serial("/dev/ttyACM0", 115200)
odom_x = 0.0
odom_y = 0.0
odom_angle = 0.0
shutdown = False
last_send_time = time.time()

def mc_set_speed(left, right):
    uart.write(b"C8.0,0.05,-1.0\n")
    s = f"B{left:.4f},{right:.4f}\n"
    uart.write(s.encode("utf-8"))
    uart.flush()
    return s

def mc_stop_motors():
    mc_set_speed(0.0, 0.0)

WHEEL_DIST = 0.125

def mc_compute_speed(msg):
    angle = msg.angular.z
    speed = msg.linear.x
    left = speed - angle * WHEEL_DIST / 2
    right = speed + angle * WHEEL_DIST / 2
    return (left, right)

def mc_read_thread():
    global odom_angle, odom_x, odom_y, shutdown
    while uart.read() != b"\n":
        pass
    while not shutdown:
        buf = uart.readline()
        cmd = buf[0:1]
        if cmd == b"A":
            try:
                ls, _, rs = buf[1:].partition(b",")
                ld = round(float(ls) * 10000) / 10000
                rd = round(float(rs) * 10000) / 10000
                if abs(ld - rd) < 0.0001:
                    odom_x += ld * math.sin(odom_angle)
                    odom_y += rd * math.cos(odom_angle)
                else:
                    a = 0
                    angle = 0
                    if ld < rd:
                        # solve ld = a * angle; rd = (d + a) * angle
                        # rd/ld = (d + a)/a = d/a + 1
                        a = WHEEL_DIST/(rd/ld - 1)
                        angle = ld / a
                    elif ld > rd:
                        # swap ld/rd in above
                        a = WHEEL_DIST/(ld/rd - 1)
                        angle = rd / a
                    odom_angle += angle
                    r = a + WHEEL_DIST / 2
                    odom_x += r * math.sin(angle)
                    odom_y += r * math.cos(angle)
            except ZeroDivisionError:
                pass # thanks python

mc_stop_motors()
time.sleep(1)
mc_reader = Thread(target=mc_read_thread)
mc_reader.start()
time.sleep(1)

class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__("mc_subscribe_cmd_vel")
        self.subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        global last_send_time
        if time.time() - last_send_time > 0.1:
            spd = mc_compute_speed(msg)
            mc_set_speed(max(min(spd[0], 0.15), -0.15), max(min(spd[1], 0.15), -0.15))
            last_send_time = time.time()

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
       t.transform.translation.x = odom_x
       t.transform.translation.y = odom_y
       t.transform.translation.z = 0.0
       t.transform.rotation.x = 0.0
       t.transform.rotation.y = 0.0
       t.transform.rotation.z = odom_angle
       t.transform.rotation.w = 1.0

       self.tf_broadcaster.sendTransform(t)

def main(args=None):
    global shutdown
    
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
    
    shutdown = True
    mc_stop_motors()
    mc_stop_motors()
    cmd_vel_subscriber.destroy_node()
    odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()