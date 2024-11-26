import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation
from threading import Thread
import time
import math
import serial

uart = serial.Serial("/dev/ttyACM0", 115200)
WHEEL_DIST = 0.125
M_PER_ENC_TICK = 0.00015339807878856412
odom_x = 0.0
odom_y = 0.0
odom_angle = 0.0
shutdown = False

def mc_set_speed(left, right):
    uart.write(b"C8.0,0.05,-1.0\n")
    s = f"B{left:.4f},{right:.4f}\n"
    uart.write(s.encode("utf-8"))
    uart.flush()
    return s

def mc_stop_motors():
    mc_set_speed(0.0, 0.0)

def mc_compute_speed(msg):
    angle = msg.angular.z
    speed = msg.linear.x
    left = speed - angle * WHEEL_DIST / 2
    right = speed + angle * WHEEL_DIST / 2
    return (left, right)

def mc_read_thread():
    global odom_angle, odom_x, odom_y, shutdown
    def handle_rollover(d):
        if d > 512:
            return d - 1024
        elif d < -512:
            return d + 1024
        return d
    left_pos = 0
    right_pos = 0
    while uart.read() != b"\n":
        pass
    while not shutdown:
        buf = uart.readline()
        cmd = buf[0:1]
        if cmd == b"A":
            try:
                ls, _, rs = buf[1:].partition(b",")
                new_left_pos = int(ls)
                new_right_pos = int(rs) * 0.00015339807878856412
                left_diff = handle_rollover(new_left_pos - left_pos)
                right_diff = handle_rollover(new_right_pos - right_pos)
                left_pos = new_left_pos
                right_pos = new_right_pos
                
                ld = left_diff * M_PER_ENC_TICK
                rd = right_diff * M_PER_ENC_TICK
                
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

class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__("mc_subscribe_cmd_vel")
        self.subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        if True: #time.time() - last_send_time > 0.1:
            spd = mc_compute_speed(msg)
            mc_set_speed(max(min(spd[0], 0.15), -0.15), max(min(spd[1], 0.15), -0.15))
            #last_send_time = time.time()

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
        
        r = Rotation.from_euler("xyz", [0, 0, odom_angle])
        q = Quaternion()
        q.x = r.as_quat()[0]
        q.y = r.as_quat()[1]
        q.z = r.as_quat()[2]
        q.w = r.as_quat()[3]
        t.transform.rotation = q

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    global shutdown, odom_x, odom_y, odom_angle
    
    rclpy.init(args=args)
    
    mc_stop_motors()
    time.sleep(1)
    mc_reader = Thread(target=mc_read_thread)
    mc_reader.start()
    time.sleep(1)
    odom_x = 0
    odom_y = 0
    odom_angle = 0

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