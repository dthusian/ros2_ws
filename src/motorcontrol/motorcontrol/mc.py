import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation
from threading import Thread
import time
import math
import serial
from status_display_msgs.msg import StatusDisplay

SPEED_LIMIT = 0.25
SPEED_SCALE = 0.8
MC_CONFIG = b"100.0,0.0,1.0"
#MC_CONFIG = b"Cl500.0,0.1,-10.0\nCr1000.0,0.1,-10.0\n"
#MC_CONFIG = b"C8.0,0.05,-0.1\n"

uart = serial.Serial("/dev/ttyACM0", 115200)
WHEEL_DIST = 0.129
M_PER_ENC_TICK = math.pi * 0.07 / 1024.0 #0.00015339807878856412
odom_x = 0.0
odom_y = 0.0
odom_angle = 0.0
shutdown = False
status = StatusDisplay()

def mc_init():
    uart.write(b"D\n\n")
    uart.flush()
    uart.write(b"C")
    uart.write(MC_CONFIG)
    uart.write(b"\n\n")
    uart.flush()

def mc_set_speed(left, right):
    mc_init()
    s = f"B{left:.4f},{right:.4f}\n"
    uart.write(s.encode("utf-8"))
    uart.flush()
    return s

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
    mc_init()
    mc_set_speed(0.0, 0.0)
    while not shutdown:
        buf = uart.readline()
        cmd = buf[0:1]
        if cmd == b"A":
            try:
                ls, _, rs = buf[1:].partition(b",")
                new_left_pos = int(ls)
                new_right_pos = int(rs)
                left_diff = handle_rollover(new_left_pos - left_pos)
                right_diff = handle_rollover(new_right_pos - right_pos)
                left_pos = new_left_pos
                right_pos = new_right_pos
                
                ld = left_diff * M_PER_ENC_TICK
                rd = -right_diff * M_PER_ENC_TICK
                
                fd = (rd + ld) / 2 
                da = (rd - ld) / WHEEL_DIST
                dx = fd * math.cos(odom_angle)
                dy = fd * math.sin(odom_angle)
                
                odom_x += dx
                odom_y += dy
                odom_angle += da
            except ZeroDivisionError:
                pass # thanks python
        elif cmd == b"D":
            mc_name = buf[1:2]
            def extract_field(name):
                pos = buf.find(name)
                return buf[pos+1:buf.find(b"\t", pos)].decode("utf-8")
            setpoint = float(extract_field(b"S"))
            velocity = float(extract_field(b"V"))
            output = float(extract_field(b"O"))
            if mc_name == b"l":
                status.left_setpoint = setpoint
                status.left_velocity = velocity
                status.left_output = output
            elif mc_name == b"r":
                status.right_setpoint = setpoint
                status.right_velocity = velocity
                status.right_output = output

class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__("mc_subscribe_cmd_vel")
        self.subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        vl, vr = mc_compute_speed(msg)
        vl *= SPEED_SCALE
        vr *= SPEED_SCALE
        mc_set_speed(max(min(vl, SPEED_LIMIT), -SPEED_LIMIT), max(min(vr, SPEED_LIMIT), -SPEED_LIMIT))

class OdomPublisher(Node):
    def __init__(self):
        super().__init__("mc_publish_odom")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)
        global logger
        logger = self.get_logger()
    
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

class StatusPublisher(Node):
    def __init__(self):
        super().__init__("mc_publish_status")
        self.publisher_ = self.create_publisher(StatusDisplay, "/amr_status", 10)
        self.timer = self.create_timer(0.05, self.timer_callback)
    
    def timer_callback(self):
        self.publisher_.publish(status)

def main(args=None):
    global shutdown, odom_x, odom_y, odom_angle
    
    rclpy.init(args=args)
    
    mc_reader = Thread(target=mc_read_thread)
    mc_reader.start()
    time.sleep(0.2)
    odom_x = 0.0
    odom_y = 0.0
    odom_angle = 0.0

    cmd_vel_subscriber = CmdVelSubscriber()
    odom_publisher = OdomPublisher()
    status_publisher = StatusPublisher()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(cmd_vel_subscriber)
    executor.add_node(odom_publisher)
    executor.add_node(status_publisher)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    
    shutdown = True
    cmd_vel_subscriber.destroy_node()
    odom_publisher.destroy_node()
    status_publisher.destroy_node()

if __name__ == '__main__':
    main()