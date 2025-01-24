import rclpy
from rclpy.node import Node
from threading import Thread
from status_display_msgs.msg import StatusDisplay
import matplotlib.pyplot as plt
import numpy as np

GRAPH_LEN = 200

ls_data = [0 for _ in range(GRAPH_LEN)]
lv_data = [0 for _ in range(GRAPH_LEN)]
lo_data = [0 for _ in range(GRAPH_LEN)]
rs_data = [0 for _ in range(GRAPH_LEN)]
rv_data = [0 for _ in range(GRAPH_LEN)]
ro_data = [0 for _ in range(GRAPH_LEN)]

class StatusSubscriber(Node):
  def __init__(self):
    super().__init__("status_subscriber")
    self.subscription = self.create_subscription(StatusDisplay, "/amr_status", self.listener_callback, 10)
    
  def listener_callback(self, msg):
    ls_data.pop(0)
    lv_data.pop(0)
    lo_data.pop(0)
    rs_data.pop(0)
    rv_data.pop(0)
    ro_data.pop(0)
    
    ls_data.append(msg.left_setpoint)
    lv_data.append(msg.left_velocity)
    lo_data.append(msg.left_output)
    rs_data.append(msg.right_setpoint)
    rv_data.append(msg.right_velocity)
    ro_data.append(msg.right_output)

def plotter_main():
  plt.ion()
  fig = plt.figure(figsize=(10, 5))
  leftv_graph = fig.add_subplot(2, 2, 1, ylim=(-0.25, 0.25))
  lefto_graph = fig.add_subplot(2, 2, 2, ylim=(-255, 255))
  rightv_graph = fig.add_subplot(2, 2, 3, ylim=(-0.25, 0.25))
  righto_graph = fig.add_subplot(2, 2, 4, ylim=(-255, 255))

  ls_line, = leftv_graph.plot(np.linspace(0, 1, GRAPH_LEN), ls_data, "r-")
  lv_line, = leftv_graph.plot(np.linspace(0, 1, GRAPH_LEN), lv_data, "b-")
  lo_line, = lefto_graph.plot(np.linspace(0, 1, GRAPH_LEN), lo_data, "g-")
  rs_line, = rightv_graph.plot(np.linspace(0, 1, GRAPH_LEN), rs_data, "r-")
  rv_line, = rightv_graph.plot(np.linspace(0, 1, GRAPH_LEN), rv_data, "b-")
  ro_line, = righto_graph.plot(np.linspace(0, 1, GRAPH_LEN), ro_data, "g-")
  while True:
    fig.canvas.draw()
    fig.canvas.flush_events()
    
    ls_line.set_ydata(ls_data)
    lv_line.set_ydata(lv_data)
    lo_line.set_ydata(lo_data)
    rs_line.set_ydata(rs_data)
    rv_line.set_ydata(rv_data)
    ro_line.set_ydata(ro_data)
    
def node_main():
  rclpy.init()
  node = StatusSubscriber()
  rclpy.spin(node)

def main():
  node_thread = Thread(target=node_main)
  node_thread.start()
  plotter_main()
  rclpy.shutdown()

if __name__ == "__main__":
  main()