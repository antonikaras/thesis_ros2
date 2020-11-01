# Import ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

# Import message files
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

# Import other libraries
import numpy as np
from scipy.spatial.transform import Rotation
import sys
import os
import threading

class SimpleRobot(Node):
  '''A simple terminal controller for the turtlebot3 robot'''
  
  def __init__(self):
    super().__init__('simpler_robot')
    #self.publisher_ = self.create_publisher(String, 'topic', 10)
    #timer_period = 0.5  # seconds
    #self.timer = self.create_timer(timer_period, self.timer_callback)
    #self.i = 0    

    # Initialize variables
    self.pos = [0.0] * 4
    self.tar_pos = [0.0] * 3
    qos = QoSProfile(depth=10)

    # Create subscribers
    # /odom
    self.create_subscription(Odometry, 'odom', self._odomCallback, qos)

    # Create publishers
    # /goal_pose
    self.goalPose_pub = self.create_publisher(PoseStamped, '/goal_pose', qos)

    # Read the keyboard inputs
    self.create_timer(0.50, self.ReadInput)  # unit: s

  def _odomCallback(self, msg:Odometry):
    pos = msg.pose.pose.position

    self.pos[0:3] = [pos.x, pos.y, pos.z]

    # Convert from quaternion to euler angles
    orient = msg.pose.pose.orientation
    quat_df = [orient.x, orient.y, orient.z, orient.w]
    rot = Rotation.from_quat(quat_df)
    rot_euler = rot.as_euler('xyz', degrees=True)
    self.pos[3] = rot_euler[2]

  def GoToPos(self):
    ''' Go to a desired pos @in : x, y, z '''
    while True:
      goal = PoseStamped()
      goal.header.frame_id = 'map'
      goal.header.stamp = self.get_clock().now().to_msg()

      # Position part
      goal.pose.position.x = self.tar_pos[0]
      goal.pose.position.y = self.tar_pos[1]
      goal.pose.position.z = self.pos[2]

      # Orientation part
      rot = Rotation.from_euler('xyz', [0.0, 0.0, self.tar_pos[2]])
      quat = rot.as_quat()
      goal.pose.orientation.x = quat[0]
      goal.pose.orientation.y = quat[1]    
      goal.pose.orientation.z = quat[2]
      goal.pose.orientation.w = quat[3]

      self.goalPose_pub.publish(goal)
    
  def ReadInput(self):
    inp = input('Type your command > ')
    inp = inp.split()
    if inp[0] == 'exit':
      pass
    elif inp[0] == 'pos':
      print(self.pos)
    elif inp[0] == 'goto':
      x = float(inp[1])
      y = float(inp[2])
      yaw = float(inp[3])
      self.tar_pos = [x, y, yaw]
      self.GoToPos()

def main(args=None):
    rclpy.init(args=args)

    SR = SimpleRobot()

    rclpy.spin(SR)
    #rclpy.spin_until_future_complete(SR, )
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    #SR.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Interrupted')
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)
