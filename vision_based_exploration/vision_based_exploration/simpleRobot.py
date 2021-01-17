# Import ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile

# Import message files
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid as OccG
from nav_msgs.msg import MapMetaData as MMD
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose

# Import other libraries
import numpy as np
from scipy.spatial.transform import Rotation
import sys
import os
import threading


class SimpleRobot(Node):

    def __init__(self):
        super().__init__('simple_robot')

        # Initialize variables
        self.in_pos = []
        self.pos = [0.0] * 3
        self.tar_pos = [0.0] * 3
        qos = QoSProfile(depth=10)

        # Create subscribers
        # /odom
        self.create_subscription(Odometry, 'odom', self._odomCallback, qos)

        # Create publishers
        # /goal_pose
        self.goalPose_pub = self.create_publisher(PoseStamped, '/goal_pose', qos)

        # Create the navigation2 action client
        self.actionClient = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.actionClient.wait_for_server()

        # Read the keyboard inputs
        self.create_timer(0.50, self.ReadInput)  # unit: s

    def _odomCallback(self, msg:Odometry):
        pos = msg.pose.pose.position

        self.pos[0:2] = [pos.x, pos.y]

        # Convert from quaternion to euler angles
        orient = msg.pose.pose.orientation
        quat_df = [orient.x, orient.y, orient.z, orient.w]
        rot = Rotation.from_quat(quat_df)
        rot_euler = rot.as_euler('xyz', degrees=True)
        self.pos[2] = rot_euler[2]

    def GoToPos(self):
        ''' Go to a desired pos @in : x, y, yaw '''
        while True:
          goal = PoseStamped()
          goal.header.frame_id = 'map'
          goal.header.stamp = self.get_clock().now().to_msg()

          # Position part
          goal.pose.position.x = self.tar_pos[0]
          goal.pose.position.y = self.tar_pos[1]

          # Orientation part
          rot = Rotation.from_euler('xyz', [0.0, 0.0, self.tar_pos[2]])
          quat = rot.as_quat()
          goal.pose.orientation.x = quat[0]
          goal.pose.orientation.y = quat[1]    
          goal.pose.orientation.z = quat[2]
          goal.pose.orientation.w = quat[3]

          self.goalPose_pub.publish(goal)

    def _sendNavGoal(self):
        goal_msg = NavigateToPose.Goal()

        # Generate the target goal
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()

        # Position part
        goal.pose.position.x = self.tar_pos[0]
        goal.pose.position.y = self.tar_pos[1]

        # Orientation part
        rot = Rotation.from_euler('xyz', [0.0, 0.0, self.tar_pos[2]])
        quat = rot.as_quat()
        goal.pose.orientation.x = quat[0]
        goal.pose.orientation.y = quat[1]    
        goal.pose.orientation.z = quat[2]
        goal.pose.orientation.w = quat[3]

        goal_msg.pose = goal

        return self.actionClient.send_goal_async(goal_msg)


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
          #self.GoToPos()
          self._sendNavGoal()


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