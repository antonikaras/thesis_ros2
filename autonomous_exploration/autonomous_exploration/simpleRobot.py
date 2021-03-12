#/usr/bin/python3

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
from tf2_msgs.msg import TFMessage
from autonomous_exploration_msgs.action import AutonomousExplorationAction

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
        self.map_pos = [0.0] * 2
        self.tar_pos = [0.0] * 3
        self.goal_id = 0
        self.remaining_distance = 0.0
        self.mapOdomOffset = [0.0, 0.0]
        qos = QoSProfile(depth=10)

        # Create subscribers
        # /odom
        self.create_subscription(Odometry, 'odom', self._odomCallback, qos)
        ## /map
        self.create_subscription(OccG, 'map', self._mapCallback, qos)
        ## /tf
        self.create_subscription(TFMessage, 'tf', self._tfCallback, qos)

        # Create publishers
        # /goal_pose
        self.goalPose_pub = self.create_publisher(PoseStamped, '/goal_pose', qos)

        # Create the navigation2 action client
        self.nav2ActionClient = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav2ActionClient.wait_for_server()

        # Create the autonomous exploration action client
        self.aeActionClient = ActionClient(self, AutonomousExplorationAction, 'autonomous_exploration')
        self.aeActionClient.wait_for_server()

        # Read the keyboard inputs
        self.create_timer(0.50, self.ReadInput)  # unit: s

    def _tfCallback(self, data:TFMessage):
        ''' Read the tf data and find the transformation between odom and map '''

        for tr in data.transforms:
            if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                self.mapOdomOffset[0] = tr.transform.translation.x
                self.mapOdomOffset[1] = tr.transform.translation.y

    def _odomCallback(self, msg:Odometry):
        pos = msg.pose.pose.position

        self.pos[0:2] = [pos.x + self.mapOdomOffset[0], pos.y + self.mapOdomOffset[1]]

        # Convert from quaternion to euler angles
        orient = msg.pose.pose.orientation
        quat_df = [orient.x, orient.y, orient.z, orient.w]
        rot = Rotation.from_quat(quat_df)
        rot_euler = rot.as_euler('xyz', degrees=True)
        self.pos[2] = rot_euler[2]

    def _mapCallback(self, data:MMD):

        self.map_pos[0] = int((self.pos[0] - data.info.origin.position.x) / data.info.resolution)
        self.map_pos[1] = int((self.pos[1] - data.info.origin.position.y) / data.info.resolution)

    def _navGoalResponseCallback(self, future:rclpy.Future):
        ''' Callback to process the request send to the navigtion2 action server '''
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._navGoalResultCallback)

    def _navGoalResultCallback(self, future:rclpy.Future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))

    def _navGoalFeedbackCallback(self, data):
        self.remaining_distance = data.feedback.distance_remaining

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

        future = self.nav2ActionClient.send_goal_async(goal_msg, feedback_callback = self._navGoalFeedbackCallback)
        future.add_done_callback(self._navGoalResponseCallback)

        return future

    def _aeGoalResponseCallback(self, future : rclpy.Future):
        goal_handle = future.result()

        # Goal wasn't accepted
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            #self.goal_sent = -1
            return
        else:
            self.get_logger().info('Autonomous exploration goal accepted :)')
        self.ae_goal_handle = goal_handle

        # Goal was accepted
        #self.goal_sent = 1
        #self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._aeGoalResultCallback)
    
    def _aeGoalResultCallback(self, future:rclpy.Future):
        pass
        #result = future.result().result
        #self.get_logger().info('Result: {0}'.format(result.result))
    
    def _aeFeedbackCallback(self, data):
        self.remaining_distance = data.feedback.remaining_distance
        self.tar_pos = data.feedback.goal
        self.goal_id = data.feedback.goal_id

    def Explore(self, timeOut : float, maxSteps : int, method : str) -> bool:
        ''' 
            Function used to call the explore action
            @timeout  : Used to avoid the recovery mode of the robot when stuck in a position 
            @maxSteps : Maximum number of goals to explore
            @method   : Frontier detection method to be used
            @status   : Return value, False -> robot stuck, True -> maxSteps reached
        '''

        # Prepare the goal message
        goal_msg = AutonomousExplorationAction.Goal()
        goal_msg.max_steps = maxSteps
        goal_msg.time_out = timeOut
        goal_msg.method = method

        # Send the goal message to the action server
        future = self.aeActionClient.send_goal_async(goal_msg, feedback_callback = self._aeFeedbackCallback)
        future.add_done_callback(self._aeGoalResponseCallback)

    def ReadInput(self):
        inp = input('Type your command > ')
        inp = inp.split()
        if len(inp) == 0:
            pass
        elif inp[0] == 'exit':
          pass
        elif inp[0] == 'pos':
          print(self.pos, self.map_pos)
        elif inp[0] == 'goto':
          x = float(inp[1])
          y = float(inp[2])
          yaw = float(inp[3])
          self.tar_pos = [x, y, yaw]
          self._sendNavGoal()
          #self.GoToPos()
        elif inp[0] == 'status':
            self.get_logger().info('Remaining distance: {}'.format(self.remaining_distance))
            self.get_logger().info('Target goal position: {}'.format(self.tar_pos))
            self.get_logger().info('Goal id: {}'.format(self.goal_id))
        elif inp[0] == 'cancel':
            self.ae_goal_handle.cancel_goal_async()
        elif inp[0] == 'explore':
            if len(inp) == 2:
                self.Explore(timeOut = 5.0, maxSteps = int(inp[1]), method = 'vis')
            else:
                self.Explore(timeOut = 5.0, maxSteps = 50, method = 'vis')


def main(args=None):
    rclpy.init(args=args)

    SR = SimpleRobot()

    try:
        rclpy.spin(SR)
    except KeyboardInterrupt:
        pass
    #rclpy.spin_until_future_complete(SR, )
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    SR.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()