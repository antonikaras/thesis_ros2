# Import ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# Import message files
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid as OccG
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8MultiArray
from nav2_msgs.action import NavigateToPose
from tf2_msgs.msg import TFMessage
from autonomous_exploration_msgs.msg import MapData, PosData

# Import other libraries
import numpy as np
from scipy.spatial.transform import Rotation
import time
import threading

class RosbridgeMsgsPublisher(Node):

    def __init__(self):

        super().__init__('rosbridge_msgs_publisher')

        # Initialize the variables
        self.pos = PosData()
        self.mp = MapData()
        self.mp.origin = [0.0] * 7
        self.inPos = []
        self.odomReceived = False
        qos = QoSProfile(depth=10)

        # Create callback group
        self.top_callback_group = ReentrantCallbackGroup()

        # Setup rate
        self.rate = self.create_rate(2)

        # Setup subscribers
        ## /odom
        self.create_subscription(Odometry, 'odom', self._odomCallback, qos, callback_group=self.top_callback_group)
        ## /map
        self.create_subscription(OccG, 'map', self._mapCallback, qos, callback_group=self.top_callback_group)
        ## /ros_unity/nav_goal
        self.create_subscription(PosData, 'rosbridge_msgs_unity/nav_goal', self._navGoalCallback, qos, callback_group=self.top_callback_group)
        ## /tf
        self.create_subscription(TFMessage, 'tf', self._tfCallback, qos, callback_group=self.top_callback_group)

        # Setup publishers
        ## /rosbridge_msgs_publisher/map
        self.rosbridgeMap_pub = self.create_publisher(MapData, '/rosbridge_msgs_publisher/map', qos)
        ## /rosbridge_msgs_publisher/robot_pos
        self.rosbridgePos_pub = self.create_publisher(PosData, '/rosbridge_msgs_publisher/robot_pos', qos)

        # Create the navigation2 action client
        self.nav2ActionClient = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav2ActionClient.wait_for_server()

        # Publish the rosbridge_msgs
        self.create_timer(0.50, self.PublishRosbridgeMsgs)  # unit: s

        self.get_logger().info('Rosbridge_msgs publisher was initiated successfully')

    def _tfCallback(self, data:TFMessage):
        ''' Read the tf data and find the transformation between odom and map '''

        for tr in data.transforms:
            if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                self.mp.origin[2] = tr.transform.translation.x
                self.mp.origin[3] = tr.transform.translation.y

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

    def _navGoalCallback(self, data:PosData):
        ''' Read the target position sent by unity and call the navigation2 action server  with this goal'''

        goal_msg = NavigateToPose.Goal()

        # Generate the target goal
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()

        # Position part
        goal.pose.position.x = data.x
        goal.pose.position.y = data.y

        # Orientation part
        rot = Rotation.from_euler('xyz', [0.0, 0.0, 0.0])
        quat = rot.as_quat()
        goal.pose.orientation.x = quat[0]
        goal.pose.orientation.y = quat[1]    
        goal.pose.orientation.z = quat[2]
        goal.pose.orientation.w = quat[3]

        goal_msg.pose = goal

        future = self.nav2ActionClient.send_goal_async(goal_msg, feedback_callback = self._navGoalFeedbackCallback)
        future.add_done_callback(self._navGoalResponseCallback)

        return future

    def _mapCallback(self, data:OccG):

        self.mp.width = data.info.height
        self.mp.height = data.info.width
        
        # Rearrange the data to be visible correctly on unity
        tmp = np.array(data.data).reshape(data.info.height, data.info.width)
        
        tmp = np.rot90(np.fliplr(tmp), -1)
        tmp = np.flip(tmp, 0)
        a = tmp.flatten()
        map = [int(el) for el in a]
        #map.append(a)
        #map.data = a
        
        #self.get_logger().info("-----> {}, {} {}".format(type(map[0]), a.shape, type(map)))
        self.mp.map = map 

        self.mp.resolution = data.info.resolution
        map_origin = np.array([data.info.origin.position.x, data.info.origin.position.y])  

        #Publish the map using the rosbridge_msg
        self.mp.origin[0:2] = map_origin
    
    def _odomCallback(self, msg:Odometry):
        ''' Odometry function callback'''
        pos = msg.pose.pose.position

        self.pos.x = pos.x + self.mp.origin[2] 
        self.pos.y = pos.y + self.mp.origin[3]

        if len(self.inPos) == 0:
            self.inPos = [pos.x, pos.y]
        
        # Convert from quaternion to euler angles
        orient = msg.pose.pose.orientation
        quat_df = [orient.x, orient.y, orient.z, orient.w]
        rot = Rotation.from_quat(quat_df)
        rot_euler = rot.as_euler('xyz', degrees=True)
        self.pos.yaw = rot_euler[2]

    def PublishRosbridgeMsgs(self):
        ''' Publish the rosbridge_msgs every 500ms so that they can be used from Unity'''

        # Publish the map using the rosbridge_msg
        self.rosbridgeMap_pub.publish(self.mp)

        # Publish the robot's position using the rosbridge_msg
        self.rosbridgePos_pub.publish(self.pos)
        

###################################################################################################
def main(args=None):
    rclpy.init(args=args)

    RBMP = RosbridgeMsgsPublisher()
    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(RBMP, executor)
    except KeyboardInterrupt:
        pass    #rclpy.spin_until_future_complete(SR, )
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    RBMP.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()