# Import ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

# Import message files
from nav_msgs.msg import OccupancyGrid as OccG
from nav_msgs.msg import Odometry
from autonomous_exploration_msgs.msg import MapData, PosData

# Import other libraries
import numpy as np
from scipy.spatial.transform import Rotation
import time

class RosbridgeMsgsPublisher(Node):

    def __init__(self):

        super().__init__('rosbridge_msgs_publisher')

        # Initialize the variables
        self.pos = PosData()
        self.mp = MapData()
        qos = QoSProfile(depth=10)

        # Setup rate
        self.rate = self.create_rate(2)

        # Setup subscribers
        ## /odom
        self.create_subscription(Odometry, 'odom', self._odomCallback, qos)
        ## /map
        self.create_subscription(OccG, 'map', self._mapCallback, qos)

        # Setup publishers
        ## /autonomous_exploration/map
        self.rosbridgeMap_pub = self.create_publisher(MapData, '/rosbridge_msgs_publisher/map', qos)
        ## /autonomous_exploration/position
        self.rosbridgePos_pub = self.create_publisher(PosData, '/robot_pos', qos)


        # Publish the rosbridge_msgs
        self.create_timer(0.50, self.PublishRosbridgeMsgs)  # unit: s

        self.get_logger().info('Rosbridge_msgs publisher was initiated successfully')

    
    def _mapCallback(self, data:OccG):
        
        self.mp.width = data.info.width
        self.mp.height = data.info.height
        self.mp.map = data.data
        self.mp.resolution = data.info.resolution
        map_origin = np.array([data.info.origin.position.x, data.info.origin.position.y])  

        #Publish the map using the rosbridge_msg
        self.mp.origin = [0.0] * 7
        self.mp.origin[0:2] = map_origin
    
    def _odomCallback(self, msg:Odometry):
        ''' Odometry function callback'''

        pos = msg.pose.pose.position

        self.pos.x = pos.x
        self.pos.y = pos.y

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

    try:
        rclpy.spin(RBMP)
    except KeyboardInterrupt:
        pass    #rclpy.spin_until_future_complete(SR, )
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    RBMP.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()