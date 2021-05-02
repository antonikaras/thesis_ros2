# Import ROS2 libraries
from interactive_map_tester.pointGroup import PointsGroup
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor

# Import message files
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from autonomous_exploration_msgs.msg import MapData, PointGroup, PointGroups

# Import other libraries
import numpy as np
import cv2 as cv
from scipy.spatial.transform import Rotation

class InteractiveMapDemo(Node):
    """ 
        Demonstrates the use of the intractive map,
        once the robot enters one of the areas it 
        will perform a specific Task
    """ 

    def __init__(self):
        super().__init__("interactive_map_demo")

        # Initialize the variables
        qos = QoSProfile(depth=10)
        self.pos = [0.0, 0.0, 0.0]
        self.mapOdomOffset = []
        self.mapPos = [0, 0]
        self.interactiveMap = []
        self.mapOrigin = [0.0, 0.0]
        self.mapResolution = 0.05
        self.robotInAreas = []
        self.width = 1
        self.height = 1
        self.pointGroups = []

        # Create subscribers
        ## /odom
        self.create_subscription(Odometry, 'odom', self._odomCallback, qos)
        ## /tf
        self.create_subscription(TFMessage, 'tf', self._tfCallback, qos)
        ## /maps_publisher/interactive_map
        self.create_subscription(MapData, 'maps_publisher/interactive_map', self._interactiveMapCallback, qos)
        ## /rosbridge_msgs_unity/point_groups
        self.create_subscription(PointGroups, 'rosbridge_msgs_unity/point_groups', self._pointGroupsCallback, qos)

        # Create a timer to check if the robot is in one of the predefined areas
        self.create_timer(.5, self.RobotInAreaChecker)  # unit: s
    
    def _pointGroupsCallback(self, msg : PointGroups) -> None:
        """ Read the point group msg published either from ROS2 or Unity and check if the robot is inside """

        groupID = 0
        for group in msg.groups:
            if len(self.pointGroups) - 1 < groupID:
                self.pointGroups.append(PointsGroup(group)) 
            else:
                numOfPoints = len(group.map_pos)
                if numOfPoints != self.pointGroups[groupID].numOfPoints:
                    self.pointGroups[groupID] = PointsGroup(group)
            
            groupID += 1


    def _tfCallback(self, data:TFMessage):
        ''' Read the tf data and find the transformation between odom and map '''

        for tr in data.transforms:
            if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                if (len(self.mapOdomOffset) == 0):
                    self.get_logger().info("Interactive map demo node was initiated successfully")
                    self.mapOdomOffset = [0.0] * 2
                self.mapOdomOffset[0] = tr.transform.translation.x
                self.mapOdomOffset[1] = tr.transform.translation.y

    def _odomCallback(self, msg:Odometry):
        
        # Don't publish the map in case the initial pose is not published
        if (len(self.mapOdomOffset) == 0):
            return
        
        pos = msg.pose.pose.position
        
        #self.pos[0:2] = [pos.x + self.mapOdomOffset[0], pos.y + self.mapOdomOffset[1]]
        self.pos[0:2] = [pos.x + self.mapOdomOffset[0], pos.y + self.mapOdomOffset[1]]

        # Convert from quaternion to euler angles
        orient = msg.pose.pose.orientation
        quat_df = [orient.x, orient.y, orient.z, orient.w]
        rot = Rotation.from_quat(quat_df)
        rot_euler = rot.as_euler('xyz', degrees=True)
        self.pos[2] = rot_euler[2]

        # Convert the robot pose to map index
        self.mapPos[0] = int((self.pos[0] - self.mapOrigin[0]) / self.mapResolution)
        self.mapPos[1] = int((self.pos[1] - self.mapOrigin[1]) / self.mapResolution)

    def _interactiveMapCallback(self, data:MapData):
        
        # Store the map Info
        self.width = data.width
        self.height = data.height
        self.mapOrigin[0] = data.origin[0]
        self.mapOrigin[1] = data.origin[1]
        self.mapResolution = data.resolution

        # Convert the interactive map to 2D
        self.interactiveMap = np.array(data.map).reshape(self.width, self.height)
    
    def RobotInAreaChecker(self):
        ''' Function that checks if the robot is inside one of the predefined areas '''
        areas = []
        
        # Process all the point groups and check if the robot is inside the convexhull
        for group in self.pointGroups:
            if group.InConvexHull(self.pos[0:2]):
                areas.append(group.groupID)
        
        for area in areas:
            if area not in self.robotInAreas:
                self.get_logger().info("Robot entered area {}".format(area))
        
        for area in self.robotInAreas:
            if area not in areas:
                self.get_logger().info("Robot exited area {}".format(area))
        
        self.robotInAreas = areas
        
        '''
        area = self.interactiveMap[self.width - self.mapPos[0], self.height - self.mapPos[1]]

        if area != self.robotInArea:
            if self.robotInArea == 0:
                self.get_logger().info("Robot entered area {}".format(area))
            else:
                self.get_logger().info("Robot exited area {}".format(self.robotInArea))

            self.robotInArea = area
        '''
        pass

###################################################################################################
def main(args=None):
    rclpy.init(args=args)

    IMD = InteractiveMapDemo()

    try:
        rclpy.spin(IMD)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()