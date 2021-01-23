# Import ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from cv_bridge import CvBridge, CvBridgeError

# Import message files
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid as OccG
from nav_msgs.msg import MapMetaData as MMD
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image
from vision_based_exploration_msgs.msg import ExplorationTargets, ExplorationTarget 

# Import other libraries
import numpy as np
from scipy.spatial.transform import Rotation
import sys
import os
import threading
import cv2 as cv

class FrontierDetectionVision(Node):

    def __init__(self):
        super().__init__('vision_based_frontier_detector')

        # Range of the lidar
        self.LidarRange = 3.5

        # Setup variables
        self.map = []
        self.in_pos = []
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_size = 0
        self.map_origin = np.array([0.0, 0.0])
        self.seq = 0
        self.bridge = CvBridge()
        qos = QoSProfile(depth=10)

        # Create subscribers
        # /map
        self.create_subscription(OccG, 'map', self._mapCallback, qos)

        # Create publishers
        # /vision_based_frontier_detection/exploration_candidates
        self.explorationCandidates_pub = self.create_publisher(ExplorationTargets, '/vision_based_frontier_detection/exploration_candidates', qos)
        # /vision_based_frontier_detection/exploration_candidates
        self.map_image_pub = self.create_publisher(Image, '/vision_based_frontier_detection/map_image_pub', qos)
        

        self.get_logger().info("Vision based frontier detector has started")

    def _mapCallback(self, data):
        self.map = data.data
        self.map_width = data.info.width
        self.map_height = data.info.height
        self.map_resolution = data.info.resolution
        self.map_size = self.map_height * self.map_width    
        self.map_origin = np.array([data.info.origin.position.x, data.info.origin.position.y])    

        # Generate new targets
        targets = self.DetectFrontiers()
        msg = ExplorationTargets()
        #msg.header.seq = self.seq
        #msg.header.stamp = rospy.Time.now()
        msg.targets = targets
        self.explorationCandidates_pub.publish(msg)

        self.seq += 1
    
    def GetArea(self, pos):
        ''' Compute the undiscovered area surrounding the target '''
        map = np.array(self.map).reshape((self.map_height, self.map_width)).T

        # Convert the robot position to map position
        i = pos[1]
        j = pos[0]

        step = int(self.LidarRange / self.map_resolution)

        areaOfInterest = map[j - step: j + step, i - step: i + step]
        #print(areaOfInterest)
        #print(type(areaOfInterest))
        area = float(np.count_nonzero(areaOfInterest == -1))
        areaNormalized = area / float(len(areaOfInterest) ** 2)


        return areaNormalized

    def pt_img2base(self, map, points):
        ''' 
            Convert the location of the target frontier from 
            the image location to x,y relative to the base
            Choose a target that lies in the known region
        '''
        pos = []

        for pt in points:
            pt = pt[0]
            if self.GetArea(pt) > 0.5:
                pos = pt
                break
        
        if len(pt) == 0:
            return [float('inf'), float('inf')] 

        x = pt[1] * self.map_resolution + self.map_origin[0]
        y = pt[0] * self.map_resolution + self.map_origin[1]

        return [x, y]

    def DetectFrontiers(self):
        ''' 
            Convert the 1D map to a 2D image and use computer 
            vision algorithms to detect the frontiers
        '''
        # Convert the 2D map to image
        # -1:unknown, 0:free, 100:obstacle 
        map = np.array(self.map).reshape((self.map_height, self.map_width)).T

        # Detect the obstacles
        obstacles = np.zeros_like(map)
        obstacles[map == 100] = 255
        obstacles = obstacles.astype(np.uint8)
        obstacles = cv.dilate(obstacles, np.ones((3, 3), np.uint8))        

        # Detect the edges, areas between known and unknown environment
        map = map.astype(np.uint8)
        edges = cv.Canny(map, 40, 120)
        edges[edges != 0] = 255

        # Detect the frontiers by substracting the obstacles from the edges image
        frontiers = np.bitwise_and(np.bitwise_not(obstacles), edges)
        #frontiers = cv.erode(frontiers, np.ones((3, 3), np.uint8))
        frontiers[frontiers != 0] = 255

        # Detect the contours in the image
        contours, _ = cv.findContours(frontiers, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        map_img = cv.UMat(np.zeros_like(map))

        clSpecs = []
        for i in range(len(contours)):
            #print(len(contours[i]))
            rgb = np.random.randint(25)
            if len(contours[i]) > 2:
                pos = self.pt_img2base(map, contours[i])
                
                if pos[0] != float('inf'):
                    clSpec = ExplorationTarget()
                    clSpec.pos = pos
                    cv.drawContours(map_img, contours, i, (100 + 6 * rgb), 2)
                    clSpecs.append(clSpec)

        map_img = cv.UMat.get(map_img)

        self.map_image_pub.publish(self.bridge.cv2_to_imgmsg(map_img, 'mono8'))

        return clSpecs

        #return None
###################################################################################################
def main(args=None):
    rclpy.init(args=args)

    FDV = FrontierDetectionVision()

    rclpy.spin(FDV)
    #rclpy.spin_until_future_complete(SR, )
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    #SR.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        while rclpy.ok():
            main()
    except KeyboardInterrupt:
        pass
        '''
        print('Interrupted')
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)
        '''