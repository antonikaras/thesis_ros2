# Import ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from cv_bridge import CvBridge, CvBridgeError

# Import message files
from nav_msgs.msg import OccupancyGrid as OccG
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Image
from autonomous_exploration_msgs.msg import ExplorationTargets, ExplorationTarget 

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
        self.pos = np.array([0.0, 0.0])
        self.map = []
        self.mapOdomOffset = []
        self.map_width = 0
        self.map_height = 0
        self.mapResolution = 0
        self.map_size = 0
        self.mapOrigin = np.array([0.0, 0.0])
        self.seq = 0
        self.bridge = CvBridge()
        qos = QoSProfile(depth=10)

        # Create subscribers
        ## /map
        self.create_subscription(OccG, 'map', self._mapCallback, qos)
        ## /odom
        self.create_subscription(Odometry, 'odom', self._odomCallback, qos)
        ## /tf
        self.create_subscription(TFMessage, 'tf', self._tfCallback, qos)


        # Create publishers
        ## /vision_based_frontier_detection/exploration_candidates
        self.explorationCandidates_pub = self.create_publisher(ExplorationTargets, '/vision_based_frontier_detection/exploration_candidates', qos)
        ## /vision_based_frontier_detection/exploration_candidates
        self.map_image_pub = self.create_publisher(Image, '/vision_based_frontier_detection/map_image_pub', qos)
        self.map_image_pub_ = self.create_publisher(Image, '/vision_based_frontier_detection/map_image_pub_', qos)
        self.map_image_pub__ = self.create_publisher(Image, '/vision_based_frontier_detection/map_image_pub__', qos)
    
    def _tfCallback(self, data:TFMessage):
        ''' Read the tf data and find the transformation between odom and map '''

        for tr in data.transforms:
            if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                if (len(self.mapOdomOffset) == 0):
                    self.get_logger().info("Vision based frontier detector has started")
                    self.mapOdomOffset = [0.0] * 2
                self.mapOdomOffset[0] = tr.transform.translation.x
                self.mapOdomOffset[1] = tr.transform.translation.y

    def _odomCallback(self, msg:Odometry):
        
        # Don't publish the map in case the initial pose is not published
        if (len(self.mapOdomOffset) == 0):
            return
        
        pos = msg.pose.pose.position

        self.pos[0:2] = [pos.x + self.mapOdomOffset[0], pos.y + self.mapOdomOffset[1]]
        
    def _mapCallback(self, data:OccG):
        self.map_width = data.info.width
        self.map_height = data.info.height
        self.mapResolution = data.info.resolution
        self.map_size = self.map_height * self.map_width    
        self.mapOrigin = np.array([data.info.origin.position.x, data.info.origin.position.y])

        # Convert the map from 1D to 2D
        self.map = np.array(data.data).reshape((self.map_height, self.map_width)).T
            
        # Generate new targets
        targets = self.DetectFrontiers()
        #self.ConvertMap()
        msg = ExplorationTargets()
        #msg.header.seq = self.seq
        #msg.header.stamp = rospy.Time.now()
        msg.targets = targets
        self.explorationCandidates_pub.publish(msg)

        self.seq += 1

    def GetArea(self, pos):
        ''' Compute the undiscovered area surrounding the target '''

        step = int(self.LidarRange / self.mapResolution)
        
        # Crop the map around the area of interest
        xs = pos[1] - step
        if xs < 0:
            xs = 0
        xf = pos[1] + step
        if xf > self.map_width:
            xf = self.map_width
        
        ys = pos[0] - step
        if ys < 0:
            ys = 0
        yf = pos[0] + step
        if yf > self.map_height:
            yf = self.map_height
        areaOfInterest = self.map[xs:xf, ys: yf].copy()
        #area = 0.7 * float(np.count_nonzero((areaOfInterest == -1))) + 0.3 * float(np.count_nonzero((areaOfInterest < 51)))
        area = float(np.count_nonzero((areaOfInterest == -1)))
        w, h = areaOfInterest.shape
        areaNormalized = area / float(w * h + 0.0001)
        #self.get_logger().info("area {}, area_normalized {}, map shape {}".format(area, areaNormalized, self.map.shape))
        #self.get_logger().info("pos {}, shape {}, step {}".format(pos, areaOfInterest.shape, step))
        
        return areaNormalized

    def FilterClusterPoints(self, cluster:list) -> list:
        """ 
            Compute the number of neighbors for each point within the cluster and
            retain only the points that have more than 50% unexplored neighbours 
            Returns the x, y position of the point, the distance to the robot and 
            the neighbour  coverage
        """

        filteredPoints = []

        #  Compute the avg unexplored area surrounding each point in the cluster
        avg_area = np.mean([self.GetArea(pt[0]) for pt in cluster])
        #self.get_logger().info("avg_area : {}".format(avg_area))

        for pt in cluster:
            pt = np.array(pt[0])
            area = self.GetArea(pt)

            if area > avg_area:
                # Convert the point from map space to world
                x = pt[1] * self.mapResolution + self.mapOrigin[0]
                y = pt[0] * self.mapResolution + self.mapOrigin[1]

                # Get the distance between the point and the robot
                dist = np.linalg.norm(np.array([x, y]) - self.pos)

                # Add the point info in the list
                filteredPoints.append([x, y, dist, area])
        
        return filteredPoints
    
    def ComputeClusterSpecs(self, filteredPoints : list) -> ExplorationTarget:
        ''' 
            Compute the center of the filtered points,the    
            closest and furthest cluster point to the robot
            return: [closest,center, furthest] 
        '''
        explorationTarget = ExplorationTarget()

        # Detect the point furthest from the robot
        tmp = np.argmax([el[2] for el in filteredPoints])
        explorationTarget.cluster_point_far = filteredPoints[tmp]

        # Compute the center of the cluster, the unexplored area 
        # surrounding it and the avg distance to the robot
        x_avg = 0.0
        y_avg = 0.0
        area = 0.0
        dist = 0.0
        for pt in filteredPoints:
            x_avg += pt[0]
            y_avg += pt[1]
            dist += pt[2]
            area += pt[3]
        x_avg /= len(filteredPoints)
        y_avg /= len(filteredPoints)
        dist /= len(filteredPoints)
        area /= len(filteredPoints)
        explorationTarget.cluster_point_center = [x_avg, y_avg, dist, area]

        # Detect the point closest to the robot
        tmp = np.argmin([el[2] for el in filteredPoints])
        explorationTarget.cluster_point_near = filteredPoints[tmp]

        return explorationTarget

    def DetectFrontiers(self) -> list:
        ''' 
            Convert the 1D map to a 2D image and use computer 
            vision algorithms to detect the frontiers
        '''
        # -1:unknown, 0:free, 100:obstacle 
        
        # Detect the obstacles
        map = self.map.copy()
        obstacles = np.zeros_like(map)
        obstacles[(map >=65)] = 255
        obstacles = obstacles.astype(np.uint8)
        obstacles = cv.dilate(obstacles, np.ones((3, 3), np.uint8))        
        #obstacles = cv.erode(obstacles, np.ones((3, 3), np.uint8))
        
        # Detect the area already explored
        explored = np.zeros_like(map)
        explored[(0 <= map) & (map <= 25)] = 255
        ## In the beginning the upper threshold should be higher
        if np.count_nonzero(((0 <= map) & (map <= 25))) < 100:
            explored[(0 <= map) & (map <= 55)] = 255
            
        explored = explored.astype(np.uint8)
        explored = cv.dilate(explored, np.ones((3, 3), np.uint8))
        #explored = cv.erode(explored, np.ones((3, 3), np.uint8))

        # Detect the edges, areas between known and unknown environment
        map = map.astype(np.uint8)
        edges = cv.Canny(explored, 40, 120)
        edges[edges != 0] = 255
        #edges = cv.dilate(edges, np.ones((3, 3), np.uint8))

        # Detect the frontiers by substracting the obstacles from the edges image
        frontiers = np.bitwise_and(np.bitwise_not(obstacles), edges)
        #frontiers = cv.erode(frontiers, np.ones((3, 3), np.uint8))
        frontiers[frontiers != 0] = 255
        #frontiers = cv.erode(frontiers, np.ones((3, 3), np.uint8))

        # Detect the contours in the image
        contours, _ = cv.findContours(frontiers, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        map_img = cv.UMat(np.zeros_like(map))

        clSpecs = []
        # Process each contour cluster seperately
        for contour in contours:
            area = cv.contourArea(contour)
            _, _,w,h = cv.boundingRect(contour)
            area2 = w * h
            #self.get_logger().info("contour length {}, area {}, area2 {}".format(len(contour), area, area2))
            #if len(contour) > 2:                
            if area2 > 100:
                # Retain only the points that have more than 50% unexplored neighbours
                filteredPoints = self.FilterClusterPoints(contour)
                #self.get_logger().info("area2 {}, filtered points {}".format(area2, len(filteredPoints)))
                if len(filteredPoints) > 0:
                    # Find the closest, furthest and center point of the cluster
                    clSpecs.append(self.ComputeClusterSpecs(filteredPoints))

                    # Draw the contour in the image
                    rgb = np.random.randint(25)
                    cv.drawContours(map_img, [contour], -1, (100 + 6 * rgb), 2)

        map_img = cv.UMat.get(map_img)

        # Rotate the image for visualization ease
        map_img = np.rot90(np.fliplr(map_img), 2)
        map_img = np.flip(map_img, 1)
        self.map_image_pub.publish(self.bridge.cv2_to_imgmsg(map_img, 'mono8'))

        obstacles = np.rot90(np.fliplr(obstacles), 2)
        obstacles = np.flip(obstacles, 1)
        self.map_image_pub_.publish(self.bridge.cv2_to_imgmsg(obstacles, 'mono8'))
        
        explored = np.rot90(np.fliplr(explored), 2)
        explored = np.flip(explored, 1)
        self.map_image_pub__.publish(self.bridge.cv2_to_imgmsg(explored, 'mono8'))

        return clSpecs 
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