# Import ROS2 libraries
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# Import message files
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from autonomous_exploration_msgs.msg import MapData, PointGroups, PointGroup
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.srv import LoadMap

# Import other libraries
import numpy as np
import cv2 as cv
from scipy.spatial.transform import Rotation
import yaml

class LoadInteractiveMap(Node):
    """
        Save the interactive map as an image
    """

    def __init__(self):

        super().__init__("maps_publisher")

        # Create callback group
        ## Topic callback group
        self.top_callback_group = ReentrantCallbackGroup()
        ## Action callback group
        self.act_callback_group = ReentrantCallbackGroup()
        
        # Initialize the variables
        self.bridge = CvBridge()
        qos = QoSProfile(depth=10)
        self.pos = [0.0, 0.0, 0.0]
        self.mapOdomOffset = []
        self.map_pos = [0, 0]
        self.pointGroups = PointGroups()

        # Create subscribers
        # /odom
        self.create_subscription(Odometry, 'odom', self._odomCallback, qos, callback_group=self.top_callback_group)
        ## /tf
        self.create_subscription(TFMessage, 'tf', self._tfCallback, qos, callback_group=self.top_callback_group)

        # Create publishers
        ## /maps_publisher/interactive_map 
        self.interactiveMap_pub = self.create_publisher(MapData, "/maps_publisher/interactive_map", qos, callback_group=self.top_callback_group)
        ## /maps_publisher/map
        self.map_pub = self.create_publisher(MapData, '/maps_publisher/map', qos, callback_group=self.top_callback_group)
        ## /maps_publisher/interactive_map_color
        self.interactiveMapColor_pub = self.create_publisher(Image, "/maps_publisher/interactive_map_color", qos, callback_group=self.top_callback_group)
        ## /rosbridge_msgs_publisher/point_groups
        self.pointGroups_pub = self.create_publisher(PointGroups, "/rosbridge_msgs_publisher/point_groups", qos, callback_group=self.top_callback_group)

        # Create service clients
        ## /map_server/load_map
        self.loadMap_srv = self.create_client(LoadMap, '/map_server/load_map', callback_group=self.act_callback_group)
        if not self.loadMap_srv.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn('LoadMap service not available')

        # Load the maps from the .yaml files
        self.LoadMaps()

        self.get_logger().info("Interactive map loader was initiated successfully")

        # Create a timer to publish the interactive map
        self.create_timer(1.0, self.MapsPublisher)  # unit: s

    def _tfCallback(self, data:TFMessage):
        ''' Read the tf data and find the transformation between odom and map '''

        for tr in data.transforms:
            if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                if (len(self.mapOdomOffset) == 0):
                    self.get_logger().info("Maps loader/publisher was initiated")
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
        self.map_pos[0] = int((self.pos[0] - self.map_origin[0]) / self.resolution)
        self.map_pos[1] = int((self.pos[1] - self.map_origin[1]) / self.resolution)

    def LoadMaps(self):
        '''Load the maps from the yaml files'''
        
        # Load the regular map
        self.resolution = 0.0
        self.map_origin = []
        map = []
        with open(r'/home/antony/map.yaml') as file:
            documents = yaml.full_load(file)
            for item, doc in documents.items():
                if item == 'resolution':
                    self.resolution = float(doc)
                elif item == 'image':
                    map = cv.imread(doc, -1)

                    # Align the map and the interactive map
                    map = np.rot90(np.fliplr(map), 1)
                    map = np.flip(map, 0)
                    self.width, self.height = map.shape
                    self.map = -1 * np.ones_like(map)
                    self.map[map == 0] = 100
                    map = map.flatten()
                    self.map = self.map.flatten()   
                elif item == 'origin':
                    self.map_origin = [0.0] * 7
                    self.map_origin[0:3] = doc
        
        # Load the interactive map
        self.interactiveMap = cv.imread("/home/antony/interactive_map.pgm", -1)
        self.interactiveMap = self.interactiveMap.flatten()

        # Create the interactive colorful map
        self.interactiveMapColor = np.zeros((len(map), 3))
        colors = 255 * np.random.rand(max(self.interactiveMap), 1, 3)

        for i in range(len(map)):
            
            if map[i] > 253:
                self.map[i] = 100
                self.interactiveMapColor[i] = np.array([255, 255, 255])
                if self.interactiveMap[i] > 0:
                    self.interactiveMapColor[i] = colors[self.interactiveMap[i] - 1, :, :]
            elif map[i] != 0:
                self.interactiveMapColor[i] = np.array([128, 128, 128])

        # Reshape the map image to width * height * 3
        self.interactiveMapColor = np.reshape(self.interactiveMapColor, (self.width, self.height, 3))
        
        # Load the point groups
        with open(r'/home/antony/point_groups.yaml') as file:
            groups = yaml.full_load(file)
            for group in groups:
                pg = PointGroup()
                pg.group_id = group[0]['group_id']
                #tmp = []
                #for pos in group[0]['map_pos']:
                #    tmp.append(pos[0])
                #    tmp.append(pos[1])
                pg.map_pos = group[0]['map_pos']
                pg.map_origin = group[0]['map_origin']
                pg.map_dims = group[0]['map_dims']
                pg.map_resolution = group[0]['map_resol']
                pg.associated_file = group[0]['assoc_fl']

                self.pointGroups.groups.append(pg)
        
        # Call the load map service
        req = LoadMap.Request()
        req._map_url = "/home/antony/map.yaml"
        fut = self.loadMap_srv.call_async(req)
        '''
        if fut.done():
            try:
                response = fut.result()
                print(response)
            except Exception as e:
                self.get_logger().warn('Service call failed %r' % (e,))
            else:
                res = {0:"Success", 1:"Map does not exist", 2:"Invalid map data", 3:"Invalid map metadata", 255:"Undifined failute"}
                
                #self.get_logger().info("Service results : " + res[response])
        '''

    def MapsPublisher(self):
        ''' Publish the map, interactive map and interactive_map_color '''
        
        # Don't publish the map in case the initial pose is not published
        if (len(self.mapOdomOffset) == 0):
            return
            
        # Publish the regular map
        map = MapData()
        map.width = self.width
        map.height = self.height
        map.map = [int(tmp) for tmp in self.map.flatten()]
        map.resolution = self.resolution
        map.origin = np.array(self.map_origin, np.float32)
        self.map_pub.publish(map)

        # Draw the position of the robot it the map
        interactiveMapColor = self.interactiveMapColor.copy()
        interactiveMapColor[self.width - self.map_pos[0], self.height - self.map_pos[1]] = np.array([255, 0, 0])
        interactiveMapColor = interactiveMapColor.astype(np.uint8)
        
        # Publish the interactive map
        map.map = [int(tmp) for tmp in self.interactiveMap.flatten()]
        self.interactiveMap_pub.publish(map)

        # Publish the interactive map with colors
        self.interactiveMapColor_pub.publish(self.bridge.cv2_to_imgmsg(interactiveMapColor, "rgb8"))
        
        # Publish the point groups
        self.pointGroups_pub.publish(self.pointGroups)

###################################################################################################
def main(args=None):
    rclpy.init(args=args)

    LIM = LoadInteractiveMap()
    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(LIM, executor)
    except KeyboardInterrupt:
        pass
    #rclpy.spin_until_future_complete(SR, )
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    #SR.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()