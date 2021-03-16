# Import ROS2 libraries
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# Import message files
from sensor_msgs.msg import Image
from autonomous_exploration_msgs.msg import MapData
from nav_msgs.msg import OccupancyGrid

# Import other libraries
import numpy as np
import cv2 as cv

class VisualizeInteractiveMap(Node):
    """
        Convert the map published from Unity to an image topic
    """

    def __init__(self):

        super().__init__("visualize_interactive_map")

        # Initialize the variables
        self.bridge = CvBridge()
        qos = QoSProfile(depth=10)

        # Create subscribers
        ## /rosbridge_msgs_unity/interactive_map
        self.create_subscription(MapData, "rosbridge_msgs_unity/interactive_map", self._mapCallback, qos)

        # Create publishers
        ## /interactive_map/image
        self.interactiveMap_Imagepub = self.create_publisher(Image, "/interactive_map/image", qos)
        ## /interactive_map/map
        self.interactiveMap_Mappub = self.create_publisher(OccupancyGrid, "/interactive_map/map", qos)

        self.get_logger().info("Interactive map to image converter initiated")

    def _mapCallback(self, data:MapData):

        # Store the map Info
        width = data.height
        height = data.width
        
        # Rearrange the data to be visible correctly on unity
        map = np.array(data.map).reshape(width, height)
        map = np.flip(map, 0)
        map = map.flatten()
        map_img = np.zeros((width * height, 3))

        # Generate the colors randomly
        colors = 255 * np.random.rand(max(map), 1, 3)

        for i in range(max(map)):
            map_img[map == (i + 1)] = colors[i, :, :]
        
        # Reshape the map image to width * height * 3
        map_img = np.reshape(map_img, (width, height, 3))
        #map_img = np.flip(map_img, 1)
        map_img = map_img.astype(np.uint8)

        # Create the interactive map
        intMap = OccupancyGrid()
        intMap.header.frame_id = 'map'
        intMap.data = [int(el) for el in map]
        intMap.info.resolution = data.resolution
        intMap.info.width = width
        intMap.info.height = height
        intMap.info.origin.position.x = float(data.origin[0])
        intMap.info.origin.position.y = float(data.origin[1])

        # Publish the image
        self.interactiveMap_Imagepub.publish(self.bridge.cv2_to_imgmsg(map_img, "rgb8"))

        # Publish the map
        self.interactiveMap_Mappub.publish(intMap)
###################################################################################################
def main(args=None):
    rclpy.init(args=args)

    VIM = VisualizeInteractiveMap()
    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(VIM)
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