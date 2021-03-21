# Import ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

# Import message files
from sensor_msgs.msg import Image
from autonomous_exploration_msgs.msg import MapData
from nav_msgs.msg import OccupancyGrid

# Import other libraries
import numpy as np
import cv2 as cv

class SaveInteractiveMap(Node):
    """
        Save the interactive map as an image
    """

    def __init__(self):

        super().__init__("visualize_interactive_map")

        # Initialize the variables
        qos = QoSProfile(depth=10)
        self.mapReceived = False
        self.interMapReceived = False
        self.mapsSaved = False

        # Create subscribers
        # /map
        self.create_subscription(OccupancyGrid, 'map', self._mapCallback, qos)
        ## /rosbridge_msgs_unity/interactive_map
        self.create_subscription(MapData, "rosbridge_msgs_unity/interactive_map", self._interactiveMapCallback, qos)

        # Create publishers
        ## /interactive_map/image
        self.interactiveMap_Imagepub = self.create_publisher(Image, "/interactive_map/image", qos)

        # Create a timer to store the interactive map
        self.create_timer(2.0, self.saveMaps)  # unit: s

        self.get_logger().info("Interactive map saver was initiated")
        
    def _mapCallback(self, data:OccupancyGrid):
        self.map = np.array(data.data)#.reshape(data.info.width, data.info.height)
        self.map_width = data.info.width
        self.map_height = data.info.height
        self.map_resolution = data.info.resolution
        
        self.mapReceived = True

    def _interactiveMapCallback(self, data:MapData):

        # Store the map Info
        width = data.height
        height = data.width
        
        # Rearrange the data to be visible correctly on unity
        map = np.array(data.map).reshape(width, height)
        map = np.rot90(np.fliplr(map), 1)
        map = np.flip(map, 0)
        map = map.flatten()
        
        self.interactiveMap = map
        self.interMapReceived = True
    
    def saveMaps(self):
        ''' Once both maps received store the interactive map as an image'''
        
        if not(self.mapReceived and self.interMapReceived and not self.mapsSaved):
            return

        map = np.array(self.map).copy()
        interactiveMap = np.array(self.interactiveMap).copy()

        if max(interactiveMap.flatten()) == 0:
            self.get_logger().warn("Interactive map doesn't contain interactive areas")
            return

        map_img = np.zeros((self.map_width * self.map_height, 3))

        # Generate the colors randomly
        colors = 255 * np.random.rand(max(interactiveMap.flatten()), 1, 3)

        for i in range(len(interactiveMap)):
            if map[i] == -1:
                map_img[i] = np.array([128, 128, 128])
            elif map[i] < 51:
                map_img[i] = np.array([255, 255, 255])
                if interactiveMap[i] > 0:
                    map_img[i] = colors[interactiveMap[i] - 1, :, :]
        
        # Reshape and rearrange the map image to width * height * 3
        map_img = np.reshape(map_img, (self.map_height, self.map_width, 3))
        map_img = np.rot90(np.fliplr(map_img), -1)
        interactiveMap = np.reshape(interactiveMap, (self.map_height, self.map_width, 1))
        interactiveMap = np.rot90(np.fliplr(interactiveMap), -1)
        map_img = map_img.astype(np.uint8)
        interactiveMap = interactiveMap.astype(np.uint8)
        
        # Save the maps
        cv.imwrite('/home/antony/interactive_map.pgm', interactiveMap)
        cv.imwrite('/home/antony/interactive_map_color.jpg', map_img)
        self.mapsSaved = True

        self.get_logger().info('Maps Saved')

        
###################################################################################################
def main(args=None):
    rclpy.init(args=args)

    SIM = SaveInteractiveMap()

    try:
        rclpy.spin(SIM)
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