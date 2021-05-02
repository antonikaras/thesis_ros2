# Python related libraries
import numpy as np
import random

# ROS2 msgs
from autonomous_exploration_msgs.msg import PointGroup

class PointsGroup:
    """ 
        Class similar to the one used in unity, created
        to handle each area in the interactive map
    """

    def __init__(self, pG : PointGroup) -> None:
        
        # Store the map related data
        self.mapOrigin = pG.map_origin
        self.mapDims = pG.map_dims
        self.mapResolution = pG.map_resolution

        # Store the group related data
        self.groupID = pG.group_id
        self.numOfPoints = int(0.5 * len(pG.map_pos))
        self.mapPos = np.zeros([self.numOfPoints, 2])
        
        for i in range(self.numOfPoints):
            self.mapPos[i, 0] = pG.map_pos[2 * i]
            self.mapPos[i, 1] = pG.map_pos[2 * i + 1]

        # Generate the convexHull
        # Check if there are enough points in the group before generating the convex hull
        self.convexHullPoints = []

        if self.numOfPoints > 3:
            self.GenerateConvexHull()

    def GenerateConvexHull(self) -> None:
        """ 
            Generate the convex hull using the points of the interactive area 
            Same code as the one used in the unity package
        """
        vertices = list(self.mapPos.copy())
        vertices = [list(tmp) for tmp in vertices]

        # Step 1: Find the vertex with the smallest x coordinate
        startPos = vertices[0]
        for vert in vertices:
            if ( vert[0] < startPos[0]):
                startPos = vert
        
        #print(startPos)
        #print(vertices)
        self.convexHullPoints.append(startPos)
        vertices.remove(startPos)

        # Step2 : Loop to generate the convex hull
        currPos = self.convexHullPoints[0]
        cnt = 0

        while True:
            
            # After 2 iterations we have to add the start position again so we can terminate the algorithm
            if (cnt == 2):
                vertices.append(self.convexHullPoints[0])
            
            # Check if there are no more points
            if (len(vertices) == 0):
                break

            # Pick the next point randomly
            nextPos = vertices[random.randint(0, len(vertices) - 1)]
            a = currPos
            b = nextPos

            # Check if there's a point to the left of ab, if so then it's the new b
            for vert in vertices:
                
                # Skip the point picked randomly
                if vert == nextPos:
                    continue
                    
                # Compare the point and the line
                # To the left = better point, so pick it as next point on the convex hull
                if self.CompareLinePoint(a, b, vert) > 0:
                    nextPos = vert
                    b = vert
            
            # Update the convexHull
            self.convexHullPoints.append(nextPos)
            currPos = nextPos
            
            # Check if we found again the first point of the convexhull
            if currPos == self.convexHullPoints[0]:
                del self.convexHullPoints[-1]
                break

            cnt += 1
                    
    @staticmethod
    def CompareLinePoint(a : np.array, b : np.array, c : np.array) -> int:
        """ Return the position of a point relative to a line """

        # Where is c in relation to a-b ?
        #  < 0 -> to the right
        #  = 0 -> on the line
        #  > 0 -> to the left
        relation = (a[0] - c[0]) * (b[1] - c[1]) - (b[0] - c[0]) * (a[1] - c[1])

        return relation
    
    def InConvexHull(self, c : list) -> bool:
        """ Check if the point is inside the convex hull """

        # Check if there are enough points to create the convex Hull
        if (self.numOfPoints < 3):
            return False

        inConvexHull = True

        # if the point is on the left of all the line segments 
        # of the convexHull then it's on the outside
        for i in range(len(self.convexHullPoints) - 1):
            a = self.convexHullPoints[i]
            b = self.convexHullPoints[i + 1]

            #print(a, b, c, self.groupID, self.CompareLinePoint(a, b, c))
            
            # Check if it's left or right of the line ab
            if (self.CompareLinePoint(a, b, c) > 0):
                inConvexHull = False
                break
        
        # Check for the last line segment
        a = self.convexHullPoints[-1]
        b = self.convexHullPoints[0]
        if (self.CompareLinePoint(a, b, c) > 0):
            inConvexHull = False

        return inConvexHull
