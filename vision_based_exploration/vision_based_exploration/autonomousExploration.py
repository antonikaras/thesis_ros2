# Import ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.qos import QoSProfile

# Import message files
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid as OccG
from nav_msgs.msg import MapMetaData as MMD
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image
from vision_based_exploration_msgs.msg import ExplorationTargets, ExplorationTarget 
from vision_based_exploration_msgs.action import AutonomousExplorationAction

# Import other libraries
import numpy as np
from scipy.spatial.transform import Rotation
import sys
import os
import threading

class AutonomousExploration(Node):

    def __init__(self):

        super().__init__('autonomous_exploration')

        # Range of the lidar
        self.LidarRange = 3.5

        # Initialize the variables
        self.in_pos = []
        self.pos = [0.0, 0.0, 0.0]
        self.VFCandidates = []
        self.map = []
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_size = 0
        self.map_origin = np.array([0.0, 0.0])
        self.prev_targets = []
        self.goal_sent = 0
        self.remaining_distance = 0.0
        self.recovery_attempts = 0
        qos = QoSProfile(depth=10)

        # Setup rate
        self.rate = self.create_rate(2)

        # Create the navigation2 action client
        self.nav2_action_Client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav2_action_Client.wait_for_server()

        # Setup subscribers
        ## /vision_based_frontier_detection/exploration_candidates
        self.create_subscription(ExplorationTargets, '/vision_based_frontier_detection/exploration_candidates', self._explorationCandidatesVFCallback, qos)
        ## /odom
        self.create_subscription(Odometry, 'odom', self._odomCallback, qos)
        ## /map
        self.create_subscription(OccG, 'map', self._mapCallback, qos)

        # Create the action server
        self.auto_explore_action_server = ActionServer(self, AutonomousExplorationAction, 'autonomous_exploration', self._aEActionCallback)

        self.get_logger().info('Autonomous explorer was initiated successfully')

    def _mapCallback(self, data:OccG):
        self.get_logger().info('--*-- Map updated --*--')
        self.map = data.data
        self.map_width = data.info.width
        self.map_height = data.info.height
        self.map_resolution = data.info.resolution
        self.map_size = self.map_height * self.map_width    
        self.map_origin = np.array([data.info.origin.position.x, data.info.origin.position.y])    


    def _odomCallback(self, msg:Odometry):
        ''' Odometry function callback'''

        pos = msg.pose.pose.position

        self.pos[0:2] = [pos.x, pos.y]

        # Convert from quaternion to euler angles
        orient = msg.pose.pose.orientation
        quat_df = [orient.x, orient.y, orient.z, orient.w]
        rot = Rotation.from_quat(quat_df)
        rot_euler = rot.as_euler('xyz', degrees=True)
        self.pos[2] = rot_euler[2]
    
    def _explorationCandidatesVFCallback(self, data:ExplorationTargets):
        ''' Read the exploration candidates detected using computer vision '''

        self.VFCandidates[:] = []
        for dt in data.targets:
            pos = np.array(dt.pos)
            self.VFCandidates.append([np.array(dt.pos), 0.0, 0.0])

    def _navGoalResponseCallback(self, future:rclpy.Future):
        ''' 
            Callback to process the request send to the navigtion2 action server 
            goal_sent -> -1 goal wasn't accepted
                      ->  0 goal is being processed
                      ->  1 goal was accepted
        '''
        
        goal_handle = future.result()

        # Goal wasn't accepted
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.goal_sent = -1
            return

        # Goal was accepted
        self.goal_sent = 1
        #self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._navGoalResultCallback)

    def _navGoalResultCallback(self, future:rclpy.Future):
        result = future.result().result
        self.target_explored = True
        self.get_logger().info('Result: {0}'.format(result.result))

    def _navGoalFeedbackCallback(self, data):
        self.remaining_distance = data.feedback.distance_remaining
        self.recovery_attempts = data.feedback.number_of_recoveries 

    def _sendNavGoal(self, goal_pos:list) -> bool:
        ''' 
            Send target position to the navigation2 controller 
            @return values: True  -> goal was previously explored
                            False -> goal wasn't explored                        
        '''
        # Check if the goal pos has been previously explored
        xs = float("{:.3f}".format(goal_pos[0]))
        ys = float("{:.3f}".format(goal_pos[1]))

        if [xs, ys] in self.prev_targets:
            self.get_logger().info('Previously explored {}'.format([xs, ys]))
            return True
        else:
            self.prev_targets.append([xs, ys])

        goal_msg = NavigateToPose.Goal()

        # Generate the target goal
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()

        # Position part
        goal.pose.position.x = float(goal_pos[0])
        goal.pose.position.y = float(goal_pos[1])

        # Orientation part
        rot = Rotation.from_euler('xyz', [0.0, 0.0, goal_pos[2]])
        quat = rot.as_quat()
        goal.pose.orientation.x = float(quat[0])
        goal.pose.orientation.y = float(quat[1])    
        goal.pose.orientation.z = float(quat[2])
        goal.pose.orientation.w = float(quat[3])

        goal_msg.pose = goal

        future = self.nav2_action_Client.send_goal_async(goal_msg)
        self.goal_sent = 0
        future.add_done_callback(self._navGoalResponseCallback)

        return False

    def GetArea(self, pos:np.array) -> float:
        ''' Compute the undiscovered area surrounding the target '''
        map = np.array(self.map).reshape((self.map_height, self.map_width)).T

        # Convert the robot position to map position
        i = int((pos[0] - self.map_origin[0]) / self.map_resolution)
        j = int((pos[1] - self.map_origin[1]) / self.map_resolution)

        step = int(self.LidarRange / self.map_resolution)

        areaOfInterest = map[j - step: j + step, i - step: i + step]
        #print(areaOfInterest)
        #print(type(areaOfInterest))
        area = float(np.count_nonzero(areaOfInterest == -1))
        areaNormalized = area / float(len(areaOfInterest) ** 2 + 0.0001)

        #print(areaNormalized)

        return areaNormalized
    
    def EvaluatePoint(self, pos:np.array) -> float:
        '''Compute the entropy in the given point'''
        # Get the distance from the robot to the target
        dist = np.linalg.norm(pos - self.pos[0:2])
        area = float(self.GetArea(pos))

        # Compute the undiscovered area surrounding the target
        if dist > 1:
            return area ** np.log(dist + 0.0001)
        else:
            return -area ** np.log(dist + 0.0001)

    def PickTargets(self):
        ''' Pick the most promising targets to explore'''

        # Copy the targets
        #self.rate.sleep()
        targets = np.array(self.VFCandidates).copy()

        if len(targets) == 0:
            return [float('inf')], 0
        scores = []
        poss = []
        for pos, _, _ in targets:
            val = self.EvaluatePoint(pos)
            xs = float("{:.3f}".format(pos[0]))
            ys = float("{:.3f}".format(pos[1]))
            #print(list(pos), self.prev_targets)
            if [xs, ys] not in self.prev_targets:
                poss.append(pos)
                scores.append(val)

        if len(scores) == 0:
            return [float('inf')], 0
        
        #self.get_logger().info('Target scores {}'.format(scores))

        return poss[np.argmax(scores)], max(scores)

    def _aEActionCallback(self, goal):

        self.get_logger().info('Autonomous explorer was called')

        # Proccess the goal inputs
        max_steps = goal.request.max_steps
        timeOut = goal.request.time_out
        method = goal.request.method

        self.get_logger().info("max_steps = {}, timeOut = {}, method = ".format(max_steps, timeOut) + method)         
        
        # Call the explore function
        succeeded = self.Explore(timeOut, max_steps, method, goal)

        # Update the status of goal
        if succeeded:
            goal.succeed()
        else:
            goal.failed()
        
        result = AutonomousExplorationAction.Result()
        result.succeeded = succeeded

        return result


    def Explore(self, timeout : float, maxSteps : int, method : str, goal) -> bool:
        ''' 
            Perform autonomous exploration using the vision based frontier detection method
            @timeout : Used to avoid the recovery mode of the robot when stuck in a position 
            @maxSteps : Maximum number of goals to explore
            @status : Return value, False -> robot stuck, True -> maxSteps reached
        '''
        cnt = 0
        finishedExploration = False
        stuck_cnt = 0
        previously_explored_cnt = 0

        # Create the feedback message
        feedback_msg = AutonomousExplorationAction.Feedback()

        while cnt < maxSteps:            
            # Check if the goal was cancelled
            #if goal.is_cancelling():
            #    self.get_logger().warn('Goal was cancelled')
            #    break
            self.get_logger().info('-----> Explore 4 <-----')
                      
            cur_target, cur_score = self.PickTargets()  
            self.get_logger().info('-----> Explore 5 <-----')

            if cur_target[0] == float('inf'):
                finishedExploration = True
                break
            
            self.get_logger().info('Trying next target {}, score {}'.format(cur_target, cur_score))
            previously_explored = self._sendNavGoal([cur_target[0], cur_target[1], 0.0])
            if previously_explored:
                previously_explored_cnt += 1
                if previously_explored_cnt > 5:
                    self.get_logger().warn('Manual control needed, exeeded number of previously explored point')
                    break
                self.rate.sleep()
            else:
                previously_explored_cnt = 0

            self.target_explored = False
            self.recovery_attempts = 0
            # Check if the next target is more promissiong than the current one            
            while (rclpy.ok() and not previously_explored):
                #self.get_logger().info('-----> Explore 9 <-----')    
                # Create the action feedback
                feedback_msg.goal = [float(cur_target[0]), float(cur_target[1]), 0.0]
                feedback_msg.pos = self.pos
                feedback_msg.goal_id = cnt
                feedback_msg.remaining_distance = self.remaining_distance
                goal.publish_feedback(feedback_msg)
                
                # Check if the robot entered recovery mode too many times 
                if self.recovery_attempts > 5:
                    self.get_logger().warn('Robot entered recovery mode too many times, cancelling goal')
                    self.nav2_action_Client._cancel_goal_async()
                    break

                next_target, next_score = self.PickTargets()
                cur_score = self.EvaluatePoint(cur_target)
                #self.get_logger().info('Cur_score {}, next_score {}'.format(cur_score, next_score))
                if cur_score > 0:
                    if next_score / cur_score > 2.5:
                        self.target_explored = True
                else:
                    self.target_explored = True

                if self.target_explored:
                    self.get_logger().info('Found better target {}, new score {}, currrent score {}'.format(next_target, next_score, cur_score))
                    self.nav2_action_Client._cancel_goal_async()
                    break
                
                #self.get_logger().info('-----> Explore 10 <-----')
                
                #self.rate.sleep()
                

            self.get_logger().info('-----> Explore 11 <-----')

            if self.target_explored:
                stuck_cnt = 0
            elif not previously_explored:
                stuck_cnt += 1
                self.get_logger().info('Target unreachable, {}'.format(stuck_cnt))
                if stuck_cnt > 5:
                    self.get_logger().warn('All candidates are unreachables manual control needed')
                    break
            else:
                stuck_cnt = 0
                self.get_logger().info("Reached target")
            
            self.get_logger().info('cnt {}'.format(cnt))
            
        # Return status
        status = finishedExploration
        if cnt == maxSteps:
            status = True
        
        return status
###################################################################################################
def main(args=None):
    rclpy.init(args=args)

    AE = AutonomousExploration()

    rclpy.spin(AE)
    #rclpy.spin_until_future_complete(SR, )
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    #SR.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        #while rclpy.ok():
        main()
    except KeyboardInterrupt:
        pass