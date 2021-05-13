# Import ROS2 libraries
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.qos import QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# Import message files
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid as OccG
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from tf2_msgs.msg import TFMessage
from autonomous_exploration_msgs.msg import ExplorationTargets, ExplorationTarget, PosData
from autonomous_exploration_msgs.action import AutonomousExplorationAction

# Import other libraries
import numpy as np
import time

class AutonomousExploration(Node):

    def __init__(self):

        super().__init__('autonomous_exploration')

        # Range of the lidar
        self.LidarRange = 3.5

        # Create callback group
        ## Topic callback group
        self.top_callback_group = ReentrantCallbackGroup()
        ## Action callback group
        self.act_callback_group = ReentrantCallbackGroup()
        ## Timer callback group
        self.timer_callback_group = ReentrantCallbackGroup()
        ## Publisher callback group
        self.pub_callback_group = ReentrantCallbackGroup()

        # Initialize the variables
        self.VFCandidates_near = []
        self.VFCandidates_center = []
        self.VFCandidates_far = []
        self.exploredTargets = []
        self.mapOdomOffset = []
        self.pos = np.array([0.0, 0.0])
        self.curTar = []
        self.curTarScore = 0.0
        self.newTarScore = 0.0
        self.newTar = []
        self.goal_sent = 0
        self.remaining_distance = 0.0
        self.recovery_attempts = 0
        self.stopThread = False
        self.mapUpdated = False
        qos = QoSProfile(depth=10)
        
        # Setup rate
        self.rate = self.create_rate(2)

        # Create the navigation2 action client                        
        self.nav2_action_Client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav2_action_Client.wait_for_server()

        # Setup subscribers
        ## /vision_based_frontier_detection/exploration_candidates
        self.create_subscription(ExplorationTargets, '/vision_based_frontier_detection/exploration_candidates', self._explorationCandidatesVFCallback, qos, callback_group=self.top_callback_group)
        ## /odom
        self.create_subscription(Odometry, 'odom', self._odomCallback, qos, callback_group=self.top_callback_group)
        ## /tf
        self.create_subscription(TFMessage, 'tf', self._tfCallback, qos, callback_group=self.top_callback_group)
        ## /map
        self.create_subscription(OccG, 'map', self._mapCallback, qos)
        
        # Setup publishers
        ## /rosbridge_msgs_publisher/current_target 
        self.curTar_pub = self.create_publisher(PosData, '/rosbridge_msgs_publisher/current_target', qos, callback_group=self.pub_callback_group)

        # Create the action server
        self.auto_explore_action_server = ActionServer(self, AutonomousExplorationAction, 'autonomous_exploration', 
                                                        execute_callback = self._aeActionCallback, 
                                                        callback_group=self.act_callback_group, 
                                                        cancel_callback = self._aeCancelCallback)
        #                                                goal_callback = self._aeGoalCallback,                      

        # Publish the exploration target
        self.create_timer(1.0, self.pubExplorationTarget, callback_group=self.timer_callback_group)  # unit: s

        self.get_logger().info('Autonomous explorer was initiated successfully')

    def pubExplorationTarget(self) -> None:
        curTar = PosData()

        if len(self.curTar) > 0:
            curTar.x = float(self.curTar[0])
            curTar.y = float(self.curTar[1])
            curTar.yaw = float(1)
        else:
            curTar.yaw = float(-1)

        self.curTar_pub.publish(curTar)

    def _mapCallback(self, data:OccG):
        
        if len(self.curTar) == 0:
            return
        # Convert the current target to map coordinates
        x = int((self.curTar[1] - data.info.origin.position.x) / data.info.resolution)
        y = int((self.curTar[0] - data.info.origin.position.y) / data.info.resolution) 
        
        # Convert the map from 1D to 2D
        map = np.array(data.data).reshape((data.info.height, data.info.width)).T

        # Compute the undiscovered area surrounding the target
        step = int(self.LidarRange / data.info.resolution)

        areaOfInterest = map[x - step: x + step, y - step: y + step].copy()
        area = float(np.count_nonzero((areaOfInterest == -1)))
        w, h = areaOfInterest.shape
        areaNormalized = area / float(w * h + 0.0001)
        dist = np.linalg.norm(self.curTar - self.pos) 

        self.curTarScore = self.EvaluatePoint([dist, areaNormalized])
            
        self.mapUpdated = True

    def _tfCallback(self, data:TFMessage):
        ''' Read the tf data and find the transformation between odom and map '''

        for tr in data.transforms:
            if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                if (len(self.mapOdomOffset) == 0):
                    self.mapOdomOffset = [0.0] * 2
                self.mapOdomOffset[0] = tr.transform.translation.x
                self.mapOdomOffset[1] = tr.transform.translation.y

    def _odomCallback(self, msg:Odometry):
        
        # Don't publish the map in case the initial pose is not published
        if (len(self.mapOdomOffset) == 0):
            return
        
        pos = msg.pose.pose.position

        self.pos[0:2] = [pos.x + self.mapOdomOffset[0], pos.y + self.mapOdomOffset[1]]   

    def _explorationCandidatesVFCallback(self, data:ExplorationTargets):
        ''' Read the exploration candidates detected using computer vision '''        
        self.VFCandidates_near[:] = []
        self.VFCandidates_center[:] = []
        self.VFCandidates_far[:] = []
        for dt in data.targets:
            self.VFCandidates_near.append(dt.cluster_point_near)
            self.VFCandidates_center.append(dt.cluster_point_center)
            self.VFCandidates_far.append(dt.cluster_point_far)
        
        # Find the best new target
        self.newTar, self.newTarScore = self.PickTargetMaxExpEntr()

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

        self.nav_goal_handle = goal_handle

        # Goal was accepted
        self.goal_sent = 1
        #self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._navGoalResultCallback)

    def _navGoalResultCallback(self, future:rclpy.Future):
        result = future.result().result
        
    def _navGoalFeedbackCallback(self, data):
        self.remaining_distance = data.feedback.distance_remaining
        self.recovery_attempts = data.feedback.number_of_recoveries 

    def _sendNavGoal(self, goal_pos:float):
        ''' 
            Send target position to the navigation2 controller                   
        '''
        # Check if the goal pos has been previously explored
        xs = float("{:.3f}".format(goal_pos[0]))
        ys = float("{:.3f}".format(goal_pos[1]))

        goal_msg = NavigateToPose.Goal()

        # Generate the target goal
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()

        # Position part
        goal.pose.position.x = float(goal_pos[0])
        goal.pose.position.y = float(goal_pos[1])

        goal_msg.pose = goal
        #self.nav2_action_Client.send_goal(goal_msg)
        future = self.nav2_action_Client.send_goal_async(goal_msg)
        future.add_done_callback(self._navGoalResponseCallback)

    def _aeGoalCallback(self, req):
        pass

    def _aeCancelCallback(self, req):
        self.get_logger().info('Autonomous exploration received cancel request')
        self.nav_goal_handle.cancel_goal_async()
        return CancelResponse.ACCEPT

    async def _aeActionCallback(self, goal):

        self.get_logger().info('Autonomous explorer was called')

        # Proccess the goal inputs
        maxSteps = goal.request.max_steps
        timeOut = goal.request.time_out
        method = goal.request.method

        self.get_logger().info("max_steps = {}, timeOut = {}, method = ".format(maxSteps, timeOut) + method)         
        
        # Call the explore function
        succeeded = self.Explore(timeOut, maxSteps, method, goal)
    
        # Update the status of goal
        if succeeded:
            self.get_logger().info("Autonomous exploration succeeded")
            goal.succeed()
        else:
            self.get_logger().warn("Autonomous exploration failed")
            goal.abort()
        
        result = AutonomousExplorationAction.Result()
        result.succeeded = succeeded

        return result
    
    def IsExplored(self, tar:np.array) -> bool:
        ''' Check if the target is already explored'''
        #self.get_logger().info("Target IsExplored--> {}".format(tar))
        already_explored = False    
        for el in self.exploredTargets:
            #self.get_logger().info("--------------> IsExplored Dist{}".format(np.linalg.norm(el - tar)))
            #self.get_logger().info("TmpTar {}, cur Tar {}".format(el, tar))
            if np.linalg.norm(el - tar) < 0.1:
                already_explored = True
                break
        return already_explored

    def PickTargetNear(self) -> float:
        ''' Pick the closest exploration target '''
        # TODO add attempts if the exploration target is not found 
        # Sort the targets in increasing order
        dists = [dt[2] for dt in self.VFCandidates_near]
        indexes = [*range(len(dists))]
        sorted_indexes = [x for _,x in sorted(zip(dists, indexes))]

        # Check if the current target has already been explored
        tar = []
        for index in sorted_indexes:
            tmp_tar = np.array([self.VFCandidates_near[index][0], self.VFCandidates_near[index][1]])
            
            if not self.IsExplored(tmp_tar):
                tar = np.array(tmp_tar)
                break
        
        self.get_logger().info("Closest target selected {}".format(tar))

        return tar

    def PickTargetFar(self) -> float:
        ''' Pick the furthest exploration target '''
        # TODO add attempts if the exploration target is not found 
        # Sort the targets in increasing order
        dists = [dt[2] for dt in self.VFCandidates_far]
        indexes = [*range(len(dists))]
        sorted_indexes = [x for _,x in sorted(zip(dists, indexes), reverse=True)]

        # Check if the current target has already been explored
        tar = []
        for index in sorted_indexes:
            tmp_tar = np.array([self.VFCandidates_far[index][0], self.VFCandidates_far[index][1]])

            if not self.IsExplored(tmp_tar):
                tar = np.array(tmp_tar)
                break
        
        self.get_logger().info("Furthest target selected {}".format(tar))

        return tar
    
    def EvaluatePoint(self, pt:float) -> float:
        '''Compute the entropy in the given point'''
        # Get the distance from the robot to the target
        dist = pt[0]
        area = pt[1]

        # Compute the undiscovered area surrounding the target
        if dist > 1:
            return area ** np.log(dist + 0.0001)
        else:
            return -area ** np.log(dist + 0.0001)

    def PickTargetMaxExpEntr(self):
        ''' Use a cost function to estimate the best goal '''
        if len(self.VFCandidates_center) == 0:
            return [], 0
        if len(self.VFCandidates_center) != len(self.VFCandidates_far):
            self.get_logger().warn("Center {}, far {}".format(len(self.VFCandidates_center), len(self.VFCandidates_far)))

        # Create a copy of the targets
        targetCriteria = np.array(self.VFCandidates_center).copy()
        targets = np.array(self.VFCandidates_far).copy()
        scores = []
        poss = []
            
        # Compute the score for each of the frontier goals
        for cnt in range(len(targets)):
            #self.get_logger().info("center {}, far {}".format(targetCriteria.shape, targets.shape))
            dist = targetCriteria[cnt][2]
            area = targetCriteria[cnt][3]
            tar = np.array([targets[cnt][0], targets[cnt][1]])
            #self.get_logger().info("Target --> {}".format(tar))
            if not self.IsExplored(tar):
                scores.append(self.EvaluatePoint([dist, area]))
                poss.append(np.array([tar[0], tar[1]]))
        
        # Return the most promissing candidate
        if len(scores) == 0:
            return [], 0
        
        return poss[np.argmax(scores)], max(scores)

    def Explore(self, timeOut : float, maxSteps : int, method : str, goal) -> bool:
        ''' 
            Perform autonomous exploration and travel to the nearest frontier point
            @timeout : Used to avoid the recovery mode of the robot when stuck in a position 
            @maxSteps : Maximum number of goals to explore
            @method : Exploration target selection method-> near, far
            @status : Return value, False -> robot stuck, True -> maxSteps reached
        '''

        # Initialize loop variables
        cnt = 0
        finishedExploration = False
        feedback_msg = AutonomousExplorationAction.Feedback()
        stuck_cnt = 0
        while cnt < maxSteps:
            # Check if there aren't more exploration targets
            if len(self.VFCandidates_center) == 0:
                self.get_logger().info('No more exploration candidates found')
                finishedExploration = True
                break

            # Get the exploration target
            tar = []
            if method == "near":
                tar = self.PickTargetNear()
            elif method == "far":
                tar = self.PickTargetFar()
            else:
                tar, self.curTarScore = self.PickTargetMaxExpEntr()
                #self.get_logger().info("{}, {}".format(tar, score))
            
            # if there is a target sent it to the navigation controller
            if len(tar) > 0:
                self.get_logger().info("Navigating to {}".format(tar))
                self.mapUpdated = False
                self.curTar = np.array(tar)
                self.exploredTargets.append(np.array(tar))
                self._sendNavGoal(tar)
                
                ts = time.time()
                ts2 = time.time()
                pos_bef = np.array([self.pos[0], self.pos[1]])

                # -1:Cancel navigation, 0:continue, 1:navigation succedded 2:found better goal
                navigation_status = 0

                # Wait until timeout or goal reached and publish feedback
                while (rclpy.ok()) and (navigation_status == 0):
                    # Create the action feedback
                    feedback_msg.goal = [float(tar[0]), float(tar[1]), 0.0]
                    feedback_msg.pos = [float(self.pos[0]), float(self.pos[1])]
                    feedback_msg.goal_id = cnt
                    feedback_msg.remaining_distance = self.remaining_distance
                    goal.publish_feedback(feedback_msg)

                    if np.linalg.norm(self.pos - self.curTar) < 0.4:
                        self.get_logger().info("Reached goal, pos {}".format(self.pos))
                        navigation_status = 1   
                        break

                    # Check if the robot stuck in the same position for too long
                    pos_now = np.array([self.pos[0], self.pos[1]])
                    if time.time() - ts > timeOut:
                        ts = time.time() 
                        if np.linalg.norm(pos_bef - pos_now) < 0.1:
                            self.get_logger().warn("Robot didn't move while navigating to the next waypoint")
                            navigation_status = -1
                            break
                        pos_bef = pos_now.copy()

                    # Check if it took too long to go to the goal
                    if (time.time() - ts2 > 60.0):
                        self.get_logger().warn("Timeout reached, too much time spent travelling to goal")
                        navigation_status = -1
                        break

                    # Check if a better goal exist, only for the Entopy method
                    if method != 'near' and method != 'far':
                        if self.curTarScore > 0.01:
                            if self.newTarScore / self.curTarScore > 2.0:
                                self.get_logger().info("Found better navigation target {}".format(self.newTar))
                                navigation_status = 2
                                self.curTar = []
                                break
                cnt += 1
            else:
                self.get_logger().warn("No more " + method + " targets, exploration stopping")
                finishedExploration = False
                self.curTar = []
                break

            # Check if the robot reached the goal
            if navigation_status > 0:
                stuck_cnt = 0
                self.curTar = []
            else:
                stuck_cnt += 1
                if stuck_cnt == 4:
                    self.get_logger().error("Robot stuck too many times, manual control needed")
                    self.curTar = []
                    break

        # Return status
        status = finishedExploration
        if cnt == maxSteps:
            status = True
        
        return status
###################################################################################################
def main(args=None):
    rclpy.init(args=args)

    AE = AutonomousExploration()
    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(AE, executor)
    except KeyboardInterrupt:
        pass    #rclpy.spin_until_future_complete(SR, )
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    AE.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()