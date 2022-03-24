#!/usr/bin/env python
import rospy
import numpy as np
import time
from geometry_msgs.msg import *
from nav_msgs.msg import *
from sensor_msgs.msg import *
from const import *
from math import *
from Queue import PriorityQueue
import copy
import argparse



ROBOT_SIZE = 0.2552  # width and height of robot in terms of stage unit


class Planner:
    def __init__(self, world_width, world_height, world_resolution, inflation_ratio=3):
        """init function of the base planner. You should develop your own planner
        using this class as a base.

        For standard mazes, width = 200, height = 200, resolution = 0.05. 
        For COM1 map, width = 2500, height = 983, resolution = 0.02

        Arguments:
            world_width {int} -- width of map in terms of pixels
            world_height {int} -- height of map in terms of pixels
            world_resolution {float} -- resolution of map

        Keyword Arguments:
            inflation_ratio {int} -- [description] (default: {3})
        """
        rospy.init_node('planner')
        self.map = None
        self.pose = None
        self.goal = None
        self.path = None
        self.action_seq = None  # output
        self.aug_map = None  # occupancy grid with inflation
        self.action_table = {}
        
        
        self.neighbors = {}



        self.world_width = world_width
        self.world_height = world_height
        self.resolution = world_resolution
        self.grid_resolution = int(1/world_resolution)
        self.inflation_ratio = inflation_ratio
        self.height_grid = int(self.world_height/self.grid_resolution)
        self.width_grid = int(self.world_width/self.grid_resolution)
        self.map_callback()
        self.sb_obs = rospy.Subscriber('/scan', LaserScan, self._obs_callback)
        self.sb_pose = rospy.Subscriber(
            '/base_pose_ground_truth', Odometry, self._pose_callback)
        self.sb_goal = rospy.Subscriber(
            '/move_base_simple/goal', PoseStamped, self._goal_callback)
        self.controller = rospy.Publisher(
            '/mobile_base/commands/velocity', Twist, queue_size=10)
        rospy.sleep(1)
        
    
        
        
        
        
        
        
        

    def map_callback(self):
        """Get the occupancy grid and inflate the obstacle by some pixels. You should implement the obstacle inflation yourself to handle uncertainty.
        """
        self.map = rospy.wait_for_message('/map', OccupancyGrid).data
        
        self.aug_map = copy.deepcopy(self.map)
        
        self.aug_map = np.array(self.aug_map)

        self.aug_map = self.aug_map.reshape(self.world_height,self.world_width)

        self.aug_map = np.where(self.aug_map < 0 , 0, self.aug_map)
        
        temp = np.nonzero(self.aug_map)
        
        inflate = int(self.inflation_ratio/2)+1
        print("inflating")
        
        
        for i in range(len(temp[0])):
            row_index = temp[0][i]
            column_index = temp[1][i]
            
            for w in range(inflate):
                
                self.aug_map[row_index,max(0,column_index-w)] = 100
                
                self.aug_map[row_index,min(self.world_width-1,column_index+w)] = 100
                
                self.aug_map[max(0,row_index-w),column_index] = 100
                
                self.aug_map[min(self.world_height-1,row_index+w),column_index] = 100
                
                
                self.aug_map[min(self.world_height-1,row_index+w),min(self.world_width-1,column_index+w)] = 100
                
                self.aug_map[min(self.world_height-1,row_index+w),max(0,column_index-w)] = 100
                self.aug_map[max(0,row_index-w),max(0,column_index-w)] = 100
                self.aug_map[max(0,row_index-w),min(self.world_width-1,column_index+w)] = 100
                
        print("inflate done")
        
        
        
        
        
        

    def _pose_callback(self, msg):
        """get the raw pose of the robot from ROS

        Arguments:
            msg {Odometry} -- pose of the robot from ROS
        """
        self.pose = msg

    def _goal_callback(self, msg):
        self.goal = msg
        self.generate_plan()

    def _get_goal_position(self):
        goal_position = self.goal.pose.position
        return (goal_position.x, goal_position.y)

    def set_goal(self, x, y, theta=0):
        """set the goal of the planner

        Arguments:
            x {int} -- x of the goal
            y {int} -- y of the goal

        Keyword Arguments:
            theta {int} -- orientation of the goal; we don't consider it in our planner (default: {0})
        """
        a = PoseStamped()
        a.pose.position.x = x
        a.pose.position.y = y
        a.pose.orientation.z = theta
        self.goal = a

    def _obs_callback(self, msg):
        """get the observation from ROS; currently not used in our planner; researve for the next assignment

        Arguments:
            msg {LaserScan} -- LaserScan ROS msg for observations
        """
        self.last_obs = msg

    def _d_from_goal(self, pose):
        """compute the distance from current pose to the goal; only for goal checking

        Arguments:
            pose {list} -- robot pose

        Returns:
            float -- distance to the goal
        """
        goal = self._get_goal_position()
        return sqrt((pose[0] - goal[0])**2 + (pose[1] - goal[1])**2)

    def _check_goal(self, pose):
        """Simple goal checking criteria, which only requires the current position is less than 0.25 from the goal position. The orientation is ignored

        Arguments:
            pose {list} -- robot post

        Returns:
            bool -- goal or not
        """
        if self._d_from_goal(pose) < 0.25:
            return True
        else:
            return False

    def create_control_msg(self, x, y, z, ax, ay, az):
        """a wrapper to generate control message for the robot.

        Arguments:
            x {float} -- vx
            y {float} -- vy
            z {float} -- vz
            ax {float} -- angular vx
            ay {float} -- angular vy
            az {float} -- angular vz

        Returns:
            Twist -- control message
        """
        message = Twist()
        message.linear.x = x
        message.linear.y = y
        message.linear.z = z
        message.angular.x = ax
        message.angular.y = ay
        message.angular.z = az
        return message
    
        
    def update_neighbors(self,x,y):
       
        tempList = []
        
        
        if not self.collision_checker(x+1, y):
            
            
            tempList.append([x+1, y])
            
        if not self.collision_checker(x-1, y):
            
            
            tempList.append([x-1, y])
            
        if not self.collision_checker(x, y+1):
            
            
            tempList.append([x, y+1])
            
        if not self.collision_checker(x, y-1):
            
            
            tempList.append([x, y-1])
        
        
       
        self.neighbors[x,y] = tempList
        
        
   
        
       
    def generate_plan(self):
        """TODO: FILL ME! This function generates the plan for the robot, given a goal.
        You should store the list of actions into self.action_seq.

        In discrete case (task 1 and task 3), the robot has only 4 heading directions
        0: east, 1: north, 2: west, 3: south

        Each action could be: (1, 0) FORWARD, (0, 1) LEFT 90 degree, (0, -1) RIGHT 90 degree

        In continuous case (task 2), the robot can have arbitrary orientations

        Each action could be: (v, \omega) where v is the linear velocity and \omega is the angular velocity
        """

        x,y,theta = self.get_current_discrete_state()
        start = (x,y)
        goal = (self.goal.pose.position.x,self.goal.pose.position.y)

        for i in range (self.width_grid):
            
            for w in range(self.height_grid):
                
                self.update_neighbors(i, w)

        print("calculating path")
        optimal_path = a_star_algorithm(self.neighbors,start,goal,self.height_grid,self.width_grid)

        #remove the first element (duplicate) if start position is not the goal
        if len(optimal_path) > 2:
            optimal_path = optimal_path[1:len(optimal_path)]
            self.action_seq = self.map_path(optimal_path)
        else:
            self.action_seq =[]
        print("finished")

    def map_path(self,optimal_path):
        
        forward = (1,0)
        left = (0,1)
        x,y,theta = self.get_current_discrete_state()
        temp_seq = []
        
        for i in range(len(optimal_path)-1) :
            if optimal_path[i+1][1] - optimal_path[i][1] == 1: #go up
                while(theta != 1):
                    x1,y1,theta1 = self.discrete_motion_predict(x, y, theta, 0, 1, dt=0.5, frequency=10)
                    temp_seq.append(left)
                    x = x1
                    y = y1
                    theta = theta1
                temp_seq.append(forward)
                
            elif optimal_path[i+1][1] - optimal_path[i][1] == -1: #go down
                while(theta != 3):
                    x1,y1,theta1 = self.discrete_motion_predict(x, y, theta, 0, 1, dt=0.5, frequency=10)
                    temp_seq.append(left)
                    x = x1
                    y = y1
                    theta = theta1
                temp_seq.append(forward)
            elif optimal_path[i+1][0] - optimal_path[i][0] == -1: #go left
                while(theta != 2):
                    x1,y1,theta1 = self.discrete_motion_predict(x, y, theta, 0, 1, dt=0.5, frequency=10)
                    temp_seq.append(left)
                    x = x1
                    y = y1
                    theta = theta1
                temp_seq.append(forward)
            elif optimal_path[i+1][0] - optimal_path[i][0] == 1: #go right
                while(theta != 0):
                    x1,y1,theta1 = self.discrete_motion_predict(x, y, theta, 0, 1, dt=0.5, frequency=10)
                    temp_seq.append(left)
                    x = x1
                    y = y1
                    theta = theta1
                temp_seq.append(forward)
            
            else:#unexcepted error
                break
        return temp_seq




    def get_current_continuous_state(self):
        """Our state is defined to be the tuple (x,y,theta). 
        x and y are directly extracted from the pose information. 
        Theta is the rotation of the robot on the x-y plane, extracted from the pose quaternion. For our continuous problem, we consider angles in radians

        Returns:
            tuple -- x, y, \theta of the robot
        """
        x = self.pose.pose.pose.position.x
        y = self.pose.pose.pose.position.y
        orientation = self.pose.pose.pose.orientation
        ori = [orientation.x, orientation.y, orientation.z,
               orientation.w]

        phi = np.arctan2(2 * (ori[0] * ori[1] + ori[2] * ori[3]), 1 - 2 *
                         (ori[1] ** 2 + ori[2] ** 2))
        return (x, y, phi)

    def get_current_discrete_state(self):
        """Our state is defined to be the tuple (x,y,theta). 
        x and y are directly extracted from the pose information. 
        Theta is the rotation of the robot on the x-y plane, extracted from the pose quaternion. For our continuous problem, we consider angles in radians

        Returns:
            tuple -- x, y, \theta of the robot in discrete space, e.g., (1, 1, 1) where the robot is facing north
        """
        x, y, phi = self.get_current_continuous_state()
        def rd(x): return int(round(x))
      
        return rd(x), rd(y), rd(phi / (np.pi / 2))


    def collision_checker(self, x, y):

        if (int(x) < 1 or int(y) < 1):
            return True

        if (int(x) > self.world_width / 20 - 1 or int(y) > self.world_height / 20 - 1):
            return True

        temp = self.aug_map[
               int(max(0, y - 1) * (self.world_height / 10)):int(max(0, y - 1) * (self.world_height / 10) + 20),
               int(x * (self.world_width / 10)):int(x * (self.world_width / 10) + 20)]
        temp = (temp[::-1])

        if temp[0, 0] == 100:

            return True
        else:

            return False
    
    def motion_predict(self, x, y, theta, v, w, dt=0.5, frequency=10):
        """predict the next pose of the robot given controls. Returns None if the robot collide with the wall
        The robot dynamics are provided in the homework description

        Arguments:
            x {float} -- current x of robot
            y {float} -- current y of robot
            theta {float} -- current theta of robot
            v {float} -- linear velocity 
            w {float} -- angular velocity

        Keyword Arguments:
            dt {float} -- time interval. DO NOT CHANGE (default: {0.5})
            frequency {int} -- simulation frequency. DO NOT CHANGE (default: {10})

        Returns:
            tuple -- next x, y, theta; return None if has collision
        """
        num_steps = int(dt * frequency)
        dx = 0
        dy = 0
        for i in range(num_steps):
            if w != 0:
                dx = - v / w * np.sin(theta) + v / w * \
                    np.sin(theta + w / frequency)
                dy = v / w * np.cos(theta) - v / w * \
                    np.cos(theta + w / frequency)
            else:
                dx = v*np.cos(theta)/frequency
                dy = v*np.sin(theta)/frequency
            x += dx
            y += dy

            if self.collision_checker(x, y):
                
                return None
            theta += w / frequency
        return x, y, theta

    def discrete_motion_predict(self, x, y, theta, v, w, dt=0.5, frequency=10):
        """discrete version of the motion predict. Note that since the ROS simulation interval is set to be 0.5 sec
        and the robot has a limited angular speed, to achieve 90 degree turns, we have to execute two discrete actions
        consecutively. This function wraps the discrete motion predict.

        Please use it for your discrete planner.

        Arguments:
            x {int} -- current x of robot
            y {int} -- current y of robot
            theta {int} -- current theta of robot
            v {int} -- linear velocity
            w {int} -- angular velocity (0, 1, 2, 3)

        Keyword Arguments:
            dt {float} -- time interval. DO NOT CHANGE (default: {0.5})
            frequency {int} -- simulation frequency. DO NOT CHANGE (default: {10})

        Returns:
            tuple -- next x, y, theta; return None if has collision
        """
        w_radian = w * np.pi/2
        first_step = self.motion_predict(x, y, theta*np.pi/2, v, w_radian)
        if first_step:
            second_step = self.motion_predict(
                first_step[0], first_step[1], first_step[2], v, w_radian)
            if second_step:
                
                return (round(second_step[0]), round(second_step[1]), round(second_step[2] / (np.pi / 2)) % 4)
        return None

    def publish_control(self):
        """publish the continuous controls
        """
        for action in self.action_seq:
            msg = self.create_control_msg(action[0], 0, 0, 0, 0, action[1])
            self.controller.publish(msg)
            rospy.sleep(0.6)

    def publish_discrete_control(self):
        """publish the discrete controls
        """
        for action in self.action_seq:
            msg = self.create_control_msg(
                action[0], 0, 0, 0, 0, action[1]*np.pi/2)
            self.controller.publish(msg)
            rospy.sleep(0.6)
            self.controller.publish(msg)
            rospy.sleep(0.6)

    def publish_stochastic_control(self):
        """publish stochastic controls in MDP.
        In MDP, we simulate the stochastic dynamics of the robot as described in the assignment description.
        Please use this function to publish your controls in task 3, MDP. DO NOT CHANGE THE PARAMETERS :)
        We will test your policy using the same function.
        """
        current_state = self.get_current_discrete_state()
        while not self._check_goal(current_state):
            current_state = self.get_current_discrete_state()
            action = self.action_table["{},{},{}".format(current_state[0],
                                                         current_state[1], current_state[2] % 4)]
            if action == (1, 0) or action == [1, 0]:
                r = np.random.rand()
                if r < 0.9:
                    action = (1, 0)
                elif r < 0.95:
                    action = (np.pi / 2, 1)
                else:
                    action = (np.pi / 2, -1)
            print("Sending actions:", action[0], action[1] * np.pi / 2)
            msg = self.create_control_msg(action[0], 0, 0, 0, 0, action[1] * np.pi / 2)
            self.controller.publish(msg)
            rospy.sleep(0.6)
            self.controller.publish(msg)
            rospy.sleep(0.6)
            time.sleep(1)

def heu(x1,x2, y1,y2):
	
	return abs(x1 - x2) + abs(y1 - y2)


def a_star_algorithm(grid,start,goal,height,width):
    
    #a priority queue would help us to always pick the smallest a star value from the first element
    all_a_star_score = PriorityQueue()
    
    #heuristic list to keep track of all grid's updated heuristic 
    heuristics = {}
    
    #list that states all grids' a star score
    a_star_score_list={}

    #assign heuristics to all grids
    for i in range (width):
        for w in range (height):
            heuristics[(i,w)] = heu(i,w,goal[0],goal[1])

    #set all grids' a star score to infinity
    for i in range (width):
        for w in range (height):
            a_star_score_list[(i,w)] = float("inf")
    #assign start node's a star score as 0
    a_star_score_list[start] = 0

    
    
    
    #initialize the queue
    all_a_star_score.put(0+heuristics[(start[0],start[1])])
    all_a_star_score_dic = {(start,start):0+heuristics[(start[0],start[1])]}
    loop = True

    
    
    while all_a_star_score.not_empty and loop:
        
        
        
        #find the path with the minimal a star score
        try:
            minimal_path = list(all_a_star_score_dic.keys())[list(all_a_star_score_dic.values()).index(all_a_star_score.get(0))]
        except:
            print("no path found")
            return []
            break
        
        del all_a_star_score_dic[minimal_path]
        
        
        
        #start from the last node in the minimal a star score path
        last_minimal_path = minimal_path[-1]
        #print(last_minimal_path,"path")
        if last_minimal_path == goal:
            
            
            return tuple(last_minimal_path)
        
        
        #find all neighbors of d[minimal_path] e.g(1,1)
        for neighbors in grid[last_minimal_path]:
            
            temp = (neighbors[0],neighbors[1])
           
            if(temp == goal):
                
                one_more_node = ()
               
                for item in minimal_path:
                    
                    one_more_node = one_more_node + (item,)
                
                one_more_node = one_more_node + ((tuple(neighbors)),)
                
                loop = False
                return one_more_node
                
            
            #calculate a star score of that neighbor (heuristics of itself + cost between different grid, assume all is 1)
            a_star_score = len(minimal_path)+ heuristics[temp]
            
            
            
            #update a star score of that neighbor if the new a_star_score is smaller
            if a_star_score < a_star_score_list[temp]:
                
                a_star_score_list[temp] = a_star_score
                
            
                #put all a star score of different neighbours into the priority queue to find the minimal
                all_a_star_score.put(a_star_score)
                
                #compute a path that made up of different points with the calculated a star score
               
                one_more_node = ()
               
                for item in minimal_path:
                    
                    one_more_node = one_more_node + (item,)
                
                one_more_node = one_more_node + ((tuple(neighbors)),)
                all_a_star_score_dic[(one_more_node)] = a_star_score
                
        
        
        


if __name__ == "__main__":
    # TODO: You can run the code using the code below
    parser = argparse.ArgumentParser()
    parser.add_argument('--goal', type=str, default='1,8',
                        help='goal position')
    parser.add_argument('--com', type=int, default=0,
                        help="if the map is com1 map")
    args = parser.parse_args()

    try:
        goal = [int(pose) for pose in args.goal.split(',')]
    except:
        raise ValueError("Please enter correct goal format")

    if args.com:
        width = 2500
        height = 983
        resolution = 0.02
    else:
        width = 200
        height = 200
        resolution = 0.05
    
    # TODO: You should change this value accordingly
    robot_radius = np.sqrt (np.square(ROBOT_SIZE) + np.square(ROBOT_SIZE))
    
    inflation_ratio = (int(robot_radius*10)+1)*2+5
 
    planner = Planner(width, height, resolution, inflation_ratio=inflation_ratio)
    planner.set_goal(goal[0], goal[1])
    if planner.goal is not None:
        planner.generate_plan()

    # You could replace this with other control publishers
    planner.publish_discrete_control()

    # save your action sequence
    result = np.array(planner.action_seq)
    
    
    
    #np.savetxt("DSDA_com1.jpg_{}_{}.txt".format(goal[0],goal[1]), result, fmt="%.2e")
    

    # for MDP, please dump your policy table into a json file
    # dump_action_table(planner.action_table, 'mdp_policy.json')

    # spin the ros
    rospy.spin()
