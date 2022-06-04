#! /usr/bin/env python

########## Import ROS Dependencies ##########

import rospy
import math
import time

#from gazebo_msgs.msg import ModelStates 
from geometry_msgs.msg import Twist,Point 
from tf import transformations
from goal_publisher.msg import GoalPoint, PointArray
from sensor_msgs.msg import LaserScan

## Import Odometry for navigation 
from nav_msgs.msg import Odometry

#############################################

############## Global Variables #############

# robot state variables
cur_pos_x = 0
cur_pos_y = 0
yaw_ = 0

# machine state
state_ = 0
state_description = 0
space_description = 0

# goal
all_goals = []
switch=0
target=Point()

#Laser 
regions_=[]
safe_distance= 0.3

# parameters
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed
dist_precision_ = 0.1
err_pos = 0 
err_yaw = 0

# publishers
pub = None
twist_msg = Twist()

#############################################


############ Callback Functions #############
#############################################


######## Get the position of the turtlebot #######

def get_cur_position(msg):
    global cur_pos_x, cur_pos_y, yaw_

    ### Get the current position of robot in the X and Y direction ###
    cur_pos_x= msg.pose.pose.position.x
    cur_pos_y= msg.pose.pose.position.y 
    

    #### yaw : Getting the orientation of the robot in the XY plane### 
    
    # Getting Rotation around each axis
    quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) 
    # Converting from Quaternion to euler form (Requirement: import transformation library)
    euler = transformations.euler_from_quaternion(quaternion) 

    # Assigning the value of rotation in Z to yaw_ variable
    # roll = euler[0], pitch = euler[1]
    yaw_ = euler[2]
    
##################################################


################ Get Goals ####################

def get_goals(msg):
    global all_goals, switch

    # The goal points should only be assigned once hence switch 

    if switch == 0:
        # Assign the goal points to a variable all_goals
        all_goals = msg.goals
        switch +=1
################################################


############## Set Target Point ################
def set_goals():
    global all_goals, cur_pos_x, cur_pos_y, target

    # Define an array based on the length of the goals
    dist_goals= [None]*(len(all_goals))

    # Eucledian distance between the current position and the goal points 
    for i in range(len(all_goals)):
        dist_goals[i]= (math.sqrt(pow((all_goals[i].x-cur_pos_x),2) + pow((all_goals[i].y-cur_pos_y),2)))
        
    # All_goals are sorted based on the increasing order of the distance
    all_goals = [all_goals for _, all_goals in sorted(zip(dist_goals,all_goals))]

    # Assigning the closest point as the target
    target= all_goals[0]
    rospy.loginfo('Target x ={0} y={1}'.format(target.x,target.y))

    # Removing the target position from the all_goals list   
    all_goals = all_goals[1:]
    

###############################################


############### Fix Yaw ################

def fix_yaw(target):
    global yaw_, pub, yaw_precision_, state_, cur_pos_y, cur_pos_x, twist_msg, err_yaw
    
    # Calculate the desired yaw 
    desired_yaw = math.atan2(target.y - cur_pos_y, target.x - cur_pos_x)

    # Calculate the error in the yaw 
    err_yaw = desired_yaw - yaw_

    # Checking if the error of the yaw is within the limit 
    if math.fabs(err_yaw) > yaw_precision_:
        
        # Rotate to minimize the error
        twist_msg.angular.z = 0.25 if err_yaw > 0 else -0.25

    # State change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        
        change_state(1)

########################################################


#################### Change State Function #############

def change_state(state):
    global state_
    # Change the state
    state_ = state
    

########################################################


############### Go straight ######################
def go_straight_ahead(target):
    global yaw_, pub, yaw_precision_, state_,dist_precision_, twist_msg, err_pos, err_yaw
    
    # Checking the error in yaw again
    desired_yaw = math.atan2(target.y - cur_pos_y, target.x - cur_pos_x)
    err_yaw = desired_yaw - yaw_

    # Checking for the error in position
    err_pos = math.sqrt(pow(target.y - cur_pos_y, 2) + pow(target.x - cur_pos_x, 2))
    
    # Comparing the error in position to required distance precision and provide a linear velocity
    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.15
        twist_msg.angular.z = 0
    else:
        # When the angular and linear error is below the precision change state
        change_state(2)
    
    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:

        change_state(0)

#############################################


############## Laser Scan ##################

def get_laser_scan(msg):
    global regions_ ,state_description, safe_distance
    
    # Getting the laser scan and dividing into 5 equal parts 
    # Laser gives a 360 degree scan we only condider 180 degree and divide into 5 regions and taking the minimum value from each region
    #360/5 = 72 #180/5 = 36

    regions_={
        'front' : min(min(msg.ranges[0:17]),min(msg.ranges[342:359]),10),
        'frleft' : min(min(msg.ranges[18:54]), 10),
        'frright' : min(min(msg.ranges[306:341]), 10),
        'left' : min(min(msg.ranges[55:90]), 10),
        'right' : min(min(msg.ranges[270:305]), 10),
    }

    # If no object is detected, go to the point
    if regions_['front']> safe_distance and regions_['frleft'] > safe_distance and regions_['frright']> safe_distance:

        state_description =1
        change_state(0)

   # When an object is detected change state to 3 
    else:

        state_description= 0 
        change_state(3)

#######################################


############# Robot rotate ############

def rob_rotate():
    global twist_msg, regions_, state_description, safe_distance , space_description

    # When an obstacle is ahead check for free space in left and right  
    if regions_['left'] >= regions_['right']:

        space_description = 1

        twist_msg.angular.z = 0.3 
        twist_msg.linear.x = -0.1
        # The robot should avoid the obstacle and align it parallel to the obstacle, also checks for L condition  
        if (0.5*safe_distance) < regions_['right'] < safe_distance and regions_ ['front'] > safe_distance :
            change_state(4)

    elif regions_['left'] < regions_['right']:
        
        space_description = 0

        twist_msg.angular.z = -0.3 
        twist_msg.linear.x = -0.1
        if (0.5*safe_distance) < regions_['left'] < safe_distance and regions_ ['front'] > safe_distance :
            change_state(4)

#########################################


########### Robot Tracing wall #########

def rob_ahead():
    global twist_msg, space_description, state_description,regions_,safe_distance

    # The next task for the robot after aligning itself is to move ahead 
    # There could be a case it goes in a randm direction and comes back and gets stuck in the loop 
    # Hence it moves forward in a way that it traces the obstacle, conditioned space_description = 1
    twist_msg.angular.z = 0.05 if space_description == 0 else -0.05
    twist_msg.linear.x = 0.3

    # When it gets closer to the object it goes back to rob_rotate and corrects the orientation 
    # Thus it keeps flipping between state 3 and 4
    if regions_['front'] < safe_distance:
        change_state(3)

########################################


############### Stop ###################

def stop():
    global twist_msg, err_pos, err_yaw, target, all_goals
    
    # After a target is reached 
    rospy.loginfo("Point reached")
    rospy.loginfo('Position accuracy: [%s]' % err_pos)
    #rospy.loginfo('Yaw accuracy : [%s]' % err_yaw)
    rospy.loginfo('No of points remaining : {}'.format(len(all_goals)))
    rospy.loginfo("####################################")

    # Stop the robot
    twist_msg.linear.x =0
    twist_msg.angular.z =0
    change_state(0)
    

##########################################################
##########################################################


########## Main Function ########

def main():
    global pub,state_,target, state_, cur_pos_x, cur_pos_y
    
    #Initialize node
    rospy.init_node('robot_navigation')
    time.sleep(0.5)

    while not rospy.is_shutdown():

        #Initialize a publisher for Twist msgs
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
        #Initialize a subscriber to get the ModelState values from gazebo 
        #rospy.Subscriber('/gazebo/model_states', ModelStates, get_cur_position)
        #rospy.wait_for_message('/gazebo/model_states', ModelStates)
        #time.sleep(1)

        #Initialize a subscriber to get the pose of the robot using Odometry 
        rospy.Subscriber('/odom', Odometry, get_cur_position)
        rospy.wait_for_message('/odom', Odometry)

        #Initialize a subscriber to goal publisher to get goals
        rospy.Subscriber('/goals', PointArray, get_goals)
        rospy.wait_for_message('/goals', PointArray) 

        #Initialize a subscriber to scan topic to get Laser scan data 
        rospy.Subscriber('/scan', LaserScan, get_laser_scan)
        rospy.wait_for_message('/scan', LaserScan)     

        # Until the all_goals list is empty 
        while all_goals != []:
            #Set the Target
            set_goals()  

            while True:
                #Correct the yaw and publish the values on Twist 
                if state_ == 0:
                    fix_yaw(target)
                    pub.publish(twist_msg)
            
                #Minimize the error in the position  
                elif state_ == 1:
                    go_straight_ahead(target)
                    pub.publish(twist_msg)

                # Once the goal is reached Stop the robot
                elif state_ == 2:
                    stop()
                    pub.publish(twist_msg)
                    break
                    
                # When an object is detected the robot should rotate till no object is detected
                elif state_ == 3:
                    rob_rotate()
                    pub.publish(twist_msg)

                # When the robot is aligned the robot should move forward
                elif state_ == 4:
                    rob_ahead()
                    pub.publish(twist_msg)

                
                time.sleep(0.05)
                
            

        time.sleep(0.05)
        # After the list is empty i.e. when all target points are covered, the code quits 
        rospy.loginfo("All targets reached")
        quit()

if __name__ == '__main__':
    main()

#################################################