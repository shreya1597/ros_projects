#!/usr/bin/env python

import rospy #importing ros stuff
import actionlib #tools and interface to set up an action server
import operator

#import ros messages
from goal_publisher.msg import PointArray #to get the goal position
from geometry_msgs.msg import PoseWithCovarianceStamped #this expresses an estimated pose with a reference coordinate frame and timestamp
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction, MoveBaseResult #for navigation
from tf import transformations
from tf.transformations import euler_from_quaternion #quaternion for raw, pitch and yaw
from geometry_msgs.msg import Twist
import time
import math
from sensor_msgs.msg import LaserScan

goal_list = list() #creating a list for goal points
time_window = 0
new_list = list()
dist_to_goal = 0
switch=0
twist_msg = Twist()
count = 0
cn = 1
safe_distance = 0.3
state = 100
switch_las = 0
frees = 0
sum1  = 0
points = 0

##############################
# DEFINING CALLBACK FOR /goals
##############################

def callback_goal(msg):
    global goal_list,switch

    #This function just takes the goals which has rewards and leaves all the goals without rewards
    if switch ==0:
        for i in range(len(msg.goals)):
            if msg.goals[i].reward > 0:
                goal_list.append([msg.goals[i].x, msg.goals[i].y, msg.goals[i].z, msg.goals[i].reward, 0.0])
                switch = switch+1


##############################
# DEFINING CALLBACK BOT LOCATION
##############################

def callback_location(pos):
    global bot_x_pos, bot_y_pos, bot_theta

    #This function provide the location of the robot using amcl node
    bot_x_pos = pos.pose.pose.position.x
    bot_y_pos = pos.pose.pose.position.y
    bot_ort = pos.pose.pose.orientation
    (roll,pitch,bot_theta) = euler_from_quaternion([bot_ort.x, bot_ort.y, bot_ort.z, bot_ort.w])


##############################
# GOAL MSG TO ACTION SERVER
##############################

def movebase_client(point):
    global X, Y,bot_goal
    
    bot_goal = MoveBaseGoal() #creates a new goal 
    bot_goal.target_pose.header.frame_id = "map"
    bot_goal.target_pose.header.stamp = rospy.Time.now()

    X = bot_goal.target_pose.pose.position.x = point[0] #moving the robot in x axis
    Y = bot_goal.target_pose.pose.position.y = point[1] #moving the robot in y axis
    bot_goal.target_pose.pose.position.z = 0

    bot_goal.target_pose.pose.orientation.x = 0
    bot_goal.target_pose.pose.orientation.y = 0
    bot_goal.target_pose.pose.orientation.z = 0
    bot_goal.target_pose.pose.orientation.w = 1.0 #No rotation of mobile base w.r.t map frame
    return bot_goal


##############################
# SORTING GOAL POINTS
##############################

def distance(goal_point):

    #The function caalculates the distance between the starting point and the goal point 
    for i in range(len(goal_point)):
        x=goal_point[i][0]
        y=goal_point[i][1]
        d=math.sqrt((bot_x_pos-x)**2 + (bot_y_pos-y)**2)
        
        #The calculated distance is then appended to the list 
        goal_list[i][4]=d

def goal_sort():
    global goal_list
    

    #Find the distance for each goal point in the list 
    distance(goal_list) 

    #Sorting the goal list based on the distance
    goal_list.sort(key= lambda x : (x[4])) 

##############################
#GOAL SELECT
##############################

def sel_goal(goal_list):
    #Select the goal point with greater reward and minimum distance 
   
        if (goal_list[1][3]/goal_list[1][4]) >= (goal_list[0][3]/goal_list[0][4]):
            swap = goal_list[1]
            goal_list[1]= goal_list[0]
            goal_list[0] = swap
    

##############################
#ODOM CALLBACK
##############################
def odom_callback():
    global twist_msg, frees

    if frees == 'front' or frees == 'back':
        twist_msg.linear.x = 0.06 if frees == 'front' else -0.06
        twist_msg.angular.z = 0
       
    if frees == 'left' or frees == 'right':
        twist_msg.angular.z = -0.3 if frees == 'right' else 0.3
        twist_msg.linear.x = 0
        


def odom_stop():
    global twist_msg

    twist_msg.linear.x = 0
    twist_msg.angular.z = 0

##############################
# Laser Scan 
##############################

def get_laser_scan(msg):
    global safe_distance, new_list, bot_theta, bot_x_pos, bot_y_pos, state, switch_las, frees
    
    regions_={
        'front' : min(min(msg.ranges[0:17]),min(msg.ranges[342:359]),10),
        'frleft' : min(min(msg.ranges[18:54]), 10),
        'frright' : min(min(msg.ranges[306:341]), 10),
        'left' : min(min(msg.ranges[55:90]), 10),
        'right' : min(min(msg.ranges[270:305]), 10),
    }

    find_space={
        'front' : (min(min(msg.ranges[0:17]),min(msg.ranges[342:359]),10)),
        #'frleft' : min(min(msg.ranges[18:54]), 10),
        #'frright' : ma(msg.ranges[306:341]), 10),
        'left' : (min(min(msg.ranges[55:90]),10)),
        'right' : (min(min(msg.ranges[270:305]),10)),
        'back' : (min(min(msg.ranges[165:195]),10))
    }
    
    frees = max(find_space.iteritems(), key=operator.itemgetter(1))[0]
    
##############################
# LOCALIZATION FUNCTION 
##############################

def loct():
    global twist_msg
    #print("In loct callback")
    twist_msg.linear.x = 0.01
    twist_msg.angular.z = 0.6283

##############################
# DEFINING THE MAIN FUNCTION 
##############################

def main():

    rospy.init_node("nav_demo") #Initialize node
    
    sub_goal = rospy.Subscriber("/goals", PointArray, callback_goal) #Subscribe to goals topic to get the goals
    rospy.wait_for_message('/goals', PointArray)
    rospy.sleep(1)

    sub_loc = rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, callback_location) #Subscribe to amcl_pose to localize the turtlebot
    rospy.wait_for_message('amcl_pose', PoseWithCovarianceStamped)

    client = actionlib.SimpleActionClient("move_base", MoveBaseAction) #creating an action client called move_base
    client.wait_for_server() # Waits till the action server has started

    rospy.Subscriber('scan', LaserScan, get_laser_scan)

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        
        global time_window, bot_theta, bot_x_pos, bot_y_pos, dist_to_goal, new_list, X, Y, count, cn, state, goal_list, points, bot_goal
        loct()
        time.sleep(2)
        pub.publish(twist_msg)
        time.sleep(5)
        
        while goal_list != []:
            goal_sort()

            if len(goal_list) > 1:
                sel_goal(goal_list)

            #Selecting the goal point
            goal= movebase_client(goal_list[0])

            #Start move base stack navigation
            client.send_goal(goal)
            
            while True:

                #Creating a time window of 0.02 seconds which increases in each iteration of the while loop
                #The first if condition of the time window collects the robot position when the time frame is between 0 and 0.025
                #i.e. the start of the interval 
                if 0 <= time_window <= 0.025:
                    f_pose_x = bot_x_pos
                    f_pose_y = bot_y_pos
                    f_theta = bot_theta

                #Increment time window
                time_window =time_window + 0.02
                
                #Calculating the error in the distance between the robot position and the goal position
                dist_to_goal = math.sqrt((bot_x_pos - X)**2 + (bot_y_pos - Y)**2)

                #Function to be executed when the robot reaches the goal point successfully 
                if client.get_state() == 3 or dist_to_goal<0.015:

                    print(" Goal point: ({}, {}) reached" .format(goal_list[0][0],goal_list[0][1]))
                    print(" Number of goal points covered = %s" % cn)
                    points = points + goal_list[0][3]
                    print(" Total reward points : {}" .format(points))
                    print ("///////////////////////////////////")
                    del goal_list[0]
                    cn = cn + 1
                    rospy.sleep(1)
                    time_window = 0
                    break
                    
                #Function to be executed when the goal point is aborted
                if client.get_state() == 4:
                    print("Goal aborted")
                    new_list.append(goal_list[0])
                    del goal_list[0]
                    client.cancel_goal()
                    time_window = 0
                    break
    
                #Function to be executed when the robot is stuck at the same place for an interval  
                if 13 <= time_window <= 13.4:
                    
                    #Get the current location of the robot at the end of the interval
                    l_pose_x = bot_x_pos
                    l_pose_y = bot_y_pos
                    l_theta = bot_theta

                    #Calculate the distance between the initial point and the last point in the interval
                    #Comparing with a threshold 
                    #If the value lies within the threshold,it can be infered that the robot is stuck  
                    dis= math.sqrt(pow((l_pose_x - f_pose_x),2) + pow((l_pose_y - f_pose_y),2))
                
                    threshold= 0.15

                    if dis < threshold:
                        print("Goal point saved for later")
                        count = count + 1
                        #Append the unreachable goals to new list for later
                        new_list.append(goal_list[0])
                        client.cancel_goal()

                        odom_callback()
                        time.sleep(2)
                        pub.publish(twist_msg)
                        time.sleep(5)
                        del goal_list[0]
                        
                        time_window = 0
                        break
                    #Reset time frame    
                    time_window = 0

                #Append new list back to goal list to navigate using move base
                if len(goal_list) == 1:
                    global sum1 
                    
                    if sum1 < 4 :
                        
                        for i in range(len(new_list)):
                            goal_list.append(new_list[i])
                            print("Goals appended")
                        
                        new_list =[]
                        
                        sum1 +=1

                time.sleep(0.02)

        # Return back to the start if the points are still not reachable
        bot_goal.target_pose.header.frame_id = "map"
        bot_goal.target_pose.header.stamp = rospy.Time.now()

        bot_goal.target_pose.pose.position.x = 0 #moving the robot to origin
        bot_goal.target_pose.pose.position.y = 0.5 #moving the robot to origin
        bot_goal.target_pose.pose.orientation.w = 1.0               

        client.send_goal(bot_goal)         

        time.sleep(0.02)



if __name__ == '__main__':
    main()
