#!/usr/bin/env python

import rospy
from goal_publisher.msg import PointArray

import math

goal_list = list()
current_x_pos = 3
current_y_pos = -4

def callback_goal(msg):
    global goal_list

    #This function just takes the goals which has rewards and leaves all the goals without rewards
    for i in range(len(msg.goals)):
        if msg.goals[i].reward > 0:
            goal_list.append([msg.goals[i].x, msg.goals[i].y, msg.goals[i].z, msg.goals[i].reward, 0.0])

    #print ("Goal list after removing zeros")
    #print goal_list


##############################
# SORTING GOAL POINTS
##############################
def dists(point):
    for i in range(len(point)):
        
        #print point 
        x=point[i][0]
        y=point[i][1]
        d=math.sqrt((current_x_pos-x)**2 + (current_y_pos-y)**2)
        goal_list[i][4]=d

def goal_sort():
    global goal_list
    dists(goal_list)

    goal_list.sort(key= lambda x : (-x[3],x[4]))
    #print ("Goal list after sorting distances")
    #print goal_list
    
    #goal_list.sort(key= lambda x : x[3], reverse = True)
    #print ("Goal list after sorting rewards")
    #print goal_list

    


##############################
#MAIN FUNCTION
##############################

def main():

    rospy.init_node("goal_demo") #Initialize node
    
    sub_goal = rospy.Subscriber("/goals", PointArray, callback_goal) #Subscribe to goals topic to get the goals
    rospy.wait_for_message('/goals', PointArray)

    rate = rospy.Rate(2)

    goal_sort() 

    if (goal_list[0][3]/goal_list[0][4]) >= (goal_list[1][3]/goal_list[1][4]):
        goal = goal_struct(goal_list[0])
        t=60*goal_list[0][4]
    else:
        goal= goal_struct(goal_list[1])
        t=60*goal_list[1][4]



if __name__ == '__main__':
    main()