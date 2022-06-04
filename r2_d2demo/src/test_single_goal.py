#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client():

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction) #create an action client called "move_base"
    client.wait_for_server() # Waits till the action server has started 
    goal = MoveBaseGoal() #creates a new goal 
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = 1 #moving the robot 1m in x axis
    goal.target_pose.pose.orientation.w = 1.0 #No rotation of mobile base w.r.t map frame

    client.send_goal(goal) #sends the goal to the action server
    wait = client.wait_for_result() #waits for the server to finish the action

    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


if __name__ == '__main__':
    try:
        #initialize a rospy node for the action client to publish and subscribe
        rospy.init_node('single_goal')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal Reached!")
    except rospy.ROSInterruptException:
        rospy.loginfo("The Testing was successful.")