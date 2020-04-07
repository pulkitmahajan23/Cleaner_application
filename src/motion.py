#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time

x=0
y=0
z=0
angle=0

def poseCallBack(poseMessage):
    global x
    global y, angle
    x = poseMessage.x
    y = poseMessage.y
    angle = poseMessage.theta 

def move(speed, distance, isForward):
    velocity = Twist()
    global x, y
    x0 = x
    y0 = y
    if (isForward):
        velocity.linear.x = abs(speed)
    else:
        velocity.linear.x = -abs(speed)
    
    distance_moved = 0.0
    loopRate = rospy.Rate(10)
    cmd_vel = '/turtle1/cmd_vel'
    velocity_pub = rospy.Publisher(cmd_vel, Twist, queue_size=10)
    while True:
        rospy.loginfo('Moving')
        velocity_pub.publish(velocity)
        loopRate.sleep()
        distance_moved = distance_moved +abs(0.5 * math.sqrt(((x-x0)**2)+((y-y0)**2)))
        print(distance_moved)
        if not (distance_moved<distance):
            rospy.loginfo("Reached destination")
            break
    velocity.linear.x = 0
    velocity_pub.publish(velocity)

def rotate(angular_speed_degree, relative_angle_degree, clockwise):
    global angle
    velocity = Twist()
    velocity.linear.x = 0
    velocity.linear.y = 0
    velocity.linear.z = 0
    velocity.angular.x = 0
    velocity.angular.y = 0
    velocity.angular.z = 0

    angle0 = angle
    angular_speed = math.radians(abs(angular_speed_degree))
    if(clockwise):
        velocity.angular.z = -abs(angular_speed)
    else:
        velocity.angular.z = abs(angular_speed)
    angle_moved = 0.0
    loopRate = rospy.Rate(10)
    cmd_vel = '/turtle1/cmd_vel'
    velocity_pub = rospy.Publisher(cmd_vel, Twist, queue_size=10)
    t0 = rospy.Time.now().to_sec()
    while True:
        rospy.loginfo("Rotating")
        velocity_pub.publish(velocity)
        t1 = rospy.Time.now().to_sec()
        angle_moved =  (t1-t0) * angular_speed_degree
        loopRate.sleep()
        if(angle_moved>relative_angle_degree):
            rospy.loginfo("Reached")
            break
    velocity.angular.z = 0
    velocity_pub.publish(velocity)


if __name__ == "__main__":
    try:
        rospy.init_node('turtle_cleaner',anonymous=True)
        cmd_vel='/turtle1/cmd_vel'
        velocity_pub = rospy.Publisher(cmd_vel,Twist,queue_size=10)
        position = '/turtle1/pose'
        pose_sub = rospy.Subscriber(position,Pose,poseCallBack)
        time.sleep(2)
        rotate(30, 90, False)
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated")
        exit(0)


