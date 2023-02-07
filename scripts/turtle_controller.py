#!/usr/bin/env python3

# import ROS for developing the node
import rospy
# this going to read turtlesim/Pose messages this time
from turtlesim.msg import Pose
# import new message from the custom package package
from robotics_lab1.msg import Turtlecontrol
# import geomtry pck for control commands
from geometry_msgs.msg import Twist

def callback_pose(data):
    global x_t
    x_t = data.x

def callback_control(data):
    global kp, xd
    kp = data.kp
    xd = data.xd

def turtle_controller():
    global x_t, kp, xd
    # init node
    rospy.init_node('turtle_controller', anonymous=True)
    # add a subscriber to it to read the position information
    rospy.Subscriber("/turtle1/pose", Pose, callback_pose)
    # add a subscrber to it to read the Turtcontrol custom message
    rospy.Subscriber("/turtle1/control_params", Turtlecontrol, callback_control)
    # add a publisher with a new topic using the cmd_vel message
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    x_t = 0.0
    kp = 0.0
    xd = 0.0
    while not rospy.is_shutdown():
        e_x = xd - x_t
        v_x = kp * e_x
        twist = Twist()
        twist.linear.x = v_x
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        turtle_controller()
    except rospy.ROSInterruptException:
        pass
