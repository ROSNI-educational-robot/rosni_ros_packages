#!/usr/bin/env python

# Python headers
import math

# ROS headers
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import JointState


class controller():
    def __init__(self, R, L):
        """
        class to calculate and publish the wheels velocities.

        Atributes:
            R: float
                wheel radius (in meters)
            L: float
                distance between wheels (in meters)
        """
        self.R = R
        self.L = L

        self.wr = 0.0
        self.wl = 0.0
        self.R_pos = 0
        self.L_pos = 0

        self.last_time = rospy.get_rostime()

        self.rad2rpm = 60/(2*math.pi)

        self.dt = 0.0

        self.odom_Sub = rospy.Subscriber(
            "odom", Odometry, self.compute_velocities)
        self.joint_state_pub = rospy.Publisher(
            "joint_states", JointState, queue_size=50)

    def publish_joint_state(self):
        current_time = rospy.get_rostime()
        self.dt = current_time - self.last_time
        pose_R = ((self.wr*math.pi/30)*self.dt.to_sec())
        self.R_pos = (self.R_pos + pose_R) % (2*math.pi)

        pose_L = ((self.wl*math.pi/30)*self.dt.to_sec())
        self.L_pos = (self.L_pos + pose_L) % (2*math.pi)

        joints = JointState()
        joints.header.stamp = current_time
        joints.name.append("right_wheel_to_motor")
        joints.position.append(self.R_pos)
        joints.velocity.append(self.wr*math.pi/30)
        joints.name.append("left_wheel_to_motor")
        joints.position.append(self.L_pos)
        joints.velocity.append(self.wl*math.pi/30)

        # publish the message
        self.joint_state_pub.publish(joints)

    def compute_velocities(self, odom_msg):
        vl = odom_msg.twist.twist.linear.x - \
            (odom_msg.twist.twist.angular.z * self.L)/2
        vr = odom_msg.twist.twist.angular.z * self.L + vl

        self.wl = vl/self.R
        self.wr = vr/self.R

        self.publish_joint_state()


if __name__ == '__main__':
    try:
        rospy.init_node('robot_dif_controller', anonymous=True)
        r_controller = controller(0.085, 0.235)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
