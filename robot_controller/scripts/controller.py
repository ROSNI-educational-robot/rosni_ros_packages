#!/usr/bin/env python3

# Python headers
import math
import serial

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

        self.wr = 0
        self.wl = 0
        self.wr_feedback = 0
        self.wl_feedback = 0
        self.R_pos = 0
        self.L_pos = 0

        self.rad2rpm = 60/(2*math.pi)

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vx = 0
        self.vy = 0
        self.vth = 0

        self.dt = 0

        self.arduino_motors = serial.Serial(
            "/dev/arduino_motors", baudrate=9600)

        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        self.joint_state_pub = rospy.Publisher(
            "joint_states", JointState, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()

    def loop(self):
        try:
            twist = rospy.wait_for_message(
                "/robot/cmd_vel", Twist, timeout=0.05)
            self.compute_velocities(twist)
        except:
            pass
        self.read_feedback()

    def read_feedback(self):
        data = self.arduino_motors.read().decode("ascii")
        if data != "":
            while data != "X":
                data = self.arduino_motors.read().decode("ascii")
            msg = ""
            data = self.arduino_motors.read().decode("ascii")
            while data != "Y":
                msg += data
                data = self.arduino_motors.read().decode("ascii")

            # [dirR, stepsR, dirL, stepsL]
            feedback = msg.split(";")

            if int(feedback[0]):
                self.wr_feedback = -float(feedback[1])  # rpm
            else:
                self.wr_feedback = float(feedback[1])  # rpm
            if int(feedback[2]):
                self.wl_feedback = -float(feedback[3])  # rpm
            else:
                self.wl_feedback = float(feedback[3])  # rpm
            self.dt = float(feedback[4])/1000.0  # seconds

        self.compute_odometry()

    def compute_odometry(self):
        vr = (self.wr_feedback*math.pi/30) * self.R
        vl = (self.wl_feedback*math.pi/30) * self.R

        self.vx = (vr + vl)/2
        self.vth = (vr - vl)/self.L

        # compute odometry in a typical way given the velocities of the robot
        delta_x = (self.vx * math.cos(self.th)) * self.dt
        delta_y = (self.vx * math.sin(self.th)) * self.dt
        delta_th = self.vth * self.dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

        current_time = rospy.get_rostime()
        # first, we'll publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            odom_quat,
            current_time,
            "base_footprint",
            "odom"
        )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.),
                              Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_footprint"
        odom.twist.twist = Twist(
            Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))

        pose_R = ((-self.wr_feedback*math.pi/30)*self.dt)
        self.R_pos = (self.R_pos + pose_R) % (2*math.pi)

        pose_L = ((self.wl_feedback*math.pi/30)*self.dt)
        self.L_pos = (self.L_pos + pose_L) % (2*math.pi)

        joints = JointState()
        joints.header.stamp = current_time
        joints.name.append("right_wheel_to_motor")
        joints.position.append(self.R_pos)
        joints.velocity.append(self.wr_feedback*math.pi/30)
        joints.name.append("left_wheel_to_motor")
        joints.position.append(self.L_pos)
        joints.velocity.append(self.wl_feedback*math.pi/30)

        # publish the message
        self.odom_pub.publish(odom)
        self.joint_state_pub.publish(joints)

    def compute_velocities(self, twist):
        vl = twist.linear.x - (twist.angular.z * self.L)/2
        vr = twist.angular.z * self.L + vl

        self.wl = vl/self.R
        self.wr = vr/self.R

        rpmR = self.wr * self.rad2rpm
        rpmL = self.wl * self.rad2rpm

        if (rpmR > 120):
            rpmL = rpmL - (rpmR-120)
            rpmR = 120
        elif (rpmL > 120):
            rpmR = rpmR - (rpmL-120)
            rpmL = 120

        dir_l = 0
        dir_r = 0

        if rpmL < 0:
            dir_l = 1
            rpmL = -rpmL

        if rpmR < 0:
            dir_r = 1
            rpmR = -rpmR

        msg = ("X"+str(dir_r)+";"+str(round(rpmR))+";" +
               str(dir_l)+";"+str(round(rpmL))+"Y").encode("utf-8")
        self.arduino_motors.write(msg)


if __name__ == '__main__':
    try:
        rospy.init_node('robot_dif_controller', anonymous=True)
        r_controller = controller(0.0425, 0.235)
        while not rospy.is_shutdown():
            r_controller.loop()
    except rospy.ROSInterruptException:
        pass
