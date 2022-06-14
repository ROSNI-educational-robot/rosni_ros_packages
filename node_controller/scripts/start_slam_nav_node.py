#!/usr/bin/env python3
import rospy
from node_controller_msgs.srv import StartSlamNav, StartSlamNavRequest, StartSlamNavResponse
import multiprocessing
import subprocess


class ServiceServer():
    def __init__(self):
        self.service = rospy.Service(
            'start_slam_nav', StartSlamNav, self.handle_node_req)
        self.slam_is_alive = 0
        self.nav_is_alive = 0

    def start_slam(self):
        print("starting Slam")
        process = subprocess.run(["roslaunch", "robot_start_up", "robot_slam.launch"])

    def start_nav(self):
        print("starting Navigation")

    def handle_node_req(self, req):
        # req = StartSlamNavRequest()
        response = StartSlamNavResponse()
        if (req.node_name == "slam"):
            self.slam_process = multiprocessing.Process(target=self.start_slam)
            self.slam_is_alive = 1
            self.slam_process.start()
            response.ok = True
        elif (req.node_name == "nav"):
            self.nav_process == multiprocessing.Process(target=self.start_nav)
            self.nav_is_alive = 1
            self.nav_process.start()
            response.ok = True
        elif (req.node_name == "stop"):
            if self.slam_is_alive:
                print("Saving map")
                subprocess.run(
                    ["rosrun", "map_server", "map_saver", "-f", "/home/pi/map"])
                print("Killing Slam")
                subprocess.run(["rosnode", "kill", "/slam_gmapping"])
                self.slam_process.join()
                self.slam_is_alive = 0
                response.ok = True
            if self.nav_is_alive:
                subprocess.run(["rosnode"], ["kill"], ["/move_base"])
                self.nav_process.join()
                self.nav_is_alive = 0
                response.ok = True
        else:
            response.ok = False

        return response


if __name__ == "__main__":
    rospy.init_node("start_slam_nav_node", anonymous=True)
    s = ServiceServer()
    print("Ready to start Slam or Navigation")
    rospy.spin()
