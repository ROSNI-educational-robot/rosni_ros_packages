#!/usr/bin/env python

import rospy
import numpy as np
import cv2 as cv
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap


def get_map_client():
    rospy.logwarn("Waiting for service")
    rospy.wait_for_service('dynamic_map')
    rospy.logwarn("Service ready")
    try:
        get_map = rospy.ServiceProxy('dynamic_map', GetMap)
        resp1 = get_map()
        rospy.logwarn("Mapa obtenido")
        mapa = np.array(list(resp1.map.data))
        # print(len(mapa))
        new_map = []
        counter = 0
        for i in range(0, 384):
            row = []
            for i in range(0, 384):
                if mapa[counter] > 0:
                    row.append(0)
                elif mapa[counter] < 0:
                    row.append(100)
                else:
                    row.append(255)
                counter += 1
            new_map.append(row)
        
        img_1=cv.imencode(".jpeg",np.array(new_map),[cv.IMWRITE_JPEG_QUALITY,90])[1].tobytes() 
        
        #ret_code, jpg_buffer = cv.imencode(
        #    ".jpg", np.array(new_map), [int(cv.IMWRITE_JPEG_QUALITY), 90])
        #sender.send_jpg(rpi_name, jpg_buffer)
        # mp = cv.imdecode(jpg_buffer, 0)
        rospy.logwarn(len(img_1))
        return img_1
        # cv.imwrite("/home/guiser/tfg_ws/mapa_conseguido.jpg",
        #            mp)
        # cv.imshow("Display window", mp)
        # k = cv.waitKey(0)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return 0


# if __name__ == "__main__":
#     rospy.init_node("map_visualizer")
#     print("init node")
#     get_map_client()
