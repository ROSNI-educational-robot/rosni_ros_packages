#!/usr/bin/env python3
import time
import rospy
import socket
import get_map
import math
import subprocess
import multiprocessing
from node_controller_msgs.srv import StartSlamNav, StartSlamNavRequest


class SockServer():
    def __init__(self):
        self.slam_process = multiprocessing.Process(target=self.slam_on)
        rospy.wait_for_service('start_slam_nav')
        try:
            self.node_req = rospy.ServiceProxy('start_slam_nav', StartSlamNav)
            self.slam_started = False

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

        self.SocketServerPublisher()

    def start_slam(self):
        rospy.wait_for_service('start_slam_nav')
        req = StartSlamNavRequest()
        req.node_name = "slam"
        res = self.node_req(req)
        return res.ok

    def stop_slam(self):
        rospy.wait_for_service('start_slam_nav')
        req = StartSlamNavRequest()
        req.node_name = "stop"
        res = self.node_req(req)
        return res.ok

    def start_nav(self):
        rospy.wait_for_service('start_slam_nav')
        req = StartSlamNavRequest()
        req.node_name = "nav"
        res = self.node_req(req)
        return res.ok

    def stop_nav(self):
        rospy.wait_for_service('start_slam_nav')
        req = StartSlamNavRequest()
        req.node_name = "stop"
        res = self.node_req(req)
        return res.ok

    def slam_on(self):
        while True:
            rospy.logwarn("getting map")
            image_data = get_map.get_map_client()
            rospy.logwarn("mapa got it")
            rospy.logwarn(len(image_data))
            len_msg = str(len(image_data))
            rospy.logwarn(len_msg)
            cnt = 0
            number_zeros = 10-len(len_msg)

            if len(len_msg) < 10:
                rospy.logwarn(len(len_msg))
                while cnt < number_zeros:
                    rospy.logwarn("+1")
                    len_msg = '0'+len_msg
                    cnt = cnt+1
            rospy.logwarn('Sending XXXXX')
            self.connection.sendall(b'XXXXX')
            self.connection.sendall(len_msg.encode("utf-8"))
            message = image_data
            rospy.logwarn(len_msg)
            self.connection.sendall(message)
            rospy.logwarn('Image sended')
    def get_ip(self):
        check_process = subprocess.check_output(
            "/home/pi/scripts/network_config/ip_get.sh")
        ip_raspi = check_process.decode("utf-8")

        if (ip_raspi[:2] != "NO"):
            print("IP: " + ip_raspi)
            return ip_raspi[:-1]
        else:
            print(
                "Cannot connect to a network!\nStarting Hotspot:\n\tssid: raspi\n\tpassword: 12345678")
            return "0"

    def SocketServerPublisher(self):
        close_socket = 0
        reinitialize_socket = 0

        while close_socket == 0 or reinitialize_socket == 1:

            # Get ip number
            ip_name = self.get_ip()#'192.168.43.12'

            # Create a TCP/IP socket
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

            # Create server adress and bind port with
            port = 54321
            server_address = (ip_name, port)
            sock.bind(server_address)
            print('Server initialized on', ip_name, 'and port', port)

            close_socket = 0

            # Listen for incoming connections
            sock.listen(10)

            while True and close_socket == 0:

                try:
                    # Wait for a connection
                    print('Waiting for a connection')
                    self.connection, client_address = sock.accept()

                except:
                    print('Server Closed')
                    close_socket = 1
                    self.connection.close()
                    sock.close()

                try:
                    print('Connection from', client_address)

                    # Receive the data in chunks of 1000 characters and retransmit it
                    while True:

                        amount_received = -1
                        message_received_ascii = ''

                        # While the amount expected stays at -1, keep listening to receive a message from the server to prove that the server received the credentials correctly
                        if amount_received == -1:
                            len_message_received_ascii = self.connection.recv(10)
                            len_message_received = int(len_message_received_ascii.decode(
                                'ascii'))

                            c = 0
                            len_rcv = 5
                            message_completed = ''

                            while c < (math.ceil(float(len_message_received/len_rcv))):
                                message_received_ascii = self.connection.recv(
                                    len_rcv)
                                message_received = message_received_ascii.decode(
                                    'ascii')
                                message_completed = message_completed+message_received
                                c = c+1

                            print(message_completed)
                            amount_received = len(message_completed)

                        # If the message has been received, send a message to make the client know
                        if message_received_ascii:
                            message = 'Data successfully transferred'
                            len_msg = str(len(message))
                            cnt = 0
                            number_zeros = 10-len(len_msg)

                            if len(len_msg) < 10:
                                while cnt < number_zeros:
                                    len_msg = '0'+len_msg
                                    cnt = cnt+1

                            message = len_msg+message
                            ascii_message = message.encode('ascii')
                            self.connection.sendall(ascii_message)
                            
                            #if self.slam_started and not ('Stop_SLAM' in message_completed):
                            #    rospy.logwarn("antes de mapa")
                            #    image_data = get_map.get_map_client()
                            #    rospy.logwarn("despues mapa")
                            #    rospy.logwarn(len(image_data))
                            #    len_msg = str(len(image_data))
                            #    rospy.logwarn(len_msg)
                            #    cnt = 0
                            #    number_zeros = 10-len(len_msg)

                            #    if len(len_msg) < 10:
                            #        rospy.logwarn(len(len_msg))
                            #        while cnt < number_zeros:
                            #            rospy.logwarn("+1")
                            #            len_msg = '0'+len_msg
                            #            cnt = cnt+1
                            #    rospy.logwarn('Sending XXXXX')
                            #    self.connection.sendall(b'XXXXX')
                            #    self.connection.sendall(len_msg.encode("utf-8"))
                            #    message = image_data
                            #    rospy.logwarn(len_msg)
                            #    self.connection.sendall(message)
                            #    self.slam_process.start()
                            #    rospy.logwarn('Image sended')
 
                            if 'Initialize_SLAM' in message_completed:
                                self.start_slam()
                                time.sleep(5)
                                self.slam_started = True
                                rospy.logwarn("antes de mapa")
                                image_data = get_map.get_map_client()
                                rospy.logwarn("despues mapa")
                                rospy.logwarn(len(image_data))
                                len_msg = str(len(image_data))
                                rospy.logwarn(len_msg)
                                cnt = 0
                                number_zeros = 10-len(len_msg)

                                if len(len_msg) < 10:
                                    rospy.logwarn(len(len_msg))
                                    while cnt < number_zeros:
                                        rospy.logwarn("+1")
                                        len_msg = '0'+len_msg
                                        cnt = cnt+1
                                rospy.logwarn('Sending XXXXX')
                                self.connection.sendall(b'XXXXX')
                                self.connection.sendall(len_msg.encode("utf-8"))
                                message = image_data
                                rospy.logwarn(len_msg)
                                self.connection.sendall(message)
                                self.slam_process.start()
                                rospy.logwarn('Image sended')

                            if 'Stop_SLAM' in message_completed:
                                self.stop_slam()
                                time.sleep(3)
                                self.slam_started = False
                                self.slam_process.terminate()
                                self.slam_process.join()
                                print('Slam Finished')

                            if 'Disconnect_Client' in message_completed:
                                self.connection.close()
                                self.connection.shutdown(socket.SHUT_RDWR)
                                close_socket = 1
                                reinitialize_socket = 1

                # Close the connection when client has been disconnected
                except:
                    print('Connection Ended')
                    self.connection.close()


if __name__ == "__main__":
    rospy.init_node("socket_node")
    sock = SockServer()
