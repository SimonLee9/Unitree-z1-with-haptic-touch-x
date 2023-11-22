#!/usr/bin/env python

import rospy
import socket
import struct

from tf.transformations import quaternion_from_euler

from geometry_msgs.msg import Pose

from std_msgs.msg import Float64 # add


# 서버 IP 주소 및 포트
SERVER_IP = '192.168.35.19'
PORT = 65534

class Client_node():
    def __init__(self):

        self.connect_flag = False
        
        # add
        '''
        self.joint_tau_subscribers = []
        for i in range(6):
            topic_name = 'joint_tau_' + str(i)
            joint_tau_subscriber = rospy.Subscriber(topic_name, Float64, self.joint_tau_callback, callback_args=i)
            self.joint_tau_subscribers.append(joint_tau_subscriber)
        '''
        # 노드 종료시 실행할 함수
        rospy.on_shutdown(self.shutdown_func)

    def connect_to_server(self):        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        # 소켓 연결 대기 시간 초과시 time out 예외 발생
        self.sock.settimeout(3)
        try:
            self.sock.connect((SERVER_IP, PORT))
            self.sock.settimeout(None)               
            rospy.loginfo("Connect")
            self.connect_flag = True
        except Exception as ex:
            rospy.loginfo(ex)       

    def shutdown_func(self):
        # 노드 종료시 실행되는 함수
        # 소켓을 닫거나, 서버 종료 명령을 보낸다
        try:
            self.sock.close()
        except:
            pass
        rospy.loginfo("Shtudown node")

    def send_msg(self):
        # send_data = 11.0
        send_data = struct.pack(
        '<d',
        11.0,
        )
        
        self.sock.sendall(send_data)

    def receive_msg(self):
        try:
            # 받을 데이터 길이(Byte)
            # 4로 설정할 경우 짧은 데이터는 그대로 반환하지만
            # 더 긴 데이터의 경우 4바이트만 반환하고 반복문에 의해 나머지 반환
            data = self.sock.recv(48)
            print(len(data))
            #rospy.loginfo('recv msg : '+str(data))
            
            data_float = struct.unpack('<d', data)
            rospy.loginfo('recv msg_unpack : '+str(data_float))
            # (x, y, z, w) = quaternion_from_euler(data_float[3], data_float[4], data_float[5])
            
            #rospy.loginfo(f'{x}, {y}, {z}, {w}')
            
            # self.haptic_pose_publisher(data_float, x, y, z, w)
            if not data:
                self.connect_flag = False
        except Exception as e:
            print(e)
            self.connect_flag = False
    
    def haptic_pose_publisher(self,data_float,x,y,z,w):
        haptic_pose_pub = rospy.Publisher('haptic_pose',Pose, queue_size=4)
        # rate = rospy.Rate(15) # 15Hz
        pose_msg=Pose()
        pose_msg.position.x=data_float[0]
        pose_msg.position.y=data_float[1]
        pose_msg.position.z=data_float[2]
        pose_msg.orientation.x=x
        pose_msg.orientation.y=y
        pose_msg.orientation.z=z
        pose_msg.orientation.w=w

        haptic_pose_pub.publish(pose_msg)
    '''
    def joint_tau_callback(self, data, joint_number):
        rospy.loginfo("Received joint_tau_{joint_number}: {data.data}")
    '''
    def main(self):        
        while not rospy.is_shutdown():
            
            if not self.connect_flag:
                self.connect_to_server()
            else:
                # 추후 Send는 subscriber의 callback 함수에서 수행
                self.send_msg()
                self.receive_msg()

if __name__ == '__main__':
    rospy.init_node('TCP_IP_node')
    node = Client_node()
    node.main()
