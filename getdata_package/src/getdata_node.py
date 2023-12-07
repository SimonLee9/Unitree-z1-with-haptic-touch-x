#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose

def sensor_callback(data):
    rospy.loginfo(f"Received sensor data: X={data.position.x}, Y={data.position.y}, Z={data.position.z}")
    #rospy.loginfo(f"X value: {data.position.x}")
    #rospy.loginfo(f"Y value: {data.position.y}")
    #rospy.loginfo(f"Z value: {data.position.z}")

def sensor_subscriber():
    rospy.init_node('sensor_subscriber', anonymous=True)
    rospy.Subscriber('ForceSensorData', Pose, sensor_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        sensor_subscriber()
    except rospy.ROSInterruptException:
        pass
