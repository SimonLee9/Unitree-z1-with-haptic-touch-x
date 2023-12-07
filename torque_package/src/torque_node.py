#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray

def calculate_torque(q, dq, Kp, Kd):
    # Simulated torque calculation
    torque = Kp * q + Kd * dq
    return torque

def callback(data):
    q = data.data[0]
    dq = data.data[1]
    #tau = data.data[2]
    Kp = data.data[3]
    Kd = data.data[4]
    #reserve = data.data[5:]

    # Calculate torque
    calculated_torque = calculate_torque(q, dq, Kp, Kd)
    
    # Publish the calculated torque
    #pub = rospy.Publisher('/z1_gazebo/Joint01_controller/calculated_torque', Float64, queue_size=10)
    #pub.publish(calculated_torque)

    # Process the received data and calculated torque
    print("Received Data:")
    print(f"q: {q}")
    print(f"dq: {dq}")
    print(f"Kp: {Kp}")
    print(f"Kd: {Kd}")
    print(f"Calculated Torque: {calculated_torque}")
    #print(f"Reserve: {reserve}")

def torque_subscriber():
    rospy.init_node('torque_node', anonymous=True)
    #rospy.Subscriber('/z1_gazebo/Joint01_controller/command', Float64MultiArray, callback)
    rospy.Subscriber('/z1_gazebo/Joint01_controller/command', Float64MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        torque_subscriber()
    except rospy.ROSInterruptException:
        pass
