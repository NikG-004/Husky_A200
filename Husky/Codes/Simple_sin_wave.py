#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
#from math import sin, pi
import matplotlib.pyplot as plt
import math
import numpy as np

# Global variables
data_set = []  # List to store the y-values of the sine wave
data = []
def generate_sine_wave():
    global data_set
    amplitude = 5.0  # Amplitude of the sine wave
    frequency = 1  # Frequency of the sine wave
    phase_shift = 0.0  # Phase shift of the sine wave
    a = 0

    while a <= 4*math.pi: 
        data.append(a)
        data_set.append(amplitude * math.sin(frequency * a + phase_shift))

        a += 0.1
    plt.plot(data, data_set)   
    plt.show()
    print(data)
    print(data_set)

def move_turtle():
    global data_set
    rospy.init_node('turtle_sine_wave')
    pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)

    rate = rospy.Rate(10)  # Update rate in Hz

    while not rospy.is_shutdown():
        generate_sine_wave()

        for y in data_set:
            twist_msg = Twist()
            twist_msg.linear.x = 1.0  # Linear velocity in the x-axis
            twist_msg.angular.z = y  # Angular velocity in the z-axis

            pub.publish(twist_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        move_turtle()
    except rospy.ROSInterruptException:
        pass
