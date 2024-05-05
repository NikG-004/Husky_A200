#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from math import atan2, sqrt, asin
import math
import matplotlib.pyplot as plt


# data = []
# data_set = []
data_x = []
data_y = []
class HuskyController:
    def __init__(self):
        rospy.init_node('husky_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.update_pose)

        self.pose = ModelStates()
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        # Find the index of 'husky' model in the model_states
        model_index = data.name.index('husky')
        # Update the pose of the 'husky' model
        self.pose = data.pose[model_index]

    def linear_vel(self, goal_x, goal_y, kp=0.2):
        current_x = self.pose.position.x
        current_y = self.pose.position.y
        distance = sqrt((goal_x - current_x) ** 2 + (goal_y - current_y) ** 2)
        return kp * distance

    def angular_vel(self, goal_x, goal_y, kp=3.2):
        current_x = self.pose.position.x
        current_y = self.pose.position.y
        current_theta = self.get_yaw()

        desired_theta = atan2(goal_y - current_y, goal_x - current_x)
        error_theta = desired_theta - current_theta

        return kp * error_theta

    def get_yaw(self):
        quaternion = self.pose.orientation
        euler = self.quaternion_to_euler(quaternion)
        return euler[2]

    @staticmethod
    def quaternion_to_euler(quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def generate_sine_wave(self):
        global data, data_set
        amplitude = 5.0  # Amplitude of the sine wave
        frequency = 0.5  # Frequency of the sine wave
        phase_shift = 0.0  # Phase shift of the sine wave
        a = 0
        data = []
        data_set = []

        while a <= 4*math.pi: 
            data.append(a)
            data_set.append(amplitude * math.sin(frequency * a + phase_shift))

            a += 0.5
           

        print(data)
        print(data_set)


    def move_husky(self):
        # global data, data_set
        
        NoofIterations = int(input("Enter the No. of Iterations : "))

        vel_msg = Twist()
        self.generate_sine_wave()
        for a in range(len(data)):
            goal_x = float(data[a])
            goal_y = float(data_set[a])
            print(goal_x)
            print(goal_y)

            while abs(round((self.angular_vel(goal_x, goal_y))/3.2, 2)) >= 0.01:
                #linear_velocity = self.linear_vel(goal_x, goal_y)
                angular_velocity = self.angular_vel(goal_x, goal_y)

                
                #vel_msg.linear.x = linear_velocity
                vel_msg.angular.z = angular_velocity
            
                self.velocity_publisher.publish(vel_msg)
                """data_x.append(self.pose.position.x)
                data_y.append(self.pose.position.y)"""
                self.rate.sleep()
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
            while round((self.linear_vel(goal_x, goal_y))/0.3, 2) >= 0.1:
                linear_velocity = self.linear_vel(goal_x, goal_y)
                #angular_velocity = self.angular_vel(goal_x, goal_y)

                #vel_msg = Twist()
                vel_msg.linear.x = linear_velocity
                #vel_msg.angular.z = angular_velocity
            
                self.velocity_publisher.publish(vel_msg)
                data_x.append(self.pose.position.x)
                data_y.append(self.pose.position.y)
                self.rate.sleep()
            vel_msg.linear.x = 0
            self.velocity_publisher.publish(vel_msg)

        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        plt.plot(data, data_set)
        plt.plot(data_x, data_y)
        plt.show()
if __name__ == '__main__':
    try:
        husky_controller = HuskyController()
        husky_controller.move_husky()
    except rospy.ROSInterruptException:
        pass
