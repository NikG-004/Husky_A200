#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from math import atan2, sqrt, asin

distance_error_sum = 0.0
distance_error_diff = 0.0
last_error = 0.0
KP = 0.18
KI = 0.00
KD = 0.04
class HuskyController:
    def __init__(self):
        rospy.init_node('husky_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.update_pose)

        self.model_pose = ModelStates()
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        # Find the index of 'husky' model in the model_states
        model_index = data.name.index('husky')

        # Update the pose of the 'husky' model
        self.model_pose = data.pose[model_index]

    def linear_vel(self, goal_x, goal_y):
        global distance_error_diff, distance_error_sum, last_error
        current_x = self.model_pose.position.x
        current_y = self.model_pose.position.y
        distance = sqrt((goal_x - current_x) ** 2 + (goal_y - current_y) ** 2)
        distance_error_sum += distance
        distance_error_diff += distance - last_error
        last_error = distance
        return KP * distance + KI * distance_error_sum + KD * distance_error_diff

    def angular_vel(self, goal_x, goal_y, kp=1.5):
        current_x = self.model_pose.position.x
        current_y = self.model_pose.position.y
        current_theta = self.get_yaw()

        desired_theta = atan2(goal_y - current_y, goal_x - current_x)
        error_theta = desired_theta - current_theta

        return kp * error_theta

    def get_yaw(self):
        quaternion = self.model_pose.orientation
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

    def move_husky(self):

        NoofIterations = int(input("Enter the no. of iterations : "))
        """a=[5,10]
        b=[5,10]
        c=0"""
        vel_msg = Twist()
        for _ in range(NoofIterations):
            goal_x = float(input("Set your desired x goal: "))
            goal_y = float(input("Set your desired y goal: "))
            tolerance = float(input("Enter the tolerance: "))
            #c+=1
            """while abs(round((self.angular_vel(goal_x, goal_y))/1.5, 2)) >= 0.01:
                #linear_velocity = self.linear_vel(goal_x, goal_y)
                angular_velocity = self.angular_vel(goal_x, goal_y)

                
                #vel_msg.linear.x = linear_velocity
                vel_msg.angular.z = angular_velocity

                self.velocity_publisher.publish(vel_msg)
                self.rate.sleep()
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
            while round((self.linear_vel(goal_x, goal_y))/0.2, 2) >= round(tolerance, 2):
                linear_velocity = self.linear_vel(goal_x, goal_y)
                #angular_velocity = self.angular_vel(goal_x, goal_y)

           
                vel_msg.linear.x = linear_velocity
                #vel_msg.angular.z = angular_velocity

                self.velocity_publisher.publish(vel_msg)
                self.rate.sleep()
            vel_msg.linear.x = 0
            self.velocity_publisher.publish(vel_msg)"""

            while round((self.linear_vel(goal_x, goal_y))/0.18, 1) >= 0.1:
                if abs(round((self.angular_vel(goal_x, goal_y))/1.5, 2)) >= 0.01:
                    angular_velocity = self.angular_vel(goal_x, goal_y)

                
                    #vel_msg.linear.x = linear_velocity
                    vel_msg.angular.z = angular_velocity

                    self.velocity_publisher.publish(vel_msg)
                    self.rate.sleep()




                elif round((self.linear_vel(goal_x, goal_y))/0.18, 2) >= 0.1:
                    linear_velocity = self.linear_vel(goal_x, goal_y)
                    #angular_velocity = self.angular_vel(goal_x, goal_y)

            
                    vel_msg.linear.x = linear_velocity
                    #vel_msg.angular.z = angular_velocity

                    self.velocity_publisher.publish(vel_msg)
                    self.rate.sleep()

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
    try:
        husky_controller = HuskyController()
        husky_controller.move_husky()
    except rospy.ROSInterruptException:
        pass
