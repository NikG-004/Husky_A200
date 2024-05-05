#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
import matplotlib.pyplot as plt
from math import sqrt, atan2

data_x_1 = []
data_y_1 = []
class square:

    def __init__(self):

        rospy.init_node("Husky_square")
        pub = rospy.Publisher("/husky_velocity_controller/cmd_vel", Twist, queue_size = 10)
        sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)

        self.model_pose = ModelStates()
        self.rate = rospy.Rate(10)

    def callback(self, data):

        self.model_pose = data.pose[1]

    def eucleadian_distance(self, goal_pose):
        current_x = self.model_pose.position.x
        current_y = self.model_pose.position.y
        return sqrt((goal_pose.x - current_x)**2 + (goal_pose.y - current_y)**2)
    
    def linear_vel(self, goal_pose, Kp=0.5):
        return Kp*self.eucleadian_distance(goal_pose)
    
    def angle(self, goal_pose):
        current_x = self.model_pose.position.x
        current_y = self.model_pose.position.y
        current_theta = self.get_yaw()

        goal_theta = atan2(goal_pose.y - current_y, goal_pose.x - current_x)
        error = goal_theta - current_theta
        return error
    
    def angular_vel(self, goal_pose, Kp=1.5):
        return Kp*self.angle(goal_pose)
    
    def get_yaw(self):
        quaternion = self.model_pose.orientation
        euler = self.quaternion_to_euler(quaternion)
        return euler
    
    def quaternion_to_euler(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)

        return yaw_z
    
    def square(self):
        global data_x, data_y
        data_x = []
        data_y = []
        
        a = -4
        while a<=4:
            data_x.append(a)
            data_y.append(a**2)
            a+=1
        print(data_x)
        print(data_y)

    def move_husky(self):
        global goal_pose
        no = int(input("Enter any no : "))

        vel_msg = Twist()
        self.square()
        goal_pose = ModelStates()
        goal_x = goal_pose.position.x
        goal_y = goal_pose.position.y
        
        for a in range(len(data_x)):
            goal_x = data_x[a]
            goal_y = data_y[a]
            print(goal_x)
            print(goal_y)

            while abs(round((self.angular_vel(goal_pose))/1.5, 2)) >= 0.01:
                    #linear_velocity = self.linear_vel(goal_x, goal_y)
                    angular_velocity = self.angular_vel(goal_pose)

                    
                    #vel_msg.linear.x = linear_velocity
                    vel_msg.angular.z = angular_velocity
                
                    self.velocity_publisher.publish(vel_msg)
                    """data_x.append(self.pose.position.x)
                    data_y.append(self.pose.position.y)"""
                    self.rate.sleep()
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
            while round((self.linear_vel(goal_pose))/0.5, 2) >= 0.1:
                linear_velocity = self.linear_vel(goal_x, goal_y)
                #angular_velocity = self.angular_vel(goal_x, goal_y)

                #vel_msg = Twist()
                vel_msg.linear.x = linear_velocity
                #vel_msg.angular.z = angular_velocity
            
                self.velocity_publisher.publish(vel_msg)
                data_x_1.append(self.model_pose.position.x)
                data_y_1.append(self.model_pose.position.y)
                self.rate.sleep()
            vel_msg.linear.x = 0
            self.velocity_publisher.publish(vel_msg)

            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
            plt.plot(data_x_1, data_y_1)
            plt.plot(data_x, data_y)
            plt.show()

if __name__ == '__main__':
    try:
        husky_controller = square()
        husky_controller.move_husky()
    except rospy.ROSInterruptException:
        pass




        



        
    
        

    