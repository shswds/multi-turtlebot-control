#!/usr/bin/env python3
#################################################################################
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Gilbert #

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64MultiArray, Float64


# if (keyboard.is_pressed() == "x"):
#     print("You pressed x")
#     # self.shutdown()

msg_1 = """
control your Turtlebot3!
-----------------------
Insert xyz - coordinate.
x : position x (m)
y : position y (m)
z : orientation z (degree: -180 ~ 180)
If you want to close, insert 's'
-----------------------
"""
current_pose = PoseWithCovarianceStamped()
# goal_xy = []
goal_xx = []
goal_yy = []

class GotoPoint():
    def __init__(self):
        rospy.init_node('map_navigation_kim1', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('tb3_1/cmd_vel', Twist, queue_size=5)
        self.odom_sub = rospy.Subscriber('/tb3_1/amcl_pose', PoseWithCovarianceStamped, self.callback)
        # self.num_list = rospy.Subscriber('my_topic', Float64MultiArray, self.callback1)
        self.coord_1_x_sub = rospy.Subscriber('coord_2_x', Float64MultiArray, self.callback_x)
        self.coord_1_y_sub = rospy.Subscriber('coord_2_y', Float64MultiArray, self.callback_y)

        position = Point()
        move_cmd = Twist()
        r = rospy.Rate(10)

        # print("goal_x 0: ", goal_x)
        # print("goal_y 0: ", goal_y)

        if(len(goal_xx) != 0 and len(goal_yy) != 0):

            for i in range(len(goal_xx)):
                print("제발 !!! : ", i)
                goal_x = goal_xx[i]
                goal_y = goal_yy[i]
                goal_x1 = goal_xx[i+1]
                goal_y1 = goal_yy[i+1]
                target_x = goal_xx[i+2]
                target_Y = goal_yy[i+2]
                # (position11, rotation) = self.get_odom()
                rotation = current_pose.pose.pose.orientation.z

                # rotation = self.get_odom()
                position = current_pose.pose.pose.position
                # print("position", position)
                last_rotation = 0
                linear_speed = 1
                

                goal_distance = sqrt(pow(target_x - position.x, 2) + pow(target_Y - position.y, 2))
                distance = goal_distance
                x_start = position.x
                y_start = position.y
                goal_z = atan2(target_Y - y_start, target_x - x_start)/pi  

                if(goal_z < 0):
                    goal_z = goal_z + 2.0
                if(rotation < 0):
                    rotation = rotation + 2.0

                max_z = 0.25
                

                while abs(rotation - goal_z) > 0.025:
                    # print("aaa : ", aaa)
                    rotation = current_pose.pose.pose.orientation.z

                    if(rotation < 0):
                        # print("!!!!!!!!!!!")
                        rotation = rotation + 2.0

                    # print(f" goal_z : {goal_z}, rotation : {rotation}")

                    if(goal_z > rotation):
                        if(goal_z > rotation + 1):
                            move_cmd.linear.x = 0.00
                            move_cmd.angular.z = -max(abs(rotation - goal_z)*0.1,max_z)
                        else:
                            move_cmd.linear.x = 0.00
                            move_cmd.angular.z = max(abs(rotation - goal_z)*0.1,max_z)
                    else:
                        if (goal_z+1 < rotation):
                            move_cmd.linear.x = 0.00
                            move_cmd.angular.z = max(abs(rotation - goal_z)*0.1,max_z)
                        else :
                            move_cmd.linear.x = 0.00
                            move_cmd.angular.z = -max(abs(rotation - goal_z)*0.1,max_z)
                            
                    self.cmd_vel.publish(move_cmd)
                    r.sleep()

                    move_cmd.angular.z = 0.00
                    self.cmd_vel.publish(move_cmd)

                while distance > 0.07:
                    print(f"target_x : {target_x}, target_Y : {target_Y}")
                    position = current_pose.pose.pose.position
                    rotation = current_pose.pose.pose.orientation.z
                    x_start = position.x
                    y_start = position.y
                    # print(f'position_x : {x_start}, position_y : {y_start}')
                    path_angle = atan2(target_Y - y_start, target_x- x_start)
                    
                    # if path_angle < -pi/4 or path_angle > pi/4:
                    #     if goal_y < 0 and y_start < goal_y:
                    #         path_angle = -2*pi + path_angle
                    #     elif goal_y >= 0 and y_start > goal_y:
                    #         path_angle = 2*pi + path_angle
                    # if last_rotation > pi-0.1 and rotation <= 0:
                    #     rotation = 2*pi + rotation
                    # elif last_rotation < -pi+0.1 and rotation > 0:
                    #     rotation = -2*pi + rotation
                    # move_cmd.angular.z = (path_angle-rotation)*0.2
                    if(rotation < 0):
                        # print("!!!!!!!!!!!")
                        rotation = rotation + 2.0

                    if(path_angle < 0):
                    # print("!!!!!!!!!!!")
                        path_angle = path_angle + 2.0

                    if(path_angle > rotation):
                        if(path_angle > rotation + 1):
                            move_cmd.linear.x = 0.00
                            move_cmd.angular.z = -abs(rotation - path_angle)*0.1
                        else:
                            move_cmd.linear.x = 0.00
                            move_cmd.angular.z = abs(rotation - path_angle)*0.1
                    else:
                        if (path_angle+1 < rotation):
                            move_cmd.linear.x = 0.00
                            move_cmd.angular.z = abs(rotation - path_angle)*0.1
                        else :
                            move_cmd.linear.x = 0.00
                            move_cmd.angular.z = -abs(rotation - path_angle)*0.1
                            
                            
                    if move_cmd.angular.z > 0:
                        move_cmd.angular.z = min(move_cmd.angular.z, 0.05)
                    else:
                        move_cmd.angular.z = max(move_cmd.angular.z, -0.05)
                    
                    distance = sqrt(pow((target_x - x_start), 2) + pow((target_Y - y_start), 2))
                    distance1 = sqrt(pow((goal_x1 - x_start), 2) + pow((goal_y1 - y_start), 2))
                    distance0 = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
                    if(distance0 > distance1 + 0.03):
                        break
                    # move_cmd.linear.x = max(linear_speed * distance, 0.03)
                    move_cmd.linear.x = 0.11

                    
                    last_rotation = rotation
                    self.cmd_vel.publish(move_cmd)
                    r.sleep()
                
                self.cmd_vel.publish(Twist())


    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])


    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    def callback(self, msg):
        global current_pose
        # print("callback on")
        if(msg != current_pose):
            current_pose = msg
            # if(current_pose.pose.pose.position.x != 0):
            #     print("x : ", current_pose.pose.pose.position.x,"y : ",current_pose.pose.pose.position.y, "z : ", current_pose.pose.pose.orientation.z)
        # return current_pose
    
    # def callback1(self, msg):
    #     global goal_xy
    #     goal_xy = msg.data
    #     # print(goal_xy)
        
    def callback_x(self, msg):
        global goal_xx
        goal_xx = msg.data
        # print("goal_x : " , goal_x)
        
    def callback_y(self, msg):
        global goal_yy
        goal_yy = msg.data
        # print("goal_y : ", goal_y)
          
    

if __name__ == '__main__':
    try:
        print(msg_1)
        while not rospy.is_shutdown():
            # print(msg_1)
            
            GotoPoint()
            # rospy.spin()

    except:
        rospy.loginfo("shutdown program.")
