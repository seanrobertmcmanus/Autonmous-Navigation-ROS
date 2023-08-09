#! /usr/bin/env python

#ROS Librarys

import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import time
from minitask5.srv import GeneralMovement as general_movement
from minitask5.srv import MoveBaseDone as move_base_srv
from minitask5.srv import ObjectIdService as object_id_service

#Maths and plotting 
import numpy as np
import matplotlib.pyplot as plt 
import tf 
import random
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point 
from std_srvs.srv import Empty, EmptyResponse
import actionlib

#Other 
import threading
import time 
from minitask5.msg import PerceptionNode as perception_node_msg
from minitask5.msg import MapPoint as map_point_msg

code = 1

class Control:
    def __init__(self):
        s = rospy.Service('movement_code', general_movement, self.change_code)
        #global code = 1
        #1=general movement
        #2=move base
        #3=beacon
        #4=rotate
        #5=stop

    def change_code(self, req):
        global code
        current_code = code
        code = req.a
        return current_code

    # def send_request(self):
    #     #wait for service to become available
    #     rospy.wait_for_service('movement_type')
    #     try:
    #         #create handle 'movement_code' for calling the service just like calling a function
    #         movement_type = rospy.ServiceProxy('movement_type', general_movement)
    #         #call the handle and give it the code as argument. It will return the desired code
    #         self.code = movement_type(self.code)
    #     except rospy.ServiceException as e:
    #         print("Service call failed: %s"%e)
 
class Laser:
    def __init__(self):
    
        self.data = []
        self.front_data = []
       
        self.right_data = []
        self.front_right_turn = []
        self.right_data_filtered = []

        self.front_min = 0
        
        self.right_min = 0
        self.front_right_turn_min = 0
        self.right_max = 0

class WallFollow:
    def __init__(self):
        rospy.init_node('Wallfollow_node')
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.laser_scan_cb)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_cb)
        self.msg_vel=Twist()
        self.LS = Laser()
        self.rate = rospy.Rate(20)
        self.velocity = 0
        self.angular_velocity = 0
        self.wall_found = False
        self.last_x = 0
        self.last_y = 0

    def odom_cb(self,msg):
        self.orientation = msg.pose.pose.orientation.z
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y

    def distance_moved(self):
        x = self.last_x-self.pos_x
        y = self.last_y-self.pos_y
        distance = math.hypot(x,y)
        if distance > 1:
            self.last_x = self.pos_x
            self.last_y = self.pos_y
            return True
        else:
            return False
        
    def stop_wf(self):
        if code != 1:
            print("stop_wf")
            self.msg_vel.linear.x= 0
            self.msg_vel.angular.z = 0
            self.publisher.publish(self.msg_vel)
            return True
        else:
            return False


    def laser_scan_cb(self, msg):
        self.LS.data = msg.ranges
        
        self.LS.right_data = msg.ranges[225:315]
        self.LS.front_data = np.concatenate((msg.ranges[0:20],msg.ranges[340:360]))
        self.front_right_turn = msg.ranges[313:317]

        self.LS.right_min = min(min(self.LS.right_data), 10)
        self.LS.front_min = min(min(self.LS.front_data), 10)
        self.LS.front_right_turn_min = min(min(self.front_right_turn), 10)

        self.LS.right_data_filtered = [v for v in self.LS.right_data if not (math.isinf(v) or math.isnan(v))]
        self.LS.right_max = max(self.LS.right_data_filtered)

    def wall_detect(self):
        if self.LS.right_min <0.8:
            #print("found")
            self.wall_found = True
            self.rh_wall_follow()
        else:
            self.wall_found = False
            #self.publish_data
            #print("not found")
            return False
    
    def walk(self):
        while(code ==1):
            if self.stop_wf():
                return
            self.wall_detect()
            self.msg_vel.linear.x= 0.1
            if self.LS.front_min < 0.5:
                self.msg_vel.linear.x= 0
                self.msg_vel.angular.z = 0.2
            self.publisher.publish(self.msg_vel)
    
    #follow rh wall until rh wall is lost
    def rh_wall_follow(self):
        time.sleep(1)
        self.last_x = self.pos_x
        self.last_y = self.pos_y
        while(code == 1):
            if self.stop_wf():
                return
            #rotate every 1 meter moved to help localize robot
            if self.distance_moved() == True and self.LS.right_min>0.3:
                start_time = rospy.Time.now()
                while not rospy.is_shutdown() and rospy.Time.now() - start_time < rospy.Duration(10.0):
                    self.msg_vel.linear.x= 0
                    self.msg_vel.angular.z = 0.5
                    self.publisher.publish(self.msg_vel)

            #get error distance from wall        
            error_distance = 0.5-self.LS.right_min
            
            #get error angle from wall. Parallel is error_angle=0
            error_angle = self.LS.data[315]-self.LS.data[225]
            if error_angle>1 or error_angle<-1 or math.isnan(error_angle):
                error_angle=0
            
            self.msg_vel.linear.x= 0.2 -(abs(error_distance)*2)
            if self.msg_vel.linear.x < 0.1:
                self.msg_vel.linear.x = 0.1
            self.msg_vel.angular.z = (-error_angle/3)+(error_distance*1.5)

            if self.LS.front_min < 0.6:
                self.msg_vel.linear.x= 0
                self.msg_vel.angular.z = 0.2
                if self.LS.front_min < 0.3:
                    self.msg_vel.angular.z = 0.3
            elif self.LS.front_right_turn_min > 1: #was 1
                self.msg_vel.angular.z = -0.2
            
            #print(self.msg_vel.linear.x)
            
            if self.LS.right_min >1:
                print("exiting")
                self.msg_vel.linear.x= 0
                self.msg_vel.angular.z = 0
                self.publisher.publish(self.msg_vel)
                self.wall_found = False
                #self.publish_data()
                break
            #self.publish_data()

            self.publisher.publish(self.msg_vel)
       
    def run(self):
            self.rate.sleep()
            while not rospy.is_shutdown():
                if self.stop_wf():
                    return
                self.walk()

class Beacon:
    def __init__(self):
        self.perception_sub = rospy.Subscriber('perception_node', perception_node_msg, self.perception_cb)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.msg_vel=Twist()
        self.angle = 0
        self.smallest = 10
        self.run = False
    
    def perception_cb(self,msg):
        #perception_objects = msg.objectOfInterest
        #print(perception_objects)
        if self.run == True:
            if msg.objects_detected:
                for i in msg.objectOfInterest:
                    if i.distance < self.smallest  and (i.object_type==2 or i.object_type==3):
                        #print(i.distance)
                        if i.distance < 0.5:
                            self.msg_vel.linear.x= 0
                            self.msg_vel.angular.z = 0
                            self.publisher.publish(self.msg_vel)
                            print("found object of type: {}".format(i.object_type))
                            self.run = False
                            break
                        self.smallest = i.distance
                        self.msg_vel.linear.x= 0.1
                        self.msg_vel.angular.z = i.angle
                        self.publisher.publish(self.msg_vel) 
                    elif math.isnan(i.distance) and (i.object_type==2 or i.object_type==3):
                        #print("isnan")
                        self.msg_vel.linear.x= 0
                        self.msg_vel.angular.z = i.angle
                        if abs(i.angle) <0.1:
                            print("found object of type: {}".format(i.object_type))
                            self.run = False
                            self.msg_vel.linear.x= 0
                            self.msg_vel.angular.z = 0
                            self.publisher.publish(self.msg_vel)
                            break
                        self.publisher.publish(self.msg_vel)

class Rotate:
    def __init__(self):
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.msg_vel=Twist()

    def run(self):
        print("Beginning full rotation")
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and rospy.Time.now() - start_time < rospy.Duration(70.0):
            self.msg_vel.linear.x= 0
            self.msg_vel.angular.z = 0.1
            self.publisher.publish(self.msg_vel)
        self.msg_vel.linear.x= 0
        self.msg_vel.angular.z = 0
        self.publisher.publish(self.msg_vel)
        print("Finished full rotation")

    def stop(self):
        self.msg_vel.linear.x= 0
        self.msg_vel.angular.z = 0
        self.publisher.publish(self.msg_vel)

class Location:
    def __init__(self, pos_x, pos_y, orientation_w):
        self.position = Point(pos_x, pos_y, 0)
        self.orientation_x = 0
        self.orientation_y = 0
        self.orientation_z = 0
        self.orientation_w = orientation_w

class map_navigation:
    def __init__(self):
        #rospy.wait_for_service('move_base_done')
        self.map_point_sub = rospy.Subscriber('pp_points', map_point_msg, self.map_point_cb)
        self.send_data = rospy.ServiceProxy('move_base_done', move_base_srv)
        self.send_id = rospy.ServiceProxy('object_id_service', object_id_service)
        self.preset_locations = [
            Location(2.36, -3.36, 1.7),
            Location(0.3, -3.76, 1.24 ),
            Location(-0.32, 0.2, 1.54),
            Location(-1.4, 0.98 ,-1.3 ),
            Location(-1.4, 0.98 , 1.54),
            Location(3.36, 1.95,2.3 ),
            Location(5.37, -0.24, -1.76),
            Location(5.49, 2.32, 1.7)
        ]
        self.locations = []
        self.temp_locations = []
        self.goal_reached = 0
        self.temp_object_id = 0
        self.object_id = 0
        
    def send_done(self):
        dummy = self.send_data(self.goal_reached)

    def send_object_id(self):
        dummy = self.send_id(self.object_id)

    # def path_planning_callback(self,msg):
    #     for i in msg.name:
    #         loc = Location(i.x,i.y,i.yaw)
    #         self.locations.append(loc)

    def map_point_cb(self,msg):
        self.temp_locations = []
        self.temp_object_id = msg.object_id
        for i in msg.visit_points:
            self.temp_locations.append(Location(i.x,i.y,i.yaw))

    def move_to_goal(self, location):
        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        #Wait for action server 
        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for the move_base action server to come up")
        
        goal = MoveBaseGoal()

        #Set up the frame params 
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Move towards the goal
        goal.target_pose.pose.position = location.position
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = location.orientation_w

        rospy.loginfo("Sending goal location")
        start_time = rospy.Time.now()
        ac.send_goal(goal)

        #ac.wait_for_result(rospy.Duration(5))

        while(rospy.Time.now() - start_time < rospy.Duration(45.0)):
            print("You have reached the destination")
            if(ac.get_state() == GoalStatus.SUCCEEDED):
                print("You have reached the destination")
                return True
        return False
        
    def run(self):
            global code
        #for location in self.locations:
            rospy.wait_for_service('/move_base/clear_costmaps')
            clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            clear_costmaps()
            self.object_id = self.temp_object_id
            self.locations = self.temp_locations
            self.locations.reverse()
            if len(self.locations) > 0:
                for i in range(2):
                    self.goal_reached = self.move_to_goal(self.locations.pop(0))
                    if len(self.locations) == 0:
                        self.goal_reached = 3
                        self.send_object_id()
                        self.send_done()
                        code = 3
            elif len(self.preset_locations) > 0:
                self.goal_reached = self.move_to_goal(self.preset_locations.pop(0))
                self.goal_reached = 2
                self.send_done()
            
if __name__ == '__main__':
    try:
        control = Control()
        Wallfollow = WallFollow()
        rotate = Rotate()
        mn = map_navigation()
        beacon = Beacon()
        while(1):
            if code==1:
                print("wall following")
                Wallfollow.run()
                
            elif code==2:
                print("In stop state")
                mn.run()

            elif code==3:
                #print("In Beaconing state")
                beacon.run = True

            elif code == 4:
                #print("In rotating state")
                rotate.run()
                code = 5

            elif code == 5:
                #print("In stop state")
                rotate.stop()
                
            
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass