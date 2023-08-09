#! /usr/bin/env python

#ROS Librarys

import rospy

#MSG Librarys 
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from minitask5.msg import *
from minitask5.srv import * 
from minitask5.srv import ObjectIdService as object_id_service

from tf.msg import tfMessage

#Service Librarys
from nav_msgs.srv import GetMap, GetMapRequest, GetPlan
from std_srvs.srv import Empty

#Other Librarys 
import numpy as np
import tf 
import math

#Description 
# The Path Planning Node
# Uses data from:
#   - MapData 
# Useage:
#   - Plan a path to objects of interest based on distance to and obstacles to avoid
#   - Publishes Best 3 paths in array for use in other nodes


#Objects 
class Position:
    def __init__(self):
        self.x = 0 # X Position
        self.y = 0 # Y Position
        self.yaw = 0 # Yaw Orientation

# Object for Storing Shape Positional Data on the map
class ObjectPosition:
    def __init__(self, name, is_object_of_interest, object_visited, visit_points):
        self.object_name = name # Name of Object on the Map 
        self.is_object_of_interest = is_object_of_interest # Is an Object of Interest
        self.object_visited = object_visited # Has the Object been visited
        self.object_visit_points = visit_points #  List of Positions object can be visited from 


class PathPlanningController:
    def __init__(self):
        self.rate = rospy.Rate(10)

        path_to_list = []
        # Path Nodes 
        self.objectPositions = []

        self.current_travel_plan = []

        self.current_location = Position()
        self.is_localized = False
        
        self.start_header_message = Header()
        self.start_header_message.frame_id = "map"

        self.end_header_message = Header()
        self.end_header_message.frame_id = "map"
        
        self.grid_map = None


        # Subs 
        self.points_sub = rospy.Subscriber("map_points", MapPoints, self.map_points_cb) 
        self.location_sub = rospy.Subscriber("location_pub", Localization, self.location_cb) 
        self.map_sub = rospy.Subscriber("map2", OccupancyGrid, self.map_cb)
        
        # Pubs 
        self.point_pub = rospy.Publisher('pp_points', MapPoint, queue_size=10)
        # Service Clients
        s = rospy.Service('object_id_service', object_id_service, self.object_id_cb)
        rospy.wait_for_service('/move_base/NavfnROS/make_plan')
        self.make_plan_client = rospy.ServiceProxy('/move_base/NavfnROS/make_plan', GetPlan)
        
    def object_id_cb(self, msg):
         for i in self.objectPositions:
            if i.object_name == msg.id:
                i.object_visited = True
                break
         return 0

    def get_closest_point_calc(self):
        lowest_point = None
        lowest_distance = 500
        object_number = None

        object_count = 0
        for i in self.objectPositions:
            if i.is_object_of_interest and (not i.object_visited):
                count = 0
                for x in i.object_visit_points:
                    count += 1
                    distance = self.distance_calc(self.current_location, x)
                    print(distance)
                    if distance < lowest_distance:
                        lowest_distance = distance
                        object_number = object_count
                        lowest_point = count 
                    count += 1
            object_count += 1
        if object_number == None:
            return False, []
        
        return True, [lowest_point, object_number]
        
    def get_closes_blue_tile(self, pose):
        lowest_point = None
        lowest_distance = 500
        object_number = None

        object_count = 0
        for i in self.objectPositions:
            if (not i.is_object_of_interest):
                count = 0
                for x in i.object_visit_points:
                    count += 1
                    distance = self.distance_calc(pose, x)
                    print(distance)
                    if distance < lowest_distance:
                        lowest_distance = distance
                        object_number = object_count
                        lowest_point = count 
                    count += 1
            object_count += 1
        if object_number == None:
            return False, []
        
        return True, [lowest_point, object_number]

    def check_map_intersects(self, list_of_points):
        for point in list_of_points:
            pose = point.pose.position
            if self.is_on_blue_tile(pose):
                return True, pose
            else:
                return False, pose

    def distance_calc(self, position_start, position_end):
        x = np.square(position_start.x - position_end.x)
        y = np.square(position_start.y - position_end.y)
        return np.sqrt(x + y)

    def is_on_blue_tile(self, position):
        gy = (position.x-(-10)) / 0.05 # grid y location
        gx = (position.y-(-10)) / 0.05 # grid x location
        position = gx * 384 + gy
        position = int(position)
        if self.grid_map[position] == 90:
            return True
        return False

    def make_plan_cb(self, start_pose, end_pose):
        start = PoseStamped()
        end = PoseStamped()
        start.pose.position.x = start_pose.x
        start.pose.position.y = start_pose.y
        start.pose.position.z = 0
        start.pose.orientation.x = 0
        start.pose.orientation.y = 0
        start.pose.orientation.z = start_pose.yaw
        start.pose.orientation.w = 0

        end.pose.position.x = end_pose.x
        end.pose.position.y = end_pose.y
        end.pose.position.z = 0
        end.pose.orientation.x = 0
        end.pose.orientation.y = 0
        end.pose.orientation.z = end_pose.yaw
        end.pose.orientation.w = 0
        start.header = self.start_header_message
        end.header = self.end_header_message
        tolerance = 10
        tolerance = 0.1
        print(start)
        print(end)
        message  = self.make_plan_client(start, end, tolerance)
        print("Here is the message {}".format(message))
        return message.plan.poses
        
    def location_cb(self, msg):
        self.is_localized = msg.is_localized
        self.current_location.x = msg.location.x
        self.current_location.y = msg.location.y
        self.current_location.yaw = msg.location.yaw

    def map_cb(self, msg):
        self.grid_map = msg.data 

    def map_points_cb(self, msg):
        if len(msg.map_points) >= 1:
            for point in msg.map_points:
                is_present = False 
                for added_points in self.objectPositions:
                    if (added_points.object_name == point.object_id):
                        is_present = True
                if not is_present:
                    new_point = ObjectPosition(point.object_id, point.is_object_of_interest, False, point.visit_points)
                    self.objectPositions.append(new_point)
                        
    def pose_pub(self, pose1, target_id, pose2):
        self.map_points_msg = MapPoint()
        self.map_points_msg.object_id = target_id
        self.map_points_msg.visit_points = []
        if pose1 != None:
            self.map_points_msg.visit_points.append(pose1)
            self.map_points_msg.is_object_of_interest = True
            if pose2 != None:
                self.map_points_msg.visit_points.append(pose1)

        else:
            self.map_points_msg.is_object_of_interest = False
        
        self.point_pub.publish(self.map_points_msg)
         

def main():
    rospy.init_node("path_planning")
    pp_controller = PathPlanningController()
    while not rospy.is_shutdown():
        mid_pose = None
        target_pose = None 
        target_id = 0
        if pp_controller.is_localized:
            if len(pp_controller.objectPositions) >= 1:
                print("3")
                point_found, values = pp_controller.get_closest_point_calc()
                print("4")
                if point_found:
                    # make initial plan
                    target_pose = pp_controller.objectPositions[values[1]].object_visit_points[values[0]]
                    target_id = pp_controller.objectPositions[values[1]].object_name
                    poses = pp_controller.make_plan_cb(pp_controller.current_location, pp_controller.objectPositions[values[1]].object_visit_points[values[0]])
                    print(poses)
                    if len(poses) >= 1:
                        good, bad_point = pp_controller.check_map_intersects(poses)
                        if not good:
                            is_blue_tile, value_of = pp_controller.get_closes_blue_tile(bad_point)
                            if is_blue_tile:
                                mid_pose = pp_controller.objectPositions[values[1]].object_visit_points[values[0]]
            
        pp_controller.pose_pub(target_pose, target_id, mid_pose)



if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 