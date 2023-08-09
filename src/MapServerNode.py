#! /usr/bin/env python
#Imports 
#ROS Librarys 
import rospy
import message_filters
#MSG Librarys 
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, Point
from tf.msg import tfMessage
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan

#Service Librarys
from nav_msgs.srv import GetMap, GetMapRequest, SetMap
from std_srvs.srv import Empty
from minitask5.msg import *
from minitask5.srv import *

#Other Librarys 
import numpy as np
import tf 
import math



#Description 

# Object for Storing Positional Data
class Position:
    def __init__(self):
        self.x = 0 # X Position
        self.y = 0 # Y Position
        self.yaw = 0 # Yaw Orientation

class ObectsOfInterest:
    def __init__(self, object_type, angle, distance):
        self.object_type = object_type
        self.angle = angle 
        self.distance = distance

class PerceptionData:
    def __init__(self):
        self.objects_detected = False
        self.objects_of_interest = []

# Object for Storing Map Object related metadata
class Shape:
    def __init__(self, object_name, type_value, length, mask_length, width, mask_width, map_value, mask_value):
        self.object_name = object_name # Name of the Object on the Map
        self.type_value = type_value # Associated Perception value
        self.map_value = map_value # Value Associated with Object on Map 
        self.length = length # Length of the physical Shape
        self.width = width # Wdith of the Physical Shape
    
        self.mask_value = mask_value
        self.mask_length = mask_length # Mask Length of Shape
        self.mask_width = mask_width # Mask Width of Shape

# Object for Storing Shape Positional Data on the map
class ShapePosition:
    def __init__(self, name, is_object_of_interest, object_visited, visit_points):
        self.object_name = name # Name of Object on the Map 
        self.is_object_of_interest = is_object_of_interest # Is an Object of Interest
        self.object_visited = object_visited # Has the Object been visited
        self.object_visit_points = visit_points #  List of Positions object can be visited from 


# Runs Calculations On OccupancyGrid Map in order to add or remove data from the map
class GridMap:
    def __init__(self, angle_min, angle_max, angle_increment, range_max, world_origin, resolution, size):
        self.shape_id = 0
        # Grid Values
        self.grid_occupied = 100 # Occupied Value of Grid Cell 
        self.grid_free = 0.05 # Free Value of Grid Cell 
        self.grid_visited = 0.01 # Visited Value of Grid Cell
        
        # Shape Values 
        self.found_shapes = [] # All objects on map that have been located + Location values
        self.shapes_of_interest_visited = [] # All object of interest that have been visited
        self.shaped_of_interest_not_visited = [] # All objects of interest that have not been visited
        # Grid Map
        self.grid_map = None # The Occupancy Grid Map


        #Positional data
        self.p_origin = world_origin # World Origin (X, Y, Yaw) of where world starts on grid
        self.current_position = Position() # Current Global Position 
        self.grid_position = Position() # Current Grid Position

        #Map Settings
        #meta data 
        self.map_resolution = resolution # Map Resoltion
        self.map_size = size # Map Size 


        #Laser Data
        self.angle_min = angle_min # Minimum Laser Scanner Position
        self.angle_max = angle_max # Maximum Laser Scanner Position 
        self.angle_increment = angle_increment # Incriment between Laser Scanner Points
        self.range_max = range_max # Maximum Range of Laser Scanner

        

    # Sets Grid Map from file / map server
    # Converts a world coordinate (px, py) -> integer grid coordinates (gx, gy)
    # X and Y are flipped as Occupancy Grid is flipped 
    def to_grid(self, px, py):
        gy = (px-self.p_origin.x) / self.map_resolution # grid y location
        gx = (py-self.p_origin.y) / self.map_resolution # grid x location
        return gx, gy
    
    # Converts integer grid coordinates (gx, gy) to world coordinates (px, py)
    def to_world(self, gx, gy):
        py = gx * self.map_resolution + self.p_origin.x
        px = gy * self.map_resolution + self.p_origin.y      
        return px, py

    # Checks if shape has already been added to the map
    def is_already_added(self, shape, grid_position):
        position = grid_position.x * self.map_size[0] + grid_position.y
        position = int(position)
        if self.grid_map[position] == 100 or not (self.grid_map[position] < 2 and self.grid_map[position] >= 0):
            return True
        return False

    def is_already_added_pos(self, position):
        if self.grid_map[position] == 100 or not (self.grid_map[position] < 2 and self.grid_map[position] >= -1):
            return True
        return False

    # Adds Shape to the Map based on central grid co-ordinate
    def add_shape(self, shape):
        added = True
        # X axis Shape start and end points
        start_x = self.grid_position.x - (shape.length // 2)
        end_x = self.grid_position.x + (shape.length // 2)
        start_x = int(start_x)
        end_x = int(end_x)
        

        # Y Axis Shape Start and end Points
        start_y = self.grid_position.y - (shape.width // 2)
        end_y = self.grid_position.y + (shape.width // 2)
        start_y = int(start_y)
        end_y = int(end_y)

        if self.is_already_added_pos(start_x) or self.is_already_added_pos(end_x) or self.is_already_added_pos(start_y) or self.is_already_added_pos(end_y):
            return False

        # X Axis Mask Values
        mask_start_x = self.grid_position.x - (shape.mask_length // 2)
        mask_end_x = self.grid_position.x + (shape.mask_length // 2)
        mask_start_x = int(mask_start_x)
        mask_end_x = int(mask_end_x)

        # Y Axis Mask Values
        mask_start_y = self.grid_position.y - (shape.mask_width // 2)
        mask_end_y = self.grid_position.y + (shape.mask_width // 2)
        mask_start_y = int(mask_start_y)
        mask_end_y = int(mask_end_y)

        # if self.is_already_added_pos(mask_start_x) or self.is_already_added_pos(mask_end_x) or self.is_already_added_pos(mask_start_x) or self.is_already_added_pos(mask_end_x):
        #     return False
        for i in range(start_y, end_y):
            for j in range(start_x, end_x):
                position = j * self.map_size[0] + i
                if self.is_already_added_pos(position):
                    return False



        # Iterate over the rows and columns that the square will occupy
        for i in range(start_y, end_y):
            for j in range(start_x, end_x):
                position = j * self.map_size[0] + i

                if self.grid_map[position] != self.grid_occupied:
                    self.grid_map[position] = shape.map_value
        


        
        count = 0 
        # Add values to mask array, change values on map to mask value of shape
        for i in range(mask_start_y, mask_end_y):
            for j in range(mask_start_x, mask_end_x):
                position = j * self.map_size[0] + i
                if self.grid_map[position] != self.grid_occupied and self.grid_map[position] != shape.map_value:
                    
                    self.grid_map[position] = shape.mask_value




        point_1 = Position()
        x, y = self.to_world(self.grid_position.x , mask_start_y)
        point_1.x = x
        point_1.y = y
        if shape.type_value == 1:
            point_1.yaw = np.radians(0)
        else:
            point_1.yaw = np.radians(90)
        point_2 = Position()
        x, y = self.to_world(mask_start_x , self.grid_position.y)
        point_2.x = x
        point_2.y = y
        if shape.type_value == 1:
            point_2.yaw = np.radians(90)
        else:
            point_2.yaw = np.radians(0)
        point_3 = Position()
        x, y = self.to_world(mask_end_x,self.grid_position.y)
        point_3.x = x
        point_3.y = y
        if shape.type_value == 1:
            point_3.yaw = np.radians(90)
        else:
            point_3.yaw = np.radians(180)
        point_4 = Position()
        x, y = self.to_world(self.grid_position.x , mask_end_y)
        point_4.x = x
        point_4.y = y
        if shape.type_value == 1:
            point_4.yaw = np.radians(0)
        else:
            point_4.yaw = np.radians(-90)

        visit_points = [point_1, point_2, point_3, point_4]
        if shape.type_value == 2 or shape.type_value== 3:
            is_obj_interest = True 
        else:
            is_obj_interest = False
        shape_pos = ShapePosition(self.shape_id, is_obj_interest, False, visit_points)
        self.found_shapes.append(shape_pos)
        self.shape_id += 1

        return True
        # Set Visitable Points
        # Sets up to 4 visitable Points 

                

        # Iterate over rows and columns that the square mask will occupy

    # Updates the map with a given object on the map if it has not already been added
    def update_object(self, world_position, shape, grid_map):
        self.grid_map = grid_map
        x, y = self.to_grid(world_position.x, world_position.y)

        self.grid_position.x = x
        self.grid_position.y = y
        self.grid_position.yaw = world_position.yaw
        if not self.is_already_added(shape, self.grid_position):
            added = self.add_shape(shape)
            if added:
                rospy.loginfo("Added Shape {}, at location x:{} y:{}".format(shape.type_value, x ,y))
            else:
                rospy.loginfo("Shape already added to position")
        else:
            rospy.loginfo("Shape already added to position")




class GridMapController:
    def __init__(self):
        self.rate = rospy.Rate(30)
        # Map Values 
        self.grid_map = None # Occupancy Grid Map 
        self.occupancy_grid_msg = OccupancyGrid() # Message Used to publish Occupancy Grid
        self.map_resolution = None # Map Resolution 
        self.map_size = None # Map size [Length, Width]
        self.world_origin = Position() # World Origin 
        self.world_position = Position() # Current World Position
        self.world_position_queue = []
        self.odom_data = Position() # Odometry Data
        self.perception_data = PerceptionData()
        # Shapes 
        self.shapes = [
            Shape("Blue Tile", 1, 10, 15, 10, 15, 90, 20), # Tile
            Shape("Red Hydrant", 2, 6, 15, 6, 15, 81, 11), # Hydrant
            Shape("Green Box", 3, 12, 20, 12, 20, 80, 10), # Green box
            # Shape(4, 15, 15, 92), # bbq
            # Shape(5, 2, 2, 91), # beer can 
        ]

        self.map_points_msg = MapPoints()
        self.scan_type = 1 #1 Noscan 2 Blue tiles 3 Full
        

        # Publishers 
        self.map_pub = rospy.Publisher('map2', OccupancyGrid, queue_size=10) # Publishes Map topic 
        self.map_points_pub = rospy.Publisher('map_points', MapPoints, queue_size=10)
        # Services Servers 
        rospy.wait_for_service('static_map')
        self.get_map_client = rospy.ServiceProxy('static_map', GetMap) # Get map service client
        self.MapScan = rospy.Service('map_scan_change', MapScan, self.map_scan_cb)
        # Services Clients 

        # Subs
        self.laser_sub = rospy.Subscriber("scan", LaserScan, self.laser_scan_cb) # Subscribes to Laser Topic
        self.perception_sub = message_filters.Subscriber('perception_node', PerceptionNode) # Subscribes to Perception Topic
        self.localization_sub = rospy.Subscriber('location_pub', Localization, self.localization_cb) # Subscribes to Localization/Location Topic 
        self.tf_sub = rospy.Subscriber('tf',tfMessage, self.tf_cb)
        self.odom_sub = message_filters.Subscriber('odom', Odometry)
        # Control Values 
        self.perception_active = False # If the Perception Callback should run
        self.laser_scanner_active = False  # If the Laser Scanner Callback should run 
        self.is_gridmap_init = False # if the GridMap Object is initialized
        ts = message_filters.TimeSynchronizer([self.perception_sub, self.odom_sub], 10)
        ts.registerCallback(self.synced_cb)

    ## Callbacks
    def get_map_points_cb(self, req):
        message = ScanPoints()
        map_points = []
        for i in self.gridmap.found_shapes:
            point = MapPoints()
            point.object_id = i.object_name
            point.is_object_of_interest = i.is_object_of_interest
            point.visit_points = i.object_visit_points 
            map_points.append(point)
        message.map_points = map_points
        return map_points

    def map_scan_cb(self, req):
        self.scan_type = req.scan 
        return 0

    ### Publishers 
    def map_pub_cb(self):
        self.rate.sleep()
        self.occupancy_grid_msg.data = self.grid_map
        self.map_pub.publish(self.occupancy_grid_msg)

    def map_points_pub_cb(self):
        self.rate.sleep()
        self.map_points_msg = MapPoints()
        self.map_points_msg.map_points = []
        self.map_points_msg.objects_detected = False
        if self.is_gridmap_init:
            if len(self.gridmap.found_shapes) >= 1:
                self.map_points_msg.objects_detected = True
                for point in self.gridmap.found_shapes:
                    add_point = MapPoint()
                    add_point.object_id = point.object_name
                    add_point.is_object_of_interest = point.is_object_of_interest
                    add_point.visit_points = point.object_visit_points
                    self.map_points_msg.map_points.append(add_point)
                
        self.map_points_pub.publish(self.map_points_msg)
                
            


    ### Services
    #### Map Retrieve 
    def get_map_cb(self):
        response = self.get_map_client()
        self.occupancy_grid_msg = response.map
        self.grid_map = np.array(self.occupancy_grid_msg.data)
        self.map_resolution = self.occupancy_grid_msg.info.resolution
        self.map_size = [self.occupancy_grid_msg.info.width, self.occupancy_grid_msg.info.height]
        quarternion = [self.occupancy_grid_msg.info.origin.orientation.x, self.occupancy_grid_msg.info.origin.orientation.y,\
                    self.occupancy_grid_msg.info.origin.orientation.z, self.occupancy_grid_msg.info.origin.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

        if yaw < 0:
            yaw = np.radians(180) + (np.radians(180)-(yaw*(-1)))

        
        self.world_origin.yaw = yaw
        self.world_origin.x = self.occupancy_grid_msg.info.origin.position.x
        self.world_origin.y = self.occupancy_grid_msg.info.origin.position.y

    ### Subscribers
    #### Laser Scan

    def laser_scan_cb(self, msg):
        if self.laser_scanner_active:
            if not self.is_gridmap_init:
                self.gridmap = GridMap(msg.angle_min, msg.angle_max, msg.angle_increment, msg.range_max,
                    self.world_origin, self.map_resolution, self.map_size
                    )
                self.is_gridmap_init = True
                self.perception_active = True
            self.laser_ranges = msg.ranges
    
    #### Localization
    def localization_cb(self, msg):
        if msg.is_localized:
            if not self.laser_scanner_active:
                self.laser_scanner_active = True
            # self.world_position.x = msg.location.x
            # self.world_position.y = msg.location.y
            # self.world_position.yaw = msg.location.yaw

        # TF call back which records odom tf data to create current pose 
    def tf_cb(self, msg_l):
        for i in msg_l.transforms:
            if i.child_frame_id == 'odom':
                msg = i.transform
                quarternion = [msg.rotation.x,msg.rotation.y,\
                            msg.rotation.z, msg.rotation.w]
                (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

                if yaw < 0:
                    #Converts yaw to 0-360 instead of 0-180, -180-0 
                    yaw = np.radians(180) + (np.radians(180)-(yaw*(-1)))
                #Sets Current Poses to custom odom class 

                self.odom_data.x = msg.translation.x
                self.odom_data.y = msg.translation.y
                self.odom_data.yaw = yaw



    def odom_cb(self,msg):
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        #if len(self.world_position_queue) == 4:
            #self.world_position_queue.pop(0)
        self.world_position = Position()
        self.world_position.yaw = self.odom_data.yaw + yaw
        self.world_position.x = self.odom_data.x + msg.pose.pose.position.x
        self.world_position.y = self.odom_data.y + msg.pose.pose.position.y
        #self.world_position_queue.append(world_position)

    #### Perception
    def perception_cb(self, msg):
        if self.perception_active:
            self.perception_data.objects_detected = msg.objects_detected
            self.perception_data.objects_of_interest = msg.objectOfInterest

    def synced_cb(self, msg_perception, msg_odom):  
        # self.rate.sleep()
        self.odom_cb(msg_odom)
        self.perception_cb(msg_perception)
        





    def map_updater(self):
        self.rate.sleep()
        if self.perception_active:
            #if len(self.world_position_queue) >= 4:
                #self.world_position = self.world_position_queue[3]
            if self.perception_data.objects_detected:
                perception_objects = self.perception_data.objects_of_interest
                position_save = Position()
                position_save.x = self.world_position.x
                position_save.y = self.world_position.y
                position_save.yaw = self.world_position.yaw
                for i in perception_objects:
                    if self.scan_type > 1:
                        if self.scan_type == 2:
                            rospy.loginfo("Current Location: x:{}, y:{}, yaw: {}".format(position_save.x, position_save.y, position_save.yaw))
                            rospy.loginfo("Handling Shape {}".format(i.object_type))
                            if i.object_type == 1:
                                position = Position()
                                angle = position_save.yaw + i.angle 
                                if not math.isnan(i.distance):
                                    if i.object_type == 3:
                                        distance = i.distance + 0.5
                                    else:
                                        distance = i.distance + 0.1
                                    position.x = position_save.x + (distance * np.cos(angle))
                                    position.y = position_save.y + (distance * np.sin(angle))
                                    rospy.loginfo("Object Location: x:{}, y:{}, yaw: {}".format(position.x, position.y , self.world_position.yaw))
                                    self.occupancy_grid = self.gridmap.update_object(position, self.shapes[i.object_type -1], self.grid_map)
                                self.map_pub_cb()
                        else:
                            position = Position()
                            angle = position_save.yaw + i.angle 
                            if not math.isnan(i.distance):
                                if i.object_type == 3:
                                    distance = i.distance + 0.5
                                else:
                                    distance = i.distance + 0.1
                                position.x = position_save.x + (distance * np.cos(angle))
                                position.y = position_save.y + (distance * np.sin(angle))
                                rospy.loginfo("Object Location: x:{}, y:{}, yaw: {}".format(position.x, position.y , self.world_position.yaw))
                                self.occupancy_grid = self.gridmap.update_object(position, self.shapes[i.object_type -1], self.grid_map)
                            self.map_pub_cb()

        




def main(): 
    rospy.init_node('map_server_two') 
    map_building = GridMapController()
    rospy.loginfo_once('Map Server 2 Node: Map Server 2 Node Ready!')
    map_building.get_map_cb()
    print("Map Server 2 Node: Map Params, Map Origin: x: {}, Map Origin y: {}, Map Origin yaw: {}, Map size: {}, Map Resolution: {}".format(map_building.world_origin.x, map_building.world_origin.y, map_building.world_origin.yaw, map_building.map_size, map_building.map_resolution))
    while not rospy.is_shutdown():
        map_building.map_updater()
        map_building.map_pub_cb()
        map_building.map_points_pub_cb() 
        pass
        #print("map x: {}, map y: {}, map yaw: {}".format(map_building.world_origin.x, map_building.world_origin.y, map_building.world_origin.yaw))




if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 