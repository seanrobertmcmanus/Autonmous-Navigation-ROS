#! /usr/bin/env python

#Description 
# The Turtle Command Node
# For use of 
#   - Turtle3 WAFFLE
# Uses data from:
#   - Receive Commands and data from other nodes 
#   - Fill in later 
# Useage:
#   - From received data determines behaviour type of robot in current state 
#   - Publishes control commands to a topic for use by the Turtle Control Node

#Imports
#ROS Librarys 
import rospy

#MSG Librarys 
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from minitask5.msg import *
from minitask5.srv import * 
from minitask5.srv import MoveBaseDone as move_base_srv

from tf.msg import tfMessage

#Service Librarys
from nav_msgs.srv import GetMap, GetMapRequest
from std_srvs.srv import Empty

#Other Librarys 
import numpy as np
import tf 
import math

class Position:
    def __init__(self):
        self.x = 0 # X Position
        self.y = 0 # Y Position
        self.yaw = 0 # Yaw Orientation

class Covariance:
    def __init__(self, x, y, yaw):
        self.x_accuracy = x, #[0]
        self.y_accuracy = y, #[1]
        self.yaw_accuracy = yaw #[7]
        self.accuracy = 100
        self.number_of_poses = 0

    def add_values(self, x, y, yaw):
        self.x_accuracy = np.sqrt(x)
        self.y_accuracy = np.sqrt(y)
        self.yaw_accuracy = np.sqrt(yaw)
        self.accuracy = np.sqrt((x + y + yaw) /3)

class ObectsOfInterest:
    def __init__(self, object_type, angle, distance):
        self.object_type = object_type
        self.angle = angle 
        self.distance = distance

class PerceptionData:
    def __init__(self):
        self.objects_detected = False
        self.objects_of_interest = []





class Controller:
    def __init__(self):
        self.rate = rospy.Rate(20)
        ## Planning Vals 
        self.current_movement = 0 # [General Movemnt, ... ]

        # Movement Node Values 
        self.general_movement_active = False # Value 1
        self.move_base_active = False # Value 2
        self.beaconing_active = False # Value 3
        self.spin_active = False # Value 4
        self.full_stop_active = False # Value 5

        # Localization Values 
        self.is_localized = False

        # Map Server Nodes
        self.blue_tile_scan_active = False
        self.full_scan_active = False

        # Perception Values 
        self.goal_reached = 0

        ## Stored Vals 
        # Positional 
        self.current_location = Position() 
        self.localization_coveriance = Covariance(0,0,0)
        self.odom_data = Position()

        # Perception
        self.perception_data = PerceptionData()
        self.perception_active = False

        # Subs
        self.amcl_pwc_sub = rospy.Subscriber('gmcl_pose', PoseWithCovarianceStamped, self.gmcl_pwc_cb)
        self.tf_sub = rospy.Subscriber('tf',tfMessage, self.tf_cb)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_cb)
        self.perception_sub = rospy.Subscriber('perception_node', PerceptionNode, self.perception_cb)
        # pubs 
        self.location_pub = rospy.Publisher('location_pub', Localization, queue_size=10)
        # Services 
        s = rospy.Service('move_base_srv', move_base_srv, self.move_base_done_cb)
        rospy.wait_for_service('movement_code')
        self.movement_req_client = rospy.ServiceProxy('movement_code', GeneralMovement)
        rospy.wait_for_service('global_localization')
        self.amcl_global_localization_client = rospy.ServiceProxy('global_localization', Empty)
        self.map_scan_change_client = rospy.ServiceProxy('map_scan_change', MapScan)
        



    # Call Backs 
    def change_map_scan(self, map_scan_val):
        #call the handle and give it the code as argument. It will return the desired code
        self.map_scan_change_client(int(map_scan_val))
    ## Pubs 
    def localization_pub(self):
        localization_message = Localization()
        localization_message.is_localized = self.is_localized
        localization_message.localization_accuracy = self.localization_coveriance.accuracy
        localization_message.location.x = self.current_location.x
        localization_message.location.y = self.current_location.y
        localization_message.location.yaw = self.current_location.yaw 
        self.location_pub.publish(localization_message)
    ## Subs 
    def gmcl_pwc_cb(self, msg):

        # if yaw < 0:
        #     yaw = np.radians(180) + (np.radians(180)-(yaw*(-1)))
        self.localization_coveriance.add_values(msg.pose.covariance[0],
                                                msg.pose.covariance[1],
                                                msg.pose.covariance[35]  
                                                    )
    # Odom CB
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
    # TF Call back
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
    # Perception CB
    def perception_cb(self, msg):
        self.perception_data.objects_detected = msg.objects_detected
        self.perception_data.objects_of_interest = msg.objectOfInterest
    
    def move_base_done_cb(self,msg):
        #do something with this!
        self.goal_reached = msg.done
        return 0

    ## Service Requests 
    # Send Movement Data
    def move_request(self, new_code_to_send):
            try:
                #call the handle and give it the code as argument. It will return the desired code
                current_code = self.movement_req_client(new_code_to_send)
                print("Current Code {}".format(current_code))
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

    # Send Global Localization 
    def global_localization(self):
        response = self.amcl_global_localization_client()
        rospy.loginfo("Localization State: Beggining Global Localization Process")

    # Pubs 

    # Functions
    def set_all_movement_false(self):
        self.general_movement_active = False # Value 1
        self.move_base_active = False # Value 2
        self.beaconing_active = False # Value 3
        self.spin_active = False # Value 4
        self.full_stop_active = False # Value 5

    def set_all_scan_false(self):
        self.blue_tile_scan_active = False # Value 1
        self.full_scan_active = False # Value 2


    def is_infront_blue_tile(self):
        if self.perception_data.objects_detected:
            for i in self.perception_data.objects_of_interest:
                #if not math.isnan(i.distance):
                if i.object_type == 1 and (i.distance < 0.5 or math.isnan(i.distance)):
                    rospy.loginfo("Controller Node: Infront of blue tile")
                    return True
        return False

def main():
    rospy.init_node('controll_node')
    controller = Controller()
    rospy.loginfo_once('Controller Node Ready!')
    controller.global_localization()
    # Set Movement State to General Movement
    controller.current_movement = 1
    controller.set_all_movement_false()
    controller.general_movement_active = True
    controller.move_request(controller.current_movement)
    controller.localization_coveriance.accuracy = 1000
    controller.is_localized = False
    while not rospy.is_shutdown():
        controller.localization_pub()
        #rospy.loginfo_once('Localization State: Current State: {}, Localization Accuracy: '.format(localization.state_dict[localization.state]))
        # If Not Localized trigger Global Localization
        if controller.localization_coveriance.accuracy < 0.18 and controller.general_movement_active:
            print("Localization Accuracy {}".format(controller.localization_coveriance.accuracy))
            controller.is_localized = True

        if controller.is_localized:
            # Test 
            # If Infront of a blue tile do full spin 
            if (not controller.spin_active) and (controller.is_infront_blue_tile == True):
                rospy.loginfo("Controller Node: Infront of blue tile activating Spin and mapping")
                controller.current_movement = 4
                controller.set_all_movement_false()
                controller.spin_active = True
                controller.move_request(controller.current_movement)
                controller.set_all_scan_false()
                controller.blue_tile_scan_active = True
                controller.change_map_scan(2)

            # Test 
            # If Localized and General Movement is active do a full scan
            # if controller.is_localized and controller.general_movement_active:
            #     rospy.loginfo("Controller Node: Localized entering spin scan")
            #     controller.current_movement = 4
            #     controller.set_all_movement_false() 
            #     controller.spin_active = True 
            #     controller.move_request(controller.current_movement)
            #     controller.set_all_scan_false()
            #     controller.full_scan_active = True
            #     controller.change_map_scan(3)

            # If Full scan is active wait 30 seconds then disable and activate move_base and blue tile 
            if controller.full_scan_active:
                controller.current_movement = 2
                rospy.sleep(30)
                controller.set_all_movement_false()
                controller.set_all_scan_false()
                controller.blue_tile_scan_active = True 
                controller.change_map_scan(2)
                controller.move_base_active = True 
                controller.move_request(2)
                while controller.goal_reached == 0:
                    if (not controller.spin_active) and (controller.is_infront_blue_tile == True):
                        controller.set_all_movement_false()
                        controller.current_movement = 5
                        controller.full_stop_active = True
                        controller.move_request(5)
                        break
                    pass
                if controller.goal_reached == 2:
                    controller.current_movement = 4
                    controller.set_all_movement_false() 
                    controller.spin_active = True 
                    controller.move_request(controller.current_movement)
                    controller.set_all_scan_false()
                    controller.full_scan_active = True
                    controller.change_map_scan(3)
                # Beaconing
                elif controller.goal_reached == 3:
                    controller.current_movement = 3
                    controller.set_all_movement_false()
                    controller.set_all_scan_false()
                    controller.beaconing_active = True 
                    controller.change_map_scan(1)
                    controller.move_request(3) 


            # Wait until 2 or 3 is returned

            # if 2 Do full scan if 3 activate beaconing 

        # If arrived at object # Path Planning 
        

        # If move_base turn of scan 



if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 

