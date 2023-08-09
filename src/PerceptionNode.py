#! /usr/bin/env python

#Description
# The Perception Node can use data from Lidar and Kinect sensors to perceive the environment and detect objects of interest 
# It can publish the detected objects to a topic for use by other nodes
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from sensor_msgs.point_cloud2 import read_points
from minitask5.msg import PerceptionNode as perception_node_msg
from minitask5.msg import PerceptionObjects as PerceptionObjects
from std_msgs.msg import Header
import math

class Follower:
   def __init__(self):
        rospy.init_node('perception_node')
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("original", 1)
        self.perception_node_pub = rospy.Publisher('perception_node', perception_node_msg, queue_size=10)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.depth_sub = rospy.Subscriber('camera/depth/image_raw', Image, self.depth_callback)
        #self.cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist, queue_size=1)
        self.rate = rospy.Rate(10)
        self.twist = Twist()

        self.num_of_pillars = 0
        self.pillar_angles = []
        self.pillar_distances = []
        self.pillar_xys = []
        self.pillar_errors = []

        self.num_of_hydrants = 0
        self.hydrant_angles = []
        self.hydrant_distances = []
        self.hydrant_xys = []
        self.hydrant_errors = []
        
        self.num_of_tiles = 0
        self.tile_angles = []
        self.tile_distances = []
        self.tiles_xys = []
        self.tile_errors = []

        self.num_of_bbqs = 0
        self.bbq_angles = []
        self.bbq_distances = []
        self.bbq_xys = []
        self.bbq_errors = []

        self.num_of_cans = 0
        self.can_angles = []
        self.can_distances = []
        self.can_xys = []
        self.can_errors = []

        self.objectsOfInterest = []
        self.any_objects_detected = False
        self.header_seq = 0 

        #calcualte radians per pixel using focal length and camera fov.
        fx = 1206.8897719532354
        fov_x = 2 * math.atan(1920 / (2 * fx))
        self.radians_per_pixel = fov_x/1920
                   
   def depth_callback(self,msg):
      depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
      self.objectsOfInterest = []
      self.any_objects_detected = False


      # self.can_distances = []
      # self.can_angles = []
      # if len(self.can_xys)!=0:
      #    for x in range(self.num_of_cans):
      #       if depth_image[self.can_xys[x][1],self.can_xys[x][0]] > 5:
      #          continue
      #       self.can_distances.append(depth_image[self.can_xys[x][1],self.can_xys[x][0]])
      #       angle_rad = self.radians_per_pixel * self.can_errors[x]
      #       self.can_angles.append(angle_rad)
      #       objectOfInterest = PerceptionObjects()
      #       objectOfInterest.object_type = 5
      #       objectOfInterest.angle = angle_rad
      #       objectOfInterest.distance = depth_image[self.can_xys[x][1],self.can_xys[x][0]]
      #       self.objectsOfInterest.append(objectOfInterest)
      #       self.any_objects_detected = True

      # self.bbq_distances = []
      # self.bbq_angles = []
      # if len(self.bbq_xys)!=0:
      #    for x in range(self.num_of_bbqs):
      #       if depth_image[self.bbq_xys[x][1],self.bbq_xys[x][0]] > 5 or depth_image[self.bbq_xys[x][1],self.bbq_xys[x][0]] < 1:
      #          continue
      #       self.bbq_distances.append(depth_image[self.bbq_xys[x][1],self.bbq_xys[x][0]])
      #       angle_rad = self.radians_per_pixel * self.bbq_errors[x]
      #       self.bbq_angles.append(angle_rad)
      #       objectOfInterest = PerceptionObjects()
      #       objectOfInterest.object_type = 4
      #       objectOfInterest.angle = angle_rad
      #       objectOfInterest.distance = depth_image[self.bbq_xys[x][1],self.bbq_xys[x][0]]
      #       self.objectsOfInterest.append(objectOfInterest)
      #       self.any_objects_detected = True


      self.tile_distances = []
      self.tile_angles = []
      #if tiles in view then get distance and angle to each tile
      if len(self.tiles_xys)!=0:
         for x in range(self.num_of_tiles):
            if depth_image[self.tiles_xys[x][1],self.tiles_xys[x][0]] > 5:
               continue
            self.tile_distances.append(depth_image[self.tiles_xys[x][1],self.tiles_xys[x][0]])
            angle_rad = self.radians_per_pixel * self.tile_errors[x]
            self.tile_angles.append(-angle_rad)
            objectOfInterest = PerceptionObjects()
            objectOfInterest.object_type = 1
            objectOfInterest.angle = -angle_rad
            objectOfInterest.distance = depth_image[self.tiles_xys[x][1],self.tiles_xys[x][0]]
            self.objectsOfInterest.append(objectOfInterest)
            self.any_objects_detected = True
  
      self.hydrant_distances = []
      self.hydrant_angles = []
      #if hydrants in view then get distance and angle to each hydrant
      if len(self.hydrant_xys)!=0:
         for x in range(self.num_of_hydrants):
            if depth_image[self.hydrant_xys[x][1],self.hydrant_xys[x][0]] > 5:
               continue
            self.hydrant_distances.append(depth_image[self.hydrant_xys[x][1],self.hydrant_xys[x][0]])
            angle_rad = self.radians_per_pixel * self.hydrant_errors[x]
            self.hydrant_angles.append(-angle_rad)
            objectOfInterest = PerceptionObjects()
            objectOfInterest.object_type = 2
            objectOfInterest.angle = -angle_rad
            objectOfInterest.distance = depth_image[self.hydrant_xys[x][1],self.hydrant_xys[x][0]]
            self.objectsOfInterest.append(objectOfInterest)
            self.any_objects_detected = True

      
      self.pillar_distances = []
      self.pillar_angles = []
      #if pillars in view then get distance and angle to each pillar
      if len(self.pillar_xys)!=0:
         for x in range(self.num_of_pillars):
            if depth_image[self.pillar_xys[x][1],self.pillar_xys[x][0]] > 5:
               continue
            self.pillar_distances.append(depth_image[self.pillar_xys[x][1],self.pillar_xys[x][0]]-0.2) 
            angle_rad = self.radians_per_pixel * self.pillar_errors[x]
            self.pillar_angles.append(-angle_rad)
            objectOfInterest = PerceptionObjects()
            objectOfInterest.object_type = 3
            objectOfInterest.angle = -angle_rad
            objectOfInterest.distance = depth_image[self.pillar_xys[x][1],self.pillar_xys[x][0]]-0.2
            self.objectsOfInterest.append(objectOfInterest)
            self.any_objects_detected = True 

      
      # message_to_send = perception_node_msg()
      # message_to_send.objects_detected = self.any_objects_detected
      # message_to_send.objectOfInterest = self.objectsOfInterest
      # message_to_send.header = self.set_header_fields()
      # self.perception_node_pub.publish(message_to_send)
      print("Hydrants found: {}, Distances from hydrants: {} , Angles to hydrants: {}".format(self.num_of_hydrants,self.hydrant_distances,self.hydrant_angles))
      print("Tiles found: {}, Distances from Tiles: {} , Angles to Tiles: {}".format(self.num_of_tiles,self.tile_distances,self.tile_angles))
      print("Pillars found: {}, Distances from Pillars: {} , Angles to Pillars: {}".format(self.num_of_pillars,self.pillar_distances,self.pillar_angles))
      #print("BBQs found: {}, Distances from BBQs: {} , Angles to BBQs: {}".format(self.num_of_bbqs,self.bbq_distances,self.bbq_angles))
      #print("Cans found: {}, Distances from Cans: {} , Angles to Cans: {}".format(self.num_of_cans,self.can_distances,self.can_angles))
   
   def set_header_fields(self):
    self.header_seq += 1 
    msg = perception_node_msg()
    msg.header = Header()
    msg.header.seq = self.header_seq
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'perception'
    return msg.header

   def image_callback(self, msg):
      image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
      (h, w) = image.shape[:2]
      #image_resized = cv2.resize(image, (w/4,h/4))
      image_resized = image
      #find center point of image 
      (resized_h,resized_w) = image_resized.shape[:2]
      centerX = resized_w/2
   
      #converts image to hsv colour space.
      hsv = cv2.cvtColor(image_resized, cv2.COLOR_BGR2HSV)
      
      #sets bounds that define hydrant.
      lower1 = np.array([0, 240, 20])
      upper1 = np.array([1, 255, 255])
      #sets bounds that define pillar.
      lower_green = np.array([60, 25, 25])
      upper_green = np.array([61, 255, 255])
      #sets bounds that define tiles.
      lower_blue = np.array([119, 220, 25])
      upper_blue = np.array([120, 255, 255])
      #set bound that define bbq.
      lower_bbq = np.array([108, 110, 25])
      upper_bbq = np.array([114, 200, 255])
      #set bound that define can.
      lower_can = np.array([2, 200, 120])
      upper_can = np.array([7, 259, 255])

      
      # mask_can = cv2.inRange(hsv, lower_can, upper_can)
      # se_can_open = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
      # se_can_close = cv2.getStructuringElement(cv2.MORPH_RECT, (20,20))
      # mask_can = cv2.morphologyEx(mask_can, cv2.MORPH_OPEN, se_can_open)
      # mask_can = cv2.morphologyEx(mask_can, cv2.MORPH_CLOSE, se_can_close)
      # im2, contours_can, hierarchy = cv2.findContours(mask_can, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

     
      # mask_bbq = cv2.inRange(hsv, lower_bbq, upper_bbq)
      # se_bbq_open = cv2.getStructuringElement(cv2.MORPH_RECT, (15,15))
      # se_bbq_open2 = cv2.getStructuringElement(cv2.MORPH_RECT, (30,30))
      # se_bbq_close = cv2.getStructuringElement(cv2.MORPH_RECT, (125,125))
      # mask_bbq = cv2.morphologyEx(mask_bbq, cv2.MORPH_OPEN, se_bbq_open)
      # mask_bbq = cv2.morphologyEx(mask_bbq, cv2.MORPH_CLOSE, se_bbq_close)
      # mask_bbq = cv2.morphologyEx(mask_bbq, cv2.MORPH_OPEN, se_bbq_open2)
      # im2, contours_bbq, hierarchy = cv2.findContours(mask_bbq, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      
      #tile opening closing
      mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
      se_blue_open = cv2.getStructuringElement(cv2.MORPH_RECT, (4,4))
      se_blue_close = cv2.getStructuringElement(cv2.MORPH_RECT, (7,7))
      mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, se_blue_open)
      mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, se_blue_close)
      im2, contours_blue, hierarchy = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      
      #hydrant opening and closing
      mask_red = cv2.inRange(hsv, lower1, upper1)
      se_red_open = cv2.getStructuringElement(cv2.MORPH_RECT, (6,6))
      se_red_close = cv2.getStructuringElement(cv2.MORPH_RECT, (100,100))
      mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, se_red_open)
      mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, se_red_close)
      im2, contours_red, hierarchy = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
   
      #pillar opening closing
      mask_green = cv2.inRange(hsv, lower_green, upper_green)
      se_green_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
      mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, se_green_open)
      im2, contours_green, hierarchy = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      
      self.tiles_xys = []
      self.hydrant_xys = []
      self.pillar_xys = []
      self.bbq_xys = []
      self.can_xys = []
      self.tile_errors = []
      self.hydrant_errors = []
      self.pillar_errors = []
      self.bbq_errors = []
      self.can_errors = []
      self.num_of_tiles = 0
      self.num_of_hydrants = 0
      self.num_of_pillars = 0
      self.num_of_bbqs = 0
      self.num_of_cans = 0

      
      # if len(contours_can) != 0:
      #    cv2.drawContours(image_resized, contours_can, -1, (120,100,255), 2)
      #    self.num_of_cans = len(contours_can)
      #    for x_iter in range(self.num_of_cans):
      #       M = cv2.moments(contours_can[x_iter])
      #       cX = int(M["m10"] / M["m00"])
      #       cY = int(M["m01"] / M["m00"])
      #       cv2.circle(image_resized, (cX, cY), 5, (0, 0, 255), -1)
      #       self.can_xys.append([cX,cY])
      #       error = -(centerX - cX)
      #       self.can_errors.append(error)

      
      # if len(contours_bbq) != 0:
      #    cv2.drawContours(image_resized, contours_bbq, -1, (120,100,255), 2)
      #    self.num_of_bbqs = len(contours_bbq)
      #    for x_iter in range(self.num_of_bbqs):
      #       M = cv2.moments(contours_bbq[x_iter])
      #       cX = int(M["m10"] / M["m00"])
      #       cY = int(M["m01"] / M["m00"])
      #       cv2.circle(image_resized, (cX, cY), 5, (0, 0, 255), -1)
      #       self.bbq_xys.append([cX,cY])
      #       error = -(centerX - cX)
      #       self.bbq_errors.append(error)

      #if blue square detected
      if len(contours_blue) != 0:
         cv2.drawContours(image_resized, contours_blue, -1, (120,100,255), 2)
         self.num_of_tiles = len(contours_blue)
         for x_iter in range(self.num_of_tiles):
            M = cv2.moments(contours_blue[x_iter])
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(image_resized, (cX, cY), 5, (0, 0, 255), -1)
            self.tiles_xys.append([cX,cY])
            error = -(centerX - cX)
            self.tile_errors.append(error)

            
            # x_b,y_b,w_b,h_b = cv2.boundingRect(contours_blue[x_iter])
            # cX = x_b+w_b/2
            # cY = y_b
            # cv2.rectangle(image_resized,(x_b,y_b),(x_b+w_b,y_b+h_b),(0,0,255),2)
            # cv2.circle(image_resized, (cX, cY), 5, (0, 0, 255), -1)
            # error = -(centerX - cX)
            # self.tile_errors.append(error)
            # self.tiles_xys.append([cX,cY])

        
      #if hydrant detected
      if len(contours_red) != 0 :
         cv2.drawContours(image_resized, contours_red, -1, (120,100,255), 2)
         self.num_of_hydrants = len(contours_red)
         for x_iter in range(self.num_of_hydrants):
            M = cv2.moments(contours_red[x_iter])
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(image_resized, (cX, cY), 5, (0, 0, 255), -1)
            self.hydrant_xys.append([cX,cY])
            error = -(centerX - cX)
            self.hydrant_errors.append(error)
            
         

      #if green pillar detected.
      if len(contours_green) != 0:
         cv2.drawContours(image_resized, contours_green, -1, (120,100,255), 2)
         self.num_of_pillars = len(contours_green)
         for x_iter in range(self.num_of_pillars):
            M = cv2.moments(contours_green[x_iter])
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(image_resized, (cX, cY), 5, (0, 0, 255), -1)
            self.pillar_xys.append([cX,cY])
            error = -(centerX - cX)
            self.pillar_errors.append(error)
        
      # image_show = cv2.resize(image_resized, None, fx=0.4, fy=0.4, interpolation=cv2.INTER_LINEAR)
      
      # cv2.imshow("original", image_show)
      cv2.waitKey(3)
   
   def run(self):
      while not rospy.is_shutdown():
         message_to_send = perception_node_msg()
         message_to_send.objects_detected = self.any_objects_detected
         message_to_send.objectOfInterest = self.objectsOfInterest
         message_to_send.header = self.set_header_fields()
         self.perception_node_pub.publish(message_to_send)
         self.rate.sleep()

if __name__ == '__main__':
    try:
      
      follower = Follower()
      follower.run()
      rospy.spin()
    except rospy.ROSInterruptException:
        pass
