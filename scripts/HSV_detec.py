#!/usr/bin/env python
import math, rospy
from geometry_msgs.msg import Twist
import cv2
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, PointCloud2
import ros_numpy

class Tracking:
    def __init__(self):
        #init variables for command tracker
        self.msg_twist=Twist()
        self.time_limit_cmd=0
        self.time_cmd=0

        #point cloud init to None
        self.point_cloud=None

        # HSV space for image filter
        self.lo_or=np.array([10,200,80])
        self.hi_or=np.array([20, 255,255])
        self.color_info=np.array([0, 165,255])

        # get the raw image from camera
        rospy.Subscriber("/head_camera/rgb/image_raw", Image, self.image_proc)
        # get the point cloud from the stereo camera
        rospy.Subscriber("/head_camera/depth_downsample/points", PointCloud2 , self.get_cloud)
        # init publisher for navigation command of tracker
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    
    #function for command creation from pointcloud data
    def goal_set(self ,goal_x ,goal_y):
        rospy.logwarn(str(goal_x)+" "+str(goal_y))

        # Calculation distance between tracker and guider the -1 take into account the height of the robot
        d=goal_x**2+goal_y**2-1

        # Get angle between the robot angle and tracker to guider vector 
        alpha=math.atan(goal_y/goal_x)
        # Calculate radius of the circle trajectory to goal
        R=d/(2*math.cos(abs(alpha)))
        # Fixed linear speed equivalent to turtlebot maximum speed
        v=0.37
        # Calculate the angular speed
        w=np.sign(alpha)*v/R

        # Calculate time from the trajectory length
        length=R*abs(math.pi-(2*alpha))
        # Critical distance between fetch and guider robot
        cri_dist=1
        # Calculate time of travel minus the critical distance
        self.time_limit_cmd= ((length-cri_dist)/v)

        # Define command twist for tracker 
        self.msg_twist.linear.x=v
        self.msg_twist.angular.z=-w
        rospy.loginfo(str(length)+" "+str(w)+" "+str(R)+" "+str(self.time_limit_cmd)+" "+str(math.pi-(2*alpha)))

    # Display info on camera output
    def display_info(self,rec,x,y):
        cv2.rectangle(self.frame, (int(rec[0]), int(rec[1])), (int(rec[0])+int(rec[2]), int(rec[3])+int(rec[1])), self.color_info, 2)
        cv2.circle(self.frame, (int(x), int(y)), 5, self.color_info, 10)
        cv2.line(self.frame, (int(x), int(y)), (int(x)+150, int(y)), self.color_info, 2)
        cv2.putText(self.frame, "Guider", (int(x)+10, int(y) -10), cv2.FONT_HERSHEY_DUPLEX, 1, self.color_info, 1, cv2.LINE_AA)
    
    # Display camera feed mask and stencil
    def display_images(self):
        stencil=cv2.bitwise_and(self.image, self.frame, mask= self.mask)
        cv2.imshow('Camera', self.frame)
        cv2.imshow('Mask', self.mask)
        cv2.imshow('Stencil', stencil)
        cv2.waitKey(3)

    #function for commanding the robot
    def cmd_robot(self):
        #rospy.logwarn(self.time_limit_cmd)

        # If this variable is positive then we set a time stamp at which the robot will stop to move
        if self.time_limit_cmd>0:
            rospy.logerr(self.time_limit_cmd)
            self.time_cmd=self.time_limit_cmd+rospy.get_rostime().to_sec()
            self.time_limit_cmd=0

        rospy.logerr(self.time_cmd)
        rospy.logerr(rospy.get_rostime().to_sec())

        # if the current time is lower than the time stamp set earlier we publish the command
        if self.time_cmd>rospy.get_rostime().to_sec():
            self.cmd_pub.publish(self.msg_twist)
        else: # else we tell the robot to stop
            stop_robot_cmd=Twist()
            self.cmd_pub.publish(stop_robot_cmd)

    # function for image processing HSV filter
    def image_proc(self,data):
        # Put the RGB image to BGR and create a HSV image
        self.frame = CvBridge().imgmsg_to_cv2(data, "bgr8")
        HSV_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)

        # preprocessing of the image
        self.image=cv2.blur(self.frame, (7, 7))
        self.mask=cv2.inRange(HSV_frame, self.lo_or, self.hi_or)
        self.mask=cv2.erode(self.mask, None, iterations=2)
        self.mask=cv2.dilate(self.mask, None, iterations=2)
        elements=cv2.findContours(self.mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

        # Choose the smallest element in the image
        size=999999
        final_ele=None
        for e in elements:
            rec=cv2.boundingRect(e)#find the rectangle that frames the element
            temp=rec[2]*rec[3] #calculate the area of the frame rectangle
            if size>temp:
                size=temp
                final_ele=e
        # If an element is defined and a point cloud is defined
        if final_ele is not None and self.point_cloud is not None:
            rec=cv2.boundingRect(final_ele)
            # get center coordinate of the element in the image
            x=int(rec[0]+(rec[2])/2)
            y=int(rec[1]+(rec[3])/2) 
        # Remap coordinate in the point cloud
            coord_robot=self.point_cloud[y/4,x/4] 

            if coord_robot[0]==float("nan"): # If the coordinate exist in the point cloud 
                for i in range(16): # We check neighbouring cases to find a replacement
                    if self.point_cloud[y/4+i%4,x/4+i//4][0]!=float("nan"):
                        coord_robot=self.point_cloud[y/4+i%4,x/4+i//4]
                        break
            # Call the goal_set function with the x y coordinate of the guider from the tracker
            self.goal_set(coord_robot[0],coord_robot[2]) 
            # send the command to a behaviour funcion
            self.cmd_robot()

            self.display_info(rec,x,y)
        else: # if the guider position is not defined we send a simple rotation to check if the guider is around

            self.msg_twist.linear.x=0
            self.msg_twist.angular.z=np.sign(self.msg_twist.angular.z)*0.7 # we rotate in the way the guider was last
            self.cmd_pub.publish(self.msg_twist)

            
        
        self.display_images() 

        self.rate.sleep() # the process hold for a given time
     
    # subscriber to point cloud topic
    def get_cloud(self,data):
        self.point_cloud = ros_numpy.point_cloud2.pointcloud2_to_array(data)
    
# Main function
if __name__=="__main__":    
    rospy.init_node('tracking', anonymous=True) #init image_proc node
    
    tracker=Tracking()

    tracker.rate=rospy.Rate(20)
    rospy.spin()  # spin() enter the program in a infinite loop