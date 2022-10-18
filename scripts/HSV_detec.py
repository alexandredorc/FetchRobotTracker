#!/usr/bin/env python
import sys, math, rospy
from geometry_msgs.msg import Twist, PoseStamped
import cv2
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf
import ros_numpy

def goal_set(goal):
    global tracker_pose
    rospy.logwarn(goal)
    d=math.sqrt(goal[2]**2+goal[0]**2) #   d/2cos(a)
    alpha=math.atan(goal[0]/goal[2])
    R=d/(2*math.sin(abs(alpha)))
    v=0.5
    w=np.sign(alpha)*v/R
    global msg_twist
    global time_limit_cmd
    time_limit_cmd= (R*abs(alpha)-0.1)/(v*0.8)
    msg_twist.linear.x=v
    msg_twist.angular.z=-w
    cmd_pub.publish(msg_twist)
    rospy.loginfo(str(v)+" "+str(w)+" "+str(R)+" "+str(alpha))

def display_info(rec,x,y):
    cv2.rectangle(frame, (int(rec[0]), int(rec[1])), (int(rec[0])+int(rec[2]), int(rec[3])+int(rec[1])), color_info, 2)
    cv2.circle(frame, (int(x), int(y)), 5, color_info, 10)
    cv2.line(frame, (int(x), int(y)), (int(x)+150, int(y)), color_info, 2)
    cv2.putText(frame, "Object !!!", (int(x)+10, int(y) -10), cv2.FONT_HERSHEY_DUPLEX, 1, color_info, 1, cv2.LINE_AA)
            
def display_images():
    stencil=cv2.bitwise_and(image, frame, mask= mask)
    cv2.putText(frame, "Couleur: {0} {1} {2}".format(color[0],color[1],color[2]), (10, 30), cv2.FONT_HERSHEY_DUPLEX, 1, color_info, 1, cv2.LINE_AA)
    cv2.imshow('Camera', frame)
    cv2.imshow('Stencil', stencil)
    cv2.imshow('Mask', mask)
    cv2.waitKey(3)

def command_robot():
    global time_cmd
    global time_limit_cmd

    rospy.logwarn(time_limit_cmd)
    if time_limit_cmd>0:
        time_cmd=time_limit_cmd+rospy.get_rostime().to_sec()
        time_limit_cmd=0


    
    rospy.logerr(time_cmd)
    rospy.logerr(rospy.get_rostime().to_sec())
    if time_cmd>rospy.get_rostime().to_sec():
        global msg_twist
       
        
        cmd_pub.publish(msg_twist)
    else:
        stop_robot_cmd=Twist()
        cmd_pub.publish(stop_robot_cmd)

def image_proc(data):
    
    global frame,image,mask
    global depth_data
    global timeStamp
    timeStamp= data.header.stamp

    frame = bridge.imgmsg_to_cv2(data, "bgr8")
    HSV_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    image=cv2.blur(frame, (7, 7))
    mask=cv2.inRange(HSV_frame, lo_or, hi_or)
    mask=cv2.erode(mask, None, iterations=1)
    mask=cv2.dilate(mask, None, iterations=1)
    elements=cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    close_elem=[]
    
    for e in elements:
        rec=cv2.boundingRect(e)
        x=int(rec[0]+(rec[2])/2)
        y=int(rec[1]+(rec[3])/2)
        
        robot_shape=cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        close_elem.append(robot_shape)
    
    if len(close_elem) > 0 and depth_data is not None:
        
        x=int(rec[0]+(rec[2])/2)
        y=int(rec[1]+(rec[3])/2)

        display_info(rec,x,y)
    
        coord_robot=depth_data[y/4,x/4]

        goal_set(coord_robot)
    command_robot()
  
    display_images()

    rate.sleep()

     
def get_depth(data):
    global depth_data
    xyz_array = ros_numpy.point_cloud2.pointcloud2_to_array(data)
    depth_data = xyz_array

def get_tracker_pose(data):
    global tracker_pose
    tracker_pose=data.pose.pose
   
    
if __name__=="__main__":
    global msg_twist
    global time
    global time_limit_cmd
    global time_cmd

    time_limit_cmd=0
    msg_twist=Twist()
    
    rospy.init_node('image_proc', anonymous=True)
    time_cmd=0

    bridge = CvBridge()
    tfListener= tf.TransformListener()

    robots=[]
    confirm_robot=[]
    color=[35,230,230]
    color_info=(0, 125, 255)
    depth_data=None
    lo_or=np.array([10,150,150])
    hi_or=np.array([30, 255,255])

    rospy.Subscriber("/head_camera/rgb/image_raw", Image, image_proc)
    rospy.Subscriber("/head_camera/depth_downsample/points", PointCloud2 , get_depth)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped , get_tracker_pose)
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    
    rate=rospy.Rate(30) # spin() enter the program in a infinite loop
    rospy.spin() 