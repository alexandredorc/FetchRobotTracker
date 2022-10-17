#!/usr/bin/env python
from ast import Num
import sys, math, rospy
from geometry_msgs.msg import Twist, PoseStamped
import cv2
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf
import ros_numpy

def goal_set(goal):
    global tracker_pose
    rospy.logwarn(goal)#   d/2cos(a)
    nav_goal = PoseStamped()
    nav_goal.header.frame_id = 'map'
    rospy.loginfo(str(math.cos(tracker_pose.orientation.z)*goal[2]))
    rospy.loginfo(str(math.sin(tracker_pose.orientation.z)*goal[0]))
    nav_goal.pose.position.x = tracker_pose.position.x+math.cos(tracker_pose.orientation.z)*goal[2]+math.sin(tracker_pose.orientation.z)*goal[0]
    nav_goal.pose.position.y = -tracker_pose.position.y+math.sin(tracker_pose.orientation.z)*goal[2]-math.cos(tracker_pose.orientation.z)*goal[0]
    nav_goal.pose.orientation.z = 0
    nav_goal.pose.orientation.w = 1
    nav_pub.publish(nav_goal)
    goal=[0]
    rospy.loginfo("going to pose "+str(nav_goal.pose.position))

def realCoor( x,y,dist ):
    angle=43.5*(x-640)/640
    angle=angle*math.pi/180
    return [math.cos(angle) * dist, math.sin( angle ) * dist-35]


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

def image_proc(data):
    
    global frame,image,mask
    global depth_data
    global timeStamp
    global num
    timeStamp= data.header.stamp

    frame = bridge.imgmsg_to_cv2(data, "bgr8")

    
    image=cv2.blur(frame, (7, 7))
    mask=cv2.inRange(image, lo_or, hi_or)
    mask=cv2.erode(mask, None, iterations=2)
    mask=cv2.dilate(mask, None, iterations=2)
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
        if 10<=num:
            num=0
            coord_robot=depth_data[y/4,x/4]

            goal_set(coord_robot)
            #rospy.logerr(coord_robot)
            
    num+=1    
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

    

    rospy.init_node('image_proc', anonymous=True)
    
    bridge = CvBridge()
    tfListener= tf.TransformListener()

    robots=[]
    confirm_robot=[]
    color=[35,230,230]
    color_info=(0, 125, 255)
    depth_data=None
    lo_or=np.array([0,0,150])
    hi_or=np.array([50, 150,255])
    global num
    num=100

    rospy.Subscriber("/head_camera/rgb/image_raw", Image, image_proc)
    rospy.Subscriber("/head_camera/depth_downsample/points", PointCloud2 , get_depth)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped , get_tracker_pose)
    nav_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped)
    
    rate=rospy.Rate(30) # spin() enter the program in a infinite loop
    rospy.spin() 