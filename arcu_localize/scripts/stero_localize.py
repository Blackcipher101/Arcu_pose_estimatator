#!/usr/bin/env python3
from numpy.lib.type_check import imag
import rospy
import message_filters
#from localizataion.msg import localizemsg
import cv2 as cv
import math
import tf2_msgs.msg
import numpy as np
import tf_conversions

import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import CompressedImage , CameraInfo
from cv_bridge import CvBridge
from utils import ARUCO_DICT, aruco_display
pub = None
def localize(ros_data,info_L):
    global pub
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image = cv.imdecode(np_arr, -1)
    #w=info_L.width
    a= np.array(info_L.K)
    callib_mat=np.reshape(a, (-1, 3))
    d= np.array([info_L.D])
    #print(d)
    #print(callib_mat)
    #f_xR=info_R.K[0]
    #fov_x = 2 * math.atan2( w, (2*f_x) )
    #d=(w/2)/math.tan(fov_x/2)
    
	# load the ArUCo dictionary, grab the ArUCo parameters, and detect
	# the markers
    print("Detecting '{}' tags....".format('DICT_5X5_100'))
    arucoDict = cv.aruco.Dictionary_get(ARUCO_DICT['DICT_4X4_100'])
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    arucoParams = cv.aruco.DetectorParameters_create()
    #corners, ids, rejected = cv.aruco.detectMarkers(image, arucoDict, parameters=arucoParams)
    #print(corners)
    l=0.005
    corners, ids, rejected_img_points = cv.aruco.detectMarkers(gray, arucoDict,parameters=arucoParams,
        cameraMatrix=callib_mat,
        distCoeff=d)
    """ broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "world"
    static_transformStamped.child_frame_id = "camera"

    static_transformStamped.transform.translation.x = float(0)
    static_transformStamped.transform.translation.y = float(0)
    static_transformStamped.transform.translation.z = float(0.5)

    quat = tf_conversions.transformations.quaternion_from_euler((-math.pi/2), 0, 0)
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]

    broadcaster.sendTransform(static_transformStamped) """
    #br = tf2_ros.TransformBroadcaster()
    makerSize=0.2
    for i in range(len(corners)):
        center_xL=(corners[i][0][0][0] + corners[i][0][2][0]) //2
        center_y=(corners[i][0][1][1] + corners[i][0][3][1]) //2
        rvec, tvec, markerPoints = cv.aruco.estimatePoseSingleMarkers(corners[i],makerSize , callib_mat,d)
        print("pose with respect to camera tralatation ",ids[i])
        print(tvec[0][0][0])
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "IMX219_left_link"
        t.child_frame_id = "marker"+str(ids[i][0])
        t.transform.translation.x = tvec[0][0][0]
        t.transform.translation.y = tvec[0][0][1]
        t.transform.translation.z = tvec[0][0][2]
        q = tf_conversions.transformations.quaternion_from_euler(rvec[0][0][0], rvec[0][0][1], rvec[0][0][1])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        tfm = tf2_msgs.msg.TFMessage([t])
        pub.publish(tfm)
        cv.aruco.drawDetectedMarkers(image, corners)
        cv.aruco.drawAxis(image, callib_mat,d, rvec, tvec, 0.01)
        
        #print(angleL)
        #print(angleR)
        
        
        

        
       
    cv.imshow('L',image) #debug output
    #cv.imshow('R',detected_markersR)
    #print(camera_info)
    cv.waitKey(20)
    



    


    



    



    
def talker():
   
        
    # Apply Perspective Transform Algorithm
    #Caliibratation code
    #matrix = cv.getPerspectiveTransform(pts1, pts2)
    #result = cv.warpPerspective(old_frame, matrix, (600, 600))
    #res = cv.warpPerspective(old_frame, matrix, (600, 280))
    
    
    rospy.init_node('listener', anonymous=True)

    global pub
    image_sub = message_filters.Subscriber('/camera1/compressed', CompressedImage)
    info_sub = message_filters.Subscriber('/camera1/camera_info', CameraInfo)
    pub=rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)

    ts = message_filters.TimeSynchronizer([image_sub, info_sub], 10)
    ts.registerCallback(localize)
    rospy.spin()
    rospy.spin()
    rospy.spin()
    

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass