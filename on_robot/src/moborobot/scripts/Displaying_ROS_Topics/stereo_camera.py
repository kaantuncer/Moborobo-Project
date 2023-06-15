#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
import sys
import time

font=cv2.FONT_HERSHEY_SIMPLEX
org=(10,30)
font_scale=0.6
color=(255,0,255)
thickness=2


bridge=CvBridge()
fps=0
counter=0
time1=time.time()
total_time=0
def func(ros_image):
    #print("Image received")
    global bridge
    global time1
    global time2
    global total_time
    global fps
    global counter
    try:
        cv_image=bridge.imgmsg_to_cv2(ros_image,"bgr8")
    except CvBridgeError as e:
        print(e)
    time2=time.time()
    total_time+=(time2-time1)
    time1=time2
    counter+=1
    if total_time>1:
        fps=counter
        counter=0
        total_time=0
    fps_str="FPS: "+ str(fps)
    cv2.putText(cv_image,fps_str,org,font,font_scale,color,thickness,cv2.LINE_AA)
    cv2.imshow("Stereo Camera",cv_image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown("Closing...")

def main(args):

    rospy.init_node("collector",anonymous=True)
    print("INITIALIZED")
    rospy.Subscriber("/zed2/zed_node/stereo/image_rect_color",Image,func)
    try:
        rospy.spin()

    except KeyboardInterrupt:
        print("Exiting...")
        cv2.destroyAllWindows()

if __name__=='__main__':
    main(sys.argv)