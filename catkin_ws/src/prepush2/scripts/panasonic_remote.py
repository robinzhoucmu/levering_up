#!/usr/bin/env python
import sys
import signal
import httplib
import rospy
from std_srvs.srv import Empty, EmptyResponse
from socket import error as socket_error
import xml.etree.ElementTree as ET
from std_msgs.msg import UInt16
conn = httplib.HTTPConnection("192.168.0.166", timeout = 10)
pub = rospy.Publisher('Vid_No', UInt16, queue_size=10)
vid_no=1
def init_connection():
    global conn
    try:
        conn.request("GET","/cam.cgi?mode=getinfo&type=capability")
        con_data = conn.getresponse()
    except socket_error as serr:
        rospy.logerr("Couldn't connect to camera. Is camera in connect to cell mode?")
        exit()
    con_data.read()
    conn.request("GET","/cam.cgi?mode=camcmd&value=recmode")
    con_data = conn.getresponse()
    con_data.read()
    rospy.loginfo("Connection to camera established")
    
def handle_rec_start(req):
    global conn
    global pub
    global vid_no
    pub.publish(vid_no)
    while True:
        rospy.sleep(.1)
        try:
            conn.request("GET","/cam.cgi?mode=camcmd&value=video_recstart")
            conn_data = conn.getresponse()
            conn_text = conn_data.read()
        except:
            conn.close()
            conn.connect()
            continue
        if(ET.fromstring(conn_text)[0].text != u'ok'):
            rospy.loginfo("Camera is recording already!")
        return EmptyResponse()
            
    
def handle_rec_stop(req):
    global conn
    global pub
    global vid_no
    while True:
        rospy.sleep(.1)
        try:
            conn.request("GET","/cam.cgi?mode=camcmd&value=video_recstop")
            conn_data = conn.getresponse()
            conn_text = conn_data.read()
        except:
            conn.close()
            conn.connect()
            continue
        
        if(ET.fromstring(conn_text)[0].text != u'ok'):
            rospy.loginfo("Oops ... camera wasn't recording!")
        else:
            vid_no = vid_no +1
        pub.publish(vid_no)
        return EmptyResponse()
        
    
    
def init_service_server():
    s_start = rospy.Service('start_rec',Empty,handle_rec_start)
    s_stop = rospy.Service('stop_rec',Empty,handle_rec_stop)
    
def signal_handler(signal, frame):
    conn.close()
    rospy.loginfo("Connection to camera closed")
    sys.exit(0)
    

if __name__ == "__main__":
    rospy.init_node('panasonic_remote')
    pub.publish(vid_no)
    signal.signal(signal.SIGINT, signal_handler)
    init_connection()
    init_service_server()
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(vid_no)
        rate.sleep()
        
    
