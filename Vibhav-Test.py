#!/usr/bin/env pyton3
from cgi import print_directory 
from re import T 
import sys 
import rospy 
from std_msgs.msg import UInt32 


def main(): 
    DataToBeDecided=10 
    Temp_topic_Publisher =rospy.Publisher('/ros_temp',UInt32,queue_size=10) 
    Temp_published_data=UInt32()
    Temp_published_data.data=DataToBeDecided 
    while not rospy.is_shutdown(): 
        rospy.init_node('serial_node', anonymous=True) 
        Temp_topic_Publisher.publish(Temp_published_data.data) 
        print(Temp_published_data)
        rospy.sleep(0.2) 
        rospy.spin 
        

if __name__=='__main__': 
    try: main() 
    except rospy.ROSInterruptException: 
        pass 
    #node_name ="serial_node" #rospy.init_node(node_name) #health_monitor() #rospy.spin()