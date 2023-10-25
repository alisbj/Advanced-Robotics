#!/usr/bin/env python3

import rospy
import pandas as pd
from sensor_msgs.msg import Range

data = []
    
#! /usr/bin/env python3
 
import rospy
from sensor_msgs.msg import Range
 
def callback(msg):
    data.append(msg.range)
    print("[",len(data),"]",msg.range)

    if len(data) == 500:
        df = pd.read_csv('Sensor_Model_Data.csv')
        df['Data_37'] = data
        df.to_csv('Sensor_Model_Data.csv', index=False)
        
 
rospy.init_node('laser_data_capture')
sub = rospy.Subscriber('/vector/laser', Range, callback)
rospy.spin()
