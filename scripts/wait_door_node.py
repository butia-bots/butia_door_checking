#! /usr/bin/env python
 
import rospy
from sensor_msgs.msg import LaserScan
import math
import statistics
from std_msgs.msg import Empty

range_calculation = False
range_position_min = 0
range_position_max = 0
closed_door_range = 0.00

def reading_gap(msg):
    global range_position_min
    global range_position_max
    global closed_door_range
    global range_calculation

    angle_gap = math.radians(rospy.get_param("/angle_gap"))
    
    if msg.angle_min < 0:
        angle_min = msg.angle_min*(-1)
    else:
        angle_min = msg.angle_min
    
    angle_max = msg.angle_max
    angle_range = angle_min + angle_max
    
    rpr = (len(msg.ranges)/angle_range) #readings per radians
    range_gap = rpr*angle_gap
    range_position_min = int((len(msg.ranges)/2)-range_gap)
    range_position_max = int((len(msg.ranges)/2)+range_gap)

    ranges = msg.ranges[range_position_min:range_position_max+1]
    ranges_median = statistics.median(ranges)
    closed_door_range = ranges_median
    range_calculation = True  

def callback(msg):
    if range_calculation == False:
        reading_gap(msg)
    ranges = msg.ranges[range_position_min:range_position_max+1]
    ranges_median = statistics.median(ranges)
    # print ('Closed door range:')
        
    if ranges_median > closed_door_range+2:
        pub.publish(Empty())
        rospy.loginfo('DOOR IS OPEN.')
    else:
        rospy.loginfo('DOOR IS CLOSED.')

def callback2(msg):
    global range_calculation
    range_calculation = False
    
    
rospy.init_node('verify_door_state')
sub = rospy.Subscriber('/scan', LaserScan, callback)
sub2 = rospy.Subscriber('/butia_navigation/analize_door', Empty, callback2)
pub = rospy.Publisher('/butia_navigation/wait_door', Empty, queue_size=1)
reading_gap
rospy.spin()
