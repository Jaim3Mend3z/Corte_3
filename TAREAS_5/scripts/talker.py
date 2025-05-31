#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
import random

def talker():
    pub = rospy.Publisher('/robot_position', Point, queue_size=10)
    rospy.init_node('position_publisher', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown():
        point = Point()
        point.x = random.uniform(0, 10)
        point.y = random.uniform(0, 10)
        point.z = 0
        pub.publish(point)
        rospy.loginfo("Publicando posición: %s", point)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
