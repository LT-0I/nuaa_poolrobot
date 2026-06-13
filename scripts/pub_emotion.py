#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def talker():
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('emotion', String, queue_size=50)
    rate = rospy.Rate(50) # 10hz
    while not rospy.is_shutdown():
        hello_str = "happy"
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass