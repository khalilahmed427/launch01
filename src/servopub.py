#!/usr/bin/env/python3.8
import rospy
from std_msgs.msg import Int16
class pub:
    def __init__(self):
        rospy.init_node('servopubnode')
        pub=rospy.Publisher('joint_pos1',Int16,queue_size=10)
        rate=rospy.Rate(10)
        while not rospy.is_shutdown():
            sig=Int16()
            sig.data=2000
            pub.publish(sig)
            rate.sleep()
def main():
    start=pub()
if __name__=='__main__':
    main()
