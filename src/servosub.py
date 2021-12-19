#!/usr/bin/env/python3.8
import rospy
from std_msgs.msg import Float64
class sub:
    def __init__(self):
        rospy.init_node('servosubnode')
        self.sub=rospy.Subscriber('servopub',Float64,self.callback)
        rospy.spin()
    def callback(self,data):
        rospy.loginfo(data.data)
def main():
    start=sub()
if __name__=='__main__':
    main()
