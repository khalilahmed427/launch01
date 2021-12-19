#!/usr/bin/env python3.8
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16
class joint:
    def __init__(self):
        rospy.init_node('jointstates')
        self.sub=rospy.Subscriber('/joint_states',JointState,self.callback)
        self.pub=rospy.Publisher('joint_pos1',Int16,queue_size=10)
        rospy.spin()
    def callback(self,data):
        a=int(abs(round((data.position[0]*180/3.1428)*11.375)))
        rospy.loginfo(a)
        self.pub.publish(a)
def main():
    test=joint()
if __name__=='__main__':
    main()