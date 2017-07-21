#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('jsk_laser')
from jsk_laser.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import Int8

class synchronizer:

    def __init__(self):
        #default we have 2....
        self.laser_num = rospy.get_param('~laser_num', 2)
        self.laser_name = []
        self.laser_syn_pub = []
        for i in range(0,self.laser_num):
            self.laser_name.append(rospy.get_param("~laser_name_"+str(i),"J"))
            rospy.Subscriber("/tiny_laserscan_"+self.laser_name[i], LaserScan, self.scancallback)
            self.laser_syn_pub.append(rospy.Publisher("/jsk_laser_serial_comm/laser_"+self.laser_name[i], Int8, queue_size = 0))

    def __del__(self):
        for i in range(0,self.laser_num):
            self.laser_syn_pub[i].publish(std_msgs.msg.Int8(88))
        rospy.sleep(1.0)
        rospy.loginfo("delete_node")

    def scancallback(self,data):
        index = data.header.frame_id.rindex("_")+1
        sender_name = data.header.frame_id[index]
        rospy.loginfo("receive data from:_ "+sender_name)
        for i in range(0,self.laser_num):
            if sender_name == self.laser_name[i]:
                self.laser_syn_pub[i].publish(std_msgs.msg.Int8(79))
            else:
                self.laser_syn_pub[i].publish(std_msgs.msg.Int8(88))


if __name__ == '__main__':
    rospy.init_node('laser_synchronizer')
    ex = synchronizer()
    try:
        rospy.spin()
    except rospy.ROSInterruptException: pass
