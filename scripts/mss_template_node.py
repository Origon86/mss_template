#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu

def callback(data):
    temp = Imu()
    temp.header.frame_id
    rospy.loginfo(rospy.get_caller_id() + "I heard %s",str(data.linear_acceleration.x))
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(data.header.frame_id))
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/tfsensors/imu1", Imu, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
