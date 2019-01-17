#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from velodyne_msgs.msg import VelodyneScan


import sensor_msgs.point_cloud2 as pc2
#import tf2_ros
#import tf2_py as tf2
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

def callback(data):
    temp = Imu()
    temp.angular_velocity.x
    temp.header.frame_id
    rospy.loginfo(rospy.get_caller_id() + "I heard %s",str(data.angular_velocity.x))
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(data.header.frame_id))

def callback1(data):
    tt =  VelodyneScan()


    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(data.packets.pop(0)))
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/tfsensors/imu1", Imu, callback)
    rospy.Subscriber("/velodyne_packets", VelodyneScan, callback1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
