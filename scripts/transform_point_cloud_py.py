#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_py as tf2
from sensor_msgs.msg import Imu

from dynamic_reconfigure.server import Server
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from transform_point_cloud.cfg import LookupTransformConfig

import numpy as np


class TransformPointCloud:
    def __init__(self):
        self.config = None
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(12))
        self.tl = tf2_ros.TransformListener(self.tf_buffer)
        self.pub = rospy.Publisher("/point_cloud_neu", PointCloud2, queue_size=2)
        self.sub = rospy.Subscriber("/velodyne_points", PointCloud2,
                                    self.point_cloud_callback, queue_size=2)
        self.sub1 = rospy.Subscriber("/tfsensors/imu1", Imu,
                                    self.point_cloud_callback1, queue_size=2)
        self.dr_server = Server(LookupTransformConfig, self.dr_callback)

        self.x=0
        self.y=0
        self.z=0
        self.w=0
        self.tx=0
        self.ty=0
        self.tz=0
        self.vecPos=[(0.0,0.0,0.0)]
        self.vecVel=[(0.0,0.0,0.0)]
        self.vecAcc=[]
        self.tM=rospy.get_rostime().secs+rospy.get_rostime().nsecs*10**(-9)
        self.tP=0
        self.deltaT=0.1

    def dr_callback(self, config, level):
        self.config = config
        return self.config

    def point_cloud_callback(self, msg):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(msg.header.seq))
        lookup_time = msg.header.stamp# + rospy.Duration(self.config.offset_lookup_time)
        #target_frame = msg.header.frame_id if self.config.target_frame == "" else self.config.target_frame
        #source_frame = msg.header.frame_id if self.config.source_frame == "" else self.config.source_frame
        target_frame="velodyne"
        source_frame=target_frame
        try:
            trans = self.tf_buffer.lookup_transform(target_frame, source_frame, lookup_time,
                                                    rospy.Duration(self.config.timeout))
            #rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(msg.header.frame_id))
        except tf2.LookupException as ex:
            rospy.logwarn(str(lookup_time.to_sec()))
            rospy.logwarn(ex)
            return
        except tf2.ExtrapolationException as ex:
            rospy.logwarn(str(lookup_time.to_sec()))
            rospy.logwarn(ex)
            return
        trans.transform.rotation.x = self.x
        trans.transform.rotation.y = self.y
        trans.transform.rotation.z = self.z
        trans.transform.rotation.w = self.w
        #trans.transform.translation.x=trans.transform.translation.x+10

        cloud_out = do_transform_cloud(msg, trans)
        do_transform_cloud()
        trans.transform.

        self.pub.publish(cloud_out)
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", str(trans.transform.rotation.x))

    def point_cloud_callback1(self, msg):
        ttt=Imu()
        ttt.header.stamp.secs+ttt.header.stamp.nsecs*10**(-9)
        self.x = msg.orientation.x
        self.y = msg.orientation.y
        self.z = msg.orientation.z
        self.w = msg.orientation.w
        #self.vecAcc.
        rospy.loginfo(rospy.get_caller_id() + "I heard %s",str(self.vecAcc.__len__()))
        self.vecAcc.append([msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z])
        self.tP=(msg.header.stamp.secs + msg.header.stamp.nsecs * 10 ** (-9))
        self.deltaT=self.tP-self.tM
        print(self.vecVel.__len__())
        #print(self.vecVel[self.vecVel.__len__()-1][0])
        self.vecVel.append([self.vecVel[self.vecVel.__len__()-1][0]+msg.linear_acceleration.x*self.deltaT,
                            self.vecVel[self.vecVel.__len__()-1][1]+msg.linear_acceleration.y*self.deltaT,
                            self.vecVel[self.vecVel.__len__()-1][2]+msg.linear_acceleration.z*self.deltaT])
        self.vecPos.append([self.vecPos[self.vecPos.__len__()-1][0]+0.5*msg.linear_acceleration.x*self.deltaT**2+self.vecVel[self.vecVel.__len__()-1][0]*self.deltaT,
                            self.vecPos[self.vecPos.__len__()-1][1]+0.5*msg.linear_acceleration.y*self.deltaT**2+self.vecVel[self.vecVel.__len__()-1][1]*self.deltaT,
                            self.vecPos[self.vecPos.__len__()-1][2]+0.5*msg.linear_acceleration.z*self.deltaT**2+self.vecVel[self.vecVel.__len__()-1][2]*self.deltaT])
        self.tM=self.tP
        #print(msg.header.stamp.nsecs)
        print(self.deltaT,msg.linear_acceleration.x)
        print(msg.linear_acceleration.x*self.deltaT)
        #print(self.vecAcc[self.vecAcc.__len__()-1][2])




if __name__ == '__main__':
    rospy.init_node('transform_point_cloud')
    fkt = TransformPointCloud()
    rospy.spin()
