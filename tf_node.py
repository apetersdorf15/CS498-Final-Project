#!/usr/bin/env python
from scipy.spatial.transform import Rotation as R
import numpy as np
import rospy

# to get commandline arguments
import sys

# because of transformations
from tf.transformations import quaternion_from_euler, quaternion_matrix, quaternion_from_matrix
from tf import TransformListener

import tf2_ros
import geometry_msgs.msg
import std_msgs
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import MarkerArray, Marker
import message_filters
import csv

class OptimusPrime():
        def __init__(self):
                self.rate = rospy.Rate(10)
                rospy.Subscriber('/odom', Odometry, self.odom_callback)
                self.right_sub = rospy.Subscriber('/ns1/velodyne_points', PointCloud2, self.right_lidar_callback)
                self.left_sub = rospy.Subscriber('/ns2/velodyne_points', PointCloud2, self.left_lidar_callback)
                # self.traj_sub = rospy.Subscriber('/trajectory_node_list', MarkerArray, self.trajCallback)

                # self.ts = message_filters.TimeSynchronizer([self.right_sub, self.left_sub], 100)
                # self.ts.registerCallback(self.lidar_callback)

                self.lidar_pub = rospy.Publisher("/velodyne_points", PointCloud2, queue_size=10)

                self.odom_broadcaster = tf2_ros.TransformBroadcaster()
                self.broadcaster = tf2_ros.StaticTransformBroadcaster()
                self.odom_trans = geometry_msgs.msg.TransformStamped()
                self.listener = TransformListener()

                self.right_lidar = np.array([])
                self.right_transform = PointCloud2()

                self.left_lidar = np.array([])
                self.left_transform = None

                self.vehicle_lidar = PointCloud2()
                self.gtPub = rospy.Publisher('gt_path', Path, queue_size=1)
                self.path = Path()
                self.poses = []
                self.trans_mat = []
                self.read_points()
                self.slam_poses = []
                self.error = 0
                self.poselist = []
                self.rotlist = []
                while not rospy.is_shutdown():
                        self.main()
                        self.rate.sleep()
                # self.trans_mat = np.array(self.trans_mat)
                # self.slam_poses = np.array(self.slam_poses)
                # print(self.trans_mat.shape)
                # self.error = np.linalg.norm((self.slam_poses - self.trans_mat), ord=1, axis=1)
                # print(self.error)

        def read_points(self):
                ground_truth = csv.reader(open('/home/alecmp2/data_set/urban39-pankyo/global_pose.csv', 'rb'), delimiter=',', quotechar='|')
                for gt in ground_truth:
                        pose = geometry_msgs.msg.PoseStamped()
                        transform = np.float_(gt)[1:].reshape(3,4)
                        homo_tf = np.vstack((transform, [0,0,0,1]))
                        quat = quaternion_from_matrix(homo_tf)
                        # pose.header.stamp.secs = int(gt[0][:10])
                        # pose.header.stamp.nsecs = int(gt[0][10:])
                        pose.header.stamp = rospy.Time.now()
                        pose.header.frame_id = 'odom'
                        pose.pose.position.x = homo_tf[0, 3]
                        pose.pose.position.y = homo_tf[1, 3]
                        pose.pose.position.z = homo_tf[2, 3]
                        pose.pose.orientation.x = quat[0]
                        pose.pose.orientation.y = quat[1]
                        pose.pose.orientation.z = quat[2]
                        pose.pose.orientation.w = quat[3]
                        self.trans_mat.append([[homo_tf[0, 3], homo_tf[1, 3], homo_tf[2, 3]], [quat[0], quat[1], quat[2], quat[3]]])
                        self.poses.append(pose)
                        if gt[0][:10] == '1559196281':
                                break

        def trajCallback(self, msg):
                for marker in msg.markers:
                        self.poselist.append([marker.pose.position.x, marker.pose.position.y, marker.pose.position.z])
                        self.rotlist.append([marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w])
                self.slam_poses.append([self.poselist, self.rotlist])
                        
        # def lidar_callback(self, left_msg):
        #         print("called back")
        #         (trans,rot) = self.listener.lookupTransform('/vehicle', '/right_velodyne', rospy.Time(0))
        #         self.right_transform = quaternion_matrix(rot)
        #         self.right_transform[:3, 3] = np.array(trans)
        #         gen = pc2.read_points(right_msg, skip_nans=True)
        #         self.right_lidar = np.array(list(gen))
        #         self.right_lidar = np.matmul(self.right_lidar, self.right_transform)

        #         (trans,rot) = self.listener.lookupTransform('/vehicle', '/left_velodyne', rospy.Time(0))
        #         self.left_transform = quaternion_matrix(rot)
        #         self.left_transform[:3, 3] = np.array(trans)
        #         gen = pc2.read_points(left_msg, skip_nans=True)
        #         self.left_lidar = np.array(list(gen))
        #         self.left_lidar = np.matmul(self.left_lidar, self.left_transform)

        #         header = std_msgs.msg.Header()
        #         header.stamp = right_msg.header.stamp
        #         header.frame_id = 'vehicle'
        #         self.vehicle_lidar = pc2.create_cloud_xyz32(header, np.vstack(self.right_lidar, self.left_lidar)) 
        
        def right_lidar_callback(self, msg):
                #print("Right Stamp:", msg.header.stamp)
                pass
                #(trans,rot) = self.listener.lookupTransform('/base_link', '/right_velodyne',  rospy.Time(0))
                # right_transform = quaternion_matrix(rot)
                # right_transform[:3, 3] = np.array(trans)
                # self.right_transform = right_transform
                # gen = list(pc2.read_points(msg, field_names = ('x', 'y', 'z'), skip_nans=True))
                # right_lidar = np.concatenate((np.array(gen), np.ones(len(gen)).reshape(-1,1)), axis=1)
                # self.right_lidar = np.matmul(self.right_transform, right_lidar.T).T[:, :3]


        def left_lidar_callback(self, msg):
                pass
                #print("Left Stamp:", msg.header.stamp)
                #(trans,rot) = self.listener.lookupTransform( '/base_link', '/left_velodyne', rospy.Time(0))
                # self.left_transform = quaternion_matrix(rot)
                # self.left_transform[:3, 3] = np.array(trans)
                # gen = list(pc2.read_points(msg, field_names = ('x', 'y', 'z'), skip_nans=True))
                # left_lidar = np.concatenate((np.array(gen), np.ones(len(gen)).reshape(-1, 1)), axis=1)
                # self.left_lidar = np.matmul(self.left_transform, left_lidar.T).T[:, :3]

        def odom_callback(self, msg):
                self.odom_trans.header.stamp = msg.header.stamp
                self.odom_trans.header.frame_id = 'odom'
                self.odom_trans.child_frame_id = 'base_link'
                self.odom_trans.transform.translation.x = msg.pose.pose.position.x
                self.odom_trans.transform.translation.y = msg.pose.pose.position.y
                self.odom_trans.transform.translation.z = msg.pose.pose.position.z
                self.odom_trans.transform.rotation = msg.pose.pose.orientation

        def main(self):

                leftTransform = geometry_msgs.msg.TransformStamped()

                _Euler = np.radians(np.array([1.42947, 44.5011, 136.385]))
                _T = [-0.334623, 0.431973, 1.94043]

                leftTransform.header.stamp = rospy.Time.now()
                leftTransform.header.frame_id = "base_link"
                leftTransform.child_frame_id = "left_velodyne"

                leftTransform.transform.translation.x = float(_T[0])
                leftTransform.transform.translation.y = float(_T[1])
                leftTransform.transform.translation.z = float(_T[2])

                quat = quaternion_from_euler(float(_Euler[0]),float(_Euler[1]),float(_Euler[2]))
                leftTransform.transform.rotation.x = quat[0]
                leftTransform.transform.rotation.y = quat[1]
                leftTransform.transform.rotation.z = quat[2]
                leftTransform.transform.rotation.w = quat[3]

                rightTransform = geometry_msgs.msg.TransformStamped()    
                _Euler = np.radians(np.array([178.899, 135.416, 43.7457]))
                _T = [-0.333596, -0.373928, 1.94377]

                rightTransform.header.stamp = rospy.Time.now()
                rightTransform.header.frame_id = "base_link"
                rightTransform.child_frame_id = "right_velodyne"

                rightTransform.transform.translation.x = float(_T[0])
                rightTransform.transform.translation.y = float(_T[1])
                rightTransform.transform.translation.z = float(_T[2])

                quat = quaternion_from_euler(float(_Euler[0]),float(_Euler[1]),float(_Euler[2]))
                rightTransform.transform.rotation.x = quat[0]
                rightTransform.transform.rotation.y = quat[1]
                rightTransform.transform.rotation.z = quat[2]
                rightTransform.transform.rotation.w = quat[3]

                # Stereo Transform
                leftStereoTransform = geometry_msgs.msg.TransformStamped() 
                _Euler = np.radians(np.array([-90.878, 0.0132, 90.3899]))
                _T = [1.64239, 0.247401, 1.58411]

                leftStereoTransform.header.stamp = rospy.Time.now()
                leftStereoTransform.header.frame_id = "base_link"
                leftStereoTransform.child_frame_id = "stereo_left"

                leftStereoTransform.transform.translation.x = float(_T[0])
                leftStereoTransform.transform.translation.y = float(_T[1])
                leftStereoTransform.transform.translation.z = float(_T[2])

                quat = quaternion_from_euler(float(_Euler[0]),float(_Euler[1]),float(_Euler[2]))
                leftStereoTransform.transform.rotation.x = quat[0]
                leftStereoTransform.transform.rotation.y = quat[1]
                leftStereoTransform.transform.rotation.z = quat[2]
                leftStereoTransform.transform.rotation.w = quat[3]

                rightStereoTransform = geometry_msgs.msg.TransformStamped() 
                _Euler = np.radians(np.array([0, 0.0, 0]))
                _T = [0.4, 0, 0]

                rightStereoTransform.header.stamp = rospy.Time.now()
                rightStereoTransform.header.frame_id = "base_link"
                rightStereoTransform.child_frame_id = "stereo_right"

                rightStereoTransform.transform.translation.x = float(_T[0])
                rightStereoTransform.transform.translation.y = float(_T[1])
                rightStereoTransform.transform.translation.z = float(_T[2])

                quat = quaternion_from_euler(float(_Euler[0]),float(_Euler[1]),float(_Euler[2]))
                rightStereoTransform.transform.rotation.x = 1
                rightStereoTransform.transform.rotation.y = 0
                rightStereoTransform.transform.rotation.z = 0
                rightStereoTransform.transform.rotation.w = 0

                # eyeTransform = geometry_msgs.msg.TransformStamped() 
                # _Euler = np.radians(np.array([0, 0.0, 0]))
                # _T = [0, 0, 0]

                # eyeTransform.header.stamp = rospy.Time.now()
                # eyeTransform.header.frame_id = "robot/base_link"
                # eyeTransform.child_frame_id = "base_link"

                # eyeTransform.transform.translation.x = float(_T[0])
                # eyeTransform.transform.translation.y = float(_T[1])
                # eyeTransform.transform.translation.z = float(_T[2])

                # quat = quaternion_from_euler(float(_Euler[0]),float(_Euler[1]),float(_Euler[2]))
                # eyeTransform.transform.rotation.x = 1
                # eyeTransform.transform.rotation.y = 0
                # eyeTransform.transform.rotation.z = 0
                # eyeTransform.transform.rotation.w = 0

                imuTransform = geometry_msgs.msg.TransformStamped() 
                _Euler = np.radians(np.array([0, 0.0, 0]))
                _T = [-0.07, 0, 1.7]

                imuTransform.header.stamp = rospy.Time.now()
                imuTransform.header.frame_id = "base_link"
                imuTransform.child_frame_id = "imu"

                imuTransform.transform.translation.x = float(_T[0])
                imuTransform.transform.translation.y = float(_T[1])
                imuTransform.transform.translation.z = float(_T[2])

                quat = quaternion_from_euler(float(_Euler[0]),float(_Euler[1]),float(_Euler[2]))
                imuTransform.transform.rotation.x = 1
                imuTransform.transform.rotation.y = 0
                imuTransform.transform.rotation.z = 0
                imuTransform.transform.rotation.w = 0

                gtTransform = geometry_msgs.msg.TransformStamped() 
                quat = [-0.01088751,  0.0173971,   0.70819536,  0.70571817]
                _T = [332458.6901, 4140632.316, 17.95512652]

                gtTransform.header.stamp = rospy.Time.now()
                gtTransform.header.frame_id = "global"
                gtTransform.child_frame_id = "map"

                gtTransform.transform.translation.x = float(_T[0])
                gtTransform.transform.translation.y = float(_T[1])
                gtTransform.transform.translation.z = float(_T[2])

                gtTransform.transform.rotation.x = quat[0]
                gtTransform.transform.rotation.y = quat[1]
                gtTransform.transform.rotation.z = quat[2]
                gtTransform.transform.rotation.w = quat[3]
                


                # # DLO Transforms
                # dlotrans = geometry_msgs.msg.TransformStamped() 
                # _Euler = np.radians(np.array([0, 0.0, 0]))
                # _T = [0, 0, 0]

                # dlotrans.header.stamp = rospy.Time.now()
                # dlotrans.header.frame_id = "base_link"
                # dlotrans.child_frame_id = "stereo_right"

                # dlotrans.transform.translation.x = float(_T[0])
                # dlotrans.transform.translation.y = float(_T[1])
                # dlotrans.transform.translation.z = float(_T[2])

                # quat = quaternion_from_euler(float(_Euler[0]),float(_Euler[1]),float(_Euler[2]))
                # dlotrans.transform.rotation.x = quat[0]
                # dlotrans.transform.rotation.y = quat[1]
                # dlotrans.transform.rotation.z = quat[2]
                # dlotrans.transform.rotation.w = quat[3]

                # dlotrans2 = geometry_msgs.msg.TransformStamped() 
                # _Euler = np.radians(np.array([0, 0.0, 0]))
                # _T = [0, 0, 0]

                # dlotrans2.header.stamp = rospy.Time.now()
                # dlotrans2.header.frame_id = "base_link"
                # dlotrans2.child_frame_id = "stereo_right"

                # dlotrans2.transform.translation.x = float(_T[0])
                # dlotrans2.transform.translation.y = float(_T[1])
                # dlotrans2.transform.translation.z = float(_T[2])

                # quat = quaternion_from_euler(float(_Euler[0]),float(_Euler[1]),float(_Euler[2]))
                # dlotrans2.transform.rotation.x = quat[0]
                # dlotrans2.transform.rotation.y = quat[1]
                # dlotrans2.transform.rotation.z = quat[2]
                # dlotrans2.transform.rotation.w = quat[3]
                self.path.header.stamp = rospy.Time.now()
                self.path.header.frame_id = 'global'
                self.path.poses = self.poses
                self.gtPub.publish(self.path)

                self.broadcaster.sendTransform(gtTransform)
                #self.broadcaster.sendTransform([leftTransform, rightTransform, leftStereoTransform, rightStereoTransform, imuTransform])
                #self.odom_broadcaster.sendTransform([self.odom_trans])

                # if self.left_lidar.size > 5 and self.right_lidar.size > 5:
                #         header = std_msgs.msg.Header()
                #         header.stamp = self.odom_trans.header.stamp
                #         header.frame_id = 'base_link'
                #         self.vehicle_lidar = pc2.create_cloud_xyz32(header, np.vstack([self.right_lidar, self.left_lidar]))

                #         self.lidar_pub.publish(self.vehicle_lidar)


if __name__ == '__main__':
        rospy.init_node('static_tf')
        x = OptimusPrime()
        rospy.spin()