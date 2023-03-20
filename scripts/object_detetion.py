#! /usr/bin/env python

import rospy
import ros_numpy
import numpy as np
import time

from sklearn.cluster import DBSCAN

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PolygonStamped, Polygon, Point32
from jsk_recognition_msgs.msg import BoundingBoxArray, BoundingBox

class LiDARObjectDetection:
    def __init__(self):
        rospy.init_node("lidar_detection_node", anonymous=False)
        rospy.Subscriber("/sw/outliers", PointCloud2, self.lidar_cb)
        self.filtered_pub = rospy.Publisher("/sw/inRange", PointCloud2, queue_size=10)
        self.bbox_pub = rospy.Publisher("/bbox", BoundingBoxArray, queue_size=10)

    def lidar_cb(self, msg):
        start_time = time.time()
        xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)

        y_condition = np.logical_and(xyz_array[:, 1] < 4, xyz_array[:, 1] > -4)
        filtered_data = xyz_array[np.logical_and(y_condition, xyz_array[:, 2] < 3)]

        self.convert_xyz_to_pc2(filtered_data, msg.header)
        
        # DBscanning
        model = DBSCAN(eps=0.8, min_samples=5).fit_predict(filtered_data)
        
        cluster_list = []
        print("len of cluster : {}".format(max(model)+1))
        for i in range(max(model)+1):
            tmp = filtered_data[model==i]
            cluster_list.append(tmp)
        
        bbox_list = BoundingBoxArray()
        bbox_list.header = msg.header
        for idx, cluster in enumerate(cluster_list):
            bbox = BoundingBox()
            x_max, y_max, z_max = np.max(cluster, axis=0)
            x_min, y_min, z_min = np.min(cluster, axis=0)
            bbox.header = msg.header
            bbox.pose.position.x = (x_max+x_min)/2
            bbox.pose.position.y = (y_max+y_min)/2
            bbox.pose.position.z = (z_max+z_min)/2
            bbox.pose.orientation.x = 0
            bbox.pose.orientation.y = 0
            bbox.pose.orientation.z = 0
            bbox.pose.orientation.w = 1
            bbox.dimensions.x = abs(x_max-x_min)
            bbox.dimensions.y = abs(y_max-y_min)
            bbox.dimensions.z = abs(z_max-z_min)
            bbox_list.boxes.append(bbox)

        self.bbox_pub.publish(bbox_list)
        print("processing time : {}s / len : {}".format(time.time()-start_time,  len(filtered_data)))
    
    def convert_xyz_to_pc2(self, xyz_array, header):
        pc_array = np.zeros(len(xyz_array), dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32),
        ])
        pc_array['x'] = xyz_array[:, 0]
        pc_array['y'] = xyz_array[:, 1]
        pc_array['z'] = xyz_array[:, 2]
        pc_array['intensity'] = 255

        pc_msg = ros_numpy.msgify(PointCloud2, pc_array, stamp=header.stamp, frame_id=header.frame_id)
        self.filtered_pub.publish(pc_msg)
        return pc_msg


if __name__ == "__main__":
    lp = LiDARObjectDetection()
    rospy.spin()