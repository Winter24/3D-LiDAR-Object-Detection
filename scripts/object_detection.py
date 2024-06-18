#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import time
from sklearn.cluster import DBSCAN
from sensor_msgs_py import point_cloud2 as pc2

from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import BoundingBox3D, BoundingBox3DArray
from std_msgs.msg import Header
from geometry_msgs.msg import Point32, Pose, Vector3

class LiDARObjectDetection(Node):
    def __init__(self):
        super().__init__("lidar_detection_node")
        self.subscription = self.create_subscription(
            PointCloud2,
            "/points_raw",
            self.lidar_cb,
            10)
        self.filtered_pub = self.create_publisher(
            PointCloud2,
            "/sw/inRange",
            10)
        self.bbox_pub = self.create_publisher(
            BoundingBox3DArray,
            "/bbox",
            10)

    def lidar_cb(self, msg):
        start_time = time.time()
        xyz_array = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))

        y_condition = np.logical_and(xyz_array[:, 1] < 4, xyz_array[:, 1] > -4)
        filtered_data = xyz_array[np.logical_and(y_condition, xyz_array[:, 2] < 3)]

        self.convert_xyz_to_pc2(filtered_data, msg.header)
        
        # DBSCAN clustering
        model = DBSCAN(eps=0.8, min_samples=5).fit(filtered_data)
        labels = model.labels_
        
        unique_labels = set(labels)
        bbox_array = BoundingBox3DArray()
        bbox_array.header = msg.header

        for k in unique_labels:
            if k == -1:
                continue  # Skip noise
            class_member_mask = (labels == k)
            cluster = filtered_data[class_member_mask]
            bbox = BoundingBox3D()
            bbox.center.position.x = np.mean(cluster[:, 0])
            bbox.center.position.y = np.mean(cluster[:, 1])
            bbox.center.position.z = np.mean(cluster[:, 2])
            bbox.size.x = max(cluster[:, 0]) - min(cluster[:, 0])
            bbox.size.y = max(cluster[:, 1]) - min(cluster[:, 1])
            bbox.size.z = max(cluster[:, 2]) - min(cluster[:, 2])
            bbox_array.boxes.append(bbox)

        self.bbox_pub.publish(bbox_array)
        print(f"Processing time: {time.time() - start_time}s, Length: {len(filtered_data)}")

    def convert_xyz_to_pc2(self, xyz_array, header):
        point_field_generators = [
            pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1)
        ]
        header = Header(frame_id=header.frame_id, stamp=self.get_clock().now().to_msg())
        pc_msg = pc2.create_cloud(header, point_field_generators, xyz_array)
        self.filtered_pub.publish(pc_msg)

def main(args=None):
    rclpy.init(args=args)
    detector = LiDARObjectDetection()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
