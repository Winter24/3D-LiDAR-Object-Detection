#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import torch
import time
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

        self.bbox_pub = self.create_publisher(
            BoundingBox3DArray,
            "/bbox",
            10)
        
        device = torch.device('cpu')
        model = torch.load("/home/winter24/gcamp_ros2_ws/pointcloud_ros2/18epoch", map_location=device)
        # model.eval()

    def lidar_cb(self, msg):
       start_time = time.time()
       xyz_array = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
       
       # Process the data through the model
       detections = self.model.predict(xyz_array)  
       bbox_array = BoundingBox3DArray()
       bbox_array.header = msg.header
       for detection in detections:
           if detection['confidence'] > 0.5:  # Use a confidence threshold suitable for your needs
               bbox = BoundingBox3D()
               bbox.center.position.x = (detection['x_min'] + detection['x_max']) / 2
               bbox.center.position.y = (detection['y_min'] + detection['y_max']) / 2
               bbox.center.position.z = 0  # Adjust if your model provides z coordinate
               bbox.size.x = detection['x_max'] - detection['x_min']
               bbox.size.y = detection['y_max'] - detection['y_min']
               bbox.size.z = 0.5  # Assuming a fixed height; adjust as needed
               bbox_array.boxes.append(bbox)
       self.bbox_pub.publish(bbox_array)
       print(f"Processing time: {time.time() - start_time}s")
    
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
