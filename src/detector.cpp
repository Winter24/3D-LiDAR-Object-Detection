#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


void callback(const sensor_msgs::PointCloud2Ptr &msg){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // std::cout << "in callback" << std::endl;
    pcl::fromROSMsg(*msg, *cloud);

    std::cout << cloud->points[0].x << std::endl;
    cloud->points[0].x = 18;
    std::cout << cloud->points[0].x << std::endl;
    std::cout << "end" << std::endl;

    
}

int main(int argc, char** argv){
    ros::init(argc, argv, "object_detector");
    ros::NodeHandle nh;
    std::cout << "start" << std::endl;

    ros::Subscriber sub = nh.subscribe("/sw/outliers", 1, callback);
    
    ros::spin();
    return 0;
}