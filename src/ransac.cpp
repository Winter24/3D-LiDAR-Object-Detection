#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

ros::Publisher pub_in;
ros::Publisher pub_out;

void vlp_callback(const sensor_msgs::PointCloud2Ptr &msg){
    clock_t start, finish;
    double duration;
    start = clock();

    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), 
                                        cloud_p (new pcl::PointCloud<pcl::PointXYZ>), 
                                        // inlierPoints (new pcl::PointCloud<pcl::PointXYZ>),
                                        outlierPoints (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*msg, *cloud);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());;

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setInputCloud (cloud);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.25);
    seg.setMaxIterations(50);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.empty()) {
        std::cerr << "Could not estimate a planar model for the given dataset" << std::endl;
        }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*outlierPoints);

    // pcl::ExtractIndices<pcl::PointXYZ> extract_out;
    // extract_out.setInputCloud (cloud);
    // extract_out.setIndices (inliers);
    // extract_out.filter (*inlierPoints);

    // sensor_msgs::PointCloud2 in_points, out_points;
    sensor_msgs::PointCloud2 out_points;
    // pcl::toROSMsg(*inlierPoints, in_points);
    pcl::toROSMsg(*outlierPoints, out_points);

    // pub_in.publish(in_points);
    pub_out.publish(out_points);

    finish = clock();
    duration = (double)(finish - start) / CLOCKS_PER_SEC;
    std::cout << "precessing time : " <<duration << "s" << std::endl;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "pub_node");
    ros::NodeHandle nh;
    std::cout << "start" << std::endl;

    // ros::Subscriber sub = nh.subscribe("/os1_cloud_node/points", 1, vlp_callback);
    ros::Subscriber sub = nh.subscribe("/velodyne_points", 1, vlp_callback);
    pub_out = nh.advertise<sensor_msgs::PointCloud2> ("/sw/outliers", 1);
    // pub_in = nh.advertise<sensor_msgs::PointCloud2> ("/sw/inliers", 1);
    // pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);

    ros::spin();
    return 0; 
}