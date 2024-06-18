#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

class RANSACNode : public rclcpp::Node {
public:
    RANSACNode() : Node("ransac_node") {
        pub_out = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sw/outliers", 10);
        sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 10,
            std::bind(&RANSACNode::vlp_callback, this, std::placeholders::_1));
    }

private:
    void vlp_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        auto start = std::chrono::high_resolution_clock::now();

        // Conversion and RANSAC processing as before, ensure all pcl calls are compatible with ROS2
        // ...

        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = finish - start;
        std::cout << "Processing time: " << duration.count() << "s" << std::endl;
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_out;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RANSACNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
