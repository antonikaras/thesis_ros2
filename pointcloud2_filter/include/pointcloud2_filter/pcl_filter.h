// PCL Libraries
#include <pcl/filters/voxel_grid.h>
#include "pcl_conversions/pcl_conversions.h"

// C++ Libraries
#include <chrono>
#include <memory>
#include <algorithm>
#include <cmath>

// ROS2 Libraries
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "tf2_msgs/msg/tf_message.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


class PCLFilter : public rclcpp::Node
{
    private:
        void _pointCloud2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const;
        //void _tf_callback(const tf2_msgs::msg::TFMessage msg) const;
        void _tfStatic_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg);
        void _imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_sub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub;
        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;

        // Horizontal number of laser beams
        int hBeams;
        // Vertical number of laser beams
        int vBeams;
        // Vertical FoV
        float vFoV;
        // Horizontal FoV
        float hFoV;
        // Robot base frame name
        std::string robotBaseFrame;
        // Sensor scan frame name
        std::string sensorScanFrame;
        // pitch angle -> used to the pointcloud filter
        double pitch;

        // Distance from the robot to the ground
        float filterThres = 0;
        
        tf2_ros::Buffer buffer_;
        tf2_ros::TransformListener listener_;
        builtin_interfaces::msg::Time simTime;
        //rclcpp::TimerBase::SharedPtr timer_;

    public:

        PCLFilter() : Node("pcl_filter"), buffer_(this->get_clock()), listener_(buffer_)
        {
            // Handle parameters
            this->declare_parameter<int>("hBeams", 1875);
            this->get_parameter("hBeams", hBeams);

            this->declare_parameter<int>("vBeams", 16);
            this->get_parameter("vBeams", vBeams);

            this->declare_parameter<float>("vFoV", 0.2617993878);
            this->get_parameter("vFoV", vFoV);

            this->declare_parameter<float>("hFoV", 3.14);
            this->get_parameter("hFoV", hFoV);

            this->declare_parameter<std::string>("robotBaseFrame", "velodyne_base_link");
            this->get_parameter("robotBaseFrame", robotBaseFrame);

            this->declare_parameter<std::string>("sensorScanFrame", "velodyne_base_scan");
            this->get_parameter("sensorScanFrame", sensorScanFrame);

            RCLCPP_INFO(this->get_logger(), "Horizontal beams: '%d, Horizontal Field of View: '%f'", hBeams, hFoV);
            RCLCPP_INFO(this->get_logger(), "Vertical beams: '%d', Vertical Field of View '%f'", vBeams, vFoV);

            // Create subscribers
            //// /points2
            pc2_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("velodyne/scan", 10, std::bind(&PCLFilter::_pointCloud2_callback, this, std::placeholders::_1));
            //// /tf_static
            tf_static_sub = this->create_subscription<tf2_msgs::msg::TFMessage>("tf_static", 10, std::bind(&PCLFilter::_tfStatic_callback, this, std::placeholders::_1));
            //// /imu
            imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("imu", 10, std::bind(&PCLFilter::_imu_callback, this, std::placeholders::_1));

            /*//// /tf
            timer_ = this->create_wall_timer(500ms, std::bind(&PCLFilter::_tf_callback, this));
            // Read the transformation between the robot's base and the lidar scanner
            try
            {
                transformStamped = buffer_.lookupTransform("velodyne_base_scan", "base_link", this->now());
                RCLCPP_INFO(this->get_logger(), "Transform.z : '%f'", transformStamped.transform.translation.z);
            }
            catch (tf2::TransformException &ex) 
            {
                RCLCPP_WARN(this->get_logger(), "%s", ex.what());
            }    
            */
            // Create publishers
            //// /points2
            pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points2", 10);

        }
       
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PCLFilter>());
  rclcpp::shutdown();
  return 0;
}