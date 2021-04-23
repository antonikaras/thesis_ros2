#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("pcl_ros2_test"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    
    pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points2", 10);

    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;
  size_t count_;
  int num_points_ = 100;

  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

    // Create the pointcloud2 data
    pcl::PointCloud<pcl::PointXYZRGB> cloud_;
    
    // Draw a sphere
    int rings = 50;
    int sectors = 50;
    float const R = 1./(float)(rings-1);
    float const S = 1./(float)(sectors-1);
    int r, s;

    for(s = 0; s < sectors; s++) for(r = 0; r < rings; r++) {
        pcl::PointXYZRGB pt;
        pt = pcl::PointXYZRGB(255, 100, 18);
        pt.x = sin( -M_PI_2 + M_PI * r * R );
        pt.y = cos(2*M_PI * s * S) * sin( M_PI * r * R );
        pt.z = sin(2*M_PI * s * S) * sin( M_PI * r * R );

        cloud_.points.push_back(pt);
    }
    /*
    for (int i = 0; i < num_points_; ++i) {
      const float fr = static_cast<float>(i) / static_cast<float>(num_points_);
      pcl::PointXYZRGB pt;
      pt = pcl::PointXYZRGB(fr * 255, 255 - fr * 255, 18 + fr * 20);
      //float const y = sin( -M_PI_2 + M_PI * r * R );
      //float const x = cos(2*M_PI * s * S) * sin( M_PI * r * R );
      //float const z = sin(2*M_PI * s * S) * sin( M_PI * r * R );

      pt.x = cos(fr * M_PI * 2.0) * 1.0;
      pt.y = sin(fr * M_PI * 2.0) * 1.0;
      pt.z = 0.0;
      cloud_.points.push_back(pt);
    }
  */
    auto pc2_msg_ = sensor_msgs::msg::PointCloud2();
    pcl::toROSMsg(cloud_, pc2_msg_);
    pc2_msg_.header.frame_id = "velodyne_base_scan";

    // Publish the messages
    publisher_->publish(message);
    pc2_msg_.header.stamp = now();
    pcl_pub_->publish(pc2_msg_);

  }
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}