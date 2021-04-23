#include <pointcloud2_filter/pcl_filter.h>

void PCLFilter::_pointCloud2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) const
{
    //RCLCPP_INFO(this->get_logger(), "I heard: '%s'");

    // Container for original & filtered data
    pcl::PCLPointCloud2 cloud ;//= new pcl::PCLPointCloud2; 

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, cloud);

    pcl::PointCloud<pcl::PointXYZ> temp_cloud;
    pcl::fromPCLPointCloud2(cloud,temp_cloud);
    
    // Perform the actual filtering - filter the points that touch the ground
    float mxz = -100, miz = 100;
    for (long unsigned int p = 0; p < temp_cloud.size(); ++p)
    {
        mxz = std::max(mxz, temp_cloud[p].z);
        miz = std::min(miz, temp_cloud[p].z);

        //RCLCPP_INFO(this->get_logger(), "x '%f', y '%f', z '%f'", temp_cloud[p].x, temp_cloud[p].y, temp_cloud[p].z); 
        if (temp_cloud[p].z < -0.95 * filterThres)
        {
          temp_cloud[p].x = 0.0;
          temp_cloud[p].y = 0.0;
          temp_cloud[p].z = 0.0;          
        }
        
    }
    
    // Convert to ROS data type
    sensor_msgs::msg::PointCloud2 pc2_msg_;
    pcl::toROSMsg(temp_cloud, pc2_msg_);
    pc2_msg_.header.frame_id = "velodyne_base_scan";

    // Publish the messages
    pc2_msg_.header.stamp = now();
    pcl_pub_->publish(pc2_msg_);
    
}
/*
void PCLFilter::_tf_callback(const tf2_msgs::msg::TFMessage msg) const
{
    int i = 2;
}
*/
void PCLFilter::_tfStatic_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
{
    for (int i = 0; i < (int)msg->transforms.size(); i++ )
    {
        //RCLCPP_INFO(this->get_logger(), "Parent '%s', child '%s'", msg->transforms[i].header.frame_id.c_str(), msg->transforms[i].child_frame_id.c_str());
        if ((msg->transforms[i].header.frame_id.c_str() == robotBaseFrame) && (msg->transforms[i].child_frame_id.c_str() == sensorScanFrame) )
            filterThres = msg->transforms[i].transform.translation.z;  
        
        //RCLCPP_INFO(this->get_logger(), "Parent '%s', child '%s', z '%f'", msg->transforms[i].header.frame_id.c_str(), msg->transforms[i].child_frame_id.c_str(), (float)msg->transforms[i].transform.translation.z);

            
            //filterThres = msg->transforms[i].transform.translation.z;
    }
}