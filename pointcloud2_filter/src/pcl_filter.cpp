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
    for (long unsigned int p = 0; p < temp_cloud.size(); ++p)
    {   
        // Multiply the thresshold value with a constant depending on the length of the ray
        // The furthest the ray is the less aqurate it is and the 'ground' needs to be lifted more
        float distConst = 0.98;
        
        // Convert the ray location to the polar coordinate system
        float dist = sqrt(temp_cloud[p].x * temp_cloud[p].x + temp_cloud[p].y * temp_cloud[p].y + temp_cloud[p].z * temp_cloud[p].z);
        float ang = atan2(temp_cloud[p].y, temp_cloud[p].x) * 180.0 / M_PI;

        if ( -117 < ang  && ang < 30)
        {
            if (dist < 2)
                distConst = 0.95;
            else if (dist < 5)
                distConst = 0.9;
            else
                distConst = 0.80;
        }  
        //RCLCPP_INFO(this->get_logger(), "x '%f', y '%f', z '%f'", temp_cloud[p].x, temp_cloud[p].y, temp_cloud[p].z); 
        if (temp_cloud[p].z < -distConst * filterThres)
        {
          temp_cloud[p].x = 0.0;
          temp_cloud[p].y = 0.0;
          temp_cloud[p].z = 0.0;          
        }
        
    }
    
    // Convert to ROS data type
    sensor_msgs::msg::PointCloud2 pc2_msg_;
    pcl::toROSMsg(temp_cloud, pc2_msg_);
    pc2_msg_.header.frame_id = sensorScanFrame;

    // Publish the messages
    pc2_msg_.header.stamp = simTime;
    pc2_msg_.fields = cloud_msg->fields;
    //pc2_msg_.point_step = cloud_msg->point_step;
    //pc2_msg_.row_step = cloud_msg->row_step;
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
        {
            filterThres = msg->transforms[i].transform.translation.z;
            simTime = msg->transforms[i].header.stamp;
            //RCLCPP_INFO(this->get_logger(), "Parent '%f'", msg->transforms[i].header.stamp.sec);
        }      

            
            //filterThres = msg->transforms[i].transform.translation.z;
    }
}

void PCLFilter::_imu_callback(const sensor_msgs::msg::imu::SharedPtr msg)
{
    int i = 1;
}