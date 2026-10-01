/*** Temporary fix to https://github.com/gazebosim/gz-sensors/issues/545  The points from the rgbd camera gazebo sensors plugin are not rotated correctly */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

class GzRgbdPointCloudFixer : public rclcpp::Node
{
  public:
    GzRgbdPointCloudFixer()
    : Node("gz_rgbd_fixer")
    {
      
      declare_parameter<std::vector<std::string>>("in_topics", {"in"});
      declare_parameter<std::vector<std::string>>("out_topics", {"out"});
        
        // Retrieve the value
      std::vector<std::string> in_topics = this->get_parameter("in_topics").as_string_array();
      std::vector<std::string> out_topics = this->get_parameter("out_topics").as_string_array();

      if(in_topics.size() == out_topics.size())
      {
        for(int32_t idx = 0; idx < in_topics.size(); ++idx)
        {

          rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber;
          rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;

          std::function<void(const sensor_msgs::msg::PointCloud2::SharedPtr msg)> bound_callback_func =
          std::bind(&GzRgbdPointCloudFixer::topic_callback, this, std::placeholders::_1, idx);

          subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                in_topics[idx], 10,
                bound_callback_func);
          publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(out_topics[idx], rclcpp::SensorDataQoS());
          subscribers_.push_back(subscriber);
          publishers_.push_back(publisher);
        }
        RCLCPP_FATAL(this->get_logger(), "GzRgbdPointCloudFixer does not have equal numbers of in/out cloud topics -- it will not publish clouds!");
      }
        
                  
      tf2::Quaternion q;
      q.setRPY( 0.0000, -1.5708,  1.5708);

      transform_.transform.rotation.x = q.x();
      transform_.transform.rotation.y = q.y();
      transform_.transform.rotation.z = q.z();
      transform_.transform.rotation.w = q.w();

      transform_.transform.translation.x = 0.0; // translation does not change
      transform_.transform.translation.y = 0.0;
      transform_.transform.translation.z = 0.0;

      transform_.child_frame_id = "child"; // this is just a dummy -- pointcloud will retain it's header
      transform_.header.frame_id = "parent";
    }
      

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_in, const int32_t& pub_idx) 
    {
      
      sensor_msgs::msg::PointCloud2 cloud_out;

      transform_.header.stamp = cloud_in->header.stamp;

      tf2::doTransform(*cloud_in, cloud_out, transform_);
        
      // cloud_out is now populated, but has the dummy header
      cloud_out.header = cloud_in->header;

    
      publishers_[pub_idx]->publish(cloud_out);
        
    }


    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> subscribers_;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> publishers_;
    
    geometry_msgs::msg::TransformStamped transform_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GzRgbdPointCloudFixer>());
  rclcpp::shutdown();
  return 0;
}