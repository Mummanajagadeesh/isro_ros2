#include <chrono> 
#include <functional> 
#include <memory> 
#include <string> 

#include <opencv2/opencv.hpp> 
#include "rclcpp/rclcpp.hpp" 
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"

#include "ImuTypes.h"

#include "rclcpp/qos.hpp"

using std::placeholders::_1;


#include "System.h"

class PosePublisher : public rclcpp::Node {
  public:
    PosePublisher(const std::string& vocab_file, const std::string& settings_file) : Node("pose_publisher")
    { 
      slam_ = std::make_shared<ORB_SLAM3::System>(vocab_file, settings_file, ORB_SLAM3::System::IMU_MONOCULAR, true);
      pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("orbslam_pose", 10);
      imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "mavros/imu/data",
        rclcpp::SensorDataQoS(),
        std::bind(&PosePublisher::imu_sub_cb, this, _1)
      );
    }

    void camera_sub_init() {
      image_transport_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
      subscription_ = image_transport_->subscribe(
        "camera_feed/image_raw",  
        1, // queue size
        std::bind(&PosePublisher::image_callback, this, std::placeholders::_1)
      );
    }
 
    ~PosePublisher() {
      slam_->Shutdown();
    }

  private:

  vector<ORB_SLAM3::IMU::Point> imu_meas;

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;

    try {
      cv_ptr = cv_bridge::toCvCopy(*msg, "bgr8");
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

    double camera_feed_published_timestamp = msg->header.stamp.sec + (msg->header.stamp.nanosec / 1e9);
    double camera_feed_received_timestamp = this->now().seconds();

    Sophus::SE3f Tcw_SE3 = slam_->TrackMonocular(gray, camera_feed_published_timestamp, imu_meas);

    if (!Tcw_SE3.matrix().isZero()) {
      Sophus::SE3f Twc_SE3 = Tcw_SE3.inverse();
      Eigen::Vector3f t = Twc_SE3.translation();
      Eigen::Quaternionf q = Twc_SE3.unit_quaternion();

      auto pose_msg = geometry_msgs::msg::PoseStamped();
      pose_msg.pose.position.x = t[0];
      pose_msg.pose.position.y = t[1];
      pose_msg.pose.position.z = t[2];
      pose_msg.pose.orientation.x = q.x();
      pose_msg.pose.orientation.y = q.y();
      pose_msg.pose.orientation.z = q.z();
      pose_msg.pose.orientation.w = q.w();

      pose_msg.header.frame_id = "base_frame";
      pose_msg.header.stamp = msg->header.stamp;

      pose_publisher_->publish(pose_msg);

      double slam_published_timestamp = this->now().seconds();
      RCLCPP_INFO(this->get_logger(), "Camera feed lag: %f, Processing time: %f)", camera_feed_received_timestamp - camera_feed_published_timestamp, slam_published_timestamp - camera_feed_received_timestamp);
    }

    imu_meas.clear();
  }
  
  // callback for the imu data
  void imu_sub_cb(const sensor_msgs::msg::Imu::ConstSharedPtr& msg){
    RCLCPP_INFO(this->get_logger(), "Received IMU");
    double imu_timestamp = msg->header.stamp.sec + (msg->header.stamp.nanosec / 1e9);
    imu_meas.push_back(ORB_SLAM3::IMU::Point(
      msg->linear_acceleration.x,
      msg->linear_acceleration.y,
      msg->linear_acceleration.z,
      msg->angular_velocity.x,
      msg->angular_velocity.y,
      msg->angular_velocity.z,
      imu_timestamp
    ));
  } 

  std::shared_ptr<ORB_SLAM3::System> slam_;
  std::shared_ptr<image_transport::ImageTransport> image_transport_;
  image_transport::Subscriber subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
};

int main(int argc, char * argv[])
{
  if (argc != 3) {
    std::cerr << "Usage: ros2 run <pkg> <node> path_to_vocabulary path_to_settings_yaml" << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<PosePublisher>(argv[1], argv[2]);
  node->camera_sub_init();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
