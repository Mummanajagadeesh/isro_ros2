#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class CameraFeedPublisher : public rclcpp::Node {
    public:
        CameraFeedPublisher() : Node("camera_feed_pub") {
            cap_.open(0);
            if (!cap_.isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open webcam.");
                rclcpp::shutdown();
                return;
            }
            timer_ = this->create_wall_timer(10ms, std::bind(&CameraFeedPublisher::publish_frame, this));
        }

        void init() {
            image_transport_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
            image_pub_ = image_transport_->advertise("camera_feed/image_raw", 1);
        }
    
        private:
            void publish_frame() {
                cv::Mat frame;
                cap_ >> frame;

                if (frame.empty()) {
                    RCLCPP_WARN(this->get_logger(), "Empty frame received from webcam.");
                    return;
                }

                // Display the frame in a window
                cv::imshow("Live Webcam Feed", frame);
                cv::waitKey(1);

                auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
                msg->header.stamp = this->now();
                image_pub_.publish(msg);
            }

            cv::VideoCapture cap_;
            rclcpp::TimerBase::SharedPtr timer_;
            std::shared_ptr<image_transport::ImageTransport> image_transport_;
            image_transport::Publisher image_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraFeedPublisher>();
    node->init();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}