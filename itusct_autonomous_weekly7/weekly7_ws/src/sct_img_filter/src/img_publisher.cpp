#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;


class ImagePublisher : public rclcpp::Node
{
  public:
    ImagePublisher()
    : Node("img_publisher")
    {
      this->declare_parameter("video_file_path", "");
      this->declare_parameter("publishing_frequency", 0);
      this->declare_parameter("publishing_topic", "");

      this->get_parameter("video_file_path", video_file_path_);
      this->get_parameter("publishing_frequency", publishing_frequency_);
      this->get_parameter("publishing_topic", publishing_topic_);

      publish_time_interval_ = std::chrono::duration<double>(1.0 / publishing_frequency_);

      publisher_ = this->create_publisher<sensor_msgs::msg::Image>(publishing_topic_, 10);

        //cv::VideoCapture cap(video_file_path_);
        cap.open(ament_index_cpp::get_package_share_directory("sct_img_filter") + video_file_path_);
        if (!cap.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video file");
            rclcpp::shutdown();
        }

      timer_ = this->create_wall_timer(
      publish_time_interval_, std::bind(&ImagePublisher::timer_callback, this));
    }

  private:
    /**
     * @brief Callback function for the timer.
     *
     * This function is called whenever the timer is triggered. It reads a frame from the video file and publishes it as a sensor_msgs::msg::Image message.
     */
    void timer_callback()
    {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty() && !videoStarted) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read frame from video file");
            rclcpp::shutdown();
        }
        else{
            videoStarted = true;
        }
        if(frame.empty() && videoStarted) {
            RCLCPP_INFO(this->get_logger(), "End of video file");
            rclcpp::shutdown();
        }

        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        publisher_->publish(*msg); // Publish the ROS image message
    }

    std::string video_file_path_;
    int publishing_frequency_;
    std::chrono::duration<double> publish_time_interval_;
    std::string publishing_topic_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    cv::VideoCapture cap;
    bool videoStarted = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImagePublisher>());
  rclcpp::shutdown();
  return 0;
}
