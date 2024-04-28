#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"


class ImageFilterer : public rclcpp::Node
{
  public:
    ImageFilterer()
    : Node("img_filterer")
    {
      this->declare_parameter("subscribing_topic", "");

      this->declare_parameter("gaussian_blur.enable", false);
      this->declare_parameter("gaussian_blur.publish", false);
      this->declare_parameter("gaussian_blur.publish_topic", "");
      this->declare_parameter("gaussian_blur.show_image", false);
      this->declare_parameter("gaussian_blur.kernel_size", 0);

      this->declare_parameter("grayscale_conversion.enable", false);
      this->declare_parameter("grayscale_conversion.publish", false);
      this->declare_parameter("grayscale_conversion.publish_topic", "");
      this->declare_parameter("grayscale_conversion.show_image", false);

      this->declare_parameter("canny_edge_detection.enable", false);
      this->declare_parameter("canny_edge_detection.publish", false);
      this->declare_parameter("canny_edge_detection.publish_topic", "");
      this->declare_parameter("canny_edge_detection.show_image", false);
      this->declare_parameter("canny_edge_detection.low_threshold", 0);
      this->declare_parameter("canny_edge_detection.high_threshold", 0);

      this->get_parameter("subscribing_topic", subscribing_topic_);

      this->get_parameter("gaussian_blur.enable", gaussian_blur_enable_);
      this->get_parameter("gaussian_blur.publish", gaussian_blur_publish_);
      this->get_parameter("gaussian_blur.publish_topic", gaussian_blur_publish_topic_);
      this->get_parameter("gaussian_blur.show_image", gaussian_blur_show_image_);
      this->get_parameter("gaussian_blur.kernel_size", gaussian_blur_kernel_size_);

      this->get_parameter("grayscale_conversion.enable", grayscale_conversion_enable_);
      this->get_parameter("grayscale_conversion.publish", grayscale_conversion_publish_);
      this->get_parameter("grayscale_conversion.publish_topic", grayscale_conversion_publish_topic_);
      this->get_parameter("grayscale_conversion.show_image", grayscale_conversion_show_image_);
      
      this->get_parameter("canny_edge_detection.enable", canny_edge_detection_enable_);
      this->get_parameter("canny_edge_detection.publish", canny_edge_detection_publish_);
      this->get_parameter("canny_edge_detection.publish_topic", canny_edge_detection_publish_topic_);
      this->get_parameter("canny_edge_detection.show_image", canny_edge_detection_show_image_);
      this->get_parameter("canny_edge_detection.low_threshold", canny_edge_detection_low_threshold_);
      this->get_parameter("canny_edge_detection.high_threshold", canny_edge_detection_high_threshold_);

      RCLCPP_INFO(this->get_logger(), "Subscribing to: %s", subscribing_topic_.c_str());
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        subscribing_topic_, 10, std::bind(&ImageFilterer::image_callback, this, std::placeholders::_1)
        );

      // Initialize only the necessary publishers
      if(gaussian_blur_publish_)
      {
        gaussianPublisher_ = this->create_publisher<sensor_msgs::msg::Image>(gaussian_blur_publish_topic_, 10);
      }
      if(grayscale_conversion_publish_)
      {
        grayscalePublisher_ = this->create_publisher<sensor_msgs::msg::Image>(grayscale_conversion_publish_topic_, 10);
      }
      if(canny_edge_detection_publish_)
      {
        cannyPublisher_ = this->create_publisher<sensor_msgs::msg::Image>(canny_edge_detection_publish_topic_, 10);
      }
    }

  private:
    /**
     * @brief Callback function for the Image message.
     *
     * This function is called whenever an Image message is received. It converts the image to OpenCV format and applies the selected filters.
     * The filtered images are shown and/or published according to the parameters.
     *
     * @param img Image message
     */
    void image_callback(const sensor_msgs::msg::Image img) const
    {
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        rclcpp::shutdown();
      }

      cv::Mat frame = cv_ptr->image;

      if(gaussian_blur_enable_)
      {
        cv::Mat gaussian_blur_frame;
        cv::GaussianBlur(frame, gaussian_blur_frame, cv::Size(gaussian_blur_kernel_size_, gaussian_blur_kernel_size_), 0);
        if(gaussian_blur_show_image_)
        {
          cv::imshow("Gaussian Blur", gaussian_blur_frame);
          cv::waitKey(1);
        }
        if(gaussian_blur_publish_)
        {
          sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", gaussian_blur_frame).toImageMsg();
          gaussianPublisher_->publish(*msg);
        }
      }

      if(grayscale_conversion_enable_)
      {
        cv::Mat grayscale_frame;
        cv::cvtColor(frame, grayscale_frame, cv::COLOR_BGR2GRAY);
        if(grayscale_conversion_show_image_)
        {
          cv::imshow("Grayscale", grayscale_frame);
          cv::waitKey(1);
        }
        if(grayscale_conversion_publish_)
        {
          sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", grayscale_frame).toImageMsg();
          grayscalePublisher_->publish(*msg);
        }
      }

      if(canny_edge_detection_enable_)
      {
        cv::Mat canny_frame;
        cv::Canny(frame, canny_frame, canny_edge_detection_low_threshold_, canny_edge_detection_high_threshold_);
        if(canny_edge_detection_show_image_)
        {
          cv::imshow("Canny Edge Detection", canny_frame);
          cv::waitKey(1);
        }
        if(canny_edge_detection_publish_)
        {
          sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", canny_frame).toImageMsg();
          cannyPublisher_->publish(*msg);
        }
      }

      cv::imshow("Video", frame);
      cv::waitKey(1);

    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr gaussianPublisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr grayscalePublisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cannyPublisher_;
    std::string subscribing_topic_, gaussian_blur_publish_topic_, grayscale_conversion_publish_topic_, canny_edge_detection_publish_topic_;
    bool gaussian_blur_enable_, gaussian_blur_publish_, gaussian_blur_show_image_, grayscale_conversion_enable_, grayscale_conversion_publish_, grayscale_conversion_show_image_, canny_edge_detection_enable_, canny_edge_detection_publish_, canny_edge_detection_show_image_;
    int gaussian_blur_kernel_size_, canny_edge_detection_low_threshold_, canny_edge_detection_high_threshold_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageFilterer>());
  rclcpp::shutdown();
  return 0;
}
