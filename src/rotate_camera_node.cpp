#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

class RotateCameraNode : public rclcpp::Node
{
public:
  RotateCameraNode()
  : Node("rotate_camera_node")
  {
    // Create publishers for rotated image + camera info
    image_pub_ = image_transport::create_publisher(this, "/camera_rotated/color/image_raw");
    ci_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_rotated/color/camera_info", 10);

    // Subscribe to the original camera topics
    image_sub_ = image_transport::create_subscription(
      this, "/camera/color/image_raw",
      std::bind(&RotateCameraNode::imageCallback, this, std::placeholders::_1),
      "raw"   // Transport hint
    );

    ci_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/color/camera_info", 10,
      std::bind(&RotateCameraNode::ciCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "RotateCameraNode started. Subscribing to /camera/color/image_raw + camera_info");
  }

private:
  // Callback: store latest CameraInfo
  void ciCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    current_ci_ = *msg;
    have_ci_ = true;
  }

  // Callback: rotate incoming image + camera_info, then publish
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    if (!have_ci_) {
      // Wait until we have at least one camera_info
      return;
    }

    // Convert ROS Image -> cv::Mat
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Rotate the image 90 deg clockwise
    cv::Mat rotated;
    cv::rotate(cv_ptr->image, rotated, cv::ROTATE_90_CLOCKWISE);

    // Make a copy of the CameraInfo, then rotate intrinsics
    sensor_msgs::msg::CameraInfo rotated_ci = current_ci_;
    rotateCameraInfo90CW(rotated_ci);

    // Create the new ROS image
    cv_bridge::CvImage out_cv;
    out_cv.header = msg->header;  // Keep same timestamp/frame_id
    out_cv.encoding = msg->encoding;
    out_cv.image = rotated;

    // Convert cv_bridge -> sensor_msgs::Image
    sensor_msgs::msg::Image::SharedPtr out_img = out_cv.toImageMsg();
    out_img->width = rotated.cols;
    out_img->height = rotated.rows;
    out_img->step = rotated.cols * static_cast<unsigned int>(rotated.elemSize());

    // Update the rotated CameraInfo dimension/size
    rotated_ci.header = msg->header;  // same timestamp/frame as the image
    rotated_ci.width  = out_img->width;
    rotated_ci.height = out_img->height;

    // Publish them
    image_pub_.publish(*out_img);
    ci_pub_->publish(rotated_ci);
  }

  // Helper: rotate intrinsics for a 90° clockwise rotation
  void rotateCameraInfo90CW(sensor_msgs::msg::CameraInfo & ci)
  {
    // Original intrinsics (K)
    double fx = ci.k[0];
    double fy = ci.k[4];
    double cx = ci.k[2];
    double cy = ci.k[5];

    // Original image size
    uint32_t w = ci.width;
    uint32_t h = ci.height;

    // After rotating 90° CW:
    //   new width = old height
    //   new height = old width
    //   new_fx = old_fy
    //   new_fy = old_fx
    //   new_cx = old_cy
    //   new_cy = (new_height - old_cx)
    ci.width  = h;
    ci.height = w;

    double new_fx = fy;
    double new_fy = fx;
    double new_cx = cy;
    double new_cy = static_cast<double>(ci.height) - cx;

    // Overwrite ci.k
    ci.k[0] = new_fx;   ci.k[1] = 0.0;      ci.k[2] = new_cx;
    ci.k[3] = 0.0;      ci.k[4] = new_fy;   ci.k[5] = new_cy;
    ci.k[6] = 0.0;      ci.k[7] = 0.0;      ci.k[8] = 1.0;

    // Also update ci.p if used
    ci.p[0] = new_fx;   ci.p[1] = 0.0;      ci.p[2] = new_cx;  ci.p[3] = 0.0;
    ci.p[4] = 0.0;      ci.p[5] = new_fy;   ci.p[6] = new_cy;  ci.p[7] = 0.0;
    ci.p[8] = 0.0;      ci.p[9] = 0.0;      ci.p[10] = 1.0;    ci.p[11] = 0.0;
    // Distortion (ci.d) is left as-is for simplicity. 
  }

  // State
  bool have_ci_ = false;
  sensor_msgs::msg::CameraInfo current_ci_;

  // Publishers
  image_transport::Publisher image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr ci_pub_;

  // Subscribers
  image_transport::Subscriber image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr ci_sub_;
};

// Standard main
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RotateCameraNode>());
  rclcpp::shutdown();
  return 0;
}