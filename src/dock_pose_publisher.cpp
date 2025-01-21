#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

/**
 * @class DockPosePublisher
 * @brief Single node that:
 *   1) Subscribes to camera image (and optional camera info).
 *   2) Detects ArUco markers using OpenCV.
 *   3) Publishes pose of the first (or a specific) marker ID.
 */
class DockPosePublisher : public rclcpp::Node
{
public:
  DockPosePublisher()
  : Node("dock_pose_publisher")
  {
    // Declare and read parameters
    dictionary_id_      = this->declare_parameter<std::string>("dictionary_id", "6x6_250");
    marker_size_        = this->declare_parameter<double>("marker_size", 0.11);  // 110 mm = 0.11 m
    use_first_detection_ = this->declare_parameter<bool>("use_first_detection", false);
    dock_tag_id_        = this->declare_parameter<int>("dock_tag_id", 245);

    // Prepare ArUco objects
    dictionary_ = cv::aruco::getPredefinedDictionary(getDictionaryId(dictionary_id_));
    detector_params_ = cv::aruco::DetectorParameters::create();

    // Publishers
    // Pose publisher: "detected_dock_pose"
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("detected_dock_pose", 10);

    // Subscribers
    // (1) Image subscription (using image_transport for convenience)
    image_sub_ = image_transport::create_subscription(
      this, 
      "/camera_rotated/color/image_raw",
      std::bind(&DockPosePublisher::imageCallback, this, std::placeholders::_1),
      "raw"  // Transport type (raw, compressed, etc.)
    );

    // (2) Camera info subscription (needed for 3D pose estimation)
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_rotated/color/camera_info", 10,
      std::bind(&DockPosePublisher::cameraInfoCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "DockPosePublisher node started");
  }

private:
  // This flag tracks whether we’ve *already* logged a detection message
  bool detection_active_ = false;

  // Called whenever a camera image arrives
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    // Convert ROS Image -> OpenCV Mat
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // Detect ArUco markers
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners, rejected;
    cv::aruco::detectMarkers(cv_ptr->image, dictionary_, corners, ids, detector_params_, rejected);

    // If no markers found, reset detection_active_ and return
    if (ids.empty()) {
      detection_active_ = false;
      return;
    }

    // If we have camera intrinsics, we can estimate pose
    if (has_camera_info_) {
      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(
        corners, marker_size_, camera_matrix_, dist_coeffs_, rvecs, tvecs);

      // Publish pose depending on our logic
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header = msg->header;  // Use the same timestamp/frame as the image

      // Go through each detected marker
      for (size_t i = 0; i < ids.size(); ++i) {
        // Convert the rvec/tvec to a Pose
        geometry_msgs::msg::Pose marker_pose = rvecTvecToPose(rvecs[i], tvecs[i]);

        if (use_first_detection_) {
          // Publish the first marker we detect and log it
          pose_msg.pose = marker_pose;
          // Only log if we haven't already
          if (!detection_active_) {
            RCLCPP_INFO(this->get_logger(),
              "Detected ID=%d at tvec=[%.3f, %.3f, %.3f]",
              ids[i], tvecs[i][0], tvecs[i][1], tvecs[i][2]);
          }
          detection_active_ = true; // We're now actively detecting
          pose_pub_->publish(pose_msg);
          return;
        } else {
          // Only publish if ID == dock_tag_id_
          if (ids[i] == dock_tag_id_) {
            pose_msg.pose = marker_pose;
            // Only log if we haven't already
            if (!detection_active_) {
              RCLCPP_INFO(this->get_logger(),
                "Detected ID=%d at tvec=[%.3f, %.3f, %.3f]",
                ids[i], tvecs[i][0], tvecs[i][1], tvecs[i][2]);
            }
            detection_active_ = true;
            pose_pub_->publish(pose_msg);
            return;
          }
        }
      }
    }
  }

  // Called when camera info arrives (intrinsics, distortion)
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    // Copy camera matrix
    camera_matrix_ = cv::Mat::zeros(3, 3, CV_64F);
    for (int i = 0; i < 9; ++i) {
      camera_matrix_.at<double>(i / 3, i % 3) = msg->k[i];
    }

    // Copy distortion coefficients (assuming 5)
    dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
    for (int i = 0; i < 5; ++i) {
      dist_coeffs_.at<double>(0, i) = msg->d[i];
    }

    has_camera_info_ = true;
  }

  // Utility: convert string -> OpenCV dictionary enum
  int getDictionaryId(const std::string & dict_name)
  {
    // Basic example that maps "6x6_250" to cv::aruco::DICT_6X6_250
    // Expand with more dictionary names as needed
    if (dict_name == "6x6_250" || dict_name == "DICT_6X6_250") {
      return cv::aruco::DICT_6X6_250;
    }
    RCLCPP_WARN(this->get_logger(), "Dictionary '%s' not recognized, defaulting to DICT_6X6_250",
                dict_name.c_str());
    return cv::aruco::DICT_6X6_250;
  }

  // Utility: convert rotation/translation vectors to a geometry_msgs pose
  geometry_msgs::msg::Pose rvecTvecToPose(const cv::Vec3d &rvec, const cv::Vec3d &tvec)
  {
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    // Convert rotation matrix -> quaternion
    tf2::Matrix3x3 tf2_mat(
      R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
      R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
      R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2)
    );
    tf2::Quaternion q;
    tf2_mat.getRotation(q);

    geometry_msgs::msg::Pose pose;
    pose.position.x = tvec[0];
    pose.position.y = tvec[1];
    pose.position.z = tvec[2];
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    return pose;
  }

private:
  // Parameters
  std::string dictionary_id_;
  double marker_size_;
  bool use_first_detection_;
  int dock_tag_id_;

  // ArUco detection
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_params_;

  // Camera intrinsics
  bool has_camera_info_ = false;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  // ROS pubs/subs
  image_transport::Subscriber image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DockPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
