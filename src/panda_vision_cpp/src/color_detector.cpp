#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <panda_vision_msgs/msg/detected_box.hpp>

class ColorDetector : public rclcpp::Node
{
public:
  ColorDetector()
  : Node("color_detector")
  {
    fx_ = declare_parameter<double>("fx", 585.756);
    fy_ = declare_parameter<double>("fy", 585.756);
    cx_ = declare_parameter<double>("cx", 320.0);
    cy_ = declare_parameter<double>("cy", 160.0);
    plane_z_ = declare_parameter<double>("plane_z", 0.07);
    min_area_ = declare_parameter<double>("min_area", 80.0);
    show_image_ = declare_parameter<bool>("show_image", false);
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image_raw");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/camera/camera_info");
    optical_frame_ = declare_parameter<std::string>("optical_frame", "camera_link_optical");
    base_frame_ = declare_parameter<std::string>("base_frame", "panda_link0");

    color_ranges_ = {
      {"R", {cv::Scalar(0, 80, 60), cv::Scalar(12, 255, 255)}},
      {"G", {cv::Scalar(35, 60, 60), cv::Scalar(85, 255, 255)}},
      {"B", {cv::Scalar(95, 80, 60), cv::Scalar(130, 255, 255)}},
    };

    coords_pub_ = create_publisher<panda_vision_msgs::msg::DetectedBox>(
      "/detected_boxes", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, true);

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::QoS(1),
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (msg->k[0] > 0.0) {
          fx_ = msg->k[0];
          fy_ = msg->k[4];
          cx_ = msg->k[2];
          cy_ = msg->k[5];
          have_intrinsics_ = true;
        }
      });

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, 10,
      std::bind(&ColorDetector::imageCallback, this, std::placeholders::_1));

    if (show_image_) {
      cv::namedWindow("Color Detection", cv::WINDOW_NORMAL);
      cv::resizeWindow("Color Detection", 640, 320);
    }

    RCLCPP_INFO(
      get_logger(),
      "Color Detector started (plane_z=%.3f, fx=%.2f)", plane_z_, fx_);
  }

  ~ColorDetector() override
  {
    if (show_image_) {
      cv::destroyAllWindows();
    }
  }

private:
  bool lookupBaseFromOptical(geometry_msgs::msg::TransformStamped & t)
  {
    try {
      t = tf_buffer_->lookupTransform(
        base_frame_, optical_frame_, tf2::TimePointZero,
        tf2::durationFromSec(1.0));
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "TF lookup failed: %s", e.what());
      return false;
    }
  }

  bool pixelToBase(
    double u, double v, const geometry_msgs::msg::TransformStamped & t,
    tf2::Vector3 & out) const
  {
    const tf2::Quaternion q(
      t.transform.rotation.x, t.transform.rotation.y,
      t.transform.rotation.z, t.transform.rotation.w);
    const tf2::Matrix3x3 rot(q);
    const tf2::Vector3 trans(
      t.transform.translation.x, t.transform.translation.y,
      t.transform.translation.z);

    const double dx = (u - cx_) / fx_;
    const double dy = (v - cy_) / fy_;

    const tf2::Vector3 dir = rot * tf2::Vector3(dx, dy, 1.0);
    if (std::abs(dir.z()) < 1e-6) {
      return false;
    }
    const double s = (plane_z_ - trans.z()) / dir.z();
    if (s < 0.0 || s > 5.0) {
      return false;
    }
    out = trans + dir * s;
    out[2] = plane_z_;
    return true;
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat frame;
    try {
      frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to convert image: %s", e.what());
      return;
    }

    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    geometry_msgs::msg::TransformStamped t;
    const bool tf_ok = lookupBaseFromOptical(t);

    for (const auto & [color_id, range] : color_ranges_) {
      cv::Mat mask;
      cv::inRange(hsv, range.first, range.second, mask);

      cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
      cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

      std::vector<std::vector<cv::Point>> contours;
      std::vector<cv::Vec4i> hierarchy;
      cv::findContours(
        mask, contours, hierarchy, cv::RETR_EXTERNAL,
        cv::CHAIN_APPROX_SIMPLE);

      for (const auto & cnt : contours) {
        const double area = cv::contourArea(cnt);
        if (area <= min_area_) {
          continue;
        }

        const cv::Rect box = cv::boundingRect(cnt);
        const double u = box.x + box.width / 2.0;
        const double v = box.y + box.height / 2.0;

        cv::rectangle(frame, box.tl(), box.br(), cv::Scalar(0, 255, 255), 2);
        cv::putText(
          frame, color_id, cv::Point(box.x, box.y - 10),
          cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);

        if (!tf_ok) {
          continue;
        }

        tf2::Vector3 pt_base;
        if (!pixelToBase(u, v, t, pt_base)) {
          RCLCPP_WARN(
            get_logger(), "%s pixel(%.0f,%.0f) did not hit plane",
            color_id.c_str(), u, v);
          continue;
        }

        panda_vision_msgs::msg::DetectedBox out;
        out.header.stamp = msg->header.stamp;
        out.header.frame_id = base_frame_;
        out.color_id = color_id;
        out.position.x = pt_base.x();
        out.position.y = pt_base.y();
        out.position.z = pt_base.z();
        coords_pub_->publish(out);

        RCLCPP_INFO(
          get_logger(), "%s,%.3f,%.3f,%.3f (pix %.0f,%.0f)",
          color_id.c_str(), out.position.x, out.position.y, out.position.z,
          u, v);
      }
    }

    if (show_image_) {
      cv::imshow("Color Detection", frame);
      cv::waitKey(1);
    }
  }

  double fx_, fy_, cx_, cy_;
  double plane_z_, min_area_;
  bool show_image_{false};
  bool have_intrinsics_{false};
  std::string image_topic_, camera_info_topic_;
  std::string optical_frame_, base_frame_;

  std::map<std::string, std::pair<cv::Scalar, cv::Scalar>> color_ranges_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<panda_vision_msgs::msg::DetectedBox>::SharedPtr coords_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ColorDetector>());
  rclcpp::shutdown();
  return 0;
}
