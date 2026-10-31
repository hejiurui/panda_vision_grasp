#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <panda_vision_msgs/msg/detected_box.hpp>

namespace mgi = moveit::planning_interface;

static const double kDegToRad = M_PI / 180.0;
static const double kGraspZOffset = 0.1644;

class PickAndPlace : public rclcpp::Node
{
public:
  explicit PickAndPlace(const rclcpp::NodeOptions & options)
  : Node("pick_and_place", options)
  {
    target_color_ = declare_parameter<std::string>("target_color", "R");
    std::transform(
      target_color_.begin(), target_color_.end(), target_color_.begin(),
      [](unsigned char c) {return std::toupper(c);});
    scan_duration_ = declare_parameter<double>("scan_duration", 5.0);
    approach_height_ = declare_parameter<double>("approach_height", 0.12);
    grasp_z_offset_ = declare_parameter<double>("grasp_z_offset", 0.045);
    gripper_close_pos_ = declare_parameter<double>("gripper_close_pos", 0.025);
    drop_x_ = declare_parameter<double>("drop_x", -0.6);
    drop_y_ = declare_parameter<double>("drop_y", 0.0);
    drop_z_ = declare_parameter<double>("drop_z", 0.18);
    drop_approach_z_ = declare_parameter<double>("drop_approach_z", 0.42);
    base_frame_ = declare_parameter<std::string>("base_frame", "panda_link0");

    sub_ = create_subscription<panda_vision_msgs::msg::DetectedBox>(
      "/detected_boxes", 10,
      std::bind(&PickAndPlace::coordsCallback, this, std::placeholders::_1));

    home_joints_ = {
      0.0, 0.0, 0.0, -90.0 * kDegToRad, 0.0, 92.0 * kDegToRad,
      50.0 * kDegToRad};
    start_joints_ = {
      0.0, 0.0, 0.0, -75.0 * kDegToRad, 0.0, 105.0 * kDegToRad,
      45.0 * kDegToRad};

    RCLCPP_INFO(
      get_logger(), "pick_and_place started (friction grasp), target=%s",
      target_color_.c_str());
  }

  void run()
  {
    auto node = shared_from_this();

    arm_ = std::make_unique<mgi::MoveGroupInterface>(node, "arm");
    gripper_ = std::make_unique<mgi::MoveGroupInterface>(node, "gripper");

    arm_->setPoseReferenceFrame(base_frame_);
    arm_->setEndEffectorLink("panda_link7");
    arm_->setPlanningPipelineId("pilz_industrial_motion_planner");
    arm_->setPlannerId("PTP");
    arm_->setPlanningTime(3.0);
    arm_->setNumPlanningAttempts(3);
    arm_->setGoalPositionTolerance(0.004);
    arm_->setGoalOrientationTolerance(0.05);
    arm_->setMaxVelocityScalingFactor(0.2);
    arm_->setMaxAccelerationScalingFactor(0.1);

    RCLCPP_INFO(get_logger(), "Moving to scanning position...");
    if (!moveToJoints(start_joints_, "scanning position")) {
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(
      get_logger(), "Scanning %.1fs for '%s'...",
      scan_duration_, target_color_.c_str());
    std::promise<void> scan_done;
    auto scan_future = scan_done.get_future();
    auto scan_timer = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(scan_duration_)),
      [&scan_done]() {scan_done.set_value();});
    scan_future.wait();
    scan_timer->cancel();
    scanning_complete_ = true;

    geometry_msgs::msg::Point target;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto it = detected_boxes_.find(target_color_);
      if (it == detected_boxes_.end()) {
        RCLCPP_WARN(get_logger(), "Target %s not found!", target_color_.c_str());
        moveToJoints(home_joints_, "home");
        rclcpp::shutdown();
        return;
      }
      target = it->second;
    }

    RCLCPP_INFO(
      get_logger(), "Target %s locked at [%.3f, %.3f, %.3f]",
      target_color_.c_str(), target.x, target.y, target.z);

    executePickAndPlace(target);
    rclcpp::shutdown();
  }

private:
  void coordsCallback(const panda_vision_msgs::msg::DetectedBox::SharedPtr msg)
  {
    if (scanning_complete_) {
      return;
    }
    std::string color_id = msg->color_id;
    std::transform(
      color_id.begin(), color_id.end(), color_id.begin(),
      [](unsigned char c) {return std::toupper(c);});

    std::lock_guard<std::mutex> lock(mutex_);
    auto & acc = samples_[color_id];
    acc.x += msg->position.x;
    acc.y += msg->position.y;
    acc.z += msg->position.z;
    acc.n += 1;
    geometry_msgs::msg::Point mean;
    mean.x = acc.x / acc.n;
    mean.y = acc.y / acc.n;
    mean.z = acc.z / acc.n;
    detected_boxes_[color_id] = mean;
  }

  bool moveToJoints(const std::vector<double> & joints, const std::string & label)
  {
    arm_->setStartStateToCurrentState();
    arm_->setJointValueTarget(joints);
    if (!static_cast<bool>(arm_->move())) {
      RCLCPP_ERROR(get_logger(), "Failed: %s", label.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "OK: %s", label.c_str());
    return true;
  }

  bool moveToPose(const geometry_msgs::msg::Pose & pose, const std::string & label)
  {
    arm_->setStartStateToCurrentState();
    arm_->setPoseTarget(pose);
    const auto result = arm_->move();
    arm_->clearPoseTargets();
    if (!static_cast<bool>(result)) {
      RCLCPP_ERROR(get_logger(), "Failed: %s", label.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "OK: %s", label.c_str());
    return true;
  }

  bool moveGripper(bool open)
  {
    const double pos = open ? 0.04 : gripper_close_pos_;
    gripper_->setStartStateToCurrentState();
    gripper_->setMaxVelocityScalingFactor(0.15);
    gripper_->setMaxAccelerationScalingFactor(0.1);
    gripper_->setJointValueTarget(
      std::vector<std::string>{"panda_finger_joint1", "panda_finger_joint2"},
      std::vector<double>{pos, pos});
    if (!static_cast<bool>(gripper_->move())) {
      RCLCPP_ERROR(
        get_logger(), "Failed to %s gripper.", open ? "open" : "close");
      return false;
    }
    return true;
  }

  static geometry_msgs::msg::Pose makeGraspPose(
    double x, double y, double tcp_z)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = tcp_z + kGraspZOffset;
    pose.orientation.x = 0.0;
    pose.orientation.y = 1.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 0.0;
    return pose;
  }

  void executePickAndPlace(const geometry_msgs::msg::Point & target)
  {
    const double grasp_z = target.z + grasp_z_offset_;
    const geometry_msgs::msg::Pose pick_pose =
      makeGraspPose(target.x, target.y, grasp_z);
    const geometry_msgs::msg::Pose above_pose = makeGraspPose(
      target.x, target.y, grasp_z + approach_height_);
    const geometry_msgs::msg::Pose drop_high =
      makeGraspPose(drop_x_, drop_y_, drop_approach_z_);
    const geometry_msgs::msg::Pose drop_low =
      makeGraspPose(drop_x_, drop_y_, drop_z_);

    RCLCPP_INFO(
      get_logger(),
      "tcp_z=%.3f above[%.2f,%.2f,%.2f] grasp[%.2f,%.2f,%.2f]",
      grasp_z, above_pose.position.x, above_pose.position.y,
      above_pose.position.z, pick_pose.position.x, pick_pose.position.y,
      pick_pose.position.z);

    if (!moveToJoints(home_joints_, "home")) {return;}
    if (!moveGripper(true)) {return;}

    if (!moveToPose(above_pose, "above box")) {return;}
    if (!moveToPose(pick_pose, "grasp height")) {return;}

    RCLCPP_INFO(get_logger(), "Closing gripper (friction grasp)...");
    if (!moveGripper(false)) {return;}
    rclcpp::sleep_for(std::chrono::milliseconds(400));

    if (!moveToPose(above_pose, "lift")) {return;}
    if (!moveToJoints(home_joints_, "home")) {return;}

    if (!moveToPose(drop_high, "above trash")) {return;}
    if (!moveToPose(drop_low, "into trash")) {return;}
    rclcpp::sleep_for(std::chrono::milliseconds(250));

    RCLCPP_INFO(get_logger(), "Releasing object...");
    if (!moveGripper(true)) {return;}
    rclcpp::sleep_for(std::chrono::milliseconds(150));

    if (!moveToPose(drop_high, "out of trash")) {return;}
    if (!moveGripper(false)) {return;}
    if (!moveToJoints(home_joints_, "home")) {return;}
    RCLCPP_INFO(get_logger(), "Pick-and-place sequence complete.");
  }

  std::string target_color_;
  std::string base_frame_;
  double scan_duration_{5.0};
  double approach_height_{0.12};
  double grasp_z_offset_{0.045};
  double gripper_close_pos_{0.025};
  double drop_x_{-0.6};
  double drop_y_{0.0};
  double drop_z_{0.18};
  double drop_approach_z_{0.42};

  std::vector<double> start_joints_;
  std::vector<double> home_joints_;

  std::unique_ptr<mgi::MoveGroupInterface> arm_;
  std::unique_ptr<mgi::MoveGroupInterface> gripper_;

  std::mutex mutex_;
  std::map<std::string, geometry_msgs::msg::Point> detected_boxes_;
  struct Acc {double x{0}, y{0}, z{0}; int n{0};};
  std::map<std::string, Acc> samples_;
  bool scanning_complete_{false};

  rclcpp::Subscription<panda_vision_msgs::msg::DetectedBox>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickAndPlace>(rclcpp::NodeOptions());

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  node->run();

  spinner.join();
  rclcpp::shutdown();
  return 0;
}
