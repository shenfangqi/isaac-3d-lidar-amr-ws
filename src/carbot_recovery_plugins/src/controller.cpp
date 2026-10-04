// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


// computeChecked is adapted from Navigation2 1.1.20, Apache-2.0.
// Only the collision exception type is changed; no log parsing or disabled checks.
#include <mutex>
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "carbot_recovery_interfaces/msg/controller_failure.hpp"
#include "carbot_recovery_interfaces/msg/runtime_state.hpp"
#include "carbot_recovery_plugins/path_id.hpp"
#include "tf2/utils.h"
namespace carbot_recovery_plugins {
using Base = nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController;
using Runtime = carbot_recovery_interfaces::msg::RuntimeState;
using Failure = carbot_recovery_interfaces::msg::ControllerFailure;
class VerifiedCollision : public nav2_core::PlannerException {
public: using nav2_core::PlannerException::PlannerException;
};
class EvidenceController : public Base {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) override {
    Base::configure(parent, name, tf, map);
    auto node = parent.lock();
    evidence_ = node->create_publisher<Failure>("/carbot_nav_recovery/controller_failure", 10);
    runtime_sub_ = node->create_subscription<Runtime>("/carbot_nav_recovery/runtime", 1,
      [this](Runtime::SharedPtr state) {std::lock_guard<std::mutex> lock(state_mutex_); runtime_ = *state;});
  }
  void activate() override {Base::activate(); evidence_->on_activate();}
  void deactivate() override {evidence_->on_deactivate(); Base::deactivate();}
  void cleanup() override {runtime_sub_.reset(); evidence_.reset(); Base::cleanup();}
  void setPlan(const nav_msgs::msg::Path & path) override {
    Base::setPlan(path);
    std::lock_guard<std::mutex> lock(state_mutex_);
    plan_id_ = pathId(path); plan_context_ = runtime_;
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & speed,
    nav2_core::GoalChecker * checker) override {
    try {return computeChecked(pose, speed, checker);}
    catch (const VerifiedCollision &) {emit(Failure::COLLISION); throw;}
    catch (...) {emit(Failure::OTHER); throw;}
  }
private:
  geometry_msgs::msg::TwistStamped computeChecked(const geometry_msgs::msg::PoseStamped &,
    const geometry_msgs::msg::Twist &, nav2_core::GoalChecker *);
  void emit(uint8_t cause) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    Failure msg; msg.header.stamp=clock_->now(); msg.header.frame_id=runtime_.header.frame_id;
    const double age=(clock_->now()-rclcpp::Time(runtime_.header.stamp)).seconds();
    bool valid=runtime_.ready && age>=0 && age<=0.5 &&
      runtime_.goal_id==plan_context_.goal_id && runtime_.localization_epoch==plan_context_.localization_epoch;
    msg.goal_id=valid ? runtime_.goal_id : "";
    msg.localization_epoch=valid ? runtime_.localization_epoch : "";
    msg.controller_id=plugin_name_; msg.path_fingerprint=plan_id_;
    msg.cause=cause; msg.collision_checked=(cause==Failure::COLLISION && valid);
    evidence_->publish(msg);
  }
  std::mutex state_mutex_;
  Runtime runtime_, plan_context_;
  uint64_t plan_id_{0};
  rclcpp::Subscription<Runtime>::SharedPtr runtime_sub_;
  rclcpp_lifecycle::LifecyclePublisher<Failure>::SharedPtr evidence_;
};
geometry_msgs::msg::TwistStamped EvidenceController::computeChecked(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed,
  nav2_core::GoalChecker * goal_checker)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  // Update for the current goal checker's state
  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist vel_tolerance;
  if (!goal_checker->getTolerances(pose_tolerance, vel_tolerance)) {
    RCLCPP_WARN(logger_, "Unable to retrieve goal checker's tolerances!");
  } else {
    goal_dist_tol_ = pose_tolerance.position.x;
  }

  // Transform path to robot base frame
  auto transformed_plan = transformGlobalPlan(pose);

  // Find look ahead distance and point on path and publish
  double lookahead_dist = getLookAheadDistance(speed);

  // Check for reverse driving
  if (allow_reversing_) {
    // Cusp check
    double dist_to_cusp = findVelocitySignChange(transformed_plan);

    // if the lookahead distance is further than the cusp, use the cusp distance instead
    if (dist_to_cusp < lookahead_dist) {
      lookahead_dist = dist_to_cusp;
    }
  }

  auto carrot_pose = getLookAheadPoint(lookahead_dist, transformed_plan);
  carrot_pub_->publish(createCarrotMsg(carrot_pose));

  double linear_vel, angular_vel;

  // Find distance^2 to look ahead point (carrot) in robot base frame
  // This is the chord length of the circle
  const double carrot_dist2 =
    (carrot_pose.pose.position.x * carrot_pose.pose.position.x) +
    (carrot_pose.pose.position.y * carrot_pose.pose.position.y);

  // Find curvature of circle (k = 1 / R)
  double curvature = 0.0;
  if (carrot_dist2 > 0.001) {
    curvature = 2.0 * carrot_pose.pose.position.y / carrot_dist2;
  }

  // Setting the velocity direction
  double sign = 1.0;
  if (allow_reversing_) {
    sign = carrot_pose.pose.position.x >= 0.0 ? 1.0 : -1.0;
  }

  linear_vel = desired_linear_vel_;

  // Make sure we're in compliance with basic constraints
  double angle_to_heading;
  if (shouldRotateToGoalHeading(carrot_pose)) {
    double angle_to_goal = tf2::getYaw(transformed_plan.poses.back().pose.orientation);
    rotateToHeading(linear_vel, angular_vel, angle_to_goal, speed);
  } else if (shouldRotateToPath(carrot_pose, angle_to_heading)) {
    rotateToHeading(linear_vel, angular_vel, angle_to_heading, speed);
  } else {
    applyConstraints(
      curvature, speed,
      costAtPose(pose.pose.position.x, pose.pose.position.y), transformed_plan,
      linear_vel, sign);

    // Apply curvature to angular velocity after constraining linear velocity
    angular_vel = linear_vel * curvature;
  }

  // Collision checking on this velocity heading
  const double & carrot_dist = hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  if (use_collision_detection_ && isCollisionImminent(pose, linear_vel, angular_vel, carrot_dist)) {
    throw VerifiedCollision("RPP collision check rejected command");
  }

  // populate and return message
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;
  return cmd_vel;
}

}
PLUGINLIB_EXPORT_CLASS(carbot_recovery_plugins::EvidenceController, nav2_core::Controller)
