#include <atomic>
#include <chrono>
#include <future>
#include <mutex>
#include <thread>
#include <cmath>
#include "nav2_core/behavior.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "carbot_recovery_interfaces/action/bounded_recovery.hpp"
#include "carbot_recovery_interfaces/srv/recovery_step.hpp"

namespace carbot_recovery_plugins {
using Action = carbot_recovery_interfaces::action::BoundedRecovery;
using Step = carbot_recovery_interfaces::srv::RecoveryStep;
using Handle = rclcpp_action::ServerGoalHandle<Action>;
using namespace std::chrono_literals;

// No velocity publisher exists unless explicitly configured for execution.
// A fresh coordinator response is required before every bounded command.
class BoundedRecovery : public nav2_core::Behavior {
public:
  ~BoundedRecovery() override {shutdown();}
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & name, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::CostmapTopicCollisionChecker>) override {
    node_=parent.lock();
    nav2_util::declare_parameter_if_not_declared(node_, name+".execution_enabled", rclcpp::ParameterValue(false));
    enabled_=node_->get_parameter(name+".execution_enabled").as_bool();
    if (enabled_) {velocity_=node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);}
    client_=node_->create_client<Step>("/carbot_nav_recovery/step");
    server_=rclcpp_action::create_server<Action>(node_, name,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>) {
        bool expected=false;
        if (!active_ || !enabled_ || !client_->service_is_ready() ||
          !busy_.compare_exchange_strong(expected,true)) {return rclcpp_action::GoalResponse::REJECT;}
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<Handle>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](const std::shared_ptr<Handle> goal) {
        if (worker_.joinable()) {worker_.join();}
        worker_=std::thread([this,goal] {execute(goal);});
      });
  }
  void activate() override {
    if (velocity_) {velocity_->on_activate();}
    active_=true;
  }
  void deactivate() override {
    shutdown();
    if (velocity_) {velocity_->on_deactivate();}
  }
  void cleanup() override {
    shutdown(); server_.reset(); client_.reset(); velocity_.reset(); node_.reset();
  }
private:
  void zero() {
    if (velocity_ && velocity_->is_activated()) {velocity_->publish(geometry_msgs::msg::Twist());}
  }
  void shutdown() {
    active_=false; zero();
    if (worker_.joinable()) {worker_.join();}
    zero();
  }
  std::shared_ptr<Step::Response> call(const std::shared_ptr<Step::Request> & request,
    const std::shared_ptr<Handle> & handle, bool stopping=false) {
    auto future=client_->async_send_request(request);
    auto deadline=std::chrono::steady_clock::now()+150ms;
    while (future.wait_for(5ms)!=std::future_status::ready) {
      if (stopping) {zero();}
      if (std::chrono::steady_clock::now()>=deadline ||
        (!stopping && (!active_ || handle->is_canceling()))) {
        client_->remove_pending_request(future); zero(); return nullptr;
      }
    }
    return future.get();
  }
  void execute(const std::shared_ptr<Handle> & handle) {
    const auto start=std::chrono::steady_clock::now();
    auto request=std::make_shared<Step::Request>();
    for (auto byte : handle->get_goal_id()) {
      const char hex[]="0123456789abcdef";
      request->token+=hex[byte>>4]; request->token+=hex[byte&15];
    }
    request->original_goal=handle->get_goal()->original_goal;
    request->failed_path_fingerprint=handle->get_goal()->failed_path_fingerprint;
    request->operation=Step::Request::START;
    auto result=std::make_shared<Action::Result>();
    result->reason="COORDINATOR_UNAVAILABLE";
    zero();
    try {
      while (active_ && rclcpp::ok() && !handle->is_canceling()) {
        if (std::chrono::steady_clock::now()-start>=30s) {result->reason="TIME_BUDGET"; break;}
        auto response=call(request,handle);
        if (!response) {result->reason="STEP_TIMEOUT"; break;}
        if (response->status!=Step::Response::RUNNING) {
          result->recovered=response->status==Step::Response::SUCCEEDED;
          result->reason=response->reason; break;
        }
        const auto & c=response->command;
        if (!std::isfinite(c.linear.x) || !std::isfinite(c.angular.z) ||
          std::abs(c.linear.x)>0.1 || std::abs(c.angular.z)>0.5 ||
          c.linear.y!=0 || c.linear.z!=0 || c.angular.x!=0 || c.angular.y!=0) {
          result->reason="INVALID_COMMAND"; break;
        }
        if (!active_ || handle->is_canceling()) {break;}
        if (std::chrono::steady_clock::now()-start>=30s) {result->reason="TIME_BUDGET"; break;}
        velocity_->publish(c);
        auto feedback=std::make_shared<Action::Feedback>();
        feedback->phase=response->reason; feedback->distance_used_m=response->distance_used_m;
        handle->publish_feedback(feedback);
        request->operation=Step::Request::TICK;
        std::this_thread::sleep_for(50ms);
      }
    } catch (const std::exception & e) {result->reason=std::string("EXECUTOR_ERROR: ")+e.what();}
    zero();
    request->operation=Step::Request::STOP;
    // Continue zeros during the stop handshake; never hand back a nonzero latch.
    bool stopped_ack=false;
    try {
      auto response=call(request,handle,true);
      stopped_ack=response && response->reason=="STOPPED";
    } catch (...) {}
    if (result->recovered && !stopped_ack) {
      result->recovered=false; result->reason="STOP_HANDSHAKE_FAILED";
    }
    for (int i=0; i<3; ++i) {zero(); std::this_thread::sleep_for(50ms);}
    result->total_elapsed_time=rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now()-start));
    try {
      if (handle->is_canceling()) {result->recovered=false; result->reason="CANCELED"; handle->canceled(result);}
      else if (result->recovered && active_) {handle->succeed(result);}
      else {result->recovered=false; handle->abort(result);}
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node_->get_logger(), "Recovery result delivery failed: %s", e.what());
    }
    busy_=false;
  }
  bool enabled_{false};
  std::atomic<bool> active_{false}, busy_{false};
  std::thread worker_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr velocity_;
  rclcpp::Client<Step>::SharedPtr client_;
  rclcpp_action::Server<Action>::SharedPtr server_;
};
}
PLUGINLIB_EXPORT_CLASS(carbot_recovery_plugins::BoundedRecovery, nav2_core::Behavior)
