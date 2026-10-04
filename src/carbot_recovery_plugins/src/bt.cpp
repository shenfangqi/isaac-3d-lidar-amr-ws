#include "nav2_behavior_tree/bt_action_node.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "carbot_recovery_interfaces/action/bounded_recovery.hpp"
#include "carbot_recovery_plugins/path_id.hpp"
namespace carbot_recovery_plugins {
using Action=carbot_recovery_interfaces::action::BoundedRecovery;
class RecoveryNode : public nav2_behavior_tree::BtActionNode<Action> {
public:
  RecoveryNode(const std::string & name, const BT::NodeConfiguration & config)
  : BtActionNode<Action>(name,"bounded_recovery",config) {}
  static BT::PortsList providedPorts() {
    return providedBasicPorts({BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<nav_msgs::msg::Path>("path")});
  }
  void on_tick() override {
    nav_msgs::msg::Path path;
    should_send_goal_=getInput("goal",goal_.original_goal) && getInput("path",path) && !path.poses.empty();
    if (should_send_goal_) {goal_.failed_path_fingerprint=pathId(path);}
  }
  BT::NodeStatus on_success() override {
    return result_.result->recovered ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};
}
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<carbot_recovery_plugins::RecoveryNode>("CarbotBoundedRecovery");
}
