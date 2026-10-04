#include <cstdlib>
#include <iostream>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav2_core/behavior.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

int main(int argc, char ** argv) {
  const char * domain=std::getenv("ROS_DOMAIN_ID");
  if (!domain || std::string(domain)!="73") {return 2;}
  rclcpp::init(argc,argv);
  int result=0;
  try {
    pluginlib::ClassLoader<nav2_core::Behavior> behaviors("nav2_core","nav2_core::Behavior");
    pluginlib::ClassLoader<nav2_core::Controller> controllers("nav2_core","nav2_core::Controller");
    pluginlib::ClassLoader<nav2_costmap_2d::Layer> layers("nav2_costmap_2d","nav2_costmap_2d::Layer");
    auto controller=controllers.createSharedInstance("carbot_recovery_plugins/EvidenceController");
    auto layer=layers.createSharedInstance("carbot_recovery_plugins/EvidenceLayer");
    auto behavior=behaviors.createSharedInstance("carbot_recovery_plugins/BoundedRecovery");
    auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("recovery_plugin_smoke");
    auto tf=std::make_shared<tf2_ros::Buffer>(node->get_clock());
    behavior->configure(node,"bounded_recovery",tf,nullptr);
    behavior->activate();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    if (!node->get_publishers_info_by_topic("/cmd_vel").empty() ||
      !node->get_publishers_info_by_topic("/cmd_vel_nav").empty()) {
      throw std::runtime_error("disabled behavior unexpectedly created velocity publisher");
    }
    behavior->deactivate(); behavior->cleanup();
    BT::BehaviorTreeFactory factory;
    factory.registerFromPlugin("libcarbot_bounded_recovery_bt_node.so");
    if (!factory.manifests().count("CarbotBoundedRecovery")) {
      throw std::runtime_error("BT registration missing");
    }
    std::cout << "PASS: controller/layer/behavior load, disabled lifecycle, no velocity publisher, BT registration\n";
  } catch (const std::exception & e) {std::cerr<<e.what()<<"\n"; result=1;}
  rclcpp::shutdown();
  return result;
}
