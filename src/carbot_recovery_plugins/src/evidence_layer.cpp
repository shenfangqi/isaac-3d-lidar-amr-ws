#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "carbot_recovery_interfaces/msg/costmap_evidence.hpp"
#include "pluginlib/class_list_macros.hpp"
namespace carbot_recovery_plugins {
class EvidenceLayer : public nav2_costmap_2d::Layer {
public:
  void onInitialize() override {
    current_=true; enabled_=true;
    pub_=node_.lock()->create_publisher<carbot_recovery_interfaces::msg::CostmapEvidence>(
      "/carbot_nav_recovery/costmap_evidence", rclcpp::QoS(1));
    pub_->on_activate();
  }
  void reset() override {current_=false;}
  bool isClearable() override {return false;}
  void updateBounds(double, double, double, double *, double *, double *, double *) override {}
  void updateCosts(nav2_costmap_2d::Costmap2D & grid, int, int, int, int) override {
    current_=true;
    auto plugins=layered_costmap_->getPlugins();
    // Must run after every obstacle/static/inflation layer; never change costs.
    if (plugins->empty() || plugins->back().get()!=this) {return;}
    if (grid.getSizeInCellsX()*grid.getSizeInCellsY()>1000000) {return;}
    carbot_recovery_interfaces::msg::CostmapEvidence msg;
    msg.grid.header.stamp=clock_->now();
    msg.grid.header.frame_id=layered_costmap_->getGlobalFrameID();
    msg.grid.metadata.update_time=msg.grid.header.stamp;
    msg.grid.metadata.size_x=grid.getSizeInCellsX();
    msg.grid.metadata.size_y=grid.getSizeInCellsY();
    msg.grid.metadata.resolution=grid.getResolution();
    msg.grid.metadata.origin.position.x=grid.getOriginX();
    msg.grid.metadata.origin.position.y=grid.getOriginY();
    msg.grid.metadata.origin.orientation.w=1.0;
    msg.grid.data.assign(grid.getCharMap(),grid.getCharMap()+grid.getSizeInCellsX()*grid.getSizeInCellsY());
    for (auto & p : getFootprint()) {
      geometry_msgs::msg::Point32 point; point.x=p.x; point.y=p.y;
      msg.footprint.points.push_back(point);
    }
    msg.layers_current=layered_costmap_->isCurrent();
    msg.update_sequence=++sequence_;
    pub_->publish(msg);
  }
private:
  uint64_t sequence_{0};
  rclcpp_lifecycle::LifecyclePublisher<carbot_recovery_interfaces::msg::CostmapEvidence>::SharedPtr pub_;
};
}
PLUGINLIB_EXPORT_CLASS(carbot_recovery_plugins::EvidenceLayer, nav2_costmap_2d::Layer)
