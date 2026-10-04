#pragma once
#include <cstdint>
#include <cstring>
#include "nav_msgs/msg/path.hpp"
namespace carbot_recovery_plugins {
// Stable bitwise fingerprint within this ROS deployment, excluding time stamps.
inline uint64_t pathId(const nav_msgs::msg::Path & path) {
  uint64_t hash = 14695981039346656037ULL;
  auto bytes = [&hash](const void * ptr, size_t count) {
    auto data = static_cast<const uint8_t *>(ptr);
    for (size_t i=0; i<count; ++i) {hash ^= data[i]; hash *= 1099511628211ULL;}
  };
  bytes(path.header.frame_id.data(), path.header.frame_id.size());
  for (const auto & p : path.poses) {
    for (double v : {p.pose.position.x, p.pose.position.y, p.pose.orientation.x,
      p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w}) {bytes(&v, sizeof(v));}
  }
  return hash;
}
}
