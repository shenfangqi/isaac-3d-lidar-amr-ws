# Jetson 履带底盘通信链路

## 架构

```text
工作站 Nav2（ROS_DOMAIN_ID=0，CycloneDDS/LAN）
  -> /cmd_vel geometry_msgs/msg/Twist
  -> Jetson micro-ROS Agent（Fast DDS，UDP 8888）
  -> ESP32 micro-ROS 履带差速控制
```

ESP32 固件和真实履带车使用 Domain 0。Isaac 仿真也保留 Domain 0，但默认加载 `cyclonedds_ros_local.xml` 并只绑定 `lo`；真实机器人必须显式加载 `configs/cyclonedds_ros_jetson.xml` 才能进入物理 LAN。这种基于网络接口的隔离避免默认仿真节点意外进入物理机器人 ROS 图。

当前 LAN DDS 配置中的静态 Jetson peer 是 `192.168.1.109`。容器内没有可用的 mDNS NSS，不能在 CycloneDDS peer 中使用 `ubuntu.local`；Jetson DHCP 地址变化后必须先更新该 peer。SSH 连接仍使用 `isaac-jetson` 别名。

## Jetson Agent

Jetson 项目：

```text
/home/shenfq/Projects/carbot-ros2
```

Agent 用户服务：

```bash
ssh isaac-jetson 'systemctl --user status micro-ros-agent.service'
ssh isaac-jetson 'journalctl --user -u micro-ros-agent.service -n 100 --no-pager'
```

Jetson 侧只运行 Agent。不要启动 `carbot_driver`，因为它会成为额外的非零 `/cmd_vel` 发布者。

## 工作站真实机器人环境

在 `ros2-dev-humble` 等使用 host 网络的项目容器内：

```bash
source /opt/ros/humble/setup.bash
source /workspace/ros-humble/isaac_3d_lidar_amr_ws/install/local_setup.bash
source /workspace/ros-humble/isaac_3d_lidar_amr_ws/scripts/real_robot_ros_env.sh
```

当前 Nav2 的 `navigation_launch.py` 将 controller 输出重映射到 `/cmd_vel_nav`，再由 `velocity_smoother` 输出标准 `/cmd_vel`。履带控制端只订阅最终的 `/cmd_vel`。

## 启动真实 Nav2 前的硬门槛

阶段 E 已提供独立的 `launch/carbot_real.launch.py`、`configs/carbot/real.yaml`、实机 Nav2/AMCL 参数和系统时间配置；这些文件已通过静态与启动参数检查，但尚未在真实硬件上完成闭环联调。不得用仿真启动器控制履带车。允许真实 Nav2 加载物理 LAN DDS 配置前，必须逐项验证：

- ESP32 对 `/cmd_vel` 的履带差速换算、限速和指令超时停车已生效。
- 实体急停有效，第一次运动验证时履带架空。
- 真实 `/odom` 存在，时间戳和 `frame_id` 正确。
- `odom -> base_footprint` 是真实底盘 TF，不能使用 Isaac `/chassis/odom` relay。
- Mid-360 提供真实点云或 `/scan`，并使用实测 `base_link -> front_3d_lidar` 外参。
- Nav2、AMCL、RViz 和传感器全部使用系统时间，即 `use_sim_time=false`。
- AMCL 使用 `manual` 或经过测量的 `fixed` Initial Pose，不使用仿真的 `odom_identity`。
- 物理 LAN Domain 0 中只有一个最终 `/cmd_vel` 发布链路。

在这些门槛完成前，只允许检查 Agent、ROS 节点和 Topic endpoint，不发送任何非零速度命令。

阶段 F 训练环境不会替代以上门槛。持续待办与每项所需证据记录在
`isaac_lab/carbot_env/hardware_calibration_backlog.yaml`；仿真结果不能把其中
任何项目标记为完成，真实测量值必须回填 Carbot 公共参数和域随机化范围。
