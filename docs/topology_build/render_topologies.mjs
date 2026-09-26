import fs from "node:fs/promises";
import path from "node:path";
import { instance } from "@viz-js/viz";

const outDir = "/home/shenfq/projects/ros-humble/isaac_3d_lidar_amr_ws/docs/topology";
await fs.mkdir(outDir, { recursive: true });

const common = String.raw`
  graph [bgcolor="transparent", pad="0.18", nodesep="0.22", ranksep="0.38", splines=ortho, fontname="Noto Sans CJK SC"];
  node [shape=box, style="rounded,filled", fontname="Noto Sans CJK SC", fontsize=13, margin="0.12,0.07", color="#B8BCC4", penwidth=1.1, fillcolor="#FFFFFF", fontcolor="#111111"];
  edge [fontname="Noto Sans CJK SC", fontsize=11, color="#667085", fontcolor="#475467", arrowsize=0.65, penwidth=1.25];
`;

const diagrams = {
  "saved-map-navigation": String.raw`digraph G {
    ${common}
    rankdir=LR;
    subgraph cluster_sim {
      label="Isaac Sim（传感器 / 机器人）"; color="#B9DDFC"; style="rounded,filled"; fillcolor="#F2F8FE";
      sim [label="Isaac Sim\nRTX LiDAR + chassis", fillcolor="#DCEEFF"];
      lidar [label="/front_3d_lidar/lidar_points\nPointCloud2", shape=note, fillcolor="#EAF5FB"];
      chassis_odom [label="/chassis/odom\nOdometry", shape=note, fillcolor="#EAF5FB"];
      clock [label="/clock", shape=note, fillcolor="#EAF5FB"];
      robot_cmd [label="/cmd_vel\nTwist", shape=note, fillcolor="#FFF3D6"];
      sim -> lidar [label="发布"];
      sim -> chassis_odom [label="发布"];
      sim -> clock [label="发布"];
      robot_cmd -> sim [label="订阅"];
    }
    subgraph cluster_nvblox {
      label="Isaac ROS / nvblox"; color="#98E1C2"; style="rounded,filled"; fillcolor="#F0FBF6";
      padder [label="pointcloud_padder", fillcolor="#D9F7E9"];
      padded [label="/front_3d_lidar/\nlidar_points_nvblox\nPointCloud2 1800×31", shape=note, fillcolor="#E7FAF1"];
      nvblox [label="nvblox_container\n└ nvblox_node", fillcolor="#CFF4E3"];
      nvblox_map [label="/nvblox_node/\nstatic_occupancy_grid\nOccupancyGrid", shape=note, fillcolor="#E7FAF1"];
      nvblox_vis [label="mesh / ESDF / slice\n可视化 topics", shape=note, fillcolor="#E7FAF1"];
      load [label="/nvblox_node/load_map\nService", shape=component, fillcolor="#E7FAF1"];
      lidar -> padder;
      padder -> padded;
      padded -> nvblox;
      nvblox -> nvblox_map;
      nvblox -> nvblox_vis;
      load -> nvblox [style=dashed, label="warehouse_v3.nvblx"];
    }
    subgraph cluster_nav {
      label="ROS 2 Humble / 保存地图导航"; color="#F5D074"; style="rounded,filled"; fillcolor="#FFF9EB";
      scan_node [label="pointcloud_to_laserscan", fillcolor="#FFF0C2"];
      scan [label="/scan\nLaserScan", shape=note, fillcolor="#FFF6DC"];
      relay [label="topic_tools relay", fillcolor="#FFF0C2"];
      odom [label="/odom\nOdometry", shape=note, fillcolor="#FFF6DC"];
      map_server [label="map_server\nwarehouse_v3.yaml", fillcolor="#FFF0C2"];
      map [label="/map\nOccupancyGrid", shape=note, fillcolor="#FFF6DC"];
      loc [label="定位分支\n① ground_truth static TF\n② AMCL + initializer", fillcolor="#E9E1FF", color="#C8B5F8"];
      tf [label="/tf + /tf_static\nmap → odom → base_link\n→ front_3d_lidar", shape=note, fillcolor="#F2EDFF"];
      bt [label="bt_navigator", fillcolor="#FFE7A3"];
      planner [label="planner_server\n+ global_costmap", fillcolor="#FFE7A3"];
      controller [label="controller_server\n+ local_costmap", fillcolor="#FFE7A3"];
      behaviors [label="behavior_server\nspin / backup / wait", fillcolor="#FFE7A3"];
      smoother [label="velocity_smoother", fillcolor="#FFE7A3"];
      life [label="lifecycle_manager_navigation", fillcolor="#FFF0C2"];
      goal [label="/navigate_to_pose\nAction", shape=component, fillcolor="#FFF6DC"];
      lidar -> scan_node;
      scan_node -> scan;
      chassis_odom -> relay;
      relay -> odom;
      map_server -> map;
      map -> loc [label="AMCL 订阅", style=dashed];
      scan -> loc [label="AMCL 订阅", style=dashed];
      chassis_odom -> loc [label="initializer", style=dashed];
      loc -> tf [label="map→odom"];
      sim -> tf [label="odom→base / 静态外参", style=dashed];
      goal -> bt;
      bt -> planner [label="ComputePathToPose"];
      planner -> controller [label="FollowPath"];
      scan -> controller [label="local costmap"];
      scan -> planner [label="global costmap"];
      map -> planner [label="static layer"];
      odom -> bt;
      odom -> smoother;
      controller -> smoother [label="速度指令"];
      behaviors -> smoother [label="恢复行为"];
      smoother -> robot_cmd;
      life -> bt [style=dotted, arrowhead=none];
      life -> planner [style=dotted, arrowhead=none];
      life -> controller [style=dotted, arrowhead=none];
      tf -> bt [style=dashed];
      tf -> planner [style=dashed];
      tf -> controller [style=dashed];
    }
    clock -> nvblox [style=dotted];
    clock -> bt [style=dotted];
  }`,
  "live-exploration": String.raw`digraph G {
    ${common}
    rankdir=LR;
    subgraph cluster_inputs {
      label="实时输入"; color="#B9DDFC"; style="rounded,filled"; fillcolor="#F2F8FE";
      sim [label="Isaac Sim / 真机驱动", fillcolor="#DCEEFF"];
      cloud [label="/front_3d_lidar/lidar_points\nPointCloud2", shape=note, fillcolor="#EAF5FB"];
      odom_src [label="/chassis/odom\nOdometry", shape=note, fillcolor="#EAF5FB"];
      tf [label="/tf + /tf_static\nodom → base_link → lidar", shape=note, fillcolor="#F2EDFF"];
      sim -> cloud; sim -> odom_src; sim -> tf;
    }
    subgraph cluster_map {
      label="实时 nvblox 建图"; color="#98E1C2"; style="rounded,filled"; fillcolor="#F0FBF6";
      padder [label="pointcloud_padder", fillcolor="#D9F7E9"];
      padded [label="/front_3d_lidar/\nlidar_points_nvblox", shape=note, fillcolor="#E7FAF1"];
      nvblox [label="nvblox_node\nTSDF / ESDF / 2D slice", fillcolor="#CFF4E3"];
      occ [label="/nvblox_node/\nstatic_occupancy_grid\nOccupancyGrid（Volatile）", shape=note, fillcolor="#E7FAF1"];
      vis [label="static_esdf_pointcloud\nmesh / slice topics", shape=note, fillcolor="#E7FAF1"];
      cloud -> padder -> padded -> nvblox;
      tf -> nvblox [style=dashed, label="位姿"];
      nvblox -> occ; nvblox -> vis;
    }
    subgraph cluster_nav {
      label="Nav2（global_frame = odom）"; color="#F5D074"; style="rounded,filled"; fillcolor="#FFF9EB";
      scan_node [label="exploration_pointcloud_to_laserscan", fillcolor="#FFF0C2"];
      scan [label="/scan\nLaserScan", shape=note, fillcolor="#FFF6DC"];
      relay [label="topic_tools relay", fillcolor="#FFF0C2"];
      odom [label="/odom\nOdometry", shape=note, fillcolor="#FFF6DC"];
      bt [label="bt_navigator", fillcolor="#FFE7A3"];
      planner [label="planner_server\n+ global_costmap", fillcolor="#FFE7A3"];
      controller [label="controller_server\n+ local_costmap", fillcolor="#FFE7A3"];
      behavior [label="behavior_server", fillcolor="#FFE7A3"];
      smooth [label="velocity_smoother", fillcolor="#FFE7A3"];
      cmd [label="/cmd_vel\nTwist → robot", shape=note, fillcolor="#FFF3D6"];
      cloud -> scan_node -> scan;
      odom_src -> relay -> odom;
      occ -> planner [label="static layer"];
      scan -> planner; scan -> controller;
      odom -> bt; odom -> smooth;
      bt -> planner [label="ComputePathToPose"];
      planner -> controller [label="FollowPath"];
      controller -> smooth;
      behavior -> smooth;
      smooth -> cmd;
    }
    subgraph cluster_explore {
      label="前沿探索（默认 disabled）"; color="#C8B5F8"; style="rounded,filled"; fillcolor="#F7F4FF";
      explorer [label="frontier_explorer", fillcolor="#E9E1FF"];
      action [label="/navigate_to_pose\nAction client", shape=component, fillcolor="#F2EDFF"];
      fronts [label="/frontier_explorer/frontiers\nMarkerArray", shape=note, fillcolor="#F2EDFF"];
      status [label="/frontier_explorer/status\nString（Transient Local）", shape=note, fillcolor="#F2EDFF"];
      enable [label="/frontier_explorer/set_enabled\nSetBool Service", shape=component, fillcolor="#F2EDFF"];
      occ -> explorer [label="地图"];
      tf -> explorer [style=dashed, label="odom→base_link"];
      enable -> explorer [style=dashed];
      explorer -> action;
      action -> bt;
      explorer -> fronts;
      explorer -> status;
      explorer -> cmd [style=dotted, arrowhead=none, label="只检查发布者，不直接发布"];
    }
  }`,
  "optional-and-interfaces": String.raw`digraph G {
    ${common}
    rankdir=LR;
    subgraph cluster_kiss {
      label="可选 KISS-ICP LiDAR Odometry"; color="#B9DDFC"; style="rounded,filled"; fillcolor="#F2F8FE";
      cloud [label="配置参数 topic\nPointCloud2", shape=note, fillcolor="#EAF5FB"];
      kiss [label="kiss_icp_node", fillcolor="#DCEEFF"];
      k_odom [label="/kiss/odometry\nOdometry", shape=note, fillcolor="#EAF5FB"];
      k_frame [label="/kiss/frame", shape=note, fillcolor="#EAF5FB"];
      k_key [label="/kiss/keypoints", shape=note, fillcolor="#EAF5FB"];
      k_map [label="/kiss/local_map", shape=note, fillcolor="#EAF5FB"];
      k_tf [label="/tf：odom_lidar ↔ base/lidar\n（publish_odom_tf 可选）", shape=note, fillcolor="#F2EDFF"];
      cloud -> kiss;
      kiss -> k_odom; kiss -> k_frame; kiss -> k_key; kiss -> k_map; kiss -> k_tf;
    }
    subgraph cluster_services {
      label="地图 / 生命周期 Service"; color="#98E1C2"; style="rounded,filled"; fillcolor="#F0FBF6";
      nvblox [label="nvblox_node", fillcolor="#CFF4E3"];
      load [label="load_map", shape=component, fillcolor="#E7FAF1"];
      save [label="save_map", shape=component, fillcolor="#E7FAF1"];
      ply [label="save_ply", shape=component, fillcolor="#E7FAF1"];
      amcl [label="amcl", fillcolor="#D9F7E9"];
      amcl_state [label="/amcl/get_state", shape=component, fillcolor="#E7FAF1"];
      init [label="amcl_pose_initializer", fillcolor="#D9F7E9"];
      load -> nvblox; save -> nvblox; ply -> nvblox; init -> amcl_state -> amcl;
    }
    subgraph cluster_actions {
      label="Nav2 Action 主链"; color="#F5D074"; style="rounded,filled"; fillcolor="#FFF9EB";
      client [label="RViz / frontier_explorer / API", fillcolor="#FFF0C2"];
      nav [label="/navigate_to_pose\nbt_navigator", shape=component, fillcolor="#FFF6DC"];
      plan [label="/compute_path_to_pose\nplanner_server", shape=component, fillcolor="#FFF6DC"];
      follow [label="/follow_path\ncontroller_server", shape=component, fillcolor="#FFF6DC"];
      spin [label="/spin · /backup · /wait\nbehavior_server", shape=component, fillcolor="#FFF6DC"];
      cmd [label="velocity_smoother → /cmd_vel", fillcolor="#FFE7A3"];
      client -> nav -> plan -> follow -> cmd;
      nav -> spin -> cmd;
    }
    subgraph cluster_tf {
      label="坐标树契约"; color="#C8B5F8"; style="rounded,filled"; fillcolor="#F7F4FF";
      map [label="map", fillcolor="#E9E1FF"];
      odom [label="odom", fillcolor="#E9E1FF"];
      base [label="base_link", fillcolor="#E9E1FF"];
      lidar [label="front_3d_lidar", fillcolor="#E9E1FF"];
      map -> odom [label="ground_truth static 或 AMCL"];
      odom -> base [label="Isaac / 里程计"];
      base -> lidar [label="静态外参"];
    }
  }`,
};

const viz = await instance();
for (const [name, source] of Object.entries(diagrams)) {
  const svg = viz.renderString(source, { format: "svg", engine: "dot" });
  await fs.writeFile(path.join(outDir, `${name}.svg`), svg);
}

