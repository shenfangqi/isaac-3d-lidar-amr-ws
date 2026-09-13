import fs from "node:fs/promises";
import { readFileSync } from "node:fs";
import { Presentation, PresentationFile } from "@oai/artifact-tool";

const root = "/home/shenfq/projects/ros-humble/isaac_3d_lidar_amr_ws";
const outDir = `${root}/docs/topology`;
const finalPptx = `${outDir}/isaac_3d_lidar_amr_ros2_topology.pptx`;
const renderDir = `${root}/docs/topology_build/rendered`;
await fs.mkdir(outDir, { recursive: true });
await fs.mkdir(renderDir, { recursive: true });

const deck = Presentation.create({ slideSize: { width: 1280, height: 720 } });

function addText(slide, name, text, position, style) {
  const shape = slide.shapes.add({
    name,
    geometry: "textbox",
    position,
    fill: "none",
    line: { style: "solid", fill: "none", width: 0 },
  });
  shape.text = text;
  shape.text.style = { typeface: "Helvetica Neue", color: "#000000", ...style };
  return shape;
}

function addFooter(slide, page) {
  addText(slide, `footer-${page}`, String(page), { left: 1184, top: 659, width: 54, height: 25 }, {
    fontSize: 14, alignment: "right", verticalAlignment: "bottom",
  });
}

function addDiagramSlide(title, subtitle, svgFile, page, notes) {
  const slide = deck.slides.add();
  slide.background.fill = "#FFFFFF";
  addText(slide, `title-${page}`, title, { left: 41, top: 28, width: 1197, height: 62 }, {
    fontSize: 38, bold: true, verticalAlignment: "top",
  });
  addText(slide, `subtitle-${page}`, subtitle, { left: 42, top: 92, width: 1197, height: 42 }, {
    fontSize: 18, color: "#475467", verticalAlignment: "top",
  });
  slide.shapes.add({
    name: `rule-${page}`,
    geometry: "rect",
    position: { left: 41, top: 137, width: 1197, height: 2 },
    fill: "#B8BCC4",
    line: { style: "solid", fill: "none", width: 0 },
  });
  slide.images.add({
    svg: readFileSync(svgFile, "utf8"),
    alt: `${title}：节点、Topic、Action、Service 与 TF 的关系图`,
    fit: "contain",
    position: { left: 35, top: 146, width: 1210, height: 500 },
  });
  addFooter(slide, page);
  slide.speakerNotes.textFrame.setText(notes);
  return slide;
}

const cover = deck.slides.add();
cover.background.fill = "#FFFFFF";
addText(cover, "cover-kicker", "ROS 2 / ISAAC SIM / NVBLOX / NAV2", { left: 41, top: 41, width: 700, height: 50 }, {
  fontSize: 24, bold: true, color: "#3D8DFF",
});
addText(cover, "cover-title", "Isaac 3D LiDAR AMR\n节点与 Topic 拓扑图", { left: 41, top: 180, width: 1050, height: 240 }, {
  fontSize: 64, bold: true, verticalAlignment: "bottom",
});
addText(cover, "cover-subtitle", "按运行模式拆分：保存地图导航 · 实时前沿探索 · 可选 KISS-ICP", { left: 41, top: 500, width: 1000, height: 65 }, {
  fontSize: 25, color: "#475467",
});
addText(cover, "cover-date", "warehouse_v3 · ROS 2 Humble · 2026-08-29", { left: 41, top: 610, width: 800, height: 40 }, {
  fontSize: 16, color: "#667085",
});
addFooter(cover, 1);
cover.speakerNotes.textFrame.setText("[Sources]\n- Local workspace launch files and node sources under /home/shenfq/projects/ros-humble/isaac_3d_lidar_amr_ws\n[/Sources]");

addDiagramSlide(
  "默认保存地图导航：传感器、定位、规划与控制形成一条闭环",
  "默认 localization_mode=ground_truth；AMCL 是可替换分支。nvblox 地图用于 3D/2.5D 可视化，Nav2 静态规划使用 /map。",
  `${outDir}/saved-map-navigation.svg`,
  2,
  "[Sources]\n- launch/nav_stack.launch.py\n- launch/nvblox_with_map.launch.py\n- src/isaac_3d_lidar_bringup/launch/xt32_nvblox.launch.py\n- configs/nav2_params.yaml\n- configs/amcl_params.yaml\n[/Sources]",
);

addDiagramSlide(
  "实时建图与前沿探索：OccupancyGrid 驱动目标选择，Nav2 负责运动",
  "frontier_explorer 默认禁用；启用后只发送 NavigateToPose，不直接发布 /cmd_vel，并检查控制话题是否存在未授权发布者。",
  `${outDir}/live-exploration.svg`,
  3,
  "[Sources]\n- src/isaac_3d_lidar_exploration/launch/explore_nvblox.launch.py\n- src/isaac_3d_lidar_exploration/isaac_3d_lidar_exploration/frontier_explorer_node.py\n- src/isaac_3d_lidar_exploration/config/nav2_live_mapping.yaml\n- src/isaac_3d_lidar_exploration/config/frontier_explorer.yaml\n[/Sources]",
);

addDiagramSlide(
  "可选里程计与接口契约：Action、Service 和 TF 把子系统连接起来",
  "KISS-ICP 当前是独立可选链路；接入主栈时必须明确 odom 来源和 TF 所有权，避免重复发布 map→odom 或 odom→base_link。",
  `${outDir}/optional-and-interfaces.svg`,
  4,
  "[Sources]\n- src/kiss-icp/ros/launch/odometry.launch.py\n- src/kiss-icp/ros/src/OdometryServer.cpp\n- src/isaac_3d_lidar_bringup/isaac_3d_lidar_bringup/amcl_pose_initializer.py\n- docs/map_gen/README.md\n- docs/map_nav/README.md\n[/Sources]",
);

const summary = deck.slides.add();
summary.background.fill = "#FFFFFF";
addText(summary, "summary-title", "读图时先确认运行模式，再核对四个单一所有权", { left: 41, top: 36, width: 1197, height: 85 }, {
  fontSize: 40, bold: true,
});
const items = [
  ["01", "地图来源", "保存地图导航：/map 来自 map_server；实时探索：全局代价地图来自 nvblox OccupancyGrid。"],
  ["02", "map→odom", "ground_truth static TF 与 AMCL 二选一；不要同时发布。"],
  ["03", "odom→base_link", "Isaac、真机里程计或 KISS-ICP 只能有一个权威来源。"],
  ["04", "/cmd_vel", "velocity_smoother / behavior_server 是允许的控制来源；teleop 与探索并用时需要 twist_mux。"],
];
let y = 155;
for (const [num, heading, body] of items) {
  addText(summary, `num-${num}`, num, { left: 42, top: y, width: 90, height: 70 }, { fontSize: 44, bold: true, color: "#3D8DFF" });
  addText(summary, `heading-${num}`, heading, { left: 150, top: y, width: 240, height: 42 }, { fontSize: 25, bold: true });
  addText(summary, `body-${num}`, body, { left: 390, top: y, width: 820, height: 70 }, { fontSize: 19, color: "#344054" });
  summary.shapes.add({ name: `line-${num}`, geometry: "rect", position: { left: 42, top: y + 82, width: 1168, height: 1 }, fill: "#D0D5DD", line: { style: "solid", fill: "none", width: 0 } });
  y += 112;
}
addFooter(summary, 5);
summary.speakerNotes.textFrame.setText("[Sources]\n- Project launch files, configs, node sources, and operational documentation listed in slides 2–4\n[/Sources]");

for (const [index, slide] of deck.slides.items.entries()) {
  const png = await deck.export({ slide, format: "png", scale: 1 });
  await fs.writeFile(`${renderDir}/slide-${index + 1}.png`, new Uint8Array(await png.arrayBuffer()));
  const layout = await slide.export({ format: "layout" });
  await fs.writeFile(`${renderDir}/slide-${index + 1}.layout.json`, await layout.text());
}
const montage = await deck.export({ format: "webp", montage: true, scale: 1 });
await fs.writeFile(`${renderDir}/montage.webp`, new Uint8Array(await montage.arrayBuffer()));
const pptx = await PresentationFile.exportPptx(deck);
await pptx.save(finalPptx);
