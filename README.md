# Panda Pick-and-Place Vision — 视觉引导的 Franka Panda 抓取放置仿真系统

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue?logo=ros) ![Gazebo](https://img.shields.io/badge/Gazebo-Fortress-blue?logo=ros&logoColor=white) ![MoveIt 2](https://img.shields.io/badge/MoveIt_2-Humble-brightgreen?logo=ros) ![C++](https://img.shields.io/badge/C++-17-blue?logo=cplusplus) ![OpenCV](https://img.shields.io/badge/OpenCV-4.x-red?logo=opencv)

Panda Pick-and-Place Vision 是一个基于 **ROS 2 Humble**、**Gazebo Fortress（ros_gz_sim）** 与 **MoveIt 2** 的视觉引导机械臂抓取放置仿真项目：Franka Panda 七自由度机械臂通过机载相机对工作台上的红 / 绿 / 蓝三色盒子做实时识别，OpenCV HSV 阈值检测出目标盒在基座坐标系下的位置，再交由 MoveIt 规划完整「扫描 → 目标锁定 → 抓取 → 抬升 → 放置 → 复位」闭环。视觉、规划、控制三层完全解耦，全部节点为 C++17 实现，一条命令拉起仿真与视觉，抓取节点按需独立运行。

## 项目概述

整个系统按「仿真 → 视觉 → 规划 → 控制」流水线构建：`panda_description` 提供 Panda 机械臂 URDF 与包含三色盒子的 Gazebo 世界，相机传感器（640×320@10Hz）经 `ros_gz_image` 桥接为 `/camera/image_raw`；`panda_vision_cpp` 的 `color_detector` 节点对图像做 HSV 阈值分割、轮廓提取与包围盒计算，把像素坐标经 TF（`panda_link0 ← camera_link`）与固定深度假设反投影到机械臂基座系，发布自定义消息 `/detected_boxes`；`panda_pick_place_cpp` 的 `pick_and_place` 节点订阅该话题，用 MoveIt 规划组 `arm` / `gripper` 依次完成预抓取、下降合爪、抬升、放置与复位，控制器由 `controller_manager`（arm / gripper / joint_state_broadcaster）经 `gz_ros2_control` 插件驱动仿真关节。

**核心特性：**

- 全链路 C++17：视觉检测（OpenCV）、抓取状态机（MoveIt `move_group_interface`）均为原生 C++ 节点，无 Python 业务逻辑。
- 自定义消息解耦：`panda_vision_msgs/DetectedBox`（`color_id` + 基座系 `position`）让视觉与抓取互不感知实现细节，可独立替换任一侧。
- 三色 HSV 检测：R / G / B 独立窄带色域，`min_area` 过滤噪点，`show_image` 实时可视化检测窗口。
- 像素 → 基座系反投影：TF 查询 + 固定深度假设 + 经验标定系数（`y_scale` / 位置补偿）把 2D 检测换算为 3D 抓取点。
- 扫描式目标锁定：抓取前在扫描位姿停留 `scan_duration` 秒收集检测结果，按 `target_color` 锁定目标，窗口内未命中自动安全复位退出。
- 完整抓取状态机：预抓取 → 开爪 → 下降 → 合爪 → 抬升回 home → 放置位松爪 → 复位，每一步失败均中止并给出日志。
- 一条命令启动整套仿真（Gazebo + 机器人 + 控制器 + MoveIt + RViz + 视觉）。

**关键词：** 机器人、ROS 2、机械臂、MoveIt 2、视觉抓取、颜色检测、OpenCV、Franka Panda、Gazebo、ros_gz_sim、ros2_control

## 系统架构

```
                  ┌────────────────────────────────────────────────────┐
  三色盒子 ──▶     │  Gazebo Fortress（ros_gz_sim）                     │
                  │  · Panda 七轴 + 二指夹爪（gz_ros2_control 驱动）    │
                  │  · 相机传感器 640×320@10Hz                         │
                  │  · /clock /camera/image_raw /camera/camera_info    │
                  └──────────────────┬─────────────────────────────────┘
                                     │ /camera/image_raw（ros_gz_image 桥接）
                                     ▼
                  ┌────────────────────────────────────────────────────┐
                  │  color_detector（panda_vision_cpp）                │
                  │  · HSV 阈值 → 轮廓 → 包围盒                        │
                  │  · TF 查询 + 深度假设 → 基座系坐标                  │
                  │  · 发布 /detected_boxes（R/G/B）                   │
                  └──────────────────┬─────────────────────────────────┘
                                     │ /detected_boxes
                                     ▼
                  ┌────────────────────────────────────────────────────┐
                  │  pick_and_place（panda_pick_place_cpp，独立运行）   │
                  │  · 扫描 → 锁定目标 → 预抓取 → 合爪 → 抬升 → 放置   │
                  └──────────────────┬─────────────────────────────────┘
                                     │ 关节轨迹
                                     ▼
                  ┌────────────────────────────────────────────────────┐
                  │  controller_manager（panda_controller）            │
                  │  arm_controller / gripper_controller               │
                  │  joint_state_broadcaster                           │
                  └────────────────────────────────────────────────────┘
                                     ▲
              move_group + RViz（panda_moveit，MoveIt 2 规划/可视化）
```

## 目录结构

```
ros2-panda-pick-and-place-vision/
└── src/
    ├── panda_bringup/                    # 总启动入口（ament_cmake）
    │   └── launch/
    │       └── pick_and_place.launch.py  # 一键启动：Gazebo + 控制器 + MoveIt + 视觉
    ├── panda_controller/                 # 控制器启动（ament_cmake）
    │   ├── config/
    │   │   └── panda_controllers.yaml    # arm/gripper/joint_state_broadcaster 配置
    │   └── launch/
    │       └── panda_controller.launch.py # robot_state_publisher + controller_manager + spawner
    ├── panda_description/                # 机器人描述与仿真资源（ament_cmake）
    │   ├── launch/
    │   │   └── gazebo.launch.py          # Gazebo 仿真 + 模型生成 + 话题桥接
    │   ├── meshes/                       # 碰撞 / 可视化网格（collision/ visual/）
    │   ├── models/                       # 仿真家具模型（咖啡桌、垃圾桶，被 world 引用）
    │   ├── urdf/                         # arm / sensors / gazebo / ros2control / materials xacro
    │   └── worlds/
    │       └── empty.world               # 红/绿/蓝盒子 + 家具 + 地面 + 光照
    ├── panda_moveit/                     # MoveIt 2 配置（ament_cmake）
    │   ├── config/                       # panda.srdf + kinematics/joint_limits/controllers 等
    │   ├── launch/
    │   │   └── moveit.launch.py          # move_group + RViz
    │   └── rviz/
    │       └── moveit.rviz
    ├── panda_pick_place_cpp/             # 抓取放置节点（C++17）
    │   └── src/
    │       └── pick_and_place.cpp        # MoveIt 抓取状态机 + 目标扫描锁定
    ├── panda_vision_cpp/                 # 视觉检测节点（C++17 + OpenCV）
    │   └── src/
    │       └── color_detector.cpp        # HSV 阈值 → 轮廓 → 基座系坐标
    └── panda_vision_msgs/                # 自定义消息（rosidl）
        └── msg/
            └── DetectedBox.msg           # Header + color_id + position
```

## 命名体系

| 类别 | 命名规则 | 示例 |
|------|----------|------|
| 功能包 | `panda_*`（ament_cmake，C++ 节点） | `panda_bringup` / `panda_vision_cpp` / `panda_pick_place_cpp` |
| 相机话题 | `/camera/image_raw`、`/camera/camera_info` | Image 640×320@10Hz（ros_gz_image / ros_gz_bridge 桥接） |
| 检测结果话题 | `/detected_boxes` | `panda_vision_msgs/msg/DetectedBox`（color_id + position） |
| TF 帧树 | `panda_link0 → camera_link → camera_link_optical` | 相机固连基座前方 (0.6, 0, 1.0)m，固定关节发布 TF |
| 目标颜色 | `R` / `G` / `B` | 对应 empty.world 红 / 绿 / 蓝盒子 |
| MoveIt 规划组 | `arm` / `gripper` | `move_group_interface` 两套独立接口 |
| 控制器 | `arm_controller` / `gripper_controller` / `joint_state_broadcaster` | 类型 `joint_trajectory_controller` / `joint_state_broadcaster` |
| 夹爪关节 | `panda_finger_joint1` / `panda_finger_joint2` | 开 0.04m / 合 0.0m |

## 环境依赖

- **操作系统**：Ubuntu 22.04（亦验证于 Windows WSL2 Ubuntu 22.04）。
- **ROS 2**：Humble Hawksbill。
- **Gazebo**：Fortress（通过 `ros_gz_sim` / `ros_gz_bridge` / `ros_gz_image` 集成）。
- **主要依赖**：

```bash
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-image \
  ros-humble-gz-ros2-control \
  ros-humble-controller-manager \
  ros-humble-joint-trajectory-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-cv-bridge \
  ros-humble-xacro \
  ros-humble-rviz2 \
  python3-colcon-common-extensions
```

## 安装步骤

1. **创建工作区并放入源码**：

```bash
mkdir -p ~/panda_ws/src
cp -r <此仓库>/src/* ~/panda_ws/src/
```

> **WSL2 注意**：不要在 `/mnt/c`（NTFS 挂载）下编译，CMake 编译器自检会报错；请把工程拷入 WSL 原生文件系统（如 `~/panda_ws`）后再编。

2. **安装依赖**：

```bash
cd ~/panda_ws
rosdep install --from-paths src --ignore-src -r -y   # 或按上面 apt 列表手动装
```

3. **编译**：

```bash
source /opt/ros/humble/setup.bash
cd ~/panda_ws
colcon build --symlink-install
source install/setup.bash
```

4. **WSL2 用户追加：禁用 FastDDS 共享内存与组播**（原生 Ubuntu 可跳过）：

WSL2 内核下 FastDDS 的 SHM 残留锁和组播发现会导致进程几分钟加不进图、CLI 状态乱跳。新建一份仅用单播回环的配置：

```bash
cat > ~/fastdds_udp.xml <<'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_only</transport_id>
      <type>UDPv4</type>
      <interfaceWhiteList>127.0.0.1</interfaceWhiteList>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="udp_only_participant" is_default_profile="true">
    <rtps>
      <userTransports><transport_id>udp_only</transport_id></userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
EOF
```

每次启动 ROS 进程前（建议写进一个 `launch_panda.sh` 包装脚本，注意 `bash -c` 不读 `.bashrc`，变量必须在脚本内显式 export）：

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds_udp.xml
rm -f /dev/shm/fastrtps_*        # 清残留 SHM 锁
```

## 运行

### 方式一：一键启动仿真 + 控制器 + MoveIt + 视觉

```bash
source ~/panda_ws/install/setup.bash
ros2 launch panda_bringup pick_and_place.launch.py
```

启动后应看到：Gazebo 中出现 Panda 机械臂与红 / 绿 / 蓝三个盒子，RViz 显示机械臂模型与 TF，`color_detector` 终端打印检测到的盒子坐标（`color_id,x,y,z`），并弹出「Color Detection」调试窗口实时框出目标。

### 方式二：手动运行抓取放置节点（主 launch 不包含）

按设计，`pick_and_place` 节点独立运行，可在视觉就绪后另开终端执行：

```bash
ros2 run panda_pick_place_cpp pick_and_place --ros-args -p target_color:=B
```

节点流程：移动到扫描位姿 → 停留 `scan_duration` 秒收集 `/detected_boxes` → 锁定目标颜色 → 回 home → 预抓取 → 开爪 → 下降合爪 → 抬升 → 放置位松爪 → 复位退出。扫描窗口内未检测到目标颜色会安全回到初始位并退出（终端打印 `Target color X not found during scan!`）。

### 方式三：仅启动仿真预览（不起控制器 / MoveIt）

```bash
ros2 launch panda_description gazebo.launch.py
```

用于快速验证 world、机械臂模型与相机画面，不涉及控制器与规划。

### 分步调试（推荐第一次运行时）

```bash
# 1) 确认相机图像与桥接
ros2 topic hz /camera/image_raw
ros2 topic echo /camera/camera_info --once

# 2) 确认 TF 链（基座 ← 相机）
ros2 run tf2_ros tf2_echo panda_link0 camera_link

# 3) 确认控制器与规划组已加载
ros2 lifecycle get /controller_manager
ros2 action list

# 4) 观察视觉检测结果
ros2 topic echo /detected_boxes --once
```

## 调参速查

| 参数 | 位置 | 默认 | 说明 |
|------|------|------|------|
| 目标颜色 `target_color` | `pick_and_place.cpp` / 主 launch | R（主 launch 声明为 B） | 要抓取的盒子颜色 |
| 扫描时长 `scan_duration` | `pick_and_place.cpp` | 3.0s | 扫描位姿下收集检测结果的窗口 |
| 相机内参 `fx` / `fy` / `cx` / `cy` | `color_detector.cpp` | 585 / 588 / 320 / 160 | 与 640×320 图像对应 |
| 深度假设 `assumed_depth` | `color_detector.cpp` | 0.1m | 固定高度假设，像素反投影到相机系 |
| 像素→基座系数 `y_scale` | `color_detector.cpp` | -10.0 | 经验标定系数 |
| 最小面积 `min_area` | `color_detector.cpp` | 1.0 | 过滤轮廓噪点 |
| 图像话题 `image_topic` | `color_detector.cpp` | `/camera/image_raw` | 可指向任意相机话题 |
| 调试窗口 `show_image` | `color_detector.cpp` | true | 弹窗实时显示检测框 |
| HSV 色域 | `color_detector.cpp` `color_ranges_` | R 0-10 / G 55-60 / B 90-128 | 饱和度、明度下限 200 |
| 位置经验补偿 | `color_detector.cpp` | B: y-0.0215 / G: y+0.01 | 抵消标定与夹爪偏差 |
| 盒子位置 | `worlds/empty.world` | 红(0.6,0.6) / 绿(0.8,0.6) / 蓝(0.4,0.6)，高 0.70m | 尺寸 0.06×0.06×0.1 |
| 机械臂位姿 | `pick_and_place.cpp` | start / home / drop 三组关节角 | 扫描位 / 搬运位 / 放置位 |

## 常见问题

- **颜色漏检 / 误检？** HSV 色域区间很窄且对光照敏感；调 `color_ranges_` 或 `min_area`，并用 `show_image:=true` 的调试窗口观察掩码效果。
- **日志报 `TF lookup failed`？** `camera_link` 的 TF 由 `robot_state_publisher` 从 URDF 固定关节发布；确认 `panda_controller.launch.py` 已启动，用 `ros2 run tf2_ros tf2_echo panda_link0 camera_link` 验证。
- **`/camera/image_raw` 无数据？** 确认 `ros_gz_image` 的 `image_bridge` 已启动、Gazebo 相机传感器存在（URDF 中 `visualize=true` 时可在 Gazebo 窗口看到画面）。
- **抓取位置有偏差？** 反投影依赖固定深度假设 `assumed_depth` 与经验系数，只对当前相机安装高度有效；调整相机或盒子高度后需重新标定 `y_scale` 与位置补偿。
- **`pick_and_place` 一启动就退出？** 扫描窗口内没有检测到目标颜色，节点按设计安全复位退出；先确认 `ros2 topic echo /detected_boxes` 中有对应颜色发布。
- **`cv::imshow` 弹不出窗口（WSL2）？** 需要 WSLg（Windows 11 默认支持）或 X server；无图形环境时把 `show_image` 置为 false。
- **节点要几分钟才进图 / 服务调用挂死（WSL2）？** FastDDS SHM 残留锁 + 组播发现失效；按「安装步骤 4」禁用 SHM 并清理 `/dev/shm/fastrtps_*`。

## License

本项目代码采用 Apache-2.0 协议开源。
