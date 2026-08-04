# 空地协同：无人机建图与人形机器人自主导航

> ROS 课程作业项目仓库：由一台仿真无人机在陌生室内场景中执行自主飞行与深度观测、
> 建立二维占据栅格图；地面侧以 N2 人形机器人为执行平台，
> 由强化学习步态策略负责关节级控制，
> 在共享地图上通过 A\* 全局规划 + DWA 局部规划完成自主导航。


## 1. 软件包功能说明

本仓库在同一个 catkin 工作空间中集成三条相互协作的技术链路：

- **无人机建图**（`src/CERLAB-UAV-Autonomy`）：
  在 Gazebo 中启动带 RGB-D 相机的四旋翼无人机，
  由 `map_manager/occupancy_map` 节点将深度帧融合为二维占据栅格地图，
  发布到 `/occupancy_map/2D_occupancy_map` 话题，并可用 `map_saver` 落盘。
- **强化学习运动控制**（`src/legged_rl`）：
  在 Isaac Gym 中训练好的 TorchScript 步态策略经 `rl_controllers` 部署包在 Gazebo 或真机上加载，
  订阅上层 `/cmd_vel`，输出 N2 各关节目标力矩。
- **人形机器人导航**（`src/legged_rl/legged_robot/ning/legged_n2_description` + `src/mbot_navigation`）：
  基于 `move_base`（A\* 全局规划 + DWA 局部规划）在建好的二维栅格图上执行路径规划，
  通过自研的 `cmd_vel_amplifier.py` 中间件解决强化学习步态在低速下的死区问题。

**程序输入**：
- Gazebo 中的仿真传感器数据（RGB-D 深度图、Odom、Scan 等），由启动脚本自动产生；
- 用户在 RViz 中通过 `2D Nav Goal` 下发的导航目标点。

**程序输出**：
- 三个 RViz 窗口中的实时可视化（无人机深度点云与占据体素、地面二维地图、导航全局/局部路径）；
- 可选：使用 `map_saver` 保存的 `pgm + yaml` 地图对。

**适用场景**：室内静态或半静态平面场景下的空地协同自主导航实验，
仿真链路完整，其中强化学习运动控制模块已在真机（N2 人形机器人）上完成部署验证。

## 2. 文件目录结构

```
legged_n2_jump-master1/
├── README.md                           本文件
├── build.sh                            Release 模式一键编译
├── simulation.sh                       启动 Gazebo + RL 控制器
├── readme.txt                          简版启动命令备忘
└── src/
    ├── CERLAB-UAV-Autonomy/            无人机自主飞行 + 建图（CERLAB 开源框架）
    │   ├── uav_simulator/              无人机 Gazebo 仿真与 PX4 桥接
    │   ├── map_manager/                占据栅格 / ESDF / 动态地图管理
    │   ├── global_planner/             无人机全局路径规划
    │   ├── trajectory_planner/         轨迹优化
    │   ├── time_optimizer/             时间最优参数化
    │   ├── autonomous_flight/          高层自主飞行状态机
    │   ├── tracking_controller/        轨迹跟踪控制器
    │   ├── onboard_detector/           机载视觉检测（YOLO 等）
    │   └── remote_control/             地面站遥控与调试脚本
    ├── legged_rl/                      人形机器人 + 强化学习控制
    │   ├── legged_base/                通用基础包
    │   │   ├── legged_common/          URDF/参数生成脚本
    │   │   └── legged_gazebo/          Gazebo 场景与人形仿真启动
    │   ├── legged_robot/
    │   │   ├── ning/legged_n2_description/   N2 描述包（本组主要开发）
    │   │   │   ├── config/nav/               导航栈参数（costmap、DWA 等）
    │   │   │   ├── launch/                   见第 7 节
    │   │   │   ├── maps/                     预建静态地图
    │   │   │   ├── meshes/, urdf/            机器人视觉与运动学描述
    │   │   │   ├── rviz/                     RViz 显示配置
    │   │   │   └── scripts/                  cmd_vel_amplifier.py, gazebo_odom_broadcaster.py
    │   │   └── ning_hw/legged_ning_hw/       真机驱动（真机部署时使用）
    │   └── rl_controller/rl_controllers/     强化学习步态控制器
    │       └── launch/                       ac_start.launch, ac_start_real.launch, ...
    └── mbot_navigation/                备用导航栈配置（amcl/gmapping/move_base）
```

**主程序入口**：无单一 Python 入口。通过 `roslaunch` 分别拉起三条链路，见第 7 节。

**参数文件**：
- 导航栈：`src/legged_rl/legged_robot/ning/legged_n2_description/config/nav/*.yaml`
- 无人机建图：`src/CERLAB-UAV-Autonomy/map_manager/cfg/occupancy_map_param.yaml`
- 无人机仿真：`src/CERLAB-UAV-Autonomy/uav_simulator/launch/*.launch`
- RL 控制器：`src/legged_rl/rl_controller/rl_controllers/config/`

**输入数据**：全部由 Gazebo 仿真在线产生，无需用户提前准备离线数据。

**运行结果**：仿真过程实时在 RViz/Gazebo 中可视化，不主动生成文件；
如需固化地图，使用 `map_server/map_saver` 保存到 `legged_n2_description/maps/`。

**用户可修改**：`config/nav/*.yaml`、`launch/*.launch`、`scripts/cmd_vel_amplifier.py` 中的放大倍率、
`map_manager/cfg/occupancy_map_param.yaml` 中的分辨率与建图范围。

**程序自动生成**：catkin 的 `build/`、`devel/`、`logs/` 三个目录，以及用户主动保存的地图 `pgm + yaml` 文件对。

## 3. 运行环境

经过测试的软硬件环境：

- **操作系统**：Ubuntu 20.04 LTS
- **ROS 版本**：ROS 1 Noetic Ninjemys
- **编译工具**：catkin-tools（`catkin build`）+ CMake ≥ 3.16
- **C++ 标准**：C++17
- **Python 版本**：Python 3.8
- **仿真平台**：Gazebo 11（随 ROS Noetic 安装）
- **可视化**：RViz 1.14
- **必需第三方 ROS 功能包**：`map_server`、`move_base`、`dwa_local_planner`、`amcl`、
  `gmapping`、`gazebo_ros`、`gazebo_ros_control`、`controller_manager`、`tf2_ros`、
  `robot_state_publisher`、`joint_state_publisher_gui`、`rviz`
- **CERLAB 建图栈附加依赖**：`libeigen3-dev`、`libpcl-dev`、`libopencv-dev`、`libompl-dev`
- **硬件要求**：
  - 仿真：普通 x86_64 PC，建议 8 GB 及以上内存与支持 OpenGL 的显卡
  - 真机（仅 RL 控制模块）：与仿真同款 N2 人形机器人 + 遥控手柄
- **说明**：本仓库运行阶段**不需要 GPU**；Isaac Gym 端的策略训练已提前完成，
  仓库中直接加载导出的 TorchScript 模型进行前向推理。

## 4. 依赖安装

以下命令均在 Ubuntu 20.04 + ROS Noetic 已完成安装的前提下执行。

### 4.1 系统级 ROS 依赖

```bash
sudo apt update
sudo apt install -y \
    ros-noetic-map-server ros-noetic-move-base ros-noetic-dwa-local-planner \
    ros-noetic-amcl ros-noetic-gmapping \
    ros-noetic-gazebo-ros ros-noetic-gazebo-ros-control ros-noetic-controller-manager \
    ros-noetic-tf2-ros ros-noetic-robot-state-publisher ros-noetic-joint-state-publisher-gui \
    ros-noetic-rviz \
    libeigen3-dev libpcl-dev libopencv-dev libompl-dev
```

一键补齐 catkin 内部依赖：

```bash
cd legged_n2_jump-master1
rosdep install --from-paths src --ignore-src -r -y
```

### 4.2 编译

```bash
./build.sh
```

`build.sh` 等价于：

```bash
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```

**注意**：切勿使用 Debug 模式编译，否则 Gazebo 仿真与 RL 控制器推理会明显卡顿。
首次编译在 8 核 CPU 上大约需要 15 到 25 分钟。

## 5. 运行前配置

首次运行前请检查以下配置项：

1. **机器人类型环境变量**（`simulation.sh` 已包含，其他 launch 单独启动时需自行导出）：

    ```bash
    export ROBOT_TYPE=ning
    ```

2. **动态库路径**：

    ```bash
    export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
    ```

3. **ROS 工作空间环境**：

    ```bash
    source ./devel/setup.bash
    ```

4. **地图文件**：`n2_navigation.launch` 默认加载
   `legged_n2_description/maps/my_world.yaml`。
   如需换成建好的新地图，请修改该 launch 中 `map_file` 参数或将新地图落到同一目录并同步文件名。

5. **禁用绝对路径**：所有路径均以 `$(find <pkg_name>)` 或工作空间相对形式给出，
   自建 launch / 修改 yaml 时也应遵循这一约定，方便他人复现。

## 6. 完整运行流程

按照 `readme.txt` 中给出的两条主线执行，每一步都请新开一个终端。
执行前请确保当前目录为工作空间根 `legged_n2_jump-master1/`。

### 主线 A：无人机建图（Module 1）

**终端 A-1** 启动无人机 Gazebo 仿真：

```bash
source ./devel/setup.bash
roslaunch uav_simulator start.launch
```

启动完成后 Gazebo 中出现无人机与预设室内场景。

**终端 A-2** 启动占据栅格建图节点：

```bash
source ./devel/setup.bash
roslaunch map_manager occupancy_map.launch
```

该节点订阅无人机深度图与位姿，
持续发布二维占据栅格图到 `/occupancy_map/2D_occupancy_map`。

**终端 A-3** 打开建图专用 RViz：

```bash
source ./devel/setup.bash
roslaunch map_manager rviz.launch
```

在 RViz 中可实时观察占据栅格图、体素地图与无人机轨迹。
建图满意后使用 `rosrun map_server map_saver -f <保存路径>/my_map` 落盘。

### 主线 B：人形机器人导航（Module 2 + Module 3）

**终端 B-1** 启动人形机器人 Gazebo 仿真与强化学习控制器：

```bash
./simulation.sh
```

脚本内部执行：

```bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export ROBOT_TYPE=ning
source ./devel/setup.bash
roslaunch rl_controllers ac_start.launch
```

启动完成后 Gazebo 中出现 N2 人形机器人。
等待约 5 到 10 秒直至终端输出 `Controller Manager: started` 或类似提示，
说明底层 RL 控制器加载完毕。

**终端 B-2** 启动导航节点：

```bash
source ./devel/setup.bash
roslaunch legged_n2_description n2_navigation.launch
```

该 launch 会拉起：`map_server`（加载 `my_world.yaml`）、`amcl` 定位、
`gazebo_odom_broadcaster.py`（Gazebo 真值里程计转 tf）、
`move_base`（A\* + DWA，`/cmd_vel` 重映射到 `/nav_cmd_vel`）、
`cmd_vel_amplifier.py`（`/nav_cmd_vel` → 放大与死区突破 → `/cmd_vel`）、
以及配套 RViz（`full_nav.rviz`）。

**终端 B-3（可选）** 如希望在导航同时进行实时建图（边建边导），可用：

```bash
source ./devel/setup.bash
roslaunch legged_n2_description n2_mapping_and_navigation.launch
```

此模式下不再加载静态 `map_server`，改由 `gmapping` 在线建图，AMCL 与 move_base 均使用其实时地图。

**步骤 C：在 RViz 中下发目标点**

打开步骤 B-2 或 B-3 弹出的 RViz 窗口，
点击工具栏 **2D Nav Goal**，在地图上单击并拖动一个箭头以指定目标位姿。
机器人将沿绿色全局路径与蓝色局部规划开始行走，直到到达目标附近。

> **等待条件**：主线 B 中的终端 B-2/B-3 必须等到终端 B-1 的 RL 控制器完全加载后再启动，
> 否则 `/cmd_vel` 无消费者，机器人会保持原地不动。

## 7. 输入说明

| 输入项 | 类型 / 话题 | 位置 | 说明 |
| --- | --- | --- | --- |
| 无人机深度图 | `sensor_msgs/Image` | Gazebo 插件自动发布 | 输入到 `map_manager` |
| 无人机位姿 | `nav_msgs/Odometry` + tf | Gazebo P3D 插件 | 用于建图射线投射 |
| 二维激光 `scan` | `sensor_msgs/LaserScan` | Gazebo 插件 | 用于 AMCL 或 gmapping |
| 真值里程计 | `nav_msgs/Odometry` | `/ground_truth/state` | 转 tf 供导航使用 |
| 静态地图 | `pgm + yaml` | `legged_n2_description/maps/` | `n2_navigation.launch` 默认加载 `my_world.yaml` |
| 导航目标 | `geometry_msgs/PoseStamped` | RViz `2D Nav Goal` | 用户交互输入 |
| 导航参数 | `yaml` | `legged_n2_description/config/nav/` | 见第 3 节 |
| 建图参数 | `yaml` | `map_manager/cfg/occupancy_map_param.yaml` | 分辨率、范围等 |

## 8. 输出说明

| 输出项 | 说明 |
| --- | --- |
| `/occupancy_map/2D_occupancy_map` | 无人机侧生成的二维占据栅格图 |
| `/occupancy_map/occupancy_map_inflated` | 膨胀后的占据地图 |
| `/move_base/NavfnROS/plan` | A\* 全局路径，RViz 绿色 |
| `/move_base/DWAPlannerROS/local_plan` | DWA 局部路径，RViz 蓝色 |
| `/nav_cmd_vel` → `/cmd_vel` | 由 `cmd_vel_amplifier` 放大后送给 RL 控制器 |
| `/joint_states`, `/tf` | RL 控制器与仿真发布 |
| `maps/<保存名>.pgm + .yaml` | 用户主动调用 `map_saver` 时才生成 |

程序默认不主动写文件，所有结果在 Gazebo/RViz 与终端中实时观察。

## 9. 运行成功的判断标准

1. 主线 A：Gazebo 中无人机稳定悬停或按预设航点飞行，
   RViz 中 `/occupancy_map/2D_occupancy_map` 覆盖范围随无人机移动逐步扩展。
2. 主线 B 终端 B-1：Gazebo 中 N2 机器人稳定站立，不倒地。
3. 主线 B 终端 B-2/B-3：RViz 中显示静态地图或 gmapping 实时地图，
   机器人本体位姿正确显示。
4. 下发 `2D Nav Goal` 后终端输出 `Got new plan`，
   RViz 中出现绿色全局路径与蓝色局部规划路径。
5. 机器人开始双足行走并最终停在目标附近。
6. 各终端均无持续报错（偶发 `TF_OLD_DATA` 警告可忽略）。

## 运行效果

以下是运行效果的视频演示：

- [导航视频](./media/vedio1.webm)
- [建图视频](./media/vedio2.mp4)

## 10. 常见问题

**Q1**：`./simulation.sh` 报错找不到 `rl_controllers`。

原因：编译未完成，或 `devel/setup.bash` 未 source。

解决：

```bash
./build.sh
source ./devel/setup.bash
./simulation.sh
```

**Q2**：机器人在 Gazebo 中一加载就摔倒。

原因：Debug 模式编译导致控制器计算滞后；或 `ROBOT_TYPE` 未导出。

解决：

```bash
export ROBOT_TYPE=ning
./build.sh    # 默认 Release
```

**Q3**：下发 `2D Nav Goal` 后机器人原地抖动、不前进。

原因：DWA 输出速度低于 RL 步态的可跟随下限。

解决：调大 `n2_navigation.launch` 中 `cmd_vel_amplifier` 参数：

```xml
<param name="min_linear_x"   value="0.41" />
<param name="min_angular_z"  value="0.55" />
<param name="scale_linear_x" value="2.0" />
```

**Q4**：主线 A 中 `/occupancy_map/2D_occupancy_map` 长时间没有数据。

原因：`uav_simulator/start.launch` 尚未成功发布深度图与位姿。

解决：先 `rostopic hz /camera/depth/points` 与 `rostopic hz /mavros/local_position/pose`
确认上游话题正常，再启动 `map_manager occupancy_map.launch`。

**Q5**：编译时提示 `ompl` 或 `PCL` 找不到。

原因：CERLAB 建图与轨迹优化依赖未安装。

解决：

```bash
sudo apt install -y libompl-dev libpcl-dev libeigen3-dev libopencv-dev
rosdep install --from-paths src --ignore-src -r -y
```

**Q6**：Windows 用户 Expand-Archive 时报路径过长。

原因：CERLAB 内含较深的 Python 子模块路径。

解决：请在 Linux 环境或启用 Windows 长路径支持后再解压。
本仓库仅面向 Ubuntu 20.04 + ROS Noetic，Windows 不作为支持平台。

## 11. 程序停止方式

- 依次在各终端按 `Ctrl + C` 停止 `roslaunch`；
- 若 Gazebo 无法退出，执行 `killall -9 gzserver gzclient` 强制清理；
- 关闭仿真后建议 `rosnode kill -a` 清理残留节点。

## 12. 补充说明

- `src/CERLAB-UAV-Autonomy/` 来自 CERLAB 开源无人机自主飞行框架，遵循其原始许可，
  本组仅在其基础上进行了少量适配（详见其子目录内的 `README.md` 与 `LICENSE`）。
- 强化学习步态策略的训练管线（Isaac Gym）不在本仓库中，
  仓库直接加载已训练好的 TorchScript 权重进行推理。
- 目前**只有强化学习运动控制模块**完成了 N2 真机部署验证，
  无人机建图与人形机器人导航两模块均在仿真中完成闭环，尚未迁移到真实硬件。
- 真机部署请使用 `rl_controllers/launch/ac_start_real.launch` 与
  `legged_robot/ning_hw/legged_ning_hw/launch/legged_ning_hw.launch`，
  并显著降低 `cmd_vel_amplifier` 的放大倍率以确保安全。
