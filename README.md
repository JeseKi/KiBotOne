# KiBotOne

KiBotOne 是一个基于 ROS 2 和 Gazebo 的移动机器人仿真项目。项目包含机器人仿真资源、自定义接口以及基础控制节点，支持手动遥控、巡航、停止和目标跟随等运行模式。

## 项目结构

```text
.
├── src
│   ├── kibot_one_control      # 机器人控制节点和 launch 文件
│   ├── kibot_one_interface    # 自定义 msg/srv 接口
│   └── kibot_one_sim          # Gazebo 模型、世界和 ROS-Gazebo bridge 配置
├── Makefile                   # 常用构建、检查和测试命令
├── pyproject.toml             # Python 开发依赖配置
└── README.md
```

## 功能概览

- Gazebo 仿真：提供 KiBotOne 机器人模型、跟随旗标模型以及普通/障碍物世界。
- ROS-Gazebo 桥接：桥接 `/cmd_vel`、`/odom`、`/tf`、`/scan`、`/flag_pose` 和 `/robot_pose`。
- 模式控制：通过 `/mode_control` 服务切换 STOP、CRUISE、MANUAL、FOLLOW 模式。
- 控制台 TUI：通过终端菜单切换模式，巡航时输入速度，手操时直接控制机器人。
- 键盘遥控：在 MANUAL 模式下使用 WASD 控制机器人移动。
- 速度看门狗：将 `/cmd_vel_raw` 转发到 `/cmd_vel`，并在控制命令超时后自动停车。
- 跟随控制：在 FOLLOW 模式下根据机器人位姿、目标旗标位姿和激光雷达数据进行局部避障跟随。

## 环境准备

项目需要 ROS 2、Gazebo Sim 和 `ros_gz_bridge`。进入项目根目录后，可以使用仓库里的终端初始化脚本激活 ROS 2 环境：

```bash
source .vscode/project-terminal-init.sh
```

如果你使用 `uv` 管理 Python 开发依赖，可以安装本项目的 Python 工具依赖：

```bash
uv sync
```

## 构建

```bash
source .vscode/project-terminal-init.sh
colcon build
source install/setup.bash
```

也可以直接使用 Makefile：

```bash
make build
source install/setup.bash
```

## 运行

### 启动基础仿真与控制

```bash
ros2 launch kibot_one_sim kibot_one.launch.py
```

该 launch 是项目的完整 bringup 入口，会启动 Gazebo、ROS-Gazebo bridge、`mode_control`、`cmd_vel_watchdog` 和 `follow_controller`，默认运行模式为 MANUAL。`follow_controller` 默认启动但只会在 FOLLOW 模式下输出控制命令。

常用参数：

```bash
ros2 launch kibot_one_sim kibot_one.launch.py \
  world:=/absolute/path/to/world.sdf \
  mode:=2 \
  stop_time_period:=0.5 \
  watch_time_period:=0.1 \
  start_follow_controller:=true \
  start_keyboard_teleop:=false
```

### 控制台 TUI

推荐使用控制台 TUI 进行日常控制。在另一个已加载环境的终端中运行：

```bash
ros2 launch kibot_one_control control_console.launch.py
```

控制台按键：

| 场景 | 按键 | 功能 |
| --- | --- | --- |
| 菜单 | `w` / `↑` | 上移选择 |
| 菜单 | `s` / `↓` | 下移选择 |
| 菜单 | `Enter` | 确认当前模式 |
| 菜单 | `q` | 退出控制台 |
| 手操 | `w` / `↑` | 前进 |
| 手操 | `s` / `↓` | 后退 |
| 手操 | `a` / `←` | 原地左转 |
| 手操 | `d` / `→` | 原地右转 |
| 手操 | `Enter` / `Space` | 停止 |
| 手操 | `q` | 停止并返回菜单 |

选择 CRUISE 模式时，控制台会提示输入线速度 `linear.x` 和角速度 `angular.z`。

### 键盘遥控

也可以单独运行旧的键盘遥控节点。在另一个已加载环境的终端中运行：

```bash
ros2 run kibot_one_control keyboard_teleop
```

也可以通过主 launch 可选启动：

```bash
ros2 launch kibot_one_sim kibot_one.launch.py start_keyboard_teleop:=true
```

按键说明：

| 按键 | 功能 |
| --- | --- |
| `w` | 前进 |
| `s` | 后退 |
| `a` | 原地左转 |
| `d` | 原地右转 |
| `space` / `x` | 停止 |
| `q` | 退出 |

键盘遥控只会在 MANUAL 模式下发布有效运动命令。

### 目标跟随

无障碍世界：

```bash
ros2 launch kibot_one_control follow_phase1.launch.py
```

障碍物世界：

```bash
ros2 launch kibot_one_control follow_phase2.launch.py
```

这两个 launch 是主 bringup 的薄封装，会选择对应世界文件，并将初始模式设为 FOLLOW。也可以直接使用主入口：

```bash
ros2 launch kibot_one_sim kibot_one.launch.py mode:=3
```

## 运行模式

模式枚举定义在 `kibot_one_interface/msg/Mode.msg`：

| 值 | 模式 | 说明 |
| --- | --- | --- |
| `0` | STOP | 停止，发布零速度 |
| `1` | CRUISE | 巡航，持续发布服务请求中给定的速度 |
| `2` | MANUAL | 手动控制，默认模式 |
| `3` | FOLLOW | 目标跟随 |

切换模式示例：

```bash
ros2 service call /mode_control kibot_one_interface/srv/Mode "{
  target_mode: 3,
  linear_velocity: {
    linear: {x: 0.0, y: 0.0, z: 0.0},
    angular: {x: 0.0, y: 0.0, z: 0.0}
  }
}"
```

切换到 CRUISE 并设置巡航速度：

```bash
ros2 service call /mode_control kibot_one_interface/srv/Mode "{
  target_mode: 1,
  linear_velocity: {
    linear: {x: 0.3, y: 0.0, z: 0.0},
    angular: {x: 0.0, y: 0.0, z: 0.0}
  }
}"
```

## 主要话题与服务

| 名称 | 类型 | 方向/用途 |
| --- | --- | --- |
| `/cmd_vel_raw` | `geometry_msgs/msg/Twist` | 控制节点输出的原始速度命令 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 发送给 Gazebo 机器人的速度命令 |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo 输出的机器人里程计 |
| `/scan` | `sensor_msgs/msg/LaserScan` | Gazebo 输出的激光雷达数据 |
| `/tf` | `tf2_msgs/msg/TFMessage` | Gazebo 输出的 TF |
| `/robot_pose` | `geometry_msgs/msg/PoseStamped` | 机器人在 Gazebo 中的位姿 |
| `/flag_pose` | `geometry_msgs/msg/PoseStamped` | 跟随目标旗标位姿 |
| `/mode` | `kibot_one_interface/msg/ModeState` | 当前运行模式 |
| `/mode_control` | `kibot_one_interface/srv/Mode` | 模式切换服务 |

## 节点说明

### `mode_control`

负责维护当前运行模式，并通过 `/mode` 发布模式状态。STOP 和 CRUISE 模式下会直接向 `/cmd_vel_raw` 发布速度命令。

主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `mode` | `2` | 初始运行模式 |
| `mode_pub_timer_period` | `0.01667` | STOP/CRUISE 速度发布周期 |
| `mode_topic` | `mode` | 模式状态话题 |

### `cmd_vel_watchdog`

订阅 `/cmd_vel_raw` 并转发到 `/cmd_vel`。如果一段时间内没有新的非零速度命令，则自动发布零速度停车。

主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `stop_time_period` | `0.5` | 命令超时时间，单位秒 |
| `watch_time_period` | `0.1` | 超时检查周期，单位秒 |

### `keyboard_teleop`

提供终端键盘遥控。默认线速度为 `0.8`，角速度为 `1.5`。

### `follow_controller`

在 FOLLOW 模式下运行。控制器订阅机器人位姿、旗标位姿和激光雷达数据，选择局部安全航向并输出 `/cmd_vel_raw`。

常用参数包括：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `stop_distance` | `0.60` | 与目标的停止距离 |
| `max_linear_speed` | `0.50` | 最大线速度 |
| `max_angular_speed` | `1.20` | 最大角速度 |
| `avoidance_range` | `2.50` | 局部避障规划距离 |
| `robot_body_radius` | `0.371` | 机器人避障半径 |
| `sector_count` | `31` | 候选航向扇区数量 |

## 开发命令

```bash
make build   # colcon build
make check   # mypy . && ruff check --fix
make test    # python -m pytest . -q
```

## GitHub Pages

项目展示页位于 `docs/index.html`。在 GitHub 仓库设置中启用 Pages，并选择 `main` 分支的 `/docs` 目录即可发布。

页面包含手操 HTML 演示、自主避障导航 HTML 演示，以及待替换的视频链接占位。

## 排查建议

- 如果 `ros2 launch` 找不到包，确认已经执行 `colcon build` 并 `source install/setup.bash`。
- 如果 Gazebo 找不到模型，优先使用项目提供的 launch 文件，它会自动配置 `GZ_SIM_RESOURCE_PATH`。
- 如果机器人没有响应速度命令，检查当前模式是否为 MANUAL/FOLLOW，且 `/cmd_vel_raw` 是否有消息。
- 如果 FOLLOW 模式不动，检查 `/robot_pose`、`/flag_pose` 和 `/scan` 是否正常发布。

## 许可证

各 ROS 2 包声明为 Apache-2.0。
