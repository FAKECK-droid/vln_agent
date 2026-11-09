# vln_agent

面向 ROS 2 Humble 的视觉语言导航 Agent。程序通过 Responses WebSocket 与模型交互，将导航、环境观察、图像获取和深度查询封装成工具，并通过 ROS 话题接收用户输入和机器人事件。

核心能力：

- 模型会话、上下文续接与压缩管理。
- 插件式工具装载和并发执行。
- Nav2 导航、环顾、代价地图观察与像素目标导航。
- RGB-D 图像对保存与像素三维坐标查询。
- 多渠道用户交互和机器事件输入。

组件接口、数据格式与工具说明见 [架构与接口文档](docs/AGENT_PLAN.md)。

## 环境与依赖

- Ubuntu 22.04、ROS 2 Humble。
- ROS 2 Nav2、pluginlib、TF2、OpenCV、Boost、OpenSSL、yaml-cpp。
- 同工作空间中的 `rgbd_image_queue`。
- 环顾功能使用 `vln_behavior_tree` 提供的行为树。

将功能包放入 ROS 工作空间的 `src/scout_mini_vln` 下，通过工作空间的依赖管理安装所需 ROS 和系统库。

## 编译

在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to vln_agent --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
```

资源较紧张的设备可以在构建命令前设置 `CMAKE_BUILD_PARALLEL_LEVEL=1`。Base64 依赖源码随包提供。

## 测试

```bash
colcon test --packages-select vln_agent --event-handlers console_direct+
colcon test-result --test-result-base build/vln_agent --verbose
```

测试覆盖会话、工具、用户交互、事件和图像坐标换算，使用本地假服务与合成数据。真实响应回放用例通过 `VLN_LIVE_FIXTURE` 指定录制文件，缺省时跳过该用例。

## 启动

准备组件配置和系统提示词文件。可在包的 `config` 目录中创建 `agent.local.yaml`，填写模型地址、API 密钥和所需工具配置。

```bash
ros2 run vln_agent vln_agent_node --ros-args \
  -p global_yaml_path:=/path/to/vln_agent/config/agent.local.yaml \
  -p system_prompt_path:=/path/to/system_prompt.md
```

`initial_instruction` 可指定首条用户指令，`loop_hz` 控制主循环频率。模型服务需提供 Responses WebSocket 接口；导航和图像工具按配置连接对应 ROS 服务。

## 项目文档

- [架构、组件接口与数据格式](docs/AGENT_PLAN.md)
- [全局配置](config/agent.yaml)
- [工具定义](plugins/tools)
- [测试源码](test)
