# VLN Agent 架构与接口

## 1. 程序组成

`vln_agent_node` 通过 Responses WebSocket 接收模型消息和函数调用，将小车动作交给工具插件执行。用户交互渠道和机器人事件分别接入独立组件，由 `VLNAgent` 主循环串接。

| 组件 | 工作内容 |
| --- | --- |
| `VLNAgent` | 初始化组件，轮询事件、完整响应、工具结果和用户输入 |
| `SessionManager` | 组装请求、限频、保存当前会话上下文和完整响应 |
| `ResponsesWsTransport` | WS/WSS 连接、认证、收发、超时探活和重连 |
| `ToolManager` | 装载工具插件、派发调用、交付逐调用结果，并提供 ToolCall 服务 |
| `SharedResources` | 按名称和类型保存工具组共享对象 |
| `EventManager` | 接收机器事件，批量交付原生输入 |
| `UserInteractionManager` | 查询当前组件信息，通过交互插件收发用户数据 |

主要数据流：

```text
用户渠道 ── UserInteractionManager ─┐
机器人事件 ── EventManager ────────┼─ VLNAgent ─ SessionManager ─ ResponsesWsTransport
工具结果 ── ToolManager ──────────┘      │
                                        └─ 完整 response ─ ToolManager ─ 工具插件
```

### 1.1 目录

```text
include/vln_agent/
  vln_agent.hpp
  session/                     # 会话与网络公共接口
  tools/                       # 工具管理、插件基类和共享资源
  events/                      # 机器事件接口
  interaction/                 # 用户交互接口
  utils/                       # JSON、时间、Base64、URL
  plugins/
    tools/{navigation,rgbd}/
    interfaces/ros_topic_interface/
src/                           # 组件实现
plugins/
  tools/{navigation,rgbd}/
  interfaces/ros_topic_interface/
srv/ToolCall.srv
config/agent.yaml
third_party/libbase64/
test/
```

插件头文件与源文件采用对应的子目录；同组工具共用 `tool.json` 和共享资源类。

## 2. 业务数据与 Responses 输入输出

`Json` 是 `nlohmann::json` 的别名，可以保存对象、数组、字符串、数值、布尔和 `null`。`Json::object()` 创建 `{}`，`Json::array()` 创建 `[]`。输入项是对象，多条输入组成数组。

### 2.1 业务 JSON

用户输入、机器事件和工具结果使用以下字段。具体业务载荷位于 `data`。

| 字段 | 含义 |
| --- | --- |
| `source` | 用户为 `user`，机器事件和工具结果为 `robot` |
| `time_stamp` | 本条信息产生时刻，UTC 毫秒字符串 |
| `data` | 业务对象，默认 `{}` |
| `instruction` | 用户指令或机器事件中的工具调用建议 |
| `origin` | 机器事件的发布者或节点实例名 |
| `event` | 机器事件类型 |
| `pose` | 可选的小车观测位姿 |
| `images` | 与原生图片逐张对应的描述对象数组 |
| `ok` | 工具函数执行是否成功 |
| `code`、`message` | 工具失败结果的错误码和说明 |

用户指令：

```json
{
  "source": "user",
  "time_stamp": "2026-11-09T06:00:00.000Z",
  "instruction": "查看门口的情况。",
  "data": {}
}
```

机器事件：

```json
{
  "source": "robot",
  "time_stamp": "2026-11-09T06:00:00.000Z",
  "origin": "ReportNavigationCompleted",
  "event": "navigation_completed",
  "data": {"message": "已到达导航目标。"}
}
```

事件说明放在 `data.message`。工具失败说明使用顶层 `code/message`，与 `ok` 同级。

二维位姿格式：

```json
{
  "frame_id": "map",
  "x": 1.2,
  "y": 2.3,
  "yaw": 0.5,
  "time_stamp": "2026-11-09T05:59:59.900Z"
}
```

距离单位为米，yaw 为相对坐标系正 x 轴的逆时针弧度角。位姿采样时间可省略；导航事件中的目标位姿还包含 `z`。

项目时间字符串使用 `YYYY-MM-DDTHH:mm:ss.SSSZ`，由 `utils::utcNow()`、`formatUtcTime()` 等函数生成。`wait_ms`、`run_ms`、`navigation_time_sec` 等时长使用数值。图片和位姿的采样时间在转发中保留；仿真采样时间通过 ROS 时钟参与计算，可选 UTC 采样字段省略。

### 2.2 原生输入项

用户输入和机器事件包装为 `role: "user"` 的消息，首个 `input_text.text` 保存业务 JSON 字符串：

```json
{
  "role": "user",
  "content": [
    {
      "type": "input_text",
      "text": "{\"source\":\"user\",\"time_stamp\":\"2026-11-09T06:00:00.000Z\",\"instruction\":\"查看门口的情况。\",\"data\":{}}"
    }
  ]
}
```

`SessionManager::addInputs()`、`EventManager::addEvents()` 和交互插件交换的都是这种原生项或项数组。图片依次放在后续 `input_image` 中，业务 JSON 的 `images[i]` 对应第 i 张图片。

图片描述由提供方组织。相机和环境工具将自定义信息放在 `images[i].metadata`；行为树事件将 `ImageContent::Image::metadata` 的键值平铺至 `images[i]`。公共检查读取图片描述数组的数量与对象类型。

工具返回值采用独立的 `function_call_output`：

```json
{
  "type": "function_call_output",
  "call_id": "call_start_1",
  "output": [
    {
      "type": "input_text",
      "text": "{\"source\":\"robot\",\"time_stamp\":\"2026-11-09T06:00:00.000Z\",\"ok\":true,\"data\":{\"task_id\":\"nav_1\",\"status\":\"running\"}}"
    }
  ]
}
```

工具图片同样追加在首个 `input_text` 之后。`ToolResult::success(data)` 和 `failure(code, message)` 构造业务结果，ToolManager 补齐公共字段并封装关联 `call_id`。

### 2.3 模型输出

Agent 从完整 `response.output[]` 中提交 `function_call`；交互层提取 `message` 中的 `output_text` 和 `refusal` 供用户观看。

```json
{
  "id": "resp_1",
  "status": "completed",
  "output": [
    {
      "type": "function_call",
      "call_id": "call_start_1",
      "name": "navigate_to_pose_start_navigation",
      "arguments": "{\"frame_id\":\"map\",\"x\":3.0,\"y\":1.0,\"yaw\":0.0}"
    }
  ]
}
```

`arguments` 是 JSON 字符串，ToolManager 将其解析成参数对象。`response.status` 表示模型本轮生成状态，工具结果中的 `data.status` 表示导航物理任务状态。上下文中的原生输出项由 SessionManager 按会话模式续接或重放。

## 3. 工具定义与执行

公共接口见 [tool_manager.hpp](../include/vln_agent/tools/tool_manager.hpp)。

### 3.1 插件与配置

每个函数对应一个 `ToolPluginBase` 派生类，工具名采用下划线，类名采用 PascalCase，例如 `depth_project` 对应 `DepthProject`。

```cpp
void initialize(const ToolContext & context, const Json & settings) override;
ToolResult execute(const Json & arguments) override;
void shutdown() noexcept override;
```

`initialize()` 接收宿主 ROS 节点、共享资源注册表、组 ID、组配置和关闭标志；第二个参数为当前函数的配置。`execute()` 执行一次调用，返回 `ToolResult`。同一插件实例可以被多个工作线程调用，函数实现按共享状态的需要加锁。

工具组的 `definition` 指向原生函数定义数组，路径相对于全局 YAML 所在目录解析。`functions` 选择启用项，并指定插件与本地 `blocking` 策略：

```yaml
tool_manager:
  service_name: /vln/tool_call
  max_concurrent_calls: 4
  max_pending_calls: 64
  groups:
    - id: navigation
      definition: ../plugins/tools/navigation/tool.json
      settings:
        action_name: /navigate_to_pose
      functions:
        - name: navigate_to_pose_start_navigation
          plugin: vln_agent/NavigateToPoseStartNavigation
          blocking: true
```

`getTools()` 按组和函数配置顺序返回定义数组。`tool.json` 中的 `async`、`strict` 和参数 Schema 随定义传入模型；YAML 中的 `blocking` 用于本地队列派发。

`SharedResources::getOrCreate<T>()` 在当前 ToolManager 内按名称和类型复用资源。资源作用域通常为 `group_id + "/资源名"`。插件和共享资源先结束等待，再随管理器关闭释放；专用 ROS 回调由 `ToolCallbacks` 的独占回调组和执行线程处理。

### 3.2 队列与结果句柄

`submit(completed_response)` 按 `output` 中函数项的顺序登记调用，立即返回 `std::vector<ToolCallHandle>`。每个句柄包含：

| 成员 | 内容 |
| --- | --- |
| `call_id` | 调用方提供的关联 ID |
| `name` | 工具名 |
| `result` | `shared_future<Json>`，值为一个原生 `function_call_output` 对象 |
| `completion` | `shared_future<Json>`，值为本次调用终态与固定计时 |

工作线程按 FIFO 从队首取调用。非阻塞调用派发后，其他空闲线程继续取后续项；阻塞调用被取出后，后续派发暂停到该函数返回。先前运行中的调用继续执行，因此 `blocking` 表示后续派发门控，整体执行和完成顺序由各函数耗时决定。

调用完成时先交付 `completion`，移出活动调用记录并释放参数，再交付 `result`。终态信息示例：

```json
{
  "call_id": "call_start_1",
  "name": "navigate_to_pose_start_navigation",
  "sequence": 0,
  "blocking": true,
  "state": "finished",
  "wait_ms": 1,
  "run_ms": 12
}
```

失败终态使用 `state: "failed"`，按结果补充 `code/message`。每次 `result` 只对应一次函数调用；导航等持续动作通过各自的 `task_id` 继续关联。

活动 `call_id` 再次提交时复用已有 future。完成后的结果与终态由持有句柄的调用方保存；历史去重也由调用方处理。

### 3.3 查询接口

查询返回 JSON 值副本。

| 方法 | 返回值 |
| --- | --- |
| `getTools()` | 启用工具的原生定义数组 |
| `getStatus()` | 管理器状态、容量、等待/运行/累计完成数量、活动数量和阻塞标志 |
| `getQueueSnapshot()` | `{"waiting":[...],"running":[...]}`，各项为调用元数据 |
| `getCallStatus(call_id)` | 活动调用元数据；完成或未知时为 `null` |

`getStatus()` 的字段为 `state`、`max_concurrent_calls`、`max_pending_calls`、`waiting_count`、`running_count`、`completed_count`、`call_count`、`blocking`。队列项包含 `call_id/name/sequence/blocking/state/wait_ms/run_ms`。

工具错误通过各自的 `result` 交付。管理器产生的错误码包括 `UNKNOWN_TOOL`、`INVALID_CALL`、`INVALID_ARGUMENTS`、`TOOL_QUEUE_FULL`、`TOOL_EXECUTION_FAILED` 和 `TOOL_MANAGER_CLOSED`。

### 3.4 ToolCall 服务

ToolManager 装载后提供 `vln_agent/srv/ToolCall`，默认服务名为 `/vln/tool_call`。机器人调用与模型调用复用同一个插件池、队列和共享资源。

```srv
string request_json
---
string outputs_json
string error
```

`request_json` 是 `status: "completed"`、`output` 为函数调用数组的对象字符串：

```json
{
  "status": "completed",
  "output": [
    {
      "type": "function_call",
      "call_id": "robot_capture_1",
      "name": "capture_pair",
      "arguments": "{\"mode\":\"latest\",\"detail\":0}"
    }
  ]
}
```

回调调用 `submit()` 后等待本批句柄就绪，将原生结果按请求顺序放入 `outputs_json` 数组字符串。工具自身的失败位于各结果的 `ok/code/message`；请求解析或提交异常写入服务的 `error`。

服务使用独立回调线程串行处理请求，批内工具按 ToolManager 的工作线程与阻塞配置执行。客户端结束等待后，已经提交的调用继续由 ToolManager 执行。机器人自行发起的结果由调用方处理，并可包装为机器事件交给模型；模型调用结果按对应的模型 `call_id` 回传。

## 4. 工具目录与共享状态

### 4.1 NavigationRuntime

`plugins/tools/navigation` 下的十二个插件共享 NavigateToPose / NavigateThroughPoses Action 客户端、TF、代价地图服务客户端、当前任务和环境变换。

| 工具 | 参数 | 结果 |
| --- | --- | --- |
| `navigate_to_pose_start_navigation` | `frame_id/x/y/yaw` | `data.task_id/status` |
| `navigate_to_pose_get_feedback` | `task_id` | `data` 中的最新状态与反馈，可带公共 `pose` |
| `navigate_to_pose_cancel_navigation` | `task_id` | 取消请求处理状态 |
| `navigate_through_poses_start_navigation` | `frame_id`、非空 `poses: [{x,y,yaw}, ...]` | `data.task_id/status` |
| `navigate_through_poses_get_feedback` | `task_id` | 最新状态、反馈、剩余位姿数量，可带公共 `pose` |
| `navigate_through_poses_cancel_navigation` | `task_id` | 多点导航取消请求处理状态 |
| `get_surroundings` | `costmap: local/global`、`range_m` | `data.environment_id`、`pose`、地图 PNG 与 `images[0].metadata` |
| `navigate_to_pixel_start_navigation` | `environment_id/pixel_x/pixel_y/yaw` | `data.environment_id/task_id/status/goal` |
| `navigate_through_pixels_start_navigation` | `environment_id`、`pixels: [{pixel_x,pixel_y,yaw}, ...]` | `data.environment_id/frame_id/task_id/status/goals` |
| `navigate_to_objects` | `pair_id/use_segmentation/instances[]`，实例可含 `stand_off_m` | `data.task_id/status/action/poses/projection` |
| `start_lookaround` | `{}` | 环顾任务的 `data.task_id/status` |
| `cancel_lookaround` | `task_id` | 取消请求处理状态 |

普通导航在 Action 目标接受后返回，任务继续运行。`get_feedback` 返回的数值包括 `distance_remaining_m`、`navigation_time_sec`、`estimated_time_remaining_sec`、`number_of_recoveries`。任务状态为 `running/succeeded/failed/canceling/canceled/rejected`；取消确认后进入 `canceling`，最终 Action 结果决定终态。

导航任务在 Action 成功、失败或取消后立即释放，结果由行为树事件上报。空闲时可直接启动下一个目标，活动任务期间返回 `NAVIGATION_BUSY`。`get_feedback` 可选查询活动任务，已结束任务返回 `TASK_NOT_FOUND`。共享资源同时维护一个物理导航任务，单点导航、多点导航、像素导航与环顾之间互斥。

多点工具向 `/navigate_through_poses` 提交一个 NavigateThroughPoses 目标，不逐点调用单点导航。所有位姿使用同一 `frame_id` 和采样时间，按 `poses` 数组顺序传递；x/y 单位米，yaw 单位弧度，所有数值必须有限。每次请求显式设置 `behavior_tree`，缺省为 `/home/yan/ros2_ws/src/scout_mini_vln/vln_behavior_tree/behavior_trees/vln_navthroughgoals.xml`。该路径由 Nav2 服务端读取，开发机无需存在相同绝对路径；组设置可覆盖路径，但不允许空值回退至默认树。

例如启动参数 `{"frame_id":"map","poses":[{"x":1.0,"y":2.0,"yaw":0.0},{"x":3.0,"y":4.0,"yaw":1.57}]}`。接受后用返回的 `data.task_id` 调用对应多点反馈、取消工具；多点工具与单点工具不交叉查询或取消任务。反馈除上述通用字段外还包含 `number_of_poses_remaining`。取消确认仅表示 `canceling`，最终 Action 结果才释放任务槽；成功、失败或取消后的 ID 返回 `TASK_NOT_FOUND`，行为树负责上报导航事件。

`get_surroundings` 通过 GetCostmap 读取局部或全局地图，以机器人为中心生成正方形环境图。`range_m` 为边长，图片上方表示截图时机器人前方。地图区域默认 `image_size=512`，完整图片还包含标题、坐标和图例；有效地图范围与比例以返回的 `map_rect`、`meters_per_pixel` 为准。

像素导航使用完整图片左上角为原点，x 向右、y 向下。选择点位于 `map_rect` 内。`yaw=0` 朝图片上方，正值逆时针；转换后的朝向为截图时机器人 yaw 加输入 yaw。环境 ID 保存该次图片到地图的变换，默认最多 8 个，FIFO 淘汰；目标接受后消费对应 ID。

`navigate_through_pixels_start_navigation` 复用同一快照转换和消费逻辑，将 `pixels` 全部转换后一次性调用多点导航，沿用 `navigate_through_poses_behavior_tree`。每个 yaw 仍是相对于截图朝向的弧度，0 朝图上方，正值逆时针。例如 `{"environment_id":"env_1","pixels":[{"pixel_x":256,"pixel_y":200,"yaw":0.0},{"pixel_x":300,"pixel_y":180,"yaw":1.57}]}`；实际有效像素范围以该图的 `map_rect` 为准。

单次点数上限使用导航组 `settings.max_pixel_goals` 配置，缺省 8，必须为正整数；`get_surroundings` 同时返回 `data.max_pixel_goals`，供模型按当前上限组织批次。空数组、超限、无效像素或朝向都会整批返回 `INVALID_ARGUMENTS`，不会截断或发送部分目标；数量错误的 `data.max_pixel_goals` 提供当前上限。Action 不可用、繁忙或拒绝目标时保留环境 ID，可修正后重试；只有目标被接受后才消费该 ID。成功返回 `data.frame_id` 和按输入顺序排列的 `data.goals: [{x,y,yaw}, ...]`，反馈与取消复用 `navigate_through_poses_get_feedback`、`navigate_through_poses_cancel_navigation`。

`navigate_to_objects` 接收 capture_pair 图片上模型检测得到的实例提示。参数与 depth_project 共用 pair_id、use_segmentation 和实例 box/points，投影坐标系固定为导航 map_frame，模型省去 target_frame。每个实例可指定 stand_off_m（米），缺省 0；模型 Schema 也允许 null 表示 0。例如：

```json
{
  "pair_id": "pair_1",
  "use_segmentation": true,
  "instances": [
    {
      "box": {"x_min": 100, "y_min": 60, "x_max": 300, "y_max": 220},
      "points": [{"x": 200, "y": 140, "label": 1}],
      "stand_off_m": 0.5
    },
    {
      "box": null,
      "points": [{"x": 400, "y": 180, "label": 1}],
      "stand_off_m": 0
    }
  ]
}
```

一次 depth_project 完成全部实例的分割与三维均值，继续共享 pair 解码、标定射线及 SAM 图像特征。全部成功后按顺序生成停靠点：首点以当前小车位置为参照，后续点以前一停靠点为参照，沿参照点到物体投影的水平连线后退 stand_off_m。0 直接使用投影 x/y；地面导航使用 x/y，投影中的 z 保留在结果内。中间停靠点 yaw 指向下一停靠点；最后一点面向最后物体，零偏移时采用接近方向，两点重合时延续此前方向。

一个实例复用 start_navigation，多个实例复用 start_through_poses_navigation，后者沿用配置的多点行为树。活动导航直接返回 NAVIGATION_BUSY，省去分割计算。任一实例投影失败时整次不启动，结果保留全部投影明细；Nav2 接受、拒绝及执行终态继续沿用现有 Action/行为树流程。

成功结果的 data 含 task_id、status、action（navigate_to_pose 或 navigate_through_poses）、frame_id、poses 和 projection。poses 是实际发送的有序 x/y/yaw，projection 是完整 depth_project 的 data。按 action 使用相应前缀的反馈和取消工具。函数 settings.rgbd_resource 默认 rgbd/rgbd，执行时查询已有资源，工具组装载顺序可独立安排。纯几何生成完成后直接提交给 Nav2，规划与到达判定由已配置的规划器、控制器和目标检查器处理。navigation 与 RGB-D 工具统一编译到 vln_agent_tool_plugins，pluginlib 使用同一库装载所有工具注册；源码和共享资源继续按组保存。

`start_lookaround` 读取当前 `map_frame → base_frame` 位姿，向同一 NavigateToPose Action 提交环顾 XML。默认路径为 `vln_behavior_tree` 安装目录的 `behavior_trees/lookaround.xml`，可通过 `lookaround_behavior_tree` 指定路径。启动和取消工具在请求确认后返回；环视图由行为树最终的 `lookaround_completed` 事件提交。环顾任务在 Action 结束后自动释放状态，过程信息由行为树事件上报。

组配置：

| 字段 | 缺省值 |
| --- | --- |
| `action_name` | `/navigate_to_pose` |
| `navigate_through_poses_action_name` | `/navigate_through_poses` |
| `navigate_through_poses_behavior_tree` | `/home/yan/ros2_ws/src/scout_mini_vln/vln_behavior_tree/behavior_trees/vln_navthroughgoals.xml` |
| `lookaround_behavior_tree` | 空，按包安装目录定位 |
| `map_frame`、`base_frame` | `map`、`base_link` |
| `local_costmap_service` | `/local_costmap/get_costmap` |
| `global_costmap_service` | `/global_costmap/get_costmap` |
| `costmap_timeout_sec` | `2.0` |
| `image_size` | `512` |
| `max_environments` | `8` |
| `max_pixel_goals` | `8`，多点像素导航单次目标数上限，正整数 |
| `shutdown_timeout_sec` | `2.0` |

导航动作函数的实现位于各插件 `.cpp`；`navigation_runtime.cpp` 管理资源生命周期和通用状态。

### 4.2 RgbdRuntime

`capture_pair` 和 `depth_project` 共享 `plugins/tools/rgbd` 中的图像、标定、TF 与分割资源。

`capture_pair` 的 `mode/detail` 对应 `LATEST_PAIR/QUEUE_PAIR/QUEUE_PAIR_AT_AGE`：latest 的 detail 为 0，index 为从 0 开始的历史索引，age 为距当前的毫秒数。工具保存原始彩色、对齐深度、CameraInfo 和深度采样时的相机变换，返回彩色图和 `data.pair_id`。`images[0].metadata.pair_id` 是同一工具缓存 ID，`source_pair_id` 记录取图服务的原始编号。

彩色图按 `capture_pair.settings.image_scale` 缩放，缺省 1.0；输出宽高取整数，缩小使用面积重采样，放大使用线性插值。1.0 保留原压缩字节。点和框均使用返回图片实际的 `width/height`，左上角为原点，x 向右、y 向下；原始网格尺寸见 `original_width/original_height`。

`depth_project` 在一次请求中处理同一 pair 的多个检测实例：

```json
{
  "pair_id": "pair_1",
  "target_frame": "map",
  "use_segmentation": true,
  "instances": [
    {
      "box": {"x_min": 100, "y_min": 60, "x_max": 300, "y_max": 220},
      "points": [{"x": 200, "y": 140, "label": 1}]
    },
    {
      "box": null,
      "points": [
        {"x": 400, "y": 180, "label": 1},
        {"x": 420, "y": 180, "label": 0}
      ]
    }
  ]
}
```

- `box` 为可选框，填 null 表示纯点提示。框使用图片边界坐标，右/下边界可等于 width/height；点使用图片内的零基整数像素。
- `label=1` 为前景，0 为背景。每个实例的提示相互独立。SAM 每实例最多 64 个 token：纯点包含一个 padding，最多 63 点；带框占两个 token，最多 62 点。
- `use_segmentation=true` 时，MobileSAM 对各实例分别解码，输出原始彩色网格上的单掩码；累加掩码内所有有效原始深度的三维坐标并求均值。
- `use_segmentation=false` 时，读取正点对应的原始深度并求三维均值，框和负点留给分割模式使用。无效深度跳过，同一实例至少有一个有效样本才能成功。

结果的 `data` 包含 `pair_id/frame_id/use_segmentation/success_count/instances`。instances 与输入顺序对应，每项含 `index/ok`；成功项附 `sample_count/x/y/z/depth_m`，分割结果附 `mask_score`，失败项附 `code/message`。部分实例失败时保留其余结果；全部失败时顶层 ok=false，仍保留完整实例明细。x/y/z 是可见表面点的均值，depth_m 是这些样本的平均相机光轴 Z 深度；这些数值用于目标定位，停车位置由导航决策选择。

深度图和 CameraInfo 使用原始对齐网格。正点按实际宽高的像素中心比例取最近原始深度像素；分割提示还原为原图坐标，由 SAM 完成内部最长边 1024 的 CUDA 预处理，原图掩码直接对应深度图。`setResizedImage()` 适用于已完成 SAM 最长边缩放的输入，与发送给模型的 image_scale 独立。

支持 `plumb_bob/rational_polynomial/equidistant` 标定及 compressedDepth PNG。16UC1 按毫米恢复，32FC1 使用头部逆深度参数。投影使用 K/D 去畸变后得到的射线。捕获时按深度采样时间保存相机到固定系的变换；fixed_frame 缺省 map。其他 target_frame 使用调用时的目标系 TF，每批查询一次。三维点先在相机系求均值，再对均值施加组合刚性变换。

性能与资源：

- 彩色图沿用捕获时已解码的原图，深度按 pair 解码一次。
- 同一标定共享惰性射线表，掩码归约按行并行执行，计算过程直接读取栅格。
- 默认配置在插件初始化时创建并预热 TensorRT 模型；最近一个真实 pair 的图像特征常驻 GPU。重复查询该 pair 只运行解码器；换图后重新编码。
- 单个 TensorRT context 由互斥锁串行使用，同一请求的实例逐个处理；正点查询可并发执行。
- 图像缓存按 max_pairs（缺省 8）FIFO 淘汰。旧标定随引用它的 pair 释放，工具结果由调用方接收。

开发板纯投影实测（OpenCV 6 线程，预热 10 次后采样 100 次，掩码覆盖全图，5% 深度为空）：

| 原始分辨率 | 射线表首次构建 | 缓存后投影均值 | P95 | 射线表内存 |
| --- | --- | --- | --- | --- |
| 640×360 | 29.08 ms | 0.290 ms | 0.330 ms | 3.52 MiB |
| 1280×720 | 108.31 ms | 1.060 ms | 1.175 ms | 14.06 MiB |
| 1920×1080 | 241.88 ms | 2.146 ms | 2.491 ms | 31.64 MiB |

掩码覆盖 25% 时对应均值为 0.119/0.447/0.774 ms；63 个正点的投影约 0.009 ms。这些数据只包含去畸变射线查询和三维均值，SAM、压缩图解码、TF 和 ROS 调度另外计时。424×235 样例图的两个实例，缓存图像后通过 ToolManager 完成分割和投影的均值为 20.18 ms，首次调用含引擎初始化为 1.85 s。样例使用恒定合成深度，验证接口和几何计算；实际相机的分割质量及深度噪声取决于现场数据。

复测命令：`./build/vln_agent/benchmark_depth_project`。测量程序位于 `test/benchmark_depth_project.cpp`，随 BUILD_TESTING 编译。

组配置包含 `service_name/camera_info_topic/fixed_frame/timeout_sec/max_pairs`，及 `segmentation`：

```yaml
segmentation:
  warmup_on_initialize: true
  engine_directory: /path/to/segment_tools/models/tensorrt/engines
  encoder_precision: tf32
  decoder_precision: fp32
  cuda_graph: true
```

warmup_on_initialize 在默认 agent.yaml 中开启。RGB-D 共享资源首次创建时，用临时 1024×1024 图像执行编码和常用的 2/3 token 提示解码；临时图不生成 pair_id，第一张真实图会重新编码。新提示长度仍可能重新捕获 CUDA Graph。自定义配置省略该开关或设为 false 时延迟到首次分割加载；缺少分割库的构建保留正点投影能力。预热失败作为初始化错误上报。

编码器与解码器各自支持 fp16/tf32/fp32。库通过 CMake 的 `mobile_sam_trt` 包链接，默认搜索相邻 segment_tools 的 tensorrt_lib/install，也可指定 mobile_sam_trt_DIR。开发机缺少该库时可编译正点投影；分割请求返回 SEGMENTATION_UNAVAILABLE。`VLN_AGENT_WITH_MOBILE_SAM=OFF` 可显式关闭链接。真实分割测试在开发板通过 `VLN_SAM_ENGINE_DIR` 与 `VLN_SAM_TEST_IMAGE` 环境变量开启。

## 5. SessionManager

公共接口见 [session_manager.hpp](../include/vln_agent/session/session_manager.hpp)。

### 5.1 接入与发送流程

构造函数读取 `session_manager` 配置并启动网络和调度线程。初始化内容由调用方添加：

```cpp
vln_agent::SessionManager session(config_path);
session.setTools(tool_definitions);
session.addInputs(initial_inputs);
session.request();

auto response = session.takeResponse();
```

| 方法 | 行为 |
| --- | --- |
| `setTools(Json)` | 替换下次请求使用的工具数组 |
| `addInputs(const Json &)` | 追加一个原生对象，或按元素追加数组 |
| `addInputs(const Json::array_t &)` | 整批追加原生对象数组 |
| `request()` | 登记发送意图，重复调用合并 |
| `takeResponse()` | 取走完整响应；无结果时为 `null` |
| `shutdown()` | 停止调度、连接并释放会话数据 |

会话维护一份当前请求、一份待发输入和一个完整响应槽。模型生成期间，新输入继续追加至待发数组。连接可用、待发输入非空、达到限频间隔，且当前轮与结果槽已交付时，调度线程组装下一次请求。

限频以相邻两次发送起点计算。`takeResponse()` 取走完整响应后，结果槽与流式文本一起释放；失败请求保留完整请求，供显式重试。

### 5.2 请求配置

```yaml
session_manager:
  base_url: http://127.0.0.1:8317/v1
  api_key: YOUR_API_KEY
  model: gpt-6.1-terra
  store: true
  reasoning_effort: low
  max_output_tokens: 8192
  parallel_tool_calls: true
  prompt_cache_key: scout-mini-vln
  max_request_hz: 5.0
  request_timeout_sec: 0
  stream_output: true
  context:
    local_history: false
    server_compaction: true
    budget_tokens: 128000
    compact_ratio: 0.8
```

每次发送 `type: "response.create"`，填入模型配置、当时的 `tools` 和本轮 `input`，文本格式为 `text.format.type: "text"`。`request_timeout_sec=0` 表示持续等待模型结果。`stream_output` 控制显示文本快照，完整响应仍由 `takeResponse()` 交付。

### 5.3 上下文与压缩

| 模式 | 配置 | 下一轮请求 |
| --- | --- | --- |
| 服务端链 | `local_history: false` | 使用最近完成的 `previous_response_id` 和新增输入 |
| 本地有效上下文 | `local_history: true` | `store: false`，重放有效输入与原生模型输出 |

本地模式在成功轮次提交 `current_request.input + response.output`，作为后续请求的上下文。收到 `compaction` 项后，保留最新压缩项及之后的内容，释放被替代的前缀。失败轮的新输入留作重试。

`compact_threshold = floor(budget_tokens × compact_ratio)`。开启 `server_compaction` 时，请求带上相应的 `context_management`；服务端执行压缩，返回的用量和压缩项用于会话状态。`budget_tokens` 是阈值计算的预算基数。

系统提示词由 Agent 在首次 `input` 中作为 `role: "system"` 消息提交，后续通过所选上下文模式续接。提示词文件的读取由 Agent 完成。

### 5.4 查询结果与错误

| 方法 | 返回内容 |
| --- | --- |
| `getTools()` | 下次请求的工具定义数组副本 |
| `getPendingInputs()` | 待发输入数组副本 |
| `getCurrentRequest()` | 在途或失败、中断后保留的完整请求副本；无当前请求时为 `null` |
| `getRequestStatus()` | 请求状态、计时、输入计数、重试标志、会话错误与 `transport` |
| `getContextStatus()` | 上下文模式、链头、有效项数、预算、usage 和压缩计数 |
| `getStreamSnapshot()` | 当前显示文本对象；关闭显示时为 `null` |

请求状态字段：

```json
{
  "current_response_id": "resp_1",
  "retry_required": false,
  "can_retry": false,
  "request_elapsed_ms": 500,
  "rate_limit_remaining_ms": 0,
  "state": "generating",
  "send_requested": false,
  "pending_input_count": 1,
  "current_input_count": 2,
  "in_flight": true,
  "response_ready": false,
  "request_interrupted": false,
  "last_error": null,
  "transport": {
    "state": "connected",
    "connected": true,
    "connection_id": 1,
    "reconnect_attempts": 0,
    "last_error": null
  }
}
```

会话状态为 `idle/waiting_interval/waiting_transport/generating/response_ready/error/closed`。请求结束后 `request_elapsed_ms` 固定为该轮耗时。

上下文状态字段为 `latest_response_id`、`mode`（`server/local`）、`history_item_count`、`budget_tokens`、`compact_threshold`、`usage`、`compaction_count`。`usage` 保留服务端返回的结构。

会话错误形如 `{"code":"REQUEST_TIMEOUT","message":"...","event":null}`。模型失败或不完整结果保留原始错误事件；完整响应结构和事件解析异常使用 `INVALID_RESPONSE`。传输错误位于 `transport.last_error`，包含 `stage/message`。

失败或中断后，`retry_required` 表示当前请求待处理；`can_retry` 为真时，调用 `request()` 将失败输入放回待发数组前部并登记重试。服务端链出现 `previous_response_not_found` 时，当前链标记为失效，`can_retry` 为假。

流式显示示例：

```json
{
  "response_id": "resp_1",
  "state": "generating",
  "content": [
    {
      "output_index": 0,
      "content_index": 0,
      "type": "output_text",
      "text": "我正在查看"
    }
  ]
}
```

每项 `text` 是该位置当前的完整累计文本。流式结束后以完整响应校准显示内容；调用 `takeResponse()` 后快照回到空的 `idle` 状态。

### 5.5 ResponsesWsTransport

传输层将 HTTP/HTTPS 基址转换为 WS/WSS Responses 地址，认证位于 Authorization 头。WSS 使用系统信任链和主机名验证。连接状态由 `connected()`、`getStatus()` 查询。

网络默认参数来自 `WsTransportOptions`：

| 参数 | 缺省值 |
| --- | --- |
| `connect_timeout` | 10000 ms |
| `write_timeout` | 10000 ms |
| `idle_timeout` | 60000 ms |
| `reconnect_delay` | 250 ms |
| `max_reconnect_delay` | 5000 ms |

网络故障进入重连退避，重连后恢复连接可用性。业务请求的重新提交由会话调用方登记。模型超时会调用 `interrupt()` 换连接，使旧轮迟到数据与下一轮连接分开；`close()` 结束网络线程。

## 6. UserInteractionManager

公共接口见 [user_interaction_manager.hpp](../include/vln_agent/interaction/user_interaction_manager.hpp)。

### 6.1 交互调用与插件

`UserInteractionSources` 保存 SessionManager、ToolManager 和 EventManager 的弱引用。当前汇总信息从 SessionManager 和 ToolManager 查询，工具定义在装载时查询一次。

```cpp
Json exchange(const Json & new_response,
              const Json & tool_outputs,
              const Json & finished_calls);
```

调用方传入本轮新取出的完整响应、原生工具结果数组和对应终态数组。无新响应时使用 `null`，无新工具结果时使用两个空数组。工具结果与终态按 `call_id` 一一对应。

交互插件继承 `UserInterfaceBase`：

```cpp
void initialize(const std::shared_ptr<rclcpp::Node> & node,
                const Json & settings) override;
Json exchange(const std::shared_ptr<const Json> & outgoing) override;
void shutdown() noexcept override;
```

插件返回已包装好的原生输入数组。`outgoing` 为当前汇总包的只读共享指针，`nullptr` 表示本次仅收输入。管理器和 ROS 接收回调在宿主线程中串行执行。

### 6.2 每个渠道独立选择传输内容

每个 `user_interaction.channels[]` 配置自己的 `plugin`、`transmit`、`include_images` 和 `settings`。省略的传输字段默认为关闭。

| 配置路径 | 数据来源 |
| --- | --- |
| `request` | `getCurrentRequest()`，展示副本移除 `tools` |
| `response.messages` | 本次完整响应与当前流式文本 |
| `response.function_calls` | 本次响应中的函数调用数组 |
| `response.tool_outputs` | 本次传入的工具结果数组 |
| `response.stream` | 当前流式响应 ID 与状态 |
| `tools.definitions` | 装载时取得的工具定义，每批随包发送 |
| `tools.status` | ToolManager 状态 |
| `tools.queue` | 等待与运行队列快照 |
| `tools.finished_calls` | 本次传入的终态计时数组 |
| `session.request_status` | SessionManager 请求状态 |
| `session.context_status` | SessionManager 上下文状态 |

每次 `exchange()` 生成启用字段的当前完整快照。`request` 取在途或失败保留的请求；待发输入通过 SessionManager 独立查询。完整响应、工具结果和终态数组采用本次传入的数据。

汇总包按下列结构分组，状态对象的字段见对应组件查询接口：

```json
{
  "type": "interaction_update",
  "batch_id": 1,
  "time_stamp": "2026-11-09T06:00:00.000Z",
  "request": null,
  "response": {
    "id": "resp_1",
    "status": "completed",
    "messages": [
      {
        "response_id": "resp_1",
        "output_index": 0,
        "content_index": 0,
        "type": "output_text",
        "text": "已开始导航。"
      }
    ],
    "function_calls": [],
    "tool_outputs": [],
    "stream": {"response_id": "", "state": "idle"}
  },
  "tools": {
    "definitions": [],
    "status": {},
    "queue": {"waiting": [], "running": []},
    "finished_calls": []
  },
  "session": {"request_status": {}, "context_status": {}}
}
```

文本项以 `response_id/output_index/content_index` 作为位置标识，客户端用当前整段 `text` 更新该位置。图片开关关闭时，原生图片项替换为 `{"type":"input_image","omitted":true}` 等占位对象；其余展示字段继续传输。展示处理将 `created_at/completed_at/expires_at/failed_at/cancelled_at` 数值字段转换为对应的 UTC `*_time_stamp`。

相同配置的插件共用一份只读包。管理器逐个投递，所有插件正常返回后释放本批数据；某插件抛异常时保留该批，重试中已接收插件得到 `nullptr`。调用方在成功交付后释放本次完成事件，并管理其历史记录。

### 6.3 RosTopicInterface

默认用户接口使用 `std_msgs/msg/String`，发布 `/vln/user_output`，订阅 `/vln/user_input`，队列深度为 10。接收话题中的 `data` 为原生输入对象或数组的 JSON 字符串。`exchange()` 发布汇总包，并取走当前收件数组。

```yaml
user_interaction:
  channels:
    - id: ros_ui
      plugin: vln_agent/RosTopicInterface
      transmit:
        request: true
        response:
          messages: true
          function_calls: true
          tool_outputs: true
          stream: true
        tools:
          definitions: true
          status: true
          queue: true
          finished_calls: true
        session:
          request_status: true
          context_status: true
      include_images: false
      settings:
        send_topic: /vln/user_output
        receive_topic: /vln/user_input
```

`id` 可作为配置中的人工标注，插件参数来自 `settings`。

## 7. EventManager

公共接口见 [event_manager.hpp](../include/vln_agent/events/event_manager.hpp)。

机器事件直接使用第 2 节的原生用户消息格式。ROS 接收话题由 `event_manager.receive_topic` 指定，默认 `/vln/robot_event`，类型为 `std_msgs/msg/String`，QoS 为 reliable、volatile、KeepLast(10)。

| 方法 | 行为 |
| --- | --- |
| `load(config_path)` | 创建接收订阅 |
| `addEvents(Json)` | 追加单条事件或整批事件 |
| `takeEvents()` | 交换并取走全部待交付事件，空时返回 `[]` |
| `getStatus()` | 返回 `state` 和 `pending_count` |
| `shutdown()` | 释放订阅并清空待交付事件 |

`state` 为 `unloaded/ready/closed`。事件缓存和 ROS 回调通过同一把互斥锁访问。一次消息中的多个事件按原顺序追加，`takeEvents()` 可以同时取出来自多个发布者的消息。

事件中的 `instruction` 可表达文字指令或 `{"function":"capture_pair"}` 这样的工具建议。Agent 将事件交给模型，模型结合上下文生成消息或函数调用；ToolManager 派发实际生成的 `function_call`。

## 8. VLNAgent 主循环

`init()` 在 `make_shared` 构造后调用，读取系统提示词，依次装载 EventManager、ToolManager、SessionManager 和 UserInteractionManager，再提交初始化输入。

ROS 参数：

| 参数 | 缺省值 | 含义 |
| --- | --- | --- |
| `global_yaml_path` | 包安装目录的 `config/agent.yaml` | 组件配置路径 |
| `system_prompt_path` | 空，启动时填写 | 系统提示词文件 |
| `initial_instruction` | 空 | 可选的首条用户指令 |
| `loop_hz` | `20.0` | 主循环频率 |

`run()` 使用单线程 ROS 执行器和 `WallRate`。每轮依次执行：

1. 处理 ROS 回调，取走机器事件并追加会话输入。
2. 取走完整模型响应，将其中的工具调用交给 ToolManager。
3. 检查活动 future，把已完成结果追加至会话，同时收集终态计时。
4. 调用用户交互组件，交付当前响应和工具结果，再接收用户输入。
5. 根据待发数量和重试标志登记正常发送，输出会话错误日志。

模型传输和工具执行分别由组件线程处理。会话失败恢复通过 `SessionManager::request()` 显式登记，当前 Agent 主循环按 `retry_required` 区分正常输入与待恢复请求。

`getHistory()` 返回 Agent 输入记录的只读引用；每次 `addInputs()` 保存该批输入和交付时间。完整响应由当前轮处理并传给交互层。

## 9. 数据保存与通用辅助

| 对象 | 保存的数据与释放时机 |
| --- | --- |
| `VLNAgent` | `history_` 累积提交的输入；活动调用句柄在结果取出后释放 |
| `SessionManager` | 当前请求、待发输入、一个完整响应及当前有效上下文；成功或显式恢复时更新，关闭时释放 |
| `ToolManager` | 等待与运行调用，完成时移出；累计计数保留为标量 |
| `UserInteractionManager` | 工具定义、一个交付批次和待交付用户输入，批次成功后释放 |
| `EventManager` | 待交付事件，`takeEvents()` 后清空 |
| `NavigationRuntime` | 当前导航任务及有限数量的环境变换 |
| `RgbdRuntime` | 有界图像对、共享标定射线表、惰性深度解码及一个 SAM 模型和当前图像特征 |

待发输入、事件缓冲和本地有效上下文按数据交付及压缩进度保存。Agent 输入记录持续累积。

`utils::businessPayload()`、`posePayload()`、`appendInputs()` 和 `functionCallOutput()` 共用业务封装与图片描述检查。JSON/YAML 类型转换、UTC 时间和 Responses 地址解析也位于 `utils`。

Base64 接口为 `encodeBase64(vector)` 和 `decodeBase64(string_view)`。底层静态集成 libbase64 v0.5.3，x86 运行时选择 SIMD，ARM64 使用 NEON，其他平台使用通用实现。输出一次分配，首次 CPU 分派采用线程安全初始化，解码保留标准字符、长度、填充位置和未使用位检查。依赖源码与许可证随包提供。
