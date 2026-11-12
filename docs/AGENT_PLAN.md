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
    tools/{navigation,capture_image,rgbd}/
    interfaces/ros_topic_interface/
src/                           # 组件实现
plugins/
  tools/{navigation,capture_image,rgbd}/
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

每个函数对应一个 `ToolPluginBase` 派生类，工具名采用下划线，类名采用 PascalCase，例如 `get_depth_point` 对应 `GetDepthPoint`。

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

`plugins/tools/navigation` 下的七个插件共享 NavigateToPose Action 客户端、TF、代价地图服务客户端、当前任务和环境变换。

| 工具 | 参数 | 结果 |
| --- | --- | --- |
| `navigate_to_pose_start_navigation` | `frame_id/x/y/yaw` | `data.task_id/status` |
| `navigate_to_pose_get_feedback` | `task_id` | `data` 中的最新状态与反馈，可带公共 `pose` |
| `navigate_to_pose_cancel_navigation` | `task_id` | 取消请求处理状态 |
| `get_surroundings` | `costmap: local/global`、`range_m` | `data.environment_id`、`pose`、地图 PNG 与 `images[0].metadata` |
| `navigate_to_pixel_start_navigation` | `environment_id/pixel_x/pixel_y/yaw` | `data.environment_id/task_id/status/goal` |
| `start_lookaround` | `{}` | 环顾任务的 `data.task_id/status` |
| `cancel_lookaround` | `task_id` | 取消请求处理状态 |

普通导航在 Action 目标接受后返回，任务继续运行。`get_feedback` 返回的数值包括 `distance_remaining_m`、`navigation_time_sec`、`estimated_time_remaining_sec`、`number_of_recoveries`。任务状态为 `running/succeeded/failed/canceling/canceled/rejected`；取消确认后进入 `canceling`，最终 Action 结果决定终态。

导航任务在 Action 成功、失败或取消后立即释放，结果由行为树事件上报。空闲时可直接启动下一个目标，活动任务期间返回 `NAVIGATION_BUSY`。`get_feedback` 可选查询活动任务，已结束任务返回 `TASK_NOT_FOUND`。共享资源同时维护一个物理导航任务。

`get_surroundings` 通过 GetCostmap 读取局部或全局地图，以机器人为中心生成正方形环境图。`range_m` 为边长，图片上方表示截图时机器人前方。地图区域默认 `image_size=512`，完整图片还包含标题、坐标和图例；有效地图范围与比例以返回的 `map_rect`、`meters_per_pixel` 为准。

像素导航使用完整图片左上角为原点，x 向右、y 向下。选择点位于 `map_rect` 内。`yaw=0` 朝图片上方，正值逆时针；转换后的朝向为截图时机器人 yaw 加输入 yaw。环境 ID 保存该次图片到地图的变换，默认最多 8 个，FIFO 淘汰；目标接受后消费对应 ID。

`start_lookaround` 读取当前 `map_frame → base_frame` 位姿，向同一 NavigateToPose Action 提交环顾 XML。默认路径为 `vln_behavior_tree` 安装目录的 `behavior_trees/lookaround.xml`，可通过 `lookaround_behavior_tree` 指定路径。启动和取消工具在请求确认后返回；环视图由行为树最终的 `lookaround_completed` 事件提交。环顾任务在 Action 结束后自动释放状态，过程信息由行为树事件上报。

组配置：

| 字段 | 缺省值 |
| --- | --- |
| `action_name` | `/navigate_to_pose` |
| `lookaround_behavior_tree` | 空，按包安装目录定位 |
| `map_frame`、`base_frame` | `map`、`base_link` |
| `local_costmap_service` | `/local_costmap/get_costmap` |
| `global_costmap_service` | `/global_costmap/get_costmap` |
| `costmap_timeout_sec` | `2.0` |
| `image_size` | `512` |
| `max_environments` | `8` |
| `shutdown_timeout_sec` | `2.0` |

导航动作函数的实现位于各插件 `.cpp`；`navigation_runtime.cpp` 管理资源生命周期和通用状态。

### 4.2 CaptureImage

`capture_image` 调用 `rgbd_image_queue/srv/GetImage` 并返回一张原始彩色压缩图。

| `mode` | `detail` | 服务操作 |
| --- | --- | --- |
| `latest` | `0` | `LATEST_COLOR` |
| `index` | 历史索引，0 为最新 | `QUEUE_COLOR` |
| `age` | 距当前的毫秒数 | `QUEUE_COLOR_AT_AGE` |

组配置提供 `service_name`，函数配置 `timeout_sec` 缺省为 `2.0`。图片按 JPEG/PNG 类型编码为 data URL，配对 ID、索引、年龄等信息位于 `images[0].metadata`。

### 4.3 RgbdRuntime

`capture_pair`、`get_depth_point` 和 `get_depth_points` 共用 `plugins/tools/rgbd` 中的资源。

`capture_pair` 的 `mode/detail` 与彩色取图相同，服务操作改为 `LATEST_PAIR/QUEUE_PAIR/QUEUE_PAIR_AT_AGE`。工具保存彩色、深度、CameraInfo 和采样时相机变换，向模型返回按 `capture_pair.settings.image_scale` 等比例缩放的彩色图和 `data.image_pair_id`。缩放因子缺省为 `1.0`，例如 `0.5` 将宽高各缩为一半；尺寸四舍五入到整数，最小为 1。

```json
{
  "image_pair_id": "pair_1",
  "pixel_x": 640,
  "pixel_y": 360,
  "target_frame": "map"
}
```

以上是 `get_depth_point` 的参数。整数像素使用 `images[0].metadata.width/height` 对应的返回彩色图网格，左上角为原点，x 向右、y 向下。`original_width/original_height` 表示缓存的原始网格。工具按实际宽高的像素中心比例映射到最近的原始深度像素，再使用原始标定反投影。结果的 `data` 包含 `image_pair_id/pixel_x/pixel_y/depth_pixel_x/depth_pixel_y/depth_m/frame_id/x/y/z`，其中 `depth_pixel_x/depth_pixel_y` 是实际读取的原始深度像素。x/y/z 的单位为米，`depth_m` 为相机光轴 Z 深度。

深度已经对齐到彩色光学系及像素网格，投影使用彩色 CameraInfo 的 K/D。支持 `plumb_bob`、`rational_polynomial`、`equidistant` 相机模型和 compressedDepth PNG：16UC1 按毫米读取，32FC1 按头部量化参数恢复米制深度。

捕获时以深度采样时间保存相机到固定系的变换。默认固定系为 `map`；查询其他目标系时，固定系中的点再换算到目标系当前的位置。历史取图依赖采样时刻仍可查询的 TF。越界或无效深度返回 `DEPTH_UNAVAILABLE`，ID 过期返回 `IMAGE_PAIR_NOT_FOUND`，坐标变换异常返回 `TF_UNAVAILABLE`。

`get_depth_points` 在一次调用中查询同一 pair 的 1～64 个像素，复用 `getDepthPoints()`，按输入顺序返回逐点结果：

```json
{
  "image_pair_id": "pair_1",
  "pixels": [
    {"pixel_x": 320, "pixel_y": 180},
    {"pixel_x": 420, "pixel_y": 200}
  ],
  "target_frame": "map"
}
```

返回 `data.image_pair_id/frame_id/success_count/points`。每个 points 元素包含输入 pixel_x/pixel_y 和 ok；成功项附带 depth_pixel_x/depth_pixel_y/depth_m/x/y/z，失败项附带 code/message。ID 和输出坐标系在批次层统一给出。部分点失败时其余点继续返回；全部失败时顶层 ok=false，顶层 code/message 取第一项错误，data.points 保留全部明细。同一个 pair 在整批期间固定，深度解码及目标坐标变换由已有共享实现复用，批量工具仅整理结果。

组配置为 `service_name`、`camera_info_topic`、`fixed_frame`、`timeout_sec`、`max_pairs`；后四项缺省为 `/camera/color/camera_info`、`map`、`2.0`、`8`。缓存按 FIFO 淘汰，同一 ID 可重复和并发查询；深度在首次查询时解码一次。相机模式在图像队列期间保持一致，CameraInfo 在捕获时保存快照。彩色图的实际分辨率及解码后的深度分辨率均与 CameraInfo 核对；缩放只处理返回的彩色图，缓存深度和标定保持原始网格。缩小使用面积重采样，放大使用线性插值，1.0 保留原压缩字节。

模型按返回的 width/height 报告像素坐标。模型服务可能继续缩放图像，其内部视觉网格由服务决定；工具采用明确的外部图片坐标约定。参考 [OpenAI 图像输入说明](https://developers.openai.com/api/docs/guides/images-vision)。

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

事件中的 `instruction` 可表达文字指令或 `{"function":"capture_image"}` 这样的工具建议。Agent 将事件交给模型，模型结合上下文生成消息或函数调用；ToolManager 派发实际生成的 `function_call`。

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
| `RgbdRuntime` | 有限数量的图像对、标定和首次查询后解码的深度 |

待发输入、事件缓冲和本地有效上下文按数据交付及压缩进度保存。Agent 输入记录持续累积。

`utils::businessPayload()`、`posePayload()`、`appendInputs()` 和 `functionCallOutput()` 共用业务封装与图片描述检查。JSON/YAML 类型转换、UTC 时间和 Responses 地址解析也位于 `utils`。

Base64 接口为 `encodeBase64(vector)` 和 `decodeBase64(string_view)`。底层静态集成 libbase64 v0.5.3，x86 运行时选择 SIMD，ARM64 使用 NEON，其他平台使用通用实现。输出一次分配，首次 CPU 分派采用线程安全初始化，解码保留标准字符、长度、填充位置和未使用位检查。依赖源码与许可证随包提供。
