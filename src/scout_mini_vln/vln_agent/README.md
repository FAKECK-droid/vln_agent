# vln_agent

ROS 2 Humble 功能包，实现 docs/AGENT_PLAN.md 第二、三部分的工具层和会话层。
两层通过原生 JSON 由宿主衔接，不依赖 vln_behavior_tree，不创建 Agent 主循环。

## 组成

- `ToolPluginBase`：固定的 initialize / execute / shutdown 插件接口。
- `SharedResources`：每个管理器独立的具名、带类型资源注册表。
- `ToolManager`：装载定义和 pluginlib 插件、FIFO 调度、调用去重及逐调用 future。
- `plugins/navigate_to_pose`：启动、查询、取消三个插件共用一个 NavigationRuntime。
- `plugins/capture_image`：订阅最新原始图像，返回 JPEG 图片与采样信息。
- `SessionManager`：独立上下文链、两份输入缓冲、发送限频与单个完整结果槽。
- `ResponsesWsTransport`：独立 WS/WSS 连接、认证、保活、网络超时和重连。
- `src/utils/`：YAML/JSON 转换、输入数组操作、工具结果包装、Base64 编解码与 WS URL 解析。

共享资源继承 `SharedResource` 并实现幂等 `shutdown() noexcept`。
资源工厂立即执行，不保存；同名资源类型不一致时报配置错误。
插件不访问 SessionManager、InputManager 或 EventManager。

## 宿主接入

下游使用 `find_package(vln_agent REQUIRED)`，链接
`vln_agent::vln_agent_tools`。头文件位于 `vln_agent/tools/`。

```cpp
#include <vln_agent/tools/tool_manager.hpp>

// node 由宿主持有；其 ROS 执行器在 load 前启动，直到 shutdown 返回。
vln_agent::ToolManager tools(node);
tools.load("/home/yan/ros2_ws/install/vln_agent/share/vln_agent/config/agent.yaml");
auto definitions = tools.getTools();  // 供会话层填入原生 tools 数组。
auto calls = tools.submit(completed_response);
for (const auto & call : calls) {
  if (call.result.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    auto input_item = call.result.get();  // 单个 function_call_output。
    // 由调用方加入后续请求；每个 call_id 只需回传一次。
  }
}
// 宿主持有尚未就绪的句柄，在后续循环继续检查。
tools.shutdown();
// 此后才停止执行器、回收执行器线程和关闭 ROS。
```

构造函数仅保存节点的非拥有引用；宿主必须保证节点存活。
`submit()` 接受 `status: completed` 的原始完整响应，按 output 顺序处理
function_call，忽略 message、reasoning、compaction。
没有函数调用时返回空数组。普通响应结构错误抛异常；
未知工具、参数解析失败、队列满和执行异常分别完成对应调用的失败结果。

同一管理器内，重复 call_id 返回原句柄，即使重提参数不同也不会再次执行。
去重记录和结果保留到管理器销毁；调用方负责收集和提交已就绪结果。
工具返回 `ToolResult{payload, images}`，payload 至少包含布尔值 ok。
管理器只生成 API 外层，不添加 source、time_stamp 或 pose。

## 配置与扩展

只读取全局 YAML 中的 `tool_manager` 段；示例见 config/agent.yaml。
definition 相对于 YAML 所在目录解析，也接受绝对路径。
每组 tool.json 是原生 function 数组，YAML 的 functions 只选择启用项。
getTools 保留定义内容，按组与函数的配置顺序展平。
组 ID、启用函数名须唯一。装载失败会关闭已经创建的插件和资源，
不暴露半套定义；修正后可以重新 load，成功装载后不支持热更新。

每个 function 对应一个无参构造的插件类，导出名称为
`vln_agent/PascalCase函数名`，共享库可导出多个类。
外部功能包可以继承相同接口，通过
`pluginlib_export_plugin_description_file(vln_agent ...)` 注册插件。
组 settings 原样注入 ToolContext；函数 settings 传给 initialize，省略时为空对象。
非阻塞函数的同一插件实例可能并发执行，临时状态须放在本次调用内。

| 本地配置 | 默认值 | 含义 |
| --- | --- | --- |
| max_concurrent_calls | 4 | 活动 execute 上限，须大于零 |
| max_pending_calls | 64 | FIFO 等待项上限，须大于零 |
| blocking | true | 本项返回前不再调度后续项，先前非阻塞项继续运行 |
| 导航组 action_name | /navigate_to_pose | Nav2 NavigateToPose Action 名称 |
| 导航组 shutdown_timeout_sec | 2.0 | 关闭时等待未确认目标回复的上限，须非负 |
| 相机组 camera_topic | 必填 | sensor_msgs/msg/Image 话题，使用 SensorDataQoS |
| 相机函数 timeout_sec | 2.0 | 首帧等待上限，须大于零 |

async 是函数定义字段，blocking 是本地调度字段，二者独立。
模型请求的 parallel_tool_calls 由 SessionManager 设置。
队列保证 FIFO 取出和提交顺序，工作线程的实际开始及完成顺序可不同。
一个阻塞项不会等待此前所有非阻塞项结束；它自身完成即允许后续项调度。

## 导航与相机行为

导航只允许一个活动任务，再次启动返回 NAVIGATION_BUSY。
启动只等服务端接受或拒绝；后续运动由 ROS Action 维持，不占工作线程槽。
本地 task_id 与 API call_id 分开；查询和取消须使用启动返回的 task_id。
查询成功时 ok 为 true，即使物理任务 status 为 failed。
取消接受后为 canceling，只有最终 Action 结果才置为 canceled。
终态记录保留到管理器关闭，新任务的回调不会覆盖旧任务。

相机无函数参数，首帧到达后使用最近收到的帧，不保证每次调用采到新帧。
结果 payload 含 frame_id、stamp、width、height，图片以原生
input_image 的 data:image/jpeg;base64 URL 返回，不把图片放入 payload。

shutdown 停止入队，立即完成未执行调用，通知插件和共享资源退出，
回收执行线程后释放插件及加载器，不 detach 线程。
内置插件为独占回调组持有可回收的 ROS 执行线程，在卸载前结束相关回调。
导航关闭时尽力请求取消本客户端目标，不把取消请求当作物理停止确认；
服务端失联或目标回复超时时会记录警告。真实导航必须由外部导航栈和急停机制保证安全。

## 会话层接入

下游链接 `vln_agent::vln_agent_session`，头文件位于
`vln_agent/session/`。该共享库不依赖 ROS、工具插件或 UI；通用辅助库也可单独链接
`vln_agent::vln_agent_utils`。公开接口中的 `Json` 别名统一定义在
`vln_agent/utils/json.hpp`。

```cpp
#include <vln_agent/session/session_manager.hpp>

vln_agent::SessionManager session(config_path);
session.setTools(tool_definitions);
session.addInputs(vln_agent::Json::array_t{system_message, initialization_request});
session.request();  // 非阻塞；初始化内容由宿主显式提供。

// 宿主后续循环中轮询：只把完整 response 交给工具层。
auto response = session.takeResponse();
if (!response.is_null()) {
  // 由宿主展示 message，并将 response 交给 ToolManager::submit()。
}
// 以后追加用户输入、机器人消息或已经就绪的 function_call_output。
session.addInputs(input_item);
session.request();
session.shutdown();
```

每个输入必须已经是 API 原生输入对象；传入 Json 数组或 Json::array_t 时逐项追加，
整批只加一次锁，保持相邻顺序。仅添加输入或更新工具不会发送请求。
request() 合并发送意图，内部调度器在连接可用、上一轮结束、结果已取走并满足限频后发送。
在途 current_request 保持不变；新输入持续进入 pending_input。
发送时才读取工具列表和最新完成的 previous_response_id，不重复发送旧历史。

takeResponse() 返回原始 event.response，尚无结果时返回 null。
reasoning、compaction、usage 和扩展字段保留原样，流式函数参数不作为工具调用输出。
stream_output 仅控制 getStreamSnapshot() 的本轮显示快照，关闭时返回 null；
快照按 output_index/content_index 区分文本，不消费完整结果。
getContextStatus() 报告最近原始 usage 和已观察到的压缩次数，不估算实时窗口占用。

只读取全局 YAML 的 session_manager 段，示例见 config/agent.yaml。
先填写实际 base_url/api_key；127.0.0.1 指运行程序所在设备，Jetson 连接桌面服务时需改为可达地址。
max_request_hz 按相邻发送开始时刻限制频率，定时条件变量自行唤醒。
request_timeout_sec 为零时允许无限等待模型；网络阶段有独立超时和保活。
WS 请求采用 response.create，context_management.compact_threshold 为
floor(budget_tokens * compact_ratio)，服务端压缩后继续使用最新响应 ID。
配置预算是应用策略，不代表模型的硬上限。

getRequestStatus() 返回 idle、waiting_interval、waiting_transport、generating、
response_ready、error 或 closed，并附带底层 transport 快照。
模型 error/failed/incomplete 和模型超时记录到 last_error；保留当前与待发输入并清除发送意图。
再次 request() 才把未完成输入放回待发数组前部，按最新已完成链头重试。
previous_response_not_found 保持上下文错误，宿主须决定如何重建会话。
网络故障只记录在 transport.last_error，底层重连不会重放业务请求；
若中断了在途请求，恢复后也须显式 request()。
模型超时主动轮换底层连接，防止旧请求的迟到事件污染下一轮。

传输层支持 http/https/ws/wss 基地址、IPv6 和末尾斜杠，统一接到 /responses。
认证仅使用 Authorization 头；WSS 校验系统 CA 和主机名。
所有 socket 操作由一个网络线程处理，会话调度使用另一个线程。
状态锁释放后才回调上层，会话锁不跨越网络操作或 join。
shutdown() 先禁止新调度，再关闭传输并回收两个线程，可重复调用。
调用者须保证对象销毁前不再有外部线程访问对象。

协议依据：[WebSocket 模式](https://developers.openai.com/api/docs/guides/websocket-mode)、
[服务端压缩](https://developers.openai.com/api/docs/guides/compaction)、
[流式完成事件](https://developers.openai.com/api/reference/resources/responses/streaming-events)。

## 构建与测试

在开发机通过 `ssh jetson-orin` 执行：

```bash
source /opt/ros/humble/setup.bash
cd /home/yan/ros2_ws
MAKEFLAGS=-j2 colcon build --packages-select vln_agent --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select vln_agent --event-handlers console_direct+
colcon test-result --test-result-base build/vln_agent --verbose
```

测试覆盖定义筛选、相对路径、配置注入、资源共享与隔离、装载事务、
FIFO 阻塞语义、并发及等待容量、独立 future、去重、错误包装和关闭。
ROS 测试使用独立 domain、回环假导航服务与合成图像，验证导航反馈、
拒绝、取消到终态、历史隔离、JPEG 解码与相机关闭。
会话测试使用进程内回环 WS/WSS 假服务，覆盖缓冲、限频、上下文续接、失败重试、
网络重连、迟到响应隔离、流式显示、分帧、TLS 校验、保活与关闭。
测试不连接模型，不启动底盘，不发布运动命令。
测试临时配置保留在 build/vln_agent/test-fixtures，ROS 日志位于 build/vln_agent/ros-logs；
测试插件仅在 BUILD_TESTING 开启时构建和注册，不在默认工具列表中。

独立下游链接验证（先执行上面的 source）：

```bash
cmake -S src/scout_mini_vln/vln_agent/test/install_consumer \
  -B build/vln_agent/install_consumer -DCMAKE_BUILD_TYPE=Release
cmake --build build/vln_agent/install_consumer -j2
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR="$PWD/build/vln_agent/ros-logs" \
  build/vln_agent/install_consumer/install_consumer \
  install/vln_agent/share/vln_agent/config/agent.yaml
build/vln_agent/install_consumer/session_consumer \
  install/vln_agent/share/vln_agent/config/agent.yaml
```

session_consumer 只验证会话库的构造、状态访问和关闭，不提交模型请求；
验证记录采用仅含会话段、指向回环关闭端口的配置。

## 验证记录（2026-10-22）

- Jetson / ROS 2 Humble / GCC 11.4.0 的 Release 构建通过。
- 43 项 GTest 全部通过：12 项工具及资源、7 项 ROS 插件、4 项辅助函数、
  11 项会话管理、9 项 WS/WSS 传输测试。
  colcon 加上 5 个 CTest 包装项共报告 48 项，错误、失败、跳过均为零。
- 初始化失败后，导航和相机回调线程均正常回收，管理器未暴露部分定义。
- 独立 CMake 下游工程仅链接 vln_agent::vln_agent_tools，编译运行通过，
  成功从安装目录发现并装载 4 个工具；未执行导航或相机函数。
- 独立会话下游目标只链接 vln_agent::vln_agent_session，编译运行通过；
  动态依赖中没有 ROS 库，不提交模型请求。
- 模型无限等待时 shutdown 回收测试通过，关闭耗时低于 500 ms；
  关闭前取消 WS 保活定时器，避免等待空闲定时器自然到期。
