# vln_agent

ROS 2 Humble 功能包，实现 docs/AGENT_PLAN.md 第二至五部分的工具层、会话层、用户交互层和机器事件层。
各层通过原生 JSON 由宿主衔接，不依赖 vln_behavior_tree，不创建 Agent 主循环。

## 组成

- `ToolResult`：独立的业务结果数据类型，定义于 `tools/tool_result.hpp`。
- `ToolPluginBase`：固定的 initialize / execute / shutdown 插件接口。
- `SharedResources`：每个管理器独立的具名、带类型资源注册表。
- `ToolManager`：装载定义和 pluginlib 插件、FIFO 调度、活动调用去重及逐调用 future。
- `plugins/tools/navigate_to_pose`：启动、查询、取消三个插件共用一个 NavigationRuntime。
- `plugins/tools/capture_image`：订阅最新原始图像，返回 JPEG 图片与采样信息。
- `SessionManager`：独立上下文链、两份输入缓冲、发送限频与单个完整结果槽。
- `ResponsesWsTransport`：独立 WS/WSS 连接、认证、保活、网络超时和重连。
- `src/utils/`：YAML/JSON 转换、输入数组操作、工具结果包装、Base64 编解码与 WS URL 解析。

共享资源继承 `SharedResource` 并实现幂等 `shutdown() noexcept`。
资源工厂立即执行，不保存；同名资源类型不一致时报配置错误。
工具插件不访问 SessionManager、UserInteractionManager 或 EventManager。

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
`ToolManager` 和 `ToolContext` 的公共头只前置声明 ROS Node；创建节点或调用 ROS API
的宿主/插件须自行包含 `<rclcpp/rclcpp.hpp>`。仅使用 `ToolResult` 不需要 ROS 头文件。
`submit()` 接受 `status: completed` 的原始完整响应，按 output 顺序处理
function_call，忽略 message、reasoning、compaction。
没有函数调用时返回空数组。普通响应结构错误抛异常；
未知工具、参数解析失败、队列满和执行异常分别完成对应调用的失败结果。

同一活动 call_id 返回原句柄；函数完成后立即从 calls 移除，不保留完成结果或历史 ID。
结果由调用方持有的 future 交付。Agent 负责保存历史、避免重复提交已完成调用，并在消费后释放自身句柄。
pending 和 running 分别受配置容量限制，calls 只包含这两类活动调用。
工具返回 `ToolResult{payload, images}`，payload 至少包含布尔值 ok 和 UTC 毫秒 time_stamp。
管理器补齐结果缺少的 time_stamp 并生成 API 外层，不添加 source 或 pose。

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
固定数量的工作线程在同一把锁下直接按 FIFO 取任务，实际进入插件及完成的顺序可不同。
工具层没有独立调度线程或中转队列；插件 execute 在队列锁之外运行。
一个阻塞项不会等待此前所有非阻塞项结束；它自身完成即允许后续项调度。

## 工具状态查询

三个查询函数均返回 JSON 副本，不消费 future。客户端可定期读取摘要，需要详情时再读取队列或单次调用。

| 方法 | 返回内容 |
| --- | --- |
| `getStatus()` | `state`（unloaded/ready/closing/closed）、`max_concurrent_calls`、`max_pending_calls`、`waiting_count`、`running_count`、`completed_count`、`call_count`、`blocking` |
| `getQueueSnapshot()` | 同一时刻的 `waiting` 和 `running` 数组，各按提交序号排列；不含已完成调用 |
| `getCallStatus(call_id)` | 等待/运行中调用的元数据；已完成或未知 ID 返回 null |

调用元数据包括 `call_id`、`name`、从零递增的 `sequence`、`blocking`、
`state`（waiting/running）、`wait_ms` 和 `run_ms`。
等待时间从登记开始计算，执行开始后冻结；运行时间从取出任务计至查询时刻。
完成后元数据即被删除，结果仍通过调用方句柄读取。
call_count 是当前活动数量，completed_count 仅为固定大小的累计计数，不包含历史记录。
结果查询不复制参数或图片，不把导航的物理任务状态当成函数执行状态。

每次查询内部一致；分别调用摘要和队列接口期间可能发生任务切换。
队列快照只扫描等待和运行项，不遍历历史结果。

## 导航与相机行为

导航只允许一个活动任务，再次启动返回 NAVIGATION_BUSY。
启动只等服务端接受或拒绝；后续运动由 ROS Action 维持，不占工作线程槽。
本地 task_id 与 API call_id 分开；查询和取消须使用启动返回的 task_id。
查询成功时 ok 为 true，即使物理任务 status 为 failed。
取消接受后为 canceling，只有最终 Action 结果才置为 canceled。
导航资源只保留一个当前任务或尚未交付的终态。
终态首次经 get_feedback、cancel_navigation 或启动调用返回后即释放，之后查询返回 TASK_NOT_FOUND。
未消费终态时启动新目标返回 NAVIGATION_RESULT_PENDING，避免覆盖待交付结果。
历史导航记录由 Agent 保存；已释放任务的迟到回调不会修改新任务。

相机无函数参数，首帧到达后使用最近收到的帧，不保证每次调用采到新帧。
结果 payload 含 frame_id、time_stamp、width、height；真实时钟下另带 image_time_stamp，图片以原生
input_image 的 data:image/jpeg;base64 URL 返回，不把图片放入 payload。

shutdown 停止入队，立即完成未执行调用，通知插件和共享资源退出，
回收执行线程后释放插件及加载器，不 detach 线程。
内置插件为独占回调组持有可回收的 ROS 执行线程，在卸载前结束相关回调。
导航关闭时尽力请求取消本客户端目标，不把取消请求当作物理停止确认；
服务端失联或目标回复超时时会记录警告。真实导航必须由外部导航栈和急停机制保证安全。

## 上下文保存方式

在全局 YAML 的 session_manager.context 中添加可选开关：

```yaml
session_manager:
  context:
    local_history: true       # 默认 false：使用原有 previous_response_id 续接
    server_compaction: false  # 默认 true；模型不支持服务端压缩时关闭
    budget_tokens: 128000
    compact_ratio: 0.8
```

- local_history=false 保持原有行为：只发送新增 input，并引用最近完成的 response ID。
- local_history=true 在 SessionManager 内存中保留已成功轮次的输入及完整 output，
  每轮发送有效上下文加新增 input，不传 previous_response_id，并强制 store=false。
  message、function_call、工具结果、图片以及服务端返回的加密推理项均按原生格式保留；
  Agent、工具和用户接口的调用方式保持一致。
- 完整响应成功后才提交上下文，失败或断线的部分输出不加入；显式重试只恢复本轮新增输入，
  不重复拼入已经提交的历史。pending 始终只包含尚未发送的新输入。
- server_compaction=true 时沿用 context_management 压缩请求。本地模式收到 compaction 项后，
  保留最后一个压缩项及其后续内容，释放被其替代的前缀；不在本机生成摘要或解密。
  关闭此项后不发送 context_management，也不会自动裁剪本地上下文。
- 本地保存仅持续到当前 SessionManager 关闭，不落盘、不跨进程恢复。
  budget_tokens 仍用于计算服务端压缩阈值，不是本地 token 硬上限。
- getContextStatus() 增加 mode（server/local）和 history_item_count，
  其他已有字段不变。只有启用本地模式才保存继续请求所需的有效上下文。

实现依据：[无存储推理上下文](https://developers.openai.com/api/docs/guides/reasoning#preserve-reasoning-without-stored-responses)、
[服务端压缩与本地输入链](https://developers.openai.com/api/docs/guides/compaction)。

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
发送时才读取工具列表；默认模式使用最新完成的 previous_response_id，
本地模式则把已提交有效上下文拼入 input。

takeResponse() 返回原始 event.response，尚无结果时返回 null；
取走完整结果后清空其流式正文与 ID，由 Agent 将这份结果作为新增事件交给界面。
reasoning、compaction、usage 和扩展字段保留原样，流式函数参数不作为工具调用输出。
stream_output 仅控制 getStreamSnapshot() 的本轮显示快照，关闭时返回 null；
快照按 output_index/content_index 区分文本，不消费完整结果。
getContextStatus() 报告最近原始 usage 和已观察到的压缩次数，不估算实时窗口占用。

只读取全局 YAML 的 session_manager 段，示例见 config/agent.yaml。
先填写实际 base_url/api_key；127.0.0.1 指运行程序所在设备，Jetson 连接桌面服务时需改为可达地址。
max_request_hz 按相邻发送开始时刻限制频率，定时条件变量自行唤醒。
request_timeout_sec 为零时允许无限等待模型；网络阶段有独立超时和保活。
WS 请求采用 response.create，context_management.compact_threshold 为
floor(budget_tokens * compact_ratio)。压缩后，默认模式继续使用最新响应 ID，
本地模式重放最新压缩项及其后内容。
配置预算是应用策略，不代表模型的硬上限。

getRequestStatus() 返回 idle、waiting_interval、waiting_transport、generating、
response_ready、error 或 closed，并附带底层 transport 快照。
`current_response_id` 是本轮已收到的响应 ID（开始新一轮时清空，完成后保留）；
`retry_required` 表示当前请求未完成、需要显式处理，`can_retry` 表示可调用 request() 登记重试。
上下文失效时前者为 true、后者为 false。登记重试后，实际发送仍需等待连接、结果槽和限频。
`request_elapsed_ms` 从发送开始计时，完成、错误、中断或关闭后冻结；
`rate_limit_remaining_ms` 是距允许下次发送的剩余毫秒数，向上取整，最小为零。

`getTools()` 返回下次发送使用的工具定义，`getPendingInputs()` 返回待发输入数组，
`getCurrentRequest()` 返回在途或失败后保留的完整请求，没有时为 null。
三者均为不消费内容的独立副本，完整输入可能包含图片，适合按需查看。
发送线程以移动方式交接发送帧，仍保留 current_request 的独立快照供查询和重试。

模型 error/failed/incomplete 和模型超时记录到 last_error；保留当前与待发输入并清除发送意图。
再次 request() 才把未完成的新输入放回待发数组前部，按当前模式重试。
默认模式遇到 previous_response_not_found 保持上下文错误，宿主须决定如何重建会话。
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

## 本次复盘决策（2026-10-22）

- 已确认实施：工具与会话查询、工作线程直接取 FIFO、删除传输层重复的 interrupted 标志、
  拆分 ToolResult 和整理头文件依赖、移动发送帧与只读引用。
- 按选择保留逐项解析并入队的 submit()。后续调用缺少 call_id/name 等结构错误仍可能
  在前半批已登记后抛异常，且没有整批回滚；调用方应提交结构完整的响应。
- 按选择保留统一报错：开启流式显示时，显示解析异常仍会让当前请求进入 INVALID_RESPONSE，
  不单独隔离为显示错误。
- 配置与输入类型检查、重复 ID 规则、关闭/连接代号检查、共享资源类型检查及 TLS 验证保留。
  它们分别约束外部输入、并发生命周期、类型转换和网络连接，不属于重复状态判断。

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
FIFO 阻塞语义、并发及等待容量、独立 future、去重、错误包装和关闭，
以及工具队列快照、完成状态、等待/运行耗时和查询副本隔离。
ROS 测试使用独立 domain、回环假导航服务与合成图像，验证导航反馈、
拒绝、取消到终态、终态一次消费、JPEG 解码与相机关闭。
会话测试使用进程内回环 WS/WSS 假服务，覆盖缓冲、限频、上下文续接、失败重试、
网络重连、迟到响应隔离、流式显示、分帧、TLS 校验、保活与关闭，
以及会话内容查询、请求耗时冻结、限频剩余时间和重试标志。
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
  src/scout_mini_vln/vln_agent/test/install_consumer/offline.yaml
ROS_DOMAIN_ID=173 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR="$PWD/build/vln_agent/ros-logs" \
  build/vln_agent/install_consumer/interaction_consumer \
  src/scout_mini_vln/vln_agent/test/install_consumer/offline.yaml
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
- 本次复盘新增的队列/调用查询、内容副本、重试标志和耗时断言均通过；
  `submit()` 的逐项入队和流式解析统一报错行为按选择保留。
- 安装后的 `tool_result.hpp`、`tool_context.hpp`、`tool_plugin_base.hpp`、
  `tool_manager.hpp` 在不添加 ROS include 路径的条件下单独编译通过。
  使用节点/执行器的宿主、插件和 ROS 测试均显式包含所需 ROS 头。
- 本次日志位于 `build/vln_agent/review-build.log`、`review-tests.log`、
  `review-test-result.log` 和 `review-install-check.log`。

## 用户交互层

链接 vln_agent::vln_agent_interaction，包含 interaction/user_interaction_manager.hpp。
节点和被查询组件由宿主持有 shared_ptr，管理器通过 weak_ptr 查询。
先装载 ToolManager，再装载 UserInteractionManager：

```cpp
auto tools = std::make_shared<vln_agent::ToolManager>(node);
tools->load(config_path);
auto session = std::make_shared<vln_agent::SessionManager>(config_path);
session->setTools(tools->getTools());
vln_agent::UserInteractionManager interaction(node, {session, tools, {}});
interaction.load(config_path);
// 外部串行 spin_some()，本次只交付新增完成响应和新增就绪结果。
auto inputs = interaction.exchange(new_response, new_tool_outputs);
session->addInputs(inputs);
// 是否 request() 仍由宿主决定。
```

user_interaction.channels 每个实例独立配置 transmit 和 include_images，缺省关闭。
默认 RosTopicInterface 发布 /vln/user_output，订阅 /vln/user_input，
std_msgs/msg/String.data 承载 JSON。输入须已经是原生 API 对象或其数组；
业务 JSON 只有 source/time_stamp 两项公共必填字段，pose 可省略。
管理器不包装输入、不补字段，不读取 pending 输入，也不消费 response 或 future。

一轮按字段并集查询一次，工具定义只查询一次；每个渠道输出一个增量汇总 JSON。
工具放在 plugins/tools，用户接口放在 plugins/interfaces。
同配置插件共享 shared_ptr<const Json>，所有目标插件接收后才提交本批文字和状态差量；
不同配置只产生筛选视图。组件不保存完成历史或历史去重集合。
Agent 只传新增完成事件；exchange() 成功后不能在空轮询中反复提交旧 response/result。
接口正常返回即表示已接收输出。若接口抛异常，用相同参数再次 exchange()；
先前已经接收者只收 nullptr，已收到的用户输入保留到本轮成功返回。
管理器不启动线程；外部须串行安排接口回调、exchange() 和 shutdown()。

展示协议含 type/batch_id/time_stamp/full 及启用的 request/response/tools/session。
full=false 时，状态字段按键替换，缺省保持、null 清除；messages 按
response_id/output_index/content_index 使用 append/replace。
流式和最终文本共用当前响应的文本位置；完整结果交付后释放该位置。
tools.finished_calls 根据本次 tool_outputs 生成 {call_id, state: "finished"} 通知，
不查询已删除的工具历史，也不保留 name/wait_ms/run_ms 等终态记录。
request 去掉重复 tools；definitions 单独缓存。include_images=false 时原生图片项变为
omitted 标记。原始 API 对象不被修改，日期在展示副本中统一为 UTC 三位毫秒。

## 实际配置和真实 API 测试

仓库根目录就是本功能包，保留此前提交历史；其他工作空间文件不属于此仓库。
config/agent.yaml 是无密钥模板；实际配置保存在忽略的 config/agent.local.yaml。
复制模板填写后，宿主把该实际配置路径传给各组件。
源码经开发机 Git push、开发板 Git pull 更新；设备私有配置使用未跟踪的 agent.local.yaml，
密钥不写入 Git，日志也不输出认证头。

只有显式运行 live_interaction_test 才发真实 API 请求，不由默认 CTest 自动触发。
本次用户授权使用开发板的回环/局域网反代，模型固定 gpt-6-luna：
纯 CPU 测试工具、ROS 原生输入、三轮 WS 上下文、工具结果回传、
流式/最终文本一致性和函数调用去重均在同一 C++ 流程验证，不运行底盘或真实导航。

```bash
source /opt/ros/humble/setup.bash
source /home/yan/ros2_ws/install/setup.bash
cd /home/yan/ros2_ws
ROS_DOMAIN_ID=175 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR="$PWD/build/vln_agent/ros-logs" \
  build/vln_agent/live_interaction_test \
  src/scout_mini_vln/vln_agent/config/agent.yaml \
  build/vln_agent/test-fixtures/live/response.json
VLN_LIVE_FIXTURE="$PWD/build/vln_agent/test-fixtures/live/response.json" \
  colcon test --packages-select vln_agent --event-handlers console_direct+
colcon test-result --test-result-base build/vln_agent --verbose
```

真实请求只使用最新 previous_response_id 和新增输入；初始随机口令不会在后续输入重发。
测试保存不含密钥的真实响应 fixture，回归测试可回放该 fixture，不再访问 API。
live_runtime.local.yaml 含实际密钥，仅保存在构建目录，权限 0600。

## 验证记录（2026-10-24）

- 先在开发机完成实现，再经 ssh jetson-orin 完整同步；Jetson Humble Release 构建通过。
- 53 项 GTest 全部通过，包含 9 项用户交互测试和真实响应回放；
  colcon 加上 6 个 CTest 包装项报告 59 项，错误、失败、跳过均为零。
- 多插件相同指针共享、分插件字段/图片开关、失败后续交与输入保留、
  流式/最终文字去重、当前请求查询、工具状态及 ROS 原生数组往返均通过。
- 三个独立下游目标仅链接各自安装导出目标，编译、装载、交换和关闭通过。
- Jetson 经 192.168.3.25:8317 的桌面反代连接 gpt-6-luna，三轮真实 WS 交互通过：
  首轮系统初始化与随机口令，第二轮只提交工具结果，第三轮只询问原始口令。
  一次测试函数调用与一次结果各展示一次，最终显示文本与完整 response 一致。
- 原始请求/响应记录在 build/vln_agent/test-fixtures/live/turn_*.json，
  完整回放 fixture 为同目录的 response.json；均不包含认证头。
  本次短对话没有触发服务端压缩，cached_tokens 为零，没有验证缓存命中。
- 构建、回归、真实测试日志位于 build/vln_agent/interaction-*.log。

## 组件数据所有权（2026-10-24）

已结束的归档历史及历史去重由高层 Agent 管理。启用 local_history 时，
SessionManager 仅额外保存下一轮请求仍需使用的有效上下文。

- ToolManager 仅持有等待/运行调用；完成即释放内部 Call、参数和结果引用。
  外部 future 在管理器销毁后仍可读取，最后一个调用方句柄释放后回收其结果。
- UserInteractionManager 仅保留当前展示状态、当前流式文本及一个待交付差量批次。
  不复制整份历史基线，不维护 responses/results/watched/finished 历史集合。
- SessionManager 保留待发输入、在途或可重试请求、尚未取走的完整结果；
  takeResponse() 清除对应流式正文。本地模式还保留下一轮仍需重放的有效上下文，
  压缩替代的前缀会释放。必要链头 ID、最近用量和固定大小统计属于当前会话状态。
- NavigationRuntime 只保留一个活动/待消费任务，终态交付后删除。
- 相机的最新帧、静态工具定义、当前错误及尚未交付的数据仍有用途，不属于已结束历史。
- getCurrentRequest() 的值复制接口保持原样，本轮不实施请求共享快照优化。

开发机 Humble Release 构建及本地回归通过；本地缺少已有真实响应 fixture，
53 项 GTest 通过，1 项回放用例跳过。目标设备验证在推送及拉取后执行。

## 机器事件层（2026-10-28）

独立链接 `vln_agent::vln_agent_events`，包含
`vln_agent/events/event_manager.hpp`。该库仅复用通用 JSON/YAML 和 ROS，
不依赖或调用 SessionManager、ToolManager、UserInteractionManager。
主 Agent 在 init() 中装载 EventManager，定频主循环把 takeEvents() 返回的整批事件原样交给
SessionManager；模型正在响应时事件进入待发输入，关闭 Agent 时释放事件订阅。

```yaml
event_manager:
  receive_topic: /vln/robot_event
```

`load(config_path)` 只读取 event_manager 段；缺省使用 /vln/robot_event。
一个 ROS 订阅接收 std_msgs/msg/String，data 为原生 Event 对象或对象数组的 JSON 文本，
采用 reliable、volatile、KeepLast(10) QoS，由宿主执行器处理回调，不启动内部线程。

```cpp
#include <vln_agent/events/event_manager.hpp>

vln_agent::EventManager events(node);  // 宿主持有 node。
events.load(config_path);
// 同进程可以直接上报；event 已是 role/content 原生消息。
events.addEvents(event);              // 也可传 Json 数组。
auto inputs = events.takeEvents();    // 一次取走全部，空时返回 []。
// inputs 由宿主交给会话；EventManager 不持有会话或工具组件。
events.shutdown();
```

每个 Event 是独立的 input[] 元素；content 第一项为 input_text，
text 保存 source=robot、time_stamp、origin 及可选 text/data/instruction 的业务 JSON，
后续可带原生 input_image。来源和 UTC 三位毫秒时间由发送者填写，
组件保留全部内容，不补字段、不解码图片、不解释 instruction.function 或设置 tool_choice。
命令是否生成工具调用由模型决定；详见 docs/AGENT_PLAN.md 第五部分。

addEvents() 和 takeEvents() 使用同一把缓冲锁，整批保持连续顺序；
takeEvents() 移出后内部立即清空，不保存完成历史或去重 ID。
getStatus() 只返回 state（unloaded/ready/closed）和 pending_count。
关闭后忽略新增事件，释放订阅和未交付数据，不关闭宿主节点。
ROS 输入无法解析或不符合对象/数组形式时记录警告并跳过，不影响后续事件。

开发机离线验证：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select vln_agent --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select vln_agent --ctest-args -R '^test_event_manager$' --output-on-failure
```

测试只使用本机 ROS 节点和合成 JSON，覆盖批量交付、文字/图片/命令原样保留、
并发追加与消费、单话题接收、坏 JSON 后继续接收、默认话题及关闭释放；
不访问模型服务或真实机器人。

验证记录（2026-10-28）：开发机 ROS 2 Humble Release 编译通过；新增 4 项 EventManager 测试全部通过。
完整离线回归 58 项 GTest 中 57 项通过，1 项已有真实响应回放因未提供 fixture 跳过，无失败。
安装后的独立下游工程仅链接 vln_agent::vln_agent_events，编译运行通过；动态依赖中没有
vln_agent_session、vln_agent_tools 或 vln_agent_interaction。未进行开发板或真实模型测试。
