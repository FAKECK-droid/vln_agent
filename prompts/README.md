# VLN 提示词套件

这套提示词面向当前 `vln_agent_node`：使用现有十个工具，把自然语言目标转成观察、定位、导航、监测和到达验证的闭环。运行时只加载一个完整文件，不需要模板变量、文件拼接或额外 Agent。

## 文件与加载方式

| 文件 | 用途 | 是否发给模型 |
| --- | --- | --- |
| [vln_system.md](vln_system.md) | 完整系统提示词：身份、输入协议、任务状态、工具规则、事件、搜索、恢复与输出 | 是，全文作为 system 消息 |
| [simple_system.md](simple_system.md) | 已有简版，保留供比较与回退 | 与完整版二选一 |
| [REVIEW_SCENARIOS.md](REVIEW_SCENARIOS.md) | 合成输入场景与预期行为，供离线评审和后续回放 | 否，按用例提供必要输入 |
| 本文件 | 接入说明、源码依据与开源参考 | 否 |

`VLNAgent::init()` 读取 `system_prompt_path` 指定文件的全文，在首次请求中加入 `role=system`；后续由 SessionManager 续接上下文。文件内所有运行规则已展开，模型不需要也不能自动读取本文链接。不要把简版和完整版同时加载，否则旧版规则会产生冲突。

在已准备好 ROS、配置和运行条件的设备上，使用源码文件路径即可；以下仅为接入示例，本次没有执行：

```bash
ros2 run vln_agent vln_agent_node --ros-args \
  -p global_yaml_path:=/path/to/vln_agent/config/agent.local.yaml \
  -p system_prompt_path:=/path/to/vln_agent/prompts/vln_system.md
```

本包 CMake 安装 `prompts/`。下次正常构建并加载工作空间环境后，也可使用包安装路径：

```bash
ros2 run vln_agent vln_agent_node --ros-args \
  -p global_yaml_path:=/path/to/vln_agent/config/agent.local.yaml \
  -p system_prompt_path:="$(ros2 pkg prefix --share vln_agent)/prompts/vln_system.md"
```

提示词不会自动生效；必须显式选择该路径并新建节点会话。当前程序只在初始化时读文件，不热重载已运行会话。若设置 `initial_instruction`，模型直接处理具体任务；否则仅报告就绪。`global_yaml_path` 使用部署环境自己的配置，提示词不包含端点、密钥或机器地址。

## 结构与执行策略

系统提示词按十章组织：

1. 身份、职责与能力边界。
2. 指令来源与证据规则。
3. 输入、图片、时间和坐标。
4. 任务理解与会话状态。
5. 决策闭环与调用顺序。
6. 十个工具的参数语义、资源生命周期和使用条件。
7. 事件驱动执行、任务关联和导航中视觉监测。
8. 目标搜索、路线跟随和到达验证。
9. 错误恢复、重试与阻塞。
10. 用户表达和原生函数输出。

任务采用单个高层 Agent 的滚动规划：保留总目标及经过的地标，一次启动一个物理动作，通过图像和事件更新下一步。规划可以有多个子目标，但不能一次提交多个相互依赖的导航。

默认持续追求原目标；“同一失败策略两次无新信息后换策略”是避免循环的软策略，不是总目标重试上限。普通物体首个观察位的约 0.8–1.2 米水平距离是可调整候选值，不是所有目标的成功判据，也不是底盘安全阈值。人员、空间限制、明确用户距离要求和真实传感器证据优先。

## 与现有实现的对应

核对日期：2026-11-12。Agent 源码基线：`97c2ed723670d631750acc066b151f22a9442a5a`。该提交是新增提示词之前已与 `origin/master` 对齐的本地基线；开发板离线，不能据此声称板端运行版本相同。

| 约定 | 已核对的本地源码/文档 | 提示词处理 |
| --- | --- | --- |
| 单文件提示词与首次 system 输入 | [src/vln_agent.cpp](../src/vln_agent.cpp)、[AGENT_PLAN §5](../docs/AGENT_PLAN.md) | 一个自包含 MD，原生 function_call，不要求业务文本 JSON |
| tools 实际列表与参数 | [navigation/tool.json](../plugins/tools/navigation/tool.json)、[rgbd/tool.json](../plugins/tools/rgbd/tool.json)、[capture_image/tool.json](../plugins/tools/capture_image/tool.json) | 十个现有函数；不假定规划中的工具已接入 |
| 调用结果与物理任务分别完成 | [AGENT_PLAN §2–4](../docs/AGENT_PLAN.md)、[navigate_to_pose_start_navigation.cpp](../plugins/tools/navigation/navigate_to_pose_start_navigation.cpp) | 区分 call_id/task_id、ok/status 和导航事件 |
| 导航/环顾共用一个任务槽 | [navigation_runtime.cpp](../plugins/tools/navigation/navigation_runtime.cpp)、[start_lookaround.cpp](../plugins/tools/navigation/start_lookaround.cpp) | 禁止并行运动，NAVIGATION_BUSY 不当作路径失败 |
| 完成/失败后释放活动记录 | [navigate_to_pose_start_navigation.cpp](../plugins/tools/navigation/navigate_to_pose_start_navigation.cpp)、[navigate_to_pose_get_feedback.cpp](../plugins/tools/navigation/navigate_to_pose_get_feedback.cpp) | 结束后直接推进替代目标；TASK_NOT_FOUND 不代表成功 |
| 像素图尺寸、方向和 ID 消费 | [get_surroundings.cpp](../plugins/tools/navigation/get_surroundings.cpp)、[utils.cpp](../plugins/tools/navigation/utils.cpp)、[navigate_to_pixel_start_navigation.cpp](../plugins/tools/navigation/navigate_to_pixel_start_navigation.cpp) | 完整图坐标、map_rect 内选点、截图相对 yaw；目标接受后重新取环境 ID |
| 缩放彩色图与原深度映射 | [rgbd/tool.json](../plugins/tools/rgbd/tool.json)、[get_depth_point.cpp](../plugins/tools/rgbd/get_depth_point.cpp)、[capture_pair.cpp](../plugins/tools/rgbd/capture_pair.cpp) | 使用返回 width/height 下的整数像素，工具负责缩放换算 |
| 采样时固定系与当前目标系 | [get_depth_point.cpp](../plugins/tools/rgbd/get_depth_point.cpp) | map 点对应历史采样；其他 target_frame 使用当前变换；动态物体重新观察 |
| 新输入推动请求，不具备定时器 | [VLNAgent::step](../src/vln_agent.cpp)、[session_manager.cpp](../src/session/session_manager.cpp) | 一次图像返回再判断下一次，不承诺采样 Hz 或伪造 sleep |
| 图片描述两种组织方式 | [AGENT_PLAN §2](../docs/AGENT_PLAN.md)；相邻 `vln_behavior_tree/docs/BEHAVIOR_TREE_PLAN.md` §3、§6 | 同时识别 metadata 嵌套与事件平铺字段 |
| BT 事件通常没有 task_id，环顾会触发子导航事件 | 相邻 `vln_behavior_tree` 的 `plugins/events/navigation_event.cpp`、`behavior_trees/lookaround.xml` 和接口文档 §8–9 | 结合动作类型、目标、origin、时序关联；环顾以 lookaround_completed 为图像完成信号 |

相邻功能包是独立仓库；上述行为树证据来自本地文件，本文不把它们描述为已同步至开发板。工作空间级 `VLN_RESPONSES_MODEL_INTERACTION_PLAN.md` 是旧的 BT 模型节点设计，标记“规划，尚未实现”，不能把其中 `kind/data` 文本输出或 InitModel 当成当前 Agent 接口。空间语义记忆设计可以指导证据和 map 坐标约束，但当前 tools 没有记忆查询/写入工具，不默认调用。

### 对简版意图的整理

保留并明确：空任务仅就绪；依赖调用等待前一步；导航完成主要靠事件；正常导航不持续调用反馈；最终失败后无需再取消旧任务；持续推进目标；导航中串行获取视觉反馈；深度边缘无效时换点或依据通路换视角。

两条需要约束的旧表述：

- “Nav2 能充分保障安全，因此目标可以设在致命障碍内部”改为允许 Nav2 判断目标可达性、避免把膨胀区全当禁区，同时把自主生成的停车位放在有通路证据的位置。模型提示词不能替代真实避障、底盘失效保护或急停。
- “所有目标不许放弃”落实为保留总目标、切换失败策略；用户取消、能力缺失、传感器/定位持续故障时如实等待恢复，避免无限无效调用。

这两项是本套提示词的行为策略选择，不是声称底层新增了安全校验。原简版保留不改。

## 开源参考与采用方式

以下均查看作者公开实现中的实际提示词，核查日期为 2026-11-12。这里重新编写中文规则，借鉴组织方法；不复制上游长段提示词，也不宣称引入模型、训练权重或实现了其导航算法。

| 来源 | 查看位置与文件 Git blob SHA | 采用的方法 | 在本项目中的适配 |
| --- | --- | --- | --- |
| [NavGPT](https://github.com/GengzeZhou/NavGPT) | [planner_prompt.py](https://github.com/GengzeZhou/NavGPT/blob/master/nav_src/prompt/planner_prompt.py)，`4ba0435817cc917d9657d5f522eb80c075be0965` | 子任务分解、历史对齐、必要回溯、只选有证据的动作 | 使用真实工具 ID 和连续 map 目标；保留证据摘要，不套用离散 viewpoint ID、Thought 模板或仿真停止半径 |
| [MapGPT](https://github.com/chen-judge/MapGPT) | [one_stage_prompt_manager.py](https://github.com/chen-judge/MapGPT/blob/main/GPT/one_stage_prompt_manager.py)，`f9c8e7164a39029d32f2459da351c2726e30be5b` | 已访问位置、连接关系、指令进度与滚动计划共同决策，减少无收益循环 | 用观测与经过记录支持关系；代价图辅助选航点，不凭语义想象拓扑连接，也不输出动作字母 |
| [InstructNav](https://github.com/LYX0501/InstructNav) | [nav_prompt.py](https://github.com/LYX0501/InstructNav/blob/main/llm_utils/nav_prompt.py)，`51dbd9c38076b481decf688c31086e9a1b893f35` | 动作与地标成对、每次推进一个子目标、综合多方向视觉线索 | 把动作链转换为观察/测量/导航工具；依据本项目拼图元数据识别方向，不照搬固定视角编号或“发现目标即完成” |

链接指向上游工作分支，可能继续变化；Git blob SHA 用于标明本次核对的文件内容，不是整个框架版本或导航性能承诺。

## 验证与本次交付范围

本次只在开发机新增/更新提示词、说明及安装资源规则。保留已有 `config/agent.yaml` 修改和简版提示词，不连接离线开发板，不运行真实导航，不调用付费模型进行任务测试。

静态核对内容：工具名和参数与本地 JSON Schema 对应；示例参数可解析且符合类型/枚举/必填字段；文档相对路径可解析；关键像素、ID 和事件语义与源码一致。评审场景是合成材料，不是实机运行记录或模型实测通过声明。

板端恢复后仍需按项目约定经 `ssh jetson-orin` 核对工作区与对应版本，再用 Git 拉取本次提交。按设备现有构建方式安装提示词资源，并在合适的隔离条件下做录制数据回放或受控验证。当前提示词不增加任务 UUID 事件关联、周期监测调度、硬件看门狗或独立急停；这些边界需由运行系统提供，不能依赖文字承诺。
