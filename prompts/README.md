# VLN 提示词

以 [simple_system.md](simple_system.md) 为主要系统提示词。两版都在每个新导航任务的首段出发前环视一次。复杂通路按代价图一次提交多个航点，开阔场景按同一 RGB-D 图的多个实例导航。行进中用 `capture_pair` 更新语义进度并准备下一段，只有其他观察方式无法判别关键方向时才再次环视。工具参数与结果格式以实际定义为准。

| 文件 | 用途 |
| --- | --- |
| [simple_system.md](simple_system.md) | 主要使用的精简提示词，全文作为 system 消息 |
| [vln_system.md](vln_system.md) | 可直接加载的完整版，包含工具选择、进度跟踪、限制与恢复细则 |
| [REVIEW_SCENARIOS.md](REVIEW_SCENARIOS.md) | 离线场景参考，不加载到模型 |

在节点启动参数中指定：

```bash
-p system_prompt_path:=/path/to/vln_agent/prompts/simple_system.md
```

一次选择一个提示词文件。`vln_agent_node` 在初始化时读取文件；更新后需新建节点会话才能加载新内容，运行中的会话不会热重载。

CMake 会将 `prompts/` 安装到包的 share 目录。使用安装路径时，在正常构建后加载工作空间环境，再选择对应文件。
