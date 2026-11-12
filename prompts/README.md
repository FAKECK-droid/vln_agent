# VLN 提示词

推荐使用 [vln_system.md](vln_system.md)：精简的导航系统提示词，保留任务原则、执行反馈和工具调用技巧。工具参数与结果格式由实际工具定义提供。

| 文件 | 用途 |
| --- | --- |
| [vln_system.md](vln_system.md) | 推荐的精简系统提示词，全文作为 system 消息 |
| [simple_system.md](simple_system.md) | 已有简版，可单独选择 |
| [REVIEW_SCENARIOS.md](REVIEW_SCENARIOS.md) | 离线场景参考，不加载到模型 |

在节点启动参数中指定所选文件，例如：

```bash
-p system_prompt_path:=/path/to/vln_agent/prompts/vln_system.md
```

一次选择一个提示词文件。`vln_agent_node` 在初始化时读取文件；更新后需新建节点会话才能加载新内容，运行中的会话不会热重载。

CMake 会将 `prompts/` 安装到包的 share 目录。使用安装路径时，在正常构建后加载工作空间环境，再选择对应文件。
