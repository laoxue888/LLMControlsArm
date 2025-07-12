

## GraphExecuter

</div>

<div align="center">

[中文](./docs/readme_zh.md) | English

</div>

GraphExecuter is an open-source software developed based on NodeGraphQt, adopting a multi-threaded graph execution architecture. Users only need to write node logic and construct graphs by connecting nodes, and the system will automatically schedule and execute complex workflows.

This system combines visual node editing with efficient parallel computing, making it suitable for scenarios such as data processing, automated tasks, and distributed computing. Developers do not need to worry about thread management—they can focus solely on implementing node functionalities to quickly build high-performance workflows.

GraphExecuter provides a convenient solution for users who require flexible, scalable, and visual orchestration, catering to diverse needs from experimentation to production.


code: [https://github.com/laoxue888/GraphExecuter](https://github.com/laoxue888/GraphExecuter)

video:

- [2025-7-5: 【开源】测试运行yolov13实时检测算法](https://www.bilibili.com/video/BV1eU3XzSEum/?vd_source=3bf4271e80f39cfee030114782480463)
- [2025-4-17: 开源DeepSeek人工智能语音对话工作流，你的AI智能语音助手啦](https://www.bilibili.com/video/BV1e15qz7ESi/?vd_source=3bf4271e80f39cfee030114782480463)
- [2025-4-2: 高效创建工作流，可实现类似unreal engine的蓝图效果，内部使用多线程高效执行节点函数](https://www.bilibili.com/video/BV1PkfKY1Esk/?vd_source=3bf4271e80f39cfee030114782480463)


## Quick Start

❇️ Install

```shell
conda create -n graph_executer python=3.12
conda activate graph_executer

cd graph_executer
pip install -r .\requirements.txt

pip install torch==2.4.1 torchvision==0.19.1 torchaudio==2.4.1 --index-url https://download.pytorch.org/whl/cu124

git clone https://github.com/laoxue888/NodeGraphQt.git
cd NodeGraphQt
pip install -e .
```

❇️Run

```shell
python main.py
```

## Related Projects

> - [NodeGraphQt](https://github.com/jchanvfx/NodeGraphQt)
> - [NodeGraphQt Documentation](https://chantonic.com/NodeGraphQt/api/index.html)
