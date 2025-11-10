# MCP Demo Project

一个基于 Model Context Protocol (MCP) 的文件操作和查询演示项目。

## 项目概述

本项目演示了如何使用 MCP (Model Context Protocol) 构建文件操作工具，并通过 LangGraph 和 LangChain 创建智能代理来使用这些工具。项目包含两个独立的 MCP 服务器和一个测试客户端。

## 项目结构

```
mcp_demo/
├── mcp_objects/
│   ├── file_mcp.py          # 基础文件操作 MCP 服务器
│   └── file_query_mcp.py    # 高级文件查询 MCP 服务器
├── test_mcp.py              # 测试客户端
├── requirements.txt         # Python 依赖
└── README.md               # 项目文档
```

## 功能特性

### File MCP 服务器 (`file_mcp.py`)
- **create_file**: 创建文件并写入内容
- **read_file**: 读取文件内容
- **append_to_file**: 向文件追加内容
- **list_files**: 列出目录中的文件和文件夹
- **file_info**: 获取文件详细信息（大小、创建时间、修改时间等）

### File Query MCP 服务器 (`file_query_mcp.py`)
- **search_files_by_name**: 根据文件名模式搜索文件
- **search_files_by_content**: 根据文件内容搜索文件
- **get_directory_tree**: 获取目录树结构
- **find_large_files**: 查找大文件
- **get_file_system_info**: 获取文件系统信息

## 快速开始

### 环境要求

- Python 3.8+
- pip (Python 包管理器)

### 安装依赖

```bash
python3 -m venv ~/ros2_env 
# 如果有问题，rf -rf ~/ros2_env 
source ~/ros2_env/bin/activate

pip install -r requirements.txt
```

### 运行测试

```bash
python test_mcp.py
```

运行后，程序会启动一个交互式命令行界面，你可以输入自然语言指令来操作文件系统。

## 使用示例

### 基础文件操作
```
用户输入: 创建一个名为 test.txt 的文件，内容为 "Hello World"
用户输入: 读取 test.txt 文件的内容
用户输入: 向 test.txt 文件追加 " - 追加内容"
```

### 高级文件查询
```
用户输入: 搜索当前目录下所有 .py 文件
用户输入: 在所有文件中搜索包含 "MCP" 的内容
用户输入: 显示当前目录的树状结构
用户输入: 查找大于 1MB 的文件
```

## MCP 服务器详情

### File MCP 服务器
- **传输方式**: stdio
- **工具数量**: 5 个
- **功能**: 基础文件创建、读取、追加、列表和信息查询

### File Query MCP 服务器
- **传输方式**: stdio
- **工具数量**: 5 个
- **功能**: 高级文件搜索、目录树展示、大文件查找、系统信息

## 技术栈

- **FastMCP**: MCP 服务器框架
- **LangGraph**: 代理工作流管理
- **LangChain**: LLM 应用框架
- **LangChain MCP Adapters**: MCP 客户端适配器
- **OpenAI API**: 大语言模型接口（支持 DeepSeek 等兼容 API）

## 依赖包

- `pymysql` - MySQL 数据库连接（未来扩展）
- `fastmcp` - MCP 服务器框架
- `langchain-mcp-adapters` - LangChain MCP 适配器
- `langgraph` - 代理工作流
- `langchain-openai` - OpenAI 兼容的 LangChain 集成
- `python-dotenv` - 环境变量管理

## 开发说明

### 添加新的 MCP 工具

1. 在相应的 MCP 服务器文件中添加新的工具函数
2. 使用 `@app.tool()` 装饰器注册工具
3. 在 `test_mcp.py` 中更新服务器配置（如果需要）

### 扩展功能

项目采用模块化设计，可以轻松添加新的 MCP 服务器：
- 数据库操作 MCP
- 网络请求 MCP
- 系统监控 MCP
- 其他自定义功能

## 故障排除

### 常见问题

1. **MCP 服务器启动失败**
   - 检查 Python 路径和文件路径
   - 确认依赖包已正确安装

2. **API 连接失败**
   - 检查环境变量配置
   - 验证 API 密钥和基础 URL

3. **权限错误**
   - 确保对目标目录有读写权限
   - 检查文件路径是否正确

## 许可证

本项目采用 MIT 许可证。

## 贡献

欢迎提交 Issue 和 Pull Request 来改进这个项目。
