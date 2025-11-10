# 机械臂控制系统

一个基于 FastAPI 的机械臂控制网页应用，提供直观的界面来控制机械臂的各个关节和夹具。

## 功能特点

- **4轴控制**: 底座旋转、肩部关节、肘部关节、腕部关节
- **夹具控制**: 打开/关闭夹具
- **实时状态监控**: 显示所有关节的当前角度和夹具状态
- **重置功能**: 一键重置机械臂到初始位置
- **操作日志**: 记录所有用户操作和系统状态
- **响应式设计**: 支持桌面和移动设备

## 安装和运行

### 1. 安装依赖

```bash
python3 -m venv venv && source venv/bin/activate && pip install -r requirements.txt


 # 改进
conda deactivate
pip install -r requirements.txt --break-system-packages
```

### 2. 启动服务器

```bash
venv/bin/python main.py

uvicorn main:app --host localhost --port 3000 --reload
```

### 3. 访问应用

服务器启动后，在浏览器中访问：
- 主界面: http://localhost:8000
- API文档: http://localhost:8000/docs

## 项目结构

```
arm_app/
├── main.py              # FastAPI 后端主程序
├── requirements.txt     # Python 依赖包
├── README.md           # 项目说明文档
└── templates/
    └── index.html      # 前端界面
```

## API 接口

- `GET /` - 返回前端界面
- `GET /status` - 获取机械臂当前状态
- `POST /control` - 控制机械臂关节运动
- `POST /gripper` - 控制夹具开合
- `POST /reset` - 重置机械臂到初始位置

## 控制范围

- **底座旋转**: -180° 到 180°
- **肩部关节**: -90° 到 90°
- **肘部关节**: -90° 到 90°
- **腕部关节**: -180° 到 180°
- **夹具**: 打开/关闭两种状态

## 技术栈

- **后端**: FastAPI, Uvicorn
- **前端**: HTML5, CSS3, JavaScript
- **模板**: Jinja2

## 使用说明

1. 启动服务器后，打开浏览器访问 http://localhost:8000
2. 使用滑块控制各个关节的角度
3. 点击按钮控制夹具的打开和关闭
4. 点击重置按钮将机械臂恢复到初始位置
5. 在状态面板查看实时状态和操作日志

## 注意事项

- 当前为模拟控制，实际硬件连接需要根据具体机械臂型号进行适配
- 所有操作都会在终端输出详细日志，便于调试
- 应用支持跨域请求，方便与其他系统集成
