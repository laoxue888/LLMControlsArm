import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi.responses import HTMLResponse
from fastapi import Request
from pydantic import BaseModel
from typing import Dict, List
import uvicorn
import datetime
import threading
from sensor_msgs.msg import JointState
from panda_arm_msg.srv import AngleControlArm, AngleControlArm_Request

control_servers_path = get_package_share_directory('control_servers')

app = FastAPI(title="机械臂控制API", description="机械臂控制网页应用后端API")

# 设置模板目录
templates = Jinja2Templates(directory=os.path.join(control_servers_path, 'app', 'templates'))

# 挂载静态文件目录
app.mount("/static", StaticFiles(directory=os.path.join(control_servers_path, 'app', 'static')), name="static")

# 允许跨域请求，方便前端调用
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 机械臂状态模型
class ArmStatus(BaseModel):
    joint1_angle: float = 0.0
    joint2_angle: float = 0.0
    joint3_angle: float = 0.0
    joint4_angle: float = 0.0
    joint5_angle: float = 0.0
    joint6_angle: float = 0.0
    joint7_angle: float = 0.0
    gripper_open: bool = False
    gripper_size: float = 0.0  # 夹具开度大小 (0-100)

# 控制指令模型
class ControlCommand(BaseModel):
    joint: str
    value: float

# 夹具控制模型
class GripperCommand(BaseModel):
    open_gripper: bool

# 夹具开度控制模型
class GripperSizeCommand(BaseModel):
    gripper_size: float


# 模拟机械臂状态
arm_status = ArmStatus()

arm_command = ArmStatus()

@app.get("/", response_class=HTMLResponse)
async def root(request: Request):
    """返回机械臂控制前端界面"""
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{timestamp}] 用户访问前端界面")
    return templates.TemplateResponse("index.html", {"request": request})

@app.get("/api")
async def api_info():
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{timestamp}] API状态检查")
    return {"message": "机械臂控制API服务运行中", "status": "正常"}

@app.get("/status")
async def get_status():
    """获取机械臂当前状态"""
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{timestamp}] 获取机械臂状态: Joint1={arm_status.joint1_angle}°, Joint2={arm_status.joint2_angle}°, Joint3={arm_status.joint3_angle}°, Joint4={arm_status.joint4_angle}°, Joint5={arm_status.joint5_angle}°, Joint6={arm_status.joint6_angle}°, Joint7={arm_status.joint7_angle}°, 夹具={'打开' if arm_status.gripper_open else '关闭'}")
    return arm_status


@app.post("/control")
async def control_arm(command: ControlCommand):
    """控制机械臂运动"""
    global arm_command, arm_status

    arm_command.joint1_angle = arm_status.joint1_angle
    arm_command.joint2_angle = arm_status.joint2_angle
    arm_command.joint3_angle = arm_status.joint3_angle
    arm_command.joint4_angle = arm_status.joint4_angle
    arm_command.joint5_angle = arm_status.joint5_angle
    arm_command.joint6_angle = arm_status.joint6_angle
    arm_command.joint7_angle = arm_status.joint7_angle
    arm_command.gripper_open = arm_status.gripper_open
    arm_command.gripper_size = arm_status.gripper_size
    
    joint = command.joint
    value = command.value
    
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{timestamp}] 用户控制指令: 关节={joint}, 目标角度={value}°")

    # #这里发送控制指令到机械臂控制系统
    # if joint == "joint1":
    #     arm_status.joint1_angle = max(-180, min(180, value))
    # elif joint == "joint2":
    #     arm_status.joint2_angle = max(-180, min(180, value))
    # elif joint == "joint3":
    #     arm_status.joint3_angle = max(-180, min(180, value))
    # elif joint == "joint4":
    #     arm_status.joint4_angle = max(-180, min(180, value))
    # elif joint == "joint5":
    #     arm_status.joint5_angle = max(-180, min(180, value))
    # elif joint == "joint6":
    #     arm_status.joint6_angle = max(-180, min(180, value))
    # elif joint == "joint7":
    #     arm_status.joint7_angle = max(-180, min(180, value))
    # else:
    #     error_msg = f"无效的关节名称: {joint}"
    #     print(f"[{timestamp}] 错误: {error_msg}")
    #     raise HTTPException(status_code=400, detail=error_msg)
    
    
    # while not self.client_angle.wait_for_service(timeout_sec=1.0):
    #     self.get_logger().info('service not available, waiting again...')

    # # 解析返回信号，通过服务发送位置信号
    # request = AngleControlArm_Request()
    # # request.position = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]  # 设置目标位置和姿态
    # request.position = [0., 0, 0., 0., 0., 0, 0.785]  # 设置目标位置和姿态

    # request.gripper_width = 0.04  # 设置夹爪宽度，单位为米

    # future = self.client_angle.call_async(request)
    # rclpy.spin_until_future_complete(self, future)

    # response = future.result()

    # if response.success:
    #     print(f'{request.position} 执行成功！')
    # else:
    #     print(f'{request.position} 执行失败！')
    
    print(f"[{timestamp}] 控制成功: 关节 {joint} 已移动到 {value} 度")
    return {"message": f"关节 {joint} 已移动到 {value} 度", "status": arm_status}

@app.post("/gripper")
async def control_gripper(command: GripperCommand):
    """控制夹具开合"""
    global arm_status
    
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    action = "打开" if command.open_gripper else "关闭"
    print(f"[{timestamp}] 用户控制夹具: {action}")
    
    arm_status.gripper_open = command.open_gripper
    # 当打开夹具时，设置默认开度为50%，关闭时设为0%
    arm_status.gripper_size = 50.0 if command.open_gripper else 0.0
    print(f"[{timestamp}] 夹具控制成功: 夹具已{action}")
    return {"message": f"夹具已{action}", "status": arm_status}

@app.post("/gripper/size")
async def control_gripper_size(command: GripperSizeCommand):
    """控制夹具开度大小"""
    global arm_status
    
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    size = max(0.0, min(100.0, command.gripper_size))  # 限制在0-100范围内
    print(f"[{timestamp}] 用户调整夹具开度: {size}%")
    
    arm_status.gripper_size = size
    # 如果开度大于0，则夹具状态为打开
    arm_status.gripper_open = size > 0
    print(f"[{timestamp}] 夹具开度调整成功: 开度已设为 {size}%")
    return {"message": f"夹具开度已设为 {size}%", "status": arm_status}

@app.post("/reset")
async def reset_arm():
    """重置机械臂到初始位置"""
    global arm_status
    
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{timestamp}] 用户执行重置操作")
    
    arm_status.joint1_angle = 0.0
    arm_status.joint2_angle = 0.0
    arm_status.joint3_angle = 0.0
    arm_status.joint4_angle = 0.0
    arm_status.joint5_angle = 0.0
    arm_status.joint6_angle = 0.0
    arm_status.joint7_angle = 0.0
    arm_status.gripper_open = False
    arm_status.gripper_size = 0.0
    
    print(f"[{timestamp}] 重置成功: 所有关节已归零，夹具已关闭")
    return {"message": "机械臂已重置", "status": arm_status}

def arm_control_app_main():
    port = 8000
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"[{timestamp}] 启动机械臂控制系统...")
    print(f"[{timestamp}] FastAPI服务器运行在: http://0.0.0.0:{port}")
    print(f"[{timestamp}] 前端界面访问: http://localhost:{port}")
    print(f"[{timestamp}] API文档访问: http://localhost:{port}/docs")
    uvicorn.run(app, host="0.0.0.0", port=port)

class JointControlApp(Node):
    """关节控制应用节点"""
    def __init__(self):
        super().__init__('joint_control_app')  # 节点名称
        self.get_logger().info('关节控制应用节点已启动')

        # 可在此处添加订阅者、发布者或服务等逻辑
        # 示例：定时打印日志
        self.timer = self.create_timer(1.0, self.timer_callback)

        # 创建一个线程来运行FastAPI应用
        api_thread = threading.Thread(target=arm_control_app_main, daemon=True)
        api_thread.start()

        # 创建订阅器，订阅/joint_states话题
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.chatter_callback,
            10  # QoS配置，与原代码的1000略有不同，ROS 2中通常使用较小的值
        )
        self.subscription  # 防止未使用变量警告

        # 创建一个服务
        self.client_angle = self.create_client(AngleControlArm, 'angle_control_arm')


    def chatter_callback(self, msg):
        """处理接收到的关节状态消息"""
        global arm_status
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        joint_names = msg.name
        joint_positions = msg.position

        arm_status.joint1_angle = round(joint_positions[0] * 180.0 / 3.1415926, 2)

        arm_status.joint2_angle = round(joint_positions[1] * 180.0 / 3.1415926, 2)

        arm_status.joint3_angle = round(joint_positions[2] * 180.0 / 3.1415926, 2)

        arm_status.joint4_angle = round(joint_positions[3] * 180.0 / 3.1415926, 2)

        arm_status.joint5_angle = round(joint_positions[4] * 180.0 / 3.1415926, 2)

        arm_status.joint6_angle = round(joint_positions[5] * 180.0 / 3.1415926, 2)

        arm_status.joint7_angle = round(joint_positions[6] * 180.0 / 3.1415926, 2)

        # print(f"[{timestamp}] 接收到关节状态消息: {arm_status}")


    def timer_callback(self):
        """定时器回调函数（示例）"""
        self.get_logger().info('运行中：关节控制应用')

def main(args=None):
    rclpy.init(args=args)
    node = JointControlApp()
    try:
        rclpy.spin(node)  # 保持节点运行
    except KeyboardInterrupt:
        node.get_logger().info('节点已停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()