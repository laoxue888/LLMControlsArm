import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray
import time

class ArmEndEffectorPosition(Node):
    def __init__(self):
        super().__init__('arm_end_effector_position')
        
        # 创建TF2监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 定时器，定期获取末端位置
        self.timer = self.create_timer(0.05, self.get_end_effector_pose)

        # 创建话题，发布关节状态
        self.pub_end_pose = self.create_publisher(
            Float64MultiArray, '/end_effector_pose', 10)

        # 定义坐标系名称（根据你的机械臂配置修改）
        self.base_frame = 'panda_link0'  # 基础坐标系
        self.end_effector_frame = 'panda_link8'  # 末端执行器坐标系
    
    def get_end_effector_pose(self):
        try:
            # 获取从基础坐标系到末端执行器坐标系的变换
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.end_effector_frame,
                rclpy.time.Time())
            
            # 提取位置和姿态信息
            position = transform.transform.translation
            rotation = transform.transform.rotation
            # rotation.y
            # 发布末端执行器位置和姿态
            end_effector_pose = Float64MultiArray(data=[position.x, position.y, position.z,rotation.x, rotation.y, rotation.z, rotation.w])
            self.pub_end_pose.publish(end_effector_pose)

            self.get_logger().info(
                f"End effector position: x={position.x:.3f}, y={position.y:.3f}, z={position.z:.3f}")
            
        except TransformException as ex:
            pass
            # self.get_logger().warn(
            #     f'Could not transform {self.base_frame} to {self.end_effector_frame}: {ex}')

def main(args=None):
    rclpy.init(args=args)
    node = ArmEndEffectorPosition()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()