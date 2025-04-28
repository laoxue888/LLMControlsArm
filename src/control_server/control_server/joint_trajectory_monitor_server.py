import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from action_msgs.msg import GoalStatusArray, GoalStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from niryo_moveit.srv import MoverService, MoverService_Request, MoverService_Response
from niryo_moveit.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory

class FollowJointTrajectoryMonitor(Node):
    def __init__(self):
        super().__init__('follow_joint_trajectory_monitor')

        # group_name = 'niryo_one_arm'
        group_name = 'panda_arm'
        
        # 设置QoS配置以匹配动作服务器
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=10
        )
        
        # 订阅动作状态主题
        self._status_sub = self.create_subscription(
            GoalStatusArray,
            '/{}_controller/follow_joint_trajectory/_action/status'.format(group_name),
            self._status_callback,
            qos_profile
        )
        
        # 订阅反馈主题
        self._feedback_sub = self.create_subscription(
            FollowJointTrajectory.Impl.FeedbackMessage,
            '/{}_controller/follow_joint_trajectory/_action/feedback'.format(group_name),
            self._feedback_callback,
            qos_profile
        )

        self.get_logger().info('开始监听/{}_controller/follow_joint_trajectory动作信息...'.format(group_name))

        self.i_test = 0
        # 发布话题
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_topic', 10)

    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('收到反馈:')
        self.get_logger().info(f'Goal ID: {feedback_msg.goal_id.uuid}')
        self.get_logger().info(f'Header: {feedback.header}')
        self.get_logger().info(f'Joint names: {feedback.joint_names}')
        if feedback.desired.positions:
            self.get_logger().info(f'Desired positions: {feedback.desired.positions}')
        if feedback.actual.positions:
            self.get_logger().info(f'Actual positions: {feedback.actual.positions}')
        if feedback.error.positions:
            self.get_logger().info(f'Error positions: {feedback.error.positions}')
        self.get_logger().info('---')

        my_joint_trajectory = JointTrajectory()
        my_joint_trajectory.header = feedback.header
        my_joint_trajectory.joint_names = feedback.joint_names
        my_joint_trajectory.points.append(feedback.desired)
        # rviz_pose = RobotTrajectory()
        # rviz_pose.joint_trajectory = my_joint_trajectory
        self.publisher_.publish(my_joint_trajectory)
    
    def _status_callback(self, status_msg):
        for status_info in status_msg.status_list:
            self.get_logger().info(f'收到状态更新 - Goal ID: {status_info.goal_info.goal_id.uuid}')
            self.get_logger().info(f'状态: {self._status_to_str(status_info.status)}')
            self.get_logger().info('---')

        # print('执行次数:', self.i_test)
        
    def _status_to_str(self, status):
        if status == GoalStatus.STATUS_UNKNOWN:
            return 'UNKNOWN'
        elif status == GoalStatus.STATUS_ACCEPTED:
            return 'ACCEPTED'
        elif status == GoalStatus.STATUS_EXECUTING:
            return 'EXECUTING'
        elif status == GoalStatus.STATUS_CANCELING:
            return 'CANCELING'
        elif status == GoalStatus.STATUS_SUCCEEDED:
            return 'SUCCEEDED'
        elif status == GoalStatus.STATUS_CANCELED:
            return 'CANCELED'
        elif status == GoalStatus.STATUS_ABORTED:
            return 'ABORTED'
        else:
            return f'UNKNOWN_STATUS_CODE_{status}'


def main(args=None):
    rclpy.init(args=args)
    monitor = FollowJointTrajectoryMonitor()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()