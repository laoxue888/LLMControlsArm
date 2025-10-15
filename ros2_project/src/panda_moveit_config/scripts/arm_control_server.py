#!/usr/bin/env python3

import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
# set pose goal with PoseStamped message
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters 
from moveit.core.kinematic_constraints import construct_joint_constraint
from panda_arm_msg.srv import ControlRvizArm, ControlRvizArm_Response
from panda_arm_msg.srv import AngleControlArm, AngleControlArm_Response
from panda_arm_msg.srv import TargetControlArm, TargetControlArm_Response

class Controller(Node):

    def __init__(self):
        super().__init__('commander')
        # self.subscription = self.create_subscription(
        #     Float64MultiArray,
        #     '/target_point',
        #     self.listener_callback,
        #     10)
        # self.subscription

        self.target_control = self.create_service(
            TargetControlArm, 
            'target_control_arm', 
            self.handle_request_target_control)

        self.angle_control = self.create_service(
            AngleControlArm, 
            'angle_control_arm', 
            self.handle_request_angle_control)

        self.pose_goal = PoseStamped()
        self.pose_goal.header.frame_id = "panda_link0"
        # instantiate MoveItPy instance and get planning component
        self.panda = MoveItPy(node_name="moveit_py")
        self.arm_planning_component_name = "panda_arm"
        self.hand_planning_component_name = "panda_hand"

        self.panda_arm = self.panda.get_planning_component(self.arm_planning_component_name)
        self.panda_hand = self.panda.get_planning_component(self.hand_planning_component_name)
        self.logger = get_logger("moveit_py.pose_goal")

        robot_model = self.panda.get_robot_model()
        self.robot_state = RobotState(robot_model)

        # self.height = 0.18
        # self.pick_height = 0.126
        # self.carrying_height = 0.3
        # self.init_angle = -0.3825

    def plan_and_execute(self,
        robot,
        planning_component,
        logger,
        single_plan_parameters=None,
        multi_plan_parameters=None,
        sleep_time=0.0,
    ):
        """Helper function to plan and execute a motion."""
        # plan to goal
        logger.info("Planning trajectory")
        if multi_plan_parameters is not None:
            plan_result = planning_component.plan(
                multi_plan_parameters=multi_plan_parameters
            )
        elif single_plan_parameters is not None:
            plan_result = planning_component.plan(
                single_plan_parameters=single_plan_parameters
            )
        else:
            plan_result = planning_component.plan()

        # execute the plan
        if plan_result:
            logger.info("Executing plan")
            robot_trajectory = plan_result.trajectory # 获取规划结果的轨迹
            robot_trajectory_msg = robot_trajectory.get_robot_trajectory_msg()
            joint_trajectory = robot_trajectory_msg.joint_trajectory # 获取关节轨迹
            # logger.info("joint_trajectory: {}".format(joint_trajectory))

            robot.execute(robot_trajectory, controllers=[]) # 控制rviz2的机械臂运动

            time.sleep(sleep_time)
            return True, joint_trajectory # 返回规划结果的轨迹
        else:
            logger.error("Planning failed")
            time.sleep(sleep_time)
            return False, None

    # function to move a gripper
    def move_to(self, x, y, z, xo, yo, zo, wo):

        self.pose_goal.pose.position.x = x
        self.pose_goal.pose.position.y = y
        self.pose_goal.pose.position.z = z
        self.pose_goal.pose.orientation.x = xo
        self.pose_goal.pose.orientation.y = yo
        self.pose_goal.pose.orientation.z = zo
        self.pose_goal.pose.orientation.w = wo
        self.panda_arm.set_goal_state(pose_stamped_msg = self.pose_goal, pose_link="panda_link8")
        return self.plan_and_execute(self.panda, self.panda_arm, self.logger, sleep_time=1.0)

    # function for a gripper action
    def gripper_action(self, width=0.0):

        self.panda_hand.set_start_state_to_current_state()

        joint_values = {"panda_finger_joint1": width}

        self.robot_state.joint_positions = joint_values
        joint_constraint = construct_joint_constraint(
            robot_state = self.robot_state,
            joint_model_group = self.panda.get_robot_model().get_joint_model_group(self.hand_planning_component_name),
        )        
        self.panda_hand.set_goal_state(motion_plan_constraints=[joint_constraint])
        return self.plan_and_execute(self.panda, self.panda_hand, self.logger, sleep_time=3.0)

    def handle_request_target_control(self, request, response):
        """"""
        position = request.position
        # gripper_state = request.open_or_close
        response = TargetControlArm_Response()
        # 使用服务通信
        # 这里加判断 ，检测是否规划成功
        plan_flag, joint_trajectory = self.move_to(position[0], position[1], position[2], position[3], position[4], position[5], position[6])
        if not plan_flag:
            response.success = False
            return response

        plan_flag, joint_trajectory = self.gripper_action(request.gripper_width)
        if not plan_flag:
            response.success = False
            return response

        response.joint_trajectory = joint_trajectory
        response.success = True
        return response

    def handle_request_angle_control(self, request, response):
        """"""
        position = request.position
        # gripper_state = request.open_or_close
        response = AngleControlArm_Response()
        # 使用服务通信
        # 这里加判断 ，检测是否规划成功
        # plan_flag, joint_trajectory = self.move_to(position[0], position[1], position[2], position[3], position[4], position[5], position[6])
    
        # # set plan start state to current state
        self.panda_arm.set_start_state_to_current_state()
        # set constraints message
        joint_values = {
                "panda_joint1": position[0],
                "panda_joint2": position[1],
                "panda_joint3": position[2],
                "panda_joint4": position[3],
                "panda_joint5": position[4],
                "panda_joint6": position[5],
                "panda_joint7": position[6],
        }
        self.robot_state.joint_positions = joint_values
        joint_constraint = construct_joint_constraint(
                robot_state=self.robot_state,
                joint_model_group=self.panda.get_robot_model().get_joint_model_group(self.arm_planning_component_name),
        )
        self.panda_arm.set_goal_state(motion_plan_constraints=[joint_constraint])

        # plan to goal
        plan_flag, joint_trajectory = self.plan_and_execute(self.panda, self.panda_arm, self.logger)
        
        if not plan_flag:
            response.success = False
            return response

        plan_flag, joint_trajectory = self.gripper_action(request.gripper_width)
        if not plan_flag:
            response.success = False
            return response

        response.joint_trajectory = joint_trajectory
        response.success = True
        return response

def main():
    """"""
    rclpy.init(args=None)

    controller = Controller()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(controller)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    rate = controller.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    executor_thread.join()

if __name__ == '__main__':
    """"""
    main()
    



