# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_demo_launch


# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder("panda", package_name="panda_moveit_config").to_moveit_configs()
#     return generate_demo_launch(moveit_config)

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("panda", package_name="panda_moveit_config").to_moveit_configs()
    
    # 获取原有的launch描述
    demo_launch = generate_demo_launch(moveit_config)
    
    # # 创建您想要添加的节点
    # arm_robot_sim_path = os.path.join(get_package_share_directory('panda_moveit_config'))
    
    # moveit_config = (
    #     MoveItConfigsBuilder(robot_name="panda")
    #     .robot_description(file_path="config/panda.urdf.xacro")
    #     .robot_description_semantic(file_path="config/panda.srdf")
    #     .trajectory_execution(file_path="config/moveit_controllers.yaml")
    #     .moveit_cpp(arm_robot_sim_path + "/config/controller_setting.yaml")
    #     .to_moveit_configs()
    # )

    # example_node = Node(
    #             name="moveit_py",
    #             package='unity_control_example',
    #             executable='mover_panda_arm',
    #             output="both",
    #             parameters=[moveit_config.to_dict(),
    #                         {"use_sim_time": True},
    #                     ],
    #         )
    
    # 将原有launch描述和新节点组合
    launch_description = LaunchDescription()
    launch_description.add_action(demo_launch)
    # launch_description.add_action(example_node)
    
    return launch_description
