from setuptools import find_packages, setup

import os

package_name = 'control_servers'
share_dir = os.path.join('share', package_name)

def get_data_files(source_dir, relative_dest):
    """
    生成文件夹复制的data_files配置
    :param source_dir: 源文件夹路径（相对于setup.py）
    :param relative_dest: 目标路径（相对于share/package_name）
    :return: data_files格式的列表 [(目标目录, [文件列表]), ...]
    """
    data_files = []
    if not os.path.exists(source_dir):
        return data_files  # 源文件夹不存在则返回空
    
    # 遍历源文件夹所有文件和子目录
    for root, _, files in os.walk(source_dir):
        if not files:
            continue  # 跳过空目录（可选）
        
        # 计算目标目录：share/包名/relative_dest/相对源文件夹的路径
        rel_path = os.path.relpath(root, source_dir)
        dest_dir = os.path.join('share', package_name, relative_dest, rel_path)
        
        # 收集当前目录下的所有文件路径
        file_paths = [os.path.join(root, file) for file in files]
        data_files.append((dest_dir, file_paths))
    
    return data_files

# 配置要复制的文件夹：源文件夹为"app"，目标相对路径为"app"
# 最终会复制到 install/control_servers/share/control_servers/app/
app_data_files = get_data_files(
    source_dir=os.path.join(os.path.dirname(__file__), 'app'),
    relative_dest='app'
)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]  + app_data_files,  # 合并自定义文件夹复制配置
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='2388245387@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 添加此行，格式：节点命令 = 包名.文件名:主函数
            'joint_control_app = control_servers.joint_control_app:main',
        ],
    },
)
