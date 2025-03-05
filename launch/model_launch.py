# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node

# 定义函数名称为：generate_launch_description
def generate_launch_description():
    sw_data_processing_node = Node(
        package="sw_data_processing",
        executable="sw_data_processing_node"
    )
    data_processing_node = Node(
        package="data_processing_node",
        executable="data_processing_node"
    )
    model_node = Node(
        package="model_node",
        executable="model_node"
    )

    motion_mux_node =Node(
        package="motion_mux",
        executable="motion_mux_node"
    )

    escape_node  =Node(
        package="escape_node",
        executable="escape_node"
    )


    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription([
        sw_data_processing_node,
        data_processing_node,
        model_node,
        motion_mux_node,
        escape_node
    ])

    # 返回让 ROS 2 根据 launch 描述执行节点
    return launch_description
