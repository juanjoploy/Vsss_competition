import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Encontrar la ruta del paquete
    pkg_share = FindPackageShare('mi_robot_urdf').find('mi_robot_urdf')

    # Ruta correcta del archivo Xacro
    default_model_path = os.path.join(pkg_share, 'urdf', 'caja_con_ruedas.urdf.xacro')

    # Argumento para el modelo
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=default_model_path,
        description='Ruta del archivo URDF/Xacro del robot'
    )

    # Definir la descripción del robot
    robot_description = {'robot_description': Command(['xacro ', LaunchConfiguration('model')])}

    # Nodo de publicación de estados del robot
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description]
    )

    # Nodo para publicar estados de las articulaciones
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # Lanzar Gazebo con plugins ROS 2
    gazebo_process = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Nodo para spawn del robot en Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'my_first_robot', '-topic', 'robot_description'],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        gazebo_process,
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity
    ])
