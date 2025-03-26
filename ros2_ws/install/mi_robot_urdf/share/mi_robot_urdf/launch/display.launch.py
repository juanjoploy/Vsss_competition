import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Obtener la ruta del paquete
    pkg_share = FindPackageShare('mi_robot_urdf').find('mi_robot_urdf')

    # Definir argumentos de lanzamiento
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=PathJoinSubstitution([pkg_share, 'urdf', 'caja_con_ruedas.urdf.xacro']),
        description='Ruta del archivo URDF/Xacro'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Abrir GUI del Joint State Publisher'
    )

    rvizconfig_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=PathJoinSubstitution([pkg_share, 'rviz', 'urdf.rviz']),
        description='Ruta del archivo de configuración de RViz'
    )

    # Obtener los valores de configuración
    model = LaunchConfiguration('model')
    gui = LaunchConfiguration('gui')
    rvizconfig = LaunchConfiguration('rvizconfig')

    # Definir la descripción del robot

    robot_description = {'robot_description': Command(['xacro ', LaunchConfiguration('model')])}


    # Definir nodos
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        condition=IfCondition(gui)
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(gui)
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[robot_description]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rvizconfig],
        output='screen'
    )

    # Retornar la descripción del lanzamiento
    return LaunchDescription([
        model_arg,
        gui_arg,
        rvizconfig_arg,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])
