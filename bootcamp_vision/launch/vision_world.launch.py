"""Monde "vision" du Jour 4 : caméra fixe au-dessus d'une table (Gazebo Ionic).

Démarre :
  - gz sim (worlds/vision_table.sdf), GUI ou headless ;
  - le pont caméra ros_gz (config/bridge_camera.yaml) -> /camera/image_raw + /camera/camera_info ;
  - les TF statiques world -> table_link -> camera_link -> camera_optical_frame ;
  - (optionnel) le spawn d'un objet sur la table (arg object:=...).

Exemples :
    ros2 launch bootcamp_vision vision_world.launch.py
    ros2 launch bootcamp_vision vision_world.launch.py headless:=true
    ros2 launch bootcamp_vision vision_world.launch.py object:=aruco aruco_id:=1
    ros2 launch bootcamp_vision vision_world.launch.py object:=shapes

Spawn a chaud (autre terminal, monde lance) :
    ros2 launch bootcamp_vision spawn_object.launch.py object:=coffee
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('bootcamp_vision')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    world = os.path.join(share, 'worlds', 'vision_table.sdf')
    cam_bridge = os.path.join(share, 'config', 'bridge_camera.yaml')

    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    obj = LaunchConfiguration('object')
    spawn_requested = PythonExpression(["'", obj, "' != ''"])

    set_models_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(share, 'models'))

    # gz sim : GUI (-r) ou serveur seul (-s -r) selon headless.
    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [world, ' -r -v4']}.items(),
        condition=UnlessCondition(headless),
    )
    gz_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [world, ' -s -r -v4']}.items(),
        condition=IfCondition(headless),
    )

    camera_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='camera_bridge',
        parameters=[{'config_file': cam_bridge, 'use_sim_time': use_sim_time}],
        output='screen',
    )

    # TF statiques : world -> table_link (plateau a z=0.40) -> camera_link (z=0.95)
    # -> camera_optical_frame (REP-103, vue du dessus : rotation pi autour de X =>
    # optique x=+X, y=-Y, z=-Z). Cf. README pour la back-projection.
    tf_table = Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_table',
        arguments=['--frame-id', 'world', '--child-frame-id', 'table_link',
                   '--z', '0.40'])
    tf_camera = Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_camera',
        arguments=['--frame-id', 'world', '--child-frame-id', 'camera_link',
                   '--z', '0.95'])
    tf_optical = Node(
        package='tf2_ros', executable='static_transform_publisher', name='tf_optical',
        arguments=['--frame-id', 'camera_link', '--child-frame-id', 'camera_optical_frame',
                   '--roll', '3.14159265'])

    # Spawn optionnel d'un objet (apres montee de gz).
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([share, 'launch', 'spawn_object.launch.py'])),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'object': obj,
            'aruco_id': LaunchConfiguration('aruco_id'),
            'digit': LaunchConfiguration('digit'),
            'pose': LaunchConfiguration('pose'),
        }.items(),
        condition=IfCondition(spawn_requested),
    )
    delayed_spawn = TimerAction(period=6.0, actions=[spawn])

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false',
                              description='true = Gazebo serveur seul (sans GUI).'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Utiliser l horloge sim.'),
        DeclareLaunchArgument('world', default_value='vision_table',
                              description='Nom du monde gz (pour le spawn).'),
        DeclareLaunchArgument('object', default_value='',
                              description='Objet a spawner au demarrage (vide = aucun).'),
        DeclareLaunchArgument('aruco_id', default_value='0', description='ID ArUco.'),
        DeclareLaunchArgument('digit', default_value='0', description='Chiffre (digit).'),
        DeclareLaunchArgument('pose', default_value='',
                              description='Pose "x y z [yaw]" de l objet (vide = centre table).'),
        set_models_path,
        gz_gui,
        gz_headless,
        camera_bridge,
        tf_table,
        tf_camera,
        tf_optical,
        delayed_spawn,
    ])
