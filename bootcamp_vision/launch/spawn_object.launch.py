"""Spawn d'un objet sur la table du monde "vision" (Gazebo Ionic).

Demarre le PONT DE SERVICES ros_gz (config/bridge_spawn.yaml) + le noeud
scripts/spawn_object.py. Le monde doit deja tourner (vision_world.launch.py).

Exemples :
    ros2 launch bootcamp_vision spawn_object.launch.py object:=shapes
    ros2 launch bootcamp_vision spawn_object.launch.py object:=aruco aruco_id:=1
    ros2 launch bootcamp_vision spawn_object.launch.py object:=digit digit:=7
    ros2 launch bootcamp_vision spawn_object.launch.py object:=coffee_pack_2
    ros2 launch bootcamp_vision spawn_object.launch.py action:=respawn name:=vision_object_0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            OpaqueFunction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGS = [
    ('world', 'vision_table', 'Nom du monde gz (doit matcher bridge_spawn.yaml).'),
    ('object', 'shapes',
     'Modele ou alias : shapes|aruco|yolo|digit|coffee|trash|<nom_modele>.'),
    ('aruco_id', '0', 'ID du marqueur ArUco (objet aruco).'),
    ('digit', '0', 'Chiffre a afficher (objet digit).'),
    ('class_id', '', 'Classe logique (vide => derivee de l objet).'),
    ('pose', '', 'Pose "x y z [yaw]" (vide => centre de la table).'),
    ('count', '1', "Nombre d'objets."),
    ('name', '', "Nom d'entite (vide => vision_object_<i>)."),
    ('action', 'spawn', 'spawn | delete | respawn.'),
]


def _spawn_node(context, *_, **__):
    params = {
        'world': LaunchConfiguration('world').perform(context),
        'object': LaunchConfiguration('object').perform(context),
        'aruco_id': int(LaunchConfiguration('aruco_id').perform(context)),
        'digit': int(LaunchConfiguration('digit').perform(context)),
        'class_id': LaunchConfiguration('class_id').perform(context),
        'pose': LaunchConfiguration('pose').perform(context),
        'count': int(LaunchConfiguration('count').perform(context)),
        'name': LaunchConfiguration('name').perform(context),
        'action': LaunchConfiguration('action').perform(context),
    }
    return [Node(package='bootcamp_vision', executable='spawn_object.py',
                 name='object_spawner', output='screen', parameters=[params])]


def generate_launch_description():
    share = get_package_share_directory('bootcamp_vision')
    bridge_file = os.path.join(share, 'config', 'bridge_spawn.yaml')

    set_models_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(share, 'models'))

    service_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='spawn_service_bridge',
        parameters=[{'config_file': bridge_file}],
        output='screen',
    )

    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in ARGS]
        + [set_models_path, service_bridge, OpaqueFunction(function=_spawn_node)]
    )
