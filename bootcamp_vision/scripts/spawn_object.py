#!/usr/bin/env python3
"""Spawn / supprime des objets sur la table du monde "vision" (Gazebo Ionic).

Jour 4 (perception) : pose un objet sur la table de vision_table.sdf pour alimenter
les TP (formes, ArUco, YOLO, chiffre, cafe, dechets). Mecanisme 100 % Gazebo Ionic :
services ros_gz_interfaces/srv/SpawnEntity (creation) et DeleteEntity (suppression),
pontes vers /world/vision_table/create et /remove (cf. config/bridge_spawn.yaml).
PAS de spawn_entity.py (Gazebo Classic).

'object' accepte un NOM DE MODELE precis OU un ALIAS DE CATEGORIE :
    shapes  -> shape_star | shape_cube | shape_cylinder
    aruco   -> aruco_parcel            (param aruco_id => texture du marqueur)
    yolo    -> trash_bottle            (cible COCO 'bottle')
    digit   -> digit_panel             (param digit => texture du chiffre)
    coffee  -> coffee_pack_1|2|3
    trash   -> trash_can | trash_bottle | trash_carton

Exemples (le pont de services doit tourner, cf. spawn_object.launch.py) :
    ros2 run bootcamp_vision spawn_object.py --ros-args -p object:=shapes
    ros2 run bootcamp_vision spawn_object.py --ros-args -p object:=aruco -p aruco_id:=1
    ros2 run bootcamp_vision spawn_object.py --ros-args -p object:=digit -p digit:=7
    ros2 run bootcamp_vision spawn_object.py --ros-args -p object:=coffee_pack_2
    ros2 run bootcamp_vision spawn_object.py --ros-args -p action:=respawn -p name:=vision_object_0
"""

import math
import os
import random

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from ros_gz_interfaces.msg import Entity, EntityFactory
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity

# Alias de categorie -> liste de modeles.
CATEGORIES = {
    'shapes': ['shape_star', 'shape_cube', 'shape_cylinder'],
    'aruco':  ['aruco_parcel'],
    'yolo':   ['trash_bottle'],
    'digit':  ['digit_panel'],
    'coffee': ['coffee_pack_1', 'coffee_pack_2', 'coffee_pack_3'],
    'trash':  ['trash_can', 'trash_bottle', 'trash_carton'],
}


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class ObjectSpawner(Node):
    def __init__(self):
        super().__init__('object_spawner')
        self.declare_parameter('world', 'vision_table')
        self.declare_parameter('object', 'shapes')      # modele ou alias de categorie
        self.declare_parameter('aruco_id', 0)           # pour aruco_parcel
        self.declare_parameter('digit', 0)              # pour digit_panel
        self.declare_parameter('class_id', '')          # vide => derive de l'objet
        self.declare_parameter('pose', '')              # "x y z [yaw]" ; vide => centre table
        self.declare_parameter('table_xy', [0.0, 0.0])  # centre du plateau
        self.declare_parameter('table_z', 0.46)         # hauteur d'apparition (plateau a 0.40)
        self.declare_parameter('jitter', 0.0)           # rayon aleatoire autour du centre (m)
        self.declare_parameter('count', 1)
        self.declare_parameter('name', '')              # vide => vision_object_<i>
        self.declare_parameter('action', 'spawn')       # spawn|delete|respawn
        self.declare_parameter('timeout', 10.0)

        self.share = get_package_share_directory('bootcamp_vision')
        self.world = self.get_parameter('world').value
        self.create_cli = self.create_client(SpawnEntity, f'/world/{self.world}/create')
        self.remove_cli = self.create_client(DeleteEntity, f'/world/{self.world}/remove')

    # ------------------------------------------------------------------ SDF
    def resolve_model(self):
        """object (alias ou nom) -> nom de modele concret."""
        obj = self.get_parameter('object').value
        if obj in CATEGORIES:
            return random.choice(CATEGORIES[obj])
        return obj  # suppose un nom de modele existant

    def build_sdf(self, model):
        path = os.path.join(self.share, 'models', model, 'model.sdf')
        if not os.path.isfile(path):
            self.get_logger().error(f"modele introuvable : '{model}' ({path})")
            return None, ''
        with open(path, 'r') as f:
            sdf = f.read()
        # Substitution de texture pour ArUco / chiffre.
        default_class = model
        if model == 'aruco_parcel':
            mid = int(self.get_parameter('aruco_id').value)
            sdf = sdf.replace('aruco_0.png', f'aruco_{mid}.png')
            default_class = str(mid)
        elif model == 'digit_panel':
            d = int(self.get_parameter('digit').value)
            sdf = sdf.replace('digit_0.png', f'digit_{d}.png')
            default_class = str(d)
        return sdf, default_class

    # ----------------------------------------------------------------- pose
    def make_pose(self, index):
        pose_str = str(self.get_parameter('pose').value).strip()
        raw = [float(v) for v in pose_str.split()] if pose_str else []
        if len(raw) >= 3:
            x, y, z = raw[0], raw[1], raw[2]
            yaw = raw[3] if len(raw) > 3 else 0.0
            if index:
                x += 0.07 * index
        else:
            cx, cy = [float(v) for v in self.get_parameter('table_xy').value]
            j = float(self.get_parameter('jitter').value)
            x = cx + (random.uniform(-j, j) if j else 0.07 * index)
            y = cy + (random.uniform(-j, j) if j else 0.0)
            z = float(self.get_parameter('table_z').value)
            yaw = random.uniform(-math.pi, math.pi) if j else 0.0
        p = Pose()
        p.position.x, p.position.y, p.position.z = x, y, z
        _, _, qz, qw = yaw_to_quat(yaw)
        p.orientation.z, p.orientation.w = qz, qw
        return p

    # -------------------------------------------------------------- services
    def _wait(self, client, label):
        if not client.wait_for_service(timeout_sec=float(self.get_parameter('timeout').value)):
            self.get_logger().error(
                f"service {label} indisponible (pont de services lance ? "
                f"cf. spawn_object.launch.py)")
            return False
        return True

    def spawn_one(self, name, sdf, pose):
        if not self._wait(self.create_cli, f'/world/{self.world}/create'):
            return False
        req = SpawnEntity.Request()
        ef = EntityFactory()
        ef.name = name
        ef.allow_renaming = False
        ef.sdf = sdf
        ef.pose = pose
        req.entity_factory = ef
        fut = self.create_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        ok = fut.result() is not None and fut.result().success
        self.get_logger().info(
            f"spawn '{name}' @ ({pose.position.x:.2f},{pose.position.y:.2f},"
            f"{pose.position.z:.2f}) -> {'OK' if ok else 'ECHEC'}")
        return ok

    def delete_one(self, name):
        if not self._wait(self.remove_cli, f'/world/{self.world}/remove'):
            return False
        req = DeleteEntity.Request()
        ent = Entity()
        ent.name = name
        ent.type = Entity.MODEL
        req.entity = ent
        fut = self.remove_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        ok = fut.result() is not None and fut.result().success
        self.get_logger().info(f"delete '{name}' -> {'OK' if ok else 'ECHEC (absent ?)'}")
        return ok

    # ------------------------------------------------------------------- run
    def names(self):
        base = self.get_parameter('name').value or 'vision_object'
        n = int(self.get_parameter('count').value)
        if self.get_parameter('name').value and n == 1:
            return [base]
        return [f'{base}_{i}' for i in range(n)]

    def run(self):
        action = self.get_parameter('action').value
        names = self.names()

        if action in ('delete', 'respawn'):
            for nm in names:
                self.delete_one(nm)
            if action == 'delete':
                return True

        model = self.resolve_model()
        sdf, default_class = self.build_sdf(model)
        if sdf is None:
            return False
        class_id = self.get_parameter('class_id').value or default_class
        self.get_logger().info(
            f"object='{self.get_parameter('object').value}' modele='{model}' "
            f"class_id='{class_id}' x{len(names)}")
        ok = True
        for i, nm in enumerate(names):
            ok = self.spawn_one(nm, sdf, self.make_pose(i)) and ok
        return ok


def main():
    rclpy.init()
    node = ObjectSpawner()
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    main()
