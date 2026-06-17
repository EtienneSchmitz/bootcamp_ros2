#!/usr/bin/env python3
"""Generation d'un dataset YOLO synthetique AUTO-LABELLISE (Jour 4, TP03).

But : produire, dans le monde vision_table.sdf (camera fixe VUE DU DESSUS), des
images variees d'objets du cours avec leurs boites englobantes 2D, au format
Ultralytics, pour ENTRAINER un YOLO custom sans annotation manuelle. On randomise
l'OBJET (pose, yaw, +/- roll/pitch, distracteurs, lumiere), PAS la sim globale.

================================ AUTO-LABELLISATION ============================
VOIE B (par defaut, implementee ici) : PROJECTION 3D -> 2D.
  On CONNAIT la pose de spawn (on la choisit) et les dimensions de chaque modele
  (table EXTENTS ci-dessous). On projette les 8 coins de la boite 3D via les
  intrinseques de la camera top-down -> boite 2D (min/max u,v). Exact, deterministe,
  100 % Python, aucun edit du monde, headless-friendly.

VOIE A (option, NON cablee) : capteur gz `boundingbox_camera`.
  Alternative "native gz". Esquisse a ajouter dans worlds/vision_table.sdf, dans le
  meme <link name="camera_link"> que vision_camera, aligne sur la camera RGB :

      <sensor name="vision_bbox" type="boundingbox_camera">
        <pose>0 0 0.95 0 1.5708 0</pose>
        <topic>boxes</topic>
        <camera>
          <box_type>2d</box_type>
          <horizontal_fov>1.2</horizontal_fov>
          <image><width>1280</width><height>720</height></image>
          <clip><near>0.05</near><far>10.0</far></clip>
        </camera>
      </sensor>

  + un <label>ENTIER</label> par modele (plugin gz::sim::systems::Label sur chaque
  model.sdf) + pont du type gz.msgs.AnnotatedAxisAligned2DBox_V cote ROS. Non retenu
  par defaut : support ros_gz_bridge incertain sous Kilted et edition de chaque modele.
===============================================================================

Pre-requis (2 terminaux) :
    # 1) Monde + camera (sans GUI) :
    ros2 launch bootcamp_vision vision_world.launch.py headless:=true
    # 2) Pont des SERVICES de spawn (create/remove), puis la capture :
    ros2 launch bootcamp_vision spawn_object.launch.py object:=''   # juste le pont
    ros2 run bootcamp_vision capture_dataset.py \
        --classes coffee_pack_1 coffee_pack_2 coffee_pack_3 trash_bottle trash_can \
        --per-class 300 --out ~/yolo_ds --val-split 0.2 --seed 42
"""

import argparse
import math
import os
import random
import subprocess
import sys
import time

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from sensor_msgs.msg import CameraInfo, Image
from ros_gz_interfaces.msg import Entity, EntityFactory
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity

# --- Geometrie fixe du monde vision_table.sdf -------------------------------
CAM_Z = 0.95          # hauteur ABSOLUE de la camera (m), regarde -Z
#                       (=> h = CAM_Z - 0.40 = 0.55 m au-dessus du plateau)
# Intrinseques de secours si /camera/camera_info n'arrive pas (cf. README).
FX_DEF, FY_DEF, CX_DEF, CY_DEF = 935.5, 935.5, 640.0, 360.0
W_DEF, H_DEF = 1280, 720

# Alias de categorie -> modeles concrets (identique a spawn_object.py).
CATEGORIES = {
    'shapes': ['shape_star', 'shape_cube', 'shape_cylinder'],
    'aruco':  ['aruco_parcel'],
    'yolo':   ['trash_bottle'],
    'digit':  ['digit_panel'],
    'coffee': ['coffee_pack_1', 'coffee_pack_2', 'coffee_pack_3'],
    'trash':  ['trash_can', 'trash_bottle', 'trash_carton'],
}

# Demi-extents AABB (hx, hy, hz) autour de l'origine du link, en metres.
# Derives des model.sdf (approximation symetrique suffisante en vue du dessus,
# leger parallaxe en z neglige). Modifier ici si un modele change de taille.
EXTENTS = {
    'shape_cube':     (0.020, 0.020, 0.020),  # boite 0.04^3
    'shape_cylinder': (0.015, 0.015, 0.020),  # cyl O0.03 x 0.04
    'shape_star':     (0.025, 0.025, 0.015),  # etoile O0.05 x 0.03
    'aruco_parcel':   (0.030, 0.030, 0.030),  # boite 0.06^3
    'trash_bottle':   (0.015, 0.015, 0.060),  # corps + col ~0.12 de haut
    'trash_can':      (0.0165, 0.0165, 0.045),  # cyl O0.033 x 0.09
    'trash_carton':   (0.030, 0.020, 0.035),  # boite 0.06 x 0.04 x 0.07
    'digit_panel':    (0.040, 0.040, 0.003),  # plaque 0.08 x 0.08 x 0.006
    'coffee_pack_1':  (0.025, 0.015, 0.040),  # boite 0.05 x 0.03 x 0.08
    'coffee_pack_2':  (0.025, 0.015, 0.040),
    'coffee_pack_3':  (0.025, 0.015, 0.040),
}

# Textures disponibles par defaut (cf. generate_aruco.py / generate_digit.py).
ARUCO_IDS = [0, 1, 2]
DIGITS = [0, 1, 2, 3]


def rpy_to_quat(roll, pitch, yaw):
    """rpy (rad) -> quaternion (x, y, z, w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def rpy_to_matrix(roll, pitch, yaw):
    """Matrice de rotation 3x3 R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


class DatasetCapturer(Node):
    """Pilote le spawn d'objets + capture d'images + ecriture des labels YOLO."""

    def __init__(self, args):
        super().__init__('dataset_capturer')
        self.args = args
        self.world = args.world
        self.share = get_package_share_directory('bootcamp_vision')

        # Clients de services de spawn/suppression (cf. spawn_object.py).
        self.create_cli = self.create_client(SpawnEntity, f'/world/{self.world}/create')
        self.remove_cli = self.create_client(DeleteEntity, f'/world/{self.world}/remove')

        # Abonnements camera (QoS sensor_data = best_effort, compatible gz bridge).
        self._img = None
        self._img_count = 0
        self.create_subscription(Image, '/camera/image_raw',
                                 self._on_image, qos_profile_sensor_data)
        self.fx, self.fy = FX_DEF, FY_DEF
        self.cx, self.cy = CX_DEF, CY_DEF
        self.width, self.height = W_DEF, H_DEF
        self._info_ok = False
        self.create_subscription(CameraInfo, '/camera/camera_info',
                                 self._on_info, qos_profile_sensor_data)

    # ----------------------------------------------------------- callbacks
    def _on_image(self, msg):
        self._img = msg
        self._img_count += 1

    def _on_info(self, msg):
        if self._info_ok:
            return
        # k = [fx 0 cx  0 fy cy  0 0 1]
        self.fx, self.fy = float(msg.k[0]), float(msg.k[4])
        self.cx, self.cy = float(msg.k[2]), float(msg.k[5])
        self.width, self.height = int(msg.width), int(msg.height)
        self._info_ok = True
        self.get_logger().info(
            f"camera_info : fx={self.fx:.1f} cx={self.cx:.1f} cy={self.cy:.1f} "
            f"{self.width}x{self.height}")

    # ------------------------------------------------------------- helpers
    def wait_services(self, timeout=15.0):
        ok = (self.create_cli.wait_for_service(timeout_sec=timeout)
              and self.remove_cli.wait_for_service(timeout_sec=timeout))
        if not ok:
            self.get_logger().error(
                "services create/remove indisponibles : lancer le pont "
                "(ros2 launch bootcamp_vision spawn_object.launch.py).")
        return ok

    def wait_camera_info(self, timeout=8.0):
        """Recupere les intrinseques (sinon garde les valeurs par defaut)."""
        t0 = time.monotonic()
        while not self._info_ok and time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
        if not self._info_ok:
            self.get_logger().warn(
                "camera_info non recu : intrinseques par defaut (935.5/640/360).")

    def resolve_model(self, entry):
        """Entree --classes (alias ou modele) -> nom de modele concret."""
        if entry in CATEGORIES:
            return random.choice(CATEGORIES[entry])
        return entry

    def build_sdf(self, model):
        """Lit le model.sdf + substitution de texture aruco/digit. Retourne (sdf, extra)."""
        path = os.path.join(self.share, 'models', model, 'model.sdf')
        if not os.path.isfile(path):
            self.get_logger().error(f"modele introuvable : '{model}' ({path})")
            return None, {}
        with open(path, 'r') as f:
            sdf = f.read()
        extra = {}
        if model == 'aruco_parcel':
            mid = random.choice(ARUCO_IDS)
            sdf = sdf.replace('aruco_0.png', f'aruco_{mid}.png')
            extra['aruco_id'] = mid
        elif model == 'digit_panel':
            d = random.choice(DIGITS)
            sdf = sdf.replace('digit_0.png', f'digit_{d}.png')
            extra['digit'] = d
        return sdf, extra

    def random_pose(self):
        """Pose aleatoire sur le plateau : (xyz, rpy). z = table_z."""
        b = self.args.table_bounds
        x = random.uniform(-b, b)
        y = random.uniform(-b, b)
        z = self.args.table_z
        yaw = random.uniform(0.0, 2 * math.pi)
        roll = pitch = 0.0
        if self.args.tilt:
            roll = random.uniform(-0.15, 0.15)
            pitch = random.uniform(-0.15, 0.15)
        return (x, y, z), (roll, pitch, yaw)

    def spawn(self, name, sdf, xyz, rpy):
        req = SpawnEntity.Request()
        ef = EntityFactory()
        ef.name = name
        ef.allow_renaming = False
        ef.sdf = sdf
        p = Pose()
        p.position.x, p.position.y, p.position.z = xyz
        qx, qy, qz, qw = rpy_to_quat(*rpy)
        p.orientation.x, p.orientation.y = qx, qy
        p.orientation.z, p.orientation.w = qz, qw
        ef.pose = p
        req.entity_factory = ef
        fut = self.create_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        return fut.result() is not None and fut.result().success

    def delete(self, name):
        req = DeleteEntity.Request()
        ent = Entity()
        ent.name = name
        ent.type = Entity.MODEL
        req.entity = ent
        fut = self.remove_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        # On ignore l'echec (objet deja absent).

    def fresh_frame(self, drop=3, timeout=5.0):
        """Attend une frame FRAICHE : jette `drop` images puis renvoie la derniere."""
        start = self._img_count
        t0 = time.monotonic()
        while self._img_count < start + drop and time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self._img

    # --------------------------------------------------------- projection (voie B)
    def project_bbox(self, model, xyz, rpy):
        """Boite 2D (umin, vmin, umax, vmax) de l'objet, ou None si hors-champ.

        8 coins en repere objet -> monde -> repere optique top-down :
            x_c = X ; y_c = -Y ; z_c = CAM_Z - Z
            u = fx*x_c/z_c + cx ; v = fy*y_c/z_c + cy
        """
        hx, hy, hz = EXTENTS.get(model, (0.03, 0.03, 0.03))
        rot = rpy_to_matrix(*rpy)
        cx0, cy0, cz0 = xyz
        us, vs = [], []
        for sx in (-hx, hx):
            for sy in (-hy, hy):
                for sz in (-hz, hz):
                    wx, wy, wz = rot @ np.array([sx, sy, sz])
                    X, Y, Z = cx0 + wx, cy0 + wy, cz0 + wz
                    zc = CAM_Z - Z
                    if zc <= 1e-3:        # derriere la camera : on ignore le coin
                        continue
                    us.append(self.fx * X / zc + self.cx)
                    vs.append(self.fy * (-Y) / zc + self.cy)
        if not us:
            return None
        umin, umax = max(0.0, min(us)), min(self.width, max(us))
        vmin, vmax = max(0.0, min(vs)), min(self.height, max(vs))
        if umax - umin < self.args.min_bbox_px or vmax - vmin < self.args.min_bbox_px:
            return None
        return umin, vmin, umax, vmax

    def to_yolo(self, box):
        """(umin,vmin,umax,vmax) px -> (cx,cy,w,h) normalises."""
        umin, vmin, umax, vmax = box
        return ((umin + umax) / 2 / self.width,
                (vmin + vmax) / 2 / self.height,
                (umax - umin) / self.width,
                (vmax - vmin) / self.height)

    def image_to_bgr(self):
        """Dernier message Image -> numpy BGR pour cv2.imwrite."""
        msg = self._img
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding.startswith('rgb'):
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        return arr

    def randomize_light(self):
        """Best-effort : varie l'intensite/teinte du soleil via gz light_config."""
        d = round(random.uniform(0.5, 0.9), 2)
        tint = round(random.uniform(-0.08, 0.08), 2)
        r, g, b = d, max(0.0, d + tint), max(0.0, d - tint)
        req = (f'name: "sun", type: DIRECTIONAL, cast_shadows: true, '
               f'direction: {{x: -0.2, y: 0.1, z: -1.0}}, '
               f'diffuse: {{r: {r}, g: {g}, b: {b}, a: 1.0}}, '
               f'specular: {{r: 0.1, g: 0.1, b: 0.1, a: 1.0}}')
        try:
            subprocess.run(
                ['gz', 'service', '-s', f'/world/{self.world}/light_config',
                 '--reqtype', 'gz.msgs.Light', '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '1500', '--req', req],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=3)
        except Exception:
            pass  # non bloquant : light_config peut etre indisponible

    # ----------------------------------------------------------------- run
    def run(self):
        a = self.args
        classes = a.classes
        out = os.path.expanduser(a.out)
        for sub in ('images/train', 'images/val', 'labels/train', 'labels/val'):
            os.makedirs(os.path.join(out, sub), exist_ok=True)

        if not self.wait_services():
            return False
        self.wait_camera_info()

        total = len(classes) * a.per_class
        done, kept = 0, 0
        names = ['ds_object'] + [f'ds_distractor_{i}' for i in range(2)]

        for class_id, entry in enumerate(classes):
            for _ in range(a.per_class):
                done += 1
                # 1) table rase : on supprime tout objet de l'iteration precedente.
                for nm in names:
                    self.delete(nm)

                # 2) objet principal de la classe courante.
                model = self.resolve_model(entry)
                sdf, _ = self.build_sdf(model)
                if sdf is None:
                    continue
                xyz, rpy = self.random_pose()
                placed = [('ds_object', model, xyz, rpy, class_id)]
                self.spawn('ds_object', sdf, xyz, rpy)

                # 3) distracteurs optionnels (autres classes, eux aussi labellises).
                ndist = random.randint(0, a.distractors) if a.distractors else 0
                for i in range(ndist):
                    dcid = random.randrange(len(classes))
                    dmodel = self.resolve_model(classes[dcid])
                    dsdf, _ = self.build_sdf(dmodel)
                    if dsdf is None:
                        continue
                    dxyz, drpy = self.random_pose()
                    nm = f'ds_distractor_{i}'
                    self.spawn(nm, dsdf, dxyz, drpy)
                    placed.append((nm, dmodel, dxyz, drpy, dcid))

                # 4) eclairage optionnel + attente d'une frame fraiche.
                if a.randomize_light:
                    self.randomize_light()
                time.sleep(a.settle)
                if self.fresh_frame() is None:
                    self.get_logger().warn(f"[{done}/{total}] pas d'image, saute.")
                    continue

                # 5) labels : projection de chaque objet, filtrage hors-champ/petit.
                lines = []
                for _nm, mdl, oxyz, orpy, cid in placed:
                    box = self.project_bbox(mdl, oxyz, orpy)
                    if box is None:
                        continue
                    cxn, cyn, wn, hn = self.to_yolo(box)
                    lines.append(f"{cid} {cxn:.6f} {cyn:.6f} {wn:.6f} {hn:.6f}")
                if not lines:
                    self.get_logger().warn(f"[{done}/{total}] aucune bbox valide, saute.")
                    continue

                # 6) ecriture image + label (split train/val).
                split = 'val' if random.random() < a.val_split else 'train'
                stem = f"{entry}_{done:06d}"
                cv2.imwrite(os.path.join(out, 'images', split, stem + '.png'),
                            self.image_to_bgr())
                with open(os.path.join(out, 'labels', split, stem + '.txt'), 'w') as f:
                    f.write('\n'.join(lines) + '\n')
                kept += 1
                if done % 25 == 0 or done == total:
                    self.get_logger().info(f"[{done}/{total}] gardes={kept}")

        # nettoyage final + data.yaml.
        for nm in names:
            self.delete(nm)
        self.write_data_yaml(out, classes)
        self.get_logger().info(f"Termine : {kept}/{total} images dans {out}")
        return True

    def write_data_yaml(self, out, classes):
        names = '[' + ', '.join(classes) + ']'
        content = (
            f"# Genere par capture_dataset.py (bootcamp_vision)\n"
            f"path: {out}\n"
            f"train: images/train\n"
            f"val: images/val\n"
            f"nc: {len(classes)}\n"
            f"names: {names}\n")
        with open(os.path.join(out, 'data.yaml'), 'w') as f:
            f.write(content)


def parse_args(argv):
    p = argparse.ArgumentParser(
        description="Generation d'un dataset YOLO synthetique auto-labellise.")
    p.add_argument('--classes', nargs='+', required=True,
                   help="presets/modeles, une classe par entree (class_id = index)")
    p.add_argument('--per-class', type=int, default=300, help="images par classe")
    p.add_argument('--out', default='~/yolo_ds', help="dossier de sortie")
    p.add_argument('--val-split', type=float, default=0.2, help="proportion validation")
    p.add_argument('--seed', type=int, default=0, help="graine (reproductibilite)")
    p.add_argument('--distractors', type=int, default=0,
                   help="nb max de distracteurs (0-2) ajoutes par image")
    p.add_argument('--randomize-light', action='store_true',
                   help="varier l'eclairage (best-effort gz light_config)")
    p.add_argument('--tilt', action='store_true',
                   help="ajouter un leger roll/pitch aleatoire")
    p.add_argument('--world', default='vision_table', help="nom du monde gz")
    p.add_argument('--table-bounds', type=float, default=0.30,
                   help="demi-etendue x/y de spawn (m)")
    p.add_argument('--table-z', type=float, default=0.46, help="hauteur de spawn (m)")
    p.add_argument('--settle', type=float, default=0.4,
                   help="attente physique apres spawn (s)")
    p.add_argument('--min-bbox-px', type=float, default=8.0,
                   help="taille minimale d'une bbox conservee (px)")
    # On ignore les --ros-args injectes par ros2 run.
    known, _ = p.parse_known_args(argv)
    return known


def main():
    args = parse_args(sys.argv[1:])
    args.distractors = max(0, min(2, args.distractors))
    random.seed(args.seed)
    np.random.seed(args.seed)
    rclpy.init()
    node = DatasetCapturer(args)
    try:
        ok = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
