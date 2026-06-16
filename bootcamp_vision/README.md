# bootcamp_vision — Monde « vision » du Jour 4 (perception)

Monde Gazebo Ionic (gz-sim 9) **léger** : une **caméra fixe** filme une **table** en
**vue du dessus** et publie son flux côté ROS 2. Aucun bras ni base mobile. Sert aux 4 TP
de vision (Formes / ArUco / YOLO / Chiffre) et fournit une bibliothèque d'objets
réutilisable aussi au Jour 5.

## Lancer

```bash
cd ~/ros2_bootcamp_ws && colcon build --packages-select bootcamp_vision && source install/setup.bash

# Monde + caméra (GUI)
ros2 launch bootcamp_vision vision_world.launch.py
# ... ou sans GUI : headless:=true
# ... avec un objet posé d'emblée : object:=aruco aruco_id:=1   (ou object:=shapes, digit, coffee, ...)
```

## Vérifier

```bash
ros2 topic list | grep camera         # /camera/image_raw  et  /camera/camera_info
ros2 topic echo /camera/camera_info --once
ros2 run rqt_image_view rqt_image_view /camera/image_raw   # ou rviz2
```

## Caméra & intrinsèques

| Paramètre | Valeur |
| --- | --- |
| Résolution | **1280 × 720**, `R8G8B8`, ~30 Hz |
| `horizontal_fov` | **1.2 rad** (≈ 68.75°) |
| fx = fy | (W/2)/tan(fov/2) = 640/tan(0.6) ≈ **935.5** |
| cx | W/2 = **640.0** |
| cy | H/2 = **360.0** |
| Topic image | `/camera/image_raw` (`sensor_msgs/Image`) |
| Topic info | `/camera/camera_info` (`sensor_msgs/CameraInfo`, `frame_id = camera_optical_frame`) |

> Les valeurs exactes sont publiées par Gazebo : `ros2 topic echo /camera/camera_info --once`
> (champ `k = [fx 0 cx  0 fy cy  0 0 1]`). Régler la résolution / le FOV dans
> `worlds/vision_table.sdf` (capteur `vision_camera`).

## Repères (TF) & transform table ↔ caméra

Publiés par `vision_world.launch.py` (`static_transform_publisher`) :

```
world ──(0,0,0.40)──▶ table_link            # centre du plateau (dessus à z=0.40)
world ──(0,0,1.40)──▶ camera_link           # boîtier caméra, 1.00 m au-dessus du plateau
camera_link ──rpy(π,0,0)──▶ camera_optical_frame   # REP-103 : x→droite, y→bas, z→avant (vers la table)
```

La caméra regarde **−Z** (vers la table). Hauteur caméra→table : **h = 1.00 m**.

## Back-projection pixel → plan de la table (vue du dessus)

Pour un pixel `(u, v)` et la table au plan `z = 0.40` (centre table = origine monde) :

```
X_table = (u - cx) / fx * h          #  h = 1.00 m
Y_table = -(v - cy) / fy * h
Z_table = 0.40
```

(Le signe `−` sur Y vient de l'orientation optique vue du dessus, cf. TF ci-dessus.)
C'est cohérent avec le helper `backproject_to_plane` des TP.

## Bibliothèque d'objets (`models/`)

Objets spawnés à la demande (services Gazebo Ionic `SpawnEntity`/`DeleteEntity`, **pas**
`spawn_entity.py`). `object` accepte un **nom de modèle** ou un **alias de catégorie** :

| Alias | Modèles | TP / usage |
| --- | --- | --- |
| `shapes` | `shape_star` (rouge), `shape_cube` (bleu), `shape_cylinder` (vert) | TP01 OpenCV/HSV |
| `aruco` | `aruco_parcel` (marqueur **DICT_4X4_50**, **taille 0.05 m**) | TP02 solvePnP |
| `yolo` | `trash_bottle` (COCO `bottle`) + variante Fuel commentée | TP03 YOLO |
| `digit` | `digit_panel` (chiffre noir/fond blanc, MNIST) | TP04 CNN |
| `coffee` | `coffee_pack_1/2/3` (paquets café, labels distincts) | classification |
| `trash` | `trash_can` (metal), `trash_bottle` (plastic), `trash_carton` (paper) | tri / Jour 5 |

```bash
# Monde lancé dans un terminal, puis :
ros2 launch bootcamp_vision spawn_object.launch.py object:=shapes
ros2 launch bootcamp_vision spawn_object.launch.py object:=aruco aruco_id:=1
ros2 launch bootcamp_vision spawn_object.launch.py object:=digit digit:=7
ros2 launch bootcamp_vision spawn_object.launch.py object:=coffee_pack_2
ros2 launch bootcamp_vision spawn_object.launch.py object:=trash count:=3 jitter:=0.15   # plusieurs, dispersés
ros2 launch bootcamp_vision spawn_object.launch.py action:=respawn name:=vision_object_0
```

### Paramètres du spawner (`spawn_object.py`)

`object`, `aruco_id`, `digit`, `class_id`, `pose` (`"x y z [yaw]"`, vide = centre table),
`table_xy`, `table_z` (défaut 0.46, l'objet retombe sur le plateau), `jitter` (dispersion
aléatoire), `count`, `name`, `action` (`spawn`/`delete`/`respawn`).

## Régénérer les textures

```bash
ros2 run bootcamp_vision generate_aruco.py --id 5      # marqueur ArUco
ros2 run bootcamp_vision generate_digit.py --digit 8   # chiffre MNIST
ros2 run bootcamp_vision generate_label.py             # labels café (1/2/3)
```

## Variantes (dans `worlds/vision_table.sdf`, en commentaire)

- **Vue inclinée 45°** : caméra reculée + inclinée (la back-projection nécessite alors la
  pose complète caméra↔table).
- **`rgbd_camera`** : ajoute la profondeur (`/camera/depth_image`, `/camera/points`) pour une
  vraie pose 3D sans plan connu — adapter alors `config/bridge_camera.yaml`.
