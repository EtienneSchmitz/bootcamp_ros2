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
world ──(0,0,0.95)──▶ camera_link           # boîtier caméra, 0.55 m au-dessus du plateau
camera_link ──rpy(π,0,0)──▶ camera_optical_frame   # REP-103 : x→droite, y→bas, z→avant (vers la table)
```

La caméra regarde **−Z** (vers la table). Hauteur caméra→table : **h = 0.55 m**
(caméra z=0.95 − plateau z=0.40). Distance choisie pour rapprocher l'échelle des objets
de la caméra eye-in-hand du SO-101 (Jour 5) : un marqueur ArUco de 5 cm fait ≈ 85 px
(≈ 14 px/module) → détection robuste ; champ visible ≈ 0.75 × 0.42 m (objets au centre).

## Back-projection pixel → plan de la table (vue du dessus)

Pour un pixel `(u, v)` et la table au plan `z = 0.40` (centre table = origine monde) :

```
X_table = (u - cx) / fx * h          #  h = 0.55 m
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

## Générer un dataset YOLO (TP03 — entraînement custom)

`capture_dataset.py` produit, par **domain randomization de l'objet**, un dataset
**auto-labellisé** au format Ultralytics — sans annotation manuelle. Méthode : on
**choisit** la pose de spawn et on **connaît** la taille de chaque modèle, donc on
**projette** la boîte 3D via les intrinsèques top-down → boîte 2D exacte (voie B ;
voie A `boundingbox_camera` esquissée en commentaire dans le script).

```bash
# Terminal 1 — monde + caméra, sans GUI :
ros2 launch bootcamp_vision vision_world.launch.py headless:=true
# Terminal 2 — pont des SERVICES de spawn (obligatoire pour la capture) :
ros2 launch bootcamp_vision spawn_object.launch.py object:=''
# Terminal 3 — capture :
ros2 run bootcamp_vision capture_dataset.py \
    --classes coffee_pack_1 coffee_pack_2 coffee_pack_3 trash_bottle trash_can \
    --per-class 300 --out ~/yolo_ds --val-split 0.2 --seed 42
```

Sortie : `~/yolo_ds/images/{train,val}/`, `labels/{train,val}/` (lignes
`class_id cx cy w h` normalisées) et `data.yaml`. Chaque entrée de `--classes` = une
classe (`class_id` = son rang). Reproductible via `--seed`.

| Option | Défaut | Rôle |
| --- | --- | --- |
| `--per-class N` | 300 | images par classe |
| `--out DIR` | `~/yolo_ds` | dossier de sortie |
| `--val-split F` | 0.2 | proportion en validation |
| `--seed N` | 0 | graine (reproductibilité) |
| `--distractors 0..2` | 0 | objets parasites (autres classes), aussi labellisés |
| `--tilt` | off | léger roll/pitch aléatoire en plus du yaw |
| `--randomize-light` | off | varier l'éclairage (best-effort `gz light_config`) |
| `--table-bounds M` | 0.30 | demi-étendue x/y de spawn (m) |
| `--settle S` | 0.4 | attente physique après spawn (s) |
| `--min-bbox-px PX` | 8 | rejette les boîtes trop petites / hors-champ |

### Entraîner puis inférer

```bash
pip install ultralytics            # non packagé sous apt
yolo detect train model=yolo11n.pt data=~/yolo_ds/data.yaml epochs=30 imgsz=640
yolo detect predict model=runs/detect/train/weights/best.pt source=~/yolo_ds/images/val
```

**Budget temps** (indicatif) : génération ≈ ~1 s/image (≈ 25 min pour 5 classes × 300).
Entraînement 30 epochs sur ~1500 images : **quelques minutes sur GPU**, **beaucoup plus
long sur CPU** (réduire `--per-class`/`epochs`/`imgsz` pour tenir dans les 8 h du TP).
Le preset **COCO** `yolo11n.pt` reste le **fallback** (détecte déjà `bottle`, etc.).

## Types `Detection` / `DetectionArray` (référence TP)

`bootcamp_vision` génère deux messages de **référence** pour publier des détections
(YOLO → table) : `bootcamp_vision/msg/Detection` et `DetectionArray`.

```python
from bootcamp_vision.msg import Detection, DetectionArray
```

| `Detection` | Champ | Sens |
| --- | --- | --- |
| classe | `class_name` (string), `class_id` (int32), `score` (float32) | sortie YOLO |
| boîte 2D px | `u`, `v`, `w`, `h` (float32) | centre + taille image |
| table | `position` (`geometry_msgs/Point`) | back-projection sur z=0.40 (optionnel) |

`DetectionArray` = `std_msgs/Header header` + `Detection[] detections`.

> **Encart — placeholder `mon_projet_interfaces`.** Les énoncés des TP importent
> `mon_projet_interfaces.msg`. Ce paquet n'existe pas ici : c'est **ton** futur paquet
> d'interfaces. En attendant, utilise les types ci-dessus comme modèle (mêmes champs),
> ou recopie `msg/Detection.msg` dans ton paquet et remplace l'import. La back-projection
> `(u,v) → position` utilise **h = 0.55 m** (caméra z=0.95 − table z=0.40), cf. section
> ci-dessus.

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
