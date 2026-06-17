cmake_minimum_required(VERSION 3.10...3.27)
project(bootcamp_vision)

find_package(ament_cmake REQUIRED)

# --- Interfaces de detection (TP03 YOLO) : types de REFERENCE Detection /
#     DetectionArray. Optionnel pour la sim, mais pratique pour les TP qui
#     publient des detections (cf. encart README, placeholder mon_projet_interfaces). ---
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# Monde, modeles, configs de pont et launch files.
install(
  DIRECTORY worlds models config launch
  DESTINATION share/${PROJECT_NAME}
)

# Spawner d'objets (Jour 4) + generateurs de textures (aruco / chiffre / labels)
# + generateur de dataset YOLO (capture_dataset.py).
# On conserve l'extension .py : appel via `ros2 run bootcamp_vision spawn_object.py`.
install(
  PROGRAMS
    scripts/spawn_object.py
    scripts/generate_aruco.py
    scripts/generate_digit.py
    scripts/generate_label.py
    scripts/capture_dataset.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
