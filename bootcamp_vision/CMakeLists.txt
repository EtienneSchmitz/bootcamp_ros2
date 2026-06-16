cmake_minimum_required(VERSION 3.10...3.27)
project(bootcamp_vision)

find_package(ament_cmake REQUIRED)

# Monde, modeles, configs de pont et launch files.
install(
  DIRECTORY worlds models config launch
  DESTINATION share/${PROJECT_NAME}
)

# Spawner d'objets (Jour 4) + generateurs de textures (aruco / chiffre / labels).
# On conserve l'extension .py : appel via `ros2 run bootcamp_vision spawn_object.py`.
install(
  PROGRAMS
    scripts/spawn_object.py
    scripts/generate_aruco.py
    scripts/generate_digit.py
    scripts/generate_label.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
