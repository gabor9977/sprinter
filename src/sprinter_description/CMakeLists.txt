
cmake_minimum_required(VERSION 3.8)
project(sprinter_description)

# Compiler-Warnungen aktivieren
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Abhängigkeiten
find_package(ament_cmake REQUIRED)

# Installiere alle relevanten Ressourcen
install(
  DIRECTORY urdf launch meshes config
  DESTINATION share/${PROJECT_NAME}
)

# Optional: Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
