cmake_minimum_required(VERSION 3.8)
project(simple_p3at_driver)

# 1. Encontrar pacotes ROS
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 2. Configurar a libAria (Localizada em /usr/local)
include_directories(/usr/local/include/Aria)
include_directories(/usr/include/Aria)
link_directories(/usr/local/lib)

# 3. Criar o Executável
add_executable(simple_p3at_node src/simple_p3at.cpp)

# 4. Linkar dependências do ROS (O SEGREDO ESTÁ AQUI)
# Isso garante que o rclcpp.hpp seja encontrado
ament_target_dependencies(simple_p3at_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
)

# 5. Linkar a libAria e bibliotecas do sistema manualmente
target_link_libraries(simple_p3at_node
  Aria
  dl
  m
  rt
  pthread
)

# 6. Instalação
install(TARGETS simple_p3at_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
