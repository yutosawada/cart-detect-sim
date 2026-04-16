cmake_minimum_required(VERSION 3.8)
project(cart_detect_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cart_detect REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom service interfaces (SetCart, RemoveCart)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetCart.srv"
  "srv/RemoveCart.srv"
  DEPENDENCIES geometry_msgs
)

add_executable(cart_detect_sim_node
  src/cart_detect_sim_node.cpp
  src/main.cpp
)
ament_target_dependencies(cart_detect_sim_node
  rclcpp
  cart_detect
  std_srvs
  geometry_msgs
  tf2
  tf2_ros
)
target_include_directories(cart_detect_sim_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Link generated interfaces to the executable
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(cart_detect_sim_node "${cpp_typesupport_target}")

install(TARGETS cart_detect_sim_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
