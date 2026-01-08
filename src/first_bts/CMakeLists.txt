cmake_minimum_required(VERSION 3.8)
project(first_bts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)

add_executable(sequence_tree src/Sequence.cpp)
ament_target_dependencies(sequence_tree rclcpp std_msgs behaviortree_cpp)

add_executable(number_checker src/number_checker.cpp)
ament_target_dependencies(number_checker
  rclcpp
  std_msgs behaviortree_cpp 
)

install(TARGETS number_checker RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS sequence_tree RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch trees DESTINATION share/${PROJECT_NAME})

ament_package()
