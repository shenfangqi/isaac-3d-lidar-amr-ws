cmake_minimum_required(VERSION 3.8)
project(carbot_hardware_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(mid360_imu_adapter src/mid360_imu_adapter.cpp)
target_include_directories(mid360_imu_adapter PRIVATE include)
target_compile_features(mid360_imu_adapter PRIVATE cxx_std_17)
ament_target_dependencies(mid360_imu_adapter rclcpp sensor_msgs nav_msgs)

install(TARGETS mid360_imu_adapter DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_imu_units test/test_imu_units.cpp)
  target_include_directories(test_imu_units PRIVATE include)
  target_compile_features(test_imu_units PRIVATE cxx_std_17)
endif()

ament_package()
