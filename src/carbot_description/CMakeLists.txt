cmake_minimum_required(VERSION 3.8)
project(carbot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_carbot_parameters
    test/test_carbot_parameters.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
