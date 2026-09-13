cmake_minimum_required(VERSION 3.8)
project(carbot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_program(XACRO_EXECUTABLE xacro REQUIRED)

  set(GENERATED_URDF ${CMAKE_CURRENT_BINARY_DIR}/generated_carbot.urdf)
  execute_process(
    COMMAND ${XACRO_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/urdf/carbot.urdf.xacro
      parameters_file:=${CMAKE_CURRENT_SOURCE_DIR}/config/carbot_parameters.yaml
    OUTPUT_FILE ${GENERATED_URDF}
    RESULT_VARIABLE XACRO_RESULT
  )
  if(NOT XACRO_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed to expand the Carbot Xacro")
  endif()

  ament_add_pytest_test(test_carbot_parameters
    test/test_carbot_parameters.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(test_carbot_description
    test/test_carbot_description.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    ENV CARBOT_GENERATED_URDF=${GENERATED_URDF}
  )
endif()

ament_package()
