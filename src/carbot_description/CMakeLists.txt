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
  add_custom_command(
    OUTPUT ${GENERATED_URDF}
    COMMAND ${XACRO_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/urdf/carbot.urdf.xacro
      parameters_file:=${CMAKE_CURRENT_SOURCE_DIR}/config/carbot_parameters.yaml
      -o ${GENERATED_URDF}
    DEPENDS
      ${CMAKE_CURRENT_SOURCE_DIR}/urdf/carbot.urdf.xacro
      ${CMAKE_CURRENT_SOURCE_DIR}/config/carbot_parameters.yaml
    COMMENT "Expanding the Carbot Xacro used by description tests"
    VERBATIM
  )
  add_custom_target(generate_carbot_test_urdf ALL DEPENDS ${GENERATED_URDF})

  ament_add_pytest_test(test_carbot_parameters
    test/test_carbot_parameters.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(test_carbot_description
    test/test_carbot_description.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    ENV CARBOT_GENERATED_URDF=${GENERATED_URDF}
  )
  ament_add_pytest_test(test_carbot_control
    test/test_carbot_control.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(test_carbot_mid360
    test/test_carbot_mid360.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
