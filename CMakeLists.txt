cmake_minimum_required(VERSION 3.15.0)
project(trossen_slate VERSION 1.0.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE "Release")

# Set system architecture
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
  set(ARCH "x86_64")
elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
  set(ARCH "aarch64")
else()
  message(FATAL_ERROR "Unknown System Architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# Check if being built with colcon
if(DEFINED ENV{COLCON_CURRENT_PREFIX})
  # Define ROS2 macro
  add_definitions(-DROS2)

  # ROS2 specific setup
  find_package(geometry_msgs REQUIRED)
  find_package(trossen_slate_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)

  set(ROS_DEPENDENCIES
    geometry_msgs
    trossen_slate_msgs
    nav_msgs
    rclcpp
    sensor_msgs
    std_msgs
    std_srvs
    tf2_geometry_msgs
    tf2_ros
  )

  ament_target_dependencies(${PROJECT_NAME} ${ROS_DEPENDENCIES})

  # Add the SLATE node executable
  add_executable(trossen_slate_node
    src/trossen_slate_node.cpp
  )
  target_link_libraries(trossen_slate_node PRIVATE ${PROJECT_NAME})

  # Set rpath to include the directory where the shared library is installed
  set_target_properties(trossen_slate_node PROPERTIES
    INSTALL_RPATH "\$ORIGIN/../lib"
    BUILD_WITH_INSTALL_RPATH TRUE
  )

  # Install ROS node
  install(
    TARGETS
      trossen_slate_node
    RUNTIME DESTINATION
      lib/${PROJECT_NAME}
  )

  ament_package()
endif()

# Add the library
add_library(${PROJECT_NAME} STATIC
  src/trossen_slate.cpp
  src/base_driver.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC include ${PROJECT_BINARY_DIR}/include)

# Set the library to be position independent so that it can be linked to a shared library
set_target_properties(${PROJECT_NAME} PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  OUTPUT_NAME trossen_slate
)

# Link the serial driver library
target_link_libraries(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/lib/${ARCH}/libchassis_driver.so)

# Add the demo executables
add_executable(advanced_demo demo/advanced_demo.cpp)
target_link_libraries(advanced_demo PRIVATE ${PROJECT_NAME})

add_executable(basic_demo demo/basic_demo.cpp)
target_link_libraries(basic_demo PRIVATE ${PROJECT_NAME})

# Set rpath to include the directory where the shared library is installed
set_target_properties(advanced_demo PROPERTIES
  INSTALL_RPATH "\$ORIGIN/../lib"
  BUILD_WITH_INSTALL_RPATH TRUE
)

set_target_properties(basic_demo PROPERTIES
  INSTALL_RPATH "\$ORIGIN/../lib"
  BUILD_WITH_INSTALL_RPATH TRUE
)

# Install the library and executables
install(DIRECTORY include/trossen_slate DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
install(TARGETS ${PROJECT_NAME} advanced_demo basic_demo
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
)

# Install the shared library
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib/${ARCH}/libchassis_driver.so
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)

if(DEFINED SKBUILD)
  # Add the Python bindings library
  set(PYBIND11_FINDPYTHON ON)
  find_package(pybind11 CONFIG REQUIRED)
  pybind11_add_module(${SKBUILD_PROJECT_NAME} MODULE python/${SKBUILD_PROJECT_NAME}.cpp)
  target_link_libraries(${SKBUILD_PROJECT_NAME} PRIVATE ${PROJECT_NAME})

  # Check if pybind11-stubgen is installed
  find_program(PYBIND11_STUBGEN pybind11-stubgen)
  if(PYBIND11_STUBGEN)
    # Generate the stub files
    add_custom_command(
      TARGET ${SKBUILD_PROJECT_NAME}
      POST_BUILD
      # Create a directory as a temporary package at ${CMAKE_BINARY_DIR}
      COMMAND mkdir ${SKBUILD_PROJECT_NAME}
      # Move the shared object file to the temporary package
      COMMAND mv ${SKBUILD_PROJECT_NAME}.cpython-*.so ${SKBUILD_PROJECT_NAME}
      # Copy the __init__.py file to the temporary package
      COMMAND cp ${CMAKE_SOURCE_DIR}/python/${SKBUILD_PROJECT_NAME}/__init__.py
        ${CMAKE_BINARY_DIR}/${SKBUILD_PROJECT_NAME}
      # Add ${CMAKE_BINARY_DIR} to the PYTHONPATH to make the temporary package importable
      COMMAND export PYTHONPATH=${CMAKE_BINARY_DIR}
      # Generate the stub files at ${CMAKE_BINARY_DIR}/stubs
      COMMAND ${PYBIND11_STUBGEN} ${SKBUILD_PROJECT_NAME}
      # Move the shared object file back to ${CMAKE_BINARY_DIR} so that it can be installed
      COMMAND mv ${SKBUILD_PROJECT_NAME}/${SKBUILD_PROJECT_NAME}.cpython-*.so ${CMAKE_BINARY_DIR}
    )
    # Install the stub files located in ${CMAKE_BINARY_DIR}/stubs/${SKBUILD_PROJECT_NAME}/
    install(
      DIRECTORY ${CMAKE_BINARY_DIR}/stubs/${SKBUILD_PROJECT_NAME}/
      DESTINATION ./${SKBUILD_PROJECT_NAME}
    )
  endif()

  # Install the Python module
  install(
    TARGETS ${SKBUILD_PROJECT_NAME}
    LIBRARY DESTINATION ./${SKBUILD_PROJECT_NAME}
  )
endif()
