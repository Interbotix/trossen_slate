cmake_minimum_required(VERSION 3.15.0)
project(trossen_slate VERSION 1.0.0)

# Set the C++ standard to 17 if not already set
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Add compiler options for GCC and Clang
if(CMAKE_COMPILER_IS_GNUXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set the build type to Release
set(CMAKE_BUILD_TYPE "Release")

# Define the serial driver library name
set(serial_driver "chassis_driver")

# Set system architecture based on the processor type
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
  set(ARCH "x86_64")
elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
  set(ARCH "aarch64")
else()
  message(FATAL_ERROR "Unknown System Architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

# Find the ament_cmake package if not building with scikit-build
if(NOT DEFINED SKBUILD)
  find_package(ament_cmake REQUIRED)
endif()

# Include the version generation script
include(cmake/Version.cmake)

# Include directories
include_directories(include ${CMAKE_CURRENT_BINARY_DIR}/include)

# Add the library
add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}.cpp
  src/base_driver.cpp
)

# Specify the include directories for the library
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Set properties for the library
set_target_properties(${PROJECT_NAME} PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  OUTPUT_NAME ${PROJECT_NAME}
)

# Link the serial driver library
target_link_libraries(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/lib/${ARCH}/libchassis_driver.so)

# Add the demo executables
add_executable(advanced_demo demo/advanced_demo.cpp)
target_link_libraries(advanced_demo PRIVATE ${PROJECT_NAME})

add_executable(basic_demo demo/basic_demo.cpp)
target_link_libraries(basic_demo PRIVATE ${PROJECT_NAME})

# Install the library headers
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

# Install the library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT "export_${PROJECT_NAME}"
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the version output file
install(
  FILES
    ${PATH_TO_VERSION_OUTPUT}
  DESTINATION
    include/${PROJECT_NAME}
)

# Install the package.xml file
install(
  FILES
    package.xml
  DESTINATION
    share/${PROJECT_NAME}
)

# Install the demo executables
install(TARGETS advanced_demo basic_demo
  RUNTIME DESTINATION bin
)

# Install the serial driver library
install(
  FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/${ARCH}/lib${serial_driver}.so
  DESTINATION
    lib
)

# If building with scikit-build, add Python bindings
if(DEFINED SKBUILD)
  # Enable finding Python
  set(PYBIND11_FINDPYTHON ON)
  find_package(pybind11 CONFIG REQUIRED)

  # Add the Python module
  pybind11_add_module(${SKBUILD_PROJECT_NAME} MODULE python/${SKBUILD_PROJECT_NAME}.cpp)
  target_link_libraries(${SKBUILD_PROJECT_NAME} PRIVATE ${PROJECT_NAME})

  # Check if pybind11-stubgen is installed
  find_program(PYBIND11_STUBGEN pybind11-stubgen)
  if(PYBIND11_STUBGEN)
    # Generate the stub files
    add_custom_command(
      TARGET ${SKBUILD_PROJECT_NAME}
      POST_BUILD
      COMMAND mkdir ${SKBUILD_PROJECT_NAME}
      COMMAND mv ${SKBUILD_PROJECT_NAME}.cpython-*.so ${SKBUILD_PROJECT_NAME}
      COMMAND cp ${CMAKE_SOURCE_DIR}/python/${SKBUILD_PROJECT_NAME}/__init__.py ${CMAKE_BINARY_DIR}/${SKBUILD_PROJECT_NAME}
      COMMAND export PYTHONPATH=${CMAKE_BINARY_DIR}
      COMMAND ${PYBIND11_STUBGEN} ${SKBUILD_PROJECT_NAME}
      COMMAND mv ${SKBUILD_PROJECT_NAME}/${SKBUILD_PROJECT_NAME}.cpython-*.so ${CMAKE_BINARY_DIR}
    )
    # Install the stub files
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

# If not building with scikit-build, export the ament package
if(NOT DEFINED SKBUILD)
  ament_export_targets("export_${PROJECT_NAME}")
  ament_package()
endif()
