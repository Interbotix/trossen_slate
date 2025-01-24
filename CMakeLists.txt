cmake_minimum_required(VERSION 3.15.0)
project(trossen_slate VERSION 0.0.1)

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
target_link_libraries(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/lib/${ARCH}/libchassis_driver.so)

# Add the demo executables
add_executable(advanced_demo demo/advanced_demo.cpp)
target_link_libraries(advanced_demo PRIVATE ${PROJECT_NAME})

add_executable(basic_demo demo/basic_demo.cpp)
target_link_libraries(basic_demo PRIVATE ${PROJECT_NAME})

# Install the library and executables
install(DIRECTORY include/trossen_slate DESTINATION ${PROJECT_BINARY_DIR}/include)
install(TARGETS ${PROJECT_NAME} advanced_demo basic_demo
  ARCHIVE DESTINATION ${PROJECT_BINARY_DIR}/lib
  RUNTIME DESTINATION ${PROJECT_BINARY_DIR}/bin
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
