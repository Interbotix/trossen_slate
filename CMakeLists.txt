cmake_minimum_required(VERSION 3.10.0)
project(trossen_slate)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE "Release")

set(serial_driver "chassis_driver")

include_directories(include)

if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
  set(ARCH "x86_64")
elseif(${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
  set(ARCH "aarch64")
else()
  message(FATAL_ERROR "Unknown System Architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/${ARCH})

add_library(slate_base
  src/slate_base.cpp
  src/base_driver.cpp
)

target_link_libraries(slate_base
  ${serial_driver}
)

add_executable(test
  src/test.cpp
)

target_link_libraries(test
  slate_base
)

install(
  TARGETS slate_base
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    test
  RUNTIME DESTINATION
    lib/${PROJECT_NAME}
)

install(
  FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/${ARCH}/lib${serial_driver}.so
  DESTINATION
    lib
)
