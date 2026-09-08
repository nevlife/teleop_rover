cmake_minimum_required(VERSION 3.16)
project(teleop_rover_v2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(PkgConfig QUIET)

add_library(teleop_control_guard src/control_guard.cpp)
target_include_directories(teleop_control_guard PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(teleop_control_guard PRIVATE -Wall -Wextra -Wpedantic)

# Media support is optional at configure time so the safety library can still
# be built and tested on a ROS-only CI runner. Production rover images must
# install the GStreamer 1.24 development packages and will get this probe.
if(PkgConfig_FOUND)
  pkg_check_modules(GSTREAMER QUIET IMPORTED_TARGET gstreamer-1.0>=1.24)
  if(GSTREAMER_FOUND)
    add_executable(teleop-media-probe src/media_probe.cpp)
    target_link_libraries(teleop-media-probe PRIVATE PkgConfig::GSTREAMER)
    target_compile_options(teleop-media-probe PRIVATE -Wall -Wextra -Wpedantic)
    install(TARGETS teleop-media-probe RUNTIME DESTINATION lib/${PROJECT_NAME})
  endif()
endif()

install(TARGETS teleop_control_guard EXPORT teleop_rover_v2Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(control_guard_test test/control_guard_test.cpp)
  target_link_libraries(control_guard_test teleop_control_guard)
endif()

ament_export_targets(teleop_rover_v2Targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
