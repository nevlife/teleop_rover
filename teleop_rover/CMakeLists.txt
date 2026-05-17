cmake_minimum_required(VERSION 3.8)
project(teleop_rover)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# teleop_rover is the telemetry/control plane bot package. It contains ONLY the
# Python net_bridge node and its msgs (via teleop_rover_msgs). It does NOT depend
# on stream_rover — the video plane lives in a separate package and runs against
# its own Zenoh router. ament_cmake is sufficient (no C++ targets).
find_package(ament_cmake REQUIRED)

# Install the Python entry point and the supporting net_bridge module.
install(PROGRAMS scripts/net_bridge.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY scripts/net_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# Vendored teleop_contracts (git submodule at <repo>/teleop_contracts).
# Install the inner package directory next to net_bridge so that the entry
# point's sys.path entry (which includes lib/teleop_rover) can resolve
# `import teleop_contracts` at runtime.
install(DIRECTORY ../teleop_contracts/teleop_contracts
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
