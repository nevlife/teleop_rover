cmake_minimum_required(VERSION 3.16)
project(teleop_rover_v2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(PkgConfig QUIET)

find_package(yaml-cpp REQUIRED)
find_package(Protobuf QUIET)

# The v2 wire contract lives in the teleop_contracts submodule; the C++
# classes are generated at build time rather than checked in, so the schema
# has exactly one source of truth.
set(TELEOP_PROTO_ROOT
  ${CMAKE_CURRENT_SOURCE_DIR}/../teleop_contracts/teleop_contracts/schema)
if(Protobuf_FOUND AND EXISTS ${TELEOP_PROTO_ROOT}/teleop/v2/control.proto)
  set(TELEOP_PROTO_DIR ${CMAKE_CURRENT_BINARY_DIR}/proto)
  file(MAKE_DIRECTORY ${TELEOP_PROTO_DIR})
  add_custom_command(
    OUTPUT ${TELEOP_PROTO_DIR}/teleop/v2/control.pb.cc
           ${TELEOP_PROTO_DIR}/teleop/v2/control.pb.h
    COMMAND protobuf::protoc
    ARGS --cpp_out=${TELEOP_PROTO_DIR} -I${TELEOP_PROTO_ROOT}
         ${TELEOP_PROTO_ROOT}/teleop/v2/control.proto
    DEPENDS ${TELEOP_PROTO_ROOT}/teleop/v2/control.proto protobuf::protoc
    COMMENT "Generating C++ for teleop/v2/control.proto"
    VERBATIM)
  add_library(teleop_control_proto ${TELEOP_PROTO_DIR}/teleop/v2/control.pb.cc)
  target_include_directories(teleop_control_proto PUBLIC ${TELEOP_PROTO_DIR})
  target_link_libraries(teleop_control_proto PUBLIC protobuf::libprotobuf)
  # Generated code is not ours to lint.
  target_compile_options(teleop_control_proto PRIVATE -w)
else()
  message(STATUS
    "protobuf or the teleop_contracts submodule is missing - "
    "the control channel will not be built")
endif()

add_library(teleop_control_guard src/control_guard.cpp src/fake_vehicle.cpp)
target_include_directories(teleop_control_guard PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(teleop_control_guard PRIVATE -Wall -Wextra -Wpedantic)

# Config parsing carries no media dependency, so it builds on a ROS-only CI
# runner alongside the safety library.
add_library(teleop_media_config src/media_config.cpp)
target_include_directories(teleop_media_config PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(teleop_media_config PUBLIC yaml-cpp::yaml-cpp)
target_compile_options(teleop_media_config PRIVATE -Wall -Wextra -Wpedantic)

# Media support is optional at configure time so the safety library can still
# be built and tested on a ROS-only CI runner. Production rover images must
# install the GStreamer 1.24 development packages and will get this probe.
if(PkgConfig_FOUND)
  pkg_check_modules(GSTREAMER QUIET IMPORTED_TARGET gstreamer-1.0>=1.24)
  # The full send path additionally needs the WebRTC/SDP libraries and a
  # WebSocket + JSON client. They are checked separately so a machine with
  # only core GStreamer still gets the probe and the pipeline builder.
  pkg_check_modules(SIGNALING QUIET IMPORTED_TARGET
    gstreamer-webrtc-1.0>=1.24
    gstreamer-sdp-1.0>=1.24
    libsoup-3.0
    json-glib-1.0)
  if(GSTREAMER_FOUND)
    add_executable(teleop-media-probe src/media_probe.cpp)
    target_link_libraries(teleop-media-probe PRIVATE PkgConfig::GSTREAMER)
    target_compile_options(teleop-media-probe PRIVATE -Wall -Wextra -Wpedantic)
    install(TARGETS teleop-media-probe RUNTIME DESTINATION lib/${PROJECT_NAME})

    add_library(teleop_video_pipeline src/video_pipeline.cpp)
    target_include_directories(teleop_video_pipeline PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
    target_link_libraries(teleop_video_pipeline
      PUBLIC teleop_media_config
      PRIVATE PkgConfig::GSTREAMER)
    target_compile_options(teleop_video_pipeline PRIVATE -Wall -Wextra -Wpedantic)
    install(TARGETS teleop_video_pipeline EXPORT teleop_rover_v2Targets
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib)

    if(SIGNALING_FOUND)
      add_library(teleop_signaling_protocol src/signaling_protocol.cpp)
      target_include_directories(teleop_signaling_protocol PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
      target_link_libraries(teleop_signaling_protocol PRIVATE PkgConfig::SIGNALING)
      target_compile_options(teleop_signaling_protocol PRIVATE -Wall -Wextra -Wpedantic)
      install(TARGETS teleop_signaling_protocol EXPORT teleop_rover_v2Targets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)

      if(TARGET teleop_control_proto)
        add_executable(teleop-rover-node src/rover_node.cpp)
        target_link_libraries(teleop-rover-node PRIVATE
          teleop_signaling_protocol teleop_video_pipeline teleop_media_config
          teleop_control_guard teleop_control_proto
          PkgConfig::GSTREAMER PkgConfig::SIGNALING)
        target_compile_options(teleop-rover-node PRIVATE -Wall -Wextra -Wpedantic)
        install(TARGETS teleop-rover-node RUNTIME DESTINATION lib/${PROJECT_NAME})
      else()
        message(STATUS "no generated control contract - teleop-rover-node skipped")
      endif()
    else()
      message(STATUS
        "gstreamer-webrtc/sdp, libsoup-3.0 or json-glib not found - "
        "teleop-rover-node will not be built")
    endif()
  endif()
endif()

install(TARGETS teleop_control_guard teleop_media_config
  EXPORT teleop_rover_v2Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(control_guard_test test/control_guard_test.cpp)
  target_link_libraries(control_guard_test teleop_control_guard)

  ament_add_gtest(media_config_test test/media_config_test.cpp)
  target_link_libraries(media_config_test teleop_media_config)

  ament_add_gtest(fake_vehicle_test test/fake_vehicle_test.cpp)
  target_link_libraries(fake_vehicle_test teleop_control_guard)

  if(TARGET teleop_video_pipeline)
    # The pipeline test defines its own main so it can gst_init before the
    # registry is queried.
    ament_add_gtest(video_pipeline_test test/video_pipeline_test.cpp)
    target_link_libraries(video_pipeline_test
      teleop_video_pipeline PkgConfig::GSTREAMER)
  endif()

  if(TARGET teleop_signaling_protocol)
    ament_add_gtest(signaling_protocol_test test/signaling_protocol_test.cpp)
    target_link_libraries(signaling_protocol_test
      teleop_signaling_protocol PkgConfig::SIGNALING)
  endif()
endif()

ament_export_targets(teleop_rover_v2Targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
