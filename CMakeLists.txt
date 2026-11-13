cmake_minimum_required(VERSION 3.16)
project(vln_agent)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rgbd_image_queue REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs calib3d)

rosidl_generate_interfaces(${PROJECT_NAME} "srv/ToolCall.srv")
rosidl_get_typesupport_target(tool_call_typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_subdirectory(third_party/libbase64 EXCLUDE_FROM_ALL)

add_library(vln_agent_utils SHARED
  src/utils/time.cpp
  src/utils/json.cpp
  src/utils/base64.cpp
  src/utils/websocket_url.cpp)
target_compile_features(vln_agent_utils PUBLIC cxx_std_17)
target_include_directories(vln_agent_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(vln_agent_utils PUBLIC nlohmann_json::nlohmann_json yaml-cpp)
target_link_libraries(vln_agent_utils PRIVATE vln_agent_base64)

add_library(vln_agent_session SHARED
  src/session/session_manager.cpp
  src/session/responses_ws_transport.cpp)
target_link_libraries(vln_agent_session PUBLIC vln_agent_utils Threads::Threads)
target_link_libraries(vln_agent_session PRIVATE Boost::boost OpenSSL::SSL OpenSSL::Crypto)
target_compile_definitions(vln_agent_session PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)

add_library(vln_agent_tools SHARED
  src/tools/tool_manager.cpp
  src/tools/shared_resources.cpp
  src/tools/ros_callbacks.cpp)
target_compile_features(vln_agent_tools PUBLIC cxx_std_17)
target_include_directories(vln_agent_tools PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(vln_agent_tools PUBLIC rclcpp pluginlib)
target_link_libraries(vln_agent_tools PUBLIC
  vln_agent_utils Threads::Threads)
target_link_libraries(vln_agent_tools PRIVATE "${tool_call_typesupport}")

# 工具插件统一导出到一个库，源码和共享资源仍按 navigation/rgbd 分组。
add_library(vln_agent_tool_plugins SHARED
  plugins/tools/navigation/navigation_runtime.cpp
  plugins/tools/navigation/navigate_to_pose_start_navigation.cpp
  plugins/tools/navigation/navigate_to_pose_get_feedback.cpp
  plugins/tools/navigation/navigate_to_pose_cancel_navigation.cpp
  plugins/tools/navigation/navigate_through_poses_start_navigation.cpp
  plugins/tools/navigation/navigate_through_poses_get_feedback.cpp
  plugins/tools/navigation/navigate_through_poses_cancel_navigation.cpp
  plugins/tools/navigation/get_surroundings.cpp
  plugins/tools/navigation/navigate_to_pixel_start_navigation.cpp
  plugins/tools/navigation/navigate_through_pixels_start_navigation.cpp
  plugins/tools/navigation/navigate_to_objects.cpp
  plugins/tools/navigation/lookaround.cpp
  plugins/tools/navigation/project_depth_surround.cpp
  plugins/tools/navigation/utils.cpp
  plugins/tools/rgbd/rgbd_runtime.cpp
  plugins/tools/rgbd/capture_pair.cpp
  plugins/tools/rgbd/depth_project.cpp
  plugins/tools/rgbd/segmentation.cpp
  plugins/tools/rgbd/utils.cpp)
target_link_libraries(vln_agent_tool_plugins vln_agent_tools ${OpenCV_LIBS})
target_include_directories(vln_agent_tool_plugins PUBLIC ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(vln_agent_tool_plugins
  rclcpp_action nav2_msgs geometry_msgs action_msgs rgbd_image_queue sensor_msgs
  tf2_ros tf2_geometry_msgs ament_index_cpp)
option(VLN_AGENT_WITH_MOBILE_SAM "Use the MobileSAM TensorRT library when available" ON)
if(VLN_AGENT_WITH_MOBILE_SAM)
  find_package(mobile_sam_trt CONFIG QUIET HINTS
    "${CMAKE_CURRENT_SOURCE_DIR}/../segment_tools/tensorrt_lib/install")
  if(mobile_sam_trt_FOUND)
    target_link_libraries(vln_agent_tool_plugins mobile_sam_trt::mobile_sam_trt)
    target_compile_definitions(vln_agent_tool_plugins PRIVATE VLN_AGENT_HAS_MOBILE_SAM)
    set_property(TARGET vln_agent_tool_plugins APPEND PROPERTY INSTALL_RPATH
      "$<TARGET_FILE_DIR:mobile_sam_trt::mobile_sam_trt>")
    message(STATUS "RGB-D segmentation: MobileSAM TensorRT")
  else()
    message(STATUS "RGB-D segmentation: library unavailable; positive-point projection enabled")
  endif()
endif()


add_library(vln_agent_interaction SHARED
  src/interaction/user_interaction_manager.cpp)
target_link_libraries(vln_agent_interaction PUBLIC vln_agent_session vln_agent_tools)
ament_target_dependencies(vln_agent_interaction PUBLIC rclcpp pluginlib)

add_library(vln_agent_events SHARED src/events/event_manager.cpp)
target_link_libraries(vln_agent_events PUBLIC vln_agent_utils Threads::Threads)
ament_target_dependencies(vln_agent_events PUBLIC rclcpp std_msgs)

add_library(vln_agent_ros_topic_interface SHARED
  plugins/interfaces/ros_topic_interface/ros_topic_interface.cpp)
target_link_libraries(vln_agent_ros_topic_interface vln_agent_utils)
ament_target_dependencies(vln_agent_ros_topic_interface rclcpp std_msgs pluginlib)

add_executable(vln_agent_node src/vln_agent.cpp)
target_link_libraries(vln_agent_node vln_agent_interaction vln_agent_events)
ament_target_dependencies(vln_agent_node rclcpp ament_index_cpp)

foreach(target vln_agent_utils vln_agent_session vln_agent_tools vln_agent_interaction vln_agent_ros_topic_interface vln_agent_tool_plugins vln_agent_events vln_agent_node)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
endforeach()
pluginlib_export_plugin_description_file(vln_agent tool_plugins.xml)
pluginlib_export_plugin_description_file(vln_agent user_interface_plugins.xml)

install(TARGETS vln_agent_utils vln_agent_session vln_agent_tools vln_agent_interaction vln_agent_events
  EXPORT export_vln_agent_tools
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS vln_agent_tool_plugins vln_agent_ros_topic_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS vln_agent_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY plugins/ DESTINATION share/${PROJECT_NAME}/plugins
  FILES_MATCHING PATTERN "tool.json")
install(DIRECTORY config DESTINATION share/${PROJECT_NAME} PATTERN "*.local.yaml" EXCLUDE)
install(DIRECTORY docs prompts DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(FILES third_party/libbase64/LICENSE third_party/libbase64/README.vendor.md
  DESTINATION share/${PROJECT_NAME}/licenses/libbase64)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_library(vln_agent_test_tools SHARED test/test_tool.cpp)
  target_link_libraries(vln_agent_test_tools vln_agent_tools)
  pluginlib_export_plugin_description_file(vln_agent test/test_plugins.xml)
  install(TARGETS vln_agent_test_tools LIBRARY DESTINATION lib)

  ament_add_gtest(test_tool_manager test/test_tool_manager.cpp TIMEOUT 40)
  target_link_libraries(test_tool_manager vln_agent_tools vln_agent_test_tools)
  ament_target_dependencies(test_tool_manager ament_index_cpp)

  ament_add_gtest(test_tool_call_service test/test_tool_call_service.cpp TIMEOUT 30)
  target_link_libraries(test_tool_call_service
    vln_agent_tools vln_agent_test_tools "${tool_call_typesupport}")
  target_compile_definitions(test_tool_call_service PRIVATE
    VLN_TEST_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}/test-fixtures")
  set_tests_properties(test_tool_call_service PROPERTIES ENVIRONMENT
    "ROS_DOMAIN_ID=177;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros-logs")

  ament_add_gtest(test_ros_tools test/test_ros_tools.cpp TIMEOUT 50)
  target_link_libraries(test_ros_tools vln_agent_tools ${OpenCV_LIBS} "${tool_call_typesupport}")
  target_include_directories(test_ros_tools PRIVATE ${OpenCV_INCLUDE_DIRS})
  ament_target_dependencies(test_ros_tools
    rclcpp_action nav2_msgs action_msgs sensor_msgs rgbd_image_queue ament_index_cpp tf2_ros tf2_geometry_msgs)
  ament_add_gtest(test_rgbd_tools test/test_rgbd_tools.cpp TIMEOUT 90)
  target_link_libraries(test_rgbd_tools vln_agent_tool_plugins)
  ament_target_dependencies(test_rgbd_tools ament_index_cpp rclcpp_action nav2_msgs)
  target_compile_definitions(test_rgbd_tools PRIVATE
    VLN_TEST_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}/test-fixtures")
  set_tests_properties(test_rgbd_tools PROPERTIES ENVIRONMENT
    "ROS_DOMAIN_ID=178;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros-logs")

  add_executable(benchmark_depth_project test/benchmark_depth_project.cpp)
  target_link_libraries(benchmark_depth_project vln_agent_tool_plugins)

  ament_add_gtest(test_navigate_through_poses test/test_navigate_through_poses.cpp TIMEOUT 45)
  target_link_libraries(test_navigate_through_poses vln_agent_tools)
  ament_target_dependencies(test_navigate_through_poses rclcpp_action nav2_msgs action_msgs ament_index_cpp)
  target_compile_definitions(test_navigate_through_poses PRIVATE
    VLN_TEST_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}/test-fixtures")
  set_tests_properties(test_navigate_through_poses PROPERTIES ENVIRONMENT
    "ROS_DOMAIN_ID=179;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros-logs")

  ament_add_gtest(test_navigation_utils test/test_navigation_utils.cpp TIMEOUT 20)
  target_link_libraries(test_navigation_utils vln_agent_tool_plugins)
  target_compile_definitions(test_navigation_utils PRIVATE
    VLN_TEST_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}/test-fixtures")

  add_library(vln_agent_ws_fixture STATIC test/ws_fixture.cpp)
  target_link_libraries(vln_agent_ws_fixture PUBLIC
    vln_agent_session Boost::boost OpenSSL::SSL OpenSSL::Crypto)
  target_compile_definitions(vln_agent_ws_fixture PRIVATE BOOST_ERROR_CODE_HEADER_ONLY
    VLN_TEST_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}/test-fixtures")

  add_library(vln_agent_test_interfaces SHARED test/test_interface.cpp)
  target_link_libraries(vln_agent_test_interfaces vln_agent_interaction)
  pluginlib_export_plugin_description_file(vln_agent test/test_interface_plugins.xml)
  install(TARGETS vln_agent_test_interfaces LIBRARY DESTINATION lib)

  add_executable(live_interaction_test test/live_interaction.cpp)
  target_link_libraries(live_interaction_test vln_agent_interaction)
  ament_target_dependencies(live_interaction_test std_msgs)

  ament_add_gtest(test_user_interaction_manager test/test_user_interaction_manager.cpp TIMEOUT 60)
  target_link_libraries(test_user_interaction_manager
    vln_agent_interaction vln_agent_test_interfaces vln_agent_test_tools vln_agent_ws_fixture)
  ament_target_dependencies(test_user_interaction_manager std_msgs)
  set_tests_properties(test_user_interaction_manager PROPERTIES ENVIRONMENT
    "ROS_DOMAIN_ID=174;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros-logs")

  ament_add_gtest(test_event_manager test/test_event_manager.cpp TIMEOUT 20)
  target_link_libraries(test_event_manager vln_agent_events)
  target_compile_definitions(test_event_manager PRIVATE
    VLN_TEST_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}/test-fixtures")
  set_tests_properties(test_event_manager PROPERTIES ENVIRONMENT
    "ROS_DOMAIN_ID=176;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros-logs")

  ament_add_gtest(test_base64 test/test_base64.cpp TIMEOUT 20)
  target_link_libraries(test_base64 vln_agent_utils OpenSSL::Crypto Threads::Threads)

  ament_add_gtest(test_utils test/test_utils.cpp TIMEOUT 15)
  target_link_libraries(test_utils vln_agent_utils)
  ament_add_gtest(test_session_manager test/test_session_manager.cpp TIMEOUT 45)
  target_link_libraries(test_session_manager vln_agent_ws_fixture)
  ament_add_gtest(test_ws_transport test/test_ws_transport.cpp TIMEOUT 45)
  target_link_libraries(test_ws_transport vln_agent_ws_fixture)

  foreach(target test_tool_manager test_ros_tools test_session_manager test_ws_transport test_user_interaction_manager)
    target_compile_definitions(${target} PRIVATE
      VLN_TEST_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}/test-fixtures")
  endforeach()
  # 仅连接专用假服务及合成相机；两个测试进程各用独立 ROS domain。
  set_tests_properties(test_tool_manager PROPERTIES ENVIRONMENT
    "ROS_DOMAIN_ID=171;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros-logs")
  set_tests_properties(test_ros_tools PROPERTIES ENVIRONMENT
    "ROS_DOMAIN_ID=172;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros-logs")
endif()

ament_export_targets(export_vln_agent_tools HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs pluginlib nlohmann_json yaml-cpp Threads rgbd_image_queue tf2_ros tf2_geometry_msgs OpenCV rosidl_default_runtime)
ament_package()

