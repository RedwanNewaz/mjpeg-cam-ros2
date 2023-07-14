cmake_minimum_required(VERSION 3.8)
project(mjpeg_cam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include

        )

set(SRC_FILES src/MjpegCam.cpp src/UsbCamera.cpp)


add_executable(mjpeg_cam_node src/mjpeg_cam_node.cpp ${SRC_FILES})

target_link_libraries(mjpeg_cam_node
        ${rclcpp_LIBRARIES}
        ${geometry_msgs_LIBRARIES}
        ${std_msgs_LIBRARIES}
        ${sensor_msgs_LIBRARIES}
        ${tf2_LIBRARIES}

        )

target_include_directories(mjpeg_cam_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mjpeg_cam_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  mjpeg_cam_node
  "sensor_msgs"
  "std_msgs"
  "geometry_msgs"
  "tf2"
  "rclcpp"
)

install(TARGETS mjpeg_cam_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()


install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})