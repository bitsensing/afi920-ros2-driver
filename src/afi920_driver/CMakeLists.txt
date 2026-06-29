cmake_minimum_required(VERSION 3.16)
project(afi920_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(afi920_msgs REQUIRED)

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  visualization_msgs
  geometry_msgs
  tf2
  tf2_ros
  afi920_msgs
)

# --- Driver Library ---
find_package(Threads REQUIRED)

add_library(afi920_driver_lib SHARED
  src/afi920_node.cpp
  src/pointcloud_builder.cpp
  src/config_client.cpp
  src/udp_receiver.cpp
  src/tcp_stream_client.cpp
  src/e2e.cpp
  src/someip_parser.cpp
  src/rdi_parser.cpp
  src/shi_parser.cpp
  src/spi_parser.cpp
)
target_include_directories(afi920_driver_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(afi920_driver_lib Threads::Threads)
ament_target_dependencies(afi920_driver_lib ${DEPENDENCIES})

# --- Composable Node Registration ---
rclcpp_components_register_node(afi920_driver_lib
  PLUGIN "afi920_driver::Afi920Node"
  EXECUTABLE afi920_node
)

# --- Install ---
install(TARGETS afi920_driver_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# --- Testing ---
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Lint paths — only driver's own code.
  set(_DRIVER_PATHS "include/afi920_driver" "src" "test" "launch" "config")
  set(_PYTHON_PATHS "launch")

  find_package(ament_cmake_cppcheck REQUIRED)
  ament_cppcheck(${_DRIVER_PATHS})

  find_package(ament_cmake_cpplint REQUIRED)
  ament_cpplint(${_DRIVER_PATHS}
    MAX_LINE_LENGTH 120
    FILTERS "-build/include_order" "-build/header_guard" "-whitespace/newline")

  find_package(ament_cmake_flake8 REQUIRED)
  ament_flake8(${_PYTHON_PATHS})

  find_package(ament_cmake_pep257 REQUIRED)
  ament_pep257(${_PYTHON_PATHS})

  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()

  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()

  # Unit tests
  ament_add_gtest(test_pointcloud_builder test/test_pointcloud_builder.cpp)
  target_link_libraries(test_pointcloud_builder afi920_driver_lib)
  ament_target_dependencies(test_pointcloud_builder sensor_msgs afi920_msgs)

  ament_add_gtest(test_someip_parser test/test_someip_parser.cpp)
  target_link_libraries(test_someip_parser afi920_driver_lib)
  ament_target_dependencies(test_someip_parser ${DEPENDENCIES})

  ament_add_gtest(test_rdi_parser test/test_rdi_parser.cpp)
  target_link_libraries(test_rdi_parser afi920_driver_lib)
  ament_target_dependencies(test_rdi_parser ${DEPENDENCIES})

  ament_add_gtest(test_shi_parser test/test_shi_parser.cpp)
  target_link_libraries(test_shi_parser afi920_driver_lib)
  ament_target_dependencies(test_shi_parser ${DEPENDENCIES})

  ament_add_gtest(test_spi_parser test/test_spi_parser.cpp)
  target_link_libraries(test_spi_parser afi920_driver_lib)
  ament_target_dependencies(test_spi_parser ${DEPENDENCIES})
endif()

ament_package()
