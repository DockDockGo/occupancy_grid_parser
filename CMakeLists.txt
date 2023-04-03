cmake_minimum_required(VERSION 3.8)
project(occupancy_grid_parser)

# === Environment ===

# Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

set(CMAKE_CXX_STANDARD 14)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()


include_directories(
  include
)

set(lib_name ${PROJECT_NAME})

# === Dependencies ===

# find dependencies

set(dep_pkgs
    rclcpp
    nav2_costmap_2d
    pluginlib)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

# === Build ===

add_library(${lib_name} SHARED src/costmap_parser.cpp)

ament_target_dependencies(${lib_name} ${dep_pkgs})

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# === Installation ===

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES 
  occupancy_grid_parser.xml
  DESTINATION share
  )

install(TARGETS ${lib_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# pluginlib_export_plugin_description_file() installs gradient_layer.xml
# file into "share" directory and sets ament indexes for it.
# This allows the plugin to be discovered as a plugin of required type.

ament_export_include_directories(include)
ament_export_libraries(${lib_name})
ament_export_dependencies(${dep_pkgs})

pluginlib_export_plugin_description_file(nav2_costmap_2d occupancy_grid_parser.xml)

ament_package()
