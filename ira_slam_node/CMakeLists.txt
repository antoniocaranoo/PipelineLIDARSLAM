cmake_minimum_required(VERSION 3.10)
project(ira_slam_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)  
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(
  include
  include/scancontext/Scancontext  
  include/kiss_matcher/core   
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(ira_slam_node
    src/iraSlamNode.cpp
    include/kiss_matcher/core/kiss_matcher/KISSMatcher.cpp
    include/kiss_matcher/core/kiss_matcher/FasterPFH.cpp
    include/kiss_matcher/core/kiss_matcher/GncSolver.cpp
    include/kiss_matcher/core/kiss_matcher/ROBINMatching.cpp
    include/scancontext/Scancontext/Scancontext.cpp
)

ament_target_dependencies(ira_slam_node
  rclcpp
  sensor_msgs
  nav_msgs
  OpenCV
  cv_bridge
  pcl_conversions
)

# Link librerie PCL e esterne
target_link_libraries(ira_slam_node
  ${PCL_LIBRARIES}        
  lz4    
  robin  
  pmc
  OpenMP::OpenMP_CXX   
)

install(TARGETS
  ira_slam_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
