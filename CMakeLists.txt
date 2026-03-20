cmake_minimum_required(VERSION 3.8)
project(radarays_gazebo_plugins VERSION 1.0.0)

add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

# DEFAULT RELEASE
if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(radarays_ros REQUIRED)
find_package(rmagine_gazebo_plugins REQUIRED)

find_library(RMAGINE_EMBREE_SPHERICAL_LIB
  NAMES rmagine_embree_spherical_gzplugin
  HINTS ${rmagine_gazebo_plugins_DIR}/../../../lib/rmagine_gazebo_plugins
)

find_library(RMAGINE_OPTIX_SPHERICAL_LIB
  NAMES rmagine_optix_spherical_gzplugin
  HINTS ${rmagine_gazebo_plugins_DIR}/../../../lib/rmagine_gazebo_plugins
)

find_library(RADARAYS_LIB
  NAMES radarays
  HINTS ${radarays_ros_DIR}/../../../lib
)

find_library(RADARAYS_CPU_LIB
  NAMES radarays_cpu
  HINTS ${radarays_ros_DIR}/../../../lib
)

find_library(RADARAYS_GPU_LIB
  NAMES radarays_gpu
  HINTS ${radarays_ros_DIR}/../../../lib
)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)

find_package(gazebo REQUIRED)

find_package(OpenCV REQUIRED)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif(OPENMP_FOUND)

find_package(rmagine 2.2.1
COMPONENTS
  core
OPTIONAL_COMPONENTS
  embree
  cuda
  optix
)

set(ROS2_DEPS
  gazebo_ros
  rclcpp
  sensor_msgs
  radarays_ros
  rmagine_gazebo_plugins
  cv_bridge
  image_transport
  tf2
  tf2_ros
  tf2_msgs
)

include_directories(
  include
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${radarays_ros_INCLUDE_DIRS}
  ${rmagine_gazebo_plugins_INCLUDE_DIRS}
)

if(TARGET rmagine::embree)

message(WARNING "BUILD RadaRays Embree Gazebo Plugin")

### BUILD SENSOR PLUGIN ###
add_library(radarays_embree_gzplugin SHARED
  src/radarays_embree_gzplugin.cpp
)

ament_target_dependencies(radarays_embree_gzplugin ${ROS2_DEPS})

target_link_libraries(radarays_embree_gzplugin
  ${GAZEBO_LIBRARIES}
  ${OpenCV_LIBS}
  rmagine::embree
  ${RMAGINE_EMBREE_SPHERICAL_LIB}
  ${RADARAYS_LIB}
  ${RADARAYS_CPU_LIB}
)

### REGISTER SENSOR PLUGIN ###
add_library(radarays_embree_sensors_gzregister SHARED
  src/radarays_embree_gzregister.cpp
)

target_link_libraries(radarays_embree_sensors_gzregister
  radarays_embree_gzplugin
  ${GAZEBO_LIBRARIES}
  ${OpenCV_LIBS}
  rmagine::embree
)

# ROS Gazebo plugins
add_library(radarays_embree_ros_gzplugin SHARED
  src/radarays_embree_ros_gzplugin.cpp)

ament_target_dependencies(radarays_embree_ros_gzplugin ${ROS2_DEPS})

target_link_libraries(radarays_embree_ros_gzplugin
  radarays_embree_gzplugin
  ${GAZEBO_LIBRARIES}
  rmagine::embree
)

install(TARGETS
  radarays_embree_gzplugin
  radarays_embree_sensors_gzregister
  radarays_embree_ros_gzplugin
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin
)

endif(TARGET rmagine::embree)

if(TARGET rmagine::optix)

message(WARNING "BUILD RadaRays Optix Gazebo Plugin")

### BUILD SENSOR PLUGIN ###
add_library(radarays_optix_gzplugin SHARED
  src/radarays_optix_gzplugin.cpp
)

ament_target_dependencies(radarays_optix_gzplugin ${ROS2_DEPS})

target_link_libraries(radarays_optix_gzplugin
  ${GAZEBO_LIBRARIES}
  ${OpenCV_LIBS}
  rmagine::optix
  ${RMAGINE_OPTIX_SPHERICAL_LIB}
  ${RADARAYS_LIB}
  ${RADARAYS_GPU_LIB}
)

### REGISTER SENSOR PLUGIN ###
add_library(radarays_optix_sensors_gzregister SHARED
  src/radarays_optix_gzregister.cpp
)

target_link_libraries(radarays_optix_sensors_gzregister
  radarays_optix_gzplugin
  ${GAZEBO_LIBRARIES}
  ${OpenCV_LIBS}
  rmagine::optix
)

# ROS Gazebo plugins
add_library(radarays_optix_ros_gzplugin SHARED
  src/radarays_optix_ros_gzplugin.cpp)

ament_target_dependencies(radarays_optix_ros_gzplugin ${ROS2_DEPS})

target_link_libraries(radarays_optix_ros_gzplugin
  radarays_optix_gzplugin
  ${GAZEBO_LIBRARIES}
  rmagine::optix
)

install(TARGETS
  radarays_optix_gzplugin
  radarays_optix_sensors_gzregister
  radarays_optix_ros_gzplugin
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin
)

endif(TARGET rmagine::optix)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch worlds urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
