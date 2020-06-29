cmake_minimum_required(VERSION 3.5.1)

project(fwmav_sim_sjtu)

set(required_components utils-urdf gui gui-osg)
set(required_libraries dart dart-utils-urdf dart-gui dart-gui-osg)
#set(DART_ROOT /usr/share)

## Find DartSim & ROS
find_package(DART 6.9.0 REQUIRED COMPONENTS ${required_components} CONFIG)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS 
 roscpp
 std_msgs
 geometry_msgs
 )

add_compile_options(-std=c++14)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${DART_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  include
)

## Main Target
add_executable(flappy_dart
src/sim_main.cpp
src/SimEnv.cpp
src/SimPanel.cpp
src/Flappy.cpp
src/Actuator.cpp
src/Wing.cpp
src/sensor/VirtualSensor.cpp
src/cjson/cJSON.c
src/cjson/CJsonObject.cpp
)
target_link_libraries(flappy_dart PUBLIC ${required_libraries} ${catkin_LIBRARIES})


## Test Targets
add_executable(ex_simple examples/main.cpp)
target_link_libraries(ex_simple PUBLIC ${required_libraries})

add_executable(ex_domino examples/domino.cpp)
target_link_libraries(ex_domino PUBLIC ${required_libraries})

add_executable(ex_imgui examples/imgui.cpp)
target_link_libraries(ex_imgui PUBLIC ${required_libraries})

add_executable(ex_json
examples/demo.cpp
src/cjson/cJSON.c
src/cjson/CJsonObject.cpp
)
target_link_libraries(ex_json PUBLIC ${required_libraries})



## Copy Resources
FILE(GLOB Config_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/config/*.json
)
FILE(GLOB Urdf_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/urdf/fwmav_sjtu/*.urdf
)
FILE(GLOB Mesh_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/urdf/fwmav_sjtu/meshes/*.stl
    ${CMAKE_CURRENT_SOURCE_DIR}/urdf/fwmav_sjtu/meshes/*.STL
)
add_custom_target(CopyConfigFiles ALL
  VERBATIM
  COMMAND_EXPAND_LISTS
  COMMAND ${CMAKE_COMMAND} -E
          make_directory "${PROJECT_BINARY_DIR}/config/"
  COMMAND ${CMAKE_COMMAND} -E
          make_directory "${PROJECT_BINARY_DIR}/urdf/fwmav_sjtu/meshes/"
  COMMAND ${CMAKE_COMMAND} -E 
         copy_if_different
                "${Config_FILES}"
                "${PROJECT_BINARY_DIR}/config/"
  COMMAND ${CMAKE_COMMAND} -E 
         copy_if_different
                "${Urdf_FILES}"
                "${PROJECT_BINARY_DIR}/urdf/fwmav_sjtu/"
  COMMAND ${CMAKE_COMMAND} -E 
         copy_if_different
                "${Mesh_FILES}"
                "${PROJECT_BINARY_DIR}/urdf/fwmav_sjtu/meshes/"
)