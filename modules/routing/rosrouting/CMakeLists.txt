project(routing_rosrouting)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
set(CMAKE_CXX_STANDARD 14)
find_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs
geometry_msgs
lanelet2_core
  lanelet2_io
  lanelet2_maps
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
  lanelet2_validation
  visualization_msgs
)
include_directories(/usr/include/eigen3)
# include_directories(${catkin_INCLUDE_DIRS})
add_library(routing_rosrouting rosrouting.cc)
target_link_libraries(routing_rosrouting
PUBLIC 
common_adapter_gflags
common_util_point_factory
map_hdmap_util
routing_core
routing_config_proto
)

set(rosrouting_node_src 
    rosrouting_node.cc
    rosrouting_node_main.cc
)
add_executable(rosrouting_node ${rosrouting_node_src})
target_include_directories(rosrouting_node PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(rosrouting_node
PUBLIC 
routing_rosrouting
${catkin_LIBRARIES}
)

add_executable(rosrouting_sim_planner rosrouting_sim_plan_main.cc )
target_include_directories(rosrouting_sim_planner PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(rosrouting_sim_planner
PUBLIC
routing_rosrouting
${catkin_LIBRARIES}
)
