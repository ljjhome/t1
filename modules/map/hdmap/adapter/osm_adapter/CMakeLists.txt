project(osm_adapter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  lanelet2_core
  lanelet2_io
  lanelet2_maps
  lanelet2_projection
  lanelet2_routing
  lanelet2_traffic_rules
  lanelet2_validation
  std_msgs
)
set(osm_adapter_test_src 
    osm_adapter.cc 
    osm_adapter_main_test.cc 
)
add_executable(osm_adapter_test ${osm_adapter_test_src})
target_link_libraries(osm_adapter_test
PUBLIC 
map_proto
cyber_common_log
cyber_common_file
map_hdmap_adapter_proto_organizer
${catkin_LIBRARIES}
)

add_executable(osm_listener osm_adapter_listener_main.cc )
target_link_libraries(osm_listener 
PUBLIC 
map_proto
${catkin_LIBRARIES}
)