project(map_pnc_map)

add_library(map_pnc_map_path path.cc )
target_include_directories(map_pnc_map_path PUBLIC ${PROJECT_SOURCE_DIR}/../../..)
target_link_libraries(map_pnc_map_path 
PUBLIC 
common_math
map_hdmap
map_hdmap_util
map_lane_proto
)

add_library(map_pnc_map_route_segments route_segments.cc )
target_link_libraries(map_pnc_map_route_segments 
PUBLIC 
common_vehicle_state_proto
map_hdmap 
map_pnc_map_path
routing_common_gflags
routing_proto
)

add_library(map_pnc_map pnc_map.cc)
target_link_libraries(map_pnc_map 
PUBLIC 
map_pnc_map_path
map_pnc_map_route_segments
common_util_point_factory
common_vehicle_state_proto
map_hdmap
planning_common_gflags
routing_common_gflags
gtest
)
