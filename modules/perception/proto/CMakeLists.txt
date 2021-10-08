project(perception_proto)

set(perception_proto_dir ${PROJECT_SOURCE_DIR}/../../..)
add_library(perception_obstacle_proto perception_obstacle.pb.cc )
target_include_directories(perception_obstacle_proto PUBLIC ${perception_proto_dir} ${protobuf_include_dir})
target_link_libraries(perception_obstacle_proto
PUBLIC 
    ${protobuf_lib}
    common_error_code_proto
    common_geometry_proto
    common_header_proto
    common_pnc_point_proto
    map_lane_proto
)
