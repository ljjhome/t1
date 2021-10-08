project(map_relative_map_proto)

set(map_relative_map_proto_dir ${PROJECT_SOURCE_DIR}/../../../..)
add_library(map_relative_map_navigation_proto navigation.pb.cc)
target_include_directories(map_relative_map_navigation_proto PUBLIC ${map_relative_map_proto_dir} ${protobuf_include_dir})
target_link_libraries(map_relative_map_navigation_proto 
PUBLIC 
    ${protobuf_lib}
    common_header_proto
    common_pnc_point_proto
    localization_proto
    map_proto
    perception_obstacle_proto
)

add_library(map_relative_map_navigator_config_proto navigator_config.pb.cc )
target_include_directories(map_relative_map_navigator_config_proto PUBLIC ${protobuf_include_dir})
target_link_libraries(map_relative_map_navigator_config_proto 
PUBLIC 
    ${protobuf_lib}
)
add_library(map_relative_map_config_proto relative_map_config.pb.cc )
target_include_directories(map_relative_map_config_proto PUBLIC ${protobuf_include_dir})
target_link_libraries(map_relative_map_config_proto 
PUBLIC 
    ${protobuf_lib}
)
