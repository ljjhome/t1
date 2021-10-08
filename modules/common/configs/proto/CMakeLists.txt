project(common_configs_proto)

add_library(common_configs_vehicle_config_proto
vehicle_config.pb.cc 
)
target_include_directories(common_configs_vehicle_config_proto PUBLIC ${PROJECT_SOURCE_DIR}/../../../.. ${protobuf_include_dir})
target_link_libraries(common_configs_vehicle_config_proto 
PUBLIC 
    ${protobuf_lib}
    common_geometry_proto
    common_header_proto
)