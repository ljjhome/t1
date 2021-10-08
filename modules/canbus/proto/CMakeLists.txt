project(canbus_proto)


add_library(canbus_devkit_proto devkit.pb.cc)
target_include_directories(canbus_devkit_proto PUBLIC ${protobuf_include_dir})
target_link_libraries(canbus_devkit_proto 
PUBLIC 
${protobuf_lib}
)

add_library(canbus_chassis_proto chassis.pb.cc)
target_include_directories(canbus_chassis_proto PUBLIC ${protobuf_include_dir})
target_link_libraries(canbus_chassis_proto
PUBLIC 
common_configs_vehicle_config_proto
common_drive_state_proto
common_geometry_proto
common_header_proto
common_vehicle_signal_proto
)