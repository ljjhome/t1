project(localization_proto)


set(localization_proto_dir ${PROJECT_SOURCE_DIR}/../../.. )
add_library(localization_pose_proto pose.pb.cc)
target_include_directories(localization_pose_proto PUBLIC ${localization_proto_dir} ${protobuf_include_dir})
target_link_libraries(localization_pose_proto 
PUBLIC
    ${protobuf_lib} 
    common_geometry_proto
)

add_library(localization_status_proto localization_status.pb.cc)
target_include_directories(localization_status_proto PUBLIC ${localization_proto_dir} ${protobuf_include_dir})
target_link_libraries(localization_status_proto 
PUBLIC
    ${protobuf_lib} 
)

add_library(localization_config_proto localization_config.pb.cc)
target_include_directories(localization_config_proto PUBLIC ${localization_proto_dir} ${protobuf_include_dir})
target_link_libraries(localization_config_proto 
PUBLIC
    ${protobuf_lib} 
)

add_library(localization_imu_proto imu.pb.cc)
target_include_directories(localization_imu_proto PUBLIC ${localization_proto_dir} ${protobuf_include_dir})
target_link_libraries(localization_imu_proto 
PUBLIC
    localization_pose_proto
    common_header_proto
    ${protobuf_lib} 
)

add_library(localization_proto localization.pb.cc)
target_include_directories(localization_proto PUBLIC ${localization_proto_dir} ${protobuf_include_dir})
target_link_libraries(localization_proto 
PUBLIC
    localization_pose_proto
    localization_status_proto
    common_geometry_proto
    common_header_proto
    common_pnc_point_proto
    ${protobuf_lib} 
)


