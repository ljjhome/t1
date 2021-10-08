project(common_proto)

# set(protobuf_lib /home/ljj/program/protobuf_install/lib/libprotobuf.so)

add_library(common_error_code_proto error_code.pb.cc)
target_include_directories(common_error_code_proto PUBLIC
                                ${protobuf_include_dir} 
                                ${PROJECT_SOURCE_DIR}/../../..
)
target_link_libraries(common_error_code_proto ${protobuf_lib})


add_library(common_drive_state_proto drive_state.pb.cc)
target_include_directories(common_drive_state_proto PUBLIC
                                ${protobuf_include_dir} 
                                ${PROJECT_SOURCE_DIR}/../../..
)
target_link_libraries(common_drive_state_proto ${protobuf_lib})

add_library(common_pnc_point_proto pnc_point.pb.cc)
target_include_directories(common_pnc_point_proto 
    PUBLIC
                                ${protobuf_include_dir} 
                                ${PROJECT_SOURCE_DIR}/../../..
)
target_link_libraries(common_pnc_point_proto ${protobuf_lib})


add_library(common_geometry_proto geometry.pb.cc)
target_include_directories(common_geometry_proto PUBLIC
                                ${protobuf_include_dir} 
                                ${PROJECT_SOURCE_DIR}/../../..
)
target_link_libraries(common_geometry_proto ${protobuf_lib})


add_library(common_direction_proto direction.pb.cc)
target_include_directories(common_direction_proto PUBLIC
                                ${protobuf_include_dir} 
                                ${PROJECT_SOURCE_DIR}/../../..
)
target_link_libraries(common_direction_proto ${protobuf_lib})


add_library(common_header_proto header.pb.cc)
target_include_directories(common_header_proto PUBLIC
                                ${protobuf_include_dir} 
                                ${PROJECT_SOURCE_DIR}/../../..
)
target_link_libraries(common_header_proto ${protobuf_lib} common_error_code_proto)


add_library(common_vehicle_signal_proto vehicle_signal.pb.cc)
target_include_directories(common_vehicle_signal_proto PUBLIC
                                ${protobuf_include_dir} 
                                ${PROJECT_SOURCE_DIR}/../../..
)
target_link_libraries(common_vehicle_signal_proto ${protobuf_lib})


add_library(common_drive_event_proto drive_event.pb.cc)
target_include_directories(common_drive_event_proto PUBLIC
                                ${protobuf_include_dir} 
                                ${PROJECT_SOURCE_DIR}/../../..
)
target_link_libraries(common_drive_event_proto ${protobuf_lib} common_header_proto localization_pose_proto)

add_library(common_proto INTERFACE)
# target_include_directories(common_proto 
#     INTERFACE 
#         ${PROJECT_SOURCE_DIR}/../../.. 
#         # ${protobuf_include_dir} 
# )
target_link_libraries(common_proto
    INTERFACE 
        common_drive_event_proto
        common_vehicle_signal_proto
        common_header_proto
        common_direction_proto
        common_geometry_proto
        common_pnc_point_proto
        common_drive_state_proto
        common_error_code_proto
)