project(map_proto)

set(map_proto_dir ${PROJECT_SOURCE_DIR}/../../..)

add_library(map_id_proto map_id.pb.cc)
target_include_directories(map_id_proto PUBLIC ${protobuf_include_dir} ${map_proto_dir})
target_link_libraries(map_id_proto PUBLIC ${protobuf_lib})

add_library(map_geometry_proto map_geometry.pb.cc)
target_include_directories(map_geometry_proto PUBLIC ${protobuf_include_dir} ${map_proto_dir})
target_link_libraries(map_geometry_proto PUBLIC ${protobuf_lib} common_geometry_proto)

add_library(map_speed_control_proto map_speed_control.pb.cc)
target_include_directories(map_speed_control_proto PUBLIC ${protobuf_include_dir} ${map_proto_dir})
target_link_libraries(map_speed_control_proto PUBLIC ${protobuf_lib} map_geometry_proto)

add_library(map_rsu_proto map_rsu.pb.cc)
target_include_directories(map_rsu_proto PUBLIC ${protobuf_include_dir} ${map_proto_dir})
target_link_libraries(map_rsu_proto PUBLIC ${protobuf_lib} map_geometry_proto map_id_proto)

add_library(map_stop_sign_proto map_stop_sign.pb.cc)
target_include_directories(map_stop_sign_proto PUBLIC ${protobuf_include_dir} ${map_proto_dir})
target_link_libraries(map_stop_sign_proto PUBLIC ${protobuf_lib} map_geometry_proto map_id_proto)

add_library(map_junction_proto map_junction.pb.cc)
target_include_directories(map_junction_proto PUBLIC ${protobuf_include_dir} ${map_proto_dir})
target_link_libraries(map_junction_proto PUBLIC ${protobuf_lib} map_geometry_proto map_id_proto)


add_library(map_pnc_junction_proto map_pnc_junction.pb.cc)
target_include_directories(map_pnc_junction_proto PUBLIC ${protobuf_include_dir} ${map_proto_dir})
target_link_libraries(map_pnc_junction_proto PUBLIC ${protobuf_lib} map_geometry_proto map_id_proto)


add_library(map_road_proto map_road.pb.cc)
target_include_directories(map_road_proto PUBLIC ${protobuf_include_dir} ${map_proto_dir})
target_link_libraries(map_road_proto PUBLIC ${protobuf_lib} map_geometry_proto map_id_proto)


add_library(map_parking_space_proto map_parking_space.pb.cc)
target_include_directories(map_parking_space_proto PUBLIC ${protobuf_include_dir} ${map_proto_dir})
target_link_libraries(map_parking_space_proto PUBLIC ${protobuf_lib} map_geometry_proto map_id_proto)


add_library(map_speed_bump_proto map_speed_bump.pb.cc)
target_include_directories(map_speed_bump_proto PUBLIC ${protobuf_include_dir} ${map_proto_dir})
target_link_libraries(map_speed_bump_proto PUBLIC ${protobuf_lib} map_geometry_proto map_id_proto)


add_library(map_signal_proto map_signal.pb.cc)
target_include_directories(map_signal_proto PUBLIC ${protobuf_include_dir} ${map_proto_dir})
target_link_libraries(map_signal_proto PUBLIC ${protobuf_lib} map_geometry_proto map_id_proto common_geometry_proto)


add_library(map_yield_sign_proto map_yield_sign.pb.cc)
target_include_directories(map_yield_sign_proto PUBLIC ${protobuf_include_dir} ${map_proto_dir})
target_link_libraries(map_yield_sign_proto PUBLIC ${protobuf_lib} map_geometry_proto map_id_proto)

add_library(map_clear_area_proto map_clear_area.pb.cc)
target_include_directories(map_clear_area_proto PUBLIC ${protobuf_include_dir} ${map_proto_dir})
target_link_libraries(map_clear_area_proto PUBLIC ${protobuf_lib} map_geometry_proto map_id_proto)

add_library(map_crosswalk_proto map_crosswalk.pb.cc)
target_include_directories(map_crosswalk_proto PUBLIC ${protobuf_include_dir} ${map_proto_dir})
target_link_libraries(map_crosswalk_proto PUBLIC ${protobuf_lib} map_geometry_proto map_id_proto)

add_library(map_overlap_proto map_overlap.pb.cc)
target_include_directories(map_overlap_proto PUBLIC ${protobuf_include_dir} ${map_proto_dir})
target_link_libraries(map_overlap_proto PUBLIC ${protobuf_lib} map_geometry_proto map_id_proto)


add_library(map_lane_proto map_lane.pb.cc)
target_include_directories(map_lane_proto PUBLIC ${protobuf_include_dir} ${map_proto_dir})
target_link_libraries(map_lane_proto PUBLIC ${protobuf_lib} map_geometry_proto map_id_proto)


add_library(map_proto map.pb.cc)
target_include_directories(map_proto PUBLIC ${protobuf_include_dir} ${map_proto_dir})
target_link_libraries(map_proto 
PUBLIC
    map_clear_area_proto
    map_crosswalk_proto
    map_junction_proto
    map_lane_proto
    map_overlap_proto
    map_parking_space_proto
    map_pnc_junction_proto
    map_road_proto
    map_signal_proto
    map_speed_bump_proto
    map_stop_sign_proto
    map_yield_sign_proto
    map_rsu_proto
)

# set(map_proto_src 
#         map_clear_area.pb.cc 
#         map_crosswalk.pb.cc 
#         map_geometry.pb.cc 
#         map_id.pb.cc 
#         map_junction.pb.cc 
#         map_lane.pb.cc 
#         map_overlap.pb.cc 
#         map_parking_space.pb.cc 
#         map_pnc_junction.pb.cc 
#         map_road.pb.cc 
#         map_rsu.pb.cc 
#         map_signal.pb.cc 
#         map_speed_bump.pb.cc 
#         map_speed_control.pb.cc 
#         map_stop_sign.pb.cc 
#         map_yield_sign.pb.cc 
#         map.pb.cc 
# )
# add_library(map_proto ${map_proto_src})
# target_include_directories(map_proto 
#     PUBLIC
#     ${PROJECT_SOURCE_DIR}/../../..
#     ${protobuf_include_dir}
#     )
# target_link_libraries(map_proto 
#     PUBLIC
#     common_proto
#     ${protobuf_lib}
#     )