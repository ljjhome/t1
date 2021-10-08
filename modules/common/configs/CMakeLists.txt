project(common_configs)

add_subdirectory(proto)

set(common_configs_dir ${PROJECT_SOURCE_DIR}/../../..)

add_library(common_configs_config_gflags config_gflags.cc )
target_include_directories(common_configs_config_gflags PUBLIC ${common_configs_dir})
target_link_libraries(common_configs_config_gflags 
PUBLIC 
    gflags
)

add_library(common_configs_vehicle_config_helper vehicle_config_helper.cc )
target_include_directories(common_configs_vehicle_config_helper PUBLIC ${common_configs_dir})
target_link_libraries(common_configs_vehicle_config_helper 
PUBLIC 
    common_configs_config_gflags
    cyber_common_file
    cyber_common_macros
    common_configs_vehicle_config_proto
    common_math_geometry
    common_pnc_point_proto
)