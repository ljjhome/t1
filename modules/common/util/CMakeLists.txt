project(common_util)

set(common_util_dir ${PROJECT_SOURCE_DIR}/../../..)

add_subdirectory(testdata)


add_library(common_util_color INTERFACE)
target_include_directories(common_util_color INTERFACE ${common_util_dir})

add_library(common_util_eigen_defs INTERFACE)
target_include_directories(common_util_eigen_defs INTERFACE ${common_util_dir})
target_link_libraries(common_util_eigen_defs INTERFACE Eigen3::Eigen)

add_library(common_util_factory INTERFACE)
target_include_directories(common_util_factory INTERFACE ${common_util_dir})
target_link_libraries(common_util_factory INTERFACE cyber_common )

add_library(common_util_future INTERFACE)
target_include_directories(common_util_future INTERFACE ${common_util_dir})
target_link_libraries(common_util_future INTERFACE absl::optional)


add_library(common_util_json_util json_util.cc)
message("------------")
message("${nlohmann_json_include_dir}")
target_include_directories(common_util_json_util 
    PUBLIC 
        ${common_util_dir} 
        # ${protobuf_include_dir} 
        ${nlohmann_json_include_dir}
        )
target_link_libraries(common_util_json_util 
    PUBLIC 
        cyber_common 
        nlohmann_json::nlohmann_json 
        # ${protobuf_lib}
        )


add_library(common_util_lru_cache INTERFACE)
target_include_directories(common_util_lru_cache INTERFACE ${common_util_dir})

add_library(common_util_map_util INTERFACE)
target_include_directories(common_util_map_util INTERFACE ${common_util_dir} {protobuf_include_dir})
target_link_libraries(common_util_map_util INTERFACE ${protobuf_lib})

add_library(common_util_message_util INTERFACE)
target_include_directories(common_util_message_util INTERFACE ${common_util_dir})
target_link_libraries(common_util_message_util INTERFACE 
    cyber_common 
    cyber_time_clock 
    absl::strings
    ${protobuf_lib}
    )


add_library(common_util_perf_util perf_util.cc)
target_include_directories(common_util_perf_util PUBLIC ${common_util_dir})
target_link_libraries(common_util_perf_util PUBLIC 
    cyber_common_log
    cyber::base 
    cyber_time_time
    absl::strings
    # ${protobuf_lib}
)

add_library(common_util_string_util string_util.cc)
target_include_directories(common_util_string_util PUBLIC ${common_util_dir})
target_link_libraries(common_util_string_util 
PUBLIC 
    absl::str_format
)

add_library(common_util_point_factory INTERFACE)
target_include_directories(common_util_point_factory INTERFACE ${common_util_dir})
target_link_libraries(common_util_point_factory
INTERFACE 
    common_math_geometry
    common_pnc_point_proto
)

add_library(common_util_points_downsampler INTERFACE)
target_include_directories(common_util_points_downsampler INTERFACE ${common_util_dir})
target_link_libraries(common_util_points_downsampler
INTERFACE 
    cyber_common
    common_math_geometry
)

add_library(common_util_time_util INTERFACE)
target_include_directories(common_util_time_util INTERFACE ${common_util_dir})
target_link_libraries(common_util_time_util
INTERFACE 
    cyber_common_macros
)

add_library(common_util_util util.cc)
target_include_directories(common_util_util PUBLIC ${common_util_dir})
target_link_libraries(common_util_util
PUBLIC 
cyber_common_log
cyber_common_types
common_configs_config_gflags
common_math 
common_math_geometry
common_geometry_proto
common_pnc_point_proto
)





