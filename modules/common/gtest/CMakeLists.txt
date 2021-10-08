project(common_gtest)

set(common_util_test_src
    ../util/factory_test.cc
    
    ../util/lru_cache_test.cc 
)
add_executable(common_util_test ${common_util_test_src})
target_include_directories(common_util_test PRIVATE ${PROJECT_SOURCE_DIR}/../../.. )
target_link_libraries(common_util_test 
        ${glog_lib} ${GTEST_LIBRARIES} 
        gmock 
        gtest_main 
        cyber::binary 
        common_proto 
        common_util_lru_cache
        )

set(common_util_json_test_src 
        ../util/json_util_test.cc
        )
add_executable(common_util_json_test ${common_util_json_test_src})
target_include_directories(common_util_json_test PUBLIC ${PROJECT_SOURCE_DIR}/../../..)
target_link_libraries(common_util_json_test 
        # ${glog_lib} 
        # ${GTEST_LIBRARIES} 
        # ${gmock_lib} 
        # ${gtest_lib}
        
        # ${gtest_main_lib}
        gmock
        gtest_main
        common_error_code_proto 
        common_util_json_util
        # pthread
        # common_util_lru_cache
        )

set(common_util_message_test_src 
        ../util/message_util_test.cc )
add_executable(common_util_message_test ${common_util_message_test_src})
target_include_directories(common_util_message_test PUBLIC ${PROJECT_SOURCE_DIR}/../../..)
target_link_libraries(common_util_message_test 
        common_util_testdata_proto 
        common_util_message_util
        gtest_main
        )


set(common_util_perf_test_src 
        ../util/perf_util_test.cc )
add_executable(common_util_perf_test ${common_util_perf_test_src})
target_include_directories(common_util_perf_test PUBLIC ${PROJECT_SOURCE_DIR}/../../..)
target_link_libraries(common_util_perf_test 
        common_util_perf_util
        gtest_main
        )

set(common_util_string_test_src 
        ../util/string_util_test.cc )
add_executable(common_util_string_test ${common_util_string_test_src})
target_include_directories(common_util_string_test PUBLIC ${PROJECT_SOURCE_DIR}/../../..)
target_link_libraries(common_util_string_test 
        common_util_string_util
        gtest_main
)

set(common_util_combine_test_src 
        ../util/points_downsampler_test.cc 
        ../util/time_util_test.cc
        ../util/util_test.cc
)
add_executable(common_util_combine_test ${common_util_combine_test_src})
target_include_directories(common_util_combine_test PUBLIC ${PROJECT_SOURCE_DIR}/../../..)
target_link_libraries(common_util_combine_test 
        common_util_points_downsampler
        common_util_time_util
        common_util_util
        common_util_testdata_proto
        gtest_main
)






#_---------------common math test
set(common_math_test_src 
        ../math/vec2d_test.cc 
        ../math/angle_test.cc
        ../math/euler_angles_zxy_test.cc
        ../math/quaternion_test.cc
        ../math/matrix_operations_test.cc
        ../math/search_test.cc
        ../math/kalman_filter_test.cc
        ../math/linear_interpolation_test.cc
        ../math/mpc_osqp_test.cc
        ../math/integral_test.cc
        ../math/cartesian_frenet_conversion_test.cc
        ../math/math_utils_test.cc
        ../math/aabox2d_test.cc
        ../math/aaboxkdtree2d_test.cc
        ../math/box2d_test.cc
        ../math/polygon2d_test.cc
        ../math/line_segment2d_test.cc
)
add_executable(common_math_test ${common_math_test_src})
target_include_directories(common_math_test PUBLIC ${PROJECT_SOURCE_DIR}/../../..)
target_link_libraries(common_math_test 
PUBLIC 
        common_math_geometry 
        common_math_angle
        common_math_euler_angle_zxy
        common_math_quaternion
        common_math_matrix_operation
        common_math_search
        common_math_kalman_filter
        common_math_linear_interpolation
        common_math_mpc_osqp
        cyber_common_log

        common_math_integral
        common_math_cartesian_frenet_conversion

        gtest_main
)


#--------------- common configs test---------------------
set(common_configs_test_src 
        ../configs/vehicle_config_helper_test.cc 
)
add_executable(common_configs_test ${common_configs_test_src})
target_include_directories(common_configs_test PUBLIC ${PROJECT_SOURCE_DIR}/../../..)
target_link_libraries(common_configs_test 
PUBLIC
        common_configs_vehicle_config_helper
        gmock
        # ${gmock_lib}
        gtest_main
)


#----------------- common status test ------------------
set(common_status_test_src 
        ../status/status_test.cc )
add_executable(common_status_test ${common_status_test_src})
target_include_directories(common_status_test PUBLIC ${PROJECT_SOURCE_DIR}/../../..)
target_link_libraries(common_status_test 
PUBLIC 
common_status
gtest_main
)

#------------common vehocle state provider test----------
add_executable(common_vehicle_state_test ../vehicle_state/vehicle_state_provider_test.cc )
target_link_libraries(common_vehicle_state_test
PUBLIC 
common_vehicle_state_provider
common_util_util
localization_proto
cyber_common_file
gtest_main
)
