project(map_gtest)

set(map_gtest_dir ${PROJECT_SOURCE_DIR}/../../..)

set(map_hdmap_test_src 
    ../hdmap/hdmap_common_test.cc
    ../hdmap/hdmap_impl_test.cc
    ../hdmap/hdmap_util_test.cc
)
add_executable(map_hdmap_test ${map_hdmap_test_src})
target_include_directories(map_hdmap_test PUBLIC ${map_gtest_dir})
target_link_libraries(map_hdmap_test
PUBLIC 
    map_hdmap
    map_hdmap_util
    cyber_common_file
    common_util_testdata_proto
    ${glog_lib}
    gtest_main
)

set(map_pnc_map_test_src 
    ../pnc_map/path_test.cc 
    ../pnc_map/route_segments_test.cc
)
add_executable(map_pnc_map_test ${map_pnc_map_test_src})
target_include_directories(map_pnc_map_test PUBLIC ${map_gtest_dir})
target_link_libraries(map_pnc_map_test
PUBLIC 
    map_pnc_map_route_segments
    map_pnc_map_path
    common_util_util
    routing_proto
    gtest_main
)

set(map_pnc_pncmap_test_src 
    ../pnc_map/pnc_map_test.cc
)
add_executable(map_pnc_pncmap_test ${map_pnc_pncmap_test_src})
target_include_directories(map_pnc_pncmap_test PUBLIC ${map_gtest_dir})
target_link_libraries(map_pnc_pncmap_test
PUBLIC 
    map_pnc_map
    common_util_util
    gtest_main
)

set(map_relative_map_test_src 
    ../relative_map/navigation_lane_test.cc 
)
add_executable(map_relative_map_test ${map_relative_map_test_src})
target_include_directories(map_relative_map_test PUBLIC ${nlohmann_json_include_dir})
target_link_libraries(map_relative_map_test
PUBLIC
nlohmann_json::nlohmann_json 
map_relative_map_navigation_lane_lib
cyber_common_file
gtest_main
)