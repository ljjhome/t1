project(hdmap)
add_subdirectory(adapter)

set(map_hdmap_dir ${PROJECT_SOURCE_DIR}/../../..)


set(map_hdmap_src 
    hdmap.cc 
    hdmap_common.cc
    hdmap_impl.cc   
)

add_library(map_hdmap ${map_hdmap_src} )
target_include_directories(map_hdmap PUBLIC ${map_hdmap_dir})
target_link_libraries(map_hdmap 
PUBLIC 
    cyber_common_file
    cyber_common_macros
    common_configs_config_gflags
    common_math 
    common_math_linear_interpolation
    common_util_util
    map_hdmap_adapter_opendrive_adapter
    map_proto
    map_relative_map_navigation_proto
    absl::base
    absl::strings
    ${glog_lib}
)

add_library(map_hdmap_util hdmap_util.cc )
target_include_directories(map_hdmap_util PUBLIC ${map_hdmap_dir})
target_link_libraries(map_hdmap_util 
PUBLIC
map_hdmap
cyber_common_file 
common_configs_config_gflags
map_relative_map_navigation_proto
absl::strings 
)