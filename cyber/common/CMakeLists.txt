project(cyber_common)

# link_directories(/home/ljj/program/protobuf_install/lib 
#                     /usr/lib/x86_64-linux-gnu)
# include_directories(/home/ljj/program/protobuf_install/include)
add_library(cyber_common_file file.cc)
target_include_directories(cyber_common_file PUBLIC ${PROJECT_SOURCE_DIR}/../..
${protobuf_include_dir})
target_link_libraries(cyber_common_file ${protobuf_lib} )

add_library(cyber_common_util INTERFACE)
target_include_directories(cyber_common_util INTERFACE ${PROJECT_SOURCE_DIR}/../..)

add_library(cyber_common_types INTERFACE)
target_include_directories(cyber_common_types INTERFACE ${PROJECT_SOURCE_DIR}/../..)

add_library(cyber_common_time_conversion INTERFACE)
target_include_directories(cyber_common_time_conversion INTERFACE ${PROJECT_SOURCE_DIR}/../..)

add_library(cyber_common_macros INTERFACE)
target_include_directories(cyber_common_macros INTERFACE ${PROJECT_SOURCE_DIR}/../..)
target_link_libraries(cyber_common_macros INTERFACE cyber::base)

add_library(cyber_common_log INTERFACE)
target_include_directories(cyber_common_log INTERFACE ${PROJECT_SOURCE_DIR}/../..)
target_link_libraries(cyber_common_log INTERFACE cyber::binary glog)


add_library(cyber_common_environment INTERFACE)
target_include_directories(cyber_common_environment INTERFACE ${PROJECT_SOURCE_DIR}/../..)
target_link_libraries(cyber_common_environment INTERFACE cyber_common_log)


add_library(cyber_common_global_data global_data.cc)
target_include_directories(cyber_common_global_data INTERFACE ${PROJECT_SOURCE_DIR}/../..)
target_link_libraries(cyber_common_global_data 
                            cyber::base 
                            cyber_common_file
                            cyber_common_util
                            cyber_common_macros
                            cyber_common_environment
                            cyber_proto)

add_library(cyber_common INTERFACE)
target_link_libraries(cyber_common
    INTERFACE
    cyber_common_file
    cyber_common_util
    cyber_common_types
    cyber_common_time_conversion
    cyber_common_macros
    cyber_common_log
    cyber_common_environment
    cyber_common_global_data
    )
