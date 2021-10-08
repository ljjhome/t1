project(common_status)

add_library(common_status INTERFACE)
target_include_directories(common_status INTERFACE ${PROJECT_SOURCE_DIR}/../../..)
target_link_libraries(common_status 
INTERFACE 
common_error_code_proto
common_util_future
${protobuf_lib}
)