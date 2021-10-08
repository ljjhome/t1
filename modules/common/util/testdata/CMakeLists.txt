project(common_util_testdata)
add_library(common_util_testdata_proto simple.pb.cc)
target_include_directories(common_util_testdata_proto PUBLIC ${PROJECT_SOURCE_DIR}/../../../.. ${protobuf_include_dir})
target_link_libraries(common_util_testdata_proto 
PUBLIC 
common_header_proto
${protobuf_lib}
)