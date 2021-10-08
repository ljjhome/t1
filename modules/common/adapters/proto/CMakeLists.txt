project(common_adapter_proto)

add_library(common_adapter_config_proto adapter_config.pb.cc )
target_include_directories(common_adapter_config_proto PUBLIC ${protobuf_include_dir})
target_link_libraries(common_adapter_config_proto
PUBLIC 
${protobuf_lib}
)
