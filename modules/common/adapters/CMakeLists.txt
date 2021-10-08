project(common_adapter)

add_subdirectory(proto)

add_library(common_adapter_gflags adapter_gflags.cc )
target_link_libraries(common_adapter_gflags 
PUBLIC 
gflags
)