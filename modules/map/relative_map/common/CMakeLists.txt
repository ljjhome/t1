project(map_relative_map_common_gflags)

add_library(map_relative_map_common_gflags relative_map_gflags.cc)
target_link_libraries(map_relative_map_common_gflags 
PUBLIC 
gflags
)