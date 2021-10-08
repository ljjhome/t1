project(routing_common)

add_library(routing_common_gflags routing_gflags.cc )
target_link_libraries(routing_common_gflags
PUBLIC 
gflags 
)