project(planning_common)

add_library(planning_common_gflags planning_gflags.cc)
target_link_libraries(planning_common_gflags 
PUBLIC 
gflags
)