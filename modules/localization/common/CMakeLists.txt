project(localization_common)

add_library(localization_common_gflags localization_gflags.cc )
target_link_libraries(localization_common_gflags
PUBLIC 
gflags 
)
