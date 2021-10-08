project(apollo_tinyxml2)
add_library(apollo_tinyxml2 tinyxml2.cpp)
target_include_directories(apollo_tinyxml2 PUBLIC ${PROJECT_SOURCE_DIR})