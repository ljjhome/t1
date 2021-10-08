project(cyber)


add_subdirectory(proto)

add_subdirectory(base)

# # cyber::binary
add_library(binary binary.cc)
target_include_directories(binary PUBLIC ${PROJECT_SOURCE_DIR}/..)
add_library(cyber::binary ALIAS binary)


add_subdirectory(common)

add_subdirectory(time)


add_subdirectory(gtest)


