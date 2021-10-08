project(cyber_base)

add_library(cyber_base INTERFACE)
add_library(cyber::base ALIAS cyber_base)
target_include_directories(cyber_base
    INTERFACE
        ${PROJECT_SOURCE_DIR}/../..
)