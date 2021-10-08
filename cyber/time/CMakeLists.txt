project(cyber_time)

set(cyber_time_dir ${PROJECT_SOURCE_DIR}/../..)
add_library(cyber_time_duration duration.cc)
target_include_directories(cyber_time_duration PUBLIC ${cyber_time_dir})

add_library(cyber_time_time time.cc)
target_include_directories(cyber_time_time PUBLIC ${cyber_time_dir})
target_link_libraries(cyber_time_time PUBLIC cyber_time_duration)

add_library(cyber_time_rate rate.cc)
target_include_directories(cyber_time_rate PUBLIC ${cyber_time_dir})
target_link_libraries(cyber_time_rate 
    PUBLIC 
        cyber_time_duration
        cyber_time_time
        cyber_common_log
    )

add_library(cyber_time_clock clock.cc)
target_include_directories(cyber_time_clock PUBLIC ${cyber_time_dir})
target_link_libraries(cyber_time_clock
    PUBLIC 
    cyber_time_time
    cyber::base 
    cyber_common
    cyber_proto
    )