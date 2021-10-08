project(cyber_gtest)

include_directories(${GTEST_INCLUDE_DIRS}
${protobuf_include_dir})
# link_directories(/home/ljj/program/protobuf_install/lib )
set(cyber_base_test_src 
        ../base/for_each_test.cc 
        ../base/atomic_hash_map_test.cc
        ../base/atomic_rw_lock_test.cc
        ../base/bounded_queue_test.cc
        ../base/object_pool_test.cc
        ../base/signal_test.cc
        ../base/unbounded_queue_test.cc
        )
add_executable(cyber_base_test  ${cyber_base_test_src})
target_link_libraries(cyber_base_test cyber::base ${GTEST_LIBRARIES} gtest_main pthread /usr/lib/gcc/x86_64-linux-gnu/7/libatomic.a)


set(cyber_common_test_src 
      ../common/environment_test.cc
      ../common/file_test.cc
      ../common/macros_test.cc
)
add_executable(cyber_common_test ${cyber_common_test_src})
target_link_libraries(cyber_common_test cyber_common ${GTEST_LIBRARIES} gmock  gtest_main pthread )

set(cyber_time_test_src 
      ../time/duration_test.cc 
      ../time/time_test.cc
      ../time/clock_test.cc
      )
add_executable(cyber_time_test ${cyber_time_test_src})
target_link_libraries(cyber_time_test 
            # ${GTEST_LIBRARIES} 
            # gmock  
            glog
            gtest_main
            cyber_time_duration 
            cyber_time_time
            cyber_time_clock
)


