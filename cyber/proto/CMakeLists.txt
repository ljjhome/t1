project(cyber_proto)

set(cyber_proto_src 
        choreography_conf.pb.cc 
        classic_conf.pb.cc 
        clock.pb.cc 
        component_conf.pb.cc 
        cyber_conf.pb.cc 
        dag_conf.pb.cc 
        parameter.pb.cc 
        perf_conf.pb.cc 
        proto_desc.pb.cc 
        qos_profile.pb.cc 
        record.pb.cc 
        role_attributes.pb.cc 
        run_mode_conf.pb.cc 
        scheduler_conf.pb.cc 
        topology_change.pb.cc 
        transport_conf.pb.cc 
        unit_test.pb.cc 
        )
add_library(cyber_proto ${cyber_proto_src})
target_include_directories(cyber_proto PUBLIC
                                ${PROJECT_SOURCE_DIR}/../..
                                ${protobuf_include_dir}
                                )
target_link_libraries(cyber_proto PUBLIC
        ${protobuf_lib} )