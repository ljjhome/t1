project(routing_proto)

set(routing_proto_dir ${PROJECT_SOURCE_DIR}/../../..)
add_library(routing_proto routing.pb.cc)
target_include_directories(routing_proto PUBLIC ${protobuf_include_dir} ${routing_proto_dir})
target_link_libraries(routing_proto 
PUBLIC 
    ${protobuf_lib}
    common_error_code_proto
    common_geometry_proto
    common_header_proto
    map_parking_space_proto
    map_geometry_proto
    
    # /home/ljj/program/protobuf_install/lib/libprotobuf.so 
)

add_library(routing_poi_proto poi.pb.cc)
target_include_directories(routing_poi_proto PUBLIC ${protobuf_include_dir} ${routing_proto_dir})
target_link_libraries(routing_poi_proto 
PUBLIC 
    ${protobuf_lib}
    # /home/ljj/program/protobuf_install/lib/libprotobuf.so 
    routing_proto
)

add_library(routing_config_proto routing_config.pb.cc)
target_include_directories(routing_config_proto PUBLIC ${protobuf_include_dir} ${routing_proto_dir})
target_link_libraries(routing_config_proto 
PUBLIC 
    ${protobuf_lib}
    # /home/ljj/program/protobuf_install/lib/libprotobuf.so 
)

add_library(routing_topo_graph_proto topo_graph.pb.cc)
target_include_directories(routing_topo_graph_proto PUBLIC /home/ljj/program/protobuf_install/include ${protobuf_include_dir} ${routing_proto_dir})
target_link_libraries(routing_topo_graph_proto 
PUBLIC 
    ${protobuf_lib}
    /home/ljj/program/protobuf_install/lib/libprotoc.so
    map_geometry_proto
    # /home/ljj/program/protobuf_install/lib/libprotobuf.so 
    
)




