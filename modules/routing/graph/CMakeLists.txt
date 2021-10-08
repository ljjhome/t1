project(routing_graph)

add_library(routing_graph_topo_range topo_range.cc )
target_link_libraries(routing_graph_topo_range 
PUBLIC 
routing_common_gflags
)

add_library(routing_graph_range_utils INTERFACE )
target_include_directories(routing_graph_range_utils INTERFACE ${CMAKE_SOURCE_DIR}/../../..)

add_library(routing_graph_topo_node topo_node.cc )
target_link_libraries(routing_graph_topo_node 
PUBLIC 
routing_graph_range_utils
routing_graph_topo_range
cyber_common_log
common_util_map_util
routing_proto
routing_topo_graph_proto

)

add_library(routing_graph_topo_range_manager topo_range_manager.cc )
target_link_libraries(routing_graph_topo_range_manager
PUBLIC 
routing_graph_topo_node
)

add_library(routing_graph_node_with_range node_with_range.cc )
target_link_libraries(routing_graph_node_with_range 
PUBLIC 
routing_graph_topo_node
)

add_library(routing_graph_topo_graph topo_graph.cc )
target_link_libraries(routing_graph_topo_graph 
PUBLIC 
routing_graph_topo_node
)

add_library(routing_graph_sub_topo_graph sub_topo_graph.cc )
target_link_libraries(routing_graph_sub_topo_graph 
PUBLIC 
routing_graph_node_with_range
)

add_library(routing_graph_topo_test_utils topo_test_utils.cc )
target_link_libraries(routing_graph_topo_test_utils
PUBLIC 
routing_graph_topo_graph
)

add_library(routing_graph INTERFACE )
target_link_libraries(routing_graph 
INTERFACE 
routing_graph_sub_topo_graph
routing_graph_topo_graph
routing_graph_topo_range_manager
)

