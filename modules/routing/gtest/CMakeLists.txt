project(routing_gtest)

set(routing_graph_test_src 
    ../graph/topo_node_test.cc 
    ../graph/topo_range_test.cc
    ../graph/topo_graph_test.cc
    ../graph/sub_topo_graph_test.cc
    ../topo_creator/graph_creator_test.cc 
)
add_executable(routing_graph_test ${routing_graph_test_src} )
target_link_libraries(routing_graph_test 
PUBLIC  
routing_graph_topo_test_utils
routing_graph_node_with_range
routing_graph_sub_topo_graph
topo_creator_graph_creator
gtest_main 
)
