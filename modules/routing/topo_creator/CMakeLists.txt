project(routing_topo_creator)

add_library(topo_creator_edge_creator edge_creator.cc )
target_link_libraries(topo_creator_edge_creator
PUBLIC 
routing_common_gflags
routing_proto
routing_config_proto
routing_topo_graph_proto
)

add_library(topo_creator_node_creator node_creator.cc )
target_link_libraries(topo_creator_node_creator 
PUBLIC 
map_lane_proto
routing_proto
routing_config_proto
routing_topo_graph_proto
)

add_library(topo_creator_graph_creator graph_creator.cc )
target_link_libraries(topo_creator_graph_creator
PUBLIC 
topo_creator_edge_creator
topo_creator_node_creator
common_configs_vehicle_config_helper
map_hdmap_adapter_opendrive_adapter
)

add_executable(topo_creator topo_creator.cc )
target_link_libraries(topo_creator
PUBLIC 
topo_creator_graph_creator
map_hdmap_util
)

