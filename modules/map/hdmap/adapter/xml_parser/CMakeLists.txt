project(xml_parser)

set(xml_parser_dir ${PROJECT_SOURCE_DIR}/../../../../..)

add_library(map_hd_xml_common_define INTERFACE )
target_include_directories(map_hd_xml_common_define INTERFACE ${xml_parser_dir})
target_link_libraries(map_hd_xml_common_define INTERFACE cyber_common_log map_proto )

add_library(map_hd_xml_status INTERFACE )
target_include_directories(map_hd_xml_status INTERFACE ${xml_parser_dir})
target_link_libraries(map_hd_xml_status INTERFACE common_status )

add_library(map_hd_xml_util_xml_parser util_xml_parser.cc )
target_include_directories(map_hd_xml_util_xml_parser PUBLIC ${xml_parser_dir})
target_link_libraries(map_hd_xml_util_xml_parser 
PUBLIC 
    map_hd_xml_common_define
    map_hd_xml_status
    cyber_common_log
    map_hdmap_adapter_coordinate_convert_tool
    apollo_tinyxml2
)


add_library(map_hd_xml_header_xml_parser header_xml_parser.cc )
target_include_directories(map_hd_xml_header_xml_parser PUBLIC ${xml_parser_dir})
target_link_libraries(map_hd_xml_header_xml_parser 
PUBLIC 
    map_hd_xml_common_define
    map_hd_xml_status
    map_hdmap_adapter_coordinate_convert_tool
    absl::strings
    apollo_tinyxml2
)

add_library(map_hd_xml_junctions_xml_parser junctions_xml_parser.cc )
target_include_directories(map_hd_xml_junctions_xml_parser PUBLIC ${xml_parser_dir})
target_link_libraries(map_hd_xml_junctions_xml_parser 
PUBLIC 
    map_hd_xml_common_define
    map_hd_xml_status
    map_hd_xml_util_xml_parser
    apollo_tinyxml2
)

add_library(map_hd_xml_lanes_xml_parser lanes_xml_parser.cc )
target_include_directories(map_hd_xml_lanes_xml_parser PUBLIC ${xml_parser_dir})
target_link_libraries(map_hd_xml_lanes_xml_parser 
PUBLIC 
    map_hd_xml_common_define
    map_hd_xml_status
    map_hd_xml_util_xml_parser
    apollo_tinyxml2
)

add_library(map_hd_xml_objects_xml_parser objects_xml_parser.cc )
target_include_directories(map_hd_xml_objects_xml_parser PUBLIC ${xml_parser_dir})
target_link_libraries(map_hd_xml_objects_xml_parser 
PUBLIC 
    map_hd_xml_common_define
    map_hd_xml_status
    map_hd_xml_util_xml_parser
    apollo_tinyxml2
)

add_library(map_hd_xml_roads_xml_parser roads_xml_parser.cc )
target_include_directories(map_hd_xml_roads_xml_parser PUBLIC ${xml_parser_dir})
target_link_libraries(map_hd_xml_roads_xml_parser 
PUBLIC 
    map_hd_xml_common_define
    map_hd_xml_lanes_xml_parser
    map_hd_xml_objects_xml_parser
    map_hd_xml_signals_xml_parser
    map_hd_xml_status
    map_hd_xml_util_xml_parser
    apollo_tinyxml2
)

add_library(map_hd_xml_signals_xml_parser signals_xml_parser.cc )
target_include_directories(map_hd_xml_signals_xml_parser PUBLIC ${xml_parser_dir})
target_link_libraries(map_hd_xml_signals_xml_parser 
PUBLIC 
    map_hd_xml_common_define
    map_hd_xml_status
    map_hd_xml_util_xml_parser
    apollo_tinyxml2
)

add_library(map_hd_xml_parser INTERFACE)
target_link_libraries(map_hd_xml_parser
INTERFACE 
map_hd_xml_header_xml_parser
map_hd_xml_junctions_xml_parser
map_hd_xml_lanes_xml_parser
map_hd_xml_objects_xml_parser
map_hd_xml_roads_xml_parser
map_hd_xml_signals_xml_parser
map_hd_xml_status
)







#----------------test for tinyxml2----------------
add_executable(xmltest xmltest.cpp)
target_link_libraries(xmltest PUBLIC apollo_tinyxml2)

