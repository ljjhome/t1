project(adapter)

add_subdirectory(xml_parser)
add_subdirectory(osm_adapter)
add_library(map_hdmap_adapter_coordinate_convert_tool coordinate_convert_tool.cc )
target_include_directories(map_hdmap_adapter_coordinate_convert_tool PUBLIC ${PROJECT_SOURCE_DIR}/../../../..)
target_link_libraries(map_hdmap_adapter_coordinate_convert_tool 
PUBLIC 
map_hd_xml_status
/home/ljj/program/proj_install/lib/libproj.so
${glog_lib}

)

add_library(map_hdmap_adapter_proto_organizer proto_organizer.cc )
target_include_directories(map_hdmap_adapter_proto_organizer PUBLIC ${PROJECT_SOURCE_DIR}/../../../..)
target_link_libraries(map_hdmap_adapter_proto_organizer 
PUBLIC 
    cyber_common_log
    common_math
    common_util_util
    map_hd_xml_common_define
    map_proto
    absl::strings
)

add_library(map_hdmap_adapter_opendrive_adapter opendrive_adapter.cc )
target_include_directories(map_hdmap_adapter_opendrive_adapter PUBLIC ${PROJECT_SOURCE_DIR}/../../../..)
target_link_libraries(map_hdmap_adapter_opendrive_adapter 
PUBLIC 
map_hd_xml_parser
map_hdmap_adapter_coordinate_convert_tool
map_hdmap_adapter_proto_organizer
cyber_common_log

)