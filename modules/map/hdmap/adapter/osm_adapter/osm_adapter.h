#pragma once

#include <string>

#include "modules/map/proto/map.pb.h"
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingGraph.h>
namespace apollo {
namespace hdmap {
namespace adapter {

class OSMAdapter {
 public:
  OSMAdapter();
  bool LoadData(apollo::hdmap::Map* pb_map);
private:
lanelet::LaneletMapPtr map;
lanelet::projection::UtmProjector::Ptr utm;
lanelet::routing::RoutingGraphPtr routing_graph_ptr;
};

}  // namespace adapter
}  // namespace hdmap
}  // namespace apollo
