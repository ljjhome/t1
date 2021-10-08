/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/routing/topo_creator/graph_creator.h"

#include <vector>

#include "absl/strings/match.h"
#include "cyber/common/file.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/math_utils.h"
#include "modules/map/hdmap/adapter/opendrive_adapter.h"
#include "modules/routing/common/routing_gflags.h"
#include "modules/routing/topo_creator/edge_creator.h"
#include "modules/routing/topo_creator/node_creator.h"

namespace apollo {
namespace routing {

using apollo::common::PointENU;
using apollo::common::VehicleConfigHelper;
using apollo::common::math::kMathEpsilon;
using apollo::common::math::Vec2d;
using apollo::hdmap::Id;
using apollo::hdmap::LaneBoundary;
using apollo::hdmap::LaneBoundaryType;
using ::google::protobuf::RepeatedPtrField;

namespace {

bool IsAllowedToCross(const LaneBoundary& boundary) {
  for (const auto& boundary_type : boundary.boundary_type()) {
    if (boundary_type.types(0) != LaneBoundaryType::DOTTED_YELLOW &&
        boundary_type.types(0) != LaneBoundaryType::DOTTED_WHITE) {
      return false;
    }
  }
  return true;
}

}  // namespace

GraphCreator::GraphCreator(const std::string& base_map_file_path,
                           const std::string& dump_topo_file_path,
                           const RoutingConfig& routing_conf)
    : base_map_file_path_(base_map_file_path),
      dump_topo_file_path_(dump_topo_file_path),
      routing_conf_(routing_conf) {}

bool GraphCreator::Create() {
  if (absl::EndsWith(base_map_file_path_, ".xml")) {
    if (!hdmap::adapter::OpendriveAdapter::LoadData(base_map_file_path_,
                                                    &pbmap_)) {
      AERROR << "Failed to load base map file from " << base_map_file_path_;
      return false;
    }
  } else {
    if (!cyber::common::GetProtoFromFile(base_map_file_path_, &pbmap_)) {
      AERROR << "Failed to load base map file from " << base_map_file_path_;
      return false;
    }
  }

  // 注意看，这个就是用proto的好处，直接把很多接口写好了，里面有一个repeated lane 
  // 自然就能用lane_size()
  AINFO << "Number of lanes: " << pbmap_.lane_size();

  graph_.set_hdmap_version(pbmap_.header().version());
  graph_.set_hdmap_district(pbmap_.header().district());

  node_index_map_.clear();
  road_id_map_.clear();
  showed_edge_id_set_.clear();

  // 先对map中的每一个road进行循环
  for (const auto& road : pbmap_.road()) {
    // 在对road中的每一个section进行循环
    for (const auto& section : road.section()) {
      // 再对每个lane进行循环
      for (const auto& lane_id : section.lane_id()) {
        // 构造一个map，key是laneid，value是它所在的road id
        road_id_map_[lane_id.id()] = road.id().id();
      }
    }
  }

  // 把type不等于hdmap::Lane::CITY_DRIVING的lane id 放到禁止集合中
  InitForbiddenLanes();

  // 获得车辆最小转弯半径
  const double min_turn_radius =
      VehicleConfigHelper::GetConfig().vehicle_param().min_turn_radius();

  // 然后对每一个lane进行循环
  for (const auto& lane : pbmap_.lane()) {
    // 先记录lane id
    const auto& lane_id = lane.id().id();
    // 看看这个lane是否被禁止，如果被禁止了就continue到下一个lane
    if (forbidden_lane_id_set_.find(lane_id) != forbidden_lane_id_set_.end()) {
      ADEBUG << "Ignored lane id: " << lane_id
             << " because its type is NOT CITY_DRIVING.";
      continue;
    }
    // 看看当前这个lane的标签是不是uturn，并且是否符合车辆转弯半径
    if (lane.turn() == hdmap::Lane::U_TURN &&
        !IsValidUTurn(lane, min_turn_radius)) {
          // 如果不符合转弯半径要求，就continue到下一个lane
      ADEBUG << "The u-turn lane radius is too small for the vehicle to turn";
      continue;
    }
    // 输出当前的laneid
    AINFO << "Current lane id: " << lane_id;
    // graph当前有多少个节点，就把这数目作为index，构造到一个map中，key是laneid，value是对应的node id
    // 也就是说在graph中，一个lane是一个node
    node_index_map_[lane_id] = graph_.node_size();
    // 我们在刚才构造号的road_id_map_，去寻找，这个laneid 有没有对应的roadid
    // 因为刚刚我们的循环是从road开始从上往下找，这里是直接对所有的lane循环
    // 理论上每一个lane都在一个road里，但是这里还是给了判断，可以看一下，是不是说明，就算没有road也行？
    const auto iter = road_id_map_.find(lane_id);
    // 这个if在判断刚才找没找到对应的roadid
    // 从后面的代码可以看出road id 目前只是 node里的一个标签，还没有看到怎么用，目前有和没有都没有关系
    if (iter != road_id_map_.end()) {
      // 如果找到了，是一种构建node的方法
      node_creator::GetPbNode(lane, iter->second, routing_conf_,
                              graph_.add_node());
    } else {
      // 如果没找到，这里是另一种构建node的方法
      AWARN << "Failed to find road id of lane " << lane_id;
      node_creator::GetPbNode(lane, "", routing_conf_, graph_.add_node());
    }
  }

  /**
   * 上面是添加node，这里是添加edge
   * */
  // 首先也是针对lane进行循环
  for (const auto& lane : pbmap_.lane()) {
    // 取出当前lane 的id
    const auto& lane_id = lane.id().id();
    // 看看是不是在禁止的lane set中
    if (forbidden_lane_id_set_.find(lane_id) != forbidden_lane_id_set_.end()) {
      // 如果是就continue，不处理
      ADEBUG << "Ignored lane id: " << lane_id
             << " because its type is NOT CITY_DRIVING.";
      continue;
    }
    // 如果是一个正常的lane
    // 从graph中取出这个lane id 对应的node，作为from_node
    const auto& from_node = graph_.node(node_index_map_[lane_id]);
    // 添加边的函数
    /**
     * @param from_node 起始节点
     * @param lane.successor_id() 终止节点
     * @param Edge::FORWARD 边的类型
     * */
    AddEdge(from_node, lane.successor_id(), Edge::FORWARD);
    if (lane.length() < FLAGS_min_length_for_lane_change) {
      continue;
    }
    if (lane.has_left_boundary() && IsAllowedToCross(lane.left_boundary())) {
      AddEdge(from_node, lane.left_neighbor_forward_lane_id(), Edge::LEFT);
    }

    if (lane.has_right_boundary() && IsAllowedToCross(lane.right_boundary())) {
      AddEdge(from_node, lane.right_neighbor_forward_lane_id(), Edge::RIGHT);
    }
  }

  // 在这里判断一下到底要存成哪种文件，根据文件名判断
  if (!absl::EndsWith(dump_topo_file_path_, ".bin") &&
      !absl::EndsWith(dump_topo_file_path_, ".txt")) {
    AERROR << "Failed to dump topo data into file, incorrect file type "
           << dump_topo_file_path_;
    return false;
  }

  // 在这里把前面配置号的proto数据给保存成txt或者bin 文件
  auto type_pos = dump_topo_file_path_.find_last_of(".") + 1;
  std::string bin_file = dump_topo_file_path_.replace(type_pos, 3, "bin");
  std::string txt_file = dump_topo_file_path_.replace(type_pos, 3, "txt");
  if (!cyber::common::SetProtoToASCIIFile(graph_, txt_file)) {
    AERROR << "Failed to dump topo data into file " << txt_file;
    return false;
  }
  AINFO << "Txt file is dumped successfully. Path: " << txt_file;
  if (!cyber::common::SetProtoToBinaryFile(graph_, bin_file)) {
    AERROR << "Failed to dump topo data into file " << bin_file;
    return false;
  }
  AINFO << "Bin file is dumped successfully. Path: " << bin_file;
  return true;
}

/**
 * 这个函数用来将两个节点的id拼接成一个同一个id
 * */
std::string GraphCreator::GetEdgeID(const std::string& from_id,
                                    const std::string& to_id) {
  return from_id + "->" + to_id;
}

/**
 * 添加边的函数
 * */
void GraphCreator::AddEdge(const Node& from_node,
                           const RepeatedPtrField<Id>& to_node_vec,
                           const Edge::DirectionType& type) {
  // 一个lane 的successor可以有很多个，所以对每个都进行循环
  for (const auto& to_id : to_node_vec) {
    // 先看看这个succor lane是不是被禁止了，如果被禁止了，就直接continue到下一个successor
    if (forbidden_lane_id_set_.find(to_id.id()) !=
        forbidden_lane_id_set_.end()) {
      ADEBUG << "Ignored lane [id = " << to_id.id();
      continue;
    }

    // 先去拼接成一个统一的id
    const std::string edge_id = GetEdgeID(from_node.lane_id(), to_id.id());
    // 如果这个边之前被添加过，就直接continue
    if (showed_edge_id_set_.count(edge_id) != 0) {
      continue;
    }
    // 如果之前没有被添加过，这是第一次添加，就先将这个id添加的特定的set中
    showed_edge_id_set_.insert(edge_id);
    // 从graph中读出这个终点的id
    const auto& iter = node_index_map_.find(to_id.id());
    // 如果不在graph中，就continue
    if (iter == node_index_map_.end()) {
      continue;
    }
    // 从graph中取出这个node
    const auto& to_node = graph_.node(iter->second);
    // 在这个函数中构造edge
    edge_creator::GetPbEdge(from_node, to_node, type, routing_conf_,
                            graph_.add_edge());
  }
}

bool GraphCreator::IsValidUTurn(const hdmap::Lane& lane, const double radius) {
  if (lane.turn() != hdmap::Lane::U_TURN) {  // not a u-turn
    return false;
  }
  // approximate the radius from start point, middle point and end point.
  if (lane.central_curve().segment().empty() ||
      !lane.central_curve().segment(0).has_line_segment()) {
    return false;
  }
  std::vector<PointENU> points;
  for (const auto& segment : lane.central_curve().segment()) {
    points.insert(points.end(), segment.line_segment().point().begin(),
                  segment.line_segment().point().end());
  }
  if (points.empty()) {
    return false;
  }
  Vec2d p1{points.front().x(), points.front().y()};
  const auto& mid = points[points.size() / 2];
  Vec2d p2{mid.x(), mid.y()};
  Vec2d p3{points.back().x(), points.back().y()};
  Vec2d q1 = ((p1 + p2) / 2);                  // middle of p1---p2
  Vec2d q2 = (p2 - p1).rotate(M_PI / 2) + q1;  // perpendicular to p1-p2
  Vec2d q3 = ((p2 + p3) / 2);                  // middle of p2 -- p3
  Vec2d q4 = (p3 - p2).rotate(M_PI / 2) + q3;  // perpendicular to p2-p3
  const double s1 = CrossProd(q3, q1, q2);
  if (std::fabs(s1) < kMathEpsilon) {  // q3 is the circle center
    return q3.DistanceTo(p1) >= radius;
  }
  const double s2 = CrossProd(q4, q1, q2);
  if (std::fabs(s2) < kMathEpsilon) {  // q4 is the circle center
    return q4.DistanceTo(p1) >= radius;
  }
  if (std::fabs(s1 - s2) < kMathEpsilon) {  // parallel case, a wide u-turn
    return true;
  }
  Vec2d center = q3 + (q4 - q3) * s1 / (s1 - s2);
  return p1.DistanceTo(center) >= radius;
}

// 用来初始禁止的lane
// 如果lane的标签不是city driving，那么就把这个lane的id放到一个set中
void GraphCreator::InitForbiddenLanes() {
  // 对每一个lane进行循环
  for (const auto& lane : pbmap_.lane()) {
    // 判断lane 的type对不对
    if (lane.type() != hdmap::Lane::CITY_DRIVING) {
      // 不对就放到禁止集合当中
      forbidden_lane_id_set_.insert(lane.id().id());
    }
  }
}

}  // namespace routing
}  // namespace apollo
