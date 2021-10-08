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

#include "modules/routing/topo_creator/node_creator.h"

#include <algorithm>
#include <cmath>

namespace apollo {
namespace routing {
namespace node_creator {

namespace {

using ::google::protobuf::RepeatedPtrField;

using apollo::hdmap::Lane;
using apollo::hdmap::LaneBoundary;
using apollo::hdmap::LaneBoundaryType;

/**
 * 这个函数是判断给出的lane boundary type 是否允许换道，也就是“out”
 * */
bool IsAllowedOut(const LaneBoundaryType& type) {
  // 如果待判断的类型是这两种中的一种，就可以换道，否则就不可以out
  if (type.types(0) == LaneBoundaryType::DOTTED_YELLOW ||
      type.types(0) == LaneBoundaryType::DOTTED_WHITE) {
    return true;
  }
  return false;
}

double GetLengthbyRate(double cur_s, double cur_total_length,
                       double target_length) {
  double new_length = cur_s / cur_total_length * target_length;
  return std::min(new_length, target_length);
}

double GetLaneLength(const Lane& lane) {
  double length = 0.0;
  for (const auto& segment : lane.central_curve().segment()) {
    length += segment.length();
  }
  return length;
}

/**
 * 这个函数给出了一个lane boundary上能够换道，out 的位置
 * 用中心线的s坐标start s 和end s 表示
 * */
void AddOutBoundary(const LaneBoundary& bound, double lane_length,
                    RepeatedPtrField<CurveRange>* const out_range) {
  // 一条lane上可以有不同的boundary type，所以这里先对所有的type进行循环
  for (int i = 0; i < bound.boundary_type_size(); ++i) {
    // 判断一下当前这个type能否换道，不能就continue到下一个类型
    if (!IsAllowedOut(bound.boundary_type(i))) {
      continue;
    }
    // 如果可以
    // 分配一个outrange内存
    CurveRange* range = out_range->Add();
    // 这个range的set_s这个量给出的是，在boundary线上的s，不是中心线的s，一定要注意
    // 上面这句话错了：应该是set_s是中心线为坐标的s，而bound.boundary_type(i).s()和bound.length()是以边线为坐标系的s
    range->mutable_start()->set_s(GetLengthbyRate(bound.boundary_type(i).s(),
                                                  bound.length(), lane_length));
    // 看看当前的这个type是不是边界上的最后一个type
    if (i != bound.boundary_type_size() - 1) {
      // 如果不是，就把下一个type 的起点设成end s
      range->mutable_end()->set_s(GetLengthbyRate(
          bound.boundary_type(i + 1).s(), bound.length(), lane_length));
    } else {
      // 如果是最后一个type，就把lane中心线长度设成end s
      range->mutable_end()->set_s(lane_length);
    }
  }
}

/**
 * 初始化node info
 * @param lane 路
 * @param road_id 路对应的road id
 * @param node 要输出的node
 * */
void InitNodeInfo(const Lane& lane, const std::string& road_id,
                  Node* const node) {
  
  // 计算这个lane 的长度
  // 可以看到，这个函数中依然是通过for循环进行累加
  double lane_length = GetLaneLength(lane);
  // node 中的变量设置
  // node 包含道路id
  node->set_lane_id(lane.id().id());
  // node 包含road  id
  node->set_road_id(road_id);
  // 这个是啥进去看看
  /**
   * @param lane.left_boundary() lane 的左边界线
   * @param lane_length lane的长度
   * @param  node->mutable_left_out() 要输出的结构
   * */
  // 处理左边线和右边线
  AddOutBoundary(lane.left_boundary(), lane_length, node->mutable_left_out());
  AddOutBoundary(lane.right_boundary(), lane_length, node->mutable_right_out());

  node->set_length(lane_length);
  node->mutable_central_curve()->CopyFrom(lane.central_curve());
  node->set_is_virtual(true);
  // 如果这个lane不在在junction中，或者左右侧有相邻的车道，那这个就不是一个virtual的，否则就是virtual的
  if (!lane.has_junction_id() ||
      lane.left_neighbor_forward_lane_id_size() > 0 ||
      lane.right_neighbor_forward_lane_id_size() > 0) {
    node->set_is_virtual(false);
  }
}

// 设置对应节点的cost，就是一个数，先不管
void InitNodeCost(const Lane& lane, const RoutingConfig& routing_config,
                  Node* const node) {
  double lane_length = GetLaneLength(lane);
  double speed_limit =
      lane.has_speed_limit() ? lane.speed_limit() : routing_config.base_speed();
  double ratio = speed_limit >= routing_config.base_speed()
                     ? std::sqrt(routing_config.base_speed() / speed_limit)
                     : 1.0;
  double cost = lane_length * ratio;
  if (lane.has_turn()) {
    if (lane.turn() == Lane::LEFT_TURN) {
      cost += routing_config.left_turn_penalty();
    } else if (lane.turn() == Lane::RIGHT_TURN) {
      cost += routing_config.right_turn_penalty();
    } else if (lane.turn() == Lane::U_TURN) {
      cost += routing_config.uturn_penalty();
    }
  }
  node->set_cost(cost);
}

}  // namespace

/**
 * 这个是构造node的函数
 * @param lane 就是hdmap里的lane
 * @param road_id 就是lane 所在的road 的id
 * @param routingconfig 配置
 * @param node 最终输出的node
 * */
void GetPbNode(const hdmap::Lane& lane, const std::string& road_id,
               const RoutingConfig& routingconfig, Node* const node) {
  InitNodeInfo(lane, road_id, node);
  InitNodeCost(lane, routingconfig, node);
}

}  // namespace node_creator
}  // namespace routing
}  // namespace apollo
