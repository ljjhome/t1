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

#include "modules/routing/core/black_list_range_generator.h"

namespace apollo {
namespace routing {

constexpr double S_GAP_FOR_BLACK = 0.01;

namespace {
// 在保证s比最大值小的情况下，稍微往s增大的方向移动一点点，为啥？
double MoveSForward(double s, double upper_bound) {
  if (s > upper_bound) {
    AERROR << "Illegal s: " << s << ", upper bound: " << upper_bound;
    return s;
  }
  if (s + S_GAP_FOR_BLACK < upper_bound) {
    return (s + S_GAP_FOR_BLACK);
  } else {
    return ((s + upper_bound) / 2.0);
  }
}
// 在保证s比0大的情况下，稍微往s减小的方向移动一点点，为啥？
double MoveSBackward(double s, double lower_bound) {
  if (s < lower_bound) {
    AERROR << "Illegal s: " << s << ", lower bound: " << lower_bound;
    return s;
  }
  if (s - S_GAP_FOR_BLACK > lower_bound) {
    return (s - S_GAP_FOR_BLACK);
  } else {
    return ((s + lower_bound) / 2.0);
  }
}

void GetOutParallelLane(const TopoNode* node,
                        std::unordered_set<const TopoNode*>* const node_set) {
  for (const auto* edge : node->OutToLeftOrRightEdge()) {
    const auto* to_node = edge->ToNode();
    if (node_set->count(to_node) == 0) {
      node_set->emplace(to_node);
      GetOutParallelLane(to_node, node_set);
    }
  }
}

void GetInParallelLane(const TopoNode* node,
                       std::unordered_set<const TopoNode*>* const node_set) {
  for (const auto* edge : node->InFromLeftOrRightEdge()) {
    const auto* from_node = edge->FromNode();
    if (node_set->count(from_node) == 0) {
      node_set->emplace(from_node);
      GetInParallelLane(from_node, node_set);
    }
  }
}

// for new navigator
// request的黑名单，既有lane 的黑名单也有road的黑名单。添加完lane添加road，添加road，起始也是找出lane来
void AddBlackMapFromRoad(const RoutingRequest& request, const TopoGraph* graph,
                         TopoRangeManager* const range_manager) {
  for (const auto& road_id : request.blacklisted_road()) {
    std::unordered_set<const TopoNode*> road_nodes_set;
    graph->GetNodesByRoadId(road_id, &road_nodes_set);
    for (const auto& node : road_nodes_set) {
      range_manager->Add(node, 0.0, node->Length());
    }
  }
}

// for new navigator
void AddBlackMapFromLane(const RoutingRequest& request, const TopoGraph* graph,
                         TopoRangeManager* const range_manager) {

  // 对request中所有的黑名单lane进行循环
  for (const auto& lane : request.blacklisted_lane()) {
    // 获得他们对应的节点
    const auto* node = graph->GetNode(lane.id());
    if (node) {
      // 如果node存在，添加到range manager当中
      range_manager->Add(node, lane.start_s(), lane.end_s());
    }
  }
}

// 这个函数，包括下面那个函数，目的是
// 添加当前这个node周围的node，out函数是指添加当前node能到达的node
// in这个函数是添加能到达当前这个node的node
void AddBlackMapFromOutParallel(const TopoNode* node, double cut_ratio,
                                TopoRangeManager* const range_manager) {
  std::unordered_set<const TopoNode*> par_node_set;
  GetOutParallelLane(node, &par_node_set);
  par_node_set.erase(node);
  for (const auto* par_node : par_node_set) {
    double par_cut_s = cut_ratio * par_node->Length();
    range_manager->Add(par_node, par_cut_s, par_cut_s);
  }
}

void AddBlackMapFromInParallel(const TopoNode* node, double cut_ratio,
                               TopoRangeManager* const range_manager) {
  std::unordered_set<const TopoNode*> par_node_set;
  GetInParallelLane(node, &par_node_set);
  par_node_set.erase(node);
  for (const auto* par_node : par_node_set) {
    double par_cut_s = cut_ratio * par_node->Length();
    range_manager->Add(par_node, par_cut_s, par_cut_s);
  }
}

}  // namespace

void BlackListRangeGenerator::GenerateBlackMapFromRequest(
    const RoutingRequest& request, const TopoGraph* graph,
    TopoRangeManager* const range_manager) const {
  AddBlackMapFromLane(request, graph, range_manager);
  AddBlackMapFromRoad(request, graph, range_manager);
  range_manager->SortAndMerge();
}

void BlackListRangeGenerator::AddBlackMapFromTerminal(
    const TopoNode* src_node, const TopoNode* dest_node, double start_s,
    double end_s, TopoRangeManager* const range_manager) const {
  // 记录起始节点的长度
  double start_length = src_node->Length();
  // 记录终止节点的长度
  double end_length = dest_node->Length();

  static constexpr double kEpsilon = 1e-2;
  // 这些是因为进位精度在调整么？
  const double start_s_adjusted =
      (start_s > start_length && start_s - start_length <= kEpsilon) ?
          start_length : start_s;
  const double end_s_adjusted =
      (end_s > end_length && end_s - end_length <= kEpsilon) ?
          end_length : end_s;
  // 有可能，是因为精度问题
  // 总之在下面通过两个判断，来保证s的值一定是大于0并且小于lane长度，因为不可能大于lane的长度
  if (start_s_adjusted < 0.0 || start_s_adjusted > start_length) {
    AERROR << "Illegal start_s: " << start_s << ", length: " << start_length;
    return;
  }
  if (end_s_adjusted < 0.0 || end_s_adjusted > end_length) {
    AERROR << "Illegal end_s: " << end_s << ", length: " << end_length;
    return;
  }

  double start_cut_s = MoveSBackward(start_s_adjusted, 0.0);
  // 在这里add的时候start s 和 end s是一样的？
  // 这些黑名单range都只是长度为0的四边形，太奇怪了
  
  range_manager->Add(src_node, start_cut_s, start_cut_s);
  AddBlackMapFromOutParallel(src_node, start_cut_s / start_length,
                             range_manager);

  double end_cut_s = MoveSForward(end_s_adjusted, end_length);
  range_manager->Add(dest_node, end_cut_s, end_cut_s);
  AddBlackMapFromInParallel(dest_node, end_cut_s / end_length, range_manager);
  range_manager->SortAndMerge();
}

}  // namespace routing
}  // namespace apollo
