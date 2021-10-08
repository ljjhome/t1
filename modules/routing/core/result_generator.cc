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
#include "modules/routing/core/result_generator.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include "cyber/common/log.h"
#include "modules/common/util/map_util.h"

namespace apollo {
namespace routing {

using apollo::common::util::ContainsKey;

bool IsCloseEnough(double value_1, double value_2) {
  static constexpr double kEpsilon = 1e-6;
  return std::fabs(value_1 - value_2) < kEpsilon;
}

const NodeWithRange& GetLargestRange(
    const std::vector<NodeWithRange>& node_vec) {
  ACHECK(!node_vec.empty());
  size_t result_idx = 0;
  double result_range_length = 0.0;
  for (size_t i = 0; i < node_vec.size(); ++i) {
    if (node_vec[i].Length() > result_range_length) {
      result_range_length = node_vec[i].Length();
      result_idx = i;
    }
  }
  return node_vec[result_idx];
}

bool ResultGenerator::ExtractBasicPassages(
    const std::vector<NodeWithRange>& nodes,
    std::vector<PassageInfo>* const passages) {
  // 节点不能是空
  ACHECK(!nodes.empty());
  // 先清空一下待输出的结果
  passages->clear();
  // 构造一个变量不知道要干啥
  std::vector<NodeWithRange> nodes_of_passage;
  // 先把第一个节点放进来，说明这些节点是有序的？
  nodes_of_passage.push_back(nodes.at(0));
  // 然后从第二个节点开始循环
  for (size_t i = 1; i < nodes.size(); ++i) {
    // 获得从前一个节点到后一个节点之间的边
    auto edge =
        nodes.at(i - 1).GetTopoNode()->GetOutEdgeTo(nodes.at(i).GetTopoNode());
    // 如果边是空，表明出问题了
    if (edge == nullptr) {
      AERROR << "Get null pointer to edge from " << nodes.at(i - 1).LaneId()
             << " to " << nodes.at(i).LaneId();
      return false;
    }
    // 如果是需要向左向右变道
    if (edge->Type() == TET_LEFT || edge->Type() == TET_RIGHT) {

      auto change_lane_type = LEFT;
      if (edge->Type() == TET_RIGHT) {
        change_lane_type = RIGHT;
      }
      // 在passages中添加换道边
      passages->emplace_back(nodes_of_passage, change_lane_type);
      nodes_of_passage.clear();
    }
    // 如果到后一个节点的路径不是换道，直接把这个node放进来
    // 如果已经放过变道passage，同样需要把这个节点放进来，
    nodes_of_passage.push_back(nodes.at(i));
  }
  // 最后把剩下的nodes放进去
  // 注意看passage里面东西的结构
  // 都是很长一段foward node，直到一个变道node，依次类推
  passages->emplace_back(nodes_of_passage, FORWARD);
  return true;
}
// 这个函数和下面那个函数是一起的
// 从所有的 to_nodes 中，寻找到一个node，这个node能由from node通过向左或者向右的变道到达
bool ResultGenerator::IsReachableFromWithChangeLane(
    const TopoNode* from_node, const PassageInfo& to_nodes,
    NodeWithRange* reachable_node) {
  for (const auto& to_node : to_nodes.nodes) {
    auto edge = to_node.GetTopoNode()->GetInEdgeFrom(from_node);
    if (edge != nullptr &&
        (edge->Type() == TET_LEFT || edge->Type() == TET_RIGHT)) {
      *reachable_node = to_node;
      return true;
    }
  }
  return false;
}

// 从所有的from nodes中，寻找到一个node，这个node能通过向左或者向右的变道，到达to_node
bool ResultGenerator::IsReachableToWithChangeLane(
    const TopoNode* to_node, const PassageInfo& from_nodes,
    NodeWithRange* reachable_node) {
  for (const auto& from_node : from_nodes.nodes) {
    auto edge = from_node.GetTopoNode()->GetOutEdgeTo(to_node);
    if (edge != nullptr &&
        (edge->Type() == TET_LEFT || edge->Type() == TET_RIGHT)) {
      *reachable_node = from_node;
      return true;
    }
  }
  return false;
}

void ResultGenerator::ExtendBackward(const TopoRangeManager& range_manager,
                                     const PassageInfo& prev_passage,
                                     PassageInfo* const curr_passage) {
  std::unordered_set<const TopoNode*> node_set_of_curr_passage;
  for (const auto& node : curr_passage->nodes) {
    node_set_of_curr_passage.insert(node.GetTopoNode());
  }
  auto& front_node = curr_passage->nodes.front();
  // if front node starts at middle
  if (!IsCloseEnough(front_node.StartS(), 0.0)) {
    if (!range_manager.Find(front_node.GetTopoNode())) {
      if (IsCloseEnough(prev_passage.nodes.front().StartS(), 0.0)) {
        front_node.SetStartS(0.0);
      } else {
        double temp_s = prev_passage.nodes.front().StartS() /
                        prev_passage.nodes.front().FullLength() *
                        front_node.FullLength();
        front_node.SetStartS(temp_s);
      }
    } else {
      return;
    }
  }

  bool allowed_to_explore = true;
  while (allowed_to_explore) {
    std::vector<NodeWithRange> pred_set;
    for (const auto& edge :
         curr_passage->nodes.front().GetTopoNode()->InFromPreEdge()) {
      const auto& pred_node = edge->FromNode();

      // if pred node has been inserted
      if (ContainsKey(node_set_of_curr_passage, pred_node)) {
        continue;
      }
      // if pred node is reachable from prev passage
      NodeWithRange reachable_node(pred_node, 0, 1);
      if (IsReachableToWithChangeLane(pred_node, prev_passage,
                                      &reachable_node)) {
        const auto* pred_range = range_manager.Find(pred_node);
        if (pred_range != nullptr && !pred_range->empty()) {
          double black_s_end = pred_range->back().EndS();
          if (!IsCloseEnough(black_s_end, pred_node->Length())) {
            pred_set.emplace_back(pred_node, black_s_end, pred_node->Length());
          }
        } else {
          pred_set.emplace_back(pred_node, 0.0, pred_node->Length());
        }
      }
    }
    if (pred_set.empty()) {
      allowed_to_explore = false;
    } else {
      allowed_to_explore = true;
      const auto& node_to_insert = GetLargestRange(pred_set);
      curr_passage->nodes.insert(curr_passage->nodes.begin(), node_to_insert);
      node_set_of_curr_passage.emplace(node_to_insert.GetTopoNode());
    }
  }
}

// 向前扩展
void ResultGenerator::ExtendForward(const TopoRangeManager& range_manager,
                                    const PassageInfo& next_passage,
                                    PassageInfo* const curr_passage) {
  // 构造一个无序集合，存放当前passage相关的node
  std::unordered_set<const TopoNode*> node_set_of_curr_passage;
  // 对当前passage里面的node进行循环，这些node是很长一段forwardnode，再加最后一个需要变道的node
  // 如果有连续变道的情况，一个passage存的就是变道前的车道node
  for (const auto& node : curr_passage->nodes) {
    // 把所有的当前的passage内的node都放到上面这个无序集合当中
    node_set_of_curr_passage.insert(node.GetTopoNode());
  }

  // 提取出当前pasasage最后一个节点，这个节点一般是换道节点
  // 对于换道节点，我们认为，换道的目标车道和当前车道是基本平行的，所以才有了如下的逻辑：
  /**
   * 当前车道的end s 比总长度要小，就看看相邻车道，如果相邻车道比例比较大，就可以参照它的来
   * */
  auto& back_node = curr_passage->nodes.back();
  if (!IsCloseEnough(back_node.EndS(), back_node.FullLength())) {
    if (!range_manager.Find(back_node.GetTopoNode())) {
      // 注意要回去找找ends怎么计算出来的
      // 这个if 是说判断最后一个node 的 ends和最后一个node的fulllength是不是一样
      if (IsCloseEnough(next_passage.nodes.back().EndS(),
                        next_passage.nodes.back().FullLength())) {
        // 如果相差很小，就直接设置成fulllength就完了
        back_node.SetEndS(back_node.FullLength());
      } else {
        // 如果相差的很大
        double adjusted_end_s = next_passage.nodes.back().EndS() /
                                next_passage.nodes.back().FullLength() *
                                back_node.FullLength();
        if (adjusted_end_s > back_node.StartS()) {
          adjusted_end_s = std::min(adjusted_end_s, back_node.FullLength());
          back_node.SetEndS(adjusted_end_s);
          ADEBUG << "ExtendForward: orig_end_s[" << back_node.EndS()
                 << "] adjusted_end_s[" << adjusted_end_s << "]";
        }
      }
    } else {
      return;
    }
  }

  // 初始时允许探索
  bool allowed_to_explore = true;
  // 这个探索的含义：
  /**
   * 因为一个passage的结尾一定是换道，我们就看看换道之前的这个道的后继节点中，有没有可以换道到目标车道的，如果有，就可以和换道前的车道连接起来，
   * 然后继续向前探索，直到没有任何一个后继节点能够换道到目标车，就停止
   * */
  while (allowed_to_explore) {
    // 构造一个后继节点集合
    std::vector<NodeWithRange> succ_set;
    // 对从当前passage最后一个节点引申出的所有edge进行循环
    for (const auto& edge :
         curr_passage->nodes.back().GetTopoNode()->OutToSucEdge()) {
      // 记录下当前edge对应的tonode
      const auto& succ_node = edge->ToNode();
      // if succ node has been inserted
      // 如果这个to node已经被包含到passage里面了就continue到下一个edge
      if (ContainsKey(node_set_of_curr_passage, succ_node)) {
        continue;
      }
      // if next passage is reachable from succ node
      // 如果还没有被包含
      // 构造一个节点，s长度只有1？
      NodeWithRange reachable_node(succ_node, 0, 1.0);
      // 判断这个后继节点是否能通过换道到达下一个passage
      if (IsReachableFromWithChangeLane(succ_node, next_passage,
                                        &reachable_node)) {
        const auto* succ_range = range_manager.Find(succ_node);
        if (succ_range != nullptr && !succ_range->empty()) {
          double black_s_start = succ_range->front().StartS();
          if (!IsCloseEnough(black_s_start, 0.0)) {
            succ_set.emplace_back(succ_node, 0.0, black_s_start);
          }
        } else {
          if (IsCloseEnough(reachable_node.EndS(),
                            reachable_node.FullLength())) {
            succ_set.emplace_back(succ_node, 0.0, succ_node->Length());
          } else {
            double push_end_s = reachable_node.EndS() /
                                reachable_node.FullLength() *
                                succ_node->Length();
            succ_set.emplace_back(succ_node, 0.0, push_end_s);
          }
        }
      }
    }
    if (succ_set.empty()) {
      allowed_to_explore = false;
    } else {
      allowed_to_explore = true;
      const auto& node_to_insert = GetLargestRange(succ_set);
      curr_passage->nodes.push_back(node_to_insert);
      node_set_of_curr_passage.emplace(node_to_insert.GetTopoNode());
    }
  }
}

// 把passage 扩展
void ResultGenerator::ExtendPassages(const TopoRangeManager& range_manager,
                                     std::vector<PassageInfo>* const passages) {
  // 读取出当前有的passage数目
  int passage_num = static_cast<int>(passages->size());
  // 对所有的passage进行循环
  for (int i = 0; i < passage_num; ++i) {
    // 如果当前passage不是最后一个passage
    if (i < passage_num - 1) {
      // 就向前扩展
      ExtendForward(range_manager, passages->at(i + 1), &(passages->at(i)));
    }
    if (i > 0) {
      // 如果passage不是第一个passage就向后扩展
      ExtendBackward(range_manager, passages->at(i - 1), &(passages->at(i)));
    }
  }
  // 然后再倒着循环一遍？
  for (int i = passage_num - 1; i >= 0; --i) {
    if (i < passage_num - 1) {
      ExtendForward(range_manager, passages->at(i + 1), &(passages->at(i)));
    }
    if (i > 0) {
      ExtendBackward(range_manager, passages->at(i - 1), &(passages->at(i)));
    }
  }
}

/**
 * 将NodeWithRange序列，转化为passage序列
 * 对每个一个node进行循环，然后取出里面的laneid和start s 和 end s
 */
void LaneNodesToPassageRegion(
    const std::vector<NodeWithRange>::const_iterator begin,
    const std::vector<NodeWithRange>::const_iterator end,
    Passage* const passage) {
  for (auto it = begin; it != end; ++it) {
    LaneSegment* seg = passage->add_segment();
    seg->set_id(it->LaneId());
    seg->set_start_s(it->StartS());
    seg->set_end_s(it->EndS());
  }
}

void LaneNodesToPassageRegion(const std::vector<NodeWithRange>& nodes,
                              Passage* const passage) {
  return LaneNodesToPassageRegion(nodes.begin(), nodes.end(), passage);
}

double CalculateDistance(const std::vector<NodeWithRange>& nodes) {
  double distance = nodes.at(0).EndS() - nodes.at(0).StartS();
  for (size_t i = 1; i < nodes.size(); ++i) {
    auto edge =
        nodes.at(i - 1).GetTopoNode()->GetOutEdgeTo(nodes.at(i).GetTopoNode());
    if (edge == nullptr || edge->Type() != TET_FORWARD) {
      continue;
    }
    distance += nodes.at(i).EndS() - nodes.at(i).StartS();
  }
  return distance;
}

void PrintDebugInfo(const std::string& road_id,
                    const std::vector<std::vector<NodeWithRange>>& nodes) {
  AINFO << "road id: " << road_id;
  for (size_t j = 0; j < nodes.size(); ++j) {
    AINFO << "\tPassage " << j;
    for (const auto& node : nodes[j]) {
      AINFO << "\t\t" << node.LaneId() << "   (" << node.StartS() << ", "
            << node.EndS() << ")";
    }
  }
}

// 这个是主要的调用函数，用来生成response
/**
 * @param map_version 地图版本？
 * @param request 请求
 * @param nodes 前面找到的routing 结果
 * @param range_manager 这个不知道是干啥的
 * @param result 这个是待求结果
 * */
bool ResultGenerator::GeneratePassageRegion(
    const std::string& map_version, const RoutingRequest& request,
    const std::vector<NodeWithRange>& nodes,
    const TopoRangeManager& range_manager, RoutingResponse* const result) {
  // 可以看到，是直接调用的下面的函数
  if (!GeneratePassageRegion(nodes, range_manager, result)) {
    return false;
  }
  // 地图版本用在这里，对result中的量进行一些设置
  result->set_map_version(map_version);
  result->mutable_measurement()->set_distance(CalculateDistance(nodes));
  // 结果中同样保存了一份request，在这里可以看到
  result->mutable_routing_request()->CopyFrom(request);
  return true;
}

// 所以终点还是看这个函数
bool ResultGenerator::GeneratePassageRegion(
    const std::vector<NodeWithRange>& nodes,
    const TopoRangeManager& range_manager, RoutingResponse* const result) {
  
  std::vector<PassageInfo> passages;
  // 先提取基本的passage
  if (!ExtractBasicPassages(nodes, &passages)) {
    return false;
  }
  // 再扩展passage
  ExtendPassages(range_manager, &passages);
  // 再把passage变成result
  CreateRoadSegments(passages, result);

  return true;
}

void ResultGenerator::AddRoadSegment(
    const std::vector<PassageInfo>& passages,
    const std::pair<std::size_t, std::size_t>& start_index,
    const std::pair<std::size_t, std::size_t>& end_index,
    RoutingResponse* result) {
  auto* road = result->add_road();

  road->set_id(passages[start_index.first].nodes[start_index.second].RoadId());
  for (std::size_t i = start_index.first;
      i <= end_index.first && i < passages.size(); ++i) {
    auto* passage = road->add_passage();
    const size_t node_start_index =
        (i == start_index.first ?
        std::max((std::size_t)0, start_index.second) : 0);
    const auto node_begin_iter = passages[i].nodes.cbegin() + node_start_index;
    ADEBUG<< "start node: " << node_begin_iter->LaneId() << ": "
           << node_begin_iter->StartS() << "; " << node_begin_iter->EndS();
    const size_t node_end_index =
         (i == end_index.first ?
         std::min(end_index.second, passages[i].nodes.size() - 1) :
         passages[i].nodes.size() - 1);
    const auto node_last_iter = passages[i].nodes.cbegin() + node_end_index;
    ADEBUG << "last node: " << node_last_iter->LaneId() << ": "
           << node_last_iter->StartS() << "; " << node_last_iter->EndS();
    auto node_end_iter = node_last_iter + 1;
    LaneNodesToPassageRegion(node_begin_iter, node_end_iter, passage);
    if (start_index.first == end_index.first) {
      passage->set_change_lane_type(FORWARD);
      passage->set_can_exit(true);
    } else {
      passage->set_change_lane_type(passages[i].change_lane_type);
      passage->set_can_exit(i == end_index.first);
    }
  }
}

/**
 * 然后就是这个最激动人心的函数，看懂这个就之后发送给控制的请求是什么样子的了
 * @param passages 前面算出来的结果
 * @param result 最终要输出的response
 * */
void ResultGenerator::CreateRoadSegments(
    const std::vector<PassageInfo>& passages, RoutingResponse* result) {
  // 先进行检查，前面算出来的不能是空集
  ACHECK(!passages.empty()) << "passages empty";
  // 注意看下面的括号里的变量，是一个，表示的是第一段passage，里面的第一个node
  NodeWithRange fake_node_range(passages.front().nodes.front());
  // 一个flag，再换道的道，初始值false
  bool in_change_lane = false;
  // 定义一个pair变量不知道要干啥
  std::pair<std::size_t, std::size_t> start_index(0, 0);
  // 然后开始循环
  // 对整个passages数组里面的每一个passage进行循环
  for (std::size_t i = 0; i < passages.size(); ++i) {
    // 读取出当前passage内的所有的node
    const auto& curr_nodes = passages[i].nodes;
    // 然后对每一个node再进行循环
    for (std::size_t j = 0; j < curr_nodes.size(); ++j) {
      // 如果当前passage的下一个passage不是所有passage中的最后一个
      // 并且
      // 
      if ((i + 1 < passages.size() &&
            // 这个函数在干啥
            /**
             * @param curr_nodes[j].GetTopoNode() 当前passage的当前node
             * @param passages[i + 1] 下一个passage
             * @param fake_node_range 第一个passage的第一个node
             * */
            // 这个函数的意思是，在下一个passage中找一个点，可以变道到当前车道？这是干啥
           IsReachableToWithChangeLane(curr_nodes[j].GetTopoNode(),
                                       passages[i + 1], &fake_node_range)) ||
          (i > 0 &&
          // 这个函数是，在上一个车道中，找到一个点，能从当前车道变道回去。。
           IsReachableFromWithChangeLane(curr_nodes[j].GetTopoNode(),
                                         passages[i - 1], &fake_node_range))) {
        if (!in_change_lane) {
          start_index = {i, j};
          in_change_lane = true;
        }
      } else {
        if (in_change_lane) {
          ADEBUG << "start_index(" << start_index.first << ", "
                 << start_index.second
                 << ") end_index(" << i << ", " << j - 1 << ")";
          AddRoadSegment(passages, start_index, {i, j - 1}, result);
        }
        ADEBUG << "start_index(" << i << ", " << j
               << ") end_index(" << i << ", " << j << ")";
        AddRoadSegment(passages, {i, j}, {i, j}, result);
        in_change_lane = false;
      }
    }
  }
  if (in_change_lane) {
    ADEBUG << "start_index(" << start_index.first << ", " << start_index.second
           << ") end_index(" << passages.size() - 1 << ", "
           << passages.back().nodes.size() - 1 << ")";
    AddRoadSegment(passages, start_index,
                   {passages.size() - 1, passages.back().nodes.size() - 1},
                   result);
  }
}

}  // namespace routing
}  // namespace apollo
