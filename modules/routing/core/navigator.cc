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

#include "modules/routing/core/navigator.h"

#include "cyber/common/file.h"
#include "modules/routing/common/routing_gflags.h"
#include "modules/routing/graph/sub_topo_graph.h"
#include "modules/routing/strategy/a_star_strategy.h"

namespace apollo {
namespace routing {

namespace {

using apollo::common::ErrorCode;

// 就是对request信息做一些显示，如果有问题就退出
bool ShowRequestInfo(const RoutingRequest& request, const TopoGraph* graph) {
  // 对每一个要经过way point 进行循环
  for (const auto& wp : request.waypoint()) {
    // 注意way point的id 就是前面找到的lane 的id 有可能是最近lane，也有可能是一定范围内的候选lane
    // 从graph中，把这个lane对应的node读出来
    const auto* node = graph->GetNode(wp.id());
    if (node == nullptr) {
      // 如果node是空就报错
      AERROR << "Way node is not found in topo graph! ID: " << wp.id();
      return false;
    }
    // 如果没问题就先显示一下
    AINFO << "Way point:\tlane id: " << wp.id() << " s: " << wp.s()
          << " x: " << wp.pose().x() << " y: " << wp.pose().y()
          << " length: " << node->Length();
  }

  // 然后去看看request里面的黑名单路
  for (const auto& bl : request.blacklisted_lane()) {
    // 同样从graph中读取这些lane 对应的node
    const auto* node = graph->GetNode(bl.id());

    if (node == nullptr) {
      // 如果有问题就退出
      AERROR << "Black list node is not found in topo graph! ID: " << bl.id();
      return false;
    }
    // 如果没有问题就显示出来
    AINFO << "Black point:\tlane id: " << bl.id()
          << " start_s: " << bl.start_s() << " end_s: " << bl.end_s()
          << " length: " << node->Length();
  }

  return true;
}

bool GetWayNodes(const RoutingRequest& request, const TopoGraph* graph,
                 std::vector<const TopoNode*>* const way_nodes,
                 std::vector<double>* const way_s) {
  // 对request中每一个way point进行循环
  for (const auto& point : request.waypoint()) {
    //读取出graph中对应的node
    const auto* cur_node = graph->GetNode(point.id());
    if (cur_node == nullptr) {
      // 如果是空就报错
      AERROR << "Cannot find way point in graph! Id: " << point.id();
      return false;
    }
    // 把读出来的node放到数组中，同样放入s
    way_nodes->push_back(cur_node);
    // 注意这个s的含义，是只当前这个路点，在所选取的邻域lane上的投影的s坐标
    way_s->push_back(point.s());
  }
  return true;
}

void SetErrorCode(const common::ErrorCode& error_code_id,
                  const std::string& error_string,
                  common::StatusPb* const error_code) {
  error_code->set_error_code(error_code_id);
  error_code->set_msg(error_string);
  if (error_code_id == common::ErrorCode::OK) {
    ADEBUG << error_string.c_str();
  } else {
    AERROR << error_string.c_str();
  }
}

void PrintDebugData(const std::vector<NodeWithRange>& nodes) {
  AINFO << "Route lane id\tis virtual\tstart s\tend s";
  for (const auto& node : nodes) {
    AINFO << node.GetTopoNode()->LaneId() << "\t"
          << node.GetTopoNode()->IsVirtual() << "\t" << node.StartS() << "\t"
          << node.EndS();
  }
}

}  // namespace

Navigator::Navigator(const std::string& topo_file_path) {
  Graph graph;
  if (!cyber::common::GetProtoFromFile(topo_file_path, &graph)) {
    AERROR << "Failed to read topology graph from " << topo_file_path;
    return;
  }

  graph_.reset(new TopoGraph());
  if (!graph_->LoadGraph(graph)) {
    AINFO << "Failed to init navigator graph failed! File path: "
          << topo_file_path;
    return;
  }
  black_list_generator_.reset(new BlackListRangeGenerator);
  result_generator_.reset(new ResultGenerator);
  is_ready_ = true;
  AINFO << "The navigator is ready.";
}

Navigator::~Navigator() {}

bool Navigator::IsReady() const { return is_ready_; }

void Navigator::Clear() { topo_range_manager_.Clear(); }


/**
 * 这个函数在开始搜索路径之前线进行一些初始化操作
 * */
bool Navigator::Init(const RoutingRequest& request, const TopoGraph* graph,
                     std::vector<const TopoNode*>* const way_nodes,
                     std::vector<double>* const way_s) {
  Clear();
  if (!GetWayNodes(request, graph_.get(), way_nodes, way_s)) {
    AERROR << "Failed to find search terminal point in graph!";
    return false;
  }
  // 根据request生成黑名单
  // 这个是构造每个lane上面有问题的段，在初始化时就构造完成
  black_list_generator_->GenerateBlackMapFromRequest(request, graph_.get(),
                                                     &topo_range_manager_);
  return true;
}

bool Navigator::MergeRoute(
    const std::vector<NodeWithRange>& node_vec,
    std::vector<NodeWithRange>* const result_node_vec) const {
  for (const auto& node : node_vec) {
    if (result_node_vec->empty() ||
        result_node_vec->back().GetTopoNode() != node.GetTopoNode()) {
      result_node_vec->push_back(node);
    } else {
      if (result_node_vec->back().EndS() < node.StartS()) {
        AERROR << "Result route is not continuous.";
        return false;
      }
      result_node_vec->back().SetEndS(node.EndS());
    }
  }
  return true;
}

bool Navigator::SearchRouteByStrategy(
    const TopoGraph* graph, const std::vector<const TopoNode*>& way_nodes,
    const std::vector<double>& way_s,
    std::vector<NodeWithRange>* const result_nodes) const {
  // 构造一个a star策略对象
  std::unique_ptr<Strategy> strategy_ptr;
  strategy_ptr.reset(new AStarStrategy(FLAGS_enable_change_lane_in_result));

  result_nodes->clear();
  std::vector<NodeWithRange> node_vec;
  // 对每一个way node进行循环，从第二个开始，因为要和第1个组成一个segment
  for (size_t i = 1; i < way_nodes.size(); ++i) {

    // 设置初始node
    const auto* way_start = way_nodes[i - 1];
    // 设置结束node
    const auto* way_end = way_nodes[i];
    // 根据way_s的定义,这里way_start_s，就是起始点在起始lane上的s坐标，way_end_s就是终止点在终止lane上的s坐标
    double way_start_s = way_s[i - 1];
    double way_end_s = way_s[i];

    // 注意这个full_range_manager已经包含了在初始化过程中屏蔽的那些range
    TopoRangeManager full_range_manager = topo_range_manager_;
    black_list_generator_->AddBlackMapFromTerminal(
        way_start, way_end, way_start_s, way_end_s, &full_range_manager);

    // 把那个黑名单地图返回出来
    SubTopoGraph sub_graph(full_range_manager.RangeMap());
    const auto* start = sub_graph.GetSubNodeWithS(way_start, way_start_s);
    if (start == nullptr) {
      AERROR << "Sub graph node is nullptr, origin node id: "
             << way_start->LaneId() << ", s:" << way_start_s;
      return false;
    }
    const auto* end = sub_graph.GetSubNodeWithS(way_end, way_end_s);
    if (end == nullptr) {
      AERROR << "Sub graph node is nullptr, origin node id: "
             << way_end->LaneId() << ", s:" << way_end_s;
      return false;
    }

    std::vector<NodeWithRange> cur_result_nodes;
    if (!strategy_ptr->Search(graph, &sub_graph, start, end,
                              &cur_result_nodes)) {
      AERROR << "Failed to search route with waypoint from " << start->LaneId()
             << " to " << end->LaneId();
      return false;
    }

    node_vec.insert(node_vec.end(), cur_result_nodes.begin(),
                    cur_result_nodes.end());
  }

  if (!MergeRoute(node_vec, result_nodes)) {
    AERROR << "Failed to merge route.";
    return false;
  }
  return true;
}

bool Navigator::SearchRoute(const RoutingRequest& request,
                            RoutingResponse* const response) {
  // 显示信息，如果有问题就报错退出
  if (!ShowRequestInfo(request, graph_.get())) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_REQUEST,
                 "Error encountered when reading request point!",
                 response->mutable_status());
    return false;
  }
  // 看看navigator是否ready
  if (!IsReady()) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_NOT_READY, "Navigator is not ready!",
                 response->mutable_status());
    return false;
  }

  // 定义一系列topo node 的序列
  std::vector<const TopoNode*> way_nodes;
  std::vector<double> way_s;
  // 初始化，如果失败就退出，就是简单的读取出graph中对应的node就行，同时还要处理一下
  // request中的黑名单lane和road，归根结底还是lane
  if (!Init(request, graph_.get(), &way_nodes, &way_s)) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_NOT_READY,
                 "Failed to initialize navigator!", response->mutable_status());
    return false;
  }
  // 定义一个结果变量存放结果？
  std::vector<NodeWithRange> result_nodes;
  if (!SearchRouteByStrategy(graph_.get(), way_nodes, way_s, &result_nodes)) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_RESPONSE,
                 "Failed to find route with request!",
                 response->mutable_status());
    return false;
  }
  // 如果结果是空就报错退出
  if (result_nodes.empty()) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_RESPONSE, "Failed to result nodes!",
                 response->mutable_status());
    return false;
  }

  result_nodes.front().SetStartS(request.waypoint().begin()->s());
  result_nodes.back().SetEndS(request.waypoint().rbegin()->s());

  // 注意看上面只是routing结果，用vector存储
  // 会在下面这个函数中转化为response，需要仔细看一下，看看response到底是什么样的
  if (!result_generator_->GeneratePassageRegion(
          graph_->MapVersion(), request, result_nodes, topo_range_manager_,
          response)) {
    SetErrorCode(ErrorCode::ROUTING_ERROR_RESPONSE,
                 "Failed to generate passage regions based on result lanes",
                 response->mutable_status());
    return false;
  }
  SetErrorCode(ErrorCode::OK, "Success!", response->mutable_status());

  PrintDebugData(result_nodes);
  return true;
}

}  // namespace routing
}  // namespace apollo
