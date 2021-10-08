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

#include "modules/routing/graph/topo_range_manager.h"

#include <algorithm>
#include <utility>

#include "cyber/common/log.h"

namespace apollo {
namespace routing {
namespace {

/**
 * @param topo_node 当前的节点
 * @param origin_range 原始range数值
 * @param block_range 输出的range vec
 * */
void merge_block_range(const TopoNode* topo_node,
                       const std::vector<NodeSRange>& origin_range,
                       std::vector<NodeSRange>* block_range) {
  // 先构造一个range vec用来存放排好序的vec
  std::vector<NodeSRange> sorted_origin_range(origin_range);
  // 然后去排序 因为里面定义了“<”负号，所以应该是能直接排？
  std::sort(sorted_origin_range.begin(), sorted_origin_range.end());
  size_t cur_index = 0;
  auto total_size = sorted_origin_range.size();
  while (cur_index < total_size) {
    NodeSRange range(sorted_origin_range[cur_index]);
    ++cur_index;
    while (cur_index < total_size &&
           range.MergeRangeOverlap(sorted_origin_range[cur_index])) {
      ++cur_index;
    }
    if (range.EndS() < topo_node->StartS() ||
        range.StartS() > topo_node->EndS()) {
      continue;
    }
    range.SetStartS(std::max(topo_node->StartS(), range.StartS()));
    range.SetEndS(std::min(topo_node->EndS(), range.EndS()));
    block_range->push_back(std::move(range));
  }
}

}  // namespace

const std::unordered_map<const TopoNode*, std::vector<NodeSRange>>&
TopoRangeManager::RangeMap() const {
  return range_map_;
}
const std::vector<NodeSRange>* TopoRangeManager::Find(
    const TopoNode* node) const {
  auto iter = range_map_.find(node);
  if (iter == range_map_.end()) {
    return nullptr;
  } else {
    return &(iter->second);
  }
}

void TopoRangeManager::PrintDebugInfo() const {
  for (const auto& map : range_map_) {
    for (const auto& range : map.second) {
      AINFO << "black lane id: " << map.first->LaneId()
            << ", start s: " << range.StartS() << ", end s: " << range.EndS();
    }
  }
}

void TopoRangeManager::Clear() { range_map_.clear(); }


void TopoRangeManager::Add(const TopoNode* node, double start_s, double end_s) {
  NodeSRange range(start_s, end_s);
  range_map_[node].push_back(range);
}

void TopoRangeManager::SortAndMerge() {
  // 对range map中的每一个节点进行循环
  for (auto& iter : range_map_) {
    // 构造一个vector
    std::vector<NodeSRange> merged_range_vec;
    merge_block_range(iter.first, iter.second, &merged_range_vec);
    iter.second.assign(merged_range_vec.begin(), merged_range_vec.end());
  }
}

}  // namespace routing
}  // namespace apollo
