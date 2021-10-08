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

#include "modules/map/pnc_map/pnc_map.h"

#include <algorithm>
#include <limits>

#include "absl/strings/str_cat.h"
#include "google/protobuf/text_format.h"

#include "modules/map/proto/map_id.pb.h"

#include "cyber/common/log.h"
#include "modules/common/util/point_factory.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/routing/common/routing_gflags.h"

DEFINE_double(
    look_backward_distance, 50,
    "look backward this distance when creating reference line from routing");

DEFINE_double(look_forward_short_distance, 180,
              "short look forward this distance when creating reference line "
              "from routing when ADC is slow");
DEFINE_double(
    look_forward_long_distance, 250,
    "look forward this distance when creating reference line from routing");

namespace apollo {
namespace hdmap {

using apollo::common::PointENU;
using apollo::common::VehicleState;
using apollo::common::util::PointFactory;
using apollo::routing::RoutingResponse;

namespace {

// Maximum lateral error used in trajectory approximation.
const double kTrajectoryApproximationMaxError = 2.0;

}  // namespace
// 这个构造函数异常简单，就是直接把hdmap赋值进来就行
PncMap::PncMap(const HDMap *hdmap) : hdmap_(hdmap) {}
// 读取出当前pncmap中的hdmap
const hdmap::HDMap *PncMap::hdmap() const { return hdmap_; }

// 给一个routing 里的way point类型，
// 在hdmap上查询到对应的lane，然后和s一起构成一个LaneWaypoint结构体
LaneWaypoint PncMap::ToLaneWaypoint(
    const routing::LaneWaypoint &waypoint) const {
  auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(waypoint.id()));
  ACHECK(lane) << "Invalid lane id: " << waypoint.id();
  return LaneWaypoint(lane, waypoint.s());
}

// 这个函数给定当前速度，计算需要向前look forward多少距离，就是速度 x 时间就行，然后做一下限幅
double PncMap::LookForwardDistance(double velocity) {
  auto forward_distance = velocity * FLAGS_look_forward_time_sec;

  return forward_distance > FLAGS_look_forward_short_distance
             ? FLAGS_look_forward_long_distance
             : FLAGS_look_forward_short_distance;
}

/**
 * 这个函数将routing pb里面定义的routing::LaneSegment变量，
 * 转化为pnc_ path 里面定义的laneSegment
 * */
LaneSegment PncMap::ToLaneSegment(const routing::LaneSegment &segment) const {
  // 从过程上看就是利用routing::LaneSegment里面带有的lane id 在hdmap里面查询lane，然后构造一个新的lane segment
  // 也就是说，在pb信息中传递id比较方便
  // 接收到pb数据之后，直接查询出一个lane ptr实体，存放到新定义的laneseg中。
  auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(segment.id()));
  ACHECK(lane) << "Invalid lane id: " << segment.id();
  return LaneSegment(lane, segment.start_s(), segment.end_s());
}

// 经过这一堆循环之后，正好找到那个比当前位置大！一次！的必经路点
void PncMap::UpdateNextRoutingWaypointIndex(int cur_index) {
  // 如果是小于0 的数，表明还没开始走，先往第一必经way point走
  if (cur_index < 0) {
    next_routing_waypoint_index_ = 0;
    return;
  }
  // 如果已经超过了所有线路数目，就赋值成最后一个way point
  if (cur_index >= static_cast<int>(route_indices_.size())) {
    next_routing_waypoint_index_ = routing_waypoint_index_.size() - 1;
    return;
  }
  // Search backwards when the car is driven backward on the route.
  while (next_routing_waypoint_index_ != 0 &&
         next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         routing_waypoint_index_[next_routing_waypoint_index_].index >
             cur_index) {
    --next_routing_waypoint_index_;
  }
  while (next_routing_waypoint_index_ != 0 &&
         next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         routing_waypoint_index_[next_routing_waypoint_index_].index ==
             cur_index &&
         adc_waypoint_.s <
             routing_waypoint_index_[next_routing_waypoint_index_].waypoint.s) {
    --next_routing_waypoint_index_;
  }
  // Search forwards
  while (next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         routing_waypoint_index_[next_routing_waypoint_index_].index <
             cur_index) {
    ++next_routing_waypoint_index_;
  }
  while (next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
         cur_index ==
             routing_waypoint_index_[next_routing_waypoint_index_].index &&
         adc_waypoint_.s >=
             routing_waypoint_index_[next_routing_waypoint_index_].waypoint.s) {
    ++next_routing_waypoint_index_;
  }
  if (next_routing_waypoint_index_ >= routing_waypoint_index_.size()) {
    next_routing_waypoint_index_ = routing_waypoint_index_.size() - 1;
  }
}

std::vector<routing::LaneWaypoint> PncMap::FutureRouteWaypoints() const {
  const auto &waypoints = routing_.routing_request().waypoint();
  return std::vector<routing::LaneWaypoint>(
      waypoints.begin() + next_routing_waypoint_index_, waypoints.end());
}

/**
 * @param adc_index 这个参数在进来之前直接给了-1
 * ！！！啊，这个函数的作用是车开了一段时间，经过了一些segment之后，可以去掉之前那些经过seg，这样缩减了搜索范围？
 * */
void PncMap::UpdateRoutingRange(int adc_index) {
  // Track routing range.
  range_lane_ids_.clear(); // 就是在这个函数中更新这个剩下的变量
  // range_start_ 这个值在这一步后会等于0， 因为adc_index传递进来时是-1，
  range_start_ = std::max(0, adc_index - 1);
  // 同理就直到这个值等于0
  range_end_ = range_start_;
  // 注意看这个route_indices_是在前面说的更新过了，数量等于rout response传递进来的lane的个数
  while (range_end_ < static_cast<int>(route_indices_.size())) {
    // 循环刚进来时，range_end_= 0，把第一lane id 放进去
    const auto &lane_id = route_indices_[range_end_].segment.lane->id().id();
    if (range_lane_ids_.count(lane_id) != 0) {
      // 只有当出现重复的lane 的时候，才会跳到这里，这很迷？
      // 是为了破除loop？
      break;
    }
    // 如果没有出现过，就只加放进来
    range_lane_ids_.insert(lane_id);
    ++range_end_;
  }
}
/**
 * 更新车辆的状态
 * */
bool PncMap::UpdateVehicleState(const VehicleState &vehicle_state) {
  // 首先看一下当前的routing_ 成员是不是valid
  if (!ValidateRouting(routing_)) {
    // 不valid直接报错退出
    AERROR << "The routing is invalid when updating vehicle state.";
    return false;
  }
  /**
   * 这里这个adc_state_是成员变量，应该就是上一次更新的车辆状态
   * 
   * */
  // 看看车辆状态中有没有x
  // 再看看在xy平面内的新老两个车辆状态的距离，是否大于横纵两个方向的阈值之和
  if (!adc_state_.has_x() ||
      (common::util::DistanceXY(adc_state_, vehicle_state) >
       FLAGS_replan_lateral_distance_threshold +
           FLAGS_replan_longitudinal_distance_threshold)) {
    // Position is reset, but not replan.
    // 如果之前的adc_state_，没有x 或者
    // 距离大于阈值
    // 就进来进行置位
    next_routing_waypoint_index_ = 0;
    adc_route_index_ = -1;
    stop_for_destination_ = false;
  }

  // 出来之后
  // 把这一帧的vehicle_state 赋值给adc_state_，相当于更新信息
  adc_state_ = vehicle_state;
  // 在routing路线里面的所有lane中找一个最近的lane上的最近点
  if (!GetNearestPointFromRouting(vehicle_state, &adc_waypoint_)) {
    // 如果没有就报错退出
    AERROR << "Failed to get waypoint from routing with point: "
           << "(" << vehicle_state.x() << ", " << vehicle_state.y() << ", "
           << vehicle_state.z() << ").";
    return false;
  }
  // 找到这个lane上的waypoint对应的lane segment序号
  int route_index = GetWaypointIndex(adc_waypoint_);
  if (route_index < 0 ||
      route_index >= static_cast<int>(route_indices_.size())) {
    AERROR << "Cannot find waypoint: " << adc_waypoint_.DebugString();
    return false;
  }

  // Track how many routing request waypoints the adc have passed.
  UpdateNextRoutingWaypointIndex(route_index); // 去找下一个必经路点，找到的结果存在了next_routing_waypoint_index_里面
  adc_route_index_ = route_index; // 将前面算出来的route index赋值
  UpdateRoutingRange(adc_route_index_);

  if (routing_waypoint_index_.empty()) {
    AERROR << "No routing waypoint index.";
    return false;
  }

  // 注意这里的routing_waypoint_index_和next_routing_waypoint_index_都是特指的routing过程中的必经路点
  if (next_routing_waypoint_index_ == routing_waypoint_index_.size() - 1) {
    stop_for_destination_ = true;
  }
  return true;
}

bool PncMap::IsNewRouting(const routing::RoutingResponse &routing) const {
  return IsNewRouting(routing_, routing);
}

// 检查一下是不是来了一个新的routing response
/**
 * @param prev 上一个response
 * @param routing 新的response
 * */
bool PncMap::IsNewRouting(const routing::RoutingResponse &prev,
                          const routing::RoutingResponse &routing) {
  // 先检查routing是不是valid
  if (!ValidateRouting(routing)) {
    // 不valid就报错退出
    ADEBUG << "The provided routing is invalid.";
    return false;
  }
  // 再调用函数检查两个pb变量是否相等
  // （1）名字相等
  // （2）包含的序列内容相同，这里直接调用的是pb库就行
  return !common::util::IsProtoEqual(prev, routing);
}

/**
 * 更新routing response进来
 * */
bool PncMap::UpdateRoutingResponse(const routing::RoutingResponse &routing) {
  // 这三个变量应该是需要根据routing response进行更新的
  range_lane_ids_.clear();  // 这个是string类型id的集合
  route_indices_.clear(); //这个是route index序列，一个route index包含一个lanesegment和一个3维坐标点，应该是某个东西的索引
  all_lane_ids_.clear();// 这个是string类型id的集合
  // 对routing输出的road进行循环
  for (int road_index = 0; road_index < routing.road_size(); ++road_index) {
    // 记录下当前的road_segment
    const auto &road_segment = routing.road(road_index);
    // 然后对road_seg中的所有passage进行循环
    for (int passage_index = 0; passage_index < road_segment.passage_size();
         ++passage_index) {
      // 然后对passage里面的所有lane进行循环
      const auto &passage = road_segment.passage(passage_index);
      for (int lane_index = 0; lane_index < passage.segment_size();
           ++lane_index) {
        // 这个all_lane_ids_就是把所欲的lane都插入进去
        all_lane_ids_.insert(passage.segment(lane_index).id());
        route_indices_.emplace_back();
        route_indices_.back().segment = // 把这个变量也更新了，注意还差一个range_lane_ids_变量在后面更新
            ToLaneSegment(passage.segment(lane_index));
        if (route_indices_.back().segment.lane == nullptr) {
          AERROR << "Failed to get lane segment from passage.";
          return false;
        }
        route_indices_.back().index = {road_index, passage_index, lane_index};
      }
    }
  }

  // 初始化一些变量
  range_start_ = 0;
  range_end_ = 0;
  adc_route_index_ = -1; // 注意这里直接给了初值-1
  next_routing_waypoint_index_ = 0;
  // 然后进入这个函数
  UpdateRoutingRange(adc_route_index_);

  
  routing_waypoint_index_.clear();
  // 检查routing 里面必经路点的情况
  const auto &request_waypoints = routing.routing_request().waypoint();
  if (request_waypoints.empty()) {
    AERROR << "Invalid routing: no request waypoints.";
    return false;
  }
  // 初始化
  int i = 0;
  // 同样对所有lane进行循环
  for (size_t j = 0; j < route_indices_.size(); ++j) {
    // 这个是为了找到 那些必经路点——waypoint所在的lane，并保存在routing_waypoint_index_里面
    while (i < request_waypoints.size() &&
           RouteSegments::WithinLaneSegment(route_indices_[j].segment,
                                            request_waypoints.Get(i))) {
      routing_waypoint_index_.emplace_back(
          LaneWaypoint(route_indices_[j].segment.lane,
                       request_waypoints.Get(i).s()),
          j);
      ++i;
    }
  }
  // 给routing request赋值
  routing_ = routing;
  // 构造一个空的waypoint
  adc_waypoint_ = LaneWaypoint();
  // 标志位置一下false
  stop_for_destination_ = false;
  return true;
}

const routing::RoutingResponse &PncMap::routing_response() const {
  return routing_;
}

/**
 * 看看这个私有成员routing也就是routing response是不是valid
 */
bool PncMap::ValidateRouting(const RoutingResponse &routing) {
  // 先计算一下road数量
  // 
  const int num_road = routing.road_size();
  // 看看road数目是不是等于0 
  if (num_road == 0) {
    // 如果road数量等于零就报错返回
    AERROR << "Route is empty.";
    return false;
  }
  // 看看routing response中是不是报了routing_request
  // 或者是request里面的路点数目是不是小于0
  if (!routing.has_routing_request() ||
      routing.routing_request().waypoint_size() < 2) {
    // 如果没有包含request或者request不合格，就报错返回
    AERROR << "Routing does not have request.";
    return false;
  }
  // 对request中的每一个路点进行检查
  for (const auto &waypoint : routing.routing_request().waypoint()) {
    // 检查路点是否包含id，并且包含s
    if (!waypoint.has_id() || !waypoint.has_s()) {
      // 一旦有任何一个不包含，就报错退出
      AERROR << "Routing waypoint has no lane_id or s.";
      return false;
    }
  }
  // 检查通过返回true
  return true;
}

// 这个waypoint是车辆当前状态所在的routing上的最近的lane 上的最近的点
int PncMap::SearchForwardWaypointIndex(int start,
                                       const LaneWaypoint &waypoint) const {
  // 在初始的时候start是-1，这里i是0
  int i = std::max(start, 0);
  // 这个i是指车辆所在位置最近的点对应的segment的标号
  while (
      i < static_cast<int>(route_indices_.size()) &&
      !RouteSegments::WithinLaneSegment(route_indices_[i].segment, waypoint)) {
    ++i;
  }
  return i;
}

int PncMap::SearchBackwardWaypointIndex(int start,
                                        const LaneWaypoint &waypoint) const {
  int i = std::min(static_cast<int>(route_indices_.size() - 1), start);
  while (i >= 0 && !RouteSegments::WithinLaneSegment(route_indices_[i].segment,
                                                     waypoint)) {
    --i;
  }
  return i;
}

int PncMap::NextWaypointIndex(int index) const {
  if (index >= static_cast<int>(route_indices_.size() - 1)) {
    return static_cast<int>(route_indices_.size()) - 1;
  } else if (index < 0) {
    return 0;
  } else {
    return index + 1;
  }
}

int PncMap::GetWaypointIndex(const LaneWaypoint &waypoint) const {
  int forward_index = SearchForwardWaypointIndex(adc_route_index_, waypoint);
  if (forward_index >= static_cast<int>(route_indices_.size())) {
    return SearchBackwardWaypointIndex(adc_route_index_, waypoint);
  }
  if (forward_index == adc_route_index_ ||
      forward_index == adc_route_index_ + 1) {
    return forward_index;
  }
  auto backward_index = SearchBackwardWaypointIndex(adc_route_index_, waypoint);
  if (backward_index < 0) {
    return forward_index;
  }

  return (backward_index + 1 == adc_route_index_) ? backward_index
                                                  : forward_index;
}

// 这个函数把一个passage变成一个roadseg
bool PncMap::PassageToSegments(routing::Passage passage,
                               RouteSegments *segments) const {
  // 初始化检查
  CHECK_NOTNULL(segments);
  // 清空便令
  segments->clear();
  // 在passage中，一个node就是一个seg，一个node也就是一个lane
  // 所以下面对passage中所有lane进行循环，其实在不变道时，一个
  for (const auto &lane : passage.segment()) {
    auto lane_ptr = hdmap_->GetLaneById(hdmap::MakeMapId(lane.id()));
    if (!lane_ptr) {
      AERROR << "Failed to find lane: " << lane.id();
      return false;
    }
    // 可以看到，每一个lane就构成了一个segment
    segments->emplace_back(lane_ptr, std::max(0.0, lane.start_s()),
                           std::min(lane_ptr->total_length(), lane.end_s()));
  }
  return !segments->empty();
}

// 获得周围的passages？
std::vector<int> PncMap::GetNeighborPassages(const routing::RoadSegment &road,
                                             int start_passage) const {
  // 合理性检查
  CHECK_GE(start_passage, 0);
  CHECK_LE(start_passage, road.passage_size());
  // 定义一个结果向量
  std::vector<int> result;
  // 获得这个passage实体，传进来的是index，在这儿查询一下
  const auto &source_passage = road.passage(start_passage);
  result.emplace_back(start_passage);
  // 如果这个passage的变道指令是forward，就直接返回，因为forward，不考虑周围passage情况？
  if (source_passage.change_lane_type() == routing::FORWARD) {
    return result;
  }
  // 如果换道不是forward，但是可以从这个车道退出，说明，没有必要换道，这个大概就是can_exit的含义？
  if (source_passage.can_exit()) {  // No need to change lane
    return result;
  }
  // 定义一个roadseg
  RouteSegments source_segments;
  // 把第一段passage变成一堆segments
  if (!PassageToSegments(source_passage, &source_segments)) {
    AERROR << "Failed to convert passage to segments";
    return result;
  }
  // 可以不用进行变道直接经过下一个必经路点的情况
  // 直接返回上面的result然后退出
  if (next_routing_waypoint_index_ < routing_waypoint_index_.size() &&
      source_segments.IsWaypointOnSegment(
          routing_waypoint_index_[next_routing_waypoint_index_].waypoint)) {
    ADEBUG << "Need to pass next waypoint[" << next_routing_waypoint_index_
           << "] before change lane";
    return result;
  }
  // 如果不是上面的情况：也就是说需要变道才能到达下一个必经路点
  // 构造一个无需集合
  std::unordered_set<std::string> neighbor_lanes;
  // 如果当前passage需要向右转
  if (source_passage.change_lane_type() == routing::LEFT) {
    // 对每一个source seg进行循环
    for (const auto &segment : source_segments) {
      // 找到这个segment的左侧能换道的lane
      for (const auto &left_id :
           segment.lane->lane().left_neighbor_forward_lane_id()) {
        // 把这个lane插入到无序集合当中
        neighbor_lanes.insert(left_id.id());
      }
    }
  // 同理处理右换道的情况
  } else if (source_passage.change_lane_type() == routing::RIGHT) {
    for (const auto &segment : source_segments) {
      for (const auto &right_id :
           segment.lane->lane().right_neighbor_forward_lane_id()) {
        neighbor_lanes.insert(right_id.id());
      }
    }
  }

  // 对这个road里面的所有passage进行循环
  for (int i = 0; i < road.passage_size(); ++i) {
    // 如果是当前的startpassage就跳过
    if (i == start_passage) {
      continue;
    }
    // 其他的road passage 都赋值给targetpassage
    const auto &target_passage = road.passage(i);
    // 读取出这些passage的segment
    for (const auto &segment : target_passage.segment()) {
      // 如果在neigbor set中，就加入result
      if (neighbor_lanes.count(segment.id())) {
        result.emplace_back(i);
        break;
      }
    }
  }
  // 所以说：这个passage只包含了当前pasaage以及它在routing内的邻居
  return result;
}

/**
 * 这个就是接口函数，被referenceline provider调用
 * */
bool PncMap::GetRouteSegments(const VehicleState &vehicle_state,
                              std::list<RouteSegments> *const route_segments) {
  double look_forward_distance =
      LookForwardDistance(vehicle_state.linear_velocity());
  double look_backward_distance = FLAGS_look_backward_distance;
  return GetRouteSegments(vehicle_state, look_backward_distance,
                          look_forward_distance, route_segments);
}

// 这个是被reference line provider 调用的接口，用来计算seg
// 要仔细看一下
bool PncMap::GetRouteSegments(const VehicleState &vehicle_state,
                              const double backward_length,
                              const double forward_length,
                              std::list<RouteSegments> *const route_segments) {
  // 先更新一下车辆状态，这个上面注释过
  // 主要目的就是找出车辆当前在routing路径上的位置，以及下一个该到哪一个必经路点了
  if (!UpdateVehicleState(vehicle_state)) {
    AERROR << "Failed to update vehicle state in pnc_map."; // 出问题就报错返回
    return false;
  }
  // Vehicle has to be this close to lane center before considering change
  // lane
  // 检查一下各种状态合不合理，不合理就报错返回
  if (!adc_waypoint_.lane || adc_route_index_ < 0 ||
      adc_route_index_ >= static_cast<int>(route_indices_.size())) {
    AERROR << "Invalid vehicle state in pnc_map, update vehicle state first.";
    return false;
  }

  const auto &route_index = route_indices_[adc_route_index_].index;
  const int road_index = route_index[0]; // 那个3维数组里面的标志
  const int passage_index = route_index[1]; // 3维数组标志
  const auto &road = routing_.road(road_index); // 读取出对应road
  // Raw filter to find all neighboring passages
  auto drive_passages = GetNeighborPassages(road, passage_index);
  // 出来之后对所有可行passage进行循环
  for (const int index : drive_passages) {
    // 记录出当前passage
    const auto &passage = road.passage(index);
    // 构造一个route seg
    RouteSegments segments;
    // 把当前passage转成route seg
    if (!PassageToSegments(passage, &segments)) {
      ADEBUG << "Failed to convert passage to lane segments.";
      continue;
    }
    
    const PointENU nearest_point =
        index == passage_index
            ? adc_waypoint_.lane->GetSmoothPoint(adc_waypoint_.s)
            : PointFactory::ToPointENU(adc_state_);
    common::SLPoint sl;
    LaneWaypoint segment_waypoint;
    if (!segments.GetProjection(nearest_point, &sl, &segment_waypoint)) {
      ADEBUG << "Failed to get projection from point: "
             << nearest_point.ShortDebugString();
      continue;
    }
    if (index != passage_index) {
      if (!segments.CanDriveFrom(adc_waypoint_)) {
        ADEBUG << "You cannot drive from current waypoint to passage: "
               << index;
        continue;
      }
    }
    route_segments->emplace_back();
    const auto last_waypoint = segments.LastWaypoint();
    if (!ExtendSegments(segments, sl.s() - backward_length,
                        sl.s() + forward_length, &route_segments->back())) {
      AERROR << "Failed to extend segments with s=" << sl.s()
             << ", backward: " << backward_length
             << ", forward: " << forward_length;
      return false;
    }
    if (route_segments->back().IsWaypointOnSegment(last_waypoint)) {
      route_segments->back().SetRouteEndWaypoint(last_waypoint);
    }
    route_segments->back().SetCanExit(passage.can_exit());
    route_segments->back().SetNextAction(passage.change_lane_type());
    const std::string route_segment_id = absl::StrCat(road_index, "_", index);
    route_segments->back().SetId(route_segment_id);
    route_segments->back().SetStopForDestination(stop_for_destination_);
    if (index == passage_index) {
      route_segments->back().SetIsOnSegment(true);
      route_segments->back().SetPreviousAction(routing::FORWARD);
    } else if (sl.l() > 0) {
      route_segments->back().SetPreviousAction(routing::RIGHT);
    } else {
      route_segments->back().SetPreviousAction(routing::LEFT);
    }
  }
  return !route_segments->empty();
}

bool PncMap::GetNearestPointFromRouting(const VehicleState &state,
                                        LaneWaypoint *waypoint) const {
  // 先定义两个常量
  const double kMaxDistance = 10.0;  // meters.
  const double kHeadingBuffer = M_PI / 10.0;
  // 将waypoint中的lane变量清空，也就是说，这里的waypoint是输出量
  waypoint->lane = nullptr;
  // 构造一个新的lane序列用来存放lane
  std::vector<LaneInfoConstPtr> lanes;
  // 将车辆状态转化为一个point 在enu下表达
  const auto point = PointFactory::ToPointENU(state);
  // 利用hdmap接口进行查询，找到一定范围内的车道，存储在上面构造的lane序列中
  const int status =
      hdmap_->GetLanesWithHeading(point, kMaxDistance, state.heading(),
                                  M_PI / 2.0 + kHeadingBuffer, &lanes);
  // 输出找到的lane 的个数
  ADEBUG << "lanes:" << lanes.size();
  // 对错误状态进行处理
  if (status < 0) {
    AERROR << "Failed to get lane from point: " << point.ShortDebugString();
    return false;
  }
  // 如果没有找到lane
  if (lanes.empty()) {
    // 直接报错退出，说明点离lane都太远了
    AERROR << "No valid lane found within " << kMaxDistance
           << " meters with heading " << state.heading();
    return false;
  }
  // 构造一个新的lane 序列，用来存放valid的lane
  std::vector<LaneInfoConstPtr> valid_lanes;
  // 从
  //要理解下面两个函数，首先要理解range_lane_ids_和all_lane_ids_是什么意思
  // 但是不管怎么样，这里就是将前面找到的lanes进行筛选，选出符合条件的，这个条件，应该是这个lane在某种类型的集合里面
  std::copy_if(lanes.begin(), lanes.end(), std::back_inserter(valid_lanes),
               [&](LaneInfoConstPtr ptr) {
                 return range_lane_ids_.count(ptr->lane().id().id()) > 0;
               });
  if (valid_lanes.empty()) {
    std::copy_if(lanes.begin(), lanes.end(), std::back_inserter(valid_lanes),
                 [&](LaneInfoConstPtr ptr) {
                   return all_lane_ids_.count(ptr->lane().id().id()) > 0;
                 });
  }

  // Get nearest_waypoints for current position
  // 定义一个最小距离，初始化成最大值
  double min_distance = std::numeric_limits<double>::infinity();
  // 对上面找到的每一个lane进行循环,这些lane都是routing response里面给出的lane
  for (const auto &lane : valid_lanes) {
    // 如果这个lane不在某个特定的集合里面就直接continue
    if (range_lane_ids_.count(lane->id().id()) == 0) {
      continue;
    }
    {
      double s = 0.0;
      double l = 0.0;
      // 如果lane没问题就往lane上面投影
      if (!lane->GetProjection({point.x(), point.y()}, &s, &l)) {
        AERROR << "fail to get projection";
        return false;
      }
      // Use large epsilon to allow projection diff
      static constexpr double kEpsilon = 0.5;
      // 然后观察投影点的位置合不合理
      if (s > (lane->total_length() + kEpsilon) || (s + kEpsilon) < 0.0) {
        continue;
      }
    }
    // 如果投影点满足要求
    double distance = 0.0;
    // 计算lane上面最近的点的距离
    common::PointENU map_point =
        lane->GetNearestPoint({point.x(), point.y()}, &distance);
    // 通过这个判断，从所有lane中找到最近的那一个，更新到waypoint中，并输出
    if (distance < min_distance) {
      min_distance = distance;
      double s = 0.0;
      double l = 0.0;
      if (!lane->GetProjection({map_point.x(), map_point.y()}, &s, &l)) {
        AERROR << "Failed to get projection for map_point: "
               << map_point.DebugString();
        return false;
      }
      waypoint->lane = lane;
      waypoint->s = s;
    }
    ADEBUG << "distance" << distance;
  }
  if (waypoint->lane == nullptr) {
    AERROR << "Failed to find nearest point: " << point.ShortDebugString();
  }
  return waypoint->lane != nullptr;
}

LaneInfoConstPtr PncMap::GetRouteSuccessor(LaneInfoConstPtr lane) const {
  if (lane->lane().successor_id().empty()) {
    return nullptr;
  }
  hdmap::Id preferred_id = lane->lane().successor_id(0);
  for (const auto &lane_id : lane->lane().successor_id()) {
    if (range_lane_ids_.count(lane_id.id()) != 0) {
      preferred_id = lane_id;
      break;
    }
  }
  return hdmap_->GetLaneById(preferred_id);
}

LaneInfoConstPtr PncMap::GetRoutePredecessor(LaneInfoConstPtr lane) const {
  if (lane->lane().predecessor_id().empty()) {
    return nullptr;
  }

  std::unordered_set<std::string> predecessor_ids;
  for (const auto &lane_id : lane->lane().predecessor_id()) {
    predecessor_ids.insert(lane_id.id());
  }

  hdmap::Id preferred_id = lane->lane().predecessor_id(0);
  for (size_t i = 1; i < route_indices_.size(); ++i) {
    auto &lane = route_indices_[i].segment.lane->id();
    if (predecessor_ids.count(lane.id()) != 0) {
      preferred_id = lane;
      break;
    }
  }
  return hdmap_->GetLaneById(preferred_id);
}

bool PncMap::ExtendSegments(const RouteSegments &segments,
                            const common::PointENU &point, double look_backward,
                            double look_forward,
                            RouteSegments *extended_segments) {
  common::SLPoint sl;
  LaneWaypoint waypoint;
  if (!segments.GetProjection(point, &sl, &waypoint)) {
    AERROR << "point: " << point.ShortDebugString() << " is not on segment";
    return false;
  }
  return ExtendSegments(segments, sl.s() - look_backward, sl.s() + look_forward,
                        extended_segments);
}

bool PncMap::ExtendSegments(const RouteSegments &segments, double start_s,
                            double end_s,
                            RouteSegments *const truncated_segments) const {
  if (segments.empty()) {
    AERROR << "The input segments is empty";
    return false;
  }
  CHECK_NOTNULL(truncated_segments);
  truncated_segments->SetProperties(segments);

  if (start_s >= end_s) {
    AERROR << "start_s(" << start_s << " >= end_s(" << end_s << ")";
    return false;
  }
  std::unordered_set<std::string> unique_lanes;
  static constexpr double kRouteEpsilon = 1e-3;
  // Extend the trajectory towards the start of the trajectory.
  if (start_s < 0) {
    const auto &first_segment = *segments.begin();
    auto lane = first_segment.lane;
    double s = first_segment.start_s;
    double extend_s = -start_s;
    std::vector<LaneSegment> extended_lane_segments;
    while (extend_s > kRouteEpsilon) {
      if (s <= kRouteEpsilon) {
        lane = GetRoutePredecessor(lane);
        if (lane == nullptr ||
            unique_lanes.find(lane->id().id()) != unique_lanes.end()) {
          break;
        }
        s = lane->total_length();
      } else {
        const double length = std::min(s, extend_s);
        extended_lane_segments.emplace_back(lane, s - length, s);
        extend_s -= length;
        s -= length;
        unique_lanes.insert(lane->id().id());
      }
    }
    truncated_segments->insert(truncated_segments->begin(),
                               extended_lane_segments.rbegin(),
                               extended_lane_segments.rend());
  }
  bool found_loop = false;
  double router_s = 0;
  for (const auto &lane_segment : segments) {
    const double adjusted_start_s = std::max(
        start_s - router_s + lane_segment.start_s, lane_segment.start_s);
    const double adjusted_end_s =
        std::min(end_s - router_s + lane_segment.start_s, lane_segment.end_s);
    if (adjusted_start_s < adjusted_end_s) {
      if (!truncated_segments->empty() &&
          truncated_segments->back().lane->id().id() ==
              lane_segment.lane->id().id()) {
        truncated_segments->back().end_s = adjusted_end_s;
      } else if (unique_lanes.find(lane_segment.lane->id().id()) ==
                 unique_lanes.end()) {
        truncated_segments->emplace_back(lane_segment.lane, adjusted_start_s,
                                         adjusted_end_s);
        unique_lanes.insert(lane_segment.lane->id().id());
      } else {
        found_loop = true;
        break;
      }
    }
    router_s += (lane_segment.end_s - lane_segment.start_s);
    if (router_s > end_s) {
      break;
    }
  }
  if (found_loop) {
    return true;
  }
  // Extend the trajectory towards the end of the trajectory.
  if (router_s < end_s && !truncated_segments->empty()) {
    auto &back = truncated_segments->back();
    if (back.lane->total_length() > back.end_s) {
      double origin_end_s = back.end_s;
      back.end_s =
          std::min(back.end_s + end_s - router_s, back.lane->total_length());
      router_s += back.end_s - origin_end_s;
    }
  }
  auto last_lane = segments.back().lane;
  while (router_s < end_s - kRouteEpsilon) {
    last_lane = GetRouteSuccessor(last_lane);
    if (last_lane == nullptr ||
        unique_lanes.find(last_lane->id().id()) != unique_lanes.end()) {
      break;
    }
    const double length = std::min(end_s - router_s, last_lane->total_length());
    truncated_segments->emplace_back(last_lane, 0, length);
    unique_lanes.insert(last_lane->id().id());
    router_s += length;
  }
  return true;
}

void PncMap::AppendLaneToPoints(LaneInfoConstPtr lane, const double start_s,
                                const double end_s,
                                std::vector<MapPathPoint> *const points) {
  if (points == nullptr || start_s >= end_s) {
    return;
  }
  double accumulate_s = 0.0;
  for (size_t i = 0; i < lane->points().size(); ++i) {
    if (accumulate_s >= start_s && accumulate_s <= end_s) {
      points->emplace_back(lane->points()[i], lane->headings()[i],
                           LaneWaypoint(lane, accumulate_s));
    }
    if (i < lane->segments().size()) {
      const auto &segment = lane->segments()[i];
      const double next_accumulate_s = accumulate_s + segment.length();
      if (start_s > accumulate_s && start_s < next_accumulate_s) {
        points->emplace_back(segment.start() + segment.unit_direction() *
                                                   (start_s - accumulate_s),
                             lane->headings()[i], LaneWaypoint(lane, start_s));
      }
      if (end_s > accumulate_s && end_s < next_accumulate_s) {
        points->emplace_back(
            segment.start() + segment.unit_direction() * (end_s - accumulate_s),
            lane->headings()[i], LaneWaypoint(lane, end_s));
      }
      accumulate_s = next_accumulate_s;
    }
    if (accumulate_s > end_s) {
      break;
    }
  }
}

}  // namespace hdmap
}  // namespace apollo
