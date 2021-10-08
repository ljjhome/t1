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

#include <limits>
#include <unordered_map>

#include "modules/routing/rosrouting/rosrouting.h"

#include "modules/common/util/point_factory.h"
#include "modules/routing/common/routing_gflags.h"

namespace apollo {
namespace routing {

using apollo::common::ErrorCode;
using apollo::common::PointENU;
using apollo::hdmap::ParkingSpaceInfoConstPtr;

std::string Routing::Name() const { return FLAGS_routing_node_name; }

Routing::Routing(){}
    //: monitor_logger_buffer_(common::monitor::MonitorMessageItem::ROUTING) {}

apollo::common::Status Routing::Init() {

  // 先找到通过creator生成的routing file
  const auto routing_map_file = apollo::hdmap::RoutingMapFile();
  AINFO << "Use routing topology graph path: " << routing_map_file;
  // 用这个routing file 去初始化一个navigator
  navigator_ptr_.reset(new Navigator(routing_map_file));
  // 获取hdmap
  hdmap_ = apollo::hdmap::HDMapUtil::BaseMapPtr();
  ACHECK(hdmap_) << "Failed to load map file:" << apollo::hdmap::BaseMapFile();
  // 初始化完成
  return apollo::common::Status::OK();
}

// 看看navigator的状态，ready了就是可以start了
apollo::common::Status Routing::Start() {
  if (!navigator_ptr_->IsReady()) {
    AERROR << "Navigator is not ready!";
    return apollo::common::Status(ErrorCode::ROUTING_ERROR,
                                  "Navigator not ready");
  }
  AINFO << "Routing service is ready.";
  AINFO << "Routing started";
  // monitor_logger_buffer_.INFO("Routing started");
  return apollo::common::Status::OK();
}

std::vector<RoutingRequest> Routing::FillLaneInfoIfMissing(
    const RoutingRequest& routing_request) {
  // 定义了一个request序列，不知道要干啥
  std::vector<RoutingRequest> fixed_requests;
  // 又定义了一个unordered_map，key是int value是一个点列
  std::unordered_map<int, std::vector<LaneWaypoint>>
      additional_lane_waypoint_map;
  // 用输入的routing_request，构造一个fixed_request对象
  RoutingRequest fixed_request(routing_request);
  // 对所有要经过的点进行循环
  for (int i = 0; i < routing_request.waypoint_size(); ++i) {
    std::cout << "in FillLaneInfoIfMissing for loop "<<std::endl;
    // 记录下当前要经过的点
    LaneWaypoint lane_waypoint(routing_request.waypoint(i));
    // 如果这个点有id这一项，就跳过？为啥？
    if (lane_waypoint.has_id()) {
      continue;
    }
    //
    // fill lane info when missing
    // 如果没有id这一项
    // 先把这个点当中pose拿出来，构造成一个enu点
    const auto point =
        common::util::PointFactory::ToPointENU(lane_waypoint.pose());
    // 定义一个laneinfo序列
    std::vector<std::shared_ptr<const hdmap::LaneInfo>> lanes;
    // look for lanes with bigger radius if not found
    // 在这个点周围的一定范围内搜索lane
    // constexpr double kRadius = 0.3;
    constexpr double kRadius = 1;
    // 这个循环是不断扩大半径的长度，如果找不到的话就扩大
    std::cout << "before find lane for loop"<<std::endl;  
    for (int i = 0; i < 20; ++i) {
      std::cout << "in find lane for loop : "<< i <<std::endl;
      // 去找最近的lane，找到就break出去
      hdmap_->GetLanes(point, kRadius + i * kRadius, &lanes);
      std::cout << "after get lanes in for loop "<<std::endl;
      if (lanes.size() > 0) {
        break;
      }
    }
    //  如果一直找不到，就报错，说找不到
    if (lanes.empty()) {
      AERROR << "Failed to find nearest lane from map at position: "
             << point.DebugString();
      return fixed_requests;  // return empty vector
    }
    // 对刚刚找到的lane进行循环
    for (size_t j = 0; j < lanes.size(); ++j) {
      double s = 0.0;
      double l = 0.0;
      // 求一个点在lane上的投影sl坐标
      lanes[j]->GetProjection({point.x(), point.y()}, &s, &l);
      // 如果是第一条lane
      if (j == 0) {
        // 记录最近的lane 的 id 以及point在其上的投影位置s
        auto waypoint_info = fixed_request.mutable_waypoint(i);
        waypoint_info->set_id(lanes[j]->id().id());
        waypoint_info->set_s(s);
      } else {
        // additional candidate lanes
        // 如果不是第一个点就记录到，additional_lane_waypoint_map里面
        // 这些点相当于是有可能替代点
        LaneWaypoint new_lane_waypoint(lane_waypoint);
        new_lane_waypoint.set_id(lanes[j]->id().id());
        new_lane_waypoint.set_s(s);
        additional_lane_waypoint_map[i].push_back(new_lane_waypoint);
      }
    }// 结束对一定范围内最近lane的 循环
  } // 结束对所有请求点的循环
  // first routing_request
  // 把上面得到的fixed_request push进去
  fixed_requests.push_back(fixed_request);

  // additional routing_requests because of lane overlaps
  // 对刚刚生成的所有的候选lane进行循环
  // 注意additional_lane_waypoint_map这个map中，共有i个关系，i是请求点的数目
  // 这三个for循环，实质上就是在做排列组合，比如有五个必经路点，每个路点有三个最近的路，然后去排列组合
  // 最终就能得到fixed_requests
  // 返回这个fixed_requests
  for (const auto& m : additional_lane_waypoint_map) {
    size_t cur_size = fixed_requests.size();
    for (size_t i = 0; i < cur_size; ++i) {
      // use index to iterate while keeping push_back
      for (const auto& lane_waypoint : m.second) {
        RoutingRequest new_request(fixed_requests[i]);
        auto waypoint_info = new_request.mutable_waypoint(m.first);
        waypoint_info->set_id(lane_waypoint.id());
        waypoint_info->set_s(lane_waypoint.s());
        fixed_requests.push_back(new_request);
      }
    }
  }

  for (const auto& fixed_request : fixed_requests) {
    ADEBUG << "Fixed routing request:" << fixed_request.DebugString();
  }
  return fixed_requests;
}

bool Routing::GetParkingID(const PointENU& parking_point,
                           std::string* parking_space_id) {
  // search current parking space id associated with parking point.
  constexpr double kDistance = 0.01;  // meter
  std::vector<ParkingSpaceInfoConstPtr> parking_spaces;
  if (hdmap_->GetParkingSpaces(parking_point, kDistance, &parking_spaces) ==
      0) {
    *parking_space_id = parking_spaces.front()->id().id();
    return true;
  }
  return false;
}

bool Routing::FillParkingID(RoutingResponse* routing_response) {
  const auto& routing_request = routing_response->routing_request();
  const bool has_parking_info = routing_request.has_parking_info();
  const bool has_parking_id =
      has_parking_info && routing_request.parking_info().has_parking_space_id();
  // return early when has parking_id
  if (has_parking_id) {
    return true;
  }
  // set parking space ID when
  //  has parking info && has parking point && NOT has parking space id && get
  //  ID successfully
  if (has_parking_info && routing_request.parking_info().has_parking_point()) {
    const PointENU parking_point =
        routing_request.parking_info().parking_point();
    std::string parking_space_id;
    if (GetParkingID(parking_point, &parking_space_id)) {
      routing_response->mutable_routing_request()
          ->mutable_parking_info()
          ->set_parking_space_id(parking_space_id);
      return true;
    }
  }
  ADEBUG << "Failed to fill parking ID";
  return false;
}

// 这个就是处理函数
/**
 * @param routing_request 输入的请求
 * @param routing_response 输出的结果
 * */
bool Routing::Process(const std::shared_ptr<RoutingRequest>& routing_request,
                      RoutingResponse* const routing_response) {
  // 检查结果内存不为空
  CHECK_NOTNULL(routing_response);
  std::cout << "before Afinfo"<<std::endl;
  AINFO << "Get new routing request:" << routing_request->DebugString();
  
  const auto& fixed_requests = FillLaneInfoIfMissing(*routing_request);
  double min_routing_length = std::numeric_limits<double>::max();
  // 对每一个fixed_requests进行循环，
  for (const auto& fixed_request : fixed_requests) {
    std::cout << "in for loop"<<std::endl;
    // 先构造一个response
    RoutingResponse routing_response_temp;
    // 然后直接去search
    // 要仔细看这个search route函数
    if (navigator_ptr_->SearchRoute(fixed_request, &routing_response_temp)) {
      // 如果search出结果就去记录整个routing 的长度
      const double routing_length =
          routing_response_temp.measurement().distance();
      // 从中选出一个最小的长度
      if (routing_length < min_routing_length) {
        routing_response->CopyFrom(routing_response_temp);
        min_routing_length = routing_length;
      }
    }
    FillParkingID(routing_response);
  }
  if (min_routing_length < std::numeric_limits<double>::max()) {
    //monitor_logger_buffer_.INFO("Routing success!");
    AINFO<<"Routing success!";
    return true;
  }

  AERROR << "Failed to search route with navigator.";
  // monitor_logger_buffer_.WARN("Routing failed! " +
  //                             routing_response->status().msg());
  AINFO<<"Routing failed! ";
  return false;
}

}  // namespace routing
}  // namespace apollo
