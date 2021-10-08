#include "modules/map/hdmap/adapter/osm_adapter/osm_adapter.h"

#include <vector>

#include "cyber/common/log.h"
#include "modules/map/hdmap/adapter/proto_organizer.h"
#include "modules/map/hdmap/adapter/xml_parser/status.h"


namespace apollo {
namespace hdmap {
namespace adapter {
// function declaration
void getRoads(const lanelet::LaneletMapPtr map, const lanelet::routing::RoutingGraphPtr routing_graph_ptr, std::vector<RoadInternal> & roads);


std::string MakeRoadIDFromLanetID(lanelet::Id id)
{
    std::string prefix = "road_";
    return prefix + std::to_string(id);
}
std::string MakesectionIDFromLanetID(lanelet::Id id)
{
    std::string prefix = "section_";
    return prefix + std::to_string(id);
}
std::string MakelaneIDFromLanetID(lanelet::Id id)
{
    std::string prefix = "lane_";
    return prefix + std::to_string(id);
}

OSMAdapter::OSMAdapter()
{
    std::string osm_file = "/home/ljj/autoT/localization_msgs/osmmap/m1.osm";
// ------利用lanelet2 api 把地图load进来------
lanelet::ErrorMessages errors;
utm = std::make_shared<lanelet::projection::UtmProjector >(lanelet::Origin({22.68261025625,114.19688142769}));
map = lanelet::load(osm_file, *utm, &errors);

// --------利用基础地图构造一个lanelet2 routing graph-------
lanelet::traffic_rules::TrafficRulesPtr traffic_rules = 
    lanelet::traffic_rules::TrafficRulesFactory::create(lanelet::Locations::Germany, lanelet::Participants::Vehicle);
routing_graph_ptr = lanelet::routing::RoutingGraph::build(*map, *traffic_rules);

}



bool OSMAdapter::LoadData(apollo::hdmap::Map* pb_map) {
std::cout << "start loading map ..." <<std::endl;





// 检查pb_map结构是否为空
CHECK_NOTNULL(pb_map);


/**
 * 先构造pbmap的header，这个直接写个函数赋值就行
 * */
PbHeader* map_header = pb_map->mutable_header();        // 待填入信息的pb map header



/**
 * 利用lanelet2接口得到一个apollo可用的road向量
 * */
std::vector<RoadInternal> roads;                        // 待填入信息的roads结构
getRoads(map, routing_graph_ptr, roads);    


/**
 * junction 结构
 * */
std::vector<JunctionInternal> junctions;                // 待填入信息的junctions结构



/**
 * Object 结构
 * */
ObjectInternal objects;                                 // 待填入信息的obj结构




/**
 * 填完了上面的信息之后，直接调用proto_organizer的接口，就可以得到
 * */
ProtoOrganizer proto_organizer;
proto_organizer.GetRoadElements(&roads);
// // 开始的时候没有junction，里面不进行循环，直接退出，下面的应该同理
// proto_organizer.GetJunctionElements(junctions);
// proto_organizer.GetObjectElements(objects);
// proto_organizer.GetOverlapElements(roads, junctions);
proto_organizer.OutputData(pb_map);

return true;

}


void getRoadInternal(RoadInternal& road_internal)
{
    
}

void getright_lane_boundary(PbLaneBoundary* lane_boundary, lanelet::Lanelet & lane)
{
        // 考虑right lane boundary 的curve
        PbCurve* lane_boundary_curve = lane_boundary->mutable_curve();
        // 对所有的右侧边界上的点进行循环
        // std::cout << "lanelet lane boundary point size ： " << lane.leftBound3d().size()<<std::endl;
        double boundary_length = 0.0;
        for(int i = 0; i < lane.rightBound3d().size()-1; i++)
        {
            lanelet::Point3d& cur_point = lane.rightBound3d()[i];
            lanelet::Point3d& next_point = lane.rightBound3d()[i+1];
            double length = sqrt((next_point.y()-cur_point.y()) * (next_point.y()-cur_point.y()) + 
                                    (next_point.x()-cur_point.x()) * (next_point.x()-cur_point.x()));
            // 注意这里的point 已经表达在enu下
            
            PbCurveSegment* curve_segment = lane_boundary_curve->add_segment();
            curve_segment->mutable_start_position()->set_x(cur_point.x());
            curve_segment->mutable_start_position()->set_y(cur_point.y());
            curve_segment->set_length(length);
            curve_segment->set_s(boundary_length);
            boundary_length += lane_boundary_curve->segment(i).length();
            
        }

        // 考虑lane boundary length
        lane_boundary->set_length(boundary_length);

        // 考虑lane boundary type
        LaneBoundaryType * lane_boundary_type = lane_boundary->add_boundary_type();
        lane_boundary_type->set_s(0.0); // 这里是0，因为一个lanelet只有一种boundary
        lane_boundary_type->add_types(hdmap::LaneBoundaryType::SOLID_WHITE);
}
void getleft_lane_boundary(PbLaneBoundary* lane_boundary, lanelet::Lanelet & lane)
{
        // 考虑right lane boundary 的curve
        PbCurve* lane_boundary_curve = lane_boundary->mutable_curve();
        // 对所有的右侧边界上的点进行循环
        // std::cout << "lanelet lane boundary point size ： " << lane.leftBound3d().size()<<std::endl;
        double boundary_length = 0.0;
        for(int i = 0; i < lane.leftBound3d().size()-1; i++)
        {
            lanelet::Point3d& cur_point = lane.leftBound3d()[i];
            lanelet::Point3d& next_point = lane.leftBound3d()[i+1];
            double length = sqrt((next_point.y()-cur_point.y()) * (next_point.y()-cur_point.y()) + 
                                    (next_point.x()-cur_point.x()) * (next_point.x()-cur_point.x()));
            // 注意这里的point 已经表达在enu下
            
            PbCurveSegment* curve_segment = lane_boundary_curve->add_segment();
            curve_segment->mutable_start_position()->set_x(cur_point.x());
            curve_segment->mutable_start_position()->set_y(cur_point.y());
            curve_segment->set_length(length);
            curve_segment->set_s(boundary_length);
            boundary_length += lane_boundary_curve->segment(i).length();
            
        }

        // 考虑lane boundary length
        lane_boundary->set_length(boundary_length);

        // 考虑lane boundary type
        LaneBoundaryType * lane_boundary_type = lane_boundary->add_boundary_type();
        lane_boundary_type->set_s(0.0); // 这里是0，因为一个lanelet只有一种boundary
        lane_boundary_type->add_types(hdmap::LaneBoundaryType::SOLID_WHITE);
}

void getCentral_curve(PbCurve* pb_lane_center, double& centerline_length, lanelet::Lanelet & lane )
{
    // -------------------------
    // 第一次写的方法
    // -------------------------
    // for(int i = 0; i < lane.centerline().size()-1; i++)
    // {
    //          auto & cur_point = lane.centerline()[i];
    //          auto & next_point = lane.centerline()[i+1];
    //         double length = sqrt((next_point.y()-cur_point.y()) * (next_point.y()-cur_point.y()) + 
    //                                 (next_point.x()-cur_point.x()) * (next_point.x()-cur_point.x()));
    //         // 注意这里的point 已经表达在enu下
    //         PbCurveSegment* curve_segment = pb_lane_center->add_segment();
    //         curve_segment->mutable_start_position()->set_x(cur_point.x());
    //         curve_segment->mutable_start_position()->set_y(cur_point.y());
    //         curve_segment->set_length(length);
    //         curve_segment->set_s(centerline_length);
    //         centerline_length += curve_segment->length();
    // }

    //--------------------------
    // 看hdmapcommon_test 可以发现第一次写的不对
    // -------------------
    CurveSegment* curve_segment = pb_lane_center->add_segment();
    curve_segment->mutable_start_position()->set_x(lane.centerline()[0].x());
    curve_segment->mutable_start_position()->set_y(lane.centerline()[0].y());


    LineSegment* line_segment = curve_segment->mutable_line_segment();
    
    apollo::common::PointENU* pt = line_segment->add_point();
    pt->set_x(lane.centerline()[0].x());
    pt->set_y(lane.centerline()[0].y());
    
    for(int i = 1; i < lane.centerline().size(); i++)
    {
        
        pt = line_segment->add_point();
        pt->set_x(lane.centerline()[i].x());
        pt->set_y(lane.centerline()[i].y());
        auto & cur_point = lane.centerline()[i-1];
        auto & next_point = lane.centerline()[i];
        double length = sqrt((next_point.y()-cur_point.y()) * (next_point.y()-cur_point.y()) + 
                                    (next_point.x()-cur_point.x()) * (next_point.x()-cur_point.x()));

        centerline_length += length;
    }
    curve_segment->set_length(centerline_length);
    curve_segment->set_s(0.0);
}
void getRoads(const lanelet::LaneletMapPtr map, const lanelet::routing::RoutingGraphPtr routing_graph_ptr, std::vector<RoadInternal> & roads)
{
    // 清空一下待输出量
    roads.clear();
    
    // 对每一个lanelet进行for循环
    for (lanelet::Lanelet & lane : map->laneletLayer) 
    {
        /**
         * 这个函数我们先假定 一个lanelet 就是一个road
         * 也就是说，一个road包含一个lanesection
         * 一个lane section 包含一个lane
         * 一个lane，就是一个lanelet
         * 
         * */
        // 所以我们在for循环的开始处创建一个RoadInternal
        RoadInternal road_internal;
        
        // 先处理road internal 里面的两个id
        road_internal.id = MakeRoadIDFromLanetID(lane.id());
        road_internal.road.mutable_id()->set_id(road_internal.id);
        // std::cout << "road_internal id : "<<road_internal.id <<std::endl;

        // 由于目前还没讨论junction ， 所以junction id 是空的


        // 处理road type
        PbRoadType pb_road_type = apollo::hdmap::Road::CITY_ROAD;
        road_internal.road.set_type(pb_road_type);

        // 下面开始处理lane
        // 先构造一个road section
        RoadSectionInternal section_internal;
        section_internal.id = MakesectionIDFromLanetID(lane.id());
        section_internal.section.mutable_id()->set_id(section_internal.id);

        // 构造一个lane internal
        LaneInternal lane_internal;
        PbLane* pblane = &lane_internal.lane;

        // 先放lane id
        pblane->mutable_id()->set_id(MakelaneIDFromLanetID(lane.id()));

        // 再放lane type
        PbLaneType pb_lane_type = hdmap::Lane::CITY_DRIVING;
        pblane->set_type(pb_lane_type);

        // 考虑right lane boundary 
        PbLaneBoundary* right_lane_boundary = pblane->mutable_right_boundary();
        getright_lane_boundary(right_lane_boundary, lane);
        
        






        // 考虑left lane boundary 
        PbLaneBoundary* left_lane_boundary = pblane->mutable_left_boundary();
        getleft_lane_boundary(left_lane_boundary, lane);




        // 设置 turn 类型
        pblane->set_turn(hdmap::Lane::NO_TURN); // 目前什么都不知道的情况下，先设置no turn
        // 设置 direction 不知道干啥的
        pblane->set_direction(hdmap::Lane::FORWARD);

        // 考虑linkage ， 道路连接关系
        // 添加后继lane
        lanelet::ConstLanelets following_lanes = routing_graph_ptr->following(lane);
        for(int i = 0; i < following_lanes.size(); i++)
        {
            PbID* pb_lane_id = pblane->add_successor_id();
            pb_lane_id->set_id(MakelaneIDFromLanetID(following_lanes[i].id()));
        }
        // 添加前序lane
        lanelet::ConstLanelets previous_lanes = routing_graph_ptr->previous(lane);
        for(int i = 0; i < previous_lanes.size(); i++)
        {
            PbID* pb_lane_id = pblane->add_predecessor_id();
            pb_lane_id->set_id(MakelaneIDFromLanetID(previous_lanes[i].id()));
        }

        // 现在要添加左右两侧的lane，这里正常来说需要写复杂的逻辑
        // 比如，先看看是不是left，再看看是不是adjacent，在看看是不是oppositie等等
        // 考虑到目前应用的地形，直接查询opposite就行

        
        // int coun= 0;
        lanelet::Lanelets left_op_lanes = map->laneletLayer.findUsages(lane.leftBound().invert());
        for(int i = 0; i<left_op_lanes.size(); i++)
        {
            pblane->add_left_neighbor_reverse_lane_id()->set_id(MakelaneIDFromLanetID(left_op_lanes[i].id()));
            // std::cout << " num of reverse lane : "<<++coun<<std::endl;
        }

        // selfReverse 不知道是什么类型的id，先不管它


        // 添加中心线
        PbCurve* pb_lane_center = pblane->mutable_central_curve();
        double centerline_length = 0.0;
        getCentral_curve(pb_lane_center, centerline_length, lane);

        // 添加中心线长度
        pblane->set_length(centerline_length);

        // 添加最高限速
        pblane->set_speed_limit(5.0); // m/s

        // 添加lane sample association
        centerline_length = 0.0;
        for(int i = 0; i< lane.centerline().size(); i++)
        {
            if(i == lane.centerline().size()-1)
            {
                auto left_lane_sample = pblane->add_left_sample();
                auto right_lane_sample = pblane->add_right_sample();
                left_lane_sample->set_s(centerline_length);
                right_lane_sample->set_s(centerline_length);
                left_lane_sample->set_width(1.75);  // m 大路宽的一半
                right_lane_sample->set_width(1.75);  // m 路宽的一半
                break;
            }
            auto & cur_point = lane.centerline()[i];
             auto & next_point = lane.centerline()[i+1];
            double length = sqrt((next_point.y()-cur_point.y()) * (next_point.y()-cur_point.y()) + 
                                    (next_point.x()-cur_point.x()) * (next_point.x()-cur_point.x()));

            auto left_lane_sample = pblane->add_left_sample();
            auto right_lane_sample = pblane->add_right_sample();
            left_lane_sample->set_s(centerline_length);
            right_lane_sample->set_s(centerline_length);
            left_lane_sample->set_width(1.75);  // m 大路宽的一半
            right_lane_sample->set_width(1.75);  // m 路宽的一半
            centerline_length += length;
        }

        // 添加road sample association
        centerline_length = 0.0;
        for(int i = 0; i< lane.centerline().size(); i++)
        {
            if(i == lane.centerline().size()-1)
            {
                auto left_road_sample = pblane->add_left_road_sample();
                auto right_road_sample = pblane->add_right_road_sample();
                left_road_sample->set_s(centerline_length);
                right_road_sample->set_s(centerline_length);
                left_road_sample->set_width(5.25);  // m 大路宽的一半
                right_road_sample->set_width(1.75);  // m 路宽的一半
                break;
            }
            auto & cur_point = lane.centerline()[i];
             auto & next_point = lane.centerline()[i+1];
            double length = sqrt((next_point.y()-cur_point.y()) * (next_point.y()-cur_point.y()) + 
                                    (next_point.x()-cur_point.x()) * (next_point.x()-cur_point.x()));

            auto left_road_sample = pblane->add_left_road_sample();
            auto right_road_sample = pblane->add_right_road_sample();
            left_road_sample->set_s(centerline_length);
            right_road_sample->set_s(centerline_length);
            left_road_sample->set_width(5.25);  // m 大路宽的一半
            right_road_sample->set_width(1.75);  // m 路宽的一半
            centerline_length += length;
        }

        











        // 把这个lane internal 加入到section 中
        section_internal.lanes.push_back(lane_internal);

        // 最后再把这个RoadSectionInternal ，添加到raod中
        road_internal.sections.push_back(section_internal);
        // 然后在for循环的结尾处，将这个组织好的road，加入到roads数组就行
        roads.push_back(road_internal);




        // // 测试lanelet的left right关键词检索情况
        // {
        //     const auto test_lanes = routing_graph_ptr->left(lane);
        //     if(!!test_lanes)
        //     {
        //         std::cout << "left member: "<<test_lanes.get().id()<<std::endl;
        //     }
        //     else
        //     {
        //         std::cout << "no left member"<<std::endl;   
        //     }
        // }

        // // 测试lanelet的adjacent left right关键词检索情况
        // {
        //     const auto ttest_lanes = routing_graph_ptr->adjacentLeft(lane);
        //     if(!!ttest_lanes)
        //     {
        //         std::cout << "adjacentLeft member: "<<ttest_lanes.get().id()<<std::endl;
        //     }
        //     else
        //     {
        //         std::cout << "no adjacentLeft member"<<std::endl;   
        //     }
        // }
        // // 测试lanelet的findusage关键词检索情况
        // {
        //     lanelet::Lanelets tlanelets = map->laneletLayer.findUsages(lane.leftBound().invert());
            
        //     if(!tlanelets.empty())
        //     {
        //         std::cout << "opposite lane: "<<tlanelets[0].id()<<std::endl;
        //     }
        //     else
        //     {
        //         std::cout << "no opposite member"<<std::endl;   
        //     }
        // }





        


        
    
    }

}
}  // namespace adapter
}  // namespace hdmap
}  // namespace apollo
