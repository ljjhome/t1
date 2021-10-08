#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>  
#include <visualization_msgs/Marker.h>

#include <iostream>
#include <vector>
#include <string>

// lanelet 
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingGraph.h>

#include "modules/routing/proto/routing.pb.h"
#include <std_msgs/String.h>

struct waypoint3d
{
    double x = 0;
    double y = 0;
    double z = 0;
};

waypoint3d pstart, pgoal;
/**
 * some global variable
 * */
/// The start pose set through RViz
geometry_msgs::PoseWithCovarianceStamped start;
/// The goal pose set through RViz
geometry_msgs::PoseStamped goal;
/// A publisher publishing the start position for RViz
ros::Publisher pubStart;
ros::Publisher pubGoal;
ros::Publisher pubrequest;
ros::Subscriber subGoal;
ros::Subscriber subStart;

// start goal state
bool start_ok = false;
bool goal_ok = false;

using namespace apollo::routing;

/**
 * start point callback 
 * */
void setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start)\
{
    std::cout << "get start point : "<< std::endl;
    // visualize 
    visualization_msgs::Marker start_marker;
    std_msgs::ColorRGBA color_start_marker;
    color_start_marker.r = 0;
    color_start_marker.g = 1;
    color_start_marker.b = 0;
    color_start_marker.a = 0.5;
    start_marker.header.frame_id = "map";
    start_marker.header.stamp = ros::Time();
    start_marker.ns = "start_goal_marker";
    start_marker.id = 1;
    start_marker.type = 0; // "arrow"
    start_marker.action = 0; // 0:add or modify
    start_marker.scale.x = 2.0; start_marker.scale.y = 0.4; start_marker.scale.z = 0.4;
    start_marker.pose = start->pose.pose;
    start_marker.color = color_start_marker;
    pubStart.publish(start_marker);

    pstart.x = start->pose.pose.position.x;
    pstart.y = start->pose.pose.position.y;
    pstart.z = start->pose.pose.position.z;
    start_ok = true;
}   


/**
 * goal point callback
 * */
void setGoal(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
    std::cout << "get goal point : "<< std::endl;
    visualization_msgs::Marker goal_marker;
    std_msgs::ColorRGBA color_goal_marker;
    color_goal_marker.r = 1;
    color_goal_marker.g = 0;
    color_goal_marker.b = 0;
    color_goal_marker.a = 0.5;
    goal_marker.header.frame_id = "map";
    goal_marker.header.stamp = ros::Time();
    goal_marker.ns = "start_goal_marker";
    goal_marker.id = 2;
    goal_marker.type = 0; // "arrow"
    goal_marker.action = 0; // 0:add or modify
    goal_marker.scale.x = 2.0; goal_marker.scale.y = 0.4; goal_marker.scale.z = 0.4;
    goal_marker.pose = goal->pose;
    goal_marker.color = color_goal_marker;
    pubGoal.publish(goal_marker);

    pgoal.x = goal->pose.position.x;
    pgoal.y = goal->pose.position.y;
    pgoal.z = goal->pose.position.z;
    goal_ok = true;
}


int main(int argc, char** argv)
{
    std::cout << "sim planner start ..." <<std::endl;
    ros::init(argc, argv, "rosrouting_sim_planner");
    ros::NodeHandle pnh("~");

    subGoal = pnh.subscribe("/move_base_simple/goal", 1, setGoal);
    subStart = pnh.subscribe("/initialpose", 1, setStart);

    pubStart = pnh.advertise<visualization_msgs::Marker>("/ljj/start_point",1 , true);
    pubGoal = pnh.advertise<visualization_msgs::Marker>("/ljj/goal_point",1 , true);

    pubrequest = pnh.advertise<std_msgs::String>("/planning/request",1,true);

    ros::Rate loop_rate(1);
    while(ros::ok())
    {
        if(start_ok && goal_ok)
        {
            RoutingRequest pbrequest;
            auto wp = pbrequest.add_waypoint();
            wp->mutable_pose()->set_x(pstart.x); 
            wp->mutable_pose()->set_y(pstart.y); 
            wp->mutable_pose()->set_z(pstart.z); 


            std::cout << "start point : "<< pbrequest.waypoint(0).pose().x()<< " , "<<
                                            pbrequest.waypoint(0).pose().y()<< " , "<<
                                            pbrequest.waypoint(0).pose().z()<< std::endl;
            wp = pbrequest.add_waypoint();
            wp->mutable_pose()->set_x(pgoal.x); 
            wp->mutable_pose()->set_y(pgoal.y); 
            wp->mutable_pose()->set_z(pgoal.z); 
            std::cout << "goal point : "<< pbrequest.waypoint(1).pose().x()<< " , "<<
                                            pbrequest.waypoint(1).pose().y()<< " , "<<
                                            pbrequest.waypoint(1).pose().z()<< std::endl;
            std::string str_request;
            bool flag = pbrequest.SerializeToString(&str_request);
            std_msgs::String msg;
            msg.data = str_request;
            pubrequest.publish(msg);
        }
        loop_rate.sleep();
        ros::spinOnce();

    }

    ros::spin();
    return 0;

}