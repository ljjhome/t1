#include "modules/routing/rosrouting/rosrouting_node.h"

#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosrouting_node");
    apollo::routing::rosRoutingNode routingnode;
    ros::spin();
    return 0;
}