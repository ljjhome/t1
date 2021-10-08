#pragma once
// ros
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <memory>

#include "modules/routing/rosrouting/rosrouting.h"

namespace apollo {
namespace routing {

class rosRoutingNode
{
public:
    rosRoutingNode();
    bool init();
    void Proc(const std_msgs::String::Ptr msg);
private:
    ros::NodeHandle pnh_;
    ros::Publisher response_pub;
    ros::Subscriber request_sub;

    Routing routing_;
    


};


}  // namespace routing
}  // namespace apollo
