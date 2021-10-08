#include "modules/routing/rosrouting/rosrouting_node.h"
#include "modules/routing/proto/routing.pb.h"

namespace apollo {
namespace routing {
rosRoutingNode::rosRoutingNode():pnh_("~")
{
    bool init_ok = init();
    if(init_ok)
    {
        std::cout << "init ok "<<std::endl;
    }
    else
    {
        std::cout << "init not ok "<<std::endl;
    }
}

bool rosRoutingNode::init()
{
    response_pub = pnh_.advertise<std_msgs::String>("/routing/response",1);
    request_sub = pnh_.subscribe("/planning/request", 1, &rosRoutingNode::Proc, this);

    return routing_.Init().ok() && routing_.Start().ok();
}

void rosRoutingNode::Proc(const std_msgs::String::Ptr msg)
{
    RoutingRequest pbrequest;
    std::string strrequest = msg->data;
    pbrequest.ParseFromString(strrequest);
    std::shared_ptr<RoutingRequest> request = std::make_shared<RoutingRequest>(pbrequest);   

    
    // 构造一个response变量
    auto response = std::make_shared<RoutingResponse>();

    std::cout << "start point : "<< request->waypoint(0).pose().x() << " , "
                                << request->waypoint(0).pose().y() << " , "
                                << request->waypoint(0).pose().z() << std::endl;
    std::cout << "goal point : "<< request->waypoint(1).pose().x() << " , "
                                << request->waypoint(1).pose().y() << " , "
                                << request->waypoint(1).pose().z() << std::endl;
    // 直接调用process函数，把请求给进去，然后输出一个response
    if (!routing_.Process(request, response.get())) {
        std::cout << "routing failed" <<std::endl;
        return ;
    }
    std::cout << "routing success " <<std::endl;
}

}  // namespace routing
}  // namespace apollo
