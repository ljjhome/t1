#include <ros/ros.h>
#include <std_msgs/String.h>
#include "modules/map/hdmap/adapter/osm_adapter/osm_adapter.h"
#include <string>
void pbmsgCallback(const std_msgs::String::Ptr msg)
{
    std::string pbstring = msg->data;
    apollo::hdmap::Map hdmap;
    hdmap.ParseFromString(pbstring);
    std::cout << "map lane size : "<< hdmap.lane_size()<<std::endl;

}
    
int main(int argc, char** argv)
{
    ros::init(argc, argv, "pblistener");
    ros::NodeHandle pnh("~");
    ros::Subscriber pbsub = pnh.subscribe("/ljj/pbmsg", 1, pbmsgCallback);
    ros::spin();
    return 0;
}