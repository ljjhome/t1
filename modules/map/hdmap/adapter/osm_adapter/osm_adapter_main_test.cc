#include "modules/map/hdmap/adapter/osm_adapter/osm_adapter.h"
#include <iostream>
#include <string>
#include "cyber/common/file.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
using namespace apollo::hdmap::adapter;
int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
    apollo::hdmap::Map hdmap;
    std::cout << "test starts "<<std::endl;
    
    OSMAdapter osm;
    osm.LoadData(&hdmap);
    std::string output_bin_map = "/home/ljj/apollo/artest/map/base_map.bin";
    apollo::cyber::common::SetProtoToBinaryFile(hdmap,output_bin_map);

    std::cout << "test ends" << std::endl;

    ros::init(argc, argv, "pbtest");
    ros::NodeHandle pnh("~");

    ros::Publisher pbpub = pnh.advertise<std_msgs::String>("/ljj/pbmsg", 1, true);
    ros::Rate loop_rate(1);
    
    std::string pbstring;
    bool flag = hdmap.SerializeToString(&pbstring);
    // msg = pbstring;std_msgs::String msg;
    std_msgs::String msg;
    msg.data = pbstring;
    while(ros::ok())
    {
      loop_rate.sleep();
      ros::spinOnce();
      pbpub.publish(msg);
    }
    return 0;
}