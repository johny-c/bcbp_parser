#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <string>

int main(int argc, char **argv) {
    const std::string testString1 = "M1ANDERSON/THOMAS     E25SVA7 MUCTLSLH 2220 284M028A0010 35C>2180      B                2922023603973570    LH 992003073260279     *30601001K09         ";
    const std::string testString2 = "M2KUCNER/TOMASZ       E2UY97R ARNAMSKL 1108 102M030C0028 316>503  W0D07489317904052UY97R AMSTLSKL 1311 102M009D0015 30F0D0748931790405";
    const std::string testString3 = "M1LINDER/TIMM         EZUCLTK TLSFRALH 1099 289M007F0034 35C>2180WW5288BLH              2922023595253850 LH                        *30600000K09";


    ros::init(argc, argv, "talker");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate( 1.0/15 ); // Send once every 15 seconds

    int count = 0;
    while (ros::ok()) {

        std_msgs::String msg;

        std::stringstream ss;
        //ss << "hello world " << count;
        ss << testString2;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();

        ++count;
    }


    return 0;
}
