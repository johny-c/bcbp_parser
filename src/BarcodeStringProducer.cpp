#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <string>
//#include <random>

int main(int argc, char **argv) {
    

    const std::string testString1 = "M1CHIOTELLIS/IOANNIS  E25SVA7 MUCTLSLH 2220 284M028A0010 35C>2180      B                2922023603973570    LH 992003073260279     *30601001K09         ";
    const std::string testString2 = "M2KUCNER/TOMASZ       E2UY97R ARNAMSKL 1108 102M030C0028 316>503  W0D07489317904052UY97R AMSTLSKL 1311 102M009D0015 30F0D0748931790405";
    const std::string testString3 = "M1LINDER/TIMM         EZUCLTK TLSFRALH 1099 289M007F0034 35C>2180WW5288BLH              2922023595253850 LH                        *30600000K09";

    const std::array<std::string, 3> testStrings = {
        testString1, testString2, testString3
    };
    
//    std::random_device rd;
//    std::mt19937 gen(rd());
//    std::uniform_int_distribution<> dis(0, testStrings.size()-1);

    ros::init(argc, argv, "barcode_string_producer");

    ros::NodeHandle n;

    ros::Publisher bcstring_pub = n.advertise<std_msgs::String>("/barcode_string_topic", 1000);

    ros::Rate loop_rate( 1.0/10 ); // Send once every 10 seconds

    int count = 0;
    while (ros::ok()) {

        std_msgs::String msg;

        std::stringstream ss;
        //ss << testStrings[dis(gen)];
        ss << testStrings[ count % testStrings.size()];
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        bcstring_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();

        ++count;
    }


    return 0;
}
