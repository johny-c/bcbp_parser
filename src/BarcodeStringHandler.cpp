#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <string>
#include <map>
#include "BCBP_Parser.h"
#include "DB.h"
#include "bcbp/PassengerInfoItem.h"
#include "bcbp/PassengerInfo.h"
#include "BarcodeStringParser.h"

class BarcodeStringHandler {
public:

    BarcodeStringHandler(bool useDB) {
        //Topic you want to publish
        pub = n.advertise<bcbp::PassengerInfo>("/passenger_info_topic", 1000);

        //Topic you want to subscribe
        sub = n.subscribe("/chatter", 1, &BarcodeStringHandler::callback, this);

        bcsp = BarcodeStringParser::getInstance();

        this->useDB = useDB;
        if (useDB){
            db = DB::getInstance();          
        }
    }

    void callback(const std_msgs::String::ConstPtr& msg) {
        ROS_INFO("I heard: [%s]", msg->data.c_str());

        list<BCBP_Item> items = bcsp->parse(msg->data);


        //            map<string, string> infoMap = bcbpp::getDesiredMap();
        //            bcbpp::printMap();
        //            
        //            bcbp::PassengerInfo pinfo;
        //            for (std::map<string,string>::iterator it=infoMap.begin(); it!=infoMap.end(); ++it){
        //                
        //                bcbp::PassengerInfoItem pii;
        //                pii.description = it->first; 
        //                pii.data = it->second;
        //                pinfo.items.push_back(pii);          
        //            }
        //
        //            // Extract flight carrier and flight number fields
        //            const string flightCarrierKey = BCBP_Item(OPERATING_CARRIER_DESIGNATOR_ID).GetDescription();
        //            const string flightCarrier = infoMap.at(flightCarrierKey);
        //            const string flightNumberKey = BCBP_Item(FLIGHT_NUMBER_ID).GetDescription();
        //            const string flightNumber = infoMap.at(flightNumberKey);            
        //            
        //            //const string flightCarrier = bcbpp::getItem(OPERATING_CARRIER_DESIGNATOR_ID);        
        //            //const string flightNumber = bcbpp::getItem(FLIGHT_NUMBER_ID);
        //
        //            // Query DB for gate number
        //            //const string flightCode = flightCarrier + flightNumber;
        //            //flightCode = flightCode.substr(0, flightCode.length() -1);
        //            //const string flightGate = db.queryGate(flightCode);
        //            
        //            // Example of using a field from a DB query
        //            const string flightGate = db.queryGate(flightCarrier, flightNumber);
        //            bcbp::PassengerInfoItem pii;
        //            pii.description = "Gate";
        //            pii.data = flightGate;
        //            pinfo.items.push_back(pii);
        //            
        //            ROS_INFO("DB responds gate: [%s]", flightGate.c_str());
        //   
        //            pub.publish(pinfo);
    }

private:
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub;
    DB* db;
    BarcodeStringParser* bcsp;
    bool useDB;

};

using std::cout;
using std::cin;

int main(int argc, char **argv) {
    //Initiate ROS
    ros::init(argc, argv, "listener");

    string input = "";
    char reply = {0};
    while (true) {
        cout << "Use DB (MySQL) to query flight information (y/n)?\n";
        getline(cin, input);

        if (input.length() == 1) {
            reply = input[0];
            break;
        }
        cout << "Invalid character, please try again\n";
    }
    bool useDB = (reply == 'y') ? true : false;
    if(useDB){
        cout << "DB use will be attempted.\n";
    }
    else{
        cout << "No DB will be used.\n";
    }

    //Create an object of class SubscribeAndPublish that will take care of everything
    
    BarcodeStringHandler barcodeStringHandler(useDB);
    /**
     * ros::spin() will enter a loop, pumping callbacks.  With this version, all
     * callbacks will be called from within this thread (the main one).  ros::spin()
     * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
     */
    ros::spin();

    return 0;
}



