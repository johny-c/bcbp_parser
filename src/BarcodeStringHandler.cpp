#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <string>
#include <map>
#include "DB.h"
#include "bcbp/PassengerInfoItem.h"
#include "bcbp/PassengerInfo.h"
#include "BCBP_Parser.h"
#include "BCBP_Utils.h"
#include "BCBP_ItemIDs.h"


using namespace bcbp;
using namespace bcbp_utils;


class BarcodeStringHandler {
public:

    BarcodeStringHandler(bool useDB) {
        //Topic you want to publish
        pub = n.advertise<bcbp::PassengerInfo>("/passenger_info_topic", 1000);

        //Topic you want to subscribe
        sub = n.subscribe("/barcode_string_topic", 1, &BarcodeStringHandler::callback, this);

        parser = BCBP_Parser::getInstance();

        this->useDB = useDB;
        if (useDB){
            db = DB::getInstance();          
        }
    }

    void callback(const std_msgs::String::ConstPtr& msg) {
        ROS_INFO("Now parsing: [%s]", msg->data.c_str());

        /* Parse all items according to BCBP protocol */
        list<BCBP_Item> items = parser->parse(msg->data);
        printTable(items);

        /* Extract the items we are interested in */
        list<BCBP_Item> desiredItems = extractDesiredItems(items);
        printTable(desiredItems);
        
        /* Construct custom ROS message */
        
        /* Get a map (id => content) for the desired items */
        map<int, string> desiredMap = listToMap(desiredItems);
        //map<string, string> desiredMap = getDesiredMapDescr(desiredItems);
        
        bcbp::PassengerInfo pinfo;
        for (std::map<int, string>::iterator it=desiredMap.begin(); it!=desiredMap.end(); ++it){

            bcbp::PassengerInfoItem pii;
            //pii.id = it->first;
            if(it->first == DATE_OF_FLIGHT_JULIAN_DATE_ID){
                pii.description = "Date of flight";
            }
            else{
                pii.description = BCBP_Item(it->first).GetDescription();
            }
            
            pii.data = it->second;
            pinfo.items.push_back(pii);          
        }
        
        string airDep = "";
        try {
            airDep = desiredMap.at(FROM_CITY_AIRPORT_CODE_ID);
        }
        catch (std::exception &cException) {
            std::cerr << "Standard exception: " << cException.what() << '\n'
                      << "--Different airport of departure\n";
        }
        

        if (useDB && airDep=="AMS"){
            
            const string flightCarrier = desiredMap.at(OPERATING_CARRIER_DESIGNATOR_ID);
            const string flightNumber = desiredMap.at(FLIGHT_NUMBER_ID); 

            // Example of using a field from a DB query
            const string flightGate = db->queryGate(flightCarrier, flightNumber);
            ROS_INFO("DB responds gate: [%s]", flightGate.c_str());
            
            bcbp::PassengerInfoItem pii;
            //pii.id = GATE_ID;
            pii.description = BCBP_Item(GATE_ID).GetDescription();
            pii.data = flightGate;
            pinfo.items.push_back(pii);
        }

        pub.publish(pinfo);
    }

private:
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub;
    DB* db;
    BCBP_Parser* parser;
    bool useDB;
};



using std::cout;
using std::cin;

int main(int argc, char **argv) {
    //Initiate ROS
    ros::init(argc, argv, "barcode_string_handler");

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


    BarcodeStringHandler barcodeStringHandler(useDB);
    /**
     * ros::spin() will enter a loop, pumping callbacks.  With this version, all
     * callbacks will be called from within this thread (the main one).  ros::spin()
     * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
     */
    ros::spin();

    return 0;
}



