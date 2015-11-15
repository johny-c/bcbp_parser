#include <map>
#include <iomanip>
#include "BCBP_Utils.h"
#include "BCBP_Item.h"
#include "boost/date_time/gregorian/gregorian.hpp"
#include "boost/algorithm/string/trim.hpp"

using std::setw;
using std::cout;
using std::map;
using std::pair;

namespace bcbp_utils{

/* Get interpretation of Passenger Description field (0-7) */
string interpretPassengerDescription(char c){
    string passengerDescr;
        switch (c) {
            case '0': passengerDescr = "Adult";
                break;
            case '1': passengerDescr = "Male";
                break;
            case '2': passengerDescr = "Female";
                break;
            case '3': passengerDescr = "Child";
                break;
            case '4': passengerDescr = "Infant";
                break;
            case '5': passengerDescr = "No passenger(cabin baggage)";
                break;
            case '6': passengerDescr = "Adult travelling with infant";
                break;
            case '7': passengerDescr = "Unaccompanied minor";
                break;
            default: passengerDescr = "-"; // Unknown
                break;
        }    
        
    return passengerDescr;
}



string convertJulianToGregorianDate(int dayOfYear){
    
    using namespace boost::gregorian;
    int curYear = day_clock::local_day().year();
    //int curYear = second_clock::local_time.date().year();
    date d(curYear, Jan, 1);
    date_duration dd(dayOfYear - 1);
    date gregorian = d + dd;
    
    return to_simple_string(gregorian); 
}



string extractPassengerName(string passengerName){
    boost::algorithm::trim(passengerName);
    static const string delimiter = "/";
    int splitPos = passengerName.find(delimiter);
    //        const string lastName = passengerName.substr(0, splitPos);
    //        const string firstName = passengerName.substr(splitPos+1, passengerName.length());
    //        mMap.insert ( pair<int,string>(PASSENGER_NAME_ID, passengerName) );

    passengerName.replace(splitPos, delimiter.length(), " ");   
    return passengerName;
}



/* Extract flight class */
string extractCompartmentCode(char c){  
    string compartmentCode;
        switch (c) {
            case 'F': compartmentCode = "First class";
                break;
            case 'J': compartmentCode = "Business Class Premium";
                break;
            case 'Y': compartmentCode = "Economy";
                break;
            default: compartmentCode = "-"; // Unknown
                break;
        }    
        
    return compartmentCode;    
}


/* Extract current passenger status */
string extractPassengerStatus(char c){
    string passengerStatus;
    switch (c) {
        case '0': passengerStatus = "ticket issuance/passenger not checked in";
            break;
        case '1': passengerStatus = "ticket issuance/passenger not checked in";
            break;
        case '2': passengerStatus = "Baggage checked/passenger checked in";
            break;
        case '3': passengerStatus = "ticket issuance/passenger not checked in";
            break;
        case '4': passengerStatus = "Passenger passed security check";
            break;
        case '5': passengerStatus = "Passenger passed gate exit";
            break;
        case '6': passengerStatus = "Transit";
            break;
        case '7': passengerStatus = "Standby. Seat number not printed on BP at check-in time";
            break;
        case '8': passengerStatus = "Boarding data revalidation done";
            break;
        case '9': passengerStatus = "Original boarding line used at time of ticket issuance";
            break;                               
        default: passengerStatus = "-"; // Unknown / Reserved for future industry use
            break;
    }    
    return passengerStatus;
}




map<int, string> getDesiredMapId(list<BCBP_Item> items){
    map<int, string> mMap;
    
    for (list<BCBP_Item>::iterator it = items.begin(); it != items.end(); ++it) {
        BCBP_Item item = *it;       
        mMap.insert(pair<int,string> (item.GetId(), item.GetData()));
    }    
    return mMap;
}


map<string, string> getDesiredMapDescr(list<BCBP_Item> items){
    map<string, string> mMap;
    
    for (list<BCBP_Item>::iterator it = items.begin(); it != items.end(); ++it) {
        BCBP_Item item = *it;       
        mMap.insert(pair<string,string> (item.GetDescription(), item.GetData()));
    }    
    return mMap;
}



/* Print the parsed string items result in a table */
void printTable(list<BCBP_Item> itemList) {
    // Print title
    const std::string title = "BOARDING PASS ITEMS";
    cout << '\n';
    for (int i = 0; i < title.length() + 2; ++i) cout << '-';
    cout << '\n';
    cout << '|' << title << "|\n";
    for (int i = 0; i < title.length() + 2; ++i) cout << '-';
    cout << '\n';

    // Print header row
    for (int i = 0; i < BCBP_Item::fieldNames.size(); ++i) {
        cout << setw(BCBP_Item::fieldWidths[i]) << BCBP_Item::fieldNames[i];
    }
    cout << '\n';

    // Print separating line
    int padding = 0;
    for (int fw : BCBP_Item::fieldWidths) padding += fw;
    for (int i = 0; i < padding; ++i) cout << '-';
    cout << '\n';

    // Print data
    for (list<BCBP_Item>::const_iterator it = itemList.begin(); it != itemList.end(); ++it) {
        (*it).print();
    }
}







/* Print a map of items (key=itemId, value=itemContent) */
void printMap(map<int, string> mMap){
    for (map<int, string>::iterator it = mMap.begin(); it != mMap.end(); ++it) {
        int itemId = (*it).first;
        string itemDesc = BCBP_Item(itemId).GetDescription();
        cout << setw(25) << itemDesc << setw(55) << (*it).second << '\n';
    }     
}


/* Print a map of items (key=itemDescription, value=itemContent) */
void printMap(map<string, string> mMap){
    for (map<string, string>::iterator it = mMap.begin(); it!=mMap.end(); ++it) {
        cout << setw(25) << (*it).first << setw(55) << (*it).second << '\n';
    }     
}


}