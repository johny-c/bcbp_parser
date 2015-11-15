#include <map>
#include <iomanip>
#include "BCBP_Utils.h"
#include "BCBP_Item.h"
#include "BCBP_Parser.h"
#include "boost/date_time/gregorian/gregorian.hpp"
#include "boost/algorithm/string/trim.hpp"

using std::setw;
using std::cout;
using std::map;
using std::pair;

namespace bcbp_utils {

    /* Get interpretation of Passenger Description field (0-7) */
    string interpretPassengerDescription(char c) {
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

    string convertJulianToGregorianDate(int dayOfYear) {

        using namespace boost::gregorian;
        int curYear = day_clock::local_day().year();
        //int curYear = second_clock::local_time.date().year();
        date d(curYear, Jan, 1);
        date_duration dd(dayOfYear - 1);
        date gregorian = d + dd;

        return to_simple_string(gregorian);
    }

    string extractPassengerName(string passengerName) {
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
    string extractCompartmentCode(char c) {
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
    string extractPassengerStatus(char c) {
        string passengerStatus;
        switch (c) {
            case '0': passengerStatus = "Ticket issuance/passenger not checked in";
                break;
            case '1': passengerStatus = "Ticket issuance/passenger checked in";
                break;
            case '2': passengerStatus = "Baggage checked/passenger not checked in";
                break;
            case '3': passengerStatus = "Baggage checked/passenger checked in";
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

    /* Get map (id => data) of desired items */
    map<int, string> listToMap(const list<BCBP_Item>& items) {
        map<int, string> mMap;

        list<BCBP_Item>::const_iterator it;
        for (it = items.begin(); it != items.end(); ++it) {
            BCBP_Item item = *it;
            mMap.insert(pair<int, string> (item.GetId(), item.GetData()));
        }
        return mMap;
    }

    /* Print a map of items (key=itemId, value=itemContent) */
    void printMap(const map<int, string>& mMap) {
        map<int, string>::const_iterator it;
        for (it = mMap.begin(); it != mMap.end(); ++it) {
            int itemId = (*it).first;
            string itemDesc = BCBP_Item(itemId).GetDescription();
            cout << setw(25) << itemDesc << setw(55) << (*it).second << '\n';
        }
    }

    /* Print table of the items from the parsed string */
    void printTable(const list<BCBP_Item>& itemList) {
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

    /* Return the item with the matching id */
    list<BCBP_Item>::const_iterator findItemById(const list<BCBP_Item>& itemList, int itemId) {
        //BCBP_Item res;
        list<BCBP_Item>::const_iterator it;
        for (it = itemList.begin(); it != itemList.end(); ++it) {
            if (it->GetId() == itemId) {            
                return it;
            }
        }

        return itemList.end();
    }
   

    //const vector<int> desiredItemIDs = 
    //{
    //    PASSENGER_NAME_ID ,
    //    OPERATING_CARRIER_PNR_CODE_ID ,
    //    FROM_CITY_AIRPORT_CODE_ID ,
    //    TO_CITY_AIRPORT_CODE_ID,
    //    OPERATING_CARRIER_DESIGNATOR_ID ,
    //    FLIGHT_NUMBER_ID ,
    //    DATE_OF_FLIGHT_JULIAN_DATE_ID ,
    //    COMPARTMENT_CODE_ID ,
    //    SEAT_NUMBER_ID ,
    //    CHECKIN_SEQUENCE_NUMBER_ID ,
    //    PASSENGER_STATUS_ID ,
    //    PASSENGER_DESCRIPTION_ID
    //};

    /* Get desired fields out of parsed barcode string */
    list<BCBP_Item> extractDesiredItems(list<BCBP_Item> items) {
        using namespace bcbp;
        list<BCBP_Item> desiredItems;
        
        /* First get the mandatory unique items */
        // Get first and last name of passenger
        list<BCBP_Item>::const_iterator it1= findItemById(items, PASSENGER_NAME_ID);
        BCBP_Item passengerNameItem = *it1;
        string pName = extractPassengerName(passengerNameItem.GetData());
        passengerNameItem.SetData(pName);
        desiredItems.push_back(passengerNameItem);


        /* Second get conditional unique items */
        // For passenger description (gender)       
        list<BCBP_Item>::const_iterator it2 = findItemById(items, PASSENGER_DESCRIPTION_ID);
        if (it2 != items.end()){
            cout << "Found passenger description by id\n";
            BCBP_Item passengerDescriptionItem = *it2;
            string passengerDescr = passengerDescriptionItem.GetData();
            if (passengerDescr.length() > 0) { // If it is included   
                passengerDescr = interpretPassengerDescription(passengerDescr.at(0));
            }
            passengerDescriptionItem.SetData(passengerDescr);
            desiredItems.push_back(passengerDescriptionItem);
        }
        else{
            cout << "Did not Found passenger description by id\n";
        }

        
        /* Third get the mandatory but repeated fields 
         * Get the ones we are interested in, a.k.a. the flight leg where 
         * airport of departure is AMS (Amsterdam)
         */
        // Skip items until airportDep = AMS     
        bool amsterdamDeparture = false;
        const string desiredDepartureAirport = "AMS";
        list<BCBP_Item>::const_iterator it;
        for (it=items.begin(); it!=items.end(); ++it){     
            BCBP_Item item = *it;
            if(item.GetId() == FROM_CITY_AIRPORT_CODE_ID 
                    && item.GetData() == desiredDepartureAirport){
                amsterdamDeparture = true;
                break;
            }
        }

        if (!amsterdamDeparture) {
            cout << "AMS DEPARTURE NOT FOUND\n\n";
            return desiredItems;
        }

        // Flight leg found
        // Go one item back, to OPERATING_CARRIER_PNR_CODE_ID
        --it;
        cout << "AMS DEPARTURE FOUND\n\n";
        
        BCBP_Item tm = *it;
        cout << "Current item in iterator: " << tm.GetDescription() << '\n';

        //    OPERATING_CARRIER_PNR_CODE_ID ,
        //    FROM_CITY_AIRPORT_CODE_ID ,
        //    TO_CITY_AIRPORT_CODE_ID,
        //    OPERATING_CARRIER_DESIGNATOR_ID ,
        //    FLIGHT_NUMBER_ID ,
        //    DATE_OF_FLIGHT_JULIAN_DATE_ID ,
        //    COMPARTMENT_CODE_ID ,
        //    SEAT_NUMBER_ID ,
        //    CHECKIN_SEQUENCE_NUMBER_ID ,
        //    PASSENGER_STATUS_ID ,    


        // ticketNumber, airportDep, airportArr, flightCarrier, flightNumber
        for (int i = 0; i < 5; ++i) {
            BCBP_Item item = *it;
            desiredItems.push_back(item);
            ++it;
        }

        // flightDate, compartmentCode, seatNumber, checkinSequence, passengerStatus
        for (int i = 0; i < 5; ++i) {
            BCBP_Item item = *it;

            switch (item.GetId()) {
                case DATE_OF_FLIGHT_JULIAN_DATE_ID:
                {
                    int dayOfYear = std::stoi(item.GetData());
                    const string dateOfFlight = convertJulianToGregorianDate(dayOfYear);
                    item.SetDescription("Date of Flight");
                    item.SetData(dateOfFlight);
                    desiredItems.push_back(item);
                }
                    break;

                case COMPARTMENT_CODE_ID:
                {
                    const string seatClass = extractCompartmentCode(item.GetData().at(0));
                    item.SetData(seatClass);
                    desiredItems.push_back(item);
                }
                    break;

                case SEAT_NUMBER_ID:
                {
                    desiredItems.push_back(item);
                }
                    break;

                case PASSENGER_STATUS_ID:
                {
                    const string passengerStatus = extractPassengerStatus(item.GetData().at(0));
                    item.SetData(passengerStatus);
                    desiredItems.push_back(item);
                }
                    break;

                default: break;
            }

            ++it;
        }

        return desiredItems;
    }







}