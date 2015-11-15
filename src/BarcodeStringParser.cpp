#include <iostream>
#include <iomanip>
#include <list>
#include <string>
#include <map>
#include "BCBP_Item.h"
#include "BarcodeStringParser.h"
#include "boost/date_time/gregorian/gregorian.hpp"

using namespace std;
using namespace bcbp;


bool BarcodeStringParser::instanceFlag = false;
BarcodeStringParser* BarcodeStringParser::singleton = nullptr;

BarcodeStringParser* BarcodeStringParser::getInstance() {
    if(! instanceFlag) {
        singleton = new BarcodeStringParser();
        instanceFlag = true;
    }
    
    return singleton;
}


BarcodeStringParser::BarcodeStringParser() {
    reset();
}


BarcodeStringParser::~BarcodeStringParser(){
    instanceFlag = false;
}



void BarcodeStringParser::reset() {
    curLeg = 0;
    curPos = 0;
    curConditionalSectionSize = 0;
    numberOfLegs = 1;
    barcodeString = "";
    itemList.clear();
}



string BarcodeStringParser::getItem(int itemId) {
    for (list<BCBP_Item>::const_iterator it = itemList.begin(); it != itemList.end(); ++it) {
        if (it->GetId() == itemId) {
            return it->GetData();
        }
    }
    return "";
}

int BarcodeStringParser::parseItem(BCBP_Item &item) {
    int fieldSize = item.GetFieldSize();
    string data = barcodeString.substr(curPos, fieldSize);
    item.SetData(data);
    itemList.push_back(item);
    curPos += fieldSize;
    //item.print();
    return fieldSize;
}

int BarcodeStringParser::parseStructuredMessage(list<int>::const_iterator &it) {
    // Parse header to get size of message
    BCBP_Item item(*it);
    int headerLength = parseItem(item);
    int curMessageSize = stoi(item.GetData(), nullptr, 16);
    cout << "PARSING STRUCTURED MESSAGE OF SIZE: " << curMessageSize << '\n';

    // Parse main message
    int curMessagePos = 0;
    while (curMessagePos < curMessageSize) {
        ++it;
        BCBP_Item itemu(*it);
        curMessagePos += parseItem(itemu);
    }

    cout << "PARSED STRUCTURED MESSAGE OF SIZE: " << curMessagePos << '\n';
    // return message length
    return curMessagePos + headerLength;
}

int BarcodeStringParser::parseConditionalSection() {

    int curSectionPos = 0;
    list<int>::const_iterator it = ITEM_ID_LIST_CONDITIONAL.begin();
    while (curSectionPos < curConditionalSectionSize) {
        BCBP_Item item(*it);

        if (curLeg > 1) {
            // Skip unique items in legs > 1 until structured message repeated
            while (item.IsUnique()) {
                ++it;
                item = BCBP_Item(*it);
            }
        } else {
            // Parse items until structured message unique
            while (item.GetId() != FIELD_SIZE_OF_FOLLOWING_STRUCTURED_MESSAGE_UNIQUE_ID) {
                curSectionPos += parseItem(item);
                if (curSectionPos >= curConditionalSectionSize)
                    return curSectionPos;
                ++it;
                item = BCBP_Item(*it);
            }
            // Parse structured message unique
            curSectionPos += parseStructuredMessage(it);
            if (curSectionPos >= curConditionalSectionSize)
                return curSectionPos;
            // Now skip items until messageRepeated
            while (item.GetId() != FIELD_SIZE_OF_FOLLOWING_STRUCTURED_MESSAGE_REPEATED_ID) {
                ++it;
                item = BCBP_Item(*it);
            }
        }

        // Parse structured message repeated
        curSectionPos += parseStructuredMessage(it);
        if (curSectionPos >= curConditionalSectionSize) {
            cout << "END OF CONDITIONAL SECTION\n\n";
            return curSectionPos;
        }
        // Now skip fields until individual airline use
        while (item.GetId() != FOR_INDIVIDUAL_AIRLINE_USE_ID) {
            ++it;
            item = BCBP_Item(*it);
        }

        int varfs = curConditionalSectionSize - curSectionPos;
        item.SetFieldSize(varfs);
        curSectionPos += parseItem(item);
    }
    return curSectionPos;
}


/* Parse a section of type (MANDATORY, CONDITIONAL or SECURITY */
int BarcodeStringParser::parseSection(BCBP_SectionType type) {

    list<int> itemIDs;

    switch (type) {
        case BCBP_SectionType::MANDATORY:
            itemIDs = ITEM_ID_LIST_MANDATORY;
            break;

        case BCBP_SectionType::CONDITIONAL:
            itemIDs = ITEM_ID_LIST_CONDITIONAL;
            return parseConditionalSection();

        case BCBP_SectionType::SECURITY:
            itemIDs = ITEM_ID_LIST_SECURITY;
            break;

        default:
            cerr << "Unknown section type!\n";
            exit(1);
    }


    for (list<int>::const_iterator it = itemIDs.begin(); it != itemIDs.end(); ++it) {
        BCBP_Item item(*it);
        if (curLeg > 1 && item.IsUnique() && type == BCBP_SectionType::MANDATORY) {
            continue;
        }

        //cout<< "Before parsing:\n"; item.print();
        parseItem(item);
        //cout << "After parsing:\n"; item.print();
        if (item.GetId() == NUMBER_OF_LEGS_ENCODED_ID) {
            numberOfLegs = atoi(item.GetData().c_str());
            cout << "NUMBER OF LEGS:" << numberOfLegs << endl;
            //bp.setNumberOfLegs(numberOfLegs);
            //numberOfLegs = numberOfLegs;
        }

        if (item.GetId() == FIELD_SIZE_OF_FOLLOWING_VARIABLE_SIZE_FIELD_ID) {
            curConditionalSectionSize = stoi(item.GetData(), nullptr, 16);
        }
    }

}

/* Parse a provided barcode string  */
list<BCBP_Item> BarcodeStringParser::parse(const string str) {

    reset();
    barcodeString = str;
    cout << barcodeString << "\n\n";

    cout << "PARSING MAIN\n";
    while (curLeg < numberOfLegs) {
        ++curLeg;
        cout << "CURRENT LEG: " << curLeg << endl;
        cout << "PARSING MANDATORY\n";
        parseSection(BCBP_SectionType::MANDATORY);
        //printTable();

        cout << "PARSING CONDITIONAL OF SIZE " << curConditionalSectionSize << '\n';
        parseSection(BCBP_SectionType::CONDITIONAL);
        //printTable();
    }

    //printTable();
    if (curPos < barcodeString.length()) {
        cout << "PARSING SECURITY\n";
        parseSection(BCBP_SectionType::SECURITY);
    } else {
        cout << "SECURITY SECTION EMPTY\n";
    }

    printTable();
    return itemList;
}


/* Print the parsed string items result in a table */
void BarcodeStringParser::printTable() {
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


///* Get desired fields out of parsed barcode string */
//map<string, string> BarcodeStringParser::getDesiredMap() {
//
//    map<string, string> mMap;
//
//    // Get first and last name of passenger
//    string passengerName = getItem(PASSENGER_NAME_ID);
//    const string delimiter = "/";
//    int splitPos = passengerName.find(delimiter);
//    //        const string lastName = passengerName.substr(0, splitPos);
//    //        const string firstName = passengerName.substr(splitPos+1, passengerName.length());
//    //        mMap.insert ( pair<int,string>(PASSENGER_NAME_ID, passengerName) );
//
//
//    passengerName.replace(splitPos, delimiter.length(), " ");
//    mMap.insert(pair<string, string>(BCBP_Item(PASSENGER_NAME_ID).GetDescription(), passengerName));
//
//
//    // Get ticket number
//    const string ticketNumber = getItem(OPERATING_CARRIER_PNR_CODE_ID);
//    //mMap.insert ( pair<int,string>(OPERATING_CARRIER_PNR_CODE_ID, ticketNumber) );
//    mMap.insert(pair<string, string>(BCBP_Item(OPERATING_CARRIER_PNR_CODE_ID).GetDescription(), ticketNumber));
//
//
//    // For passenger description (gender) , unique but conditional
//    string passengerDescr = getItem(PASSENGER_DESCRIPTION_ID);
//    if (passengerDescr.length() > 0) { // If it is included   
//        switch (passengerDescr[0]) {
//            case '0': passengerDescr = "Adult";
//                break;
//            case '1': passengerDescr = "Male";
//                break;
//            case '2': passengerDescr = "Female";
//                break;
//            case '3': passengerDescr = "Child";
//                break;
//            case '4': passengerDescr = "Infant";
//                break;
//            case '5': passengerDescr = "No passenger(cabin baggage)";
//                break;
//            case '6': passengerDescr = "Adult travelling with infant";
//                break;
//            case '7': passengerDescr = "Unaccompanied minor";
//                break;
//            default: passengerDescr = "-";
//                break;
//        }
//    }
//    //mMap.insert ( pair<int,string>(PASSENGER_DESCRIPTION_ID, passengerDescr) );
//    mMap.insert(pair<string, string>(BCBP_Item(PASSENGER_DESCRIPTION_ID).GetDescription(), passengerDescr));
//
//    // Skip items until airportDep = AMS      
//    list<BCBP_Item>::const_iterator it = itemList.begin();
//    BCBP_Item item = *it;
//    bool noAmsterdamDeparture = false;
//    while (item.GetId() != FROM_CITY_AIRPORT_CODE_ID || item.GetData() != "AMS") {
//        ++it;
//        item = *it;
//
//        if (it == itemList.end()) {
//            noAmsterdamDeparture = true;
//            break;
//        }
//    }
//
//    if (noAmsterdamDeparture) {
//        printMap();
//        return mMap;
//    }
//
//    // airportDep, airportArr, flightCarrier, flightNumber, flightDate
//    for (int i = 0; i < 5; ++i) {
//        BCBP_Item item = *it;
//        //mMap.insert ( pair<int,string>(item.GetId(), item.GetData()) );
//
//        if (item.GetId() == DATE_OF_FLIGHT_JULIAN_DATE_ID) {
//            int dayOfYear = stoi(item.GetData());
//            using namespace boost::gregorian;
//            int curYear = day_clock::local_day().year();
//            //int curYear = second_clock::local_time.date().year();
//            date d(curYear, Jan, 1);
//            date_duration dd(dayOfYear - 1);
//            date flightDate = d + dd;
//
//            const string dateOfFlight = to_simple_string(flightDate);
//            mMap.insert(pair<string, string>("Date of Flight", dateOfFlight));
//        } else {
//            mMap.insert(pair<string, string>(item.GetDescription(), item.GetData()));
//        }
//
//        ++it;
//    }
//
//    printMap();
//    return mMap;
//}
//
//
///* Print a map of items (key=itemId, value=itemContent) */
//void BarcodeStringParser::printMap() {
//    std::cout << "\n\nmMap contains:\n\n";
//    for (map<int, string>::iterator it = mMap.begin(); it != mMap.end(); ++it) {
//        int itemId = (*it).first;
//        string itemDesc = BCBP_Item(itemId).GetDescription();
//        cout << setw(25) << itemDesc << setw(55) << (*it).second << '\n';
//    }
//}





