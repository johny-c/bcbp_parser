#include <iostream>
#include <list>
#include <string>
#include <map>
#include "BCBP_Utils.h"
#include "BCBP_Item.h"
#include "BCBP_Parser.h"
#include "boost/date_time/gregorian/gregorian.hpp"

using namespace std;
using namespace bcbp;
using namespace bcbp_utils;


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



BCBP_Item BarcodeStringParser::getItem(int itemId) {
    BCBP_Item res;
    for (list<BCBP_Item>::const_iterator it = itemList.begin(); it != itemList.end(); ++it) {
        if (it->GetId() == itemId) {
            res = *it; 
            return res;
        }
    }
    return res;
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
    //cout << "PARSING STRUCTURED MESSAGE OF SIZE: " << curMessageSize << '\n';

    // Parse main message
    int curMessagePos = 0;
    while (curMessagePos < curMessageSize) {
        ++it;
        BCBP_Item itemu(*it);
        curMessagePos += parseItem(itemu);
    }

    //cout << "PARSED STRUCTURED MESSAGE OF SIZE: " << curMessagePos << '\n';
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
            //cout << "END OF CONDITIONAL SECTION\n\n";
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
            //cout << "NUMBER OF LEGS:" << numberOfLegs << endl;
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
    //cout << barcodeString << "\n\n";

   // cout << "PARSING MAIN\n";
    while (curLeg < numberOfLegs) {
        ++curLeg;
        //cout << "CURRENT LEG: " << curLeg << endl;
        //cout << "PARSING MANDATORY\n";
        parseSection(BCBP_SectionType::MANDATORY);
        //printTable();

        //cout << "PARSING CONDITIONAL OF SIZE " << curConditionalSectionSize << '\n';
        parseSection(BCBP_SectionType::CONDITIONAL);
        //printTable();
    }

    //printTable();
    if (curPos < barcodeString.length()) {
        //cout << "PARSING SECURITY\n";
        parseSection(BCBP_SectionType::SECURITY);
    } else {
        //cout << "SECURITY SECTION EMPTY\n";
    }

    //printTable(itemList);
    return itemList;
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
list<BCBP_Item> BarcodeStringParser::extractDesiredItems(list<BCBP_Item> items) {

    list<BCBP_Item> desiredItems;
    
    /* First get the mandatory unique items */
    
    // Get first and last name of passenger
    BCBP_Item passengerNameItem = getItem(PASSENGER_NAME_ID);
    string pName = extractPassengerName(passengerNameItem.GetData());
    passengerNameItem.SetData(pName);
    desiredItems.push_back(passengerNameItem);
    
    
    /* Second get conditional unique items */
    
    // For passenger description (gender) 
    BCBP_Item passengerDescriptionItem = getItem(PASSENGER_DESCRIPTION_ID);
    string passengerDescr = passengerDescriptionItem.GetData();
    if (passengerDescr.length() > 0) { // If it is included   
        passengerDescr = interpretPassengerDescription(passengerDescr[0]);
    } 
    passengerDescriptionItem.SetData(passengerDescr);
    desiredItems.push_back(passengerDescriptionItem);
    
   
    /* Third get the mandatory but repeated fields 
     * Get the ones we are interested in, a.k.a. the flight leg where 
     * airport of departure is AMS (Amsterdam)
     */

    // Skip items until airportDep = AMS      
    list<BCBP_Item>::const_iterator it = itemList.begin();
    BCBP_Item item = *it;
    bool noAmsterdamDeparture = false;
    while (item.GetId() != FROM_CITY_AIRPORT_CODE_ID || item.GetData() != "AMS") {
        ++it;
        item = *it;

        if (it == itemList.end()) {
            noAmsterdamDeparture = true;
            break;
        }
    }

    if (noAmsterdamDeparture) {
        return desiredItems;
    }
    
    // Flight leg found
    // Go one item back, to OPERATING_CARRIER_PNR_CODE_ID
    --it;

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
        
        switch(item.GetId()){
            case DATE_OF_FLIGHT_JULIAN_DATE_ID:{
                int dayOfYear = stoi(item.GetData());
                const string dateOfFlight = convertJulianToGregorianDate(dayOfYear);
                item.SetData(dateOfFlight);
                desiredItems.push_back(item);}
                break;
                
            case COMPARTMENT_CODE_ID:{
                const string seatClass = extractCompartmentCode(item.GetData().at(0));
                item.SetData(seatClass);
                desiredItems.push_back(item);}
                break;    
                
            case SEAT_NUMBER_ID:{
                desiredItems.push_back(item);}
                break;
                
            case PASSENGER_STATUS_ID:{
                const string passengerStatus = extractPassengerStatus(item.GetData().at(0));
                item.SetData(passengerStatus);
                desiredItems.push_back(item);}
                break;

            default: break;                
        }

        ++it;
    }

    return desiredItems;
}






