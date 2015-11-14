/*
 * File:   BCBP_Parser.h
 * Author: johny
 *
 * Created on October 16, 2015, 12:36 PM
 */

#ifndef BCBP_PARSER_H
#define	BCBP_PARSER_H
#include <list>
#include <map>
#include <string>
#include "BCBP_Item.h"
#include "BCBP_ItemIDs.h"

using std::map;
using std::list;
using std::string;
using namespace bcbp;


namespace bcbpp{


int parseSection(BCBP_SectionType);
int parseConditionalSection();
list<BCBP_Item> parse(const string barcodeString);
int parseItem(BCBP_Item&);
int parseStructuredMessage(list<int>::const_iterator&);
void printTable();
string getItem(int);
map<string, string> getDesiredMap();
void printMap();

const list<int> ITEM_ID_LIST_MANDATORY =
{
    FORMAT_CODE_ID ,
    NUMBER_OF_LEGS_ENCODED_ID ,
    PASSENGER_NAME_ID ,
    ELECTRONIC_TICKET_INDICATOR_ID,
    OPERATING_CARRIER_PNR_CODE_ID ,
    FROM_CITY_AIRPORT_CODE_ID ,
    TO_CITY_AIRPORT_CODE_ID,
    OPERATING_CARRIER_DESIGNATOR_ID ,
    FLIGHT_NUMBER_ID ,
    DATE_OF_FLIGHT_JULIAN_DATE_ID ,
    COMPARTMENT_CODE_ID ,
    SEAT_NUMBER_ID ,
    CHECKIN_SEQUENCE_NUMBER_ID ,
    PASSENGER_STATUS_ID ,
    FIELD_SIZE_OF_FOLLOWING_VARIABLE_SIZE_FIELD_ID
};



const list<int> ITEM_ID_LIST_CONDITIONAL =
{
    BEGINNING_OF_VERSION_NUMBER_ID,
    VERSION_NUMBER_ID        ,
    FIELD_SIZE_OF_FOLLOWING_STRUCTURED_MESSAGE_UNIQUE_ID,
    PASSENGER_DESCRIPTION_ID    ,
    SOURCE_OF_CHECKIN_ID ,
    SOURCE_OF_BOARDING_PASS_ISSUANCE_ID     ,
    DATE_OF_ISSUE_OF_BOARDING_PASS_JULIAN_DATE_ID     ,
    DOCUMENT_TYPE_ID        ,
    AIRLINE_DESIGNATOR_OF_BOARDING_PASS_ISSUER_ID         ,
    BAGGAGE_TAG_LICENCE_PLATE_NUMBER_S_ID,
    FIELD_SIZE_OF_FOLLOWING_STRUCTURED_MESSAGE_REPEATED_ID,
    AIRLINE_NUMERIC_CODE_ID ,
    DOCUMENT_FORM_SERIAL_NUMBER_ID ,
    SELECTEE_INDICATOR_ID ,
    INTERNATIONAL_DOCUMENTATION_VERIFICATION_ID ,
    MARKETING_CARRIER_DESIGNATOR_ID ,
    FREQUENT_FLYER_AIRLINE_DESIGNATOR_ID ,
    FREQUENT_FLYER_NUMBER_ID ,
    ID_AD_INDICATOR_ID ,
    FREE_BAGGAGE_ALLOWANCE_ID ,
    FOR_INDIVIDUAL_AIRLINE_USE_ID
};



const list<int> ITEM_ID_LIST_SECURITY =
{
    BEGINNING_OF_SECURITY_DATA_ID ,
    TYPE_OF_SECURITY_DATA_ID ,
    LENGTH_OF_SECURITY_DATA_ID ,
    SECURITY_DATA_ID
};




}

#endif	/* BCBP_PARSER_H */

