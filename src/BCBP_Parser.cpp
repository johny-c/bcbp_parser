#include <iostream>
#include <list>
#include <string>
#include "BCBP_Utils.h"
#include "BCBP_Item.h"
#include "BCBP_Parser.h"
#include "boost/date_time/gregorian/gregorian.hpp"

//#define _DEBUG_MODE_ 
//
//#ifdef _DEBUG_MODE_
//#define DEBUG(x) (cerr << x << endl)
//#else
//#define DEBUG(x)
//#endif

using namespace std;
using namespace bcbp;
using namespace bcbp_utils;


bool BCBP_Parser::instanceFlag = false;
BCBP_Parser* BCBP_Parser::singleton = nullptr;

BCBP_Parser* BCBP_Parser::getInstance() {
    if(! instanceFlag) {
        singleton = new BCBP_Parser();
        instanceFlag = true;
    }
    
    return singleton;
}


BCBP_Parser::BCBP_Parser() {
    reset();
}


BCBP_Parser::~BCBP_Parser(){
    instanceFlag = false;
}



void BCBP_Parser::reset() {
    curLeg = 0;
    curPos = 0;
    curConditionalSectionSize = 0;
    numberOfLegs = 1;
    barcodeString = "";
    itemList.clear();
}

/* Parse a single item */
int BCBP_Parser::parseItem(BCBP_Item& item) {
    int fieldSize = item.GetFieldSize();
    string data = barcodeString.substr(curPos, fieldSize);
    item.SetData(data);
    itemList.push_back(item);
    curPos += fieldSize;
    //item.print();
    return fieldSize;
}

/* Parse a structured message 
 * iterator it indicates initial position
 * the first 2 fields indicate the message size
 */
int BCBP_Parser::parseStructuredMessage(list<int>::const_iterator& it) {
    // Parse header to get size of message
    BCBP_Item item(*it);
    int headerLength = parseItem(item);
    int curMessageSize = stoi(item.GetData(), nullptr, 16);
    // cout << "PARSING STRUCTURED MESSAGE OF SIZE: " << curMessageSize << '\n';

    // Parse main message
    int curMessagePos = 0;
    while (curMessagePos < curMessageSize) {
        ++it;
        BCBP_Item itemu(*it);
        curMessagePos += parseItem(itemu);
    }

    // cout << "PARSED STRUCTURED MESSAGE OF SIZE: " << curMessagePos << '\n';
    // return message length
    return curMessagePos + headerLength;
}

/**
 * Parse a section with conditional items
 * @return size of section 
 */
int BCBP_Parser::parseConditionalSection() {

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
            // cout << "END OF CONDITIONAL SECTION\n\n";
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
int BCBP_Parser::parseSection(BCBP_SectionType type) {

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
            // cout << "NUMBER OF LEGS:" << numberOfLegs << endl;
        }

        if (item.GetId() == FIELD_SIZE_OF_FOLLOWING_VARIABLE_SIZE_FIELD_ID) {
            curConditionalSectionSize = stoi(item.GetData(), nullptr, 16);
        }
    }

}

/**
 * Parse a provided barcode string 
 * @param str
 * @return a container of BCBP_Items
 */
list<BCBP_Item> BCBP_Parser::parse(const string& str) {

    reset();
    barcodeString = str;
    // cout << barcodeString << "\n\n";

   // cout << "PARSING MAIN\n";
    while (curLeg < numberOfLegs) {
        ++curLeg;
        // cout << "CURRENT LEG: " << curLeg << endl;
        // cout << "PARSING MANDATORY\n";
        parseSection(BCBP_SectionType::MANDATORY);
        //printTable(itemList);

        // cout << "PARSING CONDITIONAL OF SIZE " << curConditionalSectionSize << '\n';
        parseSection(BCBP_SectionType::CONDITIONAL);
        //printTable(itemList);
    }

    //printTable(itemList);
    if (curPos < barcodeString.length()) {
        // cout << "PARSING SECURITY\n";
        parseSection(BCBP_SectionType::SECURITY);
    } else {
        // cout << "SECURITY SECTION EMPTY\n";
    }

    //printTable(itemList);
    return itemList;
}
