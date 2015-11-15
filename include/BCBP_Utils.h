/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   BCBP_Utils.h
 * Author: johny
 *
 * Created on November 15, 2015, 4:05 AM
 */

#ifndef BCBP_UTILS_H
#define BCBP_UTILS_H

#include <map>
#include <string>
#include <vector>
#include <list>
#include "BCBP_Item.h"

using std::string;
using std::vector;
using std::list;
using std::map;

namespace bcbp_utils{

/* Get interpretation of Passenger Description field */
string interpretPassengerDescription(char c);
/* Convert day Of the year to Gregorian Date */
string convertJulianToGregorianDate(int dayOfYear);
/* Remove delimiters */
string extractPassengerName(string passengerName);
/* Extract flight class */
string extractCompartmentCode(char c);
/* Extract current passenger status */
string extractPassengerStatus(char c);

list<BCBP_Item> extractDesiredItems(list<BCBP_Item> items);

map<int, string> listToMap(const list<BCBP_Item>& items);

list<BCBP_Item>::const_iterator findItemById(const list<BCBP_Item>& itemList, int itemId);

/* Print BCBP items as a table */
void printTable(const list<BCBP_Item>& itemList);

}

#endif /* BCBP_UTILS_H */

