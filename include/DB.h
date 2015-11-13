/* 
 * File:   DB.h
 * Author: johny
 *
 * Created on October 20, 2015, 3:35 PM
 */

#ifndef DB_H
#define	DB_H

#include <cppconn/driver.h>
#include <cppconn/exception.h>
#include <cppconn/resultset.h>
#include <cppconn/statement.h>
#include <string>
 
using std::string;

static const string DB_HOST = "tcp://127.0.0.1:3306";
static const string DB_USER = "root";
static const string DB_PASS = "root";
static const string DB_NAME = "air_db";
static const string FLIGHT_CARRIER_FIELD_NAME = "f_carrier";
static const string FLIGHT_NUMBER_FIELD_NAME = "f_number";
static const string FLIGHT_CODE_FIELD_NAME = "f_code";
static const string FLIGHT_GATE_FIELD_NAME = "f_gate";
static const string FLIGHTS_TABLE_NAME = "flights_tbl";
static const string STD_QUERY_STR = "SELECT " + FLIGHT_GATE_FIELD_NAME 
                               + " FROM " + FLIGHTS_TABLE_NAME 
                               + " WHERE ";


class DB
{
	public:
		DB ();
		virtual ~DB ();

		string queryGate(string flightCode);
		string queryGate(string flightCarrier, string flightNumber);

	private:
		sql::Driver *driver;
                sql::Connection *con;
                sql::Statement *stmt;
                sql::ResultSet *res;
};

#endif	/* DB_H */
