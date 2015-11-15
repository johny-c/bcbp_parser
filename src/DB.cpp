#include <stdlib.h>
#include <iostream>
#include "DB.h"
 
using namespace std;




bool DB::instanceFlag = false;
DB* DB::singleton = nullptr;

DB* DB::getInstance() {
    if(! instanceFlag) {
        singleton = new DB();
        instanceFlag = true;
        //cout << "Created DB Singleton!\n\n";
    }

    //cout << "DB Singleton exists already!\n\n";
    return singleton;
}



DB::DB(){

	try{
		driver = get_driver_instance();
                //cout << "Got a mysql cppconn driver instance\n";
		con = driver->connect(DB_HOST, DB_USER, DB_PASS);
		//cout << "Got a connection to MySQL\n";
		con->setSchema(DB_NAME);
		cout << "Connected to db " << DB_NAME << '\n';
	}
	catch (sql::SQLException &e) {
		cout << "# ERR: SQLException in " << __FILE__;
		cout << "(" << __FUNCTION__ << ") on line " << __LINE__ << endl;
		cout << "# ERR: " << e.what();
		cout << " (MySQL error code: " << e.getErrorCode();
		cout << ", SQLState: " << e.getSQLState() << " )" << endl;
	}
        
        res = nullptr;
        stmt = nullptr;      
}


DB::~DB(){
        instanceFlag = false;
        delete res;
	delete stmt;
	delete con;
}



/* Query gate based on flightCarrier and flightNumber */
string DB::queryGate(const string flightCarrier, const string flightNumber) {

	try {				
		
		stmt = con->createStatement();
                
                const string query = STD_QUERY_STR 
                        + FLIGHT_CARRIER_FIELD_NAME + " = '" + flightCarrier + "'" 
                        + " AND "
                        + FLIGHT_NUMBER_FIELD_NAME  + " = " + flightNumber + "";
		
                cout << "Running MySQL query:\n" + query + '\n';
                
		res = stmt->executeQuery( query );
		
		while (res->next()) {
		  //cout << "\t... MySQL replies: ";
		  /* Access column data by alias or column name */
		  const string str = res->getString(FLIGHT_GATE_FIELD_NAME);
		  //cout << str << '\n';
		  return str;
		  /* Access column fata by numeric offset, 1 is the first column */
		  //cout << res->getString(1) << endl;
		}

	} catch (sql::SQLException &e) {
		cout << "# ERR: SQLException in " << __FILE__;
		cout << "(" << __FUNCTION__ << ") on line " << __LINE__ << endl;
		cout << "# ERR: " << e.what();
		cout << " (MySQL error code: " << e.getErrorCode();
		cout << ", SQLState: " << e.getSQLState() << " )" << endl;
	}


	return "";
}


/* Query gate based on flightCode (flightCarrier + flightNumber) */ 
string DB::queryGate(const string flightCode) {

	try {				
		
		stmt = con->createStatement();
		
		const string query = STD_QUERY_STR 
                                            + FLIGHT_CODE_FIELD_NAME 
                                            + " = '" + flightCode + "'";
		
		res = stmt->executeQuery( query );
		
		while (res->next()) {
		  cout << "\t... MySQL replies: ";
		  /* Access column data by alias or column name */
		  const string str = res->getString(FLIGHT_GATE_FIELD_NAME);
		  cout << str << '\n';
		  return str;
		  /* Access column fata by numeric offset, 1 is the first column */
		  //cout << res->getString(1) << endl;
		}

	} catch (sql::SQLException &e) {
		cout << "# ERR: SQLException in " << __FILE__;
		cout << "(" << __FUNCTION__ << ") on line " << __LINE__ << endl;
		cout << "# ERR: " << e.what();
		cout << " (MySQL error code: " << e.getErrorCode();
		cout << ", SQLState: " << e.getSQLState() << " )" << endl;
	}


	return "";
}