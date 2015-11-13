#include <iomanip>
#include <iostream>
#include <string>
#include <array>
#include "BCBP_Item.h"
#include "BCBP_ItemIDs.h"


using std::cerr;
using std::cout;
using std::setw;
using std::string;


BCBP_Item::BCBP_Item() {}

BCBP_Item::BCBP_Item(int no, unsigned int order, string desc, unsigned int fs, int u)
: id(no), ordering(order), description(desc), fieldSize(fs), unique(u) {}

BCBP_Item::BCBP_Item(int no) : id(no) {
    using namespace bpitemids;
 switch(id) {
     /* BCBP_SectionType::MANDATORY ITEMS */		
     // Unique
     case FORMAT_CODE_ID: defineItem(1, 1, "Format Code", 1, 1, BCBP_SectionType::MANDATORY); break; 
     case NUMBER_OF_LEGS_ENCODED_ID: defineItem (5, 2, "Number of segments/flights", 1, 1, BCBP_SectionType::MANDATORY); break;  // 1,2,3 or 4
     case PASSENGER_NAME_ID: defineItem (11, 3, "Passenger name", 20, 1, BCBP_SectionType::MANDATORY); break;
     case ELECTRONIC_TICKET_INDICATOR_ID: defineItem (253, 4, "E-Ticket indicator", 1, 1, BCBP_SectionType::MANDATORY); break;		
     case OPERATING_CARRIER_PNR_CODE_ID         : defineItem (7, 5, "E-Ticket number (PNR Code)", 7, 0, BCBP_SectionType::MANDATORY);break;
     case FROM_CITY_AIRPORT_CODE_ID         : defineItem (26, 6, "Departure airport", 3, 0, BCBP_SectionType::MANDATORY);break;
     case TO_CITY_AIRPORT_CODE_ID         : defineItem (38, 7, "Arrival airport", 3, 0, BCBP_SectionType::MANDATORY);break;
     case OPERATING_CARRIER_DESIGNATOR_ID         : defineItem (42, 8, "Operating carrier", 3, 0, BCBP_SectionType::MANDATORY);break;
     case FLIGHT_NUMBER_ID         : defineItem (43, 9, "Flight Number", 5, 0, BCBP_SectionType::MANDATORY);break;
     case DATE_OF_FLIGHT_JULIAN_DATE_ID         : defineItem (46, 10, "Julian Date of Flight", 3, 0, BCBP_SectionType::MANDATORY); break;
     case COMPARTMENT_CODE_ID         : defineItem (71, 11, "Compartment Code/Class", 1, 0, BCBP_SectionType::MANDATORY);  break;
     case SEAT_NUMBER_ID         : defineItem (104, 12, "Seat number", 4, 0, BCBP_SectionType::MANDATORY); break;
     case CHECKIN_SEQUENCE_NUMBER_ID         : defineItem (107, 13, "Check-In Sequence number", 5, 0, BCBP_SectionType::MANDATORY);break;
     case PASSENGER_STATUS_ID         : defineItem (113, 14, "Passenger Status", 1, 0, BCBP_SectionType::MANDATORY);break;
     case FIELD_SIZE_OF_FOLLOWING_VARIABLE_SIZE_FIELD_ID         : defineItem (6, 15, "Following variable size segment size (hex)", 2, 0, BCBP_SectionType::MANDATORY);break;
     case BEGINNING_OF_VERSION_NUMBER_ID         : defineItem (8, 1, "Beginning of version number", 1, 1, BCBP_SectionType::CONDITIONAL); break; // '>'break;
     case VERSION_NUMBER_ID         : defineItem (9, 2, "Version number of message", 1, 1, BCBP_SectionType::CONDITIONAL); break;
     case FIELD_SIZE_OF_FOLLOWING_STRUCTURED_MESSAGE_UNIQUE_ID         : defineItem (10, 3, "Following structured message size (hex)", 2, 1, BCBP_SectionType::CONDITIONAL);      break;
     case PASSENGER_DESCRIPTION_ID         : defineItem (15, 4, "Passenger description", 1, 1, BCBP_SectionType::CONDITIONAL);break;
     case SOURCE_OF_CHECKIN_ID         : defineItem (12, 5, "Source of check-in", 1, 1, BCBP_SectionType::CONDITIONAL);break;
     case SOURCE_OF_BOARDING_PASS_ISSUANCE_ID         : defineItem (14, 6, "Source of Boarding Pass Issuance", 1, 1, BCBP_SectionType::CONDITIONAL);break;
     case DATE_OF_ISSUE_OF_BOARDING_PASS_JULIAN_DATE_ID         : defineItem (22, 7, "Julian Date of BP Issuance", 4, 1, BCBP_SectionType::CONDITIONAL);break;
     case DOCUMENT_TYPE_ID         : defineItem (16, 8, "Document Type", 1, 1, BCBP_SectionType::CONDITIONAL);break;
     case AIRLINE_DESIGNATOR_OF_BOARDING_PASS_ISSUER_ID         : defineItem (21, 9, "Airline Designator of BP Issuance", 3, 1, BCBP_SectionType::CONDITIONAL);break;
     case BAGGAGE_TAG_LICENCE_PLATE_NUMBER_S_ID         : defineItem (23, 10, "Baggage Tag License Plate Number", 13, 1, BCBP_SectionType::CONDITIONAL);break;
     case FIELD_SIZE_OF_FOLLOWING_STRUCTURED_MESSAGE_REPEATED_ID         : defineItem (17, 11, "Following structured message size (hex)", 2, 0, BCBP_SectionType::CONDITIONAL);break;
     case AIRLINE_NUMERIC_CODE_ID         : defineItem (142, 12, "Airline numeric code", 3, 0, BCBP_SectionType::CONDITIONAL);break;
     case DOCUMENT_FORM_SERIAL_NUMBER_ID         : defineItem (143, 13, "Document Form/Serial Number", 10, 0, BCBP_SectionType::CONDITIONAL);break;
     case SELECTEE_INDICATOR_ID         : defineItem (18, 14, "Selectee indicator", 1, 0, BCBP_SectionType::CONDITIONAL);break;
     case INTERNATIONAL_DOCUMENTATION_VERIFICATION_ID         : defineItem (108, 15, "International Document Verification", 1, 0, BCBP_SectionType::CONDITIONAL);break;
     case MARKETING_CARRIER_DESIGNATOR_ID         : defineItem (19, 16, "Marketing Carrier Designator", 3, 0, BCBP_SectionType::CONDITIONAL);break;
     case FREQUENT_FLYER_AIRLINE_DESIGNATOR_ID         : defineItem (20, 17, "Frequent Flyer Airline Designator", 3, 0, BCBP_SectionType::CONDITIONAL);break;
     case FREQUENT_FLYER_NUMBER_ID         : defineItem (236, 18, "Frequent Flyer Number", 16, 0, BCBP_SectionType::CONDITIONAL);break;
     case ID_AD_INDICATOR_ID         : defineItem (89, 19, "ID/AD Indicator", 1, 0, BCBP_SectionType::CONDITIONAL);break;
     case FREE_BAGGAGE_ALLOWANCE_ID         : defineItem (118, 20, "Free Baggage Allowance", 3, 0, BCBP_SectionType::CONDITIONAL);break;
     case FOR_INDIVIDUAL_AIRLINE_USE_ID         : defineItem (4, 21, "For individual airline use", 0, 0, BCBP_SectionType::CONDITIONAL);break;
     case BEGINNING_OF_SECURITY_DATA_ID         : defineItem (25, 1, "Beginning of Security Data", 1, 1, BCBP_SectionType::SECURITY);break;
     case TYPE_OF_SECURITY_DATA_ID         : defineItem (28, 2, "Type of Security Data", 1, 1, BCBP_SectionType::SECURITY);break;
     case LENGTH_OF_SECURITY_DATA_ID         : defineItem (29, 3, "Length of Security Data (hex)", 2, 1, BCBP_SectionType::SECURITY);break;
     case SECURITY_DATA_ID         : defineItem (30, 4, "Security Data", 100, 1, BCBP_SectionType::SECURITY);break;


      default:
           cerr << "No matching id for BCBP item!\n";
     }

} 


void BCBP_Item::defineItem(int no, unsigned int order, string desc, unsigned int fs, int u, BCBP_SectionType st){
    this->id = no;
    this->description = desc;
    this->fieldSize = fs;
    this->unique = u;
    this->ordering = order;
    this->sectionType = st;
}


std::array<int, 6> BCBP_Item::fieldWidths { 5, 50, 30, 10, 10, 10 };
std::array<string, 6> BCBP_Item::fieldNames { "Id", "Description", "Data", "Unique", "Ordering", "Section"};


const void BCBP_Item::print() const{
    cout << setw(BCBP_Item::fieldWidths[0]) << this->GetId()           
            << setw(BCBP_Item::fieldWidths[1]) << this->GetDescription()
            << setw(BCBP_Item::fieldWidths[2]) << this->GetData()
            << setw(BCBP_Item::fieldWidths[3]) << this->IsUnique()
            << setw(BCBP_Item::fieldWidths[4]) << this->GetOrdering()
            << setw(BCBP_Item::fieldWidths[5])  << static_cast<int> (this->getSectionType())
            << '\n';
}



 string BCBP_Item::GetData() const {
     return data;
 }

 void BCBP_Item::SetData(string data) {
     this->data = data;
 }

 string BCBP_Item::GetDescription() const {
     return description;
 }

 void BCBP_Item::SetDescription(string description) {
     this->description = description;
 }

 unsigned int BCBP_Item::GetFieldSize() const {
     return fieldSize;
 }

 void BCBP_Item::SetFieldSize(unsigned int fieldSize) {
     this->fieldSize = fieldSize;
 }

 int BCBP_Item::GetId() const {
     return id;
 }

 void BCBP_Item::SetId(int number) {
     this->id = number;
 }

 unsigned int BCBP_Item::GetOrdering() const {
     return ordering;
 }

 void BCBP_Item::SetOrdering(unsigned int ordering) {
     this->ordering = ordering;
 }

 bool BCBP_Item::IsUnique() const {
    return unique;
 }

 void BCBP_Item::SetUnique(bool unique) {
     this->unique = unique;
 }

