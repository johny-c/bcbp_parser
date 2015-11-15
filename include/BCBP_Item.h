/*
 * File:   BCBP_Item.h
 * Author: johny
 *
 * Created on October 15, 2015, 10:40 PM
 */



#ifndef BCBP_ITEM_H
#define	BCBP_ITEM_H

#include<string>
#include<array>

using std::string;

enum class BCBP_SectionType {MANDATORY, CONDITIONAL, SECURITY, NONE};


class BCBP_Item {


public:
   BCBP_Item();

   BCBP_Item(int no);

   BCBP_Item(int no, unsigned int order, string desc, unsigned int fs, int u);

   void defineItem(int no, unsigned int order, string desc, unsigned int fs, int u, BCBP_SectionType st);


    const void print() const;

    string GetData() const;

    void SetData(string data) ;

    string GetDescription() const;

    void SetDescription(string description) ;

    unsigned int GetFieldSize() const;

    void SetFieldSize(unsigned int fieldSize) ;

    int GetId() const ;

    void SetId(int number) ;

    unsigned int GetOrdering() const;

    void SetOrdering(unsigned int ordering) ;

    bool IsUnique() const;

    void SetUnique(bool unique);

    BCBP_SectionType getSectionType() const {
        return sectionType;
    }

    void setSectionType(BCBP_SectionType sectionType) {
        this->sectionType = sectionType;
    }

    static std::array<int, 7> fieldWidths;
    static std::array<string, 7> fieldNames;

private:
	int id;
        string description;
        string data;
        bool unique;
	unsigned int ordering;	
        BCBP_SectionType sectionType;
	unsigned int fieldSize;        
        
};



#endif	/* BCBP_ITEM_H */

