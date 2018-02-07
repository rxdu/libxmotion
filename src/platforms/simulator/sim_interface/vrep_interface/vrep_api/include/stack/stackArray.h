// Copyright 2006-2017 Coppelia Robotics GmbH. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// -------------------------------------------------------------------
// THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
// 
// You are free to use/modify/distribute this file for whatever purpose!
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.4.0 rev. 1 on April 5th 2017

#pragma once

#include <vector>
#include "stackObject.h"

class CStackMap;

class CStackArray : public CStackObject
{
public:
    CStackArray();
    virtual ~CStackArray();

    bool buildFromStack(int stackId);
    void buildOntoStack(int stackId);

    void appendTopStackItem(int stackId);

    CStackObject* copyYourself();

    bool pushNull();
    bool pushBool(bool d);
    bool pushFloat(float d);
    bool pushDouble(double d);
    bool pushInt(int d);
    bool pushString(const std::string& d);
    bool pushArray(CStackArray* arr);
    bool pushMap(CStackMap* map);
    bool setDoubleArray(const double* d,size_t l);
    bool setIntArray(const int* d,size_t l);

    bool isNumberArray();
    size_t getSize();

    bool isNull(size_t index);
    bool isBool(size_t index);
    bool isNumber(size_t index);
    bool isString(size_t index);
    bool isArray(size_t index,size_t minSize=0);
    bool isMap(size_t index);

    bool getBool(size_t index);
    float getFloat(size_t index);
    double getDouble(size_t index);
    int getInt(size_t index);
    std::string getString(size_t index);
    CStackArray* getArray(size_t index);
    CStackMap* getMap(size_t index);

    const std::vector<CStackObject*>* getObjects();
    const std::vector<double>* getDoubles();
    const std::vector<int>* getInts();
    const double* getDoublePointer();
    const int* getIntPointer();

protected:
    std::vector<CStackObject*> _objectValues;
    std::vector<double> _doubleValues;
    std::vector<int> _intValues;
};
