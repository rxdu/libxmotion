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
#include "v_repLib.h"
#include <string>

enum {  STACK_NULL=0,
        STACK_NUMBER,
        STACK_BOOL,
        STACK_STRING,
        STACK_ARRAY,
        STACK_MAP
};

class CStackObject
{
public:
    CStackObject();
    virtual ~CStackObject();

    virtual CStackObject* copyYourself();

    int getObjectType() const;

    static void buildItemOntoStack(int stackId,CStackObject* obj);
    static CStackObject* buildItemFromTopStackPosition(int stackId);

protected:
    int _objectType;
};
