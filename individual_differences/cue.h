//
//  cue.h
//  individual_differences
//
//  Created by Vivek Hari Sridhar on 10/08/16.
//  Copyright © 2016 Vivek Hari Sridhar. All rights reserved.
//

#ifndef cue_h
#define cue_h
#include "vector2D.h"
#include "random.h"

const int number_of_cues = 3;
const int number_of_locations = 6;

//**************************************************************************************************
//**	CLASS DEFINITION	************************************************************************
//**************************************************************************************************

class cue
{
public:
    cue(void);
    ~cue(void);
    
    void Setup(const CVec2D& set_centre, const int& timer, const int& set_reward_rad, const bool& set_status, const double& set_reliability, const bool& set_is_reward);
    
    CVec2D centre;              // position for centre of cue
    int timer;                  // time the cue has been active (reset to 0 if cue is not active)
    bool status;                // present / absent status
    double reward_radius;       // radius within which agents are rewarded
    double reliability;         // probability of reward from the cue
    
    bool is_reward;             // boolean outcome of reliability
};

#endif /* cue_h */
