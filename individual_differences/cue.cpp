//
//  cue.cpp
//  DecisionNetwork
//
//  Created by Vivek Hari Sridhar on 04/03/16.
//  Copyright © 2016 Vivek Hari Sridhar. All rights reserved.
//

#include "cue.h"

cue::cue(void) : status(0), reliability(0.0)
{
}

cue::~cue(void)
{
}

void cue::Setup(const CVec2D& set_centre, const int& set_timer, const int& set_reward_rad, const bool& set_status, const double& set_reliability, const bool& set_is_reward)
{
    centre = set_centre;    timer = set_timer;   reward_radius = set_reward_rad;    status = set_status;    reliability = set_reliability;  is_reward = set_is_reward;
}