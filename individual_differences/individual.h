//
//  individual.h
//  individual_differences
//
//  Created by Vivek Hari Sridhar on 10/08/16.
//  Copyright © 2016 Vivek Hari Sridhar. All rights reserved.
//

#ifndef individual_h
#define individual_h
#include <iostream>
#include "cue.h"

//**************************************************************************************************
//**	CLASS DEFINITION	************************************************************************
//**************************************************************************************************

class individual
{
public:
    individual(void);
    ~individual(void);
    
    void Setup(const CVec2D& set_r_centre, const CVec2D& set_direction, const double& set_max_turning_rate,
               const double& set_speed, const double& set_zone_of_deflection, const double& set_zone_of_orientation,
               const double& set_zone_of_perception, const double& set_angular_error_sd, const double& set_omega);
    
    void Move(const double& timestep_inc, const double& arena_size, const double& dev_angle);
    
    CVec2D r_centre;            // rotational centre of the agent
    CVec2D direction;           // vector representing the current direction of the agent
    CVec2D desired_direction;   // agent's desired direction of movement
    double speed;
    double max_turning_rate;    // limits turning within a timestep (degrees per timestep)
    double zone_of_deflection;
    double zone_of_orientation;
    double zone_of_perception;
    double angular_error_sd;    // angular error representing agent's error in movement / sensory integration
    double omega;               // weight on agent's preferred direction of movement
    double nnd;
    int    omega_rank;          // agent's omega rank within the group
    int    speed_rank;          // agent's speed rank within the group
    
    int zop_count;
    int zoo_count;
    int zod_count;
    int equivalence_class;
    CVec2D total_zod;           // sum of social interactions with agents within zod
    CVec2D total_zoo;           // sum of social interactions with agents within zoo
    CVec2D total_zop;           // sum of social interactions with agents within zop
    
    void TurnTowardsVector(CVec2D& vector, double timestep_inc, double dev_angle);  // turn agent in desired_direction
    void MoveMyself(const double& timestep_inc, const double& arena_size);          // move in periodic boundary space
    void AddPersonalPreference();                                                   // add goal orientedness
    
    double FrontBackDistance(CVec2D& centroid, CVec2D& group_direction);
    double DistancePtLine(CVec2D& a, CVec2D& b);
};

#endif /* individual_h */
