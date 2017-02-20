//
//  individual.h
//  DecisionNetwork
//
//  Created by Vivek Hari Sridhar on 14/01/16.
//  Copyright © 2016 Vivek Hari Sridhar. All rights reserved.
//

#ifndef individual_h
#define individual_h
#include <iostream>
#include "cue.h"

class individual
{
public:
    individual(void);
    ~individual(void);
    
    void Setup(const CVec2D& set_r_centre, const CVec2D& set_direction, const double& set_max_turning_rate,
               const double& set_speed, const double& set_zone_of_deflection, const double& set_zone_of_orientation,
               const double& set_zone_of_perception, const double& set_angular_error_sd, const double& set_omega,
               const double& set_delta, const bool& set_test_fish);
    
    void Move(const double& timestep_inc, const CVec2D& arena_centre, const CVec2D& bottom_right, double dev_angle,
              const bool& side, const int& compartment_size);
    
    CVec2D r_centre;            // Rotational centre of the agent (cm)
    CVec2D direction;           // vector representing the current direction of the agent
    CVec2D desired_direction;
    double speed;               // cm per second
    double max_turning_rate;    // maximum turning rate (degrees per second)
    double zone_of_deflection;
    double zone_of_orientation;  // used only with the three zone model
    double zone_of_perception;
    double zop_angle;
    double angular_error_sd;
    double omega;
    double delta;
    double nnd;
    double fitness;
    int    speed_rank;
    int    omega_rank;
    
    int zop_count;
    int zoo_count;
    int zod_count;
    int equivalence_class;
    CVec2D total_zod;
    CVec2D total_zoo;
    CVec2D total_zop;
    
    bool test_fish;
    
    void TurnTowardsVector(CVec2D& vector, double timestep_inc, double dev_angle);
    void MoveMyself(const double& timestep_inc, const CVec2D& bottom_right, const bool& side, const int& compartment_size);
    void AddPersonalPreference();
};

#endif /* individual_h */
