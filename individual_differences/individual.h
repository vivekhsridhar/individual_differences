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
               const double& set_delta);
    
    void Move(const double& timestep_inc, const CVec2D& arena_centre, const double& arena_size, double dev_angle,
              const int& apply_boundary);
    
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
    bool in_range;
    
    int zop_count;
    int zoo_count;
    int zod_count;
    int equivalence_class;
    CVec2D total_zod;
    CVec2D total_zoo;
    CVec2D total_zop;
    
    void TurnTowardsVector(CVec2D& vector, const double& timestep_inc, const double& dev_angle);
    void MoveMyself(const double& timestep_inc);
    void BoundaryCondition(const CVec2D& arena_centre, const double arena_size);
    void AddPersonalPreference(CVec2D& current_cue_centre);
    void RewardFunction(double& radius_at_approach);
    void Copy(const individual& other);
    
    double FrontBackDistance(CVec2D& centroid, CVec2D& group_direction);
    double DistancePtLine(CVec2D& a, CVec2D& b);
};

#endif /* individual_h */
