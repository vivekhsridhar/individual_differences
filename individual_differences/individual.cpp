//
//  individual.cpp
//  DecisionNetwork
//
//  Created by Vivek Hari Sridhar on 15/01/16.
//  Copyright © 2016 Vivek Hari Sridhar. All rights reserved.
//

#include "individual.h"
#include <limits.h>
#include <float.h>
#include <algorithm>

individual::individual(void) : max_turning_rate(0.0), speed(0.0), zone_of_deflection(0.0), zone_of_perception(0.0), zop_angle(0.0), angular_error_sd(0.0), zod_count(0.0), zop_count(0.0)
{
}

individual::~individual(void)
{
}

void individual::Setup(const CVec2D& set_r_centre, const CVec2D& set_direction, const double& set_max_turning_rate,
                       const double& set_speed, const double& set_zone_of_deflection, const double& set_zone_of_orientation,
                       const double& set_zone_of_perception, const double& set_angular_error_sd, const double& set_omega,
                       const double& set_delta, const bool& set_test_fish)
{
    r_centre = set_r_centre;	direction = set_direction; 	max_turning_rate = set_max_turning_rate;	speed = set_speed;
    zone_of_deflection = set_zone_of_deflection;    zone_of_orientation = set_zone_of_orientation;
    zone_of_perception = set_zone_of_perception;    angular_error_sd = set_angular_error_sd;    omega = set_omega;
    delta = set_delta;  nnd = 0.0;  fitness = 0.0;  speed_rank = -1;    omega_rank = -1;    test_fish = set_test_fish;
}

void individual::Move(const double& timestep_inc, const CVec2D& arena_centre, const CVec2D& bottom_right, double dev_angle, const bool& side, const int& compartment_size)	// so we have direction (current direction) and a desired direction
{
    if(fabs(desired_direction.x) < FLT_EPSILON && fabs(desired_direction.y) < FLT_EPSILON)
    {
        // do nothing
    }
    else
    {
        TurnTowardsVector(desired_direction, timestep_inc, dev_angle);
    }
    
    MoveMyself(timestep_inc, bottom_right, side, compartment_size);
}

void individual::MoveMyself(const double& timestep_inc, const CVec2D& bottom_right, const bool& side, const int& compartment_size)
{
    CVec2D velocity = direction;
    velocity *= (speed * timestep_inc);
    
    if (!test_fish)
    {
        // confine stimulus fish to their side of the tank (keep them on the other side of the partition)
        if (side == false)
        {
            if ((r_centre.x + velocity.x) >= compartment_size) velocity.x -= 1.0;
            else if ((r_centre.x + velocity.x) < 0.0) velocity.x += 1.0;
            
            if ((r_centre.y + velocity.y) >= bottom_right.y) velocity.y -= 1.0;
            else if ((r_centre.y + velocity.y) < 0.0) velocity.y += 1.0;
        }
        else
        {
            if ((r_centre.x + velocity.x) >= bottom_right.x) velocity.x -= 1.0;
            else if ((r_centre.x + velocity.x) < bottom_right.x - compartment_size) velocity.x += 1.0;
            
            if ((r_centre.y + velocity.y) >= bottom_right.y) velocity.y -= 1.0;
            else if ((r_centre.y + velocity.y) < 0.0) velocity.y += 1.0;
        }
    }
    else
    {
        // confine test fish to the main tank area
        if ((r_centre.x + velocity.x) >= bottom_right.x - compartment_size) velocity.x -= 1.0;
        else if ((r_centre.x + velocity.x) < compartment_size) velocity.x += 1.0;
        
        if ((r_centre.y + velocity.y) >= bottom_right.y) velocity.y -= 1.0;
        else if ((r_centre.y + velocity.y) < 0.0) velocity.y += 1.0;
    }
    
    r_centre += velocity;
}

void individual::TurnTowardsVector(CVec2D& vector, double timestep_inc, double dev_angle)
{
    double max_degrees = max_turning_rate * timestep_inc;
    
    vector.rotate(dev_angle);
    
    double check_angle = direction.smallestAngleTo(vector);
    
    if (check_angle <= max_degrees)
    {
        direction = vector;
    }
    else if (check_angle > max_degrees)  // should be able to go from the cross produce to the dot product
    {
        double cross_product = direction.cross(vector);
        
        if(cross_product > 0) direction.rotate(max_degrees);
        else direction.rotate(-max_degrees);
    }
}

void individual::AddPersonalPreference()
{
    desired_direction += (direction * omega);
    desired_direction = desired_direction.normalise();
}

