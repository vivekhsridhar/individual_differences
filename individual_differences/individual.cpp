//
//  individual.cpp
//  individual_differences
//
//  Created by Vivek Hari Sridhar on 10/08/16.
//  Copyright © 2016 Vivek Hari Sridhar. All rights reserved.
//

#include "individual.h"
#include <limits.h>
#include <float.h>
#include <algorithm>

//**************************************************************************************************
//**	CONSTRUCTORS AND SETUP OF CLASS 'INDIVIDUAL'    ********************************************
//**************************************************************************************************

individual::individual(void) : max_turning_rate(0.0), speed(0.0), zone_of_deflection(0.0), zone_of_perception(0.0), angular_error_sd(0.0), zod_count(0.0), zop_count(0.0)
{
}

individual::~individual(void)
{
}

void individual::Setup(const CVec2D& set_r_centre, const CVec2D& set_direction, const double& set_max_turning_rate,
                       const double& set_speed, const double& set_zone_of_deflection, const double& set_zone_of_orientation,
                       const double& set_zone_of_perception, const double& set_angular_error_sd, const double& set_omega,
                       const bool& set_test_fish)
{
    r_centre = set_r_centre;	direction = set_direction; 	max_turning_rate = set_max_turning_rate;	speed = set_speed;
    zone_of_deflection = set_zone_of_deflection;    zone_of_orientation = set_zone_of_orientation;
    zone_of_perception = set_zone_of_perception;    angular_error_sd = set_angular_error_sd;    omega = set_omega;
    test_fish = set_test_fish;
}

//**************************************************************************************************
//**	OTHER MEMBER FUNCTIONS	********************************************************************
//**************************************************************************************************

void individual::Move(const double& timestep_inc, const CVec2D& arena_centre, const CVec2D& bottom_right, double dev_angle, const bool& side, const int& compartment_size)
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
    r_centre += velocity;
    
    if (!test_fish)
    {
        // confine stimulus fish to their side of the tank (keep them on the other side of the partition)
        if (side == false)
        {
            if (r_centre.x >= compartment_size) r_centre.x = compartment_size;
            else if (r_centre.x < 0.0) r_centre.x = 0.0;
            
            if (r_centre.y >= bottom_right.y) r_centre.y = bottom_right.y;
            else if (r_centre.y < 0.0) r_centre.y = 0.0;
        }
        else
        {
            if (r_centre.x >= bottom_right.x) r_centre.x = bottom_right.x;
            else if (r_centre.x < bottom_right.x - compartment_size) r_centre.x = bottom_right.x - compartment_size;
            
            if (r_centre.y >= bottom_right.y) r_centre.y = bottom_right.y;
            else if (r_centre.y < 0.0) r_centre.y = 0.0;
        }
    }
    else
    {
        // confine test fish to the main tank area
        if (r_centre.x >= bottom_right.x - compartment_size) r_centre.x = bottom_right.x - compartment_size;
        else if (r_centre.x < compartment_size) r_centre.x = compartment_size;
        
        if (r_centre.y >= bottom_right.y) r_centre.y = bottom_right.y;
        else if (r_centre.y < 0.0) r_centre.y = 0.0;
    }
}

void individual::TurnTowardsVector(CVec2D& vector, double timestep_inc, double dev_angle)
{
    double max_degrees = max_turning_rate * timestep_inc;
    vector.rotate(dev_angle);
    
    double check_angle = direction.smallestAngleTo(vector);
    
    // turn maximally (based on turning rate) or to desired direction of movement (whichever is smaller)
    if (check_angle <= max_degrees)
    {
        direction = vector;
    }
    else if (check_angle > max_degrees)
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
