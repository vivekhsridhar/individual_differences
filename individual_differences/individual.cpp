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

individual::individual(void) : max_turning_rate(0.0), speed(0.0), zone_of_deflection(0.0), zone_of_perception(0.0), angular_error_sd(0.0), zod_count(0.0), zop_count(0.0)
{
}

individual::~individual(void)
{
}

void individual::Setup(const CVec2D& set_r_centre, const CVec2D& set_direction, const double& set_max_turning_rate,
                       const double& set_speed, const double& set_zone_of_deflection, const double& set_zone_of_orientation,
                       const double& set_zone_of_perception, const double& set_angular_error_sd, const double& set_omega,
                       const double& set_delta)
{
    r_centre = set_r_centre;	direction = set_direction; 	max_turning_rate = set_max_turning_rate;	speed = set_speed;
    zone_of_deflection = set_zone_of_deflection;    zone_of_orientation = set_zone_of_orientation;
    zone_of_perception = set_zone_of_perception;    angular_error_sd = set_angular_error_sd;    omega = set_omega;
    delta = set_delta;  nnd = 0.0;  fitness = 0.0;  omega_rank = -1;    delta_rank = -1;    speed_rank = -1;
}

void individual::Move(const double& timestep_inc, const double& arena_size, const double& dev_angle)	// so we have direction (current direction) and a desired direction
{
    if(fabs(desired_direction.x) < FLT_EPSILON && fabs(desired_direction.y) < FLT_EPSILON)
    {
        // do nothing
    }
    else
    {
        TurnTowardsVector(desired_direction, timestep_inc, dev_angle);
    }
    
    MoveMyself(timestep_inc, arena_size);
}

void individual::MoveMyself(const double& timestep_inc, const double& arena_size)
{
    CVec2D velocity = direction;
    velocity *= (speed * timestep_inc);
    r_centre += velocity;
    
    if (r_centre.x >= arena_size) r_centre.x -= arena_size;
    else if (r_centre.y >= arena_size) r_centre.y -= arena_size;
    
    if (r_centre.x < 0.0) r_centre.x += arena_size;
    else if (r_centre.y < 0.0) r_centre.y += arena_size;
}

void individual::TurnTowardsVector(CVec2D& vector, double timestep_inc, double dev_angle)
{
    // Nice algorithm
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

double individual::FrontBackDistance(CVec2D& centroid, CVec2D& group_direction)
{
    // Draw line through centroid
    CVec2D perp = group_direction;
    perp.rotate(90.0);
    
    CVec2D a;
    CVec2D b;
    a = CVec2D(centroid.x - 50 * perp.x, centroid.y - 50 * perp.y);
    b = CVec2D(centroid.x + 50 * perp.x, centroid.y + 50 * perp.y);
    
    if((centroid - r_centre).smallestAngleTo(group_direction) < 90.0)
        return -DistancePtLine(a, b);
    else
        return DistancePtLine(a, b);
}

double individual::DistancePtLine(CVec2D& a, CVec2D& b)
{
    double diffX = b.x - a.x;
    double diffY = b.y - a.y;
    if ((diffX == 0) && (diffY == 0))
    {
        diffX = r_centre.x - a.x;
        diffY = r_centre.y - a.y;
        return sqrt(diffX * diffX + diffY * diffY);
    }
    
    double t = ((r_centre.x - a.x) * diffX + (r_centre.y - a.y) * diffY) / (diffX * diffX + diffY * diffY);
    
    if (t < 0)
    {
        //point is nearest to the first point i.e x1 and y1
        diffX = r_centre.x - a.x;
        diffY = r_centre.y - a.y;
    }
    else if (t > 1)
    {
        //point is nearest to the end point i.e x2 and y2
        diffX = r_centre.x - b.x;
        diffY = r_centre.y - b.y;
    }
    else
    {
        //if perpendicular line intersect the line segment.
        diffX = r_centre.x - (a.x + t * diffX);
        diffY = r_centre.y - (a.y + t * diffY);
    }
    
    //returning shortest distance
    return sqrt(diffX * diffX + diffY * diffY);
}


