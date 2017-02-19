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

individual::individual(void) : max_turning_rate(0.0), speed(0.0), zone_of_deflection(0.0), zone_of_orientation(0.0), zone_of_perception(0.0), zop_angle(0.0), angular_error_sd(0.0), zod_count(0.0), zoo_count(0.0), zop_count(0.0)
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
    delta = set_delta;  nnd = 0.0;  fitness = 0.0;  in_range = false;
}

void individual::Move(const double& timestep_inc, const CVec2D& arena_centre, const double& arena_size, double dev_angle, const int& rad_boundary)	// so we have direction (current direction) and a desired direction
{
    if (r_centre.distanceTo(arena_centre) >= (arena_size - 2 * rad_boundary) * (arena_size - 2 * rad_boundary) / 4)   // distanceTo measures square of distances (arena_size / 2)^2
    {
        BoundaryCondition(arena_centre, arena_size);
    }
    
    if(fabs(desired_direction.x) < FLT_EPSILON && fabs(desired_direction.y) < FLT_EPSILON)
    {
        // do nothing
    }
    else
    {
        TurnTowardsVector(desired_direction, timestep_inc, dev_angle);
    }
    
    MoveMyself(timestep_inc);
}

void individual::MoveMyself(const double& timestep_inc)
{
    CVec2D velocity = direction;
    velocity *= (speed * timestep_inc);
    r_centre += velocity;
}

void individual::TurnTowardsVector(CVec2D& vector, const double& timestep_inc, const double& dev_angle)
{
    // Nice algorithm
    double max_degrees = max_turning_rate * timestep_inc;
    
    vector.rotate(dev_angle);
    
    double check_angle = direction.smallestAngleTo(vector);
    
    if(check_angle <= max_degrees)
    {
        direction = vector;
    }
    else if(check_angle > max_degrees)  // should be able to go from the cross produce to the dot product
    {
        double cross_product = direction.cross(vector);
        
        if(cross_product > 0) direction.rotate(max_degrees);
        else direction.rotate(-max_degrees);
    }
}

void individual::BoundaryCondition(const CVec2D& arena_centre, const double arena_size)
{
    CVec2D n;
    n = -(r_centre - arena_centre).normalise();
    desired_direction = n;
    desired_direction.normalise();
}

void individual::AddPersonalPreference(CVec2D& current_cue_centre)
{
    if (in_range)
    {
        CVec2D temp_vec;
        temp_vec = (current_cue_centre - r_centre).normalise();
        temp_vec *= omega;
        desired_direction += temp_vec;
        desired_direction = desired_direction.normalise();
    }
    else
    {
        desired_direction += (direction * omega);
        desired_direction = desired_direction.normalise();
    }
}

void individual::RewardFunction(double& radius_at_approach)
{
    double reward = 1.0;
    radius_at_approach = radius_at_approach - reward;
    
    fitness += reward;
}

void individual::Copy(const individual &other)
{
    r_centre = other.r_centre;	direction = other.direction; 	max_turning_rate = other.max_turning_rate;	speed = other.speed;	zone_of_deflection = other.zone_of_deflection;    zone_of_orientation = other.zone_of_orientation;
    zone_of_perception = other.zone_of_perception;  angular_error_sd = other.angular_error_sd;  omega = other.omega;
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


