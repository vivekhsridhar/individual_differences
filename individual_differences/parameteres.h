//
//  parameteres.h
//  DecisionNetwork
//
//  Created by Vivek Hari Sridhar on 17/01/16.
//  Copyright © 2016 Vivek Hari Sridhar. All rights reserved.
//

#ifndef parameteres_h
#define parameteres_h

#include "individual.h"
#include <fstream>

int     timestep_number;    // timestep number
double   timestep_inc;      // time increment (between timesteps)
CVec2D  bottom_right;
CVec2D  top_left;
CVec2D  arena_centre;
int     arena_size;
int     compartment_size;
int     total_agents;

double	angular_error_sd;
double	max_turning_rate;
double	zod;            // zone of deflection
double  zoo;            // zone of orientation
double	zop;            // zone of perception
double	speed;
double  ssd;
double  delta;
double  dsd;
double  omega;
double  osd;
double  alpha;          // blind angle

bool    record;
bool    side;

individual* agent;

int main();
void SetupSimulation();
void SetupAgents();
void CalculateSocialForces();
void MoveAgents();
CVec2D RandomBoundedPoint(bool&, bool&);

void CalculateGroupProperties(CVec2D&, CVec2D&, double&);
void Graphics(CVec2D& centroid);
void GraphicsWriter(cv::VideoWriter&, int&, const int&);

#endif /* parameteres_h */
