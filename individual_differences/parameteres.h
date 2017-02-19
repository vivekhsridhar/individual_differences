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
int     arena_size;
int     total_agents;

int     apply_boundary;

double	angular_error_sd;
double	max_turning_rate;
double	zod;            // zone of deflection
double  zoo;
double	zop;            // zone of perception
double	speed;
double  ssd;
double  delta;
double  dsd;
double  omega;
double  osd;
double  alpha;          // blind angle
double  iid;

bool    record;
bool    side;

int     current_cue_position;
CVec2D  centres[number_of_locations];
individual* agent;
cue* CS;

int main();
void SetupSimulation();
void SetupAgents();
void CalculateSocialForces();
void MoveAgents();
CVec2D RandomBoundedPoint();

void NearestNeighbourDistance();
void CalculateGroupProperties(CVec2D&, CVec2D&, double&);
bool GroupTogether();
bool Equivalent(individual&, individual&);	// testing for equivalence classes (school membership)
void EquivalenceClasses();

void Graphics();
void GraphicsWriter(cv::VideoWriter&, int&, const int&);

#endif /* parameteres_h */
