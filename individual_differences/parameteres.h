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
double   timestep_inc;       // time increment (between timesteps)
CVec2D  bottom_right;
CVec2D  top_left;
CVec2D  arena_centre;
int     arena_size;
int     total_agents;

int     reward_zone;
int     food_particles;
int     apply_boundary;
int     cue_range;
int     food_consumed;
double   cue_dist;

double	angular_error_sd;
double	max_turning_rate;
double	zod;            // zone of deflection
double  zoo;            // zone of orientation
double	zop;            // zone of perception
double  alpha;
double  omega;
double  osd;
double  delta;
double  dsd;
double	speed;
double  ssd;
double  iid;

bool    record;

int     current_cue_position;
CVec2D  centres[number_of_locations];
individual* agent;
cue* CS;

int main();
void SetupSimulation();
void SetupAgents();
void SetupEnvironment();
void CalculateSocialForces();
void MoveAgents();
void CueTiming();
void RespondToCue();
CVec2D RandomBoundedPoint();

void NearestNeighbourDistance();
bool CalculateGroupProperties(CVec2D& centroid, CVec2D& polarisation);
bool Equivalent(individual&, individual&);	// testing for equivalence classes (school membership)
void EquivalenceClasses();

void Graphics();
void GraphicsWriter(cv::VideoWriter&, int&, const int&);

#endif /* parameteres_h */
