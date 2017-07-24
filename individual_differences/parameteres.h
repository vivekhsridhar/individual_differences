//
//  parameteres.h
//  individual_differences
//
//  Created by Vivek Hari Sridhar on 10/08/16.
//  Copyright © 2016 Vivek Hari Sridhar. All rights reserved.
//

#ifndef parameteres_h
#define parameteres_h

#include "individual.h"
#include <fstream>

//**************************************************************************************************
//**	PARAMETERS	********************************************************************************
//**************************************************************************************************

int     timestep_number;
double   timestep_inc;
CVec2D  bottom_right;
CVec2D  top_left;
CVec2D  arena_centre;
int     arena_size;
int     total_agents;

int     reward_zone;            // radius of food patch (determines area within which agents are rewarded)
int     food_particles;         // number of food items within the reward_zone
int     apply_boundary;         // distance from the arena edge where boundary conditions are applied
int     cue_range;              // radius within which a cue can be detected
int     food_consumed;          // number of patches exhausted by the group
double  cue_dist;               // distance from centre of the arena at which all three food patches are placed
double  feeding_rate;           // number of food particles an agent consumes per timestep

double	angular_error_sd;       // angular error representing agent's error in movement / sensory integration
double	max_turning_rate;       // limits turning within a timestep (degrees per timestep)
double	zod;
double  zoo;
double	zop;
double  alpha;                  // blind angle
double  omega;                  // mean of gaussian from which individual omegas are picked
double  osd;                    // s.d. of gaussian from which individual omegas are picked
double	speed;                  // mean of gaussian from which individual speeds are picked
double  ssd;                    // s.d. of gaussian from which individual speeds are picked
double  iid;                    // inter-individual distance

int     current_cue_position;
CVec2D  centres[number_of_locations];
individual* agent;
cue* CS;

//**************************************************************************************************
//**	FUNCTION DECLARATIONS (DEFINITION IN 'MAIN.CPP')    ****************************************
//**************************************************************************************************

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
void CalculateGroupProperties(CVec2D& centroid, CVec2D& polarisation);
bool GroupTogether();
bool Equivalent(individual&, individual&);	// testing for equivalence classes (school membership)
void EquivalenceClasses();

void Graphics();

#endif /* parameteres_h */