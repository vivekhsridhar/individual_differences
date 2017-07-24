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
int     arena_size;             // width of central compartment where test agent is introduced
int     compartment_size;       // width of side compartments where stimulus agents may be present
int     total_agents;

double	angular_error_sd;       // angular error representing agent's error in movement / sensory integration
double	max_turning_rate;       // limits turning within a timestep (degrees per timestep)
double	zod;
double  zoo;
double	zop;
double	speed;                  // mean of gaussian from which individual speeds are picked
double  ssd;                    // s.d. of gaussian from which individual speeds are picked
double  omega;                  // mean of gaussian from which individual omegas are picked
double  osd;                    // s.d. of gaussian from which individual omegas are picked
double  alpha;                  // blind angle

bool    side;                   // side where conspecifics are present (false - left; true - right)

individual* agent;

//**************************************************************************************************
//**	FUNCTION DECLARATIONS (DEFINITION IN 'MAIN.CPP')    ****************************************
//**************************************************************************************************

int main();
void SetupSimulation();
void SetupAgents();
void CalculateSocialForces();
void MoveAgents();
CVec2D RandomBoundedPoint(bool&, bool&);

void CalculateGroupProperties(CVec2D&, CVec2D&);

void Graphics(CVec2D&);

#endif /* parameteres_h */
