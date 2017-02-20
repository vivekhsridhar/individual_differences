//
//  main.cpp
//  personality_extension:
//  is an individual based simulation of individual differences and animal collectives. Agents in the simulation perform the "sociability assay" widely used in the field of "animal personalities". In this model, we show that what the field refers to as differences in "sociabilty" can in fact be reproduced with both individual differences in social interactions and differences in speed (and a combination of the two). Thus, observing the pattern resulting from the assay is not evidence for varying "sociability" in individuals. In fact, the speed model tends to suggest that it might actually be the opposite.
//
//  ----------------------------------------------------------------------------
//  Important Parameters:
//  1.  speed (and ssd): The speed model qualitatively reproduces the pattern shown by the "sociability assay". ssd is the standard deviation (s.d.) of the normal distribution used to pick agents (so agents across replicate simulations differ in their speeds as well)
//  2.  delta (and dsd): The delta model represents social interactions between individuals as interpreted by the "personality" literature. Differences in delta also reproduces the assay pattern. dsd is the standard deviation (s.d.) of the normal distribution used to pick delta
//  3.  omega (and osd): Omega is how individuals weight their personal preference. This refers to their current direction of movement in this particular case. osd is the standard deviation (s.d.) of the normal distribution used to pick omega
//
//  ----------------------------------------------------------------------------
//  Created by Vivek Hari Sridhar on 10/08/16.
//  Copyright © 2016 Vivek Hari Sridhar. All rights reserved.
//

#include <cmath>
#include <limits.h>
#include <cfloat>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <algorithm>
#include <functional>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "parameteres.h"

using namespace std;
using namespace rnd;
using namespace cv;

std::ofstream outputFile1;

int main()
{
    // Random generator engine from a time-based seed
    unsigned seed = static_cast<unsigned>(std::chrono::high_resolution_clock::now().time_since_epoch().count());
    rnd::set_seed(seed);
    
    // Open output file
    std::string filename_gp;
    filename_gp = "sociability_assay.csv";
    outputFile1.open(filename_gp.c_str());
    
    // Set parameters
    timestep_inc = 0.1;
    arena_size = 300;
    compartment_size = 100;
    top_left.x = 0.0;
    top_left.y = 0.0;
    bottom_right.x = arena_size + 2 * compartment_size;
    bottom_right.y = arena_size;
    arena_centre.x = (bottom_right.x - top_left.x) / 2.0;
    arena_centre.y = (bottom_right.y - top_left.y) / 2.0;
    
    total_agents = 6;
    angular_error_sd = 0.02;
    max_turning_rate = 60.0;
    speed = 1.0;
    ssd = 0.1;
    delta = 1.0;
    dsd = 0.0;
    omega = 0.0;
    osd = 0.01;
    zod = 1.0;
    zoo = 36.0;
    zop = 1000000.0;
    alpha = 90.0;
    
    agent = new individual[total_agents];
    
    record = false;
    
    int num_replicates = 400;
    int num_timesteps = 100000;
    
    CVec2D centroid, polarisation;
    double rotation;
    
    outputFile1 << "side"  << ", " << "agent speed" << ", " << "agent delta" << ", " << "agent omega" << ", " << "position.x" << ", " << "position.y" << ", " << "replicate" << ", " << "direction.x" << ", " << "direction.y" << "\n";
    
    Size S(static_cast<int>(arena_size), static_cast<int>(arena_size));
    
    // loop through speed/delta and omega
    // speed and delta covary in the combination model
    // in case running a single variable model, set the other (speed/delta) to 1.0 with the corresponding s.d. to 0.0 and change the variable on the loop iterator
    for (speed = 5.0; speed >= 0.5; )
    {
        std::cout << "\n" << delta << " " << speed << "\n";
        for (omega = 0.01; omega <= 0.1; )
        {
            std::cout << omega << " ";
            for (int i = 0; i != num_replicates; ++i)
            {
                SetupSimulation();
                
                for(int j = 0; j != num_timesteps; ++j)
                {
                    MoveAgents();
                    
                    if (record)
                    {
                        VideoWriter video_writer("/Users/Vivek/Desktop/test.mp4", CV_FOURCC('M', 'P', '4', 'V'), 120, S, true);
                        if (j % 20 == 0) GraphicsWriter(video_writer, j, num_timesteps);
                    }
                    else
                    {
                        CalculateGroupProperties(centroid, polarisation, rotation);
                        //if (j % 20 == 0) Graphics(centroid);
                    }
                    
                    ++timestep_number;
                    
                    if (j != 0 && j % 10000 == 0)
                    {
                        outputFile1 << side  << ", " << agent[total_agents - 1].speed << ", " << agent[total_agents - 1].delta << ", " << agent[total_agents - 1].omega << ", " << agent[total_agents - 1].r_centre.x << ", " << agent[total_agents - 1].r_centre.y << ", " << i << ", " << agent[total_agents - 1].direction.x << ", " << agent[total_agents - 1].direction.y << "\n";
                    }
                }
            }
            
            omega += 0.01;
        }
        
        speed -= 0.5;
    }
    
    echo("simulation end");
    return 0;
}

void MoveAgents()
{
    CalculateSocialForces();
    double dev_angle = 360.0f * normal(0.0, angular_error_sd);
    
    for(int i = 0; i != total_agents; ++i)
    {
        agent[i].AddPersonalPreference();
        
        // now each fish has a unit vector which is its desired direction of travel in the next timestep
        agent[i].Move(timestep_inc, arena_centre, bottom_right, dev_angle, side, compartment_size);
    }
}

void CalculateSocialForces()
{
    double dist;
    CVec2D temp_vector;
    
    double zop_length = agent[0].zone_of_perception;
    double zoo_length = agent[0].zone_of_orientation;
    double zod_length = agent[0].zone_of_deflection;
    
    for(int clear = 0; clear != total_agents; ++clear)
    {
        agent[clear].zod_count = 0;
        agent[clear].zoo_count = 0;
        agent[clear].zop_count = 0;
        agent[clear].total_zod.Clear();
        agent[clear].total_zoo.Clear();
        agent[clear].total_zop.Clear();
    }
    
    for(int i = 0; i != total_agents; ++i)
    {
        for(int j = (i+1); j != total_agents; ++j)
        {
            // check to see if it is reasonable that you may be interacting
            if(agent[i].r_centre.x < (agent[j].r_centre.x - zop_length) || agent[i].r_centre.x > (agent[j].r_centre.x + zop_length)
               || agent[i].r_centre.y < (agent[j].r_centre.y - zop_length) || agent[i].r_centre.y > (agent[j].r_centre.y + zop_length))
            {
                // cannot be close enough to interact with
            }
            else
            {
                dist = agent[i].r_centre.distanceTo(agent[j].r_centre);	// fastest way to check distance
                
                if(dist < zod_length)	// this has highest priority
                {
                    temp_vector = (agent[j].r_centre - agent[i].r_centre);
                    temp_vector = temp_vector.normalise();
                    
                    agent[i].total_zod+= (-temp_vector);
                    agent[j].total_zod+= (temp_vector);
                    agent[i].zod_count++;
                    agent[j].zod_count++;
                }
                
                else if (dist < zoo_length)
                {
                    if((agent[j].r_centre - agent[i].r_centre).smallestAngleTo(agent[i].direction) < (180.0 - alpha/2))
                    {
                        agent[i].total_zoo += agent[j].direction.normalise();
                        agent[i].zoo_count++;
                    }
                    if((agent[i].r_centre - agent[j].r_centre).smallestAngleTo(agent[j].direction) < (180.0 - alpha/2))
                    {
                        agent[j].total_zoo += agent[i].direction.normalise();
                        agent[j].zoo_count++;
                    }
                }
                
                else if(dist < zop_length)
                {
                    temp_vector = (agent[j].r_centre - agent[i].r_centre);
                    temp_vector = temp_vector.normalise();
                    
                    if((agent[j].r_centre - agent[i].r_centre).smallestAngleTo(agent[i].direction) < (180.0 - alpha/2))
                    {
                        agent[i].total_zop += temp_vector;
                        agent[i].zop_count++;
                    }
                    if((agent[i].r_centre - agent[j].r_centre).smallestAngleTo(agent[j].direction) < (180.0 - alpha/2))
                    {
                        agent[j].total_zop += (-temp_vector);
                        agent[j].zop_count++;
                    }
                }
            }
        }
    }
    
    // now have total_zod, total_zoo and total_zop calculated for all individuals
    for(int k = 0; k != total_agents; ++k)
    {
        if(agent[k].zod_count > 0)
        {
            if(fabs(agent[k].total_zod.x) < FLT_EPSILON && fabs(agent[k].total_zod.y) < FLT_EPSILON)	// if it has completely cancelled out
            {
                agent[k].desired_direction = agent[k].direction;
            }
            else
            {
                agent[k].desired_direction = agent[k].total_zod.normalise();
            }
        }
        else if(agent[k].zoo_count > 0 && agent[k].zop_count == 0)
        {
            if(fabs(agent[k].total_zoo.x) < FLT_EPSILON && fabs(agent[k].total_zoo.y) < FLT_EPSILON)
            {
                // just do a correlated random walk
                agent[k].desired_direction = agent[k].direction;
            }
            else
            {
                agent[k].desired_direction = agent[k].total_zoo.normalise();
            }
        }
        else if(agent[k].zoo_count == 0 && agent[k].zop_count > 0)
        {
            if(fabs(agent[k].total_zop.x) < FLT_EPSILON && fabs(agent[k].total_zop.y) < FLT_EPSILON)
            {
                // just do a correlated random walk
                agent[k].desired_direction = agent[k].direction;
            }
            else
            {
                agent[k].desired_direction = agent[k].total_zop.normalise();
            }
        }
        else if(agent[k].zoo_count > 0 && agent[k].zop_count > 0)
        {
            if(fabs(agent[k].total_zoo.x) + fabs(agent[k].total_zop.x) < FLT_EPSILON && fabs(agent[k].total_zoo.y) + fabs(agent[k].total_zop.y) < FLT_EPSILON)
            {
                // just do a correlated random walk
                agent[k].desired_direction = agent[k].direction;
            }
            else
            {
                agent[k].desired_direction = (agent[k].total_zoo.normalise() + agent[k].total_zop.normalise()).normalise();
            }
        }
        
        // weigh social forces by delta
        agent[k].desired_direction *= agent[k].delta;
    }
}

void SetupSimulation()
{
    timestep_number = 0;	// timestep number
    SetupAgents();
}

void SetupAgents()
{
    CVec2D set_r_centre;
    double set_omega;
    double set_speed;
    double set_delta;
    bool   set_test_fish;
    
    double omega_mean = 0.09 * uniform() + 0.01;
    double speed_mean = 4.5 * uniform() + 0.5;
    
    side = (bool) rnd::integer(2);
    
    for(int i = 0; i != total_agents; ++i)
    {
        CVec2D set_direction(1.0, 0.0);	// need to be set to unit vectors
        set_direction.rotate(uniform() * 360.0);
        
        if (i < total_agents - 1)
        {
            // parameters for stimulus fish
            set_omega = rnd::normal(omega_mean, osd);
            set_delta = rnd::normal(delta, dsd);
            set_speed = rnd::normal(speed_mean, ssd);
            if (set_omega < 0.0) set_omega = 0.0;
            if (set_speed < 0.0) set_speed = 0.0;
            set_test_fish = false;
            set_r_centre = RandomBoundedPoint(side, set_test_fish);
        }
        else
        {
            // parameters for test fish
            set_omega = rnd::normal(omega, osd);
            set_delta = rnd::normal(delta, dsd);
            set_speed = rnd::normal(speed, ssd);
            if (set_omega < 0.0) set_omega = 0.0;
            if (set_delta < 0.0) set_delta = 0.0;
            if (set_speed < 0.0) set_speed = 0.0;
            set_test_fish = true;
            set_r_centre = RandomBoundedPoint(side, set_test_fish);
        }
        agent[i].Setup(set_r_centre, set_direction, max_turning_rate, set_speed, zod, zoo, zop, angular_error_sd, set_omega, set_delta, set_test_fish);
    }
}

CVec2D RandomBoundedPoint(bool& side, bool& test_fish)
{
    CVec2D random_point;
    if (!test_fish)
    {
        // Create randomly distributed co-ordinate in the centre of world space
        double range_x = (double) (compartment_size - top_left.x);
        double range_y = (double) (bottom_right.y - top_left.y);
        
        double random_x = uniform();
        double random_y = uniform();
        
        // Individuals start in the middle 100th of the tank
        random_x *= (range_x / 100.0f);
        random_y *= (range_y / 100.0f);
        
        if (side == false) random_x += (double) (range_x / 2.0f - range_x / 200.0f);
        else random_x += (double) (arena_size + compartment_size + range_x / 2.0f - range_x / 200.0f);
        
        random_y += (double) (range_y / 2.0f - range_y / 200.0f);
        
        random_point = CVec2D(random_x, random_y);
    }
    else
    {
        random_point = CVec2D(bottom_right.x / 2.0, bottom_right.y / 2.0);
    }
    
    return random_point;
}

void CalculateGroupProperties(CVec2D& centroid, CVec2D& polarisation, double& rotation)
{
    centroid.x = 0.0; centroid.y = 0.0;
    polarisation.x = 0.0; polarisation.y = 0.0;
    rotation = 0.0;
    
    for(int i = 0; i != total_agents - 1; ++i)
    {
        centroid += agent[i].r_centre;
        polarisation += (agent[i].direction.normalise());
    }
    centroid /= (total_agents - 1);
    polarisation /= (total_agents - 1);
    
    for(int i = 0; i != total_agents - 1; ++i)
    {
        CVec2D r;
        r = (agent[i].r_centre - centroid).normalise();
        rotation += r.cross(agent[i].direction);
    }
    rotation /= total_agents;
    rotation = abs(rotation);
}

void Graphics(CVec2D& centroid)
{
    // Draw arena
    Mat visualisation = Mat::zeros(arena_size, arena_size + 2 * compartment_size, CV_8UC3);
    
    // Draw compartments
    line(visualisation, Point(compartment_size, 0), Point(compartment_size, arena_size), Scalar(255, 255, 255));
    line(visualisation, Point(arena_size + compartment_size, 0), Point(arena_size + compartment_size, arena_size), Scalar(255, 255, 255));
    
    // Draw agents
    for (int i = 0; i != total_agents; ++i)
    {
        if (i < total_agents - 1) circle(visualisation, Point(agent[i].r_centre.x, agent[i].r_centre.y), 2, Scalar(0, 152, 255), -1, CV_AA);
        else circle(visualisation, Point(agent[i].r_centre.x, agent[i].r_centre.y), 2, Scalar(241, 240, 236), -1, CV_AA);
    }
    
    circle(visualisation, Point(centroid.x, centroid.y), sqrt(zop), Scalar(156, 188, 26), 1, CV_AA);
    
    // Display timestep number & cue counter on screen
    putText(visualisation, to_string(timestep_number), cvPoint(10,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
    
    imshow("decision_making", visualisation);
    waitKey(1);
}

void GraphicsWriter(VideoWriter& video_writer, int& timestep_number, const int& num_timesteps)
{
    // Draw arena
    Mat visualisation = Mat::zeros(arena_size, arena_size + 2 * compartment_size, CV_8UC3);
    
    // Draw compartments
    line(visualisation, Point(compartment_size, 0), Point(compartment_size, arena_size), Scalar(255, 255, 255));
    line(visualisation, Point(arena_size + compartment_size, 0), Point(arena_size + compartment_size, arena_size), Scalar(255, 255, 255));
    
    // Draw agents
    for (int i = 0; i != total_agents; ++i)
    {
        if (i < total_agents - 1) circle(visualisation, Point(agent[i].r_centre.x, agent[i].r_centre.y), 2, Scalar(0, 152, 255), -1, CV_AA);
        else circle(visualisation, Point(agent[i].r_centre.x, agent[i].r_centre.y), 2, Scalar(241, 240, 236), -1, CV_AA);
    }
    
    // Display timestep number & cue counter on screen
    putText(visualisation, to_string(timestep_number), Point(10, 30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(200, 200, 250), 1, CV_AA);
    
    // Write video
    video_writer.write(visualisation); // write the frame into the file
    
    if (timestep_number == num_timesteps)
    {
        video_writer.release();
    }
    waitKey(1);
}
