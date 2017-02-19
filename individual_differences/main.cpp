//
//  main.cpp
//  personality_extension:
//  is an individual based simulation of individual differences and animal collectives. Agents in the simulation form a school where we examine the relative position of an individual in a group based on it's trait value relative to others in the group. We also examine changes in group properties as the average trait of individuals in the group changes.
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
std::ofstream outputFile2;

int main()
{
    // Random generator engine from a time-based seed
    unsigned seed = static_cast<unsigned>(std::chrono::high_resolution_clock::now().time_since_epoch().count());
    rnd::set_seed(seed);
    
    // Open output file
    std::string filename_gp;
    filename_gp = "group_properties.csv";
    outputFile1.open(filename_gp.c_str());
    
    std::string filename_rp;
    filename_rp = "relative_positions.csv";
    outputFile2.open(filename_rp.c_str());
    
    // Set parameters
    timestep_inc = 0.1;
    arena_size = 1000;
    top_left.x = 0.0;
    top_left.y = 0.0;
    bottom_right.x = arena_size;
    bottom_right.y = arena_size;
    
    total_agents = 5;
    angular_error_sd = 0.02;
    max_turning_rate = 60.0;
    speed = 1.0;
    ssd = 0.1;
    delta = 1.0;
    dsd = 0.01;
    omega = 1.0;
    osd = 0.01;
    zod = 1.0;
    zoo = 9.0;
    zop = 90000.0;
    alpha = 90.0;
    iid = 0.0;
    
    apply_boundary = 10;
    
    agent = new individual[total_agents];
    CS = new cue[number_of_cues];
    current_cue_position = -1;
    
    fill_n(centres, number_of_locations, CVec2D(0.0, 0.0));
    
    record = false;
    
    int num_replicates = 400;
    int num_timesteps = 20000;
    
    CVec2D centroid, polarisation;
    double rotation;
    
    outputFile1 << "calculated mean delta" << ", " << "calculated mean speed" << ", " << "polarisation" << ", " << "iid" << "\n";
    outputFile2 << "agent omega" << ", " << "agent delta" << ", " << "agent speed" << ", " << "mean omega" << ", " << "mean delta" << ", " << "mean speed" << ", " << "omega rank" << ", " << "delta rank" << ", " << "speed rank" << ", " << "front back distance" << ", " << "centroid distance" << ", " << "nnd" << ", " << "timestep number" << ", " << "replicate" << "\n";
    
    Size S(static_cast<int>(arena_size), static_cast<int>(arena_size));
    
    // loop through speed/delta and omega
    // speed and delta covary in the combination model
    // in case running a single variable model, set the other (speed/delta) to 1.0 with the corresponding s.d. to 0.0 and change the variable on the loop iterator
    for (delta = 0.01; delta <= 0.1; )
    {
        std::cout << "\n" << delta << " " << speed << "\n";
        for (omega = 0.01; omega <= 0.1; )
        {
            std::cout << omega << " ";
            for (int i = 0; i != num_replicates; ++i)
            {
                SetupSimulation();
                bool result = 0;
                
                // calculations below are used to calculate each agent's ranked omega, delta and speed within it's group
                double average_omega = 0.0;
                double average_delta = 0.0;
                double average_speed = 0.0;
                double omega_rank[total_agents];
                double delta_rank[total_agents];
                double speed_rank[total_agents];
                for (int n = 0; n != total_agents; ++n)
                {
                    average_omega += agent[n].omega;
                    average_delta += agent[n].delta;
                    average_speed += agent[n].speed;
                    omega_rank[n] = agent[n].omega;
                    delta_rank[n] = agent[n].delta;
                    speed_rank[n] = agent[n].speed;
                }
                average_omega /= total_agents;
                average_delta /= total_agents;
                average_speed /= total_agents;
                
                sort(omega_rank, omega_rank + total_agents);
                sort(delta_rank, delta_rank + total_agents);
                sort(speed_rank, speed_rank + total_agents);
                
                for (int m = 0; m != total_agents; ++m) // m goes through sorted rank arrays
                {
                    for (int n = 0; n != total_agents; ++n) // n goes through agents
                    {
                        if (omega_rank[m] == agent[n].omega) agent[n].omega_rank = 5 - m;
                        if (delta_rank[m] == agent[n].delta) agent[n].delta_rank = 5 - m;
                        if (speed_rank[m] == agent[n].speed) agent[n].speed_rank = 5 - m;
                    }
                }
                
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
                        //if (j % 20 == 0) Graphics();
                    }
                    
                    ++timestep_number;
                    
                    if (j % 2000 == 0)
                    {
                        result = GroupTogether();
                        if (result)
                        {
                            CalculateGroupProperties(centroid, polarisation, rotation);
                            NearestNeighbourDistance();
                            
                            if (centroid.x > 5.0 && centroid.y > 5.0 && centroid.x < arena_size - 5.0 && centroid.y < arena_size - 5.0) // ignore output if group may be across boundaries (periodic boundary conditions)
                            {
                                for (int a = 0; a != total_agents; ++a)
                                {
                                    outputFile2 << agent[a].omega << ", " << agent[a].delta << ", " << agent[a].speed << ", " << omega << ", " << delta << ", " << speed << ", " << agent[a].omega_rank << ", " << agent[a].delta_rank << ", " << agent[a].speed_rank << ", " << agent[a].FrontBackDistance(centroid, polarisation) << ", " << (agent[a].r_centre - centroid).length() << ", " << agent[a].nnd << ", " << j << ", " << i << "\n";
                                }
                                
                                outputFile1 << average_delta << ", " << average_speed << ", " << polarisation.length() << ", " << iid << "\n";
                            }
                        }
                    }
                }
            }
            
            omega += 0.01;
        }
        
        delta += 0.01;
        speed -= 0.1;
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
        agent[i].Move(timestep_inc, arena_size, dev_angle);
    }
}

void CalculateSocialForces()
{
    CVec2D dir;
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
            temp_vector = (agent[j].r_centre - agent[i].r_centre);
            
            if (std::abs(temp_vector.x) > arena_size / 2)
            {
                if (agent[i].r_centre.x < agent[j].r_centre.x) temp_vector.x = agent[j].r_centre.x - (agent[i].r_centre.x + arena_size);
                else temp_vector.x = agent[j].r_centre.x - (agent[i].r_centre.x - arena_size);
            }
            if (std::abs(temp_vector.y) > arena_size / 2)
            {
                if (agent[i].r_centre.y < agent[j].r_centre.y) temp_vector.y = agent[j].r_centre.y - (agent[i].r_centre.y + arena_size);
                else temp_vector.y = agent[j].r_centre.y - (agent[i].r_centre.y - arena_size);
            }
            
            // check to see if it is reasonable that you may be interacting
            if(temp_vector.x * temp_vector.x > zop_length || temp_vector.y * temp_vector.y > zop_length)
            {
                // cannot be close enough to interact with
            }
            else
            {
                dist = temp_vector.length() * temp_vector.length();	// fastest way to check distance
                
                if(dist < zod_length)	// this has highest priority
                {
                    temp_vector = temp_vector.normalise();
                    
                    agent[i].total_zod+= (-temp_vector);
                    agent[j].total_zod+= (temp_vector);
                    agent[i].zod_count++;
                    agent[j].zod_count++;
                }
                
                else if (dist < zoo_length)
                {
                    if(temp_vector.smallestAngleTo(agent[i].direction) < (180.0 - alpha/2))
                    {
                        agent[i].total_zoo += agent[j].direction.normalise();
                        agent[i].zoo_count++;
                    }
                    if((-temp_vector).smallestAngleTo(agent[j].direction) < (180.0 - alpha/2))
                    {
                        agent[j].total_zoo += agent[i].direction.normalise();
                        agent[j].zoo_count++;
                    }
                }
                
                else if(dist < zop_length)
                {
                    temp_vector = temp_vector.normalise();
                    
                    if(temp_vector.smallestAngleTo(agent[i].direction) < (180.0 - alpha/2))
                    {
                        agent[i].total_zop += temp_vector;
                        agent[i].zop_count++;
                    }
                    if((-temp_vector).smallestAngleTo(agent[j].direction) < (180.0 - alpha/2))
                    {
                        agent[j].total_zop += (-temp_vector);
                        agent[j].zop_count++;
                    }
                }
            }
        }
    }
    
    // now have total_zod and total_zop calculated for all individuals
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
            if(fabs(agent[k].total_zoo.x) < FLT_EPSILON && fabs(agent[k].total_zoo.y) < FLT_EPSILON)	// if no zoo fish were found or if they cancelled each other out
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
            if(fabs(agent[k].total_zop.x) < FLT_EPSILON && fabs(agent[k].total_zop.y) < FLT_EPSILON)	// if no zoo fish were found or if they cancelled each other out
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
            if(fabs(agent[k].total_zoo.x) + fabs(agent[k].total_zop.x) < FLT_EPSILON && fabs(agent[k].total_zoo.y) + fabs(agent[k].total_zop.y) < FLT_EPSILON)	// if no zoo fish were found or if they cancelled each other out
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
    
    for(int i = 0; i != total_agents; ++i)
    {
        CVec2D set_direction(1.0, 0.0);	// need to be set to unit vectors
        set_direction.rotate(uniform() * 360.0);
        set_r_centre = RandomBoundedPoint();
        set_omega = rnd::normal(omega, osd);
        set_delta = rnd::normal(delta, dsd);
        if (set_omega < 0.0) set_omega = 0.0;
        if (set_delta < 0.0) set_delta = 0.0;
        
        double value = speed - (set_delta - delta) / dsd * ssd;
        set_speed = rnd::normal(value, ssd);
        if (set_speed < 0.0) set_speed = 0.0;
        
        agent[i].Setup(set_r_centre, set_direction, max_turning_rate, set_speed, zod, zoo, zop, angular_error_sd, set_omega, set_delta);
    }
}

CVec2D RandomBoundedPoint()
{
    // Create randomly distributed co-ordinate in the centre of world space
    double range_x = (double) (bottom_right.x - top_left.x);
    double range_y = (double) (bottom_right.y - top_left.y);
    
    double random_x = uniform();
    double random_y = uniform();
    
    // Individuals start in the middle 20th of the tank
    random_x *= (range_x / 100.0f);
    random_y *= (range_y / 100.0f);
    random_x += (double) (range_x / 2.0f - range_x / 200.0f);
    random_y += (double) (range_y / 2.0f - range_y / 200.0f);
    CVec2D random_point(random_x, random_y);
    
    return random_point;
}

void NearestNeighbourDistance()
{
    for (int i = 0; i != total_agents; ++i)
    {
        double tmp = arena_size * arena_size;
        for (int j = 0; j != total_agents; ++j)
        {
            if (i != j)
            {
                double dist = agent[i].r_centre.distanceTo(agent[j].r_centre);
                if (tmp > dist) tmp = dist;
                iid += sqrt(dist);
            }
        }
        
        agent[i].nnd += sqrt(tmp);
        iid /= ((total_agents - 1) * (total_agents - 1));
    }
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

bool GroupTogether()
{
    // need to see if the group is cohesive first - return FALSE is it has split
    EquivalenceClasses();
    
    int group_size = 0;
    int equivalence_class = 0;
    
    equivalence_class = agent[0].equivalence_class;
    
    for(int i = 0; i < total_agents; i++)
    {
        if(agent[i].equivalence_class == equivalence_class) ++group_size;
    }
    
    if(group_size == total_agents) return 1;
    else return 0;
}

bool Equivalent(individual& agent1, individual& agent2)
{
    double dist1;
    
    double largest_zone = (agent1.zone_of_deflection > agent1.zone_of_perception) ? agent1.zone_of_deflection : agent1.zone_of_perception;
    
    dist1 = agent1.r_centre.distanceTo(agent2.r_centre);	// normal distance
    
    if(dist1 < largest_zone)
    {
        return 1;
    }
    return 0;
}

void EquivalenceClasses()
{
    int n = total_agents;
    int *nf = new int[n];
    
    for(int j = 0; j != n; ++j)
    {
        nf[j] = j;
        
        for(int k = 0; k != (j); ++k)
        {
            nf[k] = nf[nf[k]];
            if(Equivalent(agent[j], agent[k])) nf[nf[nf[k]]] = j;
        }
    }
    
    for(int j = 0; j != n; ++j) nf[j] = nf[nf[j]];
    
    for(int m = 0; m != n; ++m)
    {
        agent[m].equivalence_class = nf[m];
    }
}


void Graphics()
{
    // Draw arena
    Mat visualisation = Mat::zeros(arena_size, arena_size, CV_8UC3);
    
    // Draw agents
    for (int i = 0; i != total_agents; ++i)
    {
        // agents faster than the mean speed are orange in colour
        if (agent[i].speed > speed) circle(visualisation, Point(agent[i].r_centre.x, agent[i].r_centre.y), 2, Scalar(0, 152, 255), -1, CV_AA);
        // agents slower than the mean are white
        else circle(visualisation, Point(agent[i].r_centre.x, agent[i].r_centre.y), 2, Scalar(241, 240, 236), -1, CV_AA);
        
        circle(visualisation, Point(agent[i].r_centre.x, agent[i].r_centre.y), sqrt(zop), Scalar(241, 240, 0));
    }
    
    // Display timestep number & cue counter on screen
    putText(visualisation, to_string(timestep_number), cvPoint(10,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
    
    imshow("decision_making", visualisation);
    waitKey(1);
}

void GraphicsWriter(VideoWriter& video_writer, int& timestep_number, const int& num_timesteps)
{
    // Draw arena
    Mat visualisation = Mat::zeros(arena_size, arena_size, CV_8UC3);
    
    // Draw agents
    for (int i = 0; i != total_agents; ++i)
    {
        // agents faster than the mean speed are orange in colour
        if (agent[i].speed > speed) circle(visualisation, Point(agent[i].r_centre.x, agent[i].r_centre.y), 2, Scalar(0, 152, 255), -1, CV_AA);
        // agents slower than the mean are white
        else circle(visualisation, Point(agent[i].r_centre.x, agent[i].r_centre.y), 2, Scalar(241, 240, 236), -1, CV_AA);
        
        circle(visualisation, Point(agent[i].r_centre.x, agent[i].r_centre.y), sqrt(zop), Scalar(241, 240, 0));
    }
    
    // Display timestep number & cue counter on screen
    putText(visualisation, to_string(timestep_number), cvPoint(10,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
    
    // Write video
    video_writer.write(visualisation); //writer the frame into the file
    
    if (timestep_number == num_timesteps)
    {
        video_writer.release();
    }
    waitKey(1);
}
