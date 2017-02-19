//
//  main.cpp
//  DecisionNetwork:
//  is an individual based simulation of decision making in animal groups. Agents in the simulation experience various
//  environmental cues and associated rewards. The Rescorla - Wagner model is used as a model for associative learning.
//
//  Quirks:
//  1.  the reward_radius (in CLASS CUE) is used as the independent variable to calculate reward experienced by each
//  individual. A reward function is generated based on this radius such that obtained reward is proportional to this radius.
//  The radius is reduced discretely in integers until it hits zero. Agents that approach the cue after this do not receive a
//  reward. This is initialised as reward_reset
//
//  2.  the reward_zone on the other hand is a zone within which the agents need to enter in order to receive a reward
//
//  3.  the split_between parameter determines how many individuals these units of reward are distributed among. Once set, it
//  should not change (when comparing groups of different sizes) but the reward reset and the split between have to be
//  identical. This ensures that an individual on average receives a reward of 1 unit. The rescorla-wagner learning rate
//  is set accordingly
//
//  4.  distanceTo function (in CLASS VECTOR2D) measures distance squared. That is the units used to express zone of
//  deflection, orientation and perception
//
//  Created by Vivek Hari Sridhar on 14/01/16.
//  Copyright © 2016 Vivek Hari Sridhar. All rights reserved.
//

#include <cmath>
#include <limits.h>
#include <cfloat>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "parameteres.h"
#include <armadillo>

using namespace std;
using namespace rnd;
using namespace cv;

std::ofstream outputFile1;
std::ofstream outputFile2;
std::ofstream outputFile3;

int main()
{
    // Random generator engine from a time-based seed
    unsigned seed = static_cast<unsigned>(std::chrono::high_resolution_clock::now().time_since_epoch().count());
    rnd::set_seed(seed);
    
    // Open output file
    std::string filename_gf;
    filename_gf = "group_foraging.csv";
    outputFile1.open(filename_gf.c_str());
    
    std::string filename_if;
    filename_if = "individual_foraging.csv";
    outputFile2.open(filename_if.c_str());
    
    std::string filename_fc;
    filename_fc = "food_consumption.csv";
    outputFile3.open(filename_fc.c_str());
    
    // Set parameters
    timestep_inc = 0.1;
    arena_size = 500;
    top_left.x = 0.0;
    top_left.y = 0.0;
    bottom_right.x = arena_size;
    bottom_right.y = arena_size;
    arena_centre.x = arena_size / 2.0;
    arena_centre.y = arena_size / 2.0;
    
    total_agents = 20;
    angular_error_sd = 0.02;
    max_turning_rate = 60.0;
    zod = 1.0;
    zoo = 36.0;
    zop = 225.0;
    alpha = 90.0;
    omega = 0.01;
    osd = 0.01;
    delta = 1.0;
    dsd = 0.0;
    speed = 1.0;
    ssd = 0.1;
    iid = 0.0;
    
    reward_zone = 10;
    food_particles = 50000;
    cue_dist = 150;
    apply_boundary = 10;
    cue_range = 30;
    food_consumed = 0;
    
    agent = new individual[total_agents];
    CS = new cue[number_of_cues];
    current_cue_position = -1;
    
    fill_n(centres, number_of_locations, CVec2D(0.0, 0.0));
    
    record = false;
    
    int num_replicates = 400;
    int num_timesteps = 100000;
    
    CVec2D centroid, polarisation;
    
    outputFile1 << "mean speed" << ", " << "calculated mean speed" << ", " << "mean delta" << ", " << "calculated mean delta" << ", " << "mean omega" << ", " << "calculated mean omega" << ", " << "foraging time" << "\n";
    outputFile2 << "agent speed" << ", " << "mean speed" << ", " << "calculated mean speed" << ", " << "agent delta" << ", " << "mean delta" << ", " << "calculated mean delta" << ", " << "agent omega" << ", " << "mean omega" << ", " << "calculated mean omega" << ", " << "fitness" << "\n";
    outputFile3 << "time" << ", " << "food consumed" << ", " << "mean speed" << ", " << "calculated mean speed" << ", " << "mean delta" << ", " << "calculated mean delta" << ", " << "mean omega" << ", " << "calculated mean omega" << ", " << "replicate" << "\n";
    
    Size S(static_cast<int>(arena_size), static_cast<int>(arena_size));
    
    for (speed = 2.0; speed >= 0.2; )
    {
        std::cout << "\n" << delta << " " << speed << "\n";
        for (omega = 0.01; omega <= 0.1; )
        {
            std::cout << omega << " ";
            for (int i = 0; i != num_replicates; ++i)
            {
                SetupSimulation();
                
                int time_at_consumption = 0;
                bool result = 0;
                food_consumed = 0;
                
                double average_omega = 0.0;
                double average_delta = 0.0;
                double average_speed = 0.0;
                for (int n = 0; n != total_agents; ++n)
                {
                    average_omega += agent[n].omega;
                    average_delta += agent[n].delta;
                    average_speed += agent[n].speed;
                }
                average_omega /= total_agents;
                average_delta /= total_agents;
                average_speed /= total_agents;
                
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
                    
                    if (j % 5000 == 0)
                    {
                        result = CalculateGroupProperties(centroid, polarisation);
                        if (result)
                        {
                            int total_food = 0;
                            for (int f = 0; f != number_of_cues; ++f)
                            {
                                if (CS[f].status) total_food += CS[f].reward_radius;
                            }
                            int cum_food_intake = number_of_cues * food_particles - total_food;
                            
                            outputFile3 << j << ", " << cum_food_intake << ", " << speed << ", " << average_speed << ", " << delta << ", " << average_delta << ", " << omega << ", " << average_omega << ", " << i << "\n";
                        }
                    }
                    
                    if (food_consumed == 3 && time_at_consumption == 0)
                    {
                        time_at_consumption = timestep_number;
                        break;
                    }
                    if (j == num_timesteps - 1 && food_consumed != 3) time_at_consumption = num_timesteps;
                }
                
                outputFile1 << speed << ", " << average_speed << ", " << delta << ", " << average_delta << ", " << omega << ", " << average_omega << ", " << time_at_consumption << "\n";
                
                for (int a = 0; a != total_agents; ++a)
                {
                    outputFile2 << agent[a].speed << ", " << speed << ", " << average_speed << ", " << agent[a].delta << ", " << delta << ", " << average_delta << ", " << agent[a].omega << ", " << omega << ", " << average_omega << ", " << agent[a].fitness << "\n";
                }
            }
            
            omega += 0.01;
        }
        
        speed -= 0.2;
    }
    
    echo("simulation end");
    return 0;
}

void MoveAgents()
{
    CalculateSocialForces();
    CueTiming();
    RespondToCue();
    double dev_angle = 360.0f * normal(0.0, angular_error_sd);
    
    for(int i = 0; i != total_agents; ++i)
    {
        // now each fish has a unit vector which is its desired direction of travel in the next timestep
        agent[i].Move(timestep_inc, arena_centre, arena_size, dev_angle, apply_boundary);
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

void CueTiming()
{
    if(timestep_number == 0)
    {
        int array[number_of_locations];
        for (int i = 0; i != number_of_locations; ++i) array[i] = i;
        
        for (int i = 0; i != number_of_cues; ++i)
        {
            // pick a random cue position
            do current_cue_position = rnd::integer(number_of_locations);
            while (array[current_cue_position] == -1);
            array[current_cue_position] = -1;
            
            // activate chosen cue at chosen location
            CS[i].centre = centres[current_cue_position];
            CS[i].status = true;
            CS[i].is_reward = true;
        }
    }
}

void RespondToCue()
{
    for (int j = 0; j != number_of_cues; ++j)
    {
        for (int i = 0; i != total_agents; ++i)
        {
            if (CS[j].status)
            {
                if (agent[i].r_centre.distanceTo(CS[j].centre) < cue_range * cue_range)
                {
                    agent[i].in_range = true;
                    
                    if (agent[i].r_centre.distanceTo(CS[j].centre) < reward_zone * reward_zone)
                    {
                        agent[i].RewardFunction(CS[j].reward_radius);
                    }
                }
            }
            
            agent[i].AddPersonalPreference(CS[j].centre);
            agent[i].in_range = false;
            
            if (CS[j].reward_radius <= FLT_EPSILON)
            {
                CS[j].status = false;
                CS[j].reward_radius = food_particles;
                ++food_consumed;
                break;
            }
        }
    }
}

void SetupSimulation()
{
    timestep_number = 0;	// timestep number
    SetupEnvironment();
    SetupAgents();
}

void SetupAgents()
{
    CVec2D set_r_centre;
    double set_omega;
    double set_delta;
    double set_speed;
    
    for(int i = 0; i != total_agents; ++i)
    {
        CVec2D set_direction(1.0, 0.0);	// need to be set to unit vectors
        set_direction.rotate(uniform() * 360.0);
        set_r_centre = RandomBoundedPoint();
        set_omega = rnd::normal(omega, osd);
        set_delta = rnd::normal(delta, dsd);
        if (set_omega < 0.0) set_omega = 0.0;
        if (set_delta < 0.0) set_delta = 0.0;
        
        set_speed = rnd::normal(speed, ssd);
        if (set_speed < 0.0) set_speed = 0.0;
        
        agent[i].Setup(set_r_centre, set_direction, max_turning_rate, set_speed, zod, zoo, zop, angular_error_sd, set_omega, set_delta);
    }
}

void SetupEnvironment()
{
    double theta = 2 * Pi / number_of_locations;
    for (int i = 0; i != number_of_locations; ++i)
    {
        centres[i] = arena_centre + CVec2D(cue_dist * cos(i * theta), cue_dist * sin(i * theta));
    }
    
    CVec2D set_centre;
    int set_timer;
    int set_reward_radius;
    bool set_status;
    double set_reliability;
    
    bool set_is_reward;
    bool set_appeared;
    
    for( int i = 0; i != number_of_cues; ++i )
    {
        set_centre = CVec2D(0.0, 0.0);
        set_timer = 0;
        set_reward_radius = food_particles;
        set_status = false;
        set_reliability = 1.0f;
        
        set_is_reward = false;
        set_appeared = false;
        
        CS[i].Setup(set_centre, set_timer, set_reward_radius, set_status, set_reliability, set_is_reward);
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

bool CalculateGroupProperties(CVec2D& centroid, CVec2D& polarisation)
{
    centroid.x = 0.0; centroid.y = 0.0;
    polarisation.x = 0.0; polarisation.y = 0.0;
    
    // need to see if the group is cohesive first - return FALSE is it has split
    EquivalenceClasses();
    int group_size = 0;
    
    int equivalence_class = 0;
    equivalence_class = agent[0].equivalence_class;
    
    for(int i = 0; i != total_agents; ++i)
    {
        centroid += agent[i].r_centre;
        polarisation += agent[i].direction;
        ++group_size;
    }
    centroid /= total_agents;
    polarisation /= total_agents;
    
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
    // Colours vector
    Scalar colours[8] = {Scalar(0, 0, 213), Scalar(0, 152, 255) , Scalar(0, 234, 174), Scalar(245, 165, 66), Scalar(255, 77, 124), Scalar(176, 39, 156), Scalar(167, 151,0), Scalar(0, 255, 255)};
    
    // Draw arena
    Mat visualisation = Mat::zeros(arena_size, arena_size, CV_8UC3);
    circle(visualisation, Point(arena_centre.x, arena_centre.y), arena_size / 2, Scalar(94, 73, 52), -1);
    
    // Draw cues
    for (int i = 0; i != number_of_cues; ++i)
    {
        if (CS[i].status)
        {
            if (CS[i].is_reward)
            {
                circle(visualisation, Point(CS[i].centre.x, CS[i].centre.y), 2, colours[i], -1, CV_AA);
                circle(visualisation, Point(CS[i].centre.x, CS[i].centre.y), cue_range, colours[i], 1, CV_AA);
                circle(visualisation, Point(CS[i].centre.x, CS[i].centre.y), reward_zone, colours[i], 1, CV_AA);
            }
        }
    }
    CVec2D centroid;
    CVec2D gdir;
    CVec2D perp;
    CalculateGroupProperties(centroid, gdir);
    
    // Draw agents
    for (int i = 0; i != total_agents; ++i)
    {
        double rad = agent[i].FrontBackDistance(centroid, gdir);
        if (rad > 0.0) circle(visualisation, Point(agent[i].r_centre.x, agent[i].r_centre.y), rad, Scalar(225, 225, 0));
        else circle(visualisation, Point(agent[i].r_centre.x, agent[i].r_centre.y), -rad, Scalar(0, 225, 225));
        
        if (agent[i].speed > speed) circle(visualisation, Point(agent[i].r_centre.x, agent[i].r_centre.y), 2, Scalar(182, 89, 155), -1, CV_AA);
        else circle(visualisation, Point(agent[i].r_centre.x, agent[i].r_centre.y), 2, Scalar(241, 240, 236), -1, CV_AA);
    }
    
    // Draw group direction and perpendicular vector
    perp = gdir;
    perp.rotate(90.0);
    line(visualisation, Point(centroid.x, centroid.y), Point(centroid.x + 50 * gdir.x, centroid.y + 50 * gdir.y), Scalar(0, 0, 255));
    line(visualisation, Point(centroid.x - 50 * perp.x, centroid.y - 50 * perp.y), Point(centroid.x + 50 * perp.x, centroid.y + 50 * perp.y), Scalar(255, 255, 255));
    
    // Display timestep number & cue counter on screen
    putText(visualisation, to_string(timestep_number), cvPoint(10,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
    
    imshow("decision_making", visualisation);
    waitKey(1);
}

void GraphicsWriter(VideoWriter& video_writer, int& timestep_number, const int& num_timesteps)
{
    // Colours vector
    Scalar colours[8] = {Scalar(0, 0, 213), Scalar(0, 152, 255) , Scalar(0, 234, 174), Scalar(245, 165, 66), Scalar(255, 77, 124), Scalar(176, 39, 156), Scalar(167, 151,0), Scalar(0, 255, 255)};
    
    // Draw arena
    Mat visualisation = Mat::zeros(arena_size, arena_size, CV_8UC3);
    circle(visualisation, Point(arena_centre.x, arena_centre.y), arena_size / 2, Scalar(94, 73, 52), -1);
    
    // Draw cues
    for (int i = 0; i != number_of_cues; ++i)
    {
        if (CS[i].status)
        {
            if (CS[i].is_reward)
            {
                circle(visualisation, Point(CS[i].centre.x, CS[i].centre.y), reward_zone, colours[i], -1, CV_AA);
            }
        }
    }
    
    // Draw agents
    for (int i = 0; i != total_agents; ++i)
    {
        if (agent[i].omega > omega) circle(visualisation, Point(agent[i].r_centre.x, agent[i].r_centre.y), 2, Scalar(0, 152, 255), -1, CV_AA);
        else circle(visualisation, Point(agent[i].r_centre.x, agent[i].r_centre.y), 2, Scalar(113, 204, 46), -1, CV_AA);
    }
    
    // Display timestep number & cue counter on screen
    putText(visualisation, to_string(timestep_number), Point(10, 30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(200, 200, 250), 1, CV_AA);
    
    // Write video
    video_writer.write(visualisation); //writer the frame into the file
    
    if (timestep_number == num_timesteps)
    {
        video_writer.release();
    }
    waitKey(1);
}
