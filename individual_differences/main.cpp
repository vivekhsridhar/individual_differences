//
//  main.cpp
//  individual_differences:
//
//  Created by Vivek Hari Sridhar on 10/08/16.
//  Copyright © 2016 Vivek Hari Sridhar. All rights reserved.
//

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

//**************************************************************************************************
//**	MAIN	************************************************************************************
//**************************************************************************************************

int main()
{
    // Random generator engine from a time-based seed
    unsigned seed = static_cast<unsigned>(std::chrono::high_resolution_clock::now().time_since_epoch().count());
    rnd::set_seed(seed);
    
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
    
    total_agents = 2;
    angular_error_sd = 0.02;
    max_turning_rate = 60.0;
    speed = 1.0;
    ssd = 0.1;
    omega = 0.01;
    osd = 0.01;
    zod = 1.0;
    zoo = 36.0;
    zop = 1000000.0;
    alpha = 90.0;
    
    agent = new individual[total_agents];
    
    int num_replicates = 400;
    int num_timesteps = 30000;
    
    CVec2D centroid, polarisation;
    
    // Open output file
    std::string filename_gp;
    filename_gp = "sociability_assay.csv";
    outputFile1.open(filename_gp.c_str());
    
    // Output file headers
    outputFile1 << "side"  << ", " << "agent speed" << ", " << "agent omega" << ", " << "position.x" << ", " << "position.y" << ", " << "replicate" << ", " << "direction.x" << ", " << "direction.y" << "\n";
    
    Size S(static_cast<int>(arena_size), static_cast<int>(arena_size));
    
    for (speed = 2.5; speed >= 0.5; )
    {
        std::cout << "\n" << speed << "\n";
        for (omega = 0.01; omega <= 0.1; )
        {
            std::cout << omega << " ";
            for (int i = 0; i != num_replicates; ++i)
            {
                SetupSimulation();
                
                for(int j = 0; j != num_timesteps; ++j)
                {
                    MoveAgents();
                    
                    CalculateGroupProperties(centroid, polarisation);
                    //if (j % 20 == 0) Graphics(centroid);
                    
                    ++timestep_number;
                    
                    if (j > 20000 && j % 1000 == 0)
                    {
                        outputFile1 << side  << ", " << agent[total_agents - 1].speed << ", " << agent[total_agents - 1].omega << ", " << agent[total_agents - 1].r_centre.x << ", " << agent[total_agents - 1].r_centre.y << ", " << i << ", " << agent[total_agents - 1].direction.x << ", " << agent[total_agents - 1].direction.y << "\n";
                    }
                }
            }
            
            omega += 0.02;
        }
        
        speed -= 0.25;
    }
    
    echo("simulation end");
    return 0;
}

//**************************************************************************************************
//**    OTHER GROUP LEVEL FUNCTIONS ****************************************************************
//**************************************************************************************************

void MoveAgents()
{
    CalculateSocialForces();
    double dev_angle = 360.0f * normal(0.0, angular_error_sd);
    
    for(int i = 0; i != total_agents; ++i)
    {
        agent[i].AddPersonalPreference();                                                           // inertia driven personal preference in movement direction
        agent[i].Move(timestep_inc, arena_centre, bottom_right, dev_angle, side, compartment_size); // move based on gathered desired direction
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
            temp_vector = (agent[j].r_centre - agent[i].r_centre);
            
            if(temp_vector.x * temp_vector.x > zop_length || temp_vector.y * temp_vector.y > zop_length)
            {
                // cannot be close enough to interact with
            }
            else
            {
                dist = temp_vector.length() * temp_vector.length();
                
                if(dist < zod_length)
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
    
    // now have total_zod, total_zoo and total_zop calculated for all individuals
    for(int k = 0; k != total_agents; ++k)
    {
        if(agent[k].zod_count > 0)
        {
            if(fabs(agent[k].total_zod.x) < FLT_EPSILON && fabs(agent[k].total_zod.y) < FLT_EPSILON)
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
                // do a correlated random walk
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
                // do a correlated random walk
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
                // do a correlated random walk
                agent[k].desired_direction = agent[k].direction;
            }
            else
            {
                agent[k].desired_direction = (agent[k].total_zoo.normalise() + agent[k].total_zop.normalise()).normalise();
            }
        }
    }
}

void SetupSimulation()
{
    timestep_number = 0;
    SetupAgents();
}

void SetupAgents()
{
    CVec2D set_r_centre;
    double set_omega;
    double set_speed;
    bool   set_test_fish;
    
    double omega_mean = 0.1f;
    double speed_mean = 1.0f;
    
    side = (bool) rnd::integer(2);                          // false - left; true - right
    
    for(int i = 0; i != total_agents; ++i)
    {
        CVec2D set_direction(1.0, 0.0);
        set_direction.rotate(uniform() * 360.0);
        
        if (i < total_agents - 1)
        {
            // parameters for stimulus fish
            set_omega = rnd::normal(omega_mean, osd);
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
            set_speed = rnd::normal(speed, ssd);
            if (set_omega < 0.0) set_omega = 0.0;
            if (set_speed < 0.0) set_speed = 0.0;
            set_test_fish = true;
            set_r_centre = RandomBoundedPoint(side, set_test_fish);
        }
        agent[i].Setup(set_r_centre, set_direction, max_turning_rate, set_speed, zod, zoo, zop, angular_error_sd, set_omega, set_test_fish);
    }
}

CVec2D RandomBoundedPoint(bool& side, bool& test_fish)
{
    CVec2D random_point;
    if (!test_fish)
    {
        // create randomly distributed co-ordinates in the simulated world
        double range_x = (double) (compartment_size - top_left.x);
        double range_y = (double) (bottom_right.y - top_left.y);
        
        double random_x = uniform();
        double random_y = uniform();
        
        // individuals start in the middle 100th of their world
        random_x *= (range_x / 100.0f);
        random_y *= (range_y / 100.0f);
        
        // stimulus fish start in the left or right compartment based on the boolean 'side'
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

//**************************************************************************************************
//**	OUTPUT FUNCTIONS    ************************************************************************
//**************************************************************************************************

void CalculateGroupProperties(CVec2D& centroid, CVec2D& polarisation)
{
    centroid.x = 0.0; centroid.y = 0.0;
    polarisation.x = 0.0; polarisation.y = 0.0;
    
    for(int i = 0; i != total_agents; ++i)
    {
        centroid += agent[i].r_centre;
        polarisation += (agent[i].direction.normalise());
    }
    centroid /= total_agents;
    polarisation /= total_agents;
}

//**************************************************************************************************
//**	GRAPHICS    ********************************************************************************
//**************************************************************************************************

void Graphics(CVec2D& centroid)
{
    // draw arena
    Mat visualisation = Mat::zeros(arena_size, arena_size + 2 * compartment_size, CV_8UC3);
    
    // draw compartments
    line(visualisation, Point(compartment_size, 0), Point(compartment_size, arena_size), Scalar(255, 255, 255));
    line(visualisation, Point(arena_size + compartment_size, 0), Point(arena_size + compartment_size, arena_size), Scalar(255, 255, 255));
    
    // draw agents
    for (int i = 0; i != total_agents; ++i)
    {
        if (i < total_agents - 1) circle(visualisation, Point(agent[i].r_centre.x, agent[i].r_centre.y), 2, Scalar(0, 152, 255), -1, CV_AA);
        else circle(visualisation, Point(agent[i].r_centre.x, agent[i].r_centre.y), 2, Scalar(241, 240, 236), -1, CV_AA);
    }
    
    circle(visualisation, Point(centroid.x, centroid.y), sqrt(zop), Scalar(156, 188, 26), 1, CV_AA);
    
    // display timestep number
    putText(visualisation, to_string(timestep_number), cvPoint(10,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
    
    imshow("individual_differences", visualisation);
    waitKey(1);
}


