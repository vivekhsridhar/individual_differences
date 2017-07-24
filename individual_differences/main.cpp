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
    arena_size = 1000;
    top_left.x = 0.0;
    top_left.y = 0.0;
    bottom_right.x = arena_size;
    bottom_right.y = arena_size;
    
    total_agents = 20;
    angular_error_sd = 0.02;
    max_turning_rate = 60.0;
    speed = 1.0;
    ssd = 0.1;
    omega = 1.0;
    osd = 0.01;
    zod = 1.0;
    zoo = 36.0;
    zop = 225.0;
    alpha = 90.0;
    iid = 0.0;
    
    agent = new individual[total_agents];
    
    int num_replicates = 400;
    int num_timesteps = 20000;
    
    CVec2D centroid, polarisation;
    
    // Open output file
    std::string filename_gp;
    filename_gp = "group_properties.csv";
    outputFile1.open(filename_gp.c_str());
    
    std::string filename_rp;
    filename_rp = "relative_positions.csv";
    outputFile2.open(filename_rp.c_str());
    
    // Output file headers
    outputFile1 << "calculated mean speed" << ", " << "polarisation" << ", " << "iid" << "\n";
    outputFile2 << "agent omega" << ", " << "agent speed" << ", " << "mean omega" << ", " << "mean speed" << ", " << "omega rank" << ", " << "speed rank" << ", " << "front back distance" << ", " << "centroid distance" << ", " << "nnd" << ", " << "timestep number" << ", " << "replicate" << ", " << "position x" << ", " << "position y" << ", " << "direction x" << ", " << "direction y" << "\n";
    
    Size S(static_cast<int>(arena_size), static_cast<int>(arena_size));
    
    for (speed = 1.0; speed >= 0.1; )
    {
        std::cout << "\n" << speed << "\n";
        for (omega = 0.01; omega <= 0.1; )
        {
            std::cout << omega << " ";
            for (int i = 0; i != num_replicates; ++i)
            {
                // setup arena and agents
                SetupSimulation();
                bool result = 0;
                
                // calculate each agent's ranked omega and speed within its group
                double average_omega = 0.0;
                double average_speed = 0.0;
                double omega_rank[total_agents];
                double speed_rank[total_agents];
                for (int n = 0; n != total_agents; ++n)
                {
                    average_omega += agent[n].omega;
                    average_speed += agent[n].speed;
                    omega_rank[n] = agent[n].omega;
                    speed_rank[n] = agent[n].speed;
                }
                average_omega /= total_agents;
                average_speed /= total_agents;
                
                sort(omega_rank, omega_rank + total_agents);
                sort(speed_rank, speed_rank + total_agents);
                
                for (int m = 0; m != total_agents; ++m) // m goes through sorted rank arrays
                {
                    for (int n = 0; n != total_agents; ++n) // n goes through agents
                    {
                        if (omega_rank[m] == agent[n].omega) agent[n].omega_rank = total_agents - m;
                        if (speed_rank[m] == agent[n].speed) agent[n].speed_rank = total_agents - m;
                    }
                }
                
                for(int j = 0; j != num_timesteps; ++j)
                {
                    // all agent movement happens within this function
                    MoveAgents();
                    
                    //if (j % 20 == 0) Graphics();
                    
                    ++timestep_number;
                    
                    if (j % 2000 == 0)
                    {
                        result = GroupTogether();
                        if (result)
                        {
                            CalculateGroupProperties(centroid, polarisation);
                            NearestNeighbourDistance();
                            
                            if (centroid.x > 5.0 && centroid.y > 5.0 && centroid.x < arena_size - 5.0 && centroid.y < arena_size - 5.0) // ignore output if group may be across boundaries (periodic boundary conditions)
                            {
                                for (int a = 0; a != total_agents; ++a)
                                {
                                    outputFile2 << agent[a].omega << ", " << agent[a].speed << ", " << omega << ", " << speed << ", " << agent[a].omega_rank << ", " << agent[a].speed_rank << ", " << agent[a].FrontBackDistance(centroid, polarisation) << ", " << (agent[a].r_centre - centroid).length() << ", " << agent[a].nnd << ", " << j << ", " << i << ", " << agent[a].r_centre.x << ", " << agent[a].r_centre.y << ", " << agent[a].direction.x << ", " << agent[a].direction.y<< "\n";
                                }
                                
                                outputFile1 << average_speed << ", " << polarisation.length() << ", " << iid << "\n";
                            }
                        }
                    }
                }
            }
            
            omega += 0.01;
        }
        
        speed -= 0.1;
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
        agent[i].AddPersonalPreference();                           // add agent's goal orientedness with omega
        agent[i].Move(timestep_inc, arena_size, dev_angle);         // move based on gathered desired direction
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
            
            // periodic boundary
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
    
    for(int i = 0; i != total_agents; ++i)
    {
        CVec2D set_direction(1.0, 0.0);
        set_direction.rotate(uniform() * 360.0);
        set_r_centre = RandomBoundedPoint();
        set_omega = rnd::normal(omega, osd);
        set_speed = rnd::normal(speed, ssd);
        if (set_omega < 0.0) set_omega = 0.0;
        if (set_speed < 0.0) set_speed = 0.0;
        
        agent[i].Setup(set_r_centre, set_direction, max_turning_rate, set_speed, zod, zoo, zop, angular_error_sd, set_omega);
    }
}

CVec2D RandomBoundedPoint()
{
    // create randomly distributed co-ordinates in the simulated world
    double range_x = (double) (bottom_right.x - top_left.x);
    double range_y = (double) (bottom_right.y - top_left.y);
    
    double random_x = uniform();
    double random_y = uniform();
    
    // individuals are initialised in the middle 100th of their world
    random_x *= (range_x / 100.0f);
    random_y *= (range_y / 100.0f);
    random_x += (double) (range_x / 2.0f - range_x / 200.0f);
    random_y += (double) (range_y / 2.0f - range_y / 200.0f);
    CVec2D random_point(random_x, random_y);
    
    return random_point;
}

//**************************************************************************************************
//**	OUTPUT FUNCTIONS    ************************************************************************
//**************************************************************************************************

void NearestNeighbourDistance()
{
    iid = 0.0;
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
        
        agent[i].nnd = sqrt(tmp);
    }
    
    // iid equals (1 + 2 + 3 + ... + (n - 2) + (n - 1)) * 2
    iid /= (total_agents * (total_agents - 1));
}

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

// Group Together(), Equivalent() and EquivalenceClasses() together calculate if group is cohesive
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
    
    dist1 = agent1.r_centre.distanceTo(agent2.r_centre);
    
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

//**************************************************************************************************
//**	GRAPHICS    ********************************************************************************
//**************************************************************************************************

void Graphics()
{
    // draw arena
    Mat visualisation = Mat::zeros(arena_size, arena_size, CV_8UC3);
    
    // draw agents
    for (int i = 0; i != total_agents; ++i)
    {
        // agents faster than the mean speed are orange in colour
        if (agent[i].speed > speed) circle(visualisation, Point(agent[i].r_centre.x, agent[i].r_centre.y), 2, Scalar(0, 152, 255), -1, CV_AA);
        // agents slower than the mean are white
        else circle(visualisation, Point(agent[i].r_centre.x, agent[i].r_centre.y), 2, Scalar(241, 240, 236), -1, CV_AA);
        
        circle(visualisation, Point(agent[i].r_centre.x, agent[i].r_centre.y), sqrt(zop), Scalar(241, 240, 0));
    }
    
    // display timestep number
    putText(visualisation, to_string(timestep_number), cvPoint(10,30), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
    
    imshow("decision_making", visualisation);
    waitKey(1);
}

