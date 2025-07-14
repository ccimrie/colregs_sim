#include "box2d/box2d.h"
#include "yaml-cpp/yaml.h"
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <string.h>
#include <fstream>
#include <stdlib.h>
// #include <agent.h>
#include <normal_agent.h>
#include <mass_agent.h>
// #include "agent_quad.h"
// #include "quad_tree.h"

#define PI 3.14159265
#define THRESH 0.25

double betaVary=0.01425;
double dense=1.0;

b2Body* defineBody(b2Body* particle, double radius, double density, double friction)
{
  // Define another box shape for our dynamic body.
  b2CircleShape dynamicCirc;
  dynamicCirc.m_radius=radius;

  // Define the dynamic body fixture.
  b2FixtureDef fixtureDef;
  fixtureDef.shape = &dynamicCirc;

  // Set the box density to be non-zero, so it will be dynamic.
  fixtureDef.density = density;

  // Override the default friction.
  fixtureDef.friction = friction;

  particle->CreateFixture(&fixtureDef);
  return particle;
}

using namespace std;

int main(int argc, const char* argv[])
{  
  // Parameters from input args
    string yaml_file(argv[1]);
    YAML::Node config = YAML::LoadFile(yaml_file);
    const int TT=config["TT"].as<int>();
    const string results_dir=config["results directory"].as<string>();

    B2_NOT_USED(argc);
    B2_NOT_USED(argv);

  // Set up simulated world/environment
    // Define the gravity vector.
    b2Vec2 gravity(0.0f, 0.0f);

    // Construct a world object, which will hold and simulate the rigid bodies.
    b2World world(gravity);

   // Define the dynamic body. We set its position and call the body factory.
    std::vector<std::vector<NormalAgent>> MASS; // Particles movement/dynamics in physical space
    std::vector<int> MASS_pops;
    std::vector<string> MASS_keys;
    std::uniform_real_distribution<double> distribution_seeds(0, 10000000);
    srand48(time(NULL));
    std::default_random_engine gen;
    gen.seed(time(NULL));

    YAML::Node agents=config["agents"];
    
    printf("Loaded scenario config\n");
    printf("Creating agents\n");
    for(YAML::const_iterator it=agents.begin(); it!=agents.end(); ++it)
    {
      std::string key=it->first.as<std::string>();         // <- key
      MASS_keys.push_back(key);
      // cTypeList.push_back(it->second.as<CharacterType>()); // <- value
      int N=config["agents"][key]["N"].as<int>();
      string agent_yaml_file=config["agents"][key]["yaml file"].as<string>();
      MASS.push_back(std::vector<NormalAgent>(0));
      for (int n=0; n<N; ++n)
      {
          string filename=key+"_"+to_string(n)+".txt";
          string results_file=results_dir+filename;
          double seed=distribution_seeds(gen);
          MASS.back().push_back(NormalAgent(agent_yaml_file, results_file, seed));
          // printf("Created agent\n");
          // agent temp_agent(agent_yaml_file, results_file);
          b2BodyDef temp_body=MASS.back().back().getBodyDef();
          // printf("Created temp body\n");
          MASS.back().back().setBody(world.CreateBody(&temp_body));
          // printf("Created body with radius:  %f\n", MASS.back().getRadius());
          MASS.back().back().setBody(defineBody(MASS.back().back().getBody(), MASS.back().back().getRadius(), 40, 0.3));
      }
      // MASS.push_back(temp_MASS);
      MASS_pops.push_back(N);
    }
   // Define simulation parameters
    // Prepare for simulation. Typically we use a time step of 1/60 of a
    // second (60Hz) and 10 iterations. This provides a high quality simulation
    // in most game scenarios.
    float_t timeStep = 1.0f / 60.0f;
    int32 velocityIterations = 6;
    int32 positionIterations = 2;

    std::vector<int> neighbours;
    std::vector<b2Body*> agent_bodies_temp;
    std::vector<int> neigh_ind;

    int overall_counter=0;

   // Run simulation
   
    for (int32 t=0; t<TT; ++t)
    {
        if (t%500==0) printf("%i\n", t);
        
        // For each mass 
        // 1. Check distance
        // 2. Update velocity
        // 3. Record New position

       // Calculate new velocity commands
        // for (int n=0; n<N; ++n)
        vector<vector<NormalAgent>>::iterator it_pops=MASS.begin();
        int pop_counter=0;
        for(it_pops; it_pops!=MASS.end(); ++it_pops)
        {
         // Repopulate agents if some have been removed
          while(it_pops->size()<MASS_pops[pop_counter])
          {
            // string filename=key+"_"+to_string(MASS_pops[pop_counter][1])+".txt";
            std::string key=MASS_keys[pop_counter];         // <- key
            string agent_yaml_file=config["agents"][key]["yaml file"].as<string>();
            string filename="mass_"+to_string(overall_counter)+".txt";
            ++overall_counter;
            // ++MASS_pops[pop_counter][1];
            string results_file=results_dir+filename;
            double seed=distribution_seeds(gen);
            it_pops->push_back(NormalAgent(agent_yaml_file, results_file, seed));
            b2BodyDef temp_body=it_pops->back().getBodyDef();
            it_pops->back().setBody(world.CreateBody(&temp_body));
            it_pops->back().setBody(defineBody(it_pops->back().getBody(), it_pops->back().getRadius(), 40, 0.3));
          }
          ++pop_counter;

          // vector<agent>::iterator it_ego=MASS.begin();
          vector<NormalAgent>::iterator it_ego=it_pops->begin();
          while(it_ego!=it_pops->end())
          {
            // b2Vec2 lin_velocity;
            // double ang_velocity;
            bool reached_goal=it_ego->checkGoal();

            if(reached_goal) 
            {
              it_pops->erase(it_ego);
            }
            else
            { 
              double pos_x=it_ego->getBody()->GetPosition().x;
              double pos_y=it_ego->getBody()->GetPosition().y;

              double MASS_range=it_ego->getRange();
              double vel_mag=it_ego->getMaxVel();

              vector<vector<NormalAgent>>::iterator it_pops_neigh=MASS.begin();
              for(it_pops_neigh; it_pops_neigh!=MASS.end(); ++it_pops_neigh)
              {
                for (vector<NormalAgent>::iterator it_neigh=it_pops_neigh->begin(); it_neigh!=it_pops_neigh->end(); ++it_neigh)
                {
                  if (it_neigh!=it_ego)
                  {
                    double other_pos_x=it_neigh->getBody()->GetPosition().x;
                    double other_pos_y=it_neigh->getBody()->GetPosition().y;
                    double other_vel_mag=it_neigh->getVelMag();
                    // double other_radius=it_neigh->getRadius();
                    // double other_theta=it_neigh->getBody() ->GetAngle( )*(PI/180);
                    double dist_x=pos_x-other_pos_x;
                    double dist_y=pos_y-other_pos_y;
                    double dist=sqrt(dist_x*dist_x+dist_y*dist_y);
                    double closest_dist_point=dist-it_neigh->getRadius();
                    // if (closest_dist_point<MASS_range) it_ego->updateVel(other_pos_x-pos_x, other_pos_y-pos_y, other_vel_mag, dist, other_radius, other_theta);
                    if (closest_dist_point<MASS_range) it_ego->updateVel(&*it_neigh);
                  }
                }
              }
              ++it_ego;
            }
          }
        }
 
       // Update with new velocity commands
        it_pops=MASS.begin();
        for(it_pops; it_pops!=MASS.end(); ++it_pops)
        {
          for(vector<NormalAgent>::iterator it_ego=it_pops->begin(); it_ego!=it_pops->end(); ++it_ego)
          {
            // Update the robot's heading
            // Update robot's velocity
            it_ego->updateVelMag();
            it_ego->updateTheta();

            double ang_velocity=(it_ego->getThetaAcc()*180.0)/PI;
            b2Vec2 lin_velocity;
            lin_velocity.Set(it_ego->getVelX(), it_ego->getVelY());
            
            it_ego->getBody()->SetLinearVelocity(lin_velocity);
            it_ego->getBody()->SetAngularVelocity(ang_velocity);
          }
        }

        // Instruct the world to perform a single step of simulation.
        // It is generally best to keep the time step and iterations fixed.
        world.Step(timeStep, velocityIterations, positionIterations);

        // Now print the position and angle of the body.
        it_pops=MASS.begin();
        for(it_pops; it_pops!=MASS.end(); ++it_pops)
        {
          for(vector<NormalAgent>::iterator it_ego=it_pops->begin(); it_ego!=it_pops->end(); ++it_ego) it_ego->recordStep(t);
        }
    }
   
    // When the world destructor is called, all bodies and joints are freed. This can
    // create orphaned pointers, so be careful about your world management.

    return 0;

}