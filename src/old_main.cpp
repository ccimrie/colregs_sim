#include "box2d/box2d.h"
#include "yaml-cpp/yaml.h"
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <string.h>
#include <fstream>
#include <stdlib.h> 
#include <agent.h>
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
    printf("loaded config\n");
    // string robot_file_small="../colregs_sim/results/MASS[n]_test.txt";
    // string robot_file_big="../colregs_sim/results/large_mass_test.txt";
    double TOP, RIGHT, BOTTOM, LEFT=0.0;
    const int TT=config["TT"].as<int>();
    const int N=3*config["N"].as<int>();
    const double max_size=config["max size"].as<double>();
    const double min_size=config["min size"].as<double>();
    const double world_size_x=config["world size x"].as<double>();
    const double world_size_y=config["world size y"].as<double>();
    const double vel_size_ratio=config["vel-size ratio"].as<double>();
    const double range_size_ratio=config["range-size ratio"].as<double>();
    const string results_dir=config["results directory"].as<string>();
    
    // TOP=world_size;
    // RIGHT=world_size;
    // BOTTOM=-world_size;
    // LEFT=-world_size;

    B2_NOT_USED(argc);
    B2_NOT_USED(argv);

  // Set up simulated world/environment

    // Define the gravity vector.
    b2Vec2 gravity(0.0f, 0.0f);

    // Construct a world object, which will hold and simulate the rigid bodies.
    b2World world(gravity);

    double target_offset=1.5;

    double goal_tolerance=4;

   // Define the dynamic body. We set its position and call the body factory.
    srand48(time(NULL));
    std::vector<agent> MASS(N); // Particles movement/dynamics in physical space
    std::default_random_engine gen;
    gen.seed(time(NULL));

    std::uniform_real_distribution<double> distribution_size(min_size,max_size);

    std::uniform_real_distribution<double> distribution_x(-world_size_x,-world_size_x*(2/3));
    std::uniform_real_distribution<double> distribution_y(-world_size_y,world_size_y);
    for (int n=0; n<N/3; ++n)
    // for (int n=0; n<N/2; ++n)
    {
        double r_radius=distribution_size(gen);
        string filename="agent_"+to_string(n)+".txt";
        string results_file=results_dir+filename;
        double max_vel=r_radius*vel_size_ratio;
        double rng=r_radius*range_size_ratio;

        double pos_x=distribution_x(gen);
        double pos_y=distribution_y(gen);

        MASS[n]=agent(r_radius, pos_x, pos_y, 0, max_vel, rng, 0, results_file);
        b2BodyDef temp_body=MASS[n].getBodyDef();
        MASS[n].setBody(world.CreateBody(&temp_body));
        MASS[n].setBody(defineBody(MASS[n].getBody(), MASS[n].getRadius(), 40, 0.3));
        MASS[n].setVelMag(0.2);
        MASS[n].setTarget(world_size_x*target_offset, goal_tolerance, 0, world_size_y);
    }


    std::uniform_real_distribution<double> distribution_o_x(world_size_x*(2/3.0),world_size_x);
    std::uniform_real_distribution<double> distribution_o_y(-world_size_y,world_size_y);
    // std::uniform_real_distribution<double> distribution_o_y(0,0);
    for (int n=N/3; n<(2*N)/3; ++n)
    // for (int n=N/2; n<N; ++n)
    {
        double r_radius=distribution_size(gen);
        string filename="agent_"+to_string(n)+".txt";
        string results_file=results_dir+filename;
        double max_vel=r_radius*vel_size_ratio;
        double rng=r_radius*range_size_ratio;

        double pos_x=distribution_o_x(gen);
        double pos_y=distribution_o_y(gen);

        MASS[n]=agent(r_radius, pos_x, pos_y, 180, max_vel, rng, 1, results_file);
        b2BodyDef temp_body=MASS[n].getBodyDef();
        MASS[n].setBody(world.CreateBody(&temp_body));
        MASS[n].setBody(defineBody(MASS[n].getBody(), MASS[n].getRadius(), 40, 0.3));
        MASS[n].setVelMag(0.2);
        MASS[n].setTarget(-world_size_x*target_offset, goal_tolerance, 0, world_size_y);
    }

    std::uniform_real_distribution<double> distribution_3_x(-world_size_x*(1/3.0),world_size_x*(1/3.0));
    std::uniform_real_distribution<double> distribution_3_y(world_size_y*(3/5.0),world_size_y);
    for (int n=(2*N)/3; n<N; ++n)
    {
        double r_radius=distribution_size(gen);
        string filename="agent_"+to_string(n)+".txt";
        string results_file=results_dir+filename;
        double max_vel=r_radius*vel_size_ratio;
        double rng=r_radius*range_size_ratio;

        double pos_x=distribution_3_x(gen);
        double pos_y=distribution_3_y(gen);

        MASS[n]=agent(r_radius, pos_x, pos_y, 270, max_vel, rng, 2, results_file);
        b2BodyDef temp_body=MASS[n].getBodyDef();
        MASS[n].setBody(world.CreateBody(&temp_body));
        MASS[n].setBody(defineBody(MASS[n].getBody(), MASS[n].getRadius(), 40, 0.3));
        MASS[n].setVelMag(0.2);
        MASS[n].setTarget(0, world_size_x, -world_size_y*target_offset, goal_tolerance);
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

   // Run simulation
    for (int32 t=0; t<TT; ++t)
    {
        if (t%500==0) printf("%i\n", t);
        
        // For each mass 
        // 1. Check distance
        // 2. Update velocity
        // 3. Record New position

       // Calculate new velocity commands
        for (int n=0; n<N; ++n)
        {
          // b2Vec2 lin_velocity;
          // double ang_velocity;
          double pos_x=MASS[n].getBody()->GetPosition().x;
          double pos_y=MASS[n].getBody()->GetPosition().y;
          double MASS_range=MASS[n].getRange();
          double vel_mag=MASS[n].getMaxVel();
          for (int nn=0; nn<N; ++nn)
          {
            if (nn!=n)
            {
              double other_pos_x=MASS[nn].getBody()->GetPosition().x;
              double other_pos_y=MASS[nn].getBody()->GetPosition().y;
              double other_vel_mag=MASS[nn].getVelMag();
              double other_radius=MASS[nn].getRadius();
              double other_theta=MASS[nn].getBody() ->GetAngle()*(PI/180);
              double dist_x=pos_x-other_pos_x;
              double dist_y=pos_y-other_pos_y;
              double dist=sqrt(dist_x*dist_x+dist_y*dist_y);
              double closest_dist_point=dist-MASS[nn].getRadius();
              if (closest_dist_point<MASS_range) MASS[n].updateVel(other_pos_x-pos_x, other_pos_y-pos_y, other_vel_mag, dist, other_radius, other_theta);
            }
          }
        }
 
       // Update with new velocity commands
        for (int n=0; n<N; ++n)
        {
          // Update the robot's heading
          // Update robot's velocity
          MASS[n].updateVelMag();
          MASS[n].updateTheta();

          double ang_velocity=(MASS[n].getThetaAcc()*180.0)/PI;
          b2Vec2 lin_velocity;
          lin_velocity.Set(MASS[n].getVelX(), MASS[n].getVelY());
          
          MASS[n].getBody()->SetLinearVelocity(lin_velocity);
          MASS[n].getBody()->SetAngularVelocity(ang_velocity);
        }

        // Instruct the world to perform a single step of simulation.
        // It is generally best to keep the time step and iterations fixed.
        world.Step(timeStep, velocityIterations, positionIterations);

        // Now print the position and angle of the body.
        for (int n=0; n<N; ++n) MASS[n].recordStep();
        // for (int n=0; n<N; ++n) MASS_O[n].recordStep();
    }

    // When the world destructor is called, all bodies and joints are freed. This can
    // create orphaned pointers, so be careful about your world management.

    return 0;
}