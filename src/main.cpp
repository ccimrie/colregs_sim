#include <box2d/box2d.h>
#include "yaml-cpp/yaml.h"
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <string.h>
#include <fstream>
#include <stdlib.h>
#include <agent.h>
#include <mass_agent.h>
#include <normal_agent.h>
// #include <genetic_algorithm.h>

#define PI 3.14159265

using namespace std;

b2ShapeId defineBody(b2BodyId particle_id, double radius, double density, double friction)
{
  // Define circle-mass for our dynamic body.
  b2Circle dynamic_circ;
  dynamic_circ.center=(b2Vec2){0.0f, 0.0f};
  dynamic_circ.radius=radius;

  b2ShapeDef shapeDef = b2DefaultShapeDef();
  shapeDef.density = density;
  shapeDef.material.friction = friction;

  b2Vec2 position=b2Body_GetPosition(particle_id);
  b2Rot rotation=b2Body_GetRotation(particle_id);

  b2ShapeId particle=b2CreateCircleShape(particle_id, &shapeDef, &dynamic_circ);
  return particle;
}


void setGoal(agent* ego, vector<vector<double>> goals, int selection)
{
  ego->setTarget(goals[selection][0], goals[selection][2], goals[selection][1], goals[selection][2]);
}


void checkNeigh(NormalAgent* ego, agent* neigh)
{
  b2Vec2 position=b2Body_GetPosition(ego->getBodyID());
  double pos_x=position.x;
  double pos_y=position.y;
  double colregs_range=ego->getRange();
  double vel_mag=ego->getMaxVel();

  b2Vec2 neigh_position=b2Body_GetPosition(neigh->getBodyID());
  double other_pos_x=neigh_position.x;
  double other_pos_y=neigh_position.y;
  double other_vel_mag=neigh->getVelMag();

  double dist_x=pos_x-other_pos_x;
  double dist_y=pos_y-other_pos_y;
  double dist=sqrt(dist_x*dist_x+dist_y*dist_y);
  double closest_dist_point=dist-neigh->getRadius();
  if (closest_dist_point<colregs_range)
  {
    ego->updateVel(neigh);
    ++ego->no_neigh;
    ego->sum_neigh_dist+=closest_dist_point;
  }
}


void checkNeighMass(MassAgent* ego, agent* neigh)
{
  b2Vec2 position=b2Body_GetPosition(ego->getBodyID());
  double pos_x=position.x;
  double pos_y=position.y;
  double mass_range=ego->getRange();
  double vel_mag=ego->getMaxVel();

  b2Vec2 neigh_position=b2Body_GetPosition(neigh->getBodyID());
  double other_pos_x=neigh_position.x;
  double other_pos_y=neigh_position.y;
  double other_vel_mag=neigh->getVelMag();

  double dist_x=pos_x-other_pos_x;
  double dist_y=pos_y-other_pos_y;
  double dist=sqrt(dist_x*dist_x+dist_y*dist_y);
  double closest_dist_point=dist-neigh->getRadius();
  if (closest_dist_point<mass_range)
  {
    ego->updateNeighPF(neigh);
    ++ego->no_neigh;
    ego->sum_neigh_dist+=closest_dist_point;
  }
}

int main(int argc, const char* argv[])
{
   // Parameters from input args
    string yaml_file(argv[1]);
    YAML::Node config = YAML::LoadFile(yaml_file);
    printf("Loaded scenario config\n");
    string goal_yaml_file=config["goal file"].as<string>();
    YAML::Node goal_config=YAML::LoadFile(goal_yaml_file);
    printf("Loaded goal locations config\n");

    const int TT=config["TT"].as<int>();
    const string results_dir=config["results directory"].as<string>();

    bool lane_setup=config["lanes setup"].as<bool>();

   // RNG setup
    std::uniform_real_distribution<double> distribution_seeds(0, 10000000);
    srand48(time(NULL));
    std::default_random_engine gen;
    gen.seed(time(NULL));

   // Goal locations setup
    vector<vector<double>> goal_locations;

    YAML::Node goals=goal_config["goals"];
    for(YAML::const_iterator it=goals.begin(); it!=goals.end(); ++it)
    {
      std::string key=it->first.as<std::string>();
      double goal_x=goal_config["goals"][key]["x"].as<double>();
      double goal_y=goal_config["goals"][key]["y"].as<double>();
      double tolerance=goal_config["goals"][key]["tolerance"].as<double>();
      vector<double> new_goal={goal_x, goal_y, tolerance};
      goal_locations.push_back(new_goal);
    }
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<std::mt19937::result_type> choice(0,goal_locations.size()-1);

   // Set up simulated world/environment
    // Define the gravity vector.
    b2Vec2 gravity={0.0f, 0.0f};

    // Construct a world object, which will hold and simulate the rigid bodies.
    b2WorldDef worldDef = b2DefaultWorldDef();
    worldDef.gravity=gravity;
    b2WorldId worldID=b2CreateWorld(&worldDef);


   // Create COLREG agents
    std::vector<NormalAgent> colregs;
    std::vector<int> colregs_pops;
    std::vector<string> colregs_keys;

    YAML::Node agents=config["agents"];
    for(YAML::const_iterator it=agents.begin(); it!=agents.end(); ++it)
    {
      std::string key=it->first.as<std::string>();         // <- key
      colregs_keys.push_back(key);
      // cTypeList.push_back(it->second.as<CharacterType>()); // <- value
      int N=config["agents"][key]["N"].as<int>();
      string agent_yaml_file=config["agents"][key]["yaml file"].as<string>();
      for (int n=0; n<N; ++n)
      {
        string filename=key+"_"+to_string(n)+".txt";
        string results_file=results_dir+filename;
        double seed=distribution_seeds(gen);
        colregs.push_back(NormalAgent(agent_yaml_file, results_file, seed));


        setGoal(&colregs.back(), goal_locations, choice(dev));

        double init_x, init_y, init_theta;

        if (lane_setup)
        {
          init_x=colregs.back().targ_x*-1+(1-2*drand48())*(colregs.back().var_x*10);
          init_y=colregs.back().targ_y+(1-2*drand48())*(colregs.back().var_y);
          // init_theta=(1-2*drand48())*PI;
          init_theta=atan2(colregs.back().targ_y, colregs.back().targ_x)*180.0/PI;
        }
        else
        {
          int start_loc=choice(dev);
          init_x=goal_locations[start_loc][0]+(1-2*drand48())*goal_locations[start_loc][2];
          init_y=goal_locations[start_loc][1]+(1-2*drand48())*goal_locations[start_loc][2];
          init_theta=(1-2*drand48())*180.0;
        }

        // printf("CREATING BODY: %f   %f\n", colregs.back().getBodyDef().position.x, colregs.back().getBodyDef().position.y);
        colregs.back().setBodyDefPose(init_x, init_y, init_theta);
        // b2BodyDef temp_body=colregs.back().getBodyDef();
        printf("DEFINING BODY\n");
        colregs.back().setBodyID(b2CreateBody(worldID, colregs.back().getBodyDef()));
        printf("BODY SET\n");
        // colregs.back().setBodyID(defineBody(colregs.back().getBody(), colregs.back().getRadius(), 40, 0.3));
        defineBody(colregs.back().getBodyID(), colregs.back().getRadius(), 40, 0.3);
      }
      colregs_pops.push_back(N);
    }
    printf("COLREG agents instantiated\n");

   // Create mass agent(s)
    std::vector<MassAgent> MASS;
    std::vector<int> mass_pops;
    std::vector<string> mass_keys;
    agents=config["mass agents"];
    for(YAML::const_iterator it=agents.begin(); it!=agents.end(); ++it)
    {
      std::string key=it->first.as<std::string>();         // <- key
      mass_keys.push_back(key);
      int N=config["mass agents"][key]["N"].as<int>();
      double goal_weight=config["mass agents"][key]["goal field weight"].as<double>();
      double neighbour_weight=config["mass agents"][key]["neighbour field weight"].as<double>();
      double safety_bubble=config["mass agents"][key]["safety bubble"].as<double>();
      string agent_yaml_file=config["mass agents"][key]["yaml file"].as<string>();
      for (int n=0; n<N; ++n)
      {
        string filename=key+"_"+to_string(n)+".txt";
        string results_file=results_dir+filename;
        double seed=distribution_seeds(gen);
        MASS.push_back(MassAgent(agent_yaml_file, results_file, seed, goal_weight, neighbour_weight, safety_bubble));

        int start_loc=choice(dev);

        double init_x=goal_locations[start_loc][0]+(1-2*drand48())*goal_locations[start_loc][2];
        double init_y=goal_locations[start_loc][1]+(1-2*drand48())*goal_locations[start_loc][2];
        double init_theta=(1-2*drand48())*PI;

        MASS.back().setBodyDefPose(init_x, init_y, init_theta);

        setGoal(&MASS.back(), goal_locations, choice(dev));

        // b2BodyDef temp_body=MASS.back().getBodyDef();
        MASS.back().setBodyID(b2CreateBody(worldID,MASS.back().getBodyDef()));
        // MASS.back().setBody(defineBody(MASS.back().getBody(), MASS.back().getRadius(), 40, 0.3));
        defineBody(MASS.back().getBodyID(), MASS.back().getRadius(), 40, 0.3);
      }
      mass_pops.push_back(N);
    }

   // Define simulation parameters
    // Prepare for simulation. Typically we use a time step of 1/60 of a
    // second (60Hz) and 10 iterations. This provides a high quality simulation
    // in most game scenarios.
    float timeStep = 1.0f / 60.0f;

    /* Substep count: 
      - higher->accuracy
      - lower->performance
    */
    int subStepCount=4;

    std::vector<int> neighbours;
    std::vector<b2BodyId*> agent_bodies_temp;
    std::vector<int> neigh_ind;

    int overall_counter=0;

   // Run simulation 
    for (int t=0; t<TT; ++t)
    {
      if (t%500==0) printf("%i\n", t);
      
      // For each mass 
      // 1. Check distance
      // 2. Update velocity
      // 3. Record New position

     // Calculate new velocity commands
      vector<NormalAgent>::iterator it_ego=colregs.begin();
      for(it_ego; it_ego!=colregs.end(); ++it_ego)
      {
        // bool reached_goal=it_ego->checkGoal();

        if(it_ego->checkGoal()) setGoal(&*it_ego, goal_locations, choice(dev));
        vector<NormalAgent>::iterator it_neigh=colregs.begin();
        for (it_neigh; it_neigh!=colregs.end(); ++it_neigh) if (it_neigh!=it_ego) checkNeigh(&*it_ego, &*it_neigh);
        for (vector<MassAgent>::iterator it_mass=MASS.begin(); it_mass!=MASS.end(); ++it_mass)
        {
          checkNeigh(&*it_ego, &*it_mass);
        }
      }

      for (vector<MassAgent>::iterator it_mass=MASS.begin(); it_mass!=MASS.end(); ++it_mass)
      {
        if (it_mass->checkGoal()) setGoal(&*it_mass, goal_locations, choice(dev));
        
        vector<NormalAgent>::iterator it_neigh=colregs.begin();
        for (it_neigh; it_neigh!=colregs.end(); ++it_neigh) checkNeighMass(&*it_mass, &*it_neigh);

        vector<MassAgent>::iterator it_mass_neigh=MASS.begin();
        for (it_mass_neigh; it_mass_neigh!=MASS.end(); ++it_mass_neigh)
        {
          if(it_mass!=it_mass_neigh) checkNeighMass(&*it_mass, &*it_mass_neigh);
        } 
      }

      // Update the robot's heading
      // Update robot's velocity
      for (vector<MassAgent>::iterator it_mass=MASS.begin(); it_mass!=MASS.end(); ++it_mass)
      {
        it_mass->updateVel();
        double mass_ang_velocity=it_mass->getThetaAcc();
        b2Vec2 mass_lin_velocity={it_mass->getVelX(), it_mass->getVelY()};
        // mass_lin_velocity.Set(it_mass->getVelX(), it_mass->getVelY());
      
        // it_mass->getBody()->SetLinearVelocity(mass_lin_velocity);
        b2Body_SetLinearVelocity(it_mass->getBodyID(), mass_lin_velocity);
        // it_mass->getBody()->SetAngularVelocity(mass_ang_velocity);
        b2Body_SetAngularVelocity(it_mass->getBodyID(), mass_ang_velocity);
      }

     // Update with new velocity commands
      it_ego=colregs.begin();
      for(it_ego; it_ego!=colregs.end(); ++it_ego)
      {
        // Update the robot's heading
        // Update robot's velocity
        it_ego->updateVelMag();
        it_ego->updateTheta();

        double ang_velocity=it_ego->getThetaAcc();
        b2Vec2 lin_velocity={it_ego->getVelX(), it_ego->getVelY()};
        // lin_velocity={0.0f,0.0f};
        // lin_velocity.Set(it_ego->getVelX(), it_ego->getVelY());
        
        // it_ego->getBody()->SetLinearVelocity(lin_velocity);
        b2Body_SetLinearVelocity(it_ego->getBodyID(), lin_velocity);
        b2Body_SetAngularVelocity(it_ego->getBodyID(), ang_velocity);
        // b2Body_SetAngularVelocity(it_ego->getBodyID(), 1.0f);
        // it_ego->getBody()->SetAngularVelocity(ang_velocity);
      }

      // Instruct the world to perform a single step of simulation.
      // It is generally best to keep the time step and iterations fixed.
      // printf("Angle:\n");
      // printf("\t- Before: %f\n", b2Rot_GetAngle(b2Body_GetRotation(colregs.back().getBodyID())));
      b2World_Step(worldID, timeStep, subStepCount);
      // printf("\t- After: %f\n\n", b2Rot_GetAngle(b2Body_GetRotation(colregs.back().getBodyID())));
      // Now print the position and angle of the body.
      it_ego=colregs.begin();
      for(it_ego; it_ego!=colregs.end(); ++it_ego)
      {
        it_ego->recordStep(t);
        it_ego->agentNeighReset();
      }
      for (vector<MassAgent>::iterator it_mass=MASS.begin(); it_mass!=MASS.end(); ++it_mass) it_mass->recordStep(t), it_mass->agentNeighReset();
    }
   
    // When the world destructor is called, all bodies and joints are freed. This can
    // create orphaned pointers, so be careful about your world management.
    return 0;
}