#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <vector>
#include <string.h>
#include <fstream>
#include <random>
#include <chrono>
#include <armadillo>
#include <tuple>
#include "agent.h"

#define PI 3.14159265

agent::agent()
{
  // EMPTY CONSTRUCTOR
}

agent::agent(std::string _yaml_file, std::string results_file, double _seed)
{
  yaml_file=_yaml_file;
  seed=_seed;
  initialiseAgent();
 // Reset/create file
  filename=results_file;
  outfile.open(filename);
  outfile.close();
}

void agent::initialiseAgent()
{
  // LiquidFun particle parameters
  body_def.type=b2_dynamicBody;

  // Read from yaml file
  YAML::Node config = YAML::LoadFile(yaml_file);
 // Initialise agent's position
  const double x_min=config["initial pose"]["x"]["min"].as<double>();
  const double x_max=config["initial pose"]["x"]["max"].as<double>();
  const double y_min=config["initial pose"]["y"]["min"].as<double>();
  const double y_max=config["initial pose"]["y"]["max"].as<double>();
  const double theta_min=config["initial pose"]["theta"]["min"].as<double>();
  const double theta_max=config["initial pose"]["theta"]["max"].as<double>();
  std::uniform_real_distribution<double> distribution_x(x_min, x_max);
  std::uniform_real_distribution<double> distribution_y(y_min, y_max);
  std::uniform_real_distribution<double> distribution_theta(theta_min, theta_max);
  // printf("Loaded agent's yaml file\n");

  // srand48(time(NULL));
  std::default_random_engine gen;
  gen.seed(seed);
  double pos_x=distribution_x(gen);
  double pos_y=distribution_y(gen);
  double _angle=distribution_theta(gen);
  body_def.position.Set(pos_x, pos_y);
  body_def.angle=_angle;

 // Initialise agent's size, vel, and range
  double size_min=config["size"]["min"].as<double>();
  double size_max=config["size"]["max"].as<double>();
  std::uniform_real_distribution<double> distribution_size(size_min, size_max);
  radius=distribution_size(gen);

 // Robot sensor parameters
  range=radius*config["range-size ratio"].as<double>();
  vel_max=radius*config["vel-size ratio"]["linear"].as<double>();
  vel_theta_max=config["vel-size ratio"]["angular"].as<double>()/radius;
  agent_type=config["agent type"].as<int>();

 // Set target
  targ_x=config["goal pose"]["x"]["target"].as<double>();
  var_x=config["goal pose"]["x"]["tolerance"].as<double>();
  targ_y=config["goal pose"]["y"]["target"].as<double>();
  var_y=config["goal pose"]["y"]["tolerance"].as<double>();

  cmf_dist=0.5*radius;

 // Generate forbidden zones
  YAML::Node fz=config["fobidden zones"];
  for(YAML::const_iterator it=fz.begin(); it!=fz.end(); ++it)
  {
    std::string key=it->first.as<std::string>();         // <- key
    std::vector<double> fz_coords;
    fz_coords.push_back(fz[key]["x"]["min"].as<double>());
    fz_coords.push_back(fz[key]["x"]["max"].as<double>());
    fz_coords.push_back(fz[key]["y"]["min"].as<double>());
    fz_coords.push_back(fz[key]["y"]["max"].as<double>());
    forbidden_zones.push_back(fz_coords);
  }
}

double agent::targAngle()
{
  return atan2(targ_y-getBody()->GetPosition().y, targ_x-getBody()->GetPosition().x);
}

double agent::egoAngle()
{
  return (getBody()->GetAngle()*(PI/180.0));
}


b2BodyDef agent::getBodyDef()
{
  return body_def;
}

void agent::setBody(b2Body* _body)
{
  body=_body;
}

void agent::setBodyDefPose(double _x_pos, double _y_pos, double _theta)
{ 
  body_def.position.Set(_x_pos, _y_pos);
  body_def.angle=_theta;
}

void agent::setBodyPosition(double _x_pos, double _y_pos)
{
  body->SetTransform(b2Vec2(_x_pos,_y_pos),body->GetAngle());
}

b2Body* agent::getBody()
{
  return body;
}

double agent::getVelX()
{
  // return velX;
  theta=body->GetAngle();
  return cos(PI*theta/180.0)*vel_mag;
}

double agent::getVelY()
{
  // return velY;
  theta=body->GetAngle();
  return sin(PI*theta/180.0)*vel_mag;
}

double agent::getMaxVel()
{
  return vel_max;
}

void agent::updateTheta()
{
  double gamma=0.9;
  if (new_theta_acc>0.0 || new_theta_acc<0.0)
  { 
   // Update theta_acc command
    theta_acc=new_theta_acc;//gamma*(new_theta_acc)+(1-gamma)*goalTheta();
    if (theta_acc>vel_theta_max) theta_acc=vel_theta_max;
    else if (theta_acc<-vel_theta_max) theta_acc=-vel_theta_max;
  }
  else
  {
    theta_acc=goalTheta();
    if (theta_acc>vel_theta_max) theta_acc=vel_theta_max;
    else if (theta_acc<-vel_theta_max) theta_acc=-vel_theta_max;
  }
  new_theta_acc=0;
}

double agent::getRelativeGoal(double pos, double targ_pos, double var_pos)
{
  double gamma=0.2; // Shift towards centre of lane
  double new_targ_pos;
  if (pos<(targ_pos-var_pos))
  {
    new_targ_pos=targ_pos-var_pos;
  }
  else if (pos>targ_pos+var_pos)
  {
    new_targ_pos=targ_pos+var_pos;
  }
  else
  {
    new_targ_pos=(1-gamma)*pos+gamma*(targ_pos);
  }
  return new_targ_pos;
}

double agent::goalTheta()//double x_pos, double y_pos, double x_targ, double y_targ)
{
  double pos_x=body->GetPosition().x;
  double pos_y=body->GetPosition().y;
  double heading=body->GetAngle()*(PI/180.0);

  double relative_targ_x=getRelativeGoal(pos_x, targ_x, var_x);
  double relative_targ_y=getRelativeGoal(pos_y, targ_y, var_y);

  double theta_diff=atan2(relative_targ_y-pos_y, relative_targ_x-pos_x);
  double goal_theta=theta_diff-heading;
  if (goal_theta<-PI) goal_theta+=2*PI;
  else if (goal_theta>PI) goal_theta-=2*PI;

  // double vel_x=getVelX();
  // double vel_y=getVelY();

  // double dot=vel_x*relative_targ_x + vel_y*relative_targ_y;
  // double det=vel_x*relative_targ_y - relative_targ_x*vel_y;
  // double goal_theta=atan2(det, dot);
  // printf("Goal theta:\n\tposition: %f  %f\n\tgoal position: %f  %f\n\tgoal heading: %f\n", pos_x, pos_y, relative_targ_x, relative_targ_y, goal_theta);
  return goal_theta;
}

void agent::setTarget(double _targ_x, double _var_x, double _targ_y, double _var_y)
{
  targ_x=_targ_x;
  var_x=_var_x;

  targ_y=_targ_y;
  var_y=_var_y;
}

bool agent::checkGoal()
{
  double pos_x=getBody()->GetPosition().x;
  double pos_y=getBody()->GetPosition().y;
  if (pos_x<targ_x+var_x && pos_x>targ_x-var_x && pos_y<targ_y+var_y && pos_y>targ_y-var_y) return true;
  else return false;
} 

void agent::setTheta(double _theta)
{
  theta=_theta;
}

void agent::setThetaAcc(double _theta_acc)
{
  theta_acc=_theta_acc;
}

double agent::getThetaAcc()
{
  return theta_acc;
}

double agent::getTheta()
{
  return theta;
}

double agent::getVelMag()
{
  return vel_mag;
}

void agent::updateVelMag()
{
  vel_mag=new_vel_mag;
  // vel_mag=0.0;
  // if (vel_mag==0) vel_mag=0.001;
  new_vel_mag=vel_max;
  min_dist=2*range;
  // printf("New vel mag:  %f / %f\n", vel_mag, vel_max);
}

void agent::setVelMag(double _vel_mag)
{
  vel_mag=_vel_mag;
  // updateVel();
}

void agent::setVel(double _vel_x, double _vel_y)
{
  theta=atan2(_vel_y, _vel_x);
  // updateVel(theta);
}

double agent::getRadius()
{
  return radius;
}

double agent::getRange()
{
  return range;
}

void agent::agentNeighReset()
{
  no_neigh=0;
  sum_neigh_dist=range;
}

void agent::recordStep(int t)
{
  // TODO: save output as csv file; easier to add without needing to modify python visualising/data analysis
  // Recording:
  //   - 0: x
  //   - 1: y
  //   - 2: \theta
  //   - 3: radius (size)
  //   - 4: sensor/communication range
  //   - 5: number of neighbours
  //   - 6: average distance to neighbours
  //   - 7: x-target
  //   - 8: x-target tolerance
  //   - 9: y-target
  //   - 10: y-target tolerance
  //   - 11: current world timestep
  //   - 12: agent type (for plotting and analysis)
  outfile.open(filename, std::ios_base::app);
  outfile << body->GetPosition().x  << " " 
          << body->GetPosition().y << " " 
          << body->GetAngle() << " "
          << radius << " "
          << range << " "
          << no_neigh << " "
          << sum_neigh_dist/no_neigh << " "
          << targ_x << " "
          << var_x << " "
          << targ_y << " "
          << var_y << " "
          << t << " "
          << agent_type << " ";
  outfile << std::endl; 
  outfile.close();
}