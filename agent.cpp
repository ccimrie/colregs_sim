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

  // printf("Created body\n");

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

agent::agent(double maxX, double maxY, double mxV)
{
  // LiquidFun particle parameters
  body_def.type=b2_dynamicBody;
  body_def.position.Set((drand48()-0.5)*2*maxY, (drand48()-0.5)*2*maxX);

  // Initialising velocities
  vel_max=mxV;
  velX=vel_max*2*(drand48()-0.5);
  velY=sqrt(vel_max*vel_max-velX*velX);
  double p=drand48();
  if (p<=0.5) velY*=-1;
}

void agent::updateVel(agent* neighbour)
{
 // Extract useful information from self/ego
  double pos_x=getBody()->GetPosition().x;
  double pos_y=getBody()->GetPosition().y;  
  double c_theta=body->GetAngle()*(PI/180.0);
  // printf("Angle:  %f\n", c_theta);
  double targ_theta=atan2(targ_y-body->GetPosition().y, targ_x-body->GetPosition().x);

 // Extract useful information from neighbour
  double other_pos_x=neighbour->getBody()->GetPosition().x;
  double other_pos_y=neighbour->getBody()->GetPosition().y;
  double other_vel_mag=neighbour->getVelMag();
  double other_radius=neighbour->getRadius();
  double other_theta=neighbour->getBody() ->GetAngle()*(PI/180);
  double dist_x=other_pos_x-pos_x;
  double dist_y=other_pos_y-pos_y;
  double dist=sqrt(dist_x*dist_x+dist_y*dist_y);
  double o_theta=atan2(other_pos_y, other_pos_x);//(PI*c_theta/180.0);
  dist-=(radius+other_radius);

  if (dist>min_dist) return;

 // Other vessel's relative position in ego's reference frame
  double other_c_relative_pos_x=(cos(-c_theta)*dist_x-sin(-c_theta)*dist_y);
  double other_c_relative_pos_y=(sin(-c_theta)*dist_x+cos(-c_theta)*dist_y);
  double o_c_relative_theta=atan2(other_c_relative_pos_y, other_c_relative_pos_x);
  // printf("Before/after:  (%f, %f), (%f, %f), (%f, %f)\n", other_pos_x, other_pos_y, pos_x, pos_y, other_c_relative_pos_x, other_c_relative_pos_y);

 // Other vessel's relative position in ego's reference frame rotated by angle to goal
  double other_targ_relative_pos_x=cos(-targ_theta)*dist_x-sin(-targ_theta)*dist_y;
  double other_targ_relative_pos_y=sin(-targ_theta)*dist_x+cos(-targ_theta)*dist_y;
  double o_targ_relative_theta=atan2(other_targ_relative_pos_y, other_targ_relative_pos_x);
  // c_theta=std::fmod(c_theta,360)*(PI/180);
  // other_theta=std::fmod(other_theta,360)*(PI/180);

 // Check if ship is oncoming
  double oncoming_thresh_min=2*PI*(3/8.0);
  double oncoming_thresh_max=2*PI*(5/8.0);
  
  double c_theta_norm=c_theta;
  double other_theta_norm=other_theta;
  double targ_theta_norm=targ_theta;

  if (c_theta_norm<0) c_theta_norm=2*PI+c_theta_norm;
  if (other_theta_norm<0) other_theta_norm=2*PI+other_theta_norm;
  if (targ_theta_norm<0) targ_theta_norm=2*PI+targ_theta_norm;
  
  double heading_diff_ego_frame=other_theta_norm-c_theta_norm;
  if (heading_diff_ego_frame<0) heading_diff_ego_frame=2*PI+heading_diff_ego_frame;

  double heading_diff_targ_frame=other_theta_norm-targ_theta_norm;
  if (heading_diff_targ_frame<0) heading_diff_targ_frame=2*PI+heading_diff_targ_frame;

  int lookahead_time=600;

  bool updated=false;

  int time_collision=checkFuture(lookahead_time, other_pos_x, other_pos_y, other_vel_mag, o_theta, other_radius);

  // if (time_collision<0)
  // {
  //     // Overtaking
  //   if (o_c_relative_theta<67.5*(PI/180.0) && o_c_relative_theta>-67.5*(PI/180.0) && radius>other_radius 
  //           && (heading_diff_ego_frame<PI/4 || heading_diff_ego_frame>2*PI*3/4.0))
  //   {
  //     // int time_collision=checkFuture(lookahead_time, other_pos_x, other_pos_y, other_vel_mag, o_theta, other_radius);
  //     overTakingUpdate(other_c_relative_pos_x, other_c_relative_pos_y, dist, other_radius, other_vel_mag, time_collision);
  //     updated=true;
  //   } 
  //   return;
  // }

 // Situation identification
  // Oncoming scenario 
  if (heading_diff_ego_frame>oncoming_thresh_min && heading_diff_ego_frame<oncoming_thresh_max)
  {
    // int time_collision=checkFuture(lookahead_time, other_pos_x, other_pos_y, other_vel_mag, o_theta, other_radius);
    // if (time_collision>0) printf("In oncoming event collision will occur in %i timesteps\n", time_collision);
    oncomingUpdate(other_c_relative_pos_x, other_c_relative_pos_y, dist, other_radius, time_collision);
    updated=true;
  }

  // Crossing?
  else if (other_c_relative_pos_x>0 && other_c_relative_pos_y<other_radius && (heading_diff_ego_frame>PI*1/4.0 && heading_diff_ego_frame<PI*3/4.0))
  {
    // int time_collision=checkFuture(lookahead_time, other_pos_x, other_pos_y, other_vel_mag, o_theta, other_radius);
    if (time_collision>0) printf("In crossing event collision will occur in %i timesteps\n", time_collision);
    crossingUpdate(other_c_relative_pos_x, other_c_relative_pos_y, dist, other_radius, time_collision);
    updated=true;
  }

  // Overtaking
  else if (o_c_relative_theta<67.5*(PI/180.0) && o_c_relative_theta>-67.5*(PI/180.0) && radius>other_radius 
          && (heading_diff_ego_frame<PI/4 || heading_diff_ego_frame>2*PI*3/4.0))
  {
    // int time_collision=checkFuture(lookahead_time, other_pos_x, other_pos_y, other_vel_mag, o_theta, other_radius);
    overTakingUpdate(other_c_relative_pos_x, other_c_relative_pos_y, dist, other_radius, other_vel_mag, time_collision);
    updated=true;
  } 
  else if (other_c_relative_pos_x>-other_radius && dist<cmf_dist)
  {
    double turn_angle=atan2(dist_y, dist_x);
    new_theta_acc=-turn_angle;
    updated=true;
    // printf("New turn angle: (%f,%f), (%f,%f) %f\n", pos_x, pos_y, other_pos_x, other_pos_y, turn_angle);
  }

  // if (dist<cmf_dist)
  // { 
    // int time_collision=checkFuture(lookahead_time, other_pos_x, other_pos_y, other_vel_mag, o_theta, other_radius);
    if (time_collision>0)
    {
      double max_crash_time=600.0;
      double temp_vel_mag=vel_max*(time_collision/max_crash_time);
      if (temp_vel_mag<new_vel_mag) new_vel_mag=temp_vel_mag;
    }
    else
    {
      double temp_vel_mag=vel_max*(dist/(range-radius-other_radius));
    }
  // }

  if (updated) min_dist=dist;

  // Check for general problems; is there a ship infront of us (just slow down for now?)
  // else if (other_c_relative_pos_x>-other_radius && dist<cmf_dist)
  // {
  //   // double temp_vel_mag=vel_max*((dist-cmf_dist)/(range-radius-other_radius));
  //   // temp_vel_mag=std::max(temp_vel_mag, 0.0);
  //   // if (temp_vel_mag<new_vel_mag) new_vel_mag=temp_vel_mag;
  //   // double force=(cmf_dist/dist);
  //   double pf_pos_x=(pos_x-other_pos_x);
  //   double pf_pos_y=(pos_y-other_pos_y);

  //   double new_theta=atan2(pf_pos_y, pf_pos_x);
  //   double temp_theta_acc=c_theta-new_theta;
  //   if (abs(temp_theta_acc)>abs(new_theta_acc)) new_theta_acc=temp_theta_acc;

  // }
  // else
  // {
  //   double temp_vel_mag=vel_max*((dist-cmf_dist)/(range-radius-other_radius));
  //   temp_vel_mag=std::max(temp_vel_mag, 0.0);
  //   if (temp_vel_mag<new_vel_mag) new_vel_mag=temp_vel_mag;
  // }
 // Distance from forbidden zones 
}


int agent::checkFuture(int lookahead_time, double neigh_pos_x, double neigh_pos_y, double neigh_vel_mag, double neigh_theta, double neigh_radius)
{
  double pos_x=getBody()->GetPosition().x;
  double pos_y=getBody()->GetPosition().y;
  double vel_x=getVelX();
  double vel_y=getVelY();

  double neigh_vel_x=cos(neigh_theta)*neigh_vel_mag;
  double neigh_vel_y=sin(neigh_theta)*neigh_vel_mag;
  double dt=1/60.0;

  for (int t=1; t<lookahead_time; ++t)
  {
    // printf("Sanity check at time step %i at vel (%f, %f):\n", t, vel_x, vel_y);
    // printf("\t- Starting pos:  (%f,%f)\n",pos_x,pos_y);
    pos_x+=dt*vel_x;
    pos_y+=dt*vel_y;
    neigh_pos_x+=dt*neigh_vel_x;
    neigh_pos_y+=dt*neigh_vel_y;
    // printf("\t- Next pos:  (%f,%f)\n", pos_x,pos_y);
    // printf("\t- Original pos:  (%f,%f)\n", getBody()->GetPosition().x,getBody()->GetPosition().y);

    double dist_x=(neigh_pos_x-pos_x);
    double dist_y=(neigh_pos_y-pos_y);
    double dist=sqrt(dist_x*dist_x+dist_y*dist_y);
    if (dist-radius-neigh_radius<0) return t;
  }
  return -1;
}

void agent::oncomingUpdate(double other_targ_relative_pos_x, double other_targ_relative_pos_y, double dist, double other_radius, int time_collision)
{
  if (time_collision<0) return;
 // Get new target point
  double temp_theta_acc;

  double frame_theta=egoAngle();
  double oncoming_goal_theta=newAngleWorldFrame(other_targ_relative_pos_x, other_targ_relative_pos_y, other_radius, frame_theta);

  double heading=body->GetAngle()*(PI/180.0);    
  temp_theta_acc=(oncoming_goal_theta-heading);

  // if (abs(temp_theta_acc)>abs(new_theta_acc)) new_theta_acc=temp_theta_acc;
  new_theta_acc=temp_theta_acc;

  double max_crash_time=600.0;
  double temp_vel_mag=vel_mag*(time_collision/max_crash_time);
  if (temp_vel_mag<new_vel_mag) new_vel_mag=temp_vel_mag;
}

void agent::overTakingUpdate(double other_pos_x, double other_pos_y, double dist, double other_radius, double other_vel_mag, int time_collision)
{
   // If other ship is smaller, assume you can go faster and perform overtaking procedure
    if (radius>other_radius)
    {
      double frame_theta=egoAngle();
      double overtaking_goal_theta=newAngleWorldFrame(other_pos_x, other_pos_y, other_radius, frame_theta);

      double heading=body->GetAngle()*(PI/180.0);    
      double temp_theta_acc=-(overtaking_goal_theta-heading);
      if (other_pos_x<radius+other_radius && other_pos_y<0) temp_theta_acc*=-1;    
      // if (abs(temp_theta_acc)>abs(new_theta_acc)) new_theta_acc=temp_theta_acc;
      new_theta_acc=temp_theta_acc;

      double temp_vel_mag=vel_max;
      if (time_collision>0)
      {
        double max_crash_time=6.0e2;
        double temp_vel_mag=vel_mag*(time_collision/max_crash_time);
      }
      if (temp_vel_mag<new_vel_mag) new_vel_mag=temp_vel_mag;
    }
   // If going faster than smaller ship slow appropriately
    else if (new_vel_mag>other_vel_mag) 
    {
      new_vel_mag=other_vel_mag;
    }
}

void agent::makeWayUpdate(double o_relative_theta, double other_radius)
{
 // Ship is behind, let's get out of way if they are bigger
  if (radius<other_radius)
  {
    double adjusted_theta=(-PI)-o_relative_theta;
    double temp_theta_acc=-d_theta*adjusted_theta;
    if (abs(temp_theta_acc)>abs(new_theta_acc)) new_theta_acc=temp_theta_acc;
  }
}

void agent::crossingUpdate(double other_pos_x, double other_pos_y, double dist, double other_radius, int time_collision)
{
  double temp_vel_mag=vel_max;
  if (time_collision>0)
  {  
    // double frame_theta=targAngle();
    double frame_theta=egoAngle();
    double crossing_goal_theta=newAngleWorldFrame(other_pos_x, other_pos_y, other_radius, frame_theta);
    
    double heading=body->GetAngle()*(PI/180.0);    
    double temp_theta_acc=(crossing_goal_theta-heading);
    
    // if (abs(temp_theta_acc)>abs(new_theta_acc)) new_theta_acc=temp_theta_acc;
    new_theta_acc=temp_theta_acc;
  
    double max_crash_time=6.0e2;
    double temp_vel_mag=vel_mag*(time_collision/max_crash_time);
    temp_vel_mag=std::max(temp_vel_mag,0.01);
  }

  if (temp_vel_mag<new_vel_mag) new_vel_mag=temp_vel_mag;
}

double agent::newAngleWorldFrame(double other_pos_x, double other_pos_y, double other_radius, double frame_theta)
{
  double pos_x=getBody()->GetPosition().x;
  double pos_y=getBody()->GetPosition().y;

 // Get new target point
  double new_frame_goal_x=other_pos_x;
  double new_frame_goal_y=other_pos_y-(other_radius);

  double new_goal_x=cos(frame_theta)*new_frame_goal_x-sin(frame_theta)*new_frame_goal_y+pos_x;
  double new_goal_y=sin(frame_theta)*new_frame_goal_x+cos(frame_theta)*new_frame_goal_y+pos_y;
  
  double new_goal_theta=atan2(new_goal_y, new_goal_x);

  return new_goal_theta;
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
  // printf("New theta vel:  %f\n", theta_acc);
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
  return new_targ_pos-pos;
}

double agent::goalTheta()//double x_pos, double y_pos, double x_targ, double y_targ)
{
  double pos_x=body->GetPosition().x;
  double pos_y=body->GetPosition().y;

  double relative_targ_x=getRelativeGoal(pos_x, targ_x, var_x);
  double relative_targ_y=getRelativeGoal(pos_y, targ_y, var_y);

  double vel_x=getVelX();
  double vel_y=getVelY();

  double dot=vel_x*relative_targ_x + vel_y*relative_targ_y;
  double det=vel_x*relative_targ_y - relative_targ_x*vel_y;
  double goal_theta=atan2(det, dot);
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

void agent::recordStep(int t)
{
  // Recording:
  //   - 0: x
  //   - 1: y
  //   - 2: \theta
  //   - 3: radius (size)
  //   - 4: sensor/communication range
  //   - 5: x-target
  //   - 6: x-target tolerance
  //   - 7: y-target
  //   - 8: y-target tolerance
  //   - 9: current world timestep
  //   - 10: agent type (for plotting and analysis)
  outfile.open(filename, std::ios_base::app);
  outfile << body->GetPosition().x  << " " 
          << body->GetPosition().y << " " 
          << body->GetAngle() << " "
          << radius << " "
          << range << " "
          << targ_x << " "
          << var_x << " "
          << targ_y << " "
          << var_y << " "
          << t << " "
          << agent_type << " ";
  outfile << std::endl; 
  outfile.close();
}