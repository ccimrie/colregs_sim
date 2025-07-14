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

agent::agent(double _radius, double x_min, double x_max, double y_min, double y_max, double _angle, double mxV, double rng, int _agent_type, std::string results_file)
{
	// LiquidFun particle parameters
	body_def.type=b2_dynamicBody;
	body_def.position.Set(pos_x, pos_y);
  body_def.angle=_angle;

	// Robot sensor parameters
	range=rng;
  radius=_radius;

  distribution_x(x_min, x_max);
  distribution_x(y_min, y_max);

  agent_type=_agent_type;

  cmf_dist=range/3.0;

	vel_max=mxV;
  double initial_vel=2*vel_max*(drand48()-0.5);
  velX=1;
  velY=0;
  filename=results_file;

  // Reset/create file
  outfile.open(filename);
  outfile.close();
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


void agent::setInitialPose(double x_min, double x_max, double y_min, double y_max)
{

}

 agent::getInitialPose(double x_min, double x_max, double y_min, double y_max)
{
  distribution_x(x_min, x_max);
  distribution_x(y_min, y_max);
}


// void agent::updateVel(double other_vel_x, double other_vel_y, double other_pos_x, double other_pos_y, double other_vel_mag)
void agent::updateVel(double other_pos_x, double other_pos_y, double other_vel_mag, double dist, double other_radius, double other_theta)
{
  // Conditions:
  // 1. Agent in front
  //  a. facing away
  //  b. facing toward
  // 2. Agent behind
  //  a. facing away
  //  b. facing toward

  // Check where other ship is
  double o_theta=atan2(other_pos_y, other_pos_x);//(PI*c_theta/180.0);
  double c_theta=body->GetAngle()*(PI/180.0);
  // c_theta=std::fmod(c_theta,360)*(PI/180);
  double other_relative_pos_x=cos(-c_theta)*other_pos_x-sin(-c_theta)*other_pos_y;
  double other_relative_pos_y=sin(-c_theta)*other_pos_x+cos(-c_theta)*other_pos_y;
  double o_relative_theta=atan2(other_relative_pos_y, other_relative_pos_x);
  // other_theta=std::fmod(other_theta,360)*(PI/180);

  // Check if ship is oncoming
  double oncoming_thresh_min=2*PI*(1/3.0);
  double oncoming_thresh_max=2*PI*(2/3.0);
  double c_theta_norm=c_theta;
  if (c_theta_norm<0) c_theta_norm=2*PI+c_theta_norm;
  if (other_theta<0) other_theta=2*PI+other_theta;
  double heading_diff=abs(other_theta-c_theta_norm);

  double d_v=0.5;

  // Oncoming scenario 
  if (heading_diff>oncoming_thresh_min && heading_diff<oncoming_thresh_max)
  {
    double temp_theta_acc=0;
    if ((o_relative_theta<PI/3.0 && o_relative_theta>-PI/8.0) && other_relative_pos_x>0)
    { 
      double adjusted_theta=(PI/2.0)-o_relative_theta;
      temp_theta_acc=-d_theta*adjusted_theta;
    }
    else if ((o_relative_theta>-PI/3.0 && o_relative_theta<-PI/8.0) && other_relative_pos_x>0)
    {
      double adjusted_theta=-(PI/2.0)-o_relative_theta;
      temp_theta_acc=-d_theta*adjusted_theta;
    }
    if (abs(temp_theta_acc)>abs(new_theta_acc)) new_theta_acc=temp_theta_acc;
    double temp_vel_mag;
    if (dist<cmf_dist) temp_vel_mag*=d_v;
    else temp_vel_mag=d_v*vel_max*(1.0/(dist+radius));
    // d_v*vel_max*((dist-radius)/(range+radius));
    if (temp_vel_mag<new_vel_mag) new_vel_mag=temp_vel_mag;
  }
  // Overtaking scenario
  else if ((o_relative_theta>-PI/4.0 || o_relative_theta<PI/4.0) && other_relative_pos_x>0)
  {
   // If ship is small, assume you can go faster and perform overtaking procedure
    if (radius>other_radius)
    {
      double adjusted_theta=(PI/2.0)-o_relative_theta;
      double temp_theta_acc=-d_theta*adjusted_theta;

      if (abs(temp_theta_acc)>abs(new_theta_acc)) new_theta_acc=temp_theta_acc;

      double temp_vel_mag;
      if (dist<cmf_dist) temp_vel_mag*=d_v;
      else temp_vel_mag=d_v*vel_max*(1.0/(dist+radius));
      // d_v*vel_max*((dist-radius)/(range+radius));
      if (temp_vel_mag<new_vel_mag) new_vel_mag=temp_vel_mag;
    }
   // If going faster than ship slow appropriately
    else if (new_vel_mag>other_vel_mag) 
    {
      new_vel_mag=other_vel_mag;
    }
  }
 // Crossing?
  else if (other_relative_pos_x>0)
  {
    if (o_relative_theta>0 && other_relative_pos_y<radius)
    {
      double adjusted_theta=(PI/2.0)-o_relative_theta;
      double temp_theta_acc=-d_theta*adjusted_theta;
      if (abs(temp_theta_acc)>abs(new_theta_acc)) new_theta_acc=temp_theta_acc;

      double temp_vel_mag=d_v*vel_max*(1.0/(dist+radius));
      // d_v*vel_max*((dist-radius)/(range+radius));
      if (temp_vel_mag<new_vel_mag) new_vel_mag=temp_vel_mag;
    }
    else if (o_relative_theta>(-PI/3.0))
    {
      double adjusted_theta=(-PI/3.0)-o_relative_theta;
      double temp_theta_acc=-d_theta*adjusted_theta;
      if (abs(temp_theta_acc)>abs(new_theta_acc)) new_theta_acc=temp_theta_acc;
  
      double temp_vel_mag=d_v*vel_max*(1.0/(dist+radius));
      // d_v*vel_max*((dist-radius)/(range+radius));
      if (dist<cmf_dist) temp_vel_mag*=d_v;
      else temp_vel_mag=d_v*vel_max*(1.0/(dist+radius));
      // d_v*vel_max*((dist-radius)/(range+radius));
      if (temp_vel_mag<new_vel_mag) new_vel_mag=temp_vel_mag;
    }
    else
    {
      new_vel_mag=0.0;
    }
  }
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
  if (new_theta_acc!=0.0)
  { 
    double goal_theta=goalTheta();
    if (goal_theta+new_theta_acc>PI/2.0) new_theta_acc=(PI/2.0)-goal_theta;
    else if (goal_theta+new_theta_acc<-PI/2.0) new_theta_acc=(-PI/2.0)-goal_theta;
    theta_acc=new_theta_acc;//gamma*(new_theta_acc)+(1-gamma)*goalTheta();
  }
  else
  {
    theta_acc=goalTheta();
  }
  // printf("New theta vel:  %f\n", theta_acc);
  new_theta_acc=0;
}

double agent::getRelativeGoal(double pos, double targ_pos, double var_pos)
{
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
    new_targ_pos=pos;
  }
  return new_targ_pos-pos;
}

double agent::goalTheta()//double x_pos, double y_pos, double x_targ, double y_targ)
{
  double pos_x=body->GetPosition().x;
  double pos_y=body->GetPosition().y;

  // double relative_targ_x=(targ_x-x_pos);
  // relative_targ_x-=(var_x*signbit(relative_targ_x));

  // double relative_targ_y=targ_y-y_pos;
  // relative_targ_y-=(var_y*signbit(relative_targ_y));

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

// void agent::updateVel(double c_theta)
// {
//   velX=cos(PI*c_theta/180.0)*vel_mag;
//   velY=sin(PI*c_theta/180.0)*vel_mag;
// }

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
  new_vel_mag=vel_max;
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

void agent::recordStep()
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
  //   - 9: agent type (for plotting and analysis)
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
          << agent_type << " ";
  outfile << std::endl; 
  outfile.close();
}