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

agent::agent(double _radius, double maxX, double maxY, double mxV, double cmf_dst, double sns, double rng)
{
	// LiquidFun particle parameters
	body_def.type=b2_dynamicBody;
	body_def.position.Set((drand48()-0.5)*2*maxY, (drand48()-0.5)*2*maxX);

	// Robot sensor parameters
	sense=sns;
	range=rng;
  radius=_radius;

	maxVel=mxV;
  double initial_vel=2*maxVel*(drand48()-0.5);
  velX=1;
  velY=0;
}

agent::agent(double maxX, double maxY, double mxV)
{
	// LiquidFun particle parameters
	body_def.type=b2_dynamicBody;
	body_def.position.Set((drand48()-0.5)*2*maxY, (drand48()-0.5)*2*maxX);

	// Initialising velocities
	maxVel=mxV;
	velX=maxVel*2*(drand48()-0.5);
	velY=sqrt(maxVel*maxVel-velX*velX);
	double p=drand48();
	if (p<=0.5) velY*=-1;
}


void agent::brownian()
{
	// Angles for turning
	double left=20.0*(PI/180.0);
	double right=-left;

	double p=drand48();
	if (p<=1.0/3.0) std::tie(velX, velY)=rotate(left);
	else if (p<=2.0/3.0) std::tie(velX, velY)=rotate(right);

  double vMag=sqrt(velX*velX+velY*velY);

  double v_ratio=0.0;
  if (vMag>0.0) v_ratio=(maxVel)/vMag;
  velX*=v_ratio;
  velY*=v_ratio;
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
	return velX;
}

double agent::getVelY()
{
	return velY;
}

void agent::setVel(double _vel_x, double _vel_y)
{
  double theta=atan2(velY, velX);
  double theta_new=atan2(_vel_y,_vel_x);
  std::tie(velX, velY)=rotate(theta_new-theta);
}

double agent::getVelMag()
{
  return sqrt(velX*velX+velY*velY);
}

std::tuple<double, double> agent::rotate(double theta)
{
  double new_velX=cos(theta)*velX+sin(theta)*velY;
	double new_velY=-sin(theta)*velX+cos(theta)*velY;
  return  std::make_tuple(new_velX, new_velY);
}


void agent::updateVel()//std::vector<b2Body*> robots)
{
  double theta=10*(PI/180);
  double vMag=sqrt(velX*velX+velY*velY);
  if (vMag>maxVel || vMag<maxVel)
  {
    double ratio=maxVel/vMag;
    velX*=ratio;
    velY*=ratio;
  }
}

double agent::getRadius()
{
  return radius;
}

double agent::getRange()
{
  return range;
}