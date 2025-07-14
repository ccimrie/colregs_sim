#include "mass_agent.h"
#include <cmath>
// #define M_PI 3.14159265

MassAgent::MassAgent()
{
  // EMPTY CONSTRUCTOR
}

void MassAgent::updateVel()
{
 // Calculate attractor
  double pos_x=body->GetPosition().x;
  double pos_y=body->GetPosition().y;
  double heading=body->GetAngle()*(M_PI/180.0);
  double targ_theta=atan2(targ_y-pos_y, targ_x-pos_x);
  double goal_dist=getGoalDist(pos_x, pos_y);
  double pf_x_g=goal_weight*goal_dist*cos(targ_theta);
  double pf_y_g=goal_weight*goal_dist*sin(targ_theta);

 // Combine potential fields
  double pf_x=pf_x_g+pf_x_neigh;
  double pf_y=pf_y_g+pf_y_neigh;

  pf_x_neigh=0;
  pf_y_neigh=0;

 // Update theta acceleration
  targ_theta=atan2(pf_y, pf_x);
  double theta_diff=targ_theta-heading;
  if (theta_diff<-M_PI) theta_diff+=2*M_PI;
  else if (theta_diff>M_PI) theta_diff-=2*M_PI;

  theta_acc=theta_diff;
  if (theta_acc>vel_theta_max) theta_acc=vel_theta_max;
  else if (theta_acc<-vel_theta_max) theta_acc=-vel_theta_max;
  
 // Update velocity magnitude
  vel_mag=vel_max*(1-abs(theta_diff)/M_PI);
  if (vel_mag<0) vel_mag=0;
  else if (vel_mag>vel_max) vel_mag=vel_max;
}


void MassAgent::updateNeighPF(agent* neighbour)
{
  // int true_class=...
  // int predicted_class=prediction(true_class);
 // Extract useful information from self/ego
  double pos_x=getBody()->GetPosition().x;
  double pos_y=getBody()->GetPosition().y;  

 // Extract useful information from neighbour
  double other_pos_x=neighbour->getBody()->GetPosition().x;
  double other_pos_y=neighbour->getBody()->GetPosition().y;
  double other_radius=neighbour->getRadius();
  double dist_x=other_pos_x-pos_x;
  double dist_y=other_pos_y-pos_y;
  double dist=sqrt(dist_x*dist_x+dist_y*dist_y);
  double o_theta=atan2(dist_y, dist_x);
  dist-=(radius+other_radius);

  double temp_neigh_weight=neigh_weight;
  if (dist<safety_bubble) neigh_weight=10.0; 

  pf_x_neigh+=-neigh_weight*(range-dist)*cos(o_theta);
  pf_y_neigh+=-neigh_weight*(range-dist)*sin(o_theta);
  neigh_weight=temp_neigh_weight;
}


double MassAgent::getGoalDist(double pos_x, double pos_y)
{
  double relative_targ_x=getRelativeGoal(pos_x, targ_x, var_x);
  double relative_targ_y=getRelativeGoal(pos_y, targ_y, var_y);
  // printf("Relative targets:  %f  %f\n", relative_targ_x, relative_targ_y);
  double x_dist=pos_x-relative_targ_x;
  double y_dist=pos_y-relative_targ_y;
  // printf("Relative distance:  %f  %f\n\n", x_dist, y_dist);
  double dist=sqrt(x_dist*x_dist+y_dist*y_dist);
  return dist;
}