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
// #include <armadillo>
#include <tuple>
#include "normal_agent.h"
#include <unistd.h>
#include <string>

#define PI 3.14159265
#define MPPI_ENABLED

NormalAgent::NormalAgent()
{
  // EMPTY CONSTRUCTOR
}

NormalAgent::NormalAgent(std::string yaml_file, std::string results_file, double seed) : agent(yaml_file, results_file, seed)
{
  int nmpc_ind=filename.find(".");
  filename_nmpc_seq=filename.substr(0,nmpc_ind).append("_nmpc_seq.txt");
  outfile.open(filename_nmpc_seq);
  outfile.close();

  Nx=5;
  Ny=3;
  Nu=2;
  Nph=10;
  Nch=4; // Could be 10%-20% of prediction horizon (should double check)
  Nieq=0;
  Neq=0;
  controller->setLoggerLevel(mpc::Logger::LogLevel::NONE);

  createNLMPC();

}

void NormalAgent::updateVel(agent* neighbour)
{

  // nonLinearMPC();

 // Extract useful information from self/ego
  b2Vec2 position=b2Body_GetPosition(getBodyID());
  double pos_x=position.x;
  double pos_y=position.y; 
  b2Rot rotation=b2Body_GetRotation(getBodyID());
  double heading=b2Rot_GetAngle(rotation);

  double targ_theta=atan2(targ_y-pos_y, targ_x-pos_x);

 // Extract useful information from neighbour
  b2Vec2 neigh_position=b2Body_GetPosition(neighbour->getBodyID());
  double neigh_pos_x=neigh_position.x;
  double neigh_pos_y=neigh_position.y; 
  b2Rot neigh_rotation=b2Body_GetRotation(neighbour->getBodyID());
  double neigh_heading=b2Rot_GetAngle(rotation);
  double neigh_theta=atan2(neigh_pos_y, neigh_pos_x);
  
  double neigh_vel_mag=neighbour->getVelMag();
  double neigh_radius=neighbour->getRadius();

  double dist_x=neigh_pos_x-pos_x;
  double dist_y=neigh_pos_y-pos_y;
  double dist=sqrt(dist_x*dist_x+dist_y*dist_y);
  dist-=(radius+neigh_radius);

  // if (dist>min_dist) return;

 // Other vessel's relative position in ego's reference frame
  double neigh_ego_relative_pos_x=(cos(-heading)*dist_x-sin(-heading)*dist_y);
  double neigh_ego_relative_pos_y=(sin(-heading)*dist_x+cos(-heading)*dist_y);
  double neigh_ego_relative_theta=atan2(neigh_ego_relative_pos_y, neigh_ego_relative_pos_x);

 // Other vessel's relative position in ego's reference frame rotated by angle to goal
  double neigh_targ_relative_pos_x=cos(-targ_theta)*dist_x-sin(-targ_theta)*dist_y;
  double neigh_targ_relative_pos_y=sin(-targ_theta)*dist_x+cos(-targ_theta)*dist_y;
  double neigh_targ_relative_theta=atan2(neigh_targ_relative_pos_y, neigh_targ_relative_pos_x);

 // Check if ship is oncoming
  double oncoming_thresh_min=2*PI*(3/8.0);
  double oncoming_thresh_max=2*PI*(5/8.0);
  
  double heading_norm=heading;
  double neigh_heading_norm=neigh_heading;
  double targ_theta_norm=targ_theta;

  if (heading_norm<0) heading_norm=2*PI+heading_norm;
  if (neigh_heading_norm<0) neigh_heading_norm=2*PI+neigh_heading_norm;
  if (targ_theta_norm<0) targ_theta_norm=2*PI+targ_theta_norm;
  
  double heading_diff_ego_frame=neigh_heading_norm-heading_norm;
  if (heading_diff_ego_frame<0) heading_diff_ego_frame=2*PI+heading_diff_ego_frame;

  double heading_diff_targ_frame=neigh_heading_norm-targ_theta_norm;
  if (heading_diff_targ_frame<0) heading_diff_targ_frame=2*PI+heading_diff_targ_frame;

  int lookahead_time=600;

  bool updated=false;

  double time_collision=checkFuture(lookahead_time, neigh_pos_x, neigh_pos_y, neigh_vel_mag, neigh_theta, neigh_radius);
  // if (time_collision<0) return;

 // Situation identification

  // Overtaking
  // if (!updated && neigh_ego_relative_theta<67.5*(PI/180.0) && neigh_ego_relative_theta>-67.5*(PI/180.0) && radius>neigh_radius
  if (neigh_ego_relative_theta<67.5*(PI/180.0) && neigh_ego_relative_theta>-67.5*(PI/180.0) && radius>neigh_radius 
          && (heading_diff_ego_frame<PI/4 || heading_diff_ego_frame>2*PI*3/4.0))
  {
    updated=overTakingUpdate(neigh_ego_relative_pos_x, neigh_ego_relative_pos_y, dist, neigh_radius, neigh_vel_mag, time_collision);
    // updated=true;
  } 

  // Oncoming scenario 
  if (heading_diff_ego_frame>oncoming_thresh_min && heading_diff_ego_frame<oncoming_thresh_max)
  {
    updated=oncomingUpdate(neigh_ego_relative_pos_x, neigh_ego_relative_pos_y, dist, neigh_radius, time_collision);
    // updated=true;
  }

  // Crossing
  // if (!updated && neigh_ego_relative_pos_x>0 && neigh_ego_relative_pos_y<neigh_radius && (heading_diff_ego_frame>PI*1/4.0 && heading_diff_ego_frame<PI*3/4.0))
  if (neigh_ego_relative_pos_x>0 && neigh_ego_relative_pos_y<neigh_radius && (heading_diff_ego_frame>PI*1/4.0 && heading_diff_ego_frame<PI*3/4.0))
  {
    // if (time_collision>0) printf("In crossing event collision will occur in %i timesteps\n", time_collision);
    updated=crossingUpdate(neigh_ego_relative_pos_x, neigh_ego_relative_pos_y, dist, neigh_radius, time_collision);
    // if (updated && time_collision>1) printf("In crossing event collision will occur in %i timesteps\n", time_collision);
  }

  // if (!updated && neigh_ego_relative_pos_x>-neigh_radius && dist<cmf_dist)
  // if (neigh_ego_relative_pos_x>-neigh_radius && dist<cmf_dist)
  // {
  //   double turn_angle=atan2(dist_y, dist_x);
  //   new_theta_acc=-turn_angle;
  //   updated=true;
  // }

  if (time_collision>0)
  {
    double max_crash_time=1e6;
    double offset_vel=1;
    double temp_vel_mag=(vel_mag*time_collision)/(max_crash_time*offset_vel);
    if (temp_vel_mag<new_vel_mag) new_vel_mag=temp_vel_mag;
  }
  // else
  // {
  //   double temp_vel_mag=vel_max*(dist/(range-radius-neigh_radius));
  // }
  // if (updated) min_dist=dist;

}

double NormalAgent::checkFuture(int lookahead_time, double neigh_pos_x, double neigh_pos_y, double neigh_vel_mag, double neigh_theta, double neigh_radius)
{
  b2Vec2 position=b2Body_GetPosition(getBodyID());
  double pos_x=position.x;
  double pos_y=position.y;
  double vel_x=getVelX();
  double vel_y=getVelY();

  double neigh_vel_x=cos(neigh_theta)*neigh_vel_mag;
  double neigh_vel_y=sin(neigh_theta)*neigh_vel_mag;

  double x_diff=pos_x-neigh_pos_x;
  double y_diff=pos_y-neigh_pos_y;
  double v_x_diff=vel_x-neigh_vel_x;
  double v_y_diff=vel_y-neigh_vel_y;
  double radius_joint=radius+neigh_radius;

  double a=v_x_diff*v_x_diff+v_y_diff*v_y_diff;
  double b=2*(v_x_diff*x_diff+v_y_diff*y_diff);
  double c=x_diff*x_diff+y_diff*y_diff-radius_joint*radius_joint;

  double sol0=(-b-sqrt(b*b-4*a*c))/2*a;
  double sol1=(-b+sqrt(b*b-4*a*c))/2*a;

  if (sol0>0)
  {
    // printf("\t--  time to collision:  %f\n",sol0);
    return sol0;
  } 
  else if (sol1>0) 
  {
    // printf("\t--  time to collision:  %f\n",sol1);
    return sol1;
  }
  else return -1;
}

bool NormalAgent::oncomingUpdate(double other_targ_relative_pos_x, double other_targ_relative_pos_y, double dist, double other_radius, int time_collision)
{
  if (time_collision<0) return false;
 // Get new target point
  double temp_theta_acc;

  double frame_theta=egoAngle();
  double oncoming_goal_theta=newAngleWorldFrame(other_targ_relative_pos_x, other_targ_relative_pos_y, other_radius, frame_theta);

  b2Rot rotation=b2Body_GetRotation(getBodyID());
  double heading=b2Rot_GetAngle(rotation)*(PI/180.0);
  temp_theta_acc=(oncoming_goal_theta-heading);

  if (abs(temp_theta_acc)>abs(new_theta_acc)) new_theta_acc=temp_theta_acc;
  return true;
}

bool NormalAgent::overTakingUpdate(double other_pos_x, double other_pos_y, double dist, double other_radius, double other_vel_mag, int time_collision)
{
   // If other ship is smaller, assume you can go faster and perform overtaking procedure
    if (radius>other_radius)
    {
      double frame_theta=egoAngle();
      double overtaking_goal_theta=newAngleWorldFrame(other_pos_x, other_pos_y, other_radius, frame_theta);

      b2Rot rotation=b2Body_GetRotation(getBodyID());
      double heading=b2Rot_GetAngle(rotation)*(PI/180.0);
      double temp_theta_acc=(overtaking_goal_theta-heading);

      // if (other_pos_x<radius+other_radius && other_pos_y<0) temp_theta_acc*=-1;    
      if (abs(temp_theta_acc)>abs(new_theta_acc)) new_theta_acc=temp_theta_acc;
      return true;
    }
   // If going faster than larger ship slow appropriately
    else if (new_vel_mag>other_vel_mag) 
    {
      new_vel_mag=other_vel_mag;
      // return false;
    }
    return false;
}

bool NormalAgent::makeWayUpdate(double o_relative_theta, double other_radius)
{
 // Ship is behind, let's get out of way if they are bigger
  if (radius<other_radius)
  {
    double adjusted_theta=(-PI)-o_relative_theta;
    double temp_theta_acc=-d_theta*adjusted_theta;
    if (abs(temp_theta_acc)>abs(new_theta_acc)) new_theta_acc=temp_theta_acc;
    return true;
  }
  return false;
}

bool NormalAgent::crossingUpdate(double other_pos_x, double other_pos_y, double dist, double other_radius, int time_collision)
{
  if (time_collision<0) return false;

  // double frame_theta=targAngle();
  double frame_theta=egoAngle();
  double crossing_goal_theta=newAngleWorldFrame(other_pos_x, other_pos_y, other_radius, frame_theta);
  
  b2Rot rotation=b2Body_GetRotation(getBodyID());
  double heading=b2Rot_GetAngle(rotation)*(PI/180.0);
  double temp_theta_acc=(crossing_goal_theta-heading);
  
  if (abs(temp_theta_acc)>abs(new_theta_acc)) new_theta_acc=temp_theta_acc;
  // new_theta_acc=temp_theta_acc;
  return true;
}


// template<typename t>
void NormalAgent::createNLMPC()
{
  printf("HERE0\n");
  controller->reconstructNLMPC(Nx, Nu, Ny, Nph, Nch, Nieq, Neq);
  controller->setLoggerLevel(mpc::Logger::LogLevel::NONE);
  mpc::MPPIParameters params;
  params.maximum_iteration = 1;
  params.num_rollouts = 512;
  params.sigma.resize(Nu);
  auto assignParams=[&](){return true;};
  params.sigma.setConstant(3.0);
  params.lambda = 20.0;
  params.enable_warm_start = true;
  params.state_bound_penalty = 1.0;
  params.ineq_penalty = 1.0;
  params.barrier_steepness = 1.0;
  params.eq_penalty = 0;
  params.beta = 1.0;
  params.rho = 0.4;
  params.integration_substeps = 1;

  controller->setOptimizerParameters(params);
  double ts=0.05;
  controller->setDiscretizationSamplingTime(ts);

  auto stateEqTest=[&](
                mpc::cvec<> &dX,
                const mpc::cvec<> &X,
                const mpc::cvec<> &u)
  {
      // std::cout<< "-------- Calling the state function here! --------" << std::endl;
      double theta_next=X(2)+u(1);
      dX(0) = u(0)*cos(theta_next+u(1)); // X
      dX(1) = u(0)*sin(theta_next+u(1)); // Y
      dX(2) = u(1);// theta
      // std::cout << "-------- " << X(0) << " " << X(1) << " " << X(2) << " --------\n";
      // std::cout << "-------- " << u(0) << " " << u(1) << " --------\n\n";
  };
  stateEq=stateEqTest;
  // [&](
  //               mpc::cvec<-1> &dX,
  //               const mpc::cvec<-1> &X,
  //               const mpc::cvec<-1> &u)
  // {
  //     dX(0) = u(0)*cos(theta+u(1)); // X
  //     dX(1) = u(0)*sin(theta+u(1)); // Y
  //     dX(2) = u(1);// theta
  // };
  

  auto objFuncTest=[&](const mpc::mat<-1,-1> &X,
                       const mpc::mat<-1,-1> &u)
  {
    // std::cout << "-------- Calling objective function --------\n";
    mpc::rvec<2> goal;
    goal(0)=X(0,3);
    goal(1)=X(0,4);
    // goal(1)=this->targ_y;
    double obj_val=(X(Eigen::all,Eigen::seq(0,1)).rowwise()-goal).array().square().sum();
    double cntrl_val=u.array().square().sum();
    // std::cout << "------ Values  " << X(Eigen::all,Eigen::seq(0,1)) << "\n------- " << goal(0) << std::endl;//"\n-------- " << (X(Eigen::all,Eigen::seq(0,1)).rowwise()-goal).array().square()  << "\n-------- " << (X(Eigen::all,Eigen::seq(0,1)).rowwise()-goal).array().square() << " -------\n\n" << std::endl;
    // std::cout << "------ Loss function  " << obj_val << "  " << cntrl_val << " -------\n\n" << std::endl;
    return obj_val+cntrl_val;
  };
  objFunc=objFuncTest;

  // auto initEqs=[&](t nx, t ny, t nu, t nph)
  // {
  mpc::cvec<> umin, umax;
  umin.resize(Nx);
  umax.resize(Nx);
  umin(0)=0.01;
  umin(1)=-vel_theta_max;
  umax(0)=vel_max;
  umax(1)=vel_theta_max;
  //   // // cout << vel_max << endl;
  controller->setInputBounds(umin, umax, {0, Nph});
  //   controller.setDiscretizationSamplingTime(ts);
  controller->setStateSpaceFunction([&](
                                    mpc::cvec<> &dX,
                                    const mpc::cvec<> &X,
                                    const mpc::cvec<> &u,
                                    const unsigned int &)
                                {stateEqTest(dX, X, u);});
  controller->setObjectiveFunction([&](
                                     const mpc::mat<-1,-1> &X,
                                     const mpc::mat<-1,-1> &,
                                     const mpc::mat<-1,-1> &u,
                                     const double&)
                                 {return objFuncTest(X,u);});
  // };
}

double NormalAgent::nonLinearMPC()
{
  mpc::cvec<> modelX;
  mpc::cvec<> modeldX;
  modelX.resize(Nx);
  modeldX.resize(Nx);

  b2Vec2 position=b2Body_GetPosition(getBodyID());
  
  modelX(0)=position.x;
  modelX(1)=position.y;
  modelX(2)=egoAngle();
  modelX(3)=targ_x;
  modelX(4)=targ_y;

  int batches=2;

  auto r=controller->getLastResult();
  r.cmd=mpc::cvec<2>();
  r.cmd(0)=0.5*vel_max;
  r.cmd(1)=0.0;
  std::string x_coords="";
  std::string y_coords="";
  for (int i=0; i<batches; ++i)
  {
    r=controller->optimize(modelX, r.cmd);
    auto seq=controller->getOptimalSequence();
    
    for (int j=0; j<Nph+1; ++j)
    {
      x_coords+=std::to_string(seq.state(j,0))+" ";
      y_coords+=std::to_string(seq.state(j,1))+" ";
    }

    modelX(0)=seq.state(Nph,0);
    modelX(1)=seq.state(Nph,1);
    modelX(2)=seq.state(Nph,2);
  }
  outfile.open(filename_nmpc_seq, std::ios_base::app);
  outfile << x_coords << y_coords;
  outfile << std::endl;

  outfile.close();

  return 0.0;
}


double NormalAgent::newAngleWorldFrame(double other_pos_x, double other_pos_y, double other_radius, double frame_theta)
{
  b2Vec2 position=b2Body_GetPosition(getBodyID());
  double pos_x=position.x;
  double pos_y=position.y; 

 // Get new target point
  double new_frame_goal_x=other_pos_x;
  double new_frame_goal_y=other_pos_y-2*(other_radius+radius);

  double new_goal_x=cos(-frame_theta)*new_frame_goal_x-sin(-frame_theta)*new_frame_goal_y+pos_x;
  double new_goal_y=sin(-frame_theta)*new_frame_goal_x+cos(-frame_theta)*new_frame_goal_y+pos_y;
  
  double new_goal_theta=atan2(new_goal_y, new_goal_x);

  return new_goal_theta;
}