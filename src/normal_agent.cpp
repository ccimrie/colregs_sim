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
#include <tuple>
#include "normal_agent.h"
#include <unistd.h>
#include <string>
#include <time.h>

#define PI 3.14159265
#define MPPI_ENABLED

#define NOSIT 0
#define OVERTAKING 1
#define ONCOMING 2
#define CROSSING 3

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
  // TODO: Make this part of yaml file
  Nx=6;
  Ny=3;
  Nu=2;
  Nph=20;
  Nch=2; // Could be 10%-20% of prediction horizon (should double check)
  Nieq=0;
  Neq=0;
  // controller->setLoggerLevel(mpc::Logger::LogLevel::DEEP);
  vel_max*=5;
  createNLMPC();

}

void NormalAgent::recordNeighbour(agent* neighbour)
{
 // // Extract useful information from self/ego
  b2Vec2 position=b2Body_GetPosition(getBodyID());
 //  double pos_x=position.x;
 //  double pos_y=position.y; 
 //  b2Rot rotation=b2Body_GetRotation(getBodyID());
 //  double heading=b2Rot_GetAngle(rotation);

 // Extract useful information from neighbour
  b2Vec2 neigh_position=b2Body_GetPosition(neighbour->getBodyID());
  // double neigh_pos_x=neigh_position.x;
  // double neigh_pos_y=neigh_position.y; 
  b2Rot neigh_rotation=b2Body_GetRotation(neighbour->getBodyID());
  // double neigh_heading=b2Rot_GetAngle(neigh_rotation);

  neigh_info neigh;

 // Keep in world frame, convert to other frames where appropriate
  neigh.x=neigh_position.x;
  neigh.y=neigh_position.y;
  neigh.heading=b2Rot_GetAngle(neigh_rotation);
  neigh.targ_theta=atan2(neigh.y-position.y,neigh.x-position.x);
  neigh.targ_theta=atan2(neigh.y, neigh.x);
  neigh.v_lin=neighbour->getVelMag();
  neigh.radius=neighbour->getRadius();
  neighbours.push_back(neigh);

 // shift neighbour into ego frame
  // double diff_x=neigh_pos_x-pos_x;
  // double diff_y=neigh_pos_y-pos_y;
  // neigh.x=(cos(-heading)*diff_x-sin(-heading)*diff_y);
  // neigh.y=(sin(-heading)*diff_x+cos(-heading)*diff_y);
  // double neigh_heading_norm=fmod((neigh_heading-heading),2*PI);
  // if (neigh_heading_norm>PI) neigh_heading_norm=-2*PI+neigh_heading_norm;
  // else if (neigh_heading_norm<-PI) neigh_heading_norm=2*PI+neigh_heading_norm;
  // neigh.heading=neigh_heading_norm;
  // neigh.targ_theta=atan2(neigh.y, neigh.x);
  // neigh.v_lin=neighbour->getVelMag();
  // neigh.radius=neighbour->getRadius();
  // neighbours.push_back(neigh);
}


void NormalAgent::updateVel()
{

  if (neighbours.size()<1)
  {
    double _theta_acc=goalTheta();
    setVelMag(vel_max); // To do replace with something smarter when turning
    setThetaAcc(_theta_acc);
    // std::cout << vel_max << " " << getVelMag() << std::endl;
    b2Vec2 position=b2Body_GetPosition(getBodyID());
    double pos_x=position.x;
    double pos_y=position.y; 
    int batches=1;
    std::string x_coords="";
    std::string y_coords="";
    for (int i=0; i<batches; ++i)
    {    
      for (int j=0; j<Nph+1; ++j)
      {
        x_coords+=std::to_string(pos_x)+" ";
        y_coords+=std::to_string(pos_y)+" ";
      }
      outfile.open(filename_nmpc_seq, std::ios_base::app);
      outfile << x_coords << y_coords;
      outfile << std::endl;
      outfile.close();
    }
    return;
  }

 // Extract useful information from self/ego
  b2Vec2 position=b2Body_GetPosition(getBodyID());
  // double pos_x=position.x;
  // double pos_y=position.y; 
  b2Rot rotation=b2Body_GetRotation(getBodyID());
  double heading=b2Rot_GetAngle(rotation);
  double targ_theta=goalTheta();

  /*
    1. Identify situation of each neighbour

    2. Idenitfy "key ship"; the ship that poses the most immediate danger
      - Ship with smallest TTC
      - Should probably use something like this instead: ship furthest right if collision occurs?

    3. Calculate NLMPC  with all neighbours position and key ship situation
      3.1 Overtaking: additional constraint of keeping to starboard
      3.2 Oncoming: Keep to starboard
      3.3 Crossing (give way): Go behind neighbour ship
  */

  int situation=NOSIT;
  double max_crash_time=1e6;
  double TTC=max_crash_time;

  for (auto it=neighbours.begin(); it!=neighbours.end(); ++it)
  {
    bool ttc_swap=false;
    bool sit_swap=false;
    double diff_x=it->x-position.x;
    double diff_y=it->y-position.y;
    double dist=sqrt(diff_x*diff_x+diff_y*diff_y);
    dist-=(radius+it->radius);

    std::tuple<double,double> cpa=checkFuture(it->x, it->y, it->v_lin, it->heading, it->radius);
    double time_collision=std::get<0>(cpa);
    if (time_collision<0)
    {
      printf("Need to check TCPA calculations\n");
      std::exit(0);
    }
    double dcpa=std::get<1>(cpa);
    // Key ship will be placed at start of vector
    //  - Note: worth sorting by TTC?
    if (dcpa==0 && time_collision<TTC) TTC=time_collision, ttc_swap=true;
    // current_ind++;

    // double heading_goal_frame=atan2(pos_y-targ_y,pos_x-targ_x);
    // Ego frame but pointing towards goal
    double neigh_x_targ_frame=(it->x-position.x)*cos(-(goalTheta()+heading))-(it->y-position.y)*sin(-(goalTheta()+heading));
    double neigh_y_targ_frame=(it->y-position.y)*cos(-(goalTheta()+heading))+(it->x-position.x)*sin(-(goalTheta()+heading));
    double heading_targ_frame=it->heading-(goalTheta()+heading);
    heading_targ_frame=atan2(sin(heading_targ_frame),cos(heading_targ_frame));
    // printf("Switched to ego frame pointed to goal:  %f  %f  %f\n", neigh_x_ego_frame, neigh_y_ego_frame, neigh_heading_ego_frame);

    // double neigh_heading_ego_norm=fmod(it->heading-heading, 2*PI);
    // if (neigh_heading_ego_norm>PI)
    // {
    //   neigh_heading_ego_norm=neigh_heading_ego_norm-2*PI;
    // }
    // else if (neigh_heading_ego_norm<PI)
    // {
    //   neigh_heading_ego_norm=neigh_heading_ego_norm+2*PI;
    // }

   // Should this be done in the goal frame? This ensures that the crossing is consistently detected and not changed as ego updates
   // Check situation:
    // Oncoming
    // if (it->heading>2*PI*(3/8.0) && it->heading<2*PI*(5/8.0)) 

    double oncoming_thresh=PI*(174.0/180.0);
    double heading_ego_frame=atan2(sin(it->heading-heading),cos(it->heading-heading));
    
    double overtaking_thresh=PI*(45/180.0);
    double abaft_beam_threshold=PI*((90+22.5)/180.0);
    double angle_sector=atan2(position.y-it->y, position.x-it->x);
    double angle_sector_neigh_frame=atan2(sin(angle_sector-it->heading),cos(angle_sector-it->heading));
    // if ((it->x-position.x)*cos(-heading)+(it->y-position.y)*sin(-heading)>0)
    // {
    //   printf("Oncoming - first condition met\n");
    // }
    // if (heading_ego_frame>oncoming_thresh)
    // {
    //   printf("Oncoming - second condition met  %f  %f\n", heading_ego_frame, oncoming_thresh);
    // }
    // if (heading_ego_frame<-oncoming_thresh)
    // {
    //   printf("Oncoming - third condition met\n");
    // }

    // if (it->heading>PI || it->heading<-PI) printf("neighbour angle wrapping required"), std::exit(0);
    // if (heading>PI || heading<-PI) printf("ego angle wrapping required"), std::exit(0);


    if ((it->x-position.x)*cos(-heading)+(it->y-position.y)*sin(-heading)>0 &&
        (heading_ego_frame>oncoming_thresh || heading_ego_frame<-oncoming_thresh)) 
    { 
      sit_swap=true;
      situation=ONCOMING;
      printf("Oncoming situation!\n");
      // std::exit(0);
    }
    // Overtaking
    /* Check for:
        - Is ego in overtaking sector
        - Is ego going faster
        - Are they going in same direction as ego's goal
        - Are we not currently dealing with an oncoming
    */
    // else if ((it->x-position.x)*cos(-heading)+(it->y-position.y)*sin(-heading)>0 &&
            // it->heading-heading<PI/8 && it->heading-heading>-PI/8 && 
    else if ((angle_sector_neigh_frame>abaft_beam_threshold || angle_sector_neigh_frame<-abaft_beam_threshold) && // - Is ego in overtaking sector
            (heading_ego_frame-goalTheta()<overtaking_thresh && heading_ego_frame-goalTheta()>-overtaking_thresh) && // Is neighbour going same direction as goal
            vel_mag>it->v_lin && // Is ego going faster
            situation!=ONCOMING) // Not dealing with oncoming (TO DO: break loop early if detected oncoming, and trying to sort which oncoming to deal with first)
    {
      sit_swap=true;
      situation=OVERTAKING;
      printf("Overtaking situation!\n");
    }
    // Crossing (give way)
    else if (neigh_x_targ_frame>0 && neigh_y_targ_frame<it->radius && (heading_targ_frame>PI*1/4.0 && heading_targ_frame<PI*3/4.0) 
      && situation!=ONCOMING)
    {
      sit_swap=true;
      situation=CROSSING;
      printf("Crossing situation!\n");
    }

    if (ttc_swap && !sit_swap) situation=NOSIT;

    if (sit_swap || ttc_swap) iter_swap(neighbours.begin(),it);
    // else if (it->x>0 && it->y<it->radius && (it->heading-targ_theta>PI*1/4.0 && it->heading-targ_theta<PI*3/4.0)) situation=CROSSING;
  }

 // Go towards goal if no collision will occur or if no situation
  // std::cout << TTC << " " << max_crash_time << " " << (TTC==max_crash_time) <<std::endl;
  // if (situation==OVERTAKING) std::cout<<"Okay...\n";
  if (TTC==max_crash_time && situation==NOSIT)
  {
    int batches=1;
    std::string x_coords="";
    std::string y_coords="";
    for (int i=0; i<batches; ++i)
    {    
      for (int j=0; j<Nph+1; ++j)
      {
        x_coords+=std::to_string(position.x)+" ";
        y_coords+=std::to_string(position.y)+" ";
      }
      outfile.open(filename_nmpc_seq, std::ios_base::app);
      outfile << x_coords << y_coords;
      outfile << std::endl;
      outfile.close();
    }
    // double _theta_acc=goalTheta();
    setThetaAcc(targ_theta);
    neighbours.clear();
    return;
  }
  // if (situation==OVERTAKING) std::cout<<"Okay 2...\n";
 /* Convert neighbours into global frame and mpc::cvec for inequality constraints
      - X
      - Y
      - heading (for including into model X)
      - Lin velocity
        - Note: Might need to adapt state X as well to include how neighbours change over time 
 */
  mpc::mat<> neigh_model;
  // std::cout<<"Made matrix\n";
  neigh_model.resize(neighbours.size(),5);
  // std::cout<<"Resized matrix\n";
  for (int i=0; i<neighbours.size(); ++i)
  {
    neigh_model(i,0)=neighbours[i].x;
    neigh_model(i,1)=neighbours[i].y;
    neigh_model(i,2)=neighbours[i].heading;
    neigh_model(i,3)=neighbours[i].v_lin;
    neigh_model(i,4)=neighbours[i].radius;
  }
  // std::cout<<"Added neighbours to model\n";
  nonLinearMPC(neigh_model, situation);
  neighbours.clear();
}


std::tuple<double,double> NormalAgent::checkFuture(double neigh_pos_x, double neigh_pos_y, double neigh_vel_mag, double neigh_theta, double neigh_radius)
{
  // Calculate in world frame
  b2Vec2 position=b2Body_GetPosition(getBodyID());
  // double pos_x=position.x;
  // double pos_y=position.y;
  double vel_x=getVelX();
  double vel_y=getVelY();

  double neigh_vel_x=cos(neigh_theta)*neigh_vel_mag;
  double neigh_vel_y=sin(neigh_theta)*neigh_vel_mag;

  double x_diff=position.x-neigh_pos_x;
  double y_diff=position.y-neigh_pos_y;
  double v_x_diff=vel_x-neigh_vel_x;
  double v_y_diff=vel_y-neigh_vel_y;
  double radius_joint=radius+neigh_radius;

  double a=v_x_diff*v_x_diff+v_y_diff*v_y_diff;
  double b=2*(v_x_diff*x_diff+v_y_diff*y_diff);
  double c=x_diff*x_diff+y_diff*y_diff-radius_joint*radius_joint;

  double sol0=(-b-sqrt(b*b-4*a*c))/2*a;
  double sol1=(-b+sqrt(b*b-4*a*c))/2*a;


  if (sol0>0 && sol0<sol1)
  {
    // printf("\t--  time to collision:  %f   %f\n",sol0, sol1);
    return {sol0,0};
  } 
  else if (sol1>0 && sol1<sol0) 
  {
    // printf("\t--  time to collision:  %f   %f\n",sol1, sol0);
    return {sol1,0};
  }
  else
  {
    double tcpa=-b/a;
    if (tcpa<0) tcpa=0;
    double dcpa=tcpa*tcpa*a+tcpa*b+c;
    // printf("(TCPA, DCPA) = (%f, %f)\n",tcpa, dcpa);
    return {tcpa,dcpa};
  } 
}


void NormalAgent::setMPPIParams()
{
  mpc::MPPIParameters params;
  params.maximum_iteration = 1;
  params.num_rollouts = 2048;
  params.sigma.resize(Nu);
  auto assignParams=[&](){return true;};
  params.sigma.setConstant(5.0);
  params.lambda = 5.0;
  params.enable_warm_start = true;
  params.state_bound_penalty = 0.0;
  params.ineq_penalty = 100.0;
  params.barrier_steepness = 1.0;
  params.eq_penalty = 0;
  params.beta = 1.0;
  params.rho = 0.4;
  params.integration_substeps = 10;

  controller->setOptimizerParameters(params);
  mpc::cvec<> umin, umax;
  umin.resize(Nu);
  umax.resize(Nu);
  umin(0)=0.01;
  umax(0)=vel_max;
  umin(1)=-vel_theta_max;
  umax(1)=vel_theta_max;
  //   // // cout << vel_max << endl;
  controller->setInputBounds(umin, umax, {0, Nch});
  double ts=0.1;
  controller->setDiscretizationSamplingTime(ts);
}


void NormalAgent::createNLMPC()
{
  controller->reconstructNLMPC(Nx, Nu, Ny, Nph, Nch, Nieq, Neq);
  controller->setLoggerLevel(mpc::Logger::LogLevel::NONE);
  setMPPIParams();

  auto stateEqTest=[&](
                mpc::cvec<> &dX,
                const mpc::cvec<> &X,
                const mpc::cvec<> &u)
  {
    // std::cout<< "-------- Calling the state function here! --------" << std::endl;
    double theta_next=X(2)+u(1);
    dX(0) = u(0)*cos(theta_next); // X
    dX(1) = u(0)*sin(theta_next); // Y
    dX(2) = u(1);// theta
    // std::cout << "-------- " << X(0) << " " << X(1) << " " << X(2) << " --------\n";
    // std::cout << "-------- " << u(0) << " " << u(1) << " --------\n\n";
    // std::cout << dX.transpose() << std::endl;
  };

  auto objFuncTest=[&](const mpc::mat<-1,-1> &X,
                       const mpc::mat<-1,-1> &u)
  {
    // std::cout << "-------- Calling objective function --------\n";
    mpc::rvec<2> goal;
    goal(0)=X(0,4);
    goal(1)=X(0,5);
    // goal(1)=this->targ_y;

    mpc::mat x_diffs=(X(Eigen::all,Eigen::seq(0,1)).rowwise()-goal).array().square();
    mpc::mat x_diffs_sum=x_diffs(Eigen::all,0)+x_diffs(Eigen::all,1);
    mpc::mat x_dist=x_diffs_sum.array().sqrt();

    mpc::mat u_mags=u.array().square();
    mpc::mat u_mags_sum=u_mags(Eigen::all,0)+u_mags(Eigen::all,1);
    // mpc::mat u_dist=x_diffs_sum.array().sqrt();

    double obj_val=x_dist.sum();
    double cntrl_val=(u_mags_sum.array().sqrt()).sum();
    // std::cout << "------ Values  " << X(Eigen::all,Eigen::seq(0,1)) << "\n------- " << goal(0) << std::endl;//"\n-------- " << (X(Eigen::all,Eigen::seq(0,1)).rowwise()-goal).array().square()  << "\n-------- " << (X(Eigen::all,Eigen::seq(0,1)).rowwise()-goal).array().square() << " -------\n\n" << std::endl;
    // std::cout << "------ Loss function  " << obj_val << "  " << cntrl_val << " -------\n\n" << std::endl;
    return obj_val+cntrl_val;
  };

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
}


// double NormalAgent::nonLinearMPC(std::function<void(mpc::cvec<> &in_con, const mpc::mat<> &u)> ineqFunc)
double NormalAgent::nonLinearMPC(mpc::mat<> neigh_model, int situation)
{

 /* Inequality function will need to include:
      - Keeping away from other ships
      - Adhereing to COLREG (mostly keeping to starboard?)
 */

  // TODO: add check if neighbours have changed; if not, no need to reconstruct
  // std::cout << "NLMPC...\n";
  // sleep(10);
  // if (neigh_model.rows()>0)
  // { 
    // std::cout << "Reconstructing with neighbours...\n";
  /* Number of inequalities:
    - (number of neighbours+1) * prediction horizon
  */
  int sit_add=situation>NOSIT ? 1 : 0;
  Nieq=(neigh_model.rows()+sit_add)*(Nph+1);
  controller->reconstructNLMPC(Nx+neigh_model.rows()*5, Nu, Ny, Nph, Nch, Nieq, Neq);
  // printf("sit add and neigh model size: %i  %i  %i\n", sit_add, neigh_model.rows(), Nieq);

  auto ineqFunc=[&neigh_model, situation](
                 mpc::cvec<> &in_con,
                 const mpc::mat<> &X,
                 // const mpc::mat<> &,
                 const mpc::mat<> &u)
                 // const double &)
            {
             // Constraints for not hitting any other ship
              int neigh_count=(X.cols()-6)/5;
              // std::cout<<"\t Inside function...\n";
              for (int i=0; i<X.rows(); ++i)
              {
                // printf("\t- Acquiring for time %i of %li\n", i, X.rows());
                for (int j=0; j<neigh_count; ++j)
                {
                  // printf("\t\t- Getting neighbour %i of %i\n", j+1, neigh_count);
                  double diff_x=X(i,0)-X(i,6+5*j);
                  double diff_y=X(i,1)-X(i,7+5*j);
                  in_con(j+neigh_count*i)=X(i,3)+X(i,10+5*j)-sqrt(diff_x*diff_x+diff_y*diff_y); // radius+neigh_radius < (distance between agents)**2
                  if (X(i,3)+X(i,10+5*j)<0) 
                  {
                    printf("Something wrong with radius'...\n");
                    std::cout<<X<<std::endl;
                    std::exit(0);
                  }
                  if (in_con(j+neigh_count*i)>650)
                  {
                    printf("Inequality cost of distance: \n");
                    printf("\t- x difference: %f\n\t- y difference: %f\n\t- Joint radius size: %f\n\t- Distance:  %f\n", diff_x,diff_y,X(i,3)+X(i,10+5*j),sqrt(diff_x*diff_x+diff_y*diff_y));
                    std::exit(0);
                  }
                }
              }
              // printf("Acquired for distances\n");
             // Constraint for situation
              int shift=X.rows()*neigh_count;
              switch(situation)
              {
                case NOSIT:
                  printf("No situation!\n");
                  // printf("Inequalitities with no situation and %i neighbours: \n",neigh_count);
                  break;
                case ONCOMING:
                  // printf("\t- Setting for rows (%li-%li); inequality vector total size=%li\n", X.rows()*neigh_count, in_con.rows()-1, in_con.rows());
                  for (int i=0; i<X.rows(); ++i) in_con(i+shift)=u(i,1); // angular velocity<0 i.e. always turn right
                  if (in_con(shift)==0)
                  {
                    printf("Inequalitities with oncoming situation and %i neighbours: \n",neigh_count);
                    std::cout << in_con.transpose() << std::endl;
                    std::exit(0);
                  }
                  break;
                case OVERTAKING:
                  // printf("\t- Setting for rows (%li-%li); inequality vector total size=%li\n", X.rows()*neigh_count, in_con.rows()-1, in_con.rows());
                  for (int i=0; i<X.rows(); ++i) in_con(i+shift)=X(i,9)-u(i,0); // neigh velocity < ego velocity
                  if (in_con(shift)==0)
                  {
                    printf("Inequalitities with overtaking situation and %i neighbours: \n",neigh_count);
                    std::cout << in_con.transpose() << std::endl;
                    std::exit(0);
                  }
                  break;
                case CROSSING:
                  double neigh_vel=X(0,9);
                  if (neigh_vel==0) neigh_vel=0.01;
                  double neigh_vx=neigh_vel*cos(X(0,8));
                  double neigh_vy=neigh_vel*sin(X(0,8));
                  double unit_vx=neigh_vx/(neigh_vx+neigh_vy);
                  double unit_vy=neigh_vy/(neigh_vx+neigh_vy);
                  double dist_min=2.0*(X(0,3)+X(0,10)); // distance threshold = ego radius + neighbourgh's radius
                  

                  double neigh_start_x=X(0,6);
                  double neigh_end_x=X(X.rows()-1,6);
                  double neigh_start_y=X(0,7);
                  double neigh_end_y=X(X.rows()-1,7);

                  for (int i=shift; i<in_con.rows(); ++i)
                  {
                    int ph_t=i-(shift);
                    double proj_length=(X(0,6)-X(ph_t,0))*unit_vx+(X(0,7)-X(ph_t,1))*unit_vy;
                    double x_dist=X(0,6)-X(ph_t,0)-proj_length*unit_vx;
                    double y_dist=X(0,7)-X(ph_t,1)-proj_length*unit_vy;
                    double final_value=dist_min-sqrt(x_dist*x_dist+y_dist*y_dist);
                    
                    // double dist_2=(neigh_end_y-neigh_start_y)*X(ph_t,0)-(neigh_end_x-neigh_start_x)*X(ph_t,1)+neigh_end_x*neigh_start_y-neigh_end_y*neigh_start_x;
                    // dist_2=abs(dist_2)/(sqrt((neigh_end_y-neigh_start_y)*(neigh_end_y-neigh_start_y)+(neigh_end_x-neigh_start_x)*(neigh_end_x-neigh_start_x)));
                    // printf("Crossing distance constraint:  %f   %f\n", dist_2, sqrt(x_dist*x_dist+y_dist*y_dist));
                    // if (!std::isfinite(final_value))
                    // {
                    //   std::cout << X << std::endl;
                    //   // std::cout << X.transpose() << std::endl;
                    //   std::cout << "Minimum distance:  "<< X(ph_t,3) << " " << X(ph_t,10) << std::endl;
                    //   std::cout << "Checking crossing values:  "<< ph_t << " " << neigh_vy << " " << neigh_vx << std::endl;
                    //   std::cout << "Checking correct model vals:  "<< X(ph_t,0) << " " << X(ph_t,1) << " " << X(ph_t,7) << " " << X(ph_t,6) << std::endl;
                    //   std::cout << "Getting velocity parts:  " << (neigh_vx) << " " << (1/neigh_vx) << " " << (neigh_vy) << " " << (-1/neigh_vy) <<std::endl;
                    //   std::cout << "Final value:  " << dist_min-sqrt(x_dist*x_dist+y_dist*y_dist) << std::endl;
                    //   std::exit(0);
                    // }
                    if (final_value>0) printf("Violation!\n");
                    in_con(i)=final_value;
                    if (final_value>650)
                    {
                      printf("Issue with crossing inequality:  \n");
                      printf("\t- Projected length:  %f\n\t- x distance:  %f\n\t- y distance:  %f\n\t- Minimum distance:  %f\n\t- Distance:  %f\n", proj_length, x_dist, y_dist, dist_min, sqrt(x_dist*x_dist+y_dist*y_dist));
                    }
                    // dist_min-((1/neigh_vx)*X(ph_t,0)+(-1/neigh_vy)*X(ph_t,1)+(X(ph_t,7)-X(ph_t,6))); // distance threshold < distance to neighbour's trajectory
                  }
                  if (in_con(shift)==0)
                  {
                    printf("Inequalitities with crossing situation and %i neighbours: \n", neigh_count);
                    std::cout << in_con.transpose() << std::endl << std::endl;
                    // std::exit(0);
                  }
                  break;
              }
              for (int i=0; i<in_con.size(); ++i)
              {
                if (in_con(i)>650)
                {
                  printf("Large inequality:  \n");
                  printf("\t- Situation:  %i\n\t- Inequality index/value:  %i/%f\n", situation, i, in_con(i));
                  std::exit(0);
                }
              }
              // if (in_con(shift)==0)
              // {
              //   std::cout << in_con.transpose() << std::endl << std::endl;
              //   std::exit(0);
              // }
              // printf("Inequalities done!\n");
            };
  
  controller->setIneqConFunction([&](
                               mpc::cvec<> &in_con,
                               const mpc::mat<> &X,
                               const mpc::mat<> &,
                               const mpc::mat<> &u,
                               const double &)
                           {ineqFunc(in_con, X, u);});


  auto stateEqTest=[&](
                mpc::cvec<> &dX,
                const mpc::cvec<> &X,
                const mpc::cvec<> &u)
  {
    // std::cout<< "-------- Calling the state function here! --------" << std::endl;


    bool collided=false;
    for (int i=0; i<(dX.rows()-6)/5; ++i)
    {
      double x_diff=X(0)-X(6+5*i);
      double y_diff=X(1)-X(7+5*i);
      double dist_min=X(3)+X(10+5*i);
      // std::cout<<"Doing value: " << i << std::endl;
      if (x_diff*x_diff+y_diff*y_diff>dist_min*dist_min)
      {
        if (6+5*i==10 || 7+5*i==10)
        {
          printf("Indexing gone wrong...");
          std::exit(0);
        }
        dX(6+5*i)=X(9+5*i)*cos(X(8+5*i));
        dX(7+5*i)=X(9+5*i)*sin(X(8+5*i));
        dX(8+5*i)=0.0;
        dX(9+5*i)=0.0;
        dX(10+5*i)=0.0;
      }
      else
      {
        dX(6+5*i)=0;
        dX(7+5*i)=0;
        dX(8+5*i)=0.0;
        dX(9+5*i)=0.0;
        dX(10+5*i)=0.0;
        collided=true;
      }
      if (dX(10+5*i)!=0.0) printf("change of radius:  %f\n", dX(10+5*i));
    }
    // std::cout << dX.transpose() << std::endl;

    if(!collided) 
    {
      double theta_next=X(2)+u(1);
      // std::cout << "Value of X(2) and u(1):  " << X(2) << " " <<u(1)<<std::endl;
      // std::cout << "Value of X(2):  " << X(2)<<std::endl;
      dX(0) = u(0)*cos(theta_next); // X
      dX(1) = u(0)*sin(theta_next); // Y
      dX(2) = u(1);// theta
    }
    else
    {
      // printf("Collision\n");
      dX(0)=0; // X
      dX(1)=0; // Y
      dX(2)=0;// theta
    }
    dX(3)=0.0;
    dX(4)=0.0;
    dX(5)=0.0;

    // for (int i=0; i<dX.rows(); ++i)
    // {
    //   if (dX(i)!=dX(i))
    //   {
    //     std::cout << "Found issue with state update:  "<< i << " " << theta_next << " " << u(1) << " " <<std::endl;
    //     // std::exit(0);
    //   }
    // }


    // if (dX(0)!=dX(0) || dX(1)!=dX(1))
    // {
    //   std::cout << "Issue found:  " << X(2) << " " << u(1) << " " << dX(0) << " " << dX(1) << std::endl;
    //   std::exit(0);
    // }
    // For 5+ then do update on agent 
    // std::cout << "-------- " << X(0) << " " << X(1) << " " << X(2) << " --------\n";
    // std::cout << "-------- " << u(0) << " " << u(1) << " --------\n\n";
  };
  controller->setStateSpaceFunction([&](
                                    mpc::cvec<> &dX,
                                    const mpc::cvec<> &X,
                                    const mpc::cvec<> &u,
                                    const unsigned int &)
                                {stateEqTest(dX, X, u);});
  
  setMPPIParams();
    // std::cout << "Successful!\n";
  // }
  // else 
  // {
  //   controller->reconstructNLMPC(Nx, Nu, Ny, Nph, Nch, Nieq, Neq);
  //   setMPPIParams();
  // }
  // std::cout << "Done\n";

  mpc::cvec<> modelX;
  mpc::cvec<> modeldX;
  int new_size=Nx+neigh_model.rows()*5;
  modelX.resize(new_size);
  modeldX.resize(new_size);

  b2Vec2 position=b2Body_GetPosition(getBodyID());
  b2Rot rotation=b2Body_GetRotation(getBodyID());
  // if (neigh_model.rows()>1)  sleep(3),std::cout << "Init X:  " << modelX.transpose() << std::endl;
  modelX(0)=position.x;
  modelX(1)=position.y;
  modelX(2)=egoAngle();
  modelX(3)=radius;
  modelX(4)=targ_x;
  // (targ_x-position.x)*cos(-egoAngle())-(targ_y-position.y)*sin(-egoAngle());
  modelX(5)=targ_y;
  // (targ_y-position.y)*cos(-egoAngle())+(targ_x-position.x)*sin(-egoAngle());

  // if (neigh_model.rows()>1)  sleep(3),std::cout << "First X:  " << modelX.transpose() << std::endl;

  for (int i=0; i<neigh_model.rows(); ++i)
  {
    modelX(6+5*i)=neigh_model(i,0);
    modelX(7+5*i)=neigh_model(i,1);
    modelX(8+5*i)=neigh_model(i,2);
    modelX(9+5*i)=neigh_model(i,3);
    modelX(10+5*i)=neigh_model(i,4);
  }
  // if (neigh_model.rows()>1) sleep(3),std::cout << "All X:  " << modelX.transpose() << std::endl << std::endl;

  int batches=1;

  auto r=(*controller).getLastResult();
  // std::cout << "Got last result\n";
  if (r.cmd.size()<Nu)
  {
    r.cmd=mpc::cvec<2>();
    r.cmd(0)=vel_max;
    r.cmd(1)=0.0;
  }
  std::string x_coords="";
  std::string y_coords="";
  // clock_t clkStart;
  // clock_t clkFinish;
  for (int i=0; i<batches; ++i)
  {    
    // std::cout<<"Optimizing..."<<std::endl;
    // clkStart = clock();
    r=(*controller).optimize(modelX, r.cmd);
    // clkFinish = clock();
    // std::cout << "\t\t--- Optimize time: " << clkFinish - clkStart<<"\n\n";
    // std::cout<<"Optimized"<<std::endl;
 
    // clkStart = clock();
    auto seq=(*controller).getOptimalSequence();
    // clkFinish = clock();
    for (int j=0; j<Nph+1; ++j)
    {
      double x=seq.state(j,0);//*cos(egoAngle())-seq.state(j,1)*sin(egoAngle())+position.x;
      double y=seq.state(j,1);//*cos(egoAngle())+seq.state(j,0)*sin(egoAngle())+position.y;
      x_coords+=std::to_string(x)+" ";
      y_coords+=std::to_string(y)+" ";
    }
    // modelX(0)=seq.state(Nph,0);
    // modelX(1)=seq.state(Nph,1);
    // modelX(2)=seq.state(Nph,2);
    // modelX(3)=radius;
    // modelX(4)=targ_x;
    // modelX(5)=targ_y;
    // std::cout << modelX << std::endl;
  }
  outfile.open(filename_nmpc_seq, std::ios_base::app);
  outfile << x_coords << y_coords;
  outfile << std::endl;
  outfile.close();

  if (r.cmd(0)!=r.cmd(0) || r.cmd(1)!=r.cmd(1)) 
  {
    if (situation==OVERTAKING) std::cout<<"Overtaking commands: " << r.cmd(0) << " " << r.cmd(1) << " " << vel_max << " " << vel_theta_max << "\n\n";
    else if (situation==ONCOMING) std::cout<<"Oncoming commands: " << r.cmd(0) << " " << r.cmd(1) << " " << vel_max << " " << vel_theta_max << "\n\n";
    else if (situation==CROSSING) std::cout<<"Crossing commands: " << r.cmd(0) << " " << r.cmd(1) << " " << vel_max << " " << vel_theta_max << "\n\n";
    else if (situation==NOSIT) std::cout<<"No situation commands: " << r.cmd(0) << " " << r.cmd(1) << " " << vel_max << " " << vel_theta_max << "\n\n";
    std::exit(0);
  }
  

  setVelMag(r.cmd(0));
  setThetaAcc(r.cmd(1));

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