#include "box2d/box2d.h"
#include "yaml-cpp/yaml.h"
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
#include "mpc/NLMPC.hpp"
// #include <mpc/LMPC.hpp>
// #include "LMPC.hpp"
// #include <LMPC.hpp>
// #include <armadillo>
#include <tuple>
#include "agent.h"
// #include "nlmpc.h"

struct neigh_info
{
  float x;
  float y;
  float heading;
  float targ_theta;
  float v_lin;
  float v_ang;
  float radius;

};

constexpr int placeholder=1;

class NormalAgent: public agent 
{
  public:
    NormalAgent();
    NormalAgent(std::string yaml_file, std::string results_file, double seed);
    
    void updateVel();
  
    double nonLinearMPC(mpc::mat<> neigh_model, int situation);

    std::tuple<double,double> checkFuture(double neigh_pos_x, double neigh_pos_y, double neigh_vel_mag, double neigh_theta, double neigh_radius);
    bool oncomingUpdate(double other_targ_relative_pos_x, double other_target_relative_pos_y, double dist, double other_radius, int time_collision);
    bool overTakingUpdate(double o_pos_x, double o_pos_y, double dist, double o_radius, double other_vel_mag, int time_collision);
    bool makeWayUpdate(double o_relative_theta, double other_radius);
    bool crossingUpdate(double other_relative_pos_x, double other_relative_pos_y, double dist, double other_radius, int time_collision);

    double newAngleWorldFrame(double other_pos_x, double other_pos_y, double other_radius, double frame_theta);
    // double targAngle();
    // double egoAngle();

    // template<typename t>
    void createNLMPC();
    void setMPPIParams();

    void recordNeighbour(agent* neighbour);

   // MPC variables
  private:
    std::string filename_nmpc_seq;
    int Nx;
    int Ny;
    int Nu;
    int Nph;
    int Nch;
    int Nieq;
    int Neq;
    mpc::NLMPC<> *controller=new mpc::NLMPC(placeholder,placeholder,placeholder,placeholder,placeholder,placeholder,placeholder,mpc::OptimizerType::MPPI);
    // double ts = 0.1;
    std::function<void(mpc::cvec<>&, const mpc::cvec<>&,const mpc::cvec<>&)> stateEq;
    std::function<double(const mpc::mat<>&,const mpc::mat<>&)> objFunc;

    std::vector<neigh_info> neighbours;
};
