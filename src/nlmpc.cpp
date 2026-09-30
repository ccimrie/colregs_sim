#include "nlmpc.h"
#include <functional>

using namespace std;

template<typename FnState, typename FnObj>
NLMPC<FnState,FnObj>::NLMPC(){};

template<typename FnState, typename FnObj>
template<typename t>
NLMPC<FnState,FnObj>::NLMPC(const int _nx, const int _ny, const int _nu, int _nph, int _nch, int _nieq, const int _neq, function<FnState> _stateEq, function<FnObj> _objEq)
            // function<void(mpc::cvec<>,const mpc::cvec<>,const mpc::cvec<>, const unsigned int)> _stateEq, function<void(mpc::cvec<>,mpc::cvec<>,mpc::cvec<>)> _objEq)
{
  printf("HERE\n");
  Nx=_nx;
  Ny=_ny;
  Nu=_nu;
  Nph=_nph;
  Nch=_nch; // Could be 10%-20% of prediction horizon (should double check)
  Nieq=_nieq;
  Neq=_neq;
  // double ts=0.1;
  printf("HERE0\n");
  controller.reconstructNLMPC(Nx, Nu, Ny, Nph, Nch, Nieq, Neq,mpc::OptimizerType::MPPI);

  controller.setLoggerLevel(mpc::Logger::LogLevel::NONE);
  auto createParams=[&](t sigma_test)
  {
    mpc::MPPIParameters params;
    params.maximum_iteration = 1;
    params.num_rollouts = 512;
    // consteval int Nu_temp=_nu;
    // Helper to convert runtime index to a compile-time integral constant type
    // template<size_t... Is>
    // auto make_runtime_to_compile_time_dispatched_variant(size_t index) {
    //     // Advanced dispatch logic mapping index to integral_constant
    // }

    params.sigma=mpc::cvec<sigma_test>();
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
    return params;
  };
  mpc::MPPIParameters params=createParams(_nu);
  printf("HERE1\n");
  controller.setOptimizerParameters(params);
  // printf("HERE2\n");
  // mpc::cvec<Nx> umin, umax;
  // umin(0)=0.01;
  // umin(1)=-vel_theta_max;
  // umax(0)=vel_max;
  // umax(1)=vel_theta_max;

  // // cout << vel_max << endl;

  // controller->setInputBounds(umin, umax, {0, pred_hor});
  // controller->setDiscretizationSamplingTime(ts);
  // // Defining the ODE
  // auto stateEq=[&](
  //               mpc::cvec<-1> &dX,
  //               const mpc::cvec<-1> &X,
  //               const mpc::cvec<-1> &u)
  // {
  //     dX(0) = u(0)*cos(theta+u(1)); // X
  //     dX(1) = u(0)*sin(theta+u(1)); // Y
  //     dX(2) = u(1);// theta
  // };

  // auto stateEqTest=[&](
  //               mpc::cvec<-1> &dX,
  //               const mpc::cvec<-1> &X,
  //               const mpc::cvec<-1> &u)
  // {
  //     dX(0) = u(0)*cos(theta+u(1)); // X
  //     dX(1) = u(0)*sin(theta+u(1)); // Y
  //     dX(2) = u(1);// theta
  // };
  stateEq=_stateEq;
  auto initEqs=[&](t nx, t ny, t nu, t nph)
  {

    controller.setStateSpaceFunction([&](
                                      mpc::cvec<nx> &dX,
                                      const mpc::cvec<nx> &X,
                                      const mpc::cvec<nu> &u,
                                      const unsigned int &)
                                  {stateEq(dX, X, u);});
    controller.setObjectiveFunction([&](
                                       const mpc::mat<nph+1, nx>&X,
                                       const mpc::mat<nph+1, ny> &,
                                       const mpc::mat<nph+1, nu> &u,
                                       const double&)
                                   {return objectiveFunc(X,u);});
  };
  // printf("HERE3\n");
  // // Define objective function
  // auto objectiveFunc=[&](const mpc::mat<-1, -1> &X,
  //                          const mpc::mat<-1, -1> &u)
  // {
  //   mpc::rvec<2> goal;
  //   goal(0)=targ_x;
  //   goal(1)=targ_y;
  //   double obj_val=(X(Eigen::all,Eigen::seq(0,1)).rowwise()-goal).array().square().sum();
  //   double cntrl_val=u.array().square().sum();
  //   // cout << "Loss function  " << obj_val << "  " << cntrl_val << endl;
  //   return obj_val+cntrl_val;
  // };
  // printf("HERE4\n");
}