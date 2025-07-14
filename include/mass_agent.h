#ifndef AGENT_H
#define AGENT_H
#include "agent.h"
#endif

class MassAgent : public agent
{
  public:
    MassAgent();
    MassAgent(std::string yaml_file, std::string results_file, double seed, double _goal_weight, double _neigh_weight, double _safety_bubble) : agent(yaml_file, results_file, seed){goal_weight=_goal_weight, neigh_weight=_neigh_weight, safety_bubble=_safety_bubble;};
    void updateVel();
    void updateNeighPF(agent* neighbour);
    double getGoalDist(double pos_x, double pos_y);
  private:
    double goal_weight;
    double neigh_weight;
    double pf_x_neigh;
    double pf_y_neigh;
    double safety_bubble;
};