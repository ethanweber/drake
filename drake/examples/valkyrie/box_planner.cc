

#include "drake/examples/valkyrie/box_planner.h"


void BoxPlanner::BoxPlanner(const RigidBodyTree<double>* robot,
           const std::string& alias_groups_file_name,
           const std::string& param_file_name, double dt)
           : PlanEvalBaseSystem(robot, alias_groups_file_name, param_file_name, dt)
{

}


void BoxPlanner::DoExtendedCalcUnrestrictedUpdate(
        const systems::Context<double>& context,
        systems::State<double>* state) const
{
  
}
