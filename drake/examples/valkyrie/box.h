#pragma once

// creating box val state lcm types
#include "lcmtypes/bot_core/robot_state_t.hpp"
#include "lcmtypes/bot_core/position_3d_t.hpp"
#include "lcmtypes/bot_core/vector_3d_t.hpp"

#include "lcm/lcm-cpp.hpp"
#include <iostream>

#include "drake/manipulation/util/robot_state_msg_translator.h"
#include "drake/multibody/rigid_body_tree.h"

#include <utility>

#include "drake/common/eigen_types.h"

// for the urdf parser
#include "drake/common/find_resource.h"
#include "drake/multibody/parsers/urdf_parser.h"

#include "drake/util/drakeUtil.h"

#include "drake/multibody/kinematics_cache.h"

#include "bot_core/atlas_command_t.hpp"

#include "drake/multibody/rigid_body_tree_alias_groups.h"
#include "drake/systems/controllers/plan_eval/generic_plan.h"
#include "drake/systems/controllers/qp_inverse_dynamics/param_parser.h"

// requires --config gurobi when running with bazel
#include "drake/systems/controllers/qp_inverse_dynamics/qp_inverse_dynamics.h"


#include "drake/systems/controllers/qp_inverse_dynamics/robot_kinematic_state.h"

#include "drake/examples/valkyrie/valkyrie_constants.h"
#include "robotlocomotion/robot_plan_t.hpp"

#include "drake/systems/controllers/setpoint.h"

namespace drake {
namespace examples {
namespace valkyrie {

using systems::controllers::qp_inverse_dynamics::RobotKinematicState;

class Box {
  public:
    Box(const RigidBodyTree<double>& tree, lcm::LCM& lcm);
    ~Box();
    void handle_message(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const bot_core::robot_state_t* msg);
    void publish_message();
    void publish_command();
    void compute_body_info();

  private:
    const manipulation::RobotStateLcmMessageTranslator translator_;
    VectorX<double> q_;
    VectorX<double> v_;
    // VectorX<double> position_;
    // VectorX<double> vel_;
    double kp_ = 0.000001;
    double kd_ = 0.000001;

    Eigen::Vector3d com_;
    Eigen::Isometry3d left_foot_;
    Eigen::Isometry3d right_foot_;
    lcm::LCM lcm_;
    RobotKinematicState<double> state_;

    VectorX<double> k_q_p_;
    VectorX<double> k_q_i_;
    VectorX<double> k_qd_p_;
    VectorX<double> k_f_p_;
    VectorX<double> ff_qd_;
    VectorX<double> ff_qd_d_;
    VectorX<double> ff_f_d_;
    VectorX<double> ff_const_;
};

}  // namespace valkyrie
}  // namespace examples
}  // namespace drake
