#pragma once // compile only once

#include <memory>
#include <utility>

#include "lcmtypes/bot_core/robot_state_t.hpp"

#include "drake/lcm/drake_lcm.h" // for the lcm

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/kinematics_cache.h"


#include "drake/common/eigen_types.h"

namespace drake {
namespace systems {
namespace valkyrie {


class BoxValkyrieStateEstimator : public systems::LeafSystem<double> {
  public:
    BoxValkyrieStateEstimator(const RigidBodyTree<double>& robot);

    inline const InputPortDescriptor<double>& get_input_port_state() const {
      return get_input_port(input_port_index_result_);
    }
    inline const OutputPort<double>& get_ouptut_port_state() const {
      return get_output_port(output_port_index_command_);
    }

  private:
    // this method is for the output port
    void OutputMessage(const Context<double>& context,
                        bot_core::robot_state_t* output) const;

    const RigidBodyTree<double>& robot_;
    int input_port_index_result_;
    int output_port_index_command_;
};

} // namespace valkyrie
} // namespace systems
} // namespace drake
