
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <unistd.h>

#include "drake/examples/valkyrie/box_val_state_estimator.h"

// creating box val state lcm types
#include "lcmtypes/bot_core/robot_state_t.hpp"
#include "lcmtypes/bot_core/position_3d_t.hpp"
#include "lcmtypes/bot_core/vector_3d_t.hpp"

// drawing lcm types
#include "lcmtypes/robotlocomotion/viewer2_comms_t.hpp"


#include "drake/lcm/drake_lcm.h"

// for the urdf parser
#include "drake/common/find_resource.h"
#include "drake/multibody/parsers/urdf_parser.h"

// lcm publisher and subscriber systems
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"

#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/kinematics_cache.h"
#include "drake/examples/valkyrie/robot_state_decoder.h"

#include "drake/systems/lcm/lcm_driven_loop.h"

using Eigen::Vector3d;

namespace drake {

using lcm::DrakeLcm;

namespace systems {
namespace valkyrie {

using lcm::LcmSubscriberSystem;
using lcm::LcmPublisherSystem;

BoxValkyrieStateEstimator::BoxValkyrieStateEstimator(const RigidBodyTree<double>& robot)
  : robot_(robot) {

    input_port_index_result_ = DeclareAbstractInputPort().get_index();
    output_port_index_command_ = DeclareAbstractOutputPort(&BoxValkyrieStateEstimator::OutputMessage).get_index();

  // set the name of the system
  set_name("box_val_state_estimator");

}

void BoxValkyrieStateEstimator::OutputMessage(const Context<double>& context, bot_core::robot_state_t* output) const {

  // state input (as KinematicsCache object)
  // const KinematicsCache<double>* state = EvalInputValue<KinematicsCache<double>>(context, input_port_index_result_);

  const auto& state = EvalAbstractInput(context, input_port_index_result_)->GetValue<KinematicsCache<double>>();

  // std::cout << state.get_num_cache_elements() << std::endl;

  // get the center of mass
  Vector3d com = robot_.centerOfMass(state);

  // std::cout << state.getQ() << std::endl;


  // message output
  bot_core::robot_state_t& msg = *output;

  bot_core::position_3d_t lcm_position;
  bot_core::vector_3d_t position_vector;
  bot_core::quaternion_t quaternion_vector;

//   robotlocomotion::viewer2_comms_t drawing;
//
//   drawing.format = "treeviewer_json";
//   drawing.format_version_major = 1;
//   drawing.format_version_minor = 0;
//   drawing.data = {
//         "timestamp": 1486691399249288,
//         "setgeometry": [
//                 {
//                         "path": ["robot1", "link1"],
//                         "geometry": {
//                                 "type": "box",
//                                 "color": [1, 0, 0, 0.5],
//                                 "lengths": [1, 0.5, 2]
//                         }
//                 }
//         ],
//         "settransform": [],
//         "delete": []
// };
//   drawing.num_bytes = 4;



// --------------------------------------------------------------------
  // THIS IS WHERE THE MESSAGE IS CONSTRUCTED BASED ON THE RIGID BODY TREE AND STATE DATA
// --------------------------------------------------------------------

  msg.num_joints = 0;

  // set the position vector
  position_vector.x = com[0];
  position_vector.y = com[1];
  position_vector.z = com[2];

  // set the quaternion orientation
  quaternion_vector.w = 1.0;
  quaternion_vector.x = 0.0;
  quaternion_vector.y = 0.0;
  quaternion_vector.z = 0.0;

  lcm_position.translation = position_vector;
  lcm_position.rotation = quaternion_vector;

  msg.pose = lcm_position;


  // DISPLAY THE POSITION IN DIRECTOR AS WELL




// --------------------------------------------------------------------
  // robot_state_t lcm type structure

  // struct robot_state_t
  // {
  //   int64_t utime;
  //   position_3d_t pose;
  //   twist_t twist;
  //
  //   int16_t num_joints;
  //   string joint_name[num_joints];
  //   float joint_position [num_joints];
  //   float joint_velocity [num_joints];
  //   float joint_effort[num_joints];
  //
  //   force_torque_t force_torque;
  // }

  // set the state of the output based on the input
  // the state should have the following:
  // center of mass values
  // position of each limb

// --------------------------------------------------------------------

}



void run_box_state_transformer() {

  // need the urdf file in order to create the rigid body tree
  // std::string urdf = FindResourceOrThrow(
  //         "drake/examples/valkyrie/urdf/urdf/"
  //         "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf");

  // RigidBodyTree object for valkyrie
  auto robot = std::make_unique<RigidBodyTree<double>>();

  // populate the RigidBodyTree based on the urdf file
  // parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
  //     urdf, multibody::joints::kRollPitchYaw, robot.get());

  drake::parsers::urdf::AddModelInstanceFromUrdfFile(
      FindResourceOrThrow(
          "drake/examples/valkyrie/urdf/urdf/"
          "valkyrie_A_sim_drake_one_neck_dof_wide_ankle_rom.urdf"),
      multibody::joints::kRollPitchYaw, nullptr /* weld to frame */,
    robot.get());

  // Drake LCM class
  DrakeLcm lcm;


  // builder will hold all of the systems together
  DiagramBuilder<double> builder;

// ------------------------------------------------------------
// ADD THE SYSTEMS TO THE DIAGRAMBUILDER
// ------------------------------------------------------------

  // lcm input from channel "EST_ROBOT_STATE"
  auto& valkyrie_state_subscriber = *builder.AddSystem(
    LcmSubscriberSystem::Make<bot_core::robot_state_t>("EST_ROBOT_STATE", &lcm));
  // valkyrie_state_subscriber.set_name("valkyrie_state_subscriber");

  // this decodes an LCM message into a KinematicsCache object
  RobotStateDecoder* state_decoder = builder.AddSystem(std::make_unique<RobotStateDecoder>(*robot));

  // takes in a KinematicsCache input and outputs a LCM message
  BoxValkyrieStateEstimator* estimator = builder.AddSystem(std::make_unique<BoxValkyrieStateEstimator>(*robot)); // pass in the robot and the const kinematicscache object

  // lcm output on channel "BOX_VAL_STATE"
  auto& box_valkyrie_state_publisher = *builder.AddSystem(
    LcmPublisherSystem::Make<bot_core::robot_state_t>("BOX_VAL_STATE", &lcm));
  // box_valkyrie_state_publisher.set_name("box_valkyrie_state_publisher");

// ------------------------------------------------------------
// CONNECT THE SYSTEMS TOGETHER
// ------------------------------------------------------------

  // lcm -> decoder input
  builder.Connect(valkyrie_state_subscriber.get_output_port(0),
                  state_decoder->get_input_port(0));

  // decoded results -> input kinematics
  builder.Connect(state_decoder->get_output_port(0),
                  estimator->get_input_port_state());

  // lcm message from box val estimator -> lcm message on channel
  builder.Connect(estimator->get_ouptut_port_state(),
                  box_valkyrie_state_publisher.get_input_port(0));

// ------------------------------------------------------------


  // builds the diagram described by DiagramBuilder
  std::unique_ptr<Diagram<double>> diagram = builder.Build();

  // auto context = diagram->AllocateContext();
  auto context = diagram->CreateDefaultContext();
  auto output = diagram->AllocateOutput(*context);

  // auto output = diagram->AllocateOutput(*context);

  // starts lcm receives so that they can receive data
  lcm.StartReceiveThread();

  std::cout << "Starting box valkyrie estimator." << std::endl;

  // publish the box valkyrie data
  while(true) {


    // const systems::Context<double>& sub_context = diagram->GetSubsystemContext(valkyrie_state_subscriber, *context);
    // std::cout << valkyrie_state_subscriber.GetMessageCount(*context) << std::endl;
    // std::cout << valkyrie_state_subscriber.WaitForMessage(valkyrie_state_subscriber.GetMessageCount(sub_context)) << std::endl;

    // reference to the subcontext that corresponds to the box_valkyrie_state_publisher subsystem
    const systems::Context<double>& pub_context = diagram->GetSubsystemContext(box_valkyrie_state_publisher, *context);

    // publish the data from the subsystem
    box_valkyrie_state_publisher.Publish(pub_context);

    usleep( 10000 ); // about 90 hz

  }

}

} // namespace valkyrie
} // namespace systems
} // namespace drake

// starts the program and transformation
int main() { drake::systems::valkyrie::run_box_state_transformer(); }
