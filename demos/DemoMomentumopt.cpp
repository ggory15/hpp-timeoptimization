/*
 * Copyright [2017] Max Planck Society. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * This demo shows how to use the dynamics optimization routine given
 * an appropriate configuration file. It requires an initial robot state
 * and a reference momentum trajectory. The optimized motion plan is available
 * to the user under a DynamicsSequence object (dyn_optimizer.dynamicsSequence()),
 * which is a collection of dynamic states, each of which contains information
 * about all variables, and all end-effectors of the motion plan for one time step.
 *
 * If the variable store_data within the configuration file has been set
 * to True, then a file of results will also be written. It can be found
 * next to the configuration file with the keyword "_results" appended.
 * It can be visualized with the available python script as indicated in
 * the Readme file.
 */

#include <iomanip>
#include <yaml-cpp/yaml.h>
#include <hpp/timeopt/momentumopt/dynopt/DynamicsOptimizer.hpp>
#include <hpp/timeopt/momentumopt/cntopt/ContactPlanFromFile.hpp>

#include "hpp/pinocchio/urdf/util.hh"
#include "hpp/pinocchio/humanoid-robot.hh"
#include "hpp/pinocchio/center-of-mass-computation.hh"
#include "hpp/pinocchio/joint.hh"
#include "hpp/pinocchio/configuration.hh"
#include <hpp/pinocchio/device.hh>

using namespace hpp::timeopt;

using hpp::pinocchio::HumanoidRobot;
using hpp::pinocchio::HumanoidRobotPtr_t;
using namespace hpp::pinocchio::urdf;
using hpp::pinocchio::Configuration_t;
using hpp::pinocchio::CenterOfMassComputation;
using hpp::pinocchio::CenterOfMassComputationPtr_t;
using hpp::pinocchio::Device;
using hpp::pinocchio::DevicePtr_t;

int main( int argc, char *argv[] )
{
  // load configuration file
  std::string cfg_file = CFG_SRC_PATH;
  if (argc==3) {
    if (std::strcmp(argv[1], "i"))
      cfg_file += argv[2];
  } else {
	std::cout << "Usage: ./demo -i <name of config file within config folder>" << std::endl;
    return 1;
  }

  // define problem configuration
  PlannerSetting planner_setting;
  std::string default_file = CFG_SRC_PATH + std::string("default_solver_setting.yaml");
  planner_setting.initialize(cfg_file, default_file);

  // define robot initial state from HPP
  DynamicsState ini_state;

  HumanoidRobotPtr_t red = HumanoidRobot::create ("red");
  loadRobotModel(red, 0, "", "freeflyer", "red_description", "red_robot", "", "");
  setupHumanoidRobot (red, "");
  for (std::size_t i = 0; i < 3; ++i) {
    red->rootJoint ()->lowerBound (i, -1);
    red->rootJoint ()->upperBound (i, +1);
    red->rootJoint ()->isBounded  (i, true);
  }

  Configuration_t standing (red->configSize ());
  standing << -2.0, 0, 0.8, 0, 0, 0, 1, // 7, notation (x, y, z, w)
              -0.18625990960985309, 0.0653641030070116, -0.23641850893545993, 1.25616964830234, -1.0197204334654386, -0.06533846873335888, // 6
              -0.07570903906743214, 0.17745077316021962, -0.9548127675314133, 1.3070203284158541,-0.35224411037341646,-0.17741381491935357, // 6
              0, 0, 0, // 3
              0.1754, 0.4363, 1.2453, -1.3963, -2.618, 0, 0, 0,
              0, 0,
              -0.1754, -0.4363, -0.5453, 1.3963, 2.618, 0, 0, 0;

  red->currentConfiguration (standing);
  red->computeForwardKinematics ();

  ini_state.fillInitialBodyState(red);
  // id_right_foot = 0, id_left_foot  = 1, id_right_hand = 2, id_left_hand  = 3
  ini_state.fillInitialLimbState(red->getJointByName("AnkleRoll_R_Joint"), true, 0, Eigen::Vector3d(0, 0, 0.5));
  ini_state.fillInitialLimbState(red->getJointByName("AnkleRoll_L_Joint"), true, 1, Eigen::Vector3d(0, 0, 0.5));
  ini_state.fillInitialLimbState(red->getJointByName("J_rwrist2"), true, 2);
  ini_state.fillInitialLimbState(red->getJointByName("J_lwrist2"), true, 3);

  // define reference dynamic sequence
  DynamicsSequence ref_sequence;
  ref_sequence.resize(planner_setting.get(PlannerIntParam_NumTimesteps));

  // define contact plan
  ContactPlanFromFile contact_plan;
  contact_plan.initialize(planner_setting);
  contact_plan.optimize(ini_state);

  // optimize motion
  DynamicsOptimizer dyn_optimizer;
  dyn_optimizer.initialize(planner_setting, ini_state, &contact_plan);
  dyn_optimizer.optimize(ref_sequence);


}
