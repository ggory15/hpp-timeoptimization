#include <iomanip>
#include <yaml-cpp/yaml.h>

#include "hpp/pinocchio/urdf/util.hh"
#include "hpp/pinocchio/humanoid-robot.hh"
#include "hpp/pinocchio/center-of-mass-computation.hh"
#include "hpp/pinocchio/joint.hh"
#include "hpp/pinocchio/configuration.hh"
#include <hpp/pinocchio/device.hh>

#include <hpp/timeopt/momentumopt/dynopt/DynamicsOptimizer.hpp>
#include <hpp/timeopt/momentumopt/cntopt/ContactPlanFromFootPrint.hpp>
#include <hpp/timeopt/momentumopt/setting/Robot-state.hpp>

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

  DynamicsState ini_state;

  // define robot initial state from HPP
  DevicePtr_t red = HumanoidRobot::create ("red");
  loadRobotModel(red, 0, "", "freeflyer", "red_description", "red_robot", "", "");
  
  vector_t standing (red->configSize ());
  standing << -2.0, 0, 0.8, 0, 0, 0, 1, // 7, notation (x, y, z, w)
              -0.18625990960985309, 0.0653641030070116, -0.23641850893545993, 1.25616964830234, -1.0197204334654386, -0.06533846873335888, // 6
              -0.07570903906743214, 0.17745077316021962, -0.9548127675314133, 1.3070203284158541,-0.35224411037341646,-0.17741381491935357, // 6
              0, 0, 0, // 3
              0.1754, 0.4363, 1.2453, -1.3963, -2.618, 0, 0, 0,
              0, 0,
              -0.1754, -0.4363, -0.5453, 1.3963, 2.618, 0, 0, 0;
  
  RobotState robot_(red);
  robot_.updateforwardkinematics(standing);
  robot_.computeAllTerms();

  ini_state.fillInitialBodyState(robot_.getMass(), robot_.getCOM());

  // id_right_foot = 0, id_left_foot  = 1, id_right_hand = 2, id_left_hand  = 3  
  ini_state.fillInitialLimbState(robot_.getFrameTransformation(robot_.getFrameId("r_sole")), 0, true, Eigen::Vector3d(0, 0, 0.5));
  ini_state.fillInitialLimbState(robot_.getFrameTransformation(robot_.getFrameId("l_sole")), 1, true, Eigen::Vector3d(0, 0, 0.5));
  ini_state.fillInitialLimbState(robot_.getFrameTransformation(robot_.getFrameId("J_lwrist2")), 2, false);
  ini_state.fillInitialLimbState(robot_.getFrameTransformation(robot_.getFrameId("J_rwrist2")), 3, false);

  // define reference dynamic sequence
  DynamicsSequence ref_sequence;
  ref_sequence.resize(planner_setting.get(PlannerIntParam_NumTimesteps));

  // define contact plan
  ContactPlanFromFootPrint contact_plan;
  contact_plan.initialize(planner_setting);

  FootPrints_t footPrints;
  footPrints.push_back(FootPrint(0.0, 1.0, vector3_t(-1.82, -0.021,  0.1), quaternion_t(1.0, 0.0, 0.0, 0.0), 1));
  footPrints.push_back(FootPrint(2.0, 4.5, vector3_t(-1.55, -0.500,  0.30), quaternion_t(0.9689, 0.0, -0.2474, 0.0), 1));
  footPrints.push_back(FootPrint(6.0, 9.9, vector3_t(-1.02, -0.450,  0.80), quaternion_t(1.0, 0.0, 0.0, 0.0), 1));

  ContactSet con_(0, footPrints);
  contact_plan.addContact(con_ , ini_state);

  footPrints.clear();
  footPrints.push_back(FootPrint(0.0, 2.5, vector3_t(-2.206, 0.17,  0.1), quaternion_t(1.0, 0.0, 0.0, 0.0), 1));
  footPrints.push_back(FootPrint(4.0, 6.5, vector3_t(-1.5,  0.17,  0.55), quaternion_t(0.96891, 0.0, 0.2474, 0.0), 1));
  footPrints.push_back(FootPrint(8.5, 9.9, vector3_t(-0.9,  0.17,  0.8), quaternion_t(1.0, 0.0, 0.0, 0.0), 1));

  con_.reset(1, footPrints);
  contact_plan.addContact(con_ , ini_state);

  // define goal com
  ini_state.setFinalcom(vector3_t(1.05, -0.20, 0.6) + robot_.getCOM());
  // optimize motion
  DynamicsOptimizer dyn_optimizer;
  dyn_optimizer.initialize(planner_setting, ini_state, &contact_plan);
  dyn_optimizer.optimize(ref_sequence);


}
