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


#include "hpp/pinocchio/urdf/util.hh"
#include "hpp/pinocchio/humanoid-robot.hh"
#include "hpp/pinocchio/center-of-mass-computation.hh"
#include "hpp/pinocchio/joint.hh"
#include "hpp/pinocchio/configuration.hh"
#include <hpp/pinocchio/device.hh>

//#include <hpp/timeopt/momentumopt/setting/Foot-print.hpp>
#include <hpp/timeopt/momentumopt/setting/Robot-state.hpp>

//#include <hpp/timeopt/momentumopt/cntopt/ContactPlanFromFootPrint.hpp>

using hpp::pinocchio::HumanoidRobot;
using hpp::pinocchio::HumanoidRobotPtr_t;
using namespace hpp::pinocchio::urdf;
using hpp::pinocchio::Configuration_t;
using hpp::pinocchio::CenterOfMassComputation;
using hpp::pinocchio::CenterOfMassComputationPtr_t;
using hpp::pinocchio::Device;
using hpp::pinocchio::DevicePtr_t;

#include <boost/format.hpp>
#define BOOST_TEST_MODULE test_foot
#include <boost/test/included/unit_test.hpp>

using namespace hpp::timeopt;
using namespace std;


void testProblem()
{
    DevicePtr_t red = HumanoidRobot::create ("red");
    loadRobotModel(red, 0, "", "freeflyer", "red_description", "red_robot", "", "");
    RobotState robot_(red);
    vector_t standing (red->configSize ());
    standing << -2.0, 0, 0.8, 0, 0, 0, 1, // 7, notation (x, y, z, w)
                -0.18625990960985309, 0.0653641030070116, -0.23641850893545993, 1.25616964830234, -1.0197204334654386, -0.06533846873335888, // 6
                -0.07570903906743214, 0.17745077316021962, -0.9548127675314133, 1.3070203284158541,-0.35224411037341646,-0.17741381491935357, // 6
                0, 0, 0, // 3
                0.1754, 0.4363, 1.2453, -1.3963, -2.618, 0, 0, 0,
                0, 0,
                -0.1754, -0.4363, -0.5453, 1.3963, 2.618, 0, 0, 0;
    robot_.updateforwardkinematics(standing);
    robot_.computeAllTerms();

    cout << "com" << robot_.getCOM() << endl;
    cout << "mass" << robot_.getMass() << endl;
}
BOOST_AUTO_TEST_SUITE (test_foot)
BOOST_AUTO_TEST_CASE(ttest_foot) {
    testProblem();
}

BOOST_AUTO_TEST_SUITE_END()
