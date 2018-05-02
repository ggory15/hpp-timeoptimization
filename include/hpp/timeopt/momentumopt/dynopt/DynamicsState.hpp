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

#pragma once

#include <array>
#include <vector>
#include <string>
#include <iostream>
#include <Eigen/Eigen>

#include <hpp/timeopt/momentumopt/setting/Definitions.hpp>
#include <hpp/timeopt/momentumopt/cntopt/ContactState.hpp>
#include <hpp/timeopt/fwd.hh>

namespace hpp{
    namespace timeopt {

      /**
       * This class is a container for all variables required to define a
       * dynamic state: center of mass position, linear and angular momenta
       * and its rates; forces, torques and center of pressure of the end-
       * effectors; positions, orientations, activations and contact types
       * of each end-effector.
       */
      struct DynamicsState
      {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            typedef std::array<int, Problem::n_endeffs_> IntArray;
            typedef std::array<bool, Problem::n_endeffs_> BoolArray;
            typedef std::array<ContactType, Problem::n_endeffs_> CntTypeArray;
            typedef std::array<Eigen::Vector3d, Problem::n_endeffs_> Vec3dArray;
            typedef std::array<Eigen::Quaternion<double>, Problem::n_endeffs_> OriArray;

        public:
            DynamicsState();
            ~DynamicsState(){}

            // Center of mass, linear and angular momenta
            double& time() { return dtime_; }
            double& mass() { return mass_; }
            Eigen::Vector3d& centerOfMass() { return com_; }
            Eigen::Vector3d& linearMomentum() { return lmom_; }
            Eigen::Vector3d& angularMomentum() { return amom_; }
            Eigen::Vector3d& linearMomentumRate() { return lmomd_; }
            Eigen::Vector3d& angularMomentumRate() { return amomd_; }
            Eigen::Vector3d& desiredcenterOfMass() { return comd_; }

            const double& time() const { return dtime_; }
            const double& mass() const { return mass_; }
            const Eigen::Vector3d& centerOfMass() const { return com_; }
            const Eigen::Vector3d& linearMomentum() const { return lmom_; }
            const Eigen::Vector3d& angularMomentum() const { return amom_; }
            const Eigen::Vector3d& linearMomentumRate() const { return lmomd_; }
            const Eigen::Vector3d& angularMomentumRate() const { return amomd_; }
            const Eigen::Vector3d& desiredcenterOfMass() const { return comd_; }

            // Endeffector ids, forces, torques and cops
            int& endeffectorActivationId(int eef_id) { return eefs_ids_[eef_id]; }
            Eigen::Vector3d& endeffectorCoP(int eef_id) { return eefs_cops_[eef_id]; }
            Eigen::Vector3d& endeffectorForce(int eef_id) { return eefs_frcs_[eef_id]; }
            Eigen::Vector3d& endeffectorTorque(int eef_id) { return eefs_trqs_[eef_id]; }

            const int& endeffectorActivationId(int eef_id) const { return eefs_ids_[eef_id]; }
            const Eigen::Vector3d& endeffectorCoP(int eef_id) const { return eefs_cops_[eef_id]; }
            const Eigen::Vector3d& endeffectorForce(int eef_id) const { return eefs_frcs_[eef_id]; }
            const Eigen::Vector3d& endeffectorTorque(int eef_id) const { return eefs_trqs_[eef_id]; }

            // Endeffector activations, positions, orientations and contact types
            bool& endeffectorActivation(int eef_id) { return eefs_activation_[eef_id]; }
            Eigen::Vector3d& endeffectorPosition(int eef_id) { return eefs_position_[eef_id]; }
            ContactType& endeffectorContactType(int eef_id) { return eefs_contact_type_[eef_id]; }
            Eigen::Quaternion<double>& endeffectorOrientation(int eef_id) { return eefs_orientation_[eef_id]; }

            bool endeffectorActivation(int eef_id) const { return eefs_activation_[eef_id]; }
            const Eigen::Vector3d& endeffectorPosition(int eef_id) const { return eefs_position_[eef_id]; }
            const ContactType& endeffectorContactType(int eef_id) const { return eefs_contact_type_[eef_id]; }
            const Eigen::Quaternion<double>& endeffectorOrientation(int eef_id) const  { return eefs_orientation_[eef_id]; }

            // Helper functions
            std::string toString() const;
            friend std::ostream& operator<<(std::ostream &os, const DynamicsState& obj) { return os << obj.toString(); }
            void fillInitialRobotState(const std::string cfg_file, const std::string robot_state = "initial_robot_configuration");

            // Helper functions for HPP
            void fillInitialBodyState(const DevicePtr_t& robot, const vector3_t& lmom=Eigen::Vector3d::Zero(), const vector3_t& amom=Eigen::Vector3d::Zero());
            void fillInitialLimbState(const JointPtr_t& joint, const int& eff_id, const bool& active, const vector3_t& force_ratio=Eigen::Vector3d::Zero());
            void setFinalcom(const vector3_t& com_goal){comd_ = com_goal;}
            void setMass(const double& mass) {mass_ = mass;}

        private:
              double dtime_, mass_;
              Eigen::Vector3d com_, amom_, lmom_, amomd_, lmomd_, comd_;

              IntArray eefs_ids_;
              BoolArray eefs_activation_;
              OriArray eefs_orientation_;
              CntTypeArray eefs_contact_type_;
              Vec3dArray eefs_frcs_, eefs_trqs_, eefs_cops_, eefs_position_;
      };

      /**
       * This class is a container for a sequence of dynamic states,
       * for all time steps in the optimization.
       */
      class DynamicsSequence
      {
        public:
              DynamicsSequence(){}
              ~DynamicsSequence(){}

              void resize(int num_timesteps);
              void clean() { dynamics_sequence_.clear(); }
              int size() const { return dynamics_sequence_.size(); }

              DynamicsState& dynamicsState(int time_id) { return dynamics_sequence_[time_id]; }
              const DynamicsState& dynamicsState(int time_id) const { return dynamics_sequence_[time_id]; }
              Eigen::Matrix<int, Problem::n_endeffs_, 1>& activeEndeffectorSteps() { return active_endeffector_steps_; }
              const Eigen::Matrix<int, Problem::n_endeffs_, 1>& activeEndeffectorSteps() const { return active_endeffector_steps_; }

              std::string toString() const;
              friend std::ostream& operator<<(std::ostream &os, const DynamicsSequence& obj) { return os << obj.toString(); }

        private:
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW
          std::vector<DynamicsState> dynamics_sequence_;
          Eigen::Matrix<int, Problem::n_endeffs_, 1> active_endeffector_steps_;
      };
   }
}
