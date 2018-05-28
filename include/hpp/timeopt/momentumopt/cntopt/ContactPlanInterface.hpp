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
#include <hpp/timeopt/momentumopt/cntopt/ContactState.hpp>
#include <hpp/timeopt/momentumopt/dynopt/DynamicsState.hpp>
#include <hpp/timeopt/momentumopt/setting/Foot-print.hpp>
#include <hpp/timeopt/momentumopt/setting/PlannerSetting.hpp>

namespace hpp{
    namespace timeopt {
      class ContactPlanInterface
      {
        public:
          ContactPlanInterface(){}
          ~ContactPlanInterface(){}

          void initialize(const PlannerSetting& planner_setting);
          void fillDynamicsSequence(DynamicsSequence& dynamics_sequence, const int num_timestep);

          ContactSequence& contactSequence() { return contact_sequence_; }
          const ContactSequence& contactSequence() const { return contact_sequence_; }
          
        protected:
          /*! Getter and setter methods for getting the planner variables  */
          inline const PlannerSetting& getSetting() const { return *planner_setting_; }

        protected:
          ContactSequence contact_sequence_;
          const PlannerSetting* planner_setting_;
      };
    }
}
