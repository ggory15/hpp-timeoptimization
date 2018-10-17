#include <hpp/timeopt/momentumopt/cntopt/ContactPlanInterface.hpp>

using namespace std;

namespace hpp{
    namespace timeopt {
      void ContactPlanInterface::initialize(const PlannerSetting& planner_setting)
      {
        planner_setting_ = &planner_setting;
      }

      void ContactPlanInterface::fillDynamicsSequence(DynamicsSequence& dynamics_sequence, const int num_timestep)
      {
        // configuration of end-effectors during contact
        dynamics_sequence.activeEndeffectorSteps().setZero();
        for (int eff_id=0; eff_id<Problem::n_endeffs_; eff_id++) {
          int counter = 0;
          int ini_id = 0, end_id = contact_sequence_.endeffectorContacts(eff_id).size();

          for (int cnt_id=ini_id; cnt_id<end_id; cnt_id++) {
            for (int time_id=0; time_id<num_timestep; time_id++) {
              double current_time = double(time_id+1.0)*this->getSetting().get(PlannerDoubleParam_TimeStep);
              if (current_time>=contact_sequence_.endeffectorContacts(eff_id)[cnt_id].contactActivationTime() &&
                  current_time< contact_sequence_.endeffectorContacts(eff_id)[cnt_id].contactDeactivationTime())
              {
                dynamics_sequence.dynamicsState(time_id).endeffectorActivation(eff_id) = true;
                dynamics_sequence.dynamicsState(time_id).endeffectorActivationId(eff_id) = counter++;
                dynamics_sequence.dynamicsState(time_id).endeffectorContactType(eff_id) = contact_sequence_.endeffectorContacts(eff_id)[cnt_id].contactType();
                dynamics_sequence.dynamicsState(time_id).endeffectorPosition(eff_id) = contact_sequence_.endeffectorContacts(eff_id)[cnt_id].contactPosition();
                dynamics_sequence.dynamicsState(time_id).endeffectorOrientation(eff_id) = contact_sequence_.endeffectorContacts(eff_id)[cnt_id].contactOrientation();
              }
            }
          }
          dynamics_sequence.activeEndeffectorSteps()[eff_id] = counter;
         }
      }
    }
}
