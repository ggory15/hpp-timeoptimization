#pragma once
#include <hpp/timeopt/momentumopt/cntopt/ContactPlanInterface.hpp>

namespace hpp{
    namespace timeopt {

      class ContactPlanFromFootPrint : public ContactPlanInterface
      {
        public:
          ContactPlanFromFootPrint(){
             for (int eff_id=0; eff_id<Problem::n_endeffs_; eff_id++) {
              this->contactSequence().endeffectorContacts(eff_id).clear();
             }
          }
          ~ContactPlanFromFootPrint(){}
        
        void addContact(const ContactSet con, const DynamicsState& ini_state);
      };
    }
}
