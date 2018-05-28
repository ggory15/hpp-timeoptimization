#include <hpp/timeopt/momentumopt/cntopt/ContactPlanFromFootPrint.hpp>

namespace hpp{
    namespace timeopt {

      void ContactPlanFromFootPrint::addContact(const ContactSet con, const DynamicsState& ini_state) {
            try{
                int eff_id = con.getid();
                int num = con.getfootset().size();
                
                for (int cnt_id = 0; cnt_id < num; cnt_id++){
                    this->contactSequence().endeffectorContacts(eff_id).push_back(ContactState());
                    this->contactSequence().endeffectorContacts(eff_id)[cnt_id].contactActivationTime() = con.getfootset()[cnt_id].startTime();
                    this->contactSequence().endeffectorContacts(eff_id)[cnt_id].contactDeactivationTime() = con.getfootset()[cnt_id].endTime();
                    this->contactSequence().endeffectorContacts(eff_id)[cnt_id].contactPosition() = con.getfootset()[cnt_id].position();
                    this->contactSequence().endeffectorContacts(eff_id)[cnt_id].contactType() = idToContactType(con.getfootset()[cnt_id].c_type());
                    this->contactSequence().endeffectorContacts(eff_id)[cnt_id].contactOrientation() = con.getfootset()[cnt_id].quat();
                }
            }
            catch (std::runtime_error& e){
                std::cout << "Error reading parameter ["<< e.what() << "]" << std::endl << std::endl;
            }          
        }
    }
}
    