#ifndef HPP_TIMEOPT_FOOT_PRINT_HH
#define HPP_TIMEOPT_FOOT_PRINT_HH

# include <Eigen/StdVector>
# include <hpp/timeopt/fwd.hh>
# include <hpp/timeopt/config.hh>

namespace hpp {
  namespace timeopt {
        struct FootPrint {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            FootPrint (const double& time_start, const double& time_end, const vector3_t& position, const quaternion_t& orientation, const int& contact_type) 
            {
                time_s_ = time_start;
                time_e_ = time_end;
                pos_ = position;
                ori_ = orientation;
                c_type_ = contact_type;
            }
            double time_s_, time_e_;
            int c_type_;
            vector3_t pos_;
            quaternion_t ori_;
            const vector3_t& position () const
            {
                return pos_;
            }
            const quaternion_t& quat () const
            {
                return ori_;
            }
            const double& startTime() const{
                return time_s_;
            }
            const double& endTime() const{
                return time_e_;
            }
            const int& c_type() const{
                return c_type_;
            }
        }; // struct FootPrint
   
    typedef std::vector <FootPrint, Eigen::aligned_allocator <FootPrint>> FootPrints_t;
    class ContactSet {
        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            ContactSet(const int& eef_id, const FootPrints_t& foot) {
                foot_set_ = &foot;                
                eef_id_ = eef_id;
            }
            ~ContactSet(){}
            void reset(const int& eef_id, const FootPrints_t & foot){ 
                foot_set_ = &foot;
                eef_id_ = eef_id;
            }
            const int& getid() const{
                return eef_id_;
            }
            const FootPrints_t& getfootset() const{
                return *foot_set_;
            }

        protected:
            const FootPrints_t* foot_set_;
            int eef_id_;
    }; // ContactSet
  } // namespace walkgen
} // namespace hpp

#endif // HPP_WALKGEN_FOOT_PRINT_HH
