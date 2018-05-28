#ifndef HPP_TIMEOPT_ROBOT_STATE_HH
#define HPP_TIMEOPT_ROBOT_STATE_HH

# include <hpp/timeopt/fwd.hh>
# include <hpp/timeopt/config.hh>

namespace hpp {
  namespace timeopt {
        struct RobotState {
            public: 
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                RobotState(DevicePtr_t & robot);
                ~RobotState(){}

                void updateforwardkinematics(vector_t q);
                void updateframekinematics();
                void computeAllTerms();
                vector3_t getCOM();
                double getMass();

                int getFrameId(const std::string name);
                Transform3f getFrameTransformation(const int & id);
                void setConfiguration(const vector_t & q);
                int getConfigurationSize();

            private:
                DevicePtr_t r_;
                int nq_, nv_, na_;
                vector_t q_, v_;
            }; //RobotState
  } // namespace timeopt
} // namespace hpp

#endif // HPP_TIMEOPT_ROBOT_STATE_HH
