#include <hpp/pinocchio/device.hh>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>

using hpp::pinocchio::DevicePtr_t;

#include <hpp/timeopt/momentumopt/setting/Robot-state.hpp>

namespace hpp {
  namespace timeopt {
    RobotState::RobotState(DevicePtr_t & robot) {
        r_ = robot;
        nq_ = r_->configSize();
        nv_ = nq_ -6;
        na_ = na_ -1;
        q_.resize(nq_); 
        q_.setZero();
        v_.resize(nv_);
        v_.setZero();
    }
    
    void RobotState::updateforwardkinematics(vector_t q){
        q_ = q;
        se3::forwardKinematics(r_->model(), r_->data(), q_);
    }
    void RobotState::updateframekinematics(){
        se3::framesForwardKinematics(r_->model(), r_->data());
    }
    void RobotState::computeAllTerms(){
        se3::computeAllTerms(r_->model(), r_->data(), q_, v_);
    }
    vector3_t RobotState::getCOM() {
        return r_->data().com[0];
    }
    double RobotState::getMass(){
        return r_->data().M(0,0);
    }
    int RobotState::getFrameId(const std::string name){
        return r_->model().getFrameId(name);
    }
    Transform3f RobotState::getFrameTransformation(const int & id){ 
        se3::Frame f = r_->model().frames[id];
        if (f.type == se3::JOINT)
            return r_->data().oMi[f.parent];
        else  
            return r_->data().oMi[f.parent] * f.placement;            
    }
    void RobotState::setConfiguration(const vector_t & q){
        r_->currentConfiguration(q);
        q_ = q;
        this->updateforwardkinematics(q_);
        this->updateframekinematics();
    }
    int RobotState::getConfigurationSize(){
        return nq_;
    }
  } // namespace timeopt
} // namespace hpp

