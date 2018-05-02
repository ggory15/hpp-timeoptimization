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

#include <iomanip>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <hpp/timeopt/yaml_eigen.h>
#include <hpp/timeopt/momentumopt/dynopt/DynamicsOptimizer.hpp>

using namespace hpp::timeopt::solver;

namespace hpp{
    namespace timeopt {

    void DynamicsOptimizer::initialize(PlannerSetting& planner_setting, const DynamicsState& ini_state, ContactPlanInterface* contact_plan)
    {
      ini_state_ = ini_state;
      contact_plan_ = contact_plan;
      planner_setting_ = &planner_setting;

      if (!this->getSetting().get(PlannerBoolParam_UseDefaultSolverSetting)) { model_.configSetting(this->getSetting().get(PlannerStringParam_ConfigFile)); }
      else                                                                   { model_.configSetting(this->getSetting().get(PlannerStringParam_DefaultSolverSettingFile)); }

      com_pos_goal_ = ini_state.desiredcenterOfMass(); // for hpp
      mass_times_gravity_ = ini_state_.mass() * this->getSetting().get(PlannerDoubleParam_Gravity);
        
      friction_cone_.getCone(this->getSetting().get(PlannerDoubleParam_FrictionCoefficient), cone_matrix_);
      dynamicsSequence().resize(this->getSetting().get(PlannerIntParam_NumTimesteps));
      for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++) { dynamicsSequence().dynamicsState(time_id).time() = this->getSetting().get(PlannerDoubleParam_TimeStep); }

      contact_plan_->fillDynamicsSequence(this->dynamicsSequence());
      this->initializeOptimizationVariables();
    }

    void DynamicsOptimizer::initializeOptimizationVariables()
    {
      num_vars_ = 0.;
      double inf_value = SolverSetting::inf;

      // center of mass, linear and angular momentum
      com_.initialize('C', 3, this->getSetting().get(PlannerIntParam_NumTimesteps), -inf_value, inf_value, num_vars_);
      lmom_.initialize('C', 3, this->getSetting().get(PlannerIntParam_NumTimesteps), -inf_value, inf_value, num_vars_);
      amom_.initialize('C', 3, this->getSetting().get(PlannerIntParam_NumTimesteps), -inf_value, inf_value, num_vars_);

      // time variable, linear and angular momentum rates
      if (this->getSetting().heuristic() == Heuristic::TimeOptimization) {
        dt_.initialize('C', 1, this->getSetting().get(PlannerIntParam_NumTimesteps), -inf_value, inf_value, num_vars_);
        lmomd_.initialize('C', 3, this->getSetting().get(PlannerIntParam_NumTimesteps), -inf_value, inf_value, num_vars_);
        amomd_.initialize('C', 3, this->getSetting().get(PlannerIntParam_NumTimesteps), -inf_value, inf_value, num_vars_);
      }

      // upper and lower bound variables, forces, cops, torques
      for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++) {
        lb_var_[eff_id].initialize('C', 3, dynamicsSequence().activeEndeffectorSteps()[eff_id], -inf_value, inf_value, num_vars_);
        ub_var_[eff_id].initialize('C', 3, dynamicsSequence().activeEndeffectorSteps()[eff_id], -inf_value, inf_value, num_vars_);
        frc_world_[eff_id].initialize('C', 3, dynamicsSequence().activeEndeffectorSteps()[eff_id], -inf_value, inf_value, num_vars_);
        cop_local_[eff_id].initialize('C', 2, dynamicsSequence().activeEndeffectorSteps()[eff_id], -inf_value, inf_value, num_vars_);
        trq_local_[eff_id].initialize('C', 1, dynamicsSequence().activeEndeffectorSteps()[eff_id], -inf_value, inf_value, num_vars_);
      }
    }

    void DynamicsOptimizer::updateTrackingObjective()
    {
      this->getSetting().get(PlannerVectorParam_WeightLinearMomentum) = this->getSetting().get(PlannerVectorParam_WeightDynamicTrackingLinearMomentum);
      this->getSetting().get(PlannerVectorParam_WeightAngularMomentum) = this->getSetting().get(PlannerVectorParam_WeightDynamicTrackingAngularMomentum);
    }

    ExitCode DynamicsOptimizer::optimize(const DynamicsSequence& ref_sequence, bool update_tracking_objective)
    {
      if (update_tracking_objective)
        this->updateTrackingObjective();

      solve_time_ = 0.0;
      has_converged_ = false;
      internalOptimize(ref_sequence, true);

      if (this->getSetting().heuristic() == Heuristic::TimeOptimization) {
        for (int iter_id=1; iter_id<=this->getSetting().get(PlannerIntParam_MaxNumTimeIterations); iter_id++) {
          internalOptimize(ref_sequence);
          if (has_converged_) { break; }
        }
      }

      this->saveToFile(ref_sequence);
      return exitcode_;
    }

    void DynamicsOptimizer::internalOptimize(const DynamicsSequence& ref_sequence, bool is_first_time)
    {
      try
      {
        // add variables to model
        vars_.clear();
        for (int var_id=0; var_id<num_vars_; var_id++)
          vars_.push_back(Var());

        model_.clean();
        addVariableToModel(com_, model_, vars_);
        addVariableToModel(lmom_, model_, vars_);
        addVariableToModel(amom_, model_, vars_);

        if (this->getSetting().heuristic() == Heuristic::TimeOptimization) {
              addVariableToModel(dt_, model_, vars_);
              addVariableToModel(lmomd_, model_, vars_);
              addVariableToModel(amomd_, model_, vars_);
        }

        for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++) {
          addVariableToModel(lb_var_[eff_id], model_, vars_);
          addVariableToModel(ub_var_[eff_id], model_, vars_);
          addVariableToModel(frc_world_[eff_id], model_, vars_);
          addVariableToModel(cop_local_[eff_id], model_, vars_);
          addVariableToModel(trq_local_[eff_id], model_, vars_);
        }

        // adding quadratic objective
        quad_objective_.clear();

        if (this->getSetting().get(PlannerIntParam_NumViapoints) > 0) {
          for (int via_id=0; via_id<this->getSetting().get(PlannerIntParam_NumViapoints); via_id++)
            for (int axis_id=0; axis_id<3; axis_id++)
                  quad_objective_.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightCenterOfMassViapoint)[axis_id], LinExpr(vars_[com_.id(axis_id,int(this->getSetting().get(PlannerCVectorParam_Viapoints)[via_id](0)/this->getSetting().get(PlannerDoubleParam_TimeStep)))]) - LinExpr(this->getSetting().get(PlannerCVectorParam_Viapoints)[via_id](axis_id+1)) );
        }

        for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++) {
          for (int axis_id=0; axis_id<3; axis_id++) {

            // penalty on center of mass, linear and angular momentum
            if (time_id==this->getSetting().get(PlannerIntParam_NumTimesteps)-1) {
                  quad_objective_.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightCenterOfMass)[axis_id], LinExpr(vars_[com_.id(axis_id,time_id)]) - LinExpr(com_pos_goal_[axis_id]));
                  quad_objective_.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightFinalLinearMomentum)[axis_id], LinExpr(vars_[lmom_.id(axis_id,time_id)]) - LinExpr(ref_sequence.dynamicsState(time_id).linearMomentum()[axis_id]));
                  quad_objective_.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightFinalAngularMomentum)[axis_id], LinExpr(vars_[amom_.id(axis_id,time_id)]) - LinExpr(ref_sequence.dynamicsState(time_id).angularMomentum()[axis_id]));
            } else {
                  quad_objective_.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightLinearMomentum)[axis_id], LinExpr(vars_[lmom_.id(axis_id,time_id)]) - LinExpr(ref_sequence.dynamicsState(time_id).linearMomentum()[axis_id]));
                  quad_objective_.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightAngularMomentum)[axis_id], LinExpr(vars_[amom_.id(axis_id,time_id)]) - LinExpr(ref_sequence.dynamicsState(time_id).angularMomentum()[axis_id]));
            }

            // penalty on linear and angular momentum rates
            if (this->getSetting().heuristic() == Heuristic::TimeOptimization) {
              quad_objective_.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightLinearMomentumRate)[axis_id], vars_[lmomd_.id(axis_id,time_id)]);
              quad_objective_.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightAngularMomentumRate)[axis_id], vars_[amomd_.id(axis_id,time_id)]);
            } else {
              if (time_id==0) {
                quad_objective_.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightLinearMomentumRate)[axis_id], (LinExpr(vars_[lmom_.id(axis_id,time_id)]) - LinExpr(ini_state_.linearMomentum()[axis_id]))*(1.0/dynamicsSequence().dynamicsState(time_id).time()));
                quad_objective_.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightAngularMomentumRate)[axis_id], (LinExpr(vars_[amom_.id(axis_id,time_id)]) - LinExpr(ini_state_.angularMomentum()[axis_id]))*(1.0/dynamicsSequence().dynamicsState(time_id).time()));
              } else {
                quad_objective_.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightLinearMomentumRate)[axis_id], (LinExpr(vars_[lmom_.id(axis_id,time_id)]) - LinExpr(vars_[lmom_.id(axis_id,time_id-1)]))*(1.0/dynamicsSequence().dynamicsState(time_id).time()));
                quad_objective_.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightAngularMomentumRate)[axis_id], (LinExpr(vars_[amom_.id(axis_id,time_id)]) - LinExpr(vars_[amom_.id(axis_id,time_id-1)]))*(1.0/dynamicsSequence().dynamicsState(time_id).time()));
              }
            }

                // penalty on forces
                for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++) {
                  if (dynamicsSequence().dynamicsState(time_id).endeffectorActivation(eff_id)) {
                    if (eff_id==static_cast<int>(Problem::EffId::id_right_foot) || eff_id==static_cast<int>(Problem::EffId::id_left_foot)) { quad_objective_.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightLegForce)[axis_id], vars_[frc_world_[eff_id].id(axis_id,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))]); }
                                                                                                                                      else { quad_objective_.addQuaTerm(this->getSetting().get(PlannerVectorParam_WeightArmForce)[axis_id], vars_[frc_world_[eff_id].id(axis_id,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))]); }
                  }
            }

            // penalty on rate of forces
            for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++) {
              Eigen::Vector3d ctrl_rate_penalty = Eigen::Vector3d::Zero();
              if (Problem::isHand(eff_id)) { ctrl_rate_penalty = this->getSetting().get(PlannerVectorParam_WeightArmForceRate); }
                                      else { ctrl_rate_penalty = this->getSetting().get(PlannerVectorParam_WeightLegForceRate); }

              if (dynamicsSequence().dynamicsState(time_id).endeffectorActivation(eff_id)) {
                // current force
                LinExpr current_force = vars_[frc_world_[eff_id].id(axis_id,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))];

                // next force
                LinExpr next_force = 0.0;
                if (time_id==this->getSetting().get(PlannerIntParam_NumTimesteps)-1) { next_force = current_force; }
                else {
                  if (dynamicsSequence().dynamicsState(time_id+1).endeffectorActivation(eff_id))
                    next_force = vars_[frc_world_[eff_id].id(axis_id,dynamicsSequence().dynamicsState(time_id+1).endeffectorActivationId(eff_id))];
                }

                // previous force
                LinExpr previous_force = 0.0;
                if (time_id==0) { previous_force = ini_state_.endeffectorForce(eff_id)[axis_id]; }
                else {
                  if (dynamicsSequence().dynamicsState(time_id-1).endeffectorActivation(eff_id))
                    previous_force = vars_[frc_world_[eff_id].id(axis_id,dynamicsSequence().dynamicsState(time_id-1).endeffectorActivationId(eff_id))];
                }

                // penalty on force smoothness
                quad_objective_.addQuaTerm(ctrl_rate_penalty[axis_id], next_force - current_force);
                quad_objective_.addQuaTerm(ctrl_rate_penalty[axis_id], current_force - previous_force);
              }
            }
          }

          // penalty on torques
          for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++) {
            if (dynamicsSequence().dynamicsState(time_id).endeffectorActivation(eff_id)) {
              if (eff_id==static_cast<int>(Problem::EffId::id_right_foot) || eff_id==static_cast<int>(Problem::EffId::id_left_foot)) { quad_objective_.addQuaTerm(this->getSetting().get(PlannerDoubleParam_WeightLegTorque), vars_[trq_local_[eff_id].id(0,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))]); }
                                                                                                                                else { quad_objective_.addQuaTerm(this->getSetting().get(PlannerDoubleParam_WeightArmTorque), vars_[trq_local_[eff_id].id(0,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))]); }
            }
          }

          // penalty on time
          if (this->getSetting().heuristic() == Heuristic::TimeOptimization) {
                quad_objective_.addQuaTerm(this->getSetting().get(PlannerDoubleParam_WeightTimePenalty), LinExpr(vars_[dt_.id(0,time_id)]));
                quad_objective_.addQuaTerm(this->getSetting().get(PlannerDoubleParam_WeightTimeRegularization), LinExpr(vars_[dt_.id(0,time_id)]) - LinExpr(this->getSetting().get(PlannerDoubleParam_TimeStep)));
          }
        }

        if (!is_first_time && this->getSetting().heuristic() == Heuristic::TimeOptimization) {
          for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++) {
            for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++) {
              if (dynamicsSequence().dynamicsState(time_id).endeffectorActivation(eff_id)) {
                Eigen::Matrix3d rot = dynamicsSequence().dynamicsState(time_id).endeffectorOrientation(eff_id).toRotationMatrix();
                Eigen::Vector3d rx = rot.col(0);
                Eigen::Vector3d ry = rot.col(1);

                LinExpr lx, ly, lz, fx, fy, fz;
                lx = LinExpr(dynamicsSequence().dynamicsState(time_id).endeffectorPosition(eff_id).x()) - LinExpr(vars_[com_.id(0,time_id)]) + LinExpr(vars_[cop_local_[eff_id].id(0,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))])*rx(0) + LinExpr(vars_[cop_local_[eff_id].id(1,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))])*ry(0);
                ly = LinExpr(dynamicsSequence().dynamicsState(time_id).endeffectorPosition(eff_id).y()) - LinExpr(vars_[com_.id(1,time_id)]) + LinExpr(vars_[cop_local_[eff_id].id(0,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))])*rx(1) + LinExpr(vars_[cop_local_[eff_id].id(1,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))])*ry(1);
                lz = LinExpr(dynamicsSequence().dynamicsState(time_id).endeffectorPosition(eff_id).z()) - LinExpr(vars_[com_.id(2,time_id)]) + LinExpr(vars_[cop_local_[eff_id].id(0,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))])*rx(2) + LinExpr(vars_[cop_local_[eff_id].id(1,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))])*ry(2);
                fx = vars_[frc_world_[eff_id].id(0,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))];
                fy = vars_[frc_world_[eff_id].id(1,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))];
                fz = vars_[frc_world_[eff_id].id(2,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))];

                double lx_val, ly_val, lz_val, fx_val, fy_val, fz_val;
                lx_val = dynamicsSequence().dynamicsState(time_id).endeffectorPosition(eff_id).x() - dynamicsSequence().dynamicsState(time_id).centerOfMass().x() + rx(0)*dynamicsSequence().dynamicsState(time_id).endeffectorCoP(eff_id).x() + ry(0)*dynamicsSequence().dynamicsState(time_id).endeffectorCoP(eff_id).y();
                ly_val = dynamicsSequence().dynamicsState(time_id).endeffectorPosition(eff_id).y() - dynamicsSequence().dynamicsState(time_id).centerOfMass().y() + rx(1)*dynamicsSequence().dynamicsState(time_id).endeffectorCoP(eff_id).x() + ry(1)*dynamicsSequence().dynamicsState(time_id).endeffectorCoP(eff_id).y();
                lz_val = dynamicsSequence().dynamicsState(time_id).endeffectorPosition(eff_id).z() - dynamicsSequence().dynamicsState(time_id).centerOfMass().z() + rx(2)*dynamicsSequence().dynamicsState(time_id).endeffectorCoP(eff_id).x() + ry(2)*dynamicsSequence().dynamicsState(time_id).endeffectorCoP(eff_id).y();
                fx_val = dynamicsSequence().dynamicsState(time_id).endeffectorForce(eff_id).x();
                fy_val = dynamicsSequence().dynamicsState(time_id).endeffectorForce(eff_id).y();
                fz_val = dynamicsSequence().dynamicsState(time_id).endeffectorForce(eff_id).z();

                LinExpr ub_x, lb_x, ub_y, lb_y, ub_z, lb_z;
                ub_x = ( LinExpr(std::pow(-lz_val+fy_val, 2.0) + std::pow( ly_val+fz_val, 2.0)) + ((LinExpr()-lz+fy)-(-lz_val+fy_val))*2.0*(-lz_val+fy_val) + (( ly+fz)-( ly_val+fz_val))*2.0*( ly_val+fz_val) ) - LinExpr(vars_[ub_var_[eff_id].id(0,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))]);
                lb_x = ( LinExpr(std::pow(-lz_val-fy_val, 2.0) + std::pow( ly_val-fz_val, 2.0)) + ((LinExpr()-lz-fy)-(-lz_val-fy_val))*2.0*(-lz_val-fy_val) + (( ly-fz)-( ly_val-fz_val))*2.0*( ly_val-fz_val) ) - LinExpr(vars_[lb_var_[eff_id].id(0,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))]);
                ub_y = ( LinExpr(std::pow( lz_val+fx_val, 2.0) + std::pow(-lx_val+fz_val, 2.0)) + (( lz+fx)-( lz_val+fx_val))*2.0*( lz_val+fx_val) + ((LinExpr()-lx+fz)-(-lx_val+fz_val))*2.0*(-lx_val+fz_val) ) - LinExpr(vars_[ub_var_[eff_id].id(1,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))]);
                lb_y = ( LinExpr(std::pow( lz_val-fx_val, 2.0) + std::pow(-lx_val-fz_val, 2.0)) + (( lz-fx)-( lz_val-fx_val))*2.0*( lz_val-fx_val) + ((LinExpr()-lx-fz)-(-lx_val-fz_val))*2.0*(-lx_val-fz_val) ) - LinExpr(vars_[lb_var_[eff_id].id(1,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))]);
                ub_z = ( LinExpr(std::pow(-ly_val+fx_val, 2.0) + std::pow( lx_val+fy_val, 2.0)) + ((LinExpr()-ly+fx)-(-ly_val+fx_val))*2.0*(-ly_val+fx_val) + (( lx+fy)-( lx_val+fy_val))*2.0*( lx_val+fy_val) ) - LinExpr(vars_[ub_var_[eff_id].id(2,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))]);
                lb_z = ( LinExpr(std::pow(-ly_val-fx_val, 2.0) + std::pow( lx_val-fy_val, 2.0)) + ((LinExpr()-ly-fx)-(-ly_val-fx_val))*2.0*(-ly_val-fx_val) + (( lx-fy)-( lx_val-fy_val))*2.0*( lx_val-fy_val) ) - LinExpr(vars_[lb_var_[eff_id].id(2,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))]);

                double w_soft_constraint = model_.getStgs().get(SolverDoubleParam_SoftConstraintWeight);
                quad_objective_.addQuaTerm(w_soft_constraint, ub_x);
                quad_objective_.addQuaTerm(w_soft_constraint, lb_x);
                quad_objective_.addQuaTerm(w_soft_constraint, ub_y);
                quad_objective_.addQuaTerm(w_soft_constraint, lb_y);
                quad_objective_.addQuaTerm(w_soft_constraint, ub_z);
                quad_objective_.addQuaTerm(w_soft_constraint, lb_z);
              }
            }
          }
        }

        model_.setObjective(quad_objective_, 0.0);

        // constant time horizon with time adaptation
        if (this->getSetting().get(PlannerBoolParam_IsTimeHorizonFixed) && this->getSetting().heuristic() == Heuristic::TimeOptimization) {
          lin_cons_ = 0.0;
          for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++)
                lin_cons_ += vars_[dt_.id(0,time_id)];
          model_.addLinConstr(lin_cons_, "=", this->getSetting().get(PlannerIntParam_NumTimesteps)*this->getSetting().get(PlannerDoubleParam_TimeStep));
        }

        // upper and lower bounds constraints
        for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++) {
              if (this->getSetting().heuristic() == Heuristic::TimeOptimization) {
            model_.addLinConstr(vars_[dt_.id(0,time_id)], ">", this->getSetting().get(PlannerVectorParam_TimeRange)[0]);
            model_.addLinConstr(vars_[dt_.id(0,time_id)], "<", this->getSetting().get(PlannerVectorParam_TimeRange)[1]);
              }
          for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++) {
            if (dynamicsSequence().dynamicsState(time_id).endeffectorActivation(eff_id)) {
              model_.addLinConstr(vars_[trq_local_[eff_id].id(0,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))], ">", this->getSetting().get(PlannerVectorParam_TorqueRange)[0]);
              model_.addLinConstr(vars_[trq_local_[eff_id].id(0,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))], "<", this->getSetting().get(PlannerVectorParam_TorqueRange)[1]);
              model_.addLinConstr(vars_[cop_local_[eff_id].id(0,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))], ">", this->getSetting().get(PlannerArrayParam_CenterOfPressureRange)[eff_id][0]);
              model_.addLinConstr(vars_[cop_local_[eff_id].id(0,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))], "<", this->getSetting().get(PlannerArrayParam_CenterOfPressureRange)[eff_id][1]);
              model_.addLinConstr(vars_[cop_local_[eff_id].id(1,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))], ">", this->getSetting().get(PlannerArrayParam_CenterOfPressureRange)[eff_id][2]);
              model_.addLinConstr(vars_[cop_local_[eff_id].id(1,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))], "<", this->getSetting().get(PlannerArrayParam_CenterOfPressureRange)[eff_id][3]);
            }
          }
        }

        // friction cone constraints
        for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++) {
          for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++) {
            if (dynamicsSequence().dynamicsState(time_id).endeffectorContactType(eff_id) != ContactType::FullContact) {
              if (dynamicsSequence().dynamicsState(time_id).endeffectorActivation(eff_id)) {
                Eigen::Matrix3d eff_rotation = dynamicsSequence().dynamicsState(time_id).endeffectorOrientation(eff_id).toRotationMatrix();
                if (this->getSetting().get(PlannerBoolParam_IsFrictionConeLinear)) {  // using a linear representation
                  Eigen::Matrix<double,4,3> rotated_cone_matrix_ = cone_matrix_*eff_rotation.transpose();
                  for (int row_id=0; row_id<4; row_id++) {
                    lin_cons_ = 0.0;
                    for (int axis_id=0; axis_id<3; axis_id++)
                      lin_cons_ += LinExpr(vars_[frc_world_[eff_id].id(axis_id,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))])*rotated_cone_matrix_(row_id,axis_id);
                    model_.addLinConstr(lin_cons_, "<", 0.0);
                  }
                } else {  // using a second-order cone representation
                  LinExpr fx = 0.0, fy = 0.0, fz = 0.0;
                  for (int axis_id=0; axis_id<3; axis_id++) {
                    fx += LinExpr(vars_[frc_world_[eff_id].id(axis_id,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))])*eff_rotation(axis_id,0);
                    fy += LinExpr(vars_[frc_world_[eff_id].id(axis_id,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))])*eff_rotation(axis_id,1);
                    fz += LinExpr(vars_[frc_world_[eff_id].id(axis_id,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))])*eff_rotation(axis_id,2);
                  }
                  quad_cons_.clear();    quad_cons_.addQuaTerm(1.0, fx);    quad_cons_.addQuaTerm(1.0, fy);
                  model_.addSocConstr(quad_cons_, "<", fz*this->getSetting().get(PlannerDoubleParam_FrictionCoefficient));
                }
              }
            }
          }
        }

        for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++) {
              if (this->getSetting().heuristic() == Heuristic::TimeOptimization) {
                if (is_first_time) {
                  // center of mass constraint
                  for (int axis_id=0; axis_id<3; axis_id++) {
                    if (time_id==0) { lin_cons_ = LinExpr(vars_[com_.id(axis_id,time_id)]) - LinExpr(ini_state_.centerOfMass()[axis_id]) - LinExpr(vars_[lmom_.id(axis_id,time_id)])*(dynamicsSequence().dynamicsState(time_id).time()/ini_state_.mass()); }
                    else            { lin_cons_ = LinExpr(vars_[com_.id(axis_id,time_id)]) - LinExpr(vars_[com_.id(axis_id,time_id-1)])  - LinExpr(vars_[lmom_.id(axis_id,time_id)])*(dynamicsSequence().dynamicsState(time_id).time()/ini_state_.mass()); }
                    model_.addLinConstr(lin_cons_, "=", 0.0);
                  }

                  // linear momentum constraint
                  for (int axis_id=0; axis_id<3; axis_id++) {
                    if (time_id==0) { lin_cons_ = LinExpr(vars_[lmom_.id(axis_id,time_id)]) - LinExpr(ini_state_.linearMomentum()[axis_id]) - LinExpr(vars_[lmomd_.id(axis_id,time_id)])*dynamicsSequence().dynamicsState(time_id).time(); }
                    else            { lin_cons_ = LinExpr(vars_[lmom_.id(axis_id,time_id)]) - LinExpr(vars_[lmom_.id(axis_id,time_id-1)])   - LinExpr(vars_[lmomd_.id(axis_id,time_id)])*dynamicsSequence().dynamicsState(time_id).time(); }
                    model_.addLinConstr(lin_cons_, "=", 0.0);
                  }

                  // angular momentum constraint
                  for (int axis_id=0; axis_id<3; axis_id++) {
                    if (time_id==0) { lin_cons_ = LinExpr(vars_[amom_.id(axis_id,time_id)]) - LinExpr(ini_state_.angularMomentum()[axis_id]) - LinExpr(vars_[amomd_.id(axis_id,time_id)])*dynamicsSequence().dynamicsState(time_id).time(); }
                    else            { lin_cons_ = LinExpr(vars_[amom_.id(axis_id,time_id)]) - LinExpr(vars_[amom_.id(axis_id,time_id-1)])    - LinExpr(vars_[amomd_.id(axis_id,time_id)])*dynamicsSequence().dynamicsState(time_id).time(); }
                    model_.addLinConstr(lin_cons_, "=", 0.0);
                  }
            } else {
              LinExpr lmom, lmomd, amomd, dt = vars_[dt_.id(0,time_id)];
              double lmom_val, lmomd_val, amomd_val, dt_val = dynamicsSequence().dynamicsState(time_id).time();

              // center of mass constraint
              for (int axis_id=0; axis_id<3; axis_id++) {
                lmom  = vars_[lmom_.id(axis_id,time_id)];
                lmom_val  = dynamicsSequence().dynamicsState(time_id).linearMomentum()[axis_id];
                lin_cons_ = 0.0;
                if (time_id==0) { lin_cons_ += LinExpr(ini_state_.centerOfMass()[axis_id]) - LinExpr(vars_[com_.id(axis_id,time_id)]); }
                else            { lin_cons_ += LinExpr(vars_[com_.id(axis_id,time_id-1)])  - LinExpr(vars_[com_.id(axis_id,time_id)]); }
                lin_cons_ += ( LinExpr(std::pow(dt_val+lmom_val, 2.0)) + ((dt+lmom)-(dt_val+lmom_val))*2.0*(dt_val+lmom_val) )*(0.25/ini_state_.mass())
                           - ( LinExpr(std::pow(dt_val-lmom_val, 2.0)) + ((dt-lmom)-(dt_val-lmom_val))*2.0*(dt_val-lmom_val) )*(0.25/ini_state_.mass());
                model_.addLinConstr(lin_cons_, "=", 0.0);
              }

              // linear momentum constraint
              for (int axis_id=0; axis_id<3; axis_id++) {
                lmomd  = vars_[lmomd_.id(axis_id,time_id)];
                lmomd_val  = dynamicsSequence().dynamicsState(time_id).linearMomentumRate()[axis_id];
                lin_cons_ = 0.0;
                if (time_id==0) { lin_cons_ += LinExpr(ini_state_.linearMomentum()[axis_id]) - LinExpr(vars_[lmom_.id(axis_id,time_id)]); }
                else            { lin_cons_ += LinExpr(vars_[lmom_.id(axis_id,time_id-1)]) - LinExpr(vars_[lmom_.id(axis_id,time_id)]); }
                lin_cons_ += ( LinExpr(std::pow(dt_val+lmomd_val, 2.0)) + ((dt+lmomd)-(dt_val+lmomd_val))*2.0*(dt_val+lmomd_val) )*0.25
                           - ( LinExpr(std::pow(dt_val-lmomd_val, 2.0)) + ((dt-lmomd)-(dt_val-lmomd_val))*2.0*(dt_val-lmomd_val) )*0.25;
                model_.addLinConstr(lin_cons_, "=", 0.0);
              }

              // angular momentum constraint
              for (int axis_id=0; axis_id<3; axis_id++) {
                amomd  = vars_[amomd_.id(axis_id,time_id)];
                amomd_val  = dynamicsSequence().dynamicsState(time_id).angularMomentumRate()[axis_id];
                lin_cons_ = 0.0;
                if (time_id==0) { lin_cons_ += LinExpr(ini_state_.angularMomentum()[axis_id]) - LinExpr(vars_[amom_.id(axis_id,time_id)]); }
                else            { lin_cons_ += LinExpr(vars_[amom_.id(axis_id,time_id-1)]) - LinExpr(vars_[amom_.id(axis_id,time_id)]); }
                lin_cons_ += ( LinExpr(std::pow(dt_val+amomd_val, 2.0)) + ((dt+amomd)-(dt_val+amomd_val))*2.0*(dt_val+amomd_val) )*0.25
                           - ( LinExpr(std::pow(dt_val-amomd_val, 2.0)) + ((dt-amomd)-(dt_val-amomd_val))*2.0*(dt_val-amomd_val) )*0.25;
                model_.addLinConstr(lin_cons_, "=", 0.0);
              }
            }

            // linear momentum rate constraint
            for (int axis_id=0; axis_id<3; axis_id++) {
              lin_cons_ = LinExpr(ini_state_.mass()*this->getSetting().get(PlannerVectorParam_GravityVector)[axis_id]) - LinExpr(vars_[lmomd_.id(axis_id,time_id)]);
              for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++)
                if (dynamicsSequence().dynamicsState(time_id).endeffectorActivation(eff_id)) { lin_cons_ += LinExpr(vars_[frc_world_[eff_id].id(axis_id,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))])*mass_times_gravity_; }
              model_.addLinConstr(lin_cons_, "=", 0.0);
            }

            // angular momentum rate constraint
            for (int axis_id=0; axis_id<3; axis_id++) {
              lin_cons_ = LinExpr() - LinExpr(vars_[amomd_.id(axis_id,time_id)]);
              for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++) {
                if (dynamicsSequence().dynamicsState(time_id).endeffectorActivation(eff_id)) {
                  Eigen::Matrix3d rot = dynamicsSequence().dynamicsState(time_id).endeffectorOrientation(eff_id).toRotationMatrix();
                  lin_cons_ += (LinExpr(vars_[ub_var_[eff_id].id(axis_id,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))])-LinExpr(vars_[lb_var_[eff_id].id(axis_id,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))]))*0.25*mass_times_gravity_;
                  lin_cons_ += LinExpr(vars_[trq_local_[eff_id].id(0,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))])*rot.col(2)[axis_id];
                }
              }
              model_.addLinConstr(lin_cons_, "=", 0.0);
            }
              } else {
            // center of mass constraint
            for (int axis_id=0; axis_id<3; axis_id++) {
              if (time_id==0) { lin_cons_ = LinExpr(vars_[com_.id(axis_id,time_id)]) - LinExpr(ini_state_.centerOfMass()[axis_id]) - LinExpr(vars_[lmom_.id(axis_id,time_id)])*(dynamicsSequence().dynamicsState(time_id).time()/ini_state_.mass()); }
              else            { lin_cons_ = LinExpr(vars_[com_.id(axis_id,time_id)]) - LinExpr(vars_[com_.id(axis_id,time_id-1)])  - LinExpr(vars_[lmom_.id(axis_id,time_id)])*(dynamicsSequence().dynamicsState(time_id).time()/ini_state_.mass()); }
              model_.addLinConstr(lin_cons_, "=", 0.0);
            }

            // linear momentum constraint
            for (int axis_id=0; axis_id<3; axis_id++) {
              if (time_id==0) { lin_cons_ = LinExpr(vars_[lmom_.id(axis_id,time_id)]) - LinExpr(dynamicsSequence().dynamicsState(time_id).time()*(ini_state_.mass()*this->getSetting().get(PlannerVectorParam_GravityVector)[axis_id])) - LinExpr(ini_state_.linearMomentum()(axis_id)); }
              else            { lin_cons_ = LinExpr(vars_[lmom_.id(axis_id,time_id)]) - LinExpr(dynamicsSequence().dynamicsState(time_id).time()*(ini_state_.mass()*this->getSetting().get(PlannerVectorParam_GravityVector)[axis_id])) - LinExpr(vars_[lmom_.id(axis_id,time_id-1)]); }

              for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++)
                if (dynamicsSequence().dynamicsState(time_id).endeffectorActivation(eff_id)) { lin_cons_ += LinExpr(vars_[frc_world_[eff_id].id(axis_id,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))])*(-dynamicsSequence().dynamicsState(time_id).time()*mass_times_gravity_); }
              if (time_id==0) { lin_cons_ += LinExpr(this->getSetting().get(PlannerVectorParam_ExternalForce)[axis_id])*(-dynamicsSequence().dynamicsState(time_id).time()*mass_times_gravity_); }
              model_.addLinConstr(lin_cons_, "=", 0.0);
            }

            // angular momentum constraint
            for (int axis_id=0; axis_id<3; axis_id++) {
              if (time_id==0) { lin_cons_ = LinExpr(vars_[amom_.id(axis_id,time_id)]) - LinExpr(ini_state_.angularMomentum()[axis_id]); }
              else            { lin_cons_ = LinExpr(vars_[amom_.id(axis_id,time_id)]) - LinExpr(vars_[amom_.id(axis_id,time_id-1)]); }

              for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++) {
                if (dynamicsSequence().dynamicsState(time_id).endeffectorActivation(eff_id)) {
                  lin_cons_ += (LinExpr(vars_[lb_var_[eff_id].id(axis_id,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))])-LinExpr(vars_[ub_var_[eff_id].id(axis_id,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))]))*(0.25*dynamicsSequence().dynamicsState(time_id).time()*mass_times_gravity_);

                  Eigen::Vector3d rz = dynamicsSequence().dynamicsState(time_id).endeffectorOrientation(eff_id).toRotationMatrix().col(2);
                  lin_cons_ += LinExpr(vars_[trq_local_[eff_id].id(0,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))])*(-rz[axis_id]*dynamicsSequence().dynamicsState(time_id).time());
                }
              }
              model_.addLinConstr(lin_cons_, "=", 0.0);
            }
          }
        }

        QuadConstrApprox qapprox = QuadConstrApprox::None;
        switch (this->getSetting().heuristic()) {
          case Heuristic::TimeOptimization: { qapprox = QuadConstrApprox::None; break; }
          case Heuristic::TrustRegion: { qapprox = QuadConstrApprox::TrustRegion; break; }
          case Heuristic::SoftConstraint: { qapprox = QuadConstrApprox::SoftConstraint; break; }
          default: { qapprox = QuadConstrApprox::None; break; }
        }

        for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++) {
          for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++) {
            if (dynamicsSequence().dynamicsState(time_id).endeffectorActivation(eff_id)) {
              Eigen::Matrix3d rot = dynamicsSequence().dynamicsState(time_id).endeffectorOrientation(eff_id).toRotationMatrix();
              Eigen::Vector3d rx = rot.col(0);
              Eigen::Vector3d ry = rot.col(1);

              LinExpr lx, ly , lz, fx, fy, fz;
              lx = LinExpr(dynamicsSequence().dynamicsState(time_id).endeffectorPosition(eff_id).x()) - LinExpr(vars_[com_.id(0,time_id)]) + LinExpr(vars_[cop_local_[eff_id].id(0,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))])*rx(0) + LinExpr(vars_[cop_local_[eff_id].id(1,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))])*ry(0);
              ly = LinExpr(dynamicsSequence().dynamicsState(time_id).endeffectorPosition(eff_id).y()) - LinExpr(vars_[com_.id(1,time_id)]) + LinExpr(vars_[cop_local_[eff_id].id(0,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))])*rx(1) + LinExpr(vars_[cop_local_[eff_id].id(1,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))])*ry(1);
              lz = LinExpr(dynamicsSequence().dynamicsState(time_id).endeffectorPosition(eff_id).z()) - LinExpr(vars_[com_.id(2,time_id)]) + LinExpr(vars_[cop_local_[eff_id].id(0,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))])*rx(2) + LinExpr(vars_[cop_local_[eff_id].id(1,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))])*ry(2);
              fx = vars_[frc_world_[eff_id].id(0,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))];
              fy = vars_[frc_world_[eff_id].id(1,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))];
              fz = vars_[frc_world_[eff_id].id(2,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))];

              // upper and lower bounds on momentum rate constraints
              quad_cons_.clear();    quad_cons_.addQuaTerm(1.0, LinExpr()-lz+fy);    quad_cons_.addQuaTerm(1.0,  ly+fz);
              model_.addQuaConstr(quad_cons_, "<", vars_[ub_var_[eff_id].id(0,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))], qapprox);

              quad_cons_.clear();    quad_cons_.addQuaTerm(1.0,  lz+fx);    quad_cons_.addQuaTerm(1.0, LinExpr()-lx+fz);
              model_.addQuaConstr(quad_cons_, "<", vars_[ub_var_[eff_id].id(1,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))], qapprox);

              quad_cons_.clear();    quad_cons_.addQuaTerm(1.0, LinExpr()-ly+fx);    quad_cons_.addQuaTerm(1.0,  lx+fy);
              model_.addQuaConstr(quad_cons_, "<", vars_[ub_var_[eff_id].id(2,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))], qapprox);

              quad_cons_.clear();    quad_cons_.addQuaTerm(1.0, LinExpr()-lz-fy);    quad_cons_.addQuaTerm(1.0,  ly-fz);
              model_.addQuaConstr(quad_cons_, "<", vars_[lb_var_[eff_id].id(0,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))], qapprox);

              quad_cons_.clear();    quad_cons_.addQuaTerm(1.0,  lz-fx);    quad_cons_.addQuaTerm(1.0, LinExpr()-lx-fz);
              model_.addQuaConstr(quad_cons_, "<", vars_[lb_var_[eff_id].id(1,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))], qapprox);

              quad_cons_.clear();    quad_cons_.addQuaTerm(1.0, LinExpr()-ly-fx);    quad_cons_.addQuaTerm(1.0,  lx-fy);
              model_.addQuaConstr(quad_cons_, "<", vars_[lb_var_[eff_id].id(2,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))], qapprox);

              // end-effector length constraint
              quad_cons_.clear();  quad_cons_.addQuaTerm(1.0, lx-this->getSetting().get(PlannerArrayParam_EndeffectorOffset)[eff_id].x());  quad_cons_.addQuaTerm(1.0, ly-this->getSetting().get(PlannerArrayParam_EndeffectorOffset)[eff_id].y());  quad_cons_.addQuaTerm(1.0, lz-this->getSetting().get(PlannerArrayParam_EndeffectorOffset)[eff_id].z());
              model_.addQuaConstr(quad_cons_, "<", pow(this->getSetting().get(PlannerVectorParam_MaxEndeffectorLengths)[eff_id],2.0));
            }
          }
        }

        // formulate problem in standard conic form and solve it
        timer_.start();
        exitcode_ = model_.optimize();
        solve_time_ += timer_.stop();

        // extract solution
        solution_.resize(num_vars_, 1);  solution_.setZero();
        for (int var_id=0; var_id<num_vars_; var_id++) { solution_(var_id) = vars_[var_id].get(SolverDoubleParam_X); }
      }
      catch(...)
      {
        std::cout << "Exception during optimization" << std::endl;
      }

      if (this->getSetting().heuristic() == Heuristic::TimeOptimization)
        saveSolution(dt_);
        saveSolution(com_);
        saveSolution(amom_);
        saveSolution(lmom_);

      if (this->getSetting().heuristic() == Heuristic::TimeOptimization) {
        saveSolution(lmomd_);
        saveSolution(amomd_);
      }
      for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++) {
        saveSolution(frc_world_[eff_id]);
        saveSolution(cop_local_[eff_id]);
        saveSolution(trq_local_[eff_id]);
      }

      // computation of linear momentum out of forces
      Eigen::Vector3d frc, len, trq;
      Eigen::MatrixXd compos(3,this->getSetting().get(PlannerIntParam_NumTimesteps)); compos.setZero();
      Eigen::MatrixXd linmom(3,this->getSetting().get(PlannerIntParam_NumTimesteps)); linmom.setZero();
      Eigen::MatrixXd angmom(3,this->getSetting().get(PlannerIntParam_NumTimesteps)); angmom.setZero();

      if (this->getSetting().heuristic() == Heuristic::TimeOptimization) {
        for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++) {
          if (time_id==0) {
            compos.col(time_id) = ini_state_.centerOfMass();
            linmom.col(time_id) = ini_state_.linearMomentum();
          } else {
            compos.col(time_id) = compos.col(time_id-1);
            linmom.col(time_id) = linmom.col(time_id-1);
          }

          linmom.col(time_id) += ini_state_.mass()*this->getSetting().get(PlannerVectorParam_GravityVector)*vars_[dt_.id(0,time_id)].get(SolverDoubleParam_X);
          for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++) {
            if (dynamicsSequence().dynamicsState(time_id).endeffectorActivation(eff_id)) {
              frc.x() = mass_times_gravity_*vars_[frc_world_[eff_id].id(0,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))].get(SolverDoubleParam_X);
              frc.y() = mass_times_gravity_*vars_[frc_world_[eff_id].id(1,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))].get(SolverDoubleParam_X);
              frc.z() = mass_times_gravity_*vars_[frc_world_[eff_id].id(2,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))].get(SolverDoubleParam_X);
              linmom.col(time_id).head(3) += vars_[dt_.id(0,time_id)].get(SolverDoubleParam_X)*frc;
            }
          }
          compos.col(time_id).head(3) += 1.0/ini_state_.mass()*vars_[dt_.id(0,time_id)].get(SolverDoubleParam_X)*linmom.col(time_id).head(3);
        }
      }

      // computation of angular momentum out of forces and lengths
      for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++) {
        if (time_id == 0) { angmom.col(time_id) = ini_state_.angularMomentum(); }
        else              { angmom.col(time_id) = angmom.col(time_id-1); }

        for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++) {
          if (dynamicsSequence().dynamicsState(time_id).endeffectorActivation(eff_id)) {
            Eigen::Matrix3d rot = dynamicsSequence().dynamicsState(time_id).endeffectorOrientation(eff_id).toRotationMatrix();
            Eigen::Vector3d rx = rot.col(0);
            Eigen::Vector3d ry = rot.col(1);
            Eigen::Vector3d rz = rot.col(2);

            if (this->getSetting().heuristic() == Heuristic::TimeOptimization) {
              len.x() = dynamicsSequence().dynamicsState(time_id).endeffectorPosition(eff_id).x() - compos(0,time_id) + rx(0)*vars_[cop_local_[eff_id].id(0,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))].get(SolverDoubleParam_X) + ry(0)*vars_[cop_local_[eff_id].id(1,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))].get(SolverDoubleParam_X);
              len.y() = dynamicsSequence().dynamicsState(time_id).endeffectorPosition(eff_id).y() - compos(1,time_id) + rx(1)*vars_[cop_local_[eff_id].id(0,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))].get(SolverDoubleParam_X) + ry(1)*vars_[cop_local_[eff_id].id(1,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))].get(SolverDoubleParam_X);
              len.z() = dynamicsSequence().dynamicsState(time_id).endeffectorPosition(eff_id).z() - compos(2,time_id) + rx(2)*vars_[cop_local_[eff_id].id(0,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))].get(SolverDoubleParam_X) + ry(2)*vars_[cop_local_[eff_id].id(1,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))].get(SolverDoubleParam_X);
            } else {
              len.x() = dynamicsSequence().dynamicsState(time_id).endeffectorPosition(eff_id).x() - vars_[com_.id(0,time_id)].get(SolverDoubleParam_X) + rx(0)*vars_[cop_local_[eff_id].id(0,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))].get(SolverDoubleParam_X) + ry(0)*vars_[cop_local_[eff_id].id(1,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))].get(SolverDoubleParam_X);
              len.y() = dynamicsSequence().dynamicsState(time_id).endeffectorPosition(eff_id).y() - vars_[com_.id(1,time_id)].get(SolverDoubleParam_X) + rx(1)*vars_[cop_local_[eff_id].id(0,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))].get(SolverDoubleParam_X) + ry(1)*vars_[cop_local_[eff_id].id(1,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))].get(SolverDoubleParam_X);
              len.z() = dynamicsSequence().dynamicsState(time_id).endeffectorPosition(eff_id).z() - vars_[com_.id(2,time_id)].get(SolverDoubleParam_X) + rx(2)*vars_[cop_local_[eff_id].id(0,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))].get(SolverDoubleParam_X) + ry(2)*vars_[cop_local_[eff_id].id(1,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))].get(SolverDoubleParam_X);
            }
            frc.x() = mass_times_gravity_*vars_[frc_world_[eff_id].id(0,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))].get(SolverDoubleParam_X);
            frc.y() = mass_times_gravity_*vars_[frc_world_[eff_id].id(1,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))].get(SolverDoubleParam_X);
            frc.z() = mass_times_gravity_*vars_[frc_world_[eff_id].id(2,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))].get(SolverDoubleParam_X);
            trq     = rz * vars_[trq_local_[eff_id].id(0,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id))].get(SolverDoubleParam_X);
            angmom.col(time_id).head(3) += dynamicsSequence().dynamicsState(time_id).time()*(len.cross(frc)+trq);
          }
        }
      }
      if (this->getSetting().heuristic() != Heuristic::TimeOptimization) { amom_.setGuessValue(angmom); }
      storeSolution();

      if (this->getSetting().heuristic() == Heuristic::TimeOptimization) {
        // computing convergence error
        if (is_first_time) { last_convergence_err_ = SolverSetting::inf; }
        else               { last_convergence_err_ = convergence_err_; }
        com_.getGuessValue(com_guess_);   double com_err  = (com_guess_-compos).norm()/this->getSetting().get(PlannerIntParam_NumTimesteps);
        lmom_.getGuessValue(lmom_guess_); double lmom_err = (lmom_guess_-linmom).norm()/this->getSetting().get(PlannerIntParam_NumTimesteps);
        amom_.getGuessValue(amom_guess_); double amom_err = (amom_guess_-angmom).norm()/this->getSetting().get(PlannerIntParam_NumTimesteps);
        convergence_err_ = std::max(com_err, std::max(lmom_err, amom_err));

        if (convergence_err_ < this->getSetting().get(PlannerDoubleParam_MaxTimeResidualTolerance)) { has_converged_ = true; }
        if (std::abs(last_convergence_err_-convergence_err_) < this->getSetting().get(PlannerDoubleParam_MinTimeResidualImprovement)) { has_converged_ = true; }
        if (has_converged_) { amom_.setGuessValue(angmom); storeSolution(); }
      }
    }

    void DynamicsOptimizer::addVariableToModel(const OptimizationVariable& opt_var, Model& model, std::vector<Var>& vars)
    {
      opt_var.getValues(mat_lb_, mat_ub_, mat_guess_, size_, variable_type_);
      for (int col_id=0; col_id<opt_var.getNumCols(); col_id++)
        for (int row_id=0; row_id<opt_var.getNumRows(); row_id++)
          switch (variable_type_) {
            case 'C': { vars[opt_var.id(row_id,col_id)] = model.addVar(VarType::Continuous, double(mat_lb_(row_id,col_id)), double(mat_ub_(row_id,col_id)), double(mat_guess_(row_id,col_id))); break; }
            default: { throw std::runtime_error("At add_var_to_model, variable type not handled"); }
          }

      for (int col_id=0; col_id<opt_var.getNumCols(); col_id++)
        for (int row_id=0; row_id<opt_var.getNumRows(); row_id++)
          vars[opt_var.id(row_id,col_id)].set(SolverDoubleParam_X, mat_guess_(row_id,col_id));
    }

    void DynamicsOptimizer::saveSolution(OptimizationVariable& opt_var)
    {
      mat_guess_.resize(opt_var.getNumRows(), opt_var.getNumCols()); mat_guess_.setZero();
      for (int col_id=0; col_id<opt_var.getNumCols(); col_id++) {
        for (int row_id=0; row_id<opt_var.getNumRows(); row_id++)
              mat_guess_(row_id,col_id) = solution_(opt_var.id(row_id,col_id));
      }
      opt_var.setGuessValue(mat_guess_);
    }

    void DynamicsOptimizer::storeSolution()
    {
      com_.getGuessValue(mat_guess_);
      for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++) {
        dynamicsSequence().dynamicsState(time_id).centerOfMass() = Eigen::Vector3d(mat_guess_.block<3,1>(0,time_id));
      }
      lmom_.getGuessValue(mat_guess_);
      for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++)
        dynamicsSequence().dynamicsState(time_id).linearMomentum() = Eigen::Vector3d(mat_guess_.block<3,1>(0,time_id));

      amom_.getGuessValue(mat_guess_);
      for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++)
        dynamicsSequence().dynamicsState(time_id).angularMomentum() = Eigen::Vector3d(mat_guess_.block<3,1>(0,time_id));

      if (this->getSetting().heuristic() == Heuristic::TimeOptimization) {
        dt_.getGuessValue(mat_guess_);
        for (int time=0; time<this->getSetting().get(PlannerIntParam_NumTimesteps); time++)
          dynamicsSequence().dynamicsState(time).time() = mat_guess_(0,time);

        lmomd_.getGuessValue(mat_guess_);
        for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++)
          dynamicsSequence().dynamicsState(time_id).linearMomentumRate() = Eigen::Vector3d(mat_guess_.block<3,1>(0,time_id));

        amomd_.getGuessValue(mat_guess_);
        for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++)
          dynamicsSequence().dynamicsState(time_id).angularMomentumRate() = Eigen::Vector3d(mat_guess_.block<3,1>(0,time_id));
      }

      for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++) {
        frc_world_[eff_id].getGuessValue(mat_guess_);
        for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++)
          if (dynamicsSequence().dynamicsState(time_id).endeffectorActivation(eff_id))
            dynamicsSequence().dynamicsState(time_id).endeffectorForce(eff_id) = Eigen::Vector3d(mat_guess_.block<3,1>(0,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id)));

        cop_local_[eff_id].getGuessValue(mat_guess_);
        for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++)
          if (dynamicsSequence().dynamicsState(time_id).endeffectorActivation(eff_id))
            dynamicsSequence().dynamicsState(time_id).endeffectorCoP(eff_id) = Eigen::Vector3d(mat_guess_(0,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id)), mat_guess_(1,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id)), 0.0);

        trq_local_[eff_id].getGuessValue(mat_guess_);
        for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++)
          if (dynamicsSequence().dynamicsState(time_id).endeffectorActivation(eff_id))
            dynamicsSequence().dynamicsState(time_id).endeffectorTorque(eff_id) = Eigen::Vector3d(0.0, 0.0, mat_guess_(0,dynamicsSequence().dynamicsState(time_id).endeffectorActivationId(eff_id)));
      }
    }

    void DynamicsOptimizer::saveToFile(const DynamicsSequence& ref_sequence)
    {
      if (this->getSetting().get(PlannerBoolParam_StoreData)) {
        try
        {
          YAML::Node cfg_pars = YAML::LoadFile(this->getSetting().get(PlannerStringParam_ConfigFile).c_str());

          YAML::Node qcqp_cfg;
          qcqp_cfg["dynopt_params"]["time_step"] = this->getSetting().get(PlannerDoubleParam_TimeStep);
          qcqp_cfg["dynopt_params"]["end_com"] = com_pos_goal_;
          qcqp_cfg["dynopt_params"]["robot_mass"] = ini_state_.mass();
          qcqp_cfg["dynopt_params"]["n_act_eefs"] = this->getSetting().get(PlannerIntParam_NumActiveEndeffectors);
          qcqp_cfg["dynopt_params"]["ini_com"] = ini_state_.centerOfMass();
          qcqp_cfg["dynopt_params"]["time_horizon"] = this->getSetting().get(PlannerDoubleParam_TimeHorizon);

          com_.getGuessValue(mat_guess_);   qcqp_cfg["dynopt_params"]["com_motion"] = mat_guess_;
          lmom_.getGuessValue(mat_guess_);  qcqp_cfg["dynopt_params"]["lin_mom"] = mat_guess_;
          amom_.getGuessValue(mat_guess_);  qcqp_cfg["dynopt_params"]["ang_mom"] = mat_guess_;

          // building momentum references
          for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++) { mat_guess_.col(time_id) = ref_sequence.dynamicsState(time_id).centerOfMass();  } qcqp_cfg["dynopt_params"]["com_motion_ref"] = mat_guess_;
          for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++) { mat_guess_.col(time_id) = ref_sequence.dynamicsState(time_id).linearMomentum(); } qcqp_cfg["dynopt_params"]["lin_mom_ref"] = mat_guess_;
          for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++) { mat_guess_.col(time_id) = ref_sequence.dynamicsState(time_id).angularMomentum(); } qcqp_cfg["dynopt_params"]["ang_mom_ref"] = mat_guess_;

          // saving vector of time-steps
          mat_guess_.resize(1, this->getSetting().get(PlannerIntParam_NumTimesteps)); mat_guess_.setZero();
          mat_guess_(0,0) = dynamicsSequence().dynamicsState(0).time();
          for (int time_id=1; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++)
            mat_guess_(0,time_id) = mat_guess_(0,time_id-1) + dynamicsSequence().dynamicsState(time_id).time();
          qcqp_cfg["dynopt_params"]["time_vec"] = mat_guess_;

          // saving vector of forces, torques and cops
          for (int eff_id=0; eff_id<this->getSetting().get(PlannerIntParam_NumActiveEndeffectors); eff_id++) {
            mat_guess_.resize(3, this->getSetting().get(PlannerIntParam_NumTimesteps)); mat_guess_.setZero();
            for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++)
              if (dynamicsSequence().dynamicsState(time_id).endeffectorActivation(eff_id))
                mat_guess_.col(time_id).head(3) = dynamicsSequence().dynamicsState(time_id).endeffectorForce(eff_id);
            qcqp_cfg["dynopt_params"]["eef_frc_"+std::to_string(eff_id)] = mat_guess_;

            mat_guess_.resize(1, this->getSetting().get(PlannerIntParam_NumTimesteps)); mat_guess_.setZero();
            for (int time_id=0; time_id<this->getSetting().get(PlannerIntParam_NumTimesteps); time_id++)
              if (dynamicsSequence().dynamicsState(time_id).endeffectorActivation(eff_id))
                mat_guess_(0, time_id) = dynamicsSequence().dynamicsState(time_id).endeffectorTorque(eff_id).z();
            qcqp_cfg["dynopt_params"]["eef_trq_"+std::to_string(eff_id)] = mat_guess_;

            cop_local_[eff_id].getGuessValue(mat_guess_);
            qcqp_cfg["dynopt_params"]["eef_cop_"+std::to_string(eff_id)] = mat_guess_;

            // Contact information
            vector_t num_con(4); num_con.setZero();
            vector_t foot_state(9); foot_state.setZero();

            for (int eff_id=0; eff_id<Problem::n_endeffs_; eff_id++){
              int num = contact_plan_->contactSequence().endeffectorContacts(eff_id).size();
              num_con(eff_id) = num;

              for (int cnt_id = 0; cnt_id < num; cnt_id++){
                foot_state(0) = contact_plan_->contactSequence().endeffectorContacts(eff_id)[cnt_id].contactActivationTime();
                foot_state(1) = contact_plan_->contactSequence().endeffectorContacts(eff_id)[cnt_id].contactDeactivationTime();
                foot_state.segment(2,3) = contact_plan_->contactSequence().endeffectorContacts(eff_id)[cnt_id].contactPosition();
                foot_state(5) = contact_plan_->contactSequence().endeffectorContacts(eff_id)[cnt_id].contactOrientation().x();
                foot_state(6) = contact_plan_->contactSequence().endeffectorContacts(eff_id)[cnt_id].contactOrientation().y();
                foot_state(7) = contact_plan_->contactSequence().endeffectorContacts(eff_id)[cnt_id].contactOrientation().z();
                foot_state(8) = contact_plan_->contactSequence().endeffectorContacts(eff_id)[cnt_id].contactOrientation().w();
                qcqp_cfg["cntopt_params"]["contact_sequence_" + std::to_string(eff_id)+"  #" + std::to_string(cnt_id)] = foot_state;
              }              
            }          
            qcqp_cfg["cntopt_params"]["numbers of contact sequence"] = num_con; 
          }
          std::ofstream file_out(this->getSetting().get(PlannerStringParam_SaveDynamicsFile)); file_out << qcqp_cfg;
        }
        catch (YAML::ParserException &e) { std::cout << e.what() << "\n"; }
      }
    }



  }
}
