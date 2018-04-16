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

#include <Eigen/Eigen>
#include <hpp/timeopt/fwd.hh>

namespace hpp{
    namespace timeopt {        
        inline vector3_t ConvertHPPframe(const vector3_t vec) {
            return Eigen::Vector3d(-vec(1), vec(0), vec(2));
        }
        inline matrix3_t ConvertHPPframe(const matrix3_t mat) {
            matrix3_t mat_conv;
            mat_conv.setZero(); mat_conv(0, 1) = -1.0; mat_conv(1, 0) = 1.0; mat_conv(2, 2) = 1.0;

            return mat_conv*mat;
        }
        inline Eigen::MatrixXd ConvertHPPframe(const Eigen::MatrixXd mat) {
            matrix3_t mat_conv;
            mat_conv.setZero(); mat_conv(0, 1) = -1.0; mat_conv(1, 0) = 1.0; mat_conv(2, 2) = 1.0;

            return mat_conv*mat;
        }
        inline vector3_t ConvertOptframe(const vector3_t vec){
            return Eigen::Vector3d(vec(1), -vec(0), vec(2));
        }
        inline Eigen::Quaternion<double> ConvertOptframe(const Eigen::Quaternion<double> quat) {
            matrix3_t rot = quat.toRotationMatrix();
            matrix3_t mat_conv;
            mat_conv.setZero(); mat_conv(0, 1) = 1.0; mat_conv(1, 0) = -1.0; mat_conv(2, 2) = 1.0;
            Eigen::Quaternion<double> quat_new;
            quat_new = mat_conv*rot;

            return quat_new;
        }
      };
}

