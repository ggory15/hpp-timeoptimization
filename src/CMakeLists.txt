#
# Copyright (c) 2015 CNRS
# Authors: Florent Lamiraux
#
#
# This file is part of hpp-walkgen
# hpp-walkgen is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-walkgen is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-walkgen  If not, see
# <http://www.gnu.org/licenses/>.

SET(LIBRARY_NAME ${PROJECT_NAME})

SET(${LIBRARY_NAME}_SOURCES
  opt_solver/optimizer/EqRoutine.cpp
  opt_solver/optimizer/InfoPrinter.cpp
  opt_solver/optimizer/IPSolver.cpp
  opt_solver/optimizer/LinSolver.cpp
  opt_solver/optimizer/SparseCholesky.cpp

  opt_solver/interface/Cone.cpp
  opt_solver/interface/SolverSetting.cpp
  opt_solver/interface/Exprs.cpp
  opt_solver/interface/Model.cpp
  opt_solver/interface/OptVar.cpp
  opt_solver/interface/Var.cpp

  momentumopt/cntopt/ContactState.cpp
  momentumopt/cntopt/ContactPlanFromFootPrint.cpp
  momentumopt/cntopt/ContactPlanInterface.cpp

  momentumopt/dynopt/DynamicsState.cpp
  momentumopt/dynopt/DynamicsOptimizer.cpp

  momentumopt/setting/PlannerSetting.cpp
  momentumopt/setting/Robot-state.cpp
  )

ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  ${${LIBRARY_NAME}_SOURCES}
  )

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-core)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-pinocchio)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-util)

TARGET_LINK_LIBRARIES (${LIBRARY_NAME} ${Boost_LIBRARIES})

INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_DEFINITIONS CFG_SRC_PATH="${CONFIG_PATH}/")
