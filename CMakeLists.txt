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

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CXX_DISABLE_WERROR TRUE)
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/eigen.cmake)
INCLUDE(cmake/test.cmake)


ADD_REQUIRED_DEPENDENCY("eigen3 >= 3.0")
ADD_REQUIRED_DEPENDENCY("yaml-cpp >= 0.5")
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${YAML-CPP_INCLUDE_DIRS})

set(BUILD_DOCUMENTATION OFF)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -g -Wall -Wwrite-strings -pedantic -O3 -funroll-loops -fPIC")


SET(PROJECT_NAME hpp-timeopt)
SET(PROJECT_DESCRIPTION
  "COM Generator based on Pondon's works for hpp framework"
  )
SET(PROJECT_URL "")

SETUP_PROJECT()
SEARCH_FOR_BOOST()

# Activate hpp-util logging if requested
SET (HPP_DEBUG FALSE CACHE BOOL "trigger hpp-util debug output")
IF (HPP_DEBUG)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_DEBUG")
ENDIF()

set(INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
SET(${PROJECT_NAME}_HEADERS
  include/hpp/timeopt/fwd.hh

  include/hpp/timeopt/momentumopt/cntopt/ContactPlanFromFile.hpp
  include/hpp/timeopt/momentumopt/cntopt/ContactPlanInterface.hpp
  include/hpp/timeopt/momentumopt/cntopt/ContactState.hpp

  include/hpp/timeopt/momentumopt/dynopt/DynamicsOptimizer.hpp
  include/hpp/timeopt/momentumopt/dynopt/DynamicsState.hpp

  include/hpp/timeopt/momentumopt/setting/Definitions.hpp
  include/hpp/timeopt/momentumopt/setting/PlannerParams.hpp
  include/hpp/timeopt/momentumopt/setting/PlannerSetting.hpp

  include/hpp/timeopt/momentumopt/utilities/Clock.hpp

  include/hpp/timeopt/opt_solver/optimizer/EqRoutine.hpp
  include/hpp/timeopt/opt_solver/optimizer/InfoPrinter.hpp
  include/hpp/timeopt/opt_solver/optimizer/IPSolver.hpp
  include/hpp/timeopt/opt_solver/optimizer/LinSolver.hpp
  include/hpp/timeopt/opt_solver/optimizer/SparseCholesky.hpp

  include/hpp/timeopt/opt_solver/interface/Cone.hpp
  include/hpp/timeopt/opt_solver/interface/Exprs.hpp
  include/hpp/timeopt/opt_solver/interface/Model.hpp
  include/hpp/timeopt/opt_solver/interface/OptVar.hpp
  include/hpp/timeopt/opt_solver/interface/Solver.hpp
  include/hpp/timeopt/opt_solver/interface/SolverParams.hpp
  include/hpp/timeopt/opt_solver/interface/SolverSetting.hpp
  include/hpp/timeopt/opt_solver/interface/Var.hpp

  include/hpp/timeopt/yaml_eigen.h
   
)

ADD_REQUIRED_DEPENDENCY("eigen3 >= 3.2")
ADD_REQUIRED_DEPENDENCY("hpp-util >= 3")
ADD_REQUIRED_DEPENDENCY("hpp-constraints >= 4")
ADD_REQUIRED_DEPENDENCY("hpp-pinocchio >= 4")
ADD_REQUIRED_DEPENDENCY("hpp-core >= 4")

SEARCH_FOR_BOOST ()

get_filename_component(TEST_PATH test ABSOLUTE)
get_filename_component(CONFIG_PATH config ABSOLUTE)

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(test/opt_solver)
ADD_SUBDIRECTORY(test/momentumopt)

ADD_SUBDIRECTORY(demos)

PKG_CONFIG_APPEND_LIBS("hpp-timeopt")

SETUP_PROJECT_FINALIZE()
