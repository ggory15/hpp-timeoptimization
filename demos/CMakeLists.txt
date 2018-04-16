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

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)

# ADD_TESTCASE(NAME)
# ------------------------
#
# Define a test named `NAME'.
#
# This macro will create a binary from `NAME.cc', link it against
# Boost and add it to the test suite.
#
MACRO(ADD_TESTCASE NAME GENERATED)
  IF (${GENERATED} STREQUAL TRUE)
    add_executable(${NAME} ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.cpp)
  ELSE()
    add_executable(${NAME} ${NAME}.cpp)
  ENDIF()
  set_target_properties(DemoMomentumopt PROPERTIES COMPILE_DEFINITIONS CFG_SRC_PATH="${CONFIG_PATH}/")
  PKG_CONFIG_USE_DEPENDENCY(${NAME} hpp-core)

  # Link against Boost and project library.
  TARGET_LINK_LIBRARIES(${NAME}
    ${Boost_LIBRARIES} hpp-timeopt yaml-cpp hpp-core hpp-pinocchio
    )
ENDMACRO(ADD_TESTCASE)

ADD_TESTCASE(DemoMomentumopt FALSE)

