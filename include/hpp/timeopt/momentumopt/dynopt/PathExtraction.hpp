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

#include <array>
#include <vector>
#include <string>
#include <iostream>
#include <Eigen/Eigen>

#include <hpp/timeopt/fwd.hh>
#include <hpp/core/path-vector.hh>
#include <hpp/core/path.hh>
#include <hpp/timeopt/config.hh>

namespace hpp{
    namespace timeopt {
        class HPP_timeopt_DLLAPI COMPath : public core::Path
        {
            public:
            static COMPathPtr_t create (const core::PathPtr_t path)
            {
              COMPathPtr_t ptr (new COMPath (path));
              ptr->init (ptr);
              return ptr;
            }
            static COMPathPtr_t createCopy (const COMPath& other)
            {
              COMPathPtr_t ptr (new COMPath (other));
              ptr->init (ptr);
              return ptr;
            }
            static COMPathPtr_t createCopy (const COMPath& other,
                const ConstraintSetPtr_t& c)
            {
              COMPathPtr_t ptr (new COMPath (other, c));
              ptr->init (ptr);
              return ptr;
            }
            void add (const Eigen::Vector3d& td)
            {
              tds_.push_back (td);
            }
            virtual Configuration_t initial () const {
              return path_->initial ();
            }

            virtual Configuration_t end () const {
              return path_->end ();
            }
            virtual core::PathPtr_t copy () const {
              return createCopy (*this);
            }
            virtual core::PathPtr_t copy (const ConstraintSetPtr_t& c) const {
              return createCopy (*this, c);
            }


            virtual ~COMPath () throw () {}

            protected:
            COMPath (const core::PathPtr_t path) :
              Path (path->timeRange (), path->outputSize (),
                  path->outputDerivativeSize ()),
              path_ (path->copy ()), a_ (1), b_(0)
            {}
            COMPath (const COMPath &other) :
              Path (other), path_ (other.path_->copy ()),
              tds_ (other.tds_), a_ (other.a_), b_(other.b_)
            {}
            COMPath (const core::PathPtr_t path, const ConstraintSetPtr_t& c) :
              Path (path->timeRange (), path->outputSize (),
                  path->outputDerivativeSize ()),
              path_ (path->copy ()), a_ (1), b_(0)
            {
              constraints (c);
            }
            COMPath (const COMPath &other, const ConstraintSetPtr_t& c) :
              Path (other), path_ (other.path_->copy ()),
              a_ (other.a_), b_(other.b_)
            {
              constraints (c);
            }


            virtual bool impl_compute (ConfigurationOut_t config, value_type t) const
            {
              return (*path_) (config, t);
            }

            void init (const COMPathPtr_t& self)
            {
              Path::init (self);
            }

            virtual std::ostream& print (std::ostream& os) const {
              return os << "COMPath: " << *path_;
            }

            private:
            typedef std::vector <Eigen::Vector3d> COMpose_t;

            core::PathPtr_t path_;
            COMpose_t tds_;
            value_type a_, b_;
    };
  }
}
