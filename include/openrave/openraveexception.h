// -*- coding: utf-8 -*-
// Copyright (C) 2020 OpenRAVE
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/** \file exception.h
    \brief OpenRAVE exception definition.
 */
#ifndef OPENRAVE_EXCEPTION_H
#define OPENRAVE_EXCEPTION_H

#include <openrave/config.h>

#include <exception>

namespace OpenRAVE {

/// %OpenRAVE error codes
enum OpenRAVEErrorCode {
    ORE_Failed=0,
    ORE_InvalidArguments=1, ///< passed in input arguments are not valid
    ORE_EnvironmentNotLocked=2,
    ORE_CommandNotSupported=3, ///< string command could not be parsed or is not supported
    ORE_Assert=4,
    ORE_InvalidPlugin=5, ///< shared object is not a valid plugin
    ORE_InvalidInterfaceHash=6, ///< interface hashes do not match between plugins
    ORE_NotImplemented=7, ///< function is not implemented by the interface.
    ORE_InconsistentConstraints=8, ///< returned solutions or trajectories do not follow the constraints of the planner/module. The constraints invalidated here are planning constraints, not programming constraints.
    ORE_NotInitialized=9, ///< when object is used without it getting fully initialized
    ORE_InvalidState=10, ///< the state of the object is not consistent with its parameters, or cannot be used. This is usually due to a programming error where a vector is not the correct length, etc.
    ORE_Timeout=11, ///< process timed out
    ORE_InvalidURI=12, ///< uri in input scene file is invalid, causing scene loading failures
};

/// \brief Exception that all OpenRAVE internal methods throw; the error codes are held in \ref OpenRAVEErrorCode.
class OPENRAVE_API OpenRAVEException : public std::exception
{
public:
    OpenRAVEException();
    OpenRAVEException(const std::string& s, OpenRAVEErrorCode error=ORE_Failed);
    virtual ~OpenRAVEException() throw() {
    }
    char const* what() const throw();
    const std::string& message() const;
    OpenRAVEErrorCode GetCode() const;
private:
    std::string _s;
    OpenRAVEErrorCode _error;
};

typedef OpenRAVEException openrave_exception;

} // namespace OpenRAVE

#define OPENRAVE_EXCEPTION_FORMAT0(s, errorcode) OpenRAVE::OpenRAVEException(boost::str(boost::format("[%s:%d] %s")%(__PRETTY_FUNCTION__)%(__LINE__)%(s)),errorcode)

/// adds the function name and line number to an openrave exception
#define OPENRAVE_EXCEPTION_FORMAT(s, args, errorcode) OpenRAVE::OpenRAVEException(boost::str(boost::format("[%s:%d] ")%(__PRETTY_FUNCTION__)%(__LINE__)) + boost::str(boost::format(s)%args),errorcode)

#define OPENRAVE_ASSERT_FORMAT(testexpr, s, args, errorcode) { if( !(testexpr) ) { throw OpenRAVE::OpenRAVEException(boost::str(boost::format("[%s:%d] (%s) failed ")%(__PRETTY_FUNCTION__)%(__LINE__)%(# testexpr)) + boost::str(boost::format(s)%args),errorcode); } }

#define OPENRAVE_ASSERT_FORMAT0(testexpr, s, errorcode) { if( !(testexpr) ) { throw OpenRAVE::OpenRAVEException(boost::str(boost::format("[%s:%d] (%s) failed %s")%(__PRETTY_FUNCTION__)%(__LINE__)%(# testexpr)%(s)),errorcode); } }

// note that expr1 and expr2 will be evaluated twice if not equal
#define OPENRAVE_ASSERT_OP_FORMAT(expr1,op,expr2,s, args, errorcode) { if( !((expr1) op (expr2)) ) { throw OpenRAVE::OpenRAVEException(boost::str(boost::format("[%s:%d] %s %s %s, (eval %s %s %s) ")%(__PRETTY_FUNCTION__)%(__LINE__)%(# expr1)%(# op)%(# expr2)%(expr1)%(# op)%(expr2)) + boost::str(boost::format(s)%args),errorcode); } }

#define OPENRAVE_ASSERT_OP_FORMAT0(expr1,op,expr2,s, errorcode) { if( !((expr1) op (expr2)) ) { throw OpenRAVE::OpenRAVEException(boost::str(boost::format("[%s:%d] %s %s %s, (eval %s %s %s) %s")%(__PRETTY_FUNCTION__)%(__LINE__)%(# expr1)%(# op)%(# expr2)%(expr1)%(# op)%(expr2)%(s)),errorcode); } }

#define OPENRAVE_ASSERT_OP(expr1,op,expr2) { if( !((expr1) op (expr2)) ) { throw OpenRAVE::OpenRAVEException(boost::str(boost::format("[%s:%d] %s %s %s, (eval %s %s %s) ")%(__PRETTY_FUNCTION__)%(__LINE__)%(# expr1)%(# op)%(# expr2)%(expr1)%(# op)%(expr2)),OpenRAVE::ORE_Assert); } }

#define OPENRAVE_DUMMY_IMPLEMENTATION { throw OPENRAVE_EXCEPTION_FORMAT0("not implemented",OpenRAVE::ORE_NotImplemented); }

#endif // OPENRAVE_EXCEPTION_H
