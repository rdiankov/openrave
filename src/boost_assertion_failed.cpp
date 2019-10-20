// -*- coding: utf-8 -*-
// Copyright (C) 2018 Rosen Diankov
#if !defined(OPENRAVE_DISABLE_ASSERT_HANDLER) && (defined(BOOST_ENABLE_ASSERT_HANDLER))

#define BOOST_SYSTEM_NO_DEPRECATED
#include <openrave/openrave.h>
#include <boost/format.hpp>

// Derived from https://gcc.gnu.org/wiki/Visibility
#if !(defined _WIN32 || defined __CYGWIN__)
  #if __GNUC__ >= 4
    #define HIDDEN  __attribute__ ((visibility ("hidden")))
  #else
    #define HIDDEN
  #endif
#endif

/// Modifications controlling %boost library behavior.
namespace boost
{
HIDDEN void assertion_failed(char const * expr, char const * function, char const * file, long line)
{
    throw OpenRAVE::openrave_exception(boost::str(boost::format("[%s:%d] -> %s, expr: %s")%file%line%function%expr),OpenRAVE::ORE_Assert);
}

#if BOOST_VERSION>104600
HIDDEN void assertion_failed_msg(char const * expr, char const * msg, char const * function, char const * file, long line)
{
    throw OpenRAVE::openrave_exception(boost::str(boost::format("[%s:%d] -> %s, expr: %s, msg: %s")%file%line%function%expr%msg),OpenRAVE::ORE_Assert);
}
#endif

}
#endif
