// -*- coding: utf-8 -*-
// Copyright (C) 2006-2020 Guangning Tan, Kei Usui, Rosen Diankov <rosen.diankov@gmail.com>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef PLUGINS_POSTUREDESCRIBER_POSTURESUPPORTTYPES_H
#define PLUGINS_POSTUREDESCRIBER_POSTURESUPPORTTYPES_H
#include <cstdint> // uint16_t
#include <type_traits> // underlying_type

namespace OpenRAVE {

// https://stackoverflow.com/questions/12059774/c11-standard-conformant-bitmasks-using-enum-class
enum class NeighbouringTwoJointsRelation : uint16_t {
    NTJR_Unknown                 = 0x0,
    NTJR_Parallel                = 0x1,
    NTJR_Perpendicular           = 0x2,
    NTJR_Intersect               = 0x4,
    NTJR_Overlap                 = NTJR_Intersect | NTJR_Parallel,      // 0x5
    NTJR_Intersect_Perpendicular = NTJR_Intersect | NTJR_Perpendicular, // 0x6
};

enum class RobotPostureSupportType : uint16_t {
    RPST_NoSupport  = 0x0, ///< unsupported
    RPST_6R_General = 0x1, ///< general 6R robots with the last joint axes intersecting at a point
    RPST_4R_Type_A  = 0x2, ///< a special type of 4R robot the last three parallel joint axes perpendicular to the first joint axis
};

/// can do bit operations with enum class
template <typename T>
inline constexpr T operator&(T x, T y)
{
    using UT = typename std::underlying_type<T>::type;
    return static_cast<T>(static_cast<UT>(x) & static_cast<UT>(y));
}

template <typename T>
inline constexpr T operator|(T x, T y)
{
    using UT = typename std::underlying_type<T>::type;
    return static_cast<T>(static_cast<UT>(x) | static_cast<UT>(y));
}

template <typename T>
inline T operator&=(T& x, T y)
{
    return x = x & y;
}

template <typename T>
inline T operator|=(T& x, T y)
{
    return x = x | y;
}

} // namespace OpenRAVE

#endif // PLUGINS_POSTUREDESCRIBER_SUPPORTTYPES_H
