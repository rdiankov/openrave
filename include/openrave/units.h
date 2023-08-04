// -*- coding: utf-8 -*-
// Copyright (C) 2023
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
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
#ifndef MUJIN_OPENRAVE_UNITS_H
#define MUJIN_OPENRAVE_UNITS_H

#include <openrave/config.h>
#include <openrave/openraveexception.h>

#include <stdint.h>
#include <cstring>
#include <sstream>

#include <boost/format.hpp>

namespace OpenRAVE {

/// \brief length unit
enum LengthUnit : int8_t
{
    LU_Meter = 0,
    LU_Decimeter = -1,
    LU_Centimeter = -2,
    LU_Millimeter = -3,
    LU_DeciMillimeter = -4,
    LU_Micrometer = -6,
    LU_Nanometer = -9,
    LU_Inch = 0x10,
    LU_Foot = 0x11,
};

OPENRAVE_API const char* GetLengthUnitString(LengthUnit unit);

OPENRAVE_API LengthUnit GetLengthUnitFromString(const char* pLengthUnit, LengthUnit defaultLengthUnit);
OPENRAVE_API LengthUnit GetLengthUnitFromString(const std::string& pLengthUnit, LengthUnit defaultLengthUnit);

/// \brief mass unit
enum MassUnit : int8_t
{
    MU_Gram = 0,
    MU_Kilogram = 3,
    MU_Milligram = -3,
    MU_Pound = 0x10,
};

OPENRAVE_API const char* GetMassUnitString(MassUnit unit);

OPENRAVE_API MassUnit GetMassUnitFromString(const char* pMassUnit, MassUnit defaultMassUnit);
OPENRAVE_API MassUnit GetMassUnitFromString(const std::string& pMassUnit, MassUnit defaultMassUnit);

/// \brief time unit
enum TimeUnit : int8_t
{
    TU_Second = 0,
    TU_Millisecond = -3,
    TU_Microsecond = -6,
    TU_Nanosecond = -9,
    TU_Picosecond = -12,
};

OPENRAVE_API const char* GetTimeUnitString(TimeUnit unit);

OPENRAVE_API TimeUnit GetTimeUnitFromString(const char* pTimeUnit, TimeUnit defaultTimeUnit);
OPENRAVE_API TimeUnit GetTimeUnitFromString(const std::string& pTimeUnit, TimeUnit defaultTimeUnit);

/// \brief angle unit
enum AngleUnit : int8_t
{
    AU_Radian = 0,
    AU_Degree = 1
};

OPENRAVE_API const char* GetAngleUnitString(AngleUnit unit);

OPENRAVE_API AngleUnit GetAngleUnitFromString(const char* pAngleUnit, AngleUnit defaultAngleUnit);
OPENRAVE_API AngleUnit GetAngleUnitFromString(const std::string& pAngleUnit, AngleUnit defaultAngleUnit);

/// \brief holds a struct of unit of the fundamental types so users know which they are working with.
class OPENRAVE_API UnitInfo
{
public:
    inline bool operator==(const UnitInfo& rhs) const {
        return lengthUnit == rhs.lengthUnit && massUnit == rhs.massUnit && timeUnit == rhs.timeUnit && angleUnit == rhs.angleUnit;
    }
    inline bool operator!=(const UnitInfo& rhs) const {
        return lengthUnit != rhs.lengthUnit || massUnit != rhs.massUnit || timeUnit != rhs.timeUnit || angleUnit != rhs.angleUnit;
    }

    LengthUnit lengthUnit = LU_Millimeter; ///< standard in industrial applications
    MassUnit massUnit = MU_Kilogram; ///< SI unit
    TimeUnit timeUnit = TU_Second; ///< SI unit
    AngleUnit angleUnit = AU_Degree; ///< easier to set for users
};
BOOST_STATIC_ASSERT(sizeof(UnitInfo)==4);

//
// Length
//

// \brief how many units in a meter
template <typename T>
inline T GetLengthUnitStandardValue(const char* pUnit)
{
    // ordered from most likely
    if (strcmp(pUnit, "m") == 0 ) {
        return T(1.0);
    }
    if (strcmp(pUnit, "mm") == 0 || strcmp(pUnit, "millimeter") == 0 ) {
        return T(1000.0);
    }
    if (strcmp(pUnit, "um") == 0 || strcmp(pUnit, "micrometer") == 0 ) {
        return T(1e6);
    }
    if (strcmp(pUnit, "nm") == 0 ) {
        return T(1e9);
    }
    if (strcmp(pUnit, "cm") == 0 ) {
        return T(100.0);
    }
    if (strcmp(pUnit, "dm") == 0 ) {
        return T(10.0);
    }
    if (strcmp(pUnit, "in") == 0 || strcmp(pUnit, "inch") == 0 ) {
        return T(39.370078740157481); // 25.4 mm/in
    }
    if (strcmp(pUnit, "ft") == 0 || strcmp(pUnit, "foot") == 0 || strcmp(pUnit, "feet") == 0 ) {
        return T(3.2808398950131235); // 304.8 mm/ft
    }
    if (strcmp(pUnit, "meter") == 0 ) {
        return T(1.0);
    }
    if (strcmp(pUnit, "dmm") == 0 ) {
        return T(1e4);
    }

    throw OPENRAVE_EXCEPTION_FORMAT("Unsupported length unit '%s'", pUnit, ORE_LengthUnitInvalid);
}

template <typename T>
inline T GetLengthUnitStandardValue(const std::string& s) {
    return GetLengthUnitStandardValue<T>(s.c_str());
}

// \brief how many units in a meter
template <typename T>
inline T GetLengthUnitStandardValue(const LengthUnit unit)
{
    if( unit == OpenRAVE::LU_Meter ) {
        return T(1.0);
    }
    if( unit == OpenRAVE::LU_Millimeter ) {
        return T(1000.0);
    }
    if( unit == OpenRAVE::LU_Micrometer ) {
        return T(1e6);
    }
    if( unit == OpenRAVE::LU_Nanometer ) {
        return T(1e9);
    }
    if( unit == OpenRAVE::LU_Centimeter ) {
        return T(100.0);
    }
    if( unit == OpenRAVE::LU_Decimeter ) {
        return T(10.0);
    }
    if( unit == OpenRAVE::LU_Inch ) {
        return T(39.370078740157481); // 25.4 mm/in
    }
    if( unit == OpenRAVE::LU_Foot ) {
        return T(3.2808398950131235); // 304.8 mm/ft
    }
    if( unit == OpenRAVE::LU_DeciMillimeter ) {
        return T(1e4);
    }

    throw OPENRAVE_EXCEPTION_FORMAT("Unsupported length unit '%s'", GetLengthUnitString(unit), ORE_LengthUnitInvalid);
}

template <typename T>
inline T GetLengthUnitConversionScale(const std::string &sourceLengthUnit, const std::string &targetLengthUnit)
{
    if( sourceLengthUnit == targetLengthUnit ) {
        return T(1.0);
    }
    return GetLengthUnitStandardValue<T>(targetLengthUnit) / GetLengthUnitStandardValue<T>(sourceLengthUnit);
}

template <typename T>
inline T GetLengthUnitConversionScale(const LengthUnit sourceLengthUnit, const LengthUnit targetLengthUnit)
{
    if( sourceLengthUnit == targetLengthUnit ) {
        return T(1.0);
    }

    return GetLengthUnitStandardValue<T>(targetLengthUnit) / GetLengthUnitStandardValue<T>(sourceLengthUnit);
}

//
// Mass
//

// \brief how many units in a gram
template <typename T>
inline T GetMassUnitStandardValue(const char* pUnit)
{
    // ordered from most likely
    if (strcmp(pUnit, "g") == 0 ) {
        return T(1.0);
    }
    if (strcmp(pUnit, "mg") == 0 ) {
        return T(1000.0);
    }
    if (strcmp(pUnit, "kg") == 0 ) {
        return T(0.001);
    }
    if (strcmp(pUnit, "lb") == 0 ) {
        return T(0.002204622621848776);
    }
    throw OPENRAVE_EXCEPTION_FORMAT("Unsupported mass unit '%s'", pUnit, ORE_LengthUnitInvalid);
}

template <typename T>
inline T GetMassUnitStandardValue(const std::string& s) {
    return GetMassUnitStandardValue<T>(s.c_str());
}

// \brief how many units in a gram
template <typename T>
inline T GetMassUnitStandardValue(const MassUnit unit)
{
    if( unit == OpenRAVE::MU_Gram ) {
        return T(1.0);
    }
    if( unit == OpenRAVE::MU_Milligram ) {
        return T(1000.0);
    }
    if( unit == OpenRAVE::MU_Kilogram ) {
        return T(0.001);
    }
    if( unit == OpenRAVE::MU_Pound ) {
        return T(0.002204622621848776);
    }

    throw OPENRAVE_EXCEPTION_FORMAT("Unsupported mass unit '%s'", GetMassUnitString(unit), ORE_LengthUnitInvalid);
}

template <typename T>
inline T GetMassUnitConversionScale(const std::string &sourceMassUnit, const std::string &targetMassUnit)
{
    if( sourceMassUnit == targetMassUnit ) {
        return T(1.0);
    }
    return GetMassUnitStandardValue<T>(targetMassUnit) / GetMassUnitStandardValue<T>(sourceMassUnit);
}

template <typename T>
inline T GetMassUnitConversionScale(const MassUnit sourceMassUnit, const MassUnit targetMassUnit)
{
    if( sourceMassUnit == targetMassUnit ) {
        return T(1.0);
    }

    return GetMassUnitStandardValue<T>(targetMassUnit) / GetMassUnitStandardValue<T>(sourceMassUnit);
}

//
// Time
//

// \brief how many units in a second
template <typename T>
inline T GetTimeUnitStandardValue(const char* pUnit)
{
    // ordered from most likely
    if (strcmp(pUnit, "s") == 0 ) {
        return T(1.0);
    }
    if (strcmp(pUnit, "ms") == 0 ) {
        return T(1e3);
    }
    if (strcmp(pUnit, "us") == 0 ) {
        return T(1e6);
    }
    if (strcmp(pUnit, "ns") == 0 ) {
        return T(1e9);
    }
    if (strcmp(pUnit, "ps") == 0 ) {
        return T(1e12);
    }
    throw OPENRAVE_EXCEPTION_FORMAT("Unsupported time unit '%s'", pUnit, ORE_LengthUnitInvalid);
}

template <typename T>
inline T GetTimeUnitStandardValue(const std::string& s) {
    return GetTimeUnitStandardValue<T>(s.c_str());
}

// \brief how many units in a second
template <typename T>
inline T GetTimeUnitStandardValue(const TimeUnit unit)
{
    if( unit == OpenRAVE::TU_Second ) {
        return T(1.0);
    }
    if( unit == OpenRAVE::TU_Millisecond ) {
        return T(1e3);
    }
    if( unit == OpenRAVE::TU_Microsecond ) {
        return T(1e6);
    }
    if( unit == OpenRAVE::TU_Nanosecond ) {
        return T(1e9);
    }
    if( unit == OpenRAVE::TU_Picosecond ) {
        return T(1e12);
    }

    throw OPENRAVE_EXCEPTION_FORMAT("Unsupported time unit '%s'", GetTimeUnitString(unit), ORE_LengthUnitInvalid);
}

template <typename T>
inline T GetTimeUnitConversionScale(const std::string &sourceTimeUnit, const std::string &targetTimeUnit)
{
    if( sourceTimeUnit == targetTimeUnit ) {
        return T(1.0);
    }
    return GetTimeUnitStandardValue<T>(targetTimeUnit) / GetTimeUnitStandardValue<T>(sourceTimeUnit);
}

template <typename T>
inline T GetTimeUnitConversionScale(const TimeUnit sourceTimeUnit, const TimeUnit targetTimeUnit)
{
    if( sourceTimeUnit == targetTimeUnit ) {
        return T(1.0);
    }

    return GetTimeUnitStandardValue<T>(targetTimeUnit) / GetTimeUnitStandardValue<T>(sourceTimeUnit);
}


//
// Angle
//

// \brief how many units in a radian
template <typename T>
inline T GetAngleUnitStandardValue(const char* pUnit)
{
    // ordered from most likely
    if (strcmp(pUnit, "rad") == 0 ) {
        return T(1.0);
    }
    if (strcmp(pUnit, "deg") == 0 ) {
        return T(57.29577951308232);
    }
    throw OPENRAVE_EXCEPTION_FORMAT("Unsupported angle unit '%s'", pUnit, ORE_LengthUnitInvalid);
}

template <typename T>
inline T GetAngleUnitStandardValue(const std::string& s) {
    return GetAngleUnitStandardValue<T>(s.c_str());
}

// \brief how many units in a radian
template <typename T>
inline T GetAngleUnitStandardValue(const AngleUnit unit)
{
    if( unit == OpenRAVE::AU_Radian ) {
        return T(1.0);
    }
    if( unit == OpenRAVE::AU_Degree ) {
        return T(57.29577951308232);
    }

    throw OPENRAVE_EXCEPTION_FORMAT("Unsupported angle unit '%s'", GetAngleUnitString(unit), ORE_LengthUnitInvalid);
}

template <typename T>
inline T GetAngleUnitConversionScale(const std::string &sourceAngleUnit, const std::string &targetAngleUnit)
{
    if( sourceAngleUnit == targetAngleUnit ) {
        return T(1.0);
    }
    return GetAngleUnitStandardValue<T>(targetAngleUnit) / GetAngleUnitStandardValue<T>(sourceAngleUnit);
}

template <typename T>
inline T GetAngleUnitConversionScale(const AngleUnit sourceAngleUnit, const AngleUnit targetAngleUnit)
{
    if( sourceAngleUnit == targetAngleUnit ) {
        return T(1.0);
    }

    return GetAngleUnitStandardValue<T>(targetAngleUnit) / GetAngleUnitStandardValue<T>(sourceAngleUnit);
}


} // end namespace OpenRAVE

#endif
