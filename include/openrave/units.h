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

/// \brief time duration unit
enum TimeDurationUnit : int8_t
{
    TDU_Second = 0,
    TDU_Millisecond = -3,
    TDU_Microsecond = -6,
    TDU_Nanosecond = -9,
    TDU_Picosecond = -12,
};

OPENRAVE_API const char* GetTimeDurationUnitString(TimeDurationUnit unit);

OPENRAVE_API TimeDurationUnit GetTimeDurationUnitFromString(const char* pTimeDurationUnit, TimeDurationUnit defaultTimeDurationUnit);
OPENRAVE_API TimeDurationUnit GetTimeDurationUnitFromString(const std::string& pTimeDurationUnit, TimeDurationUnit defaultTimeDurationUnit);

/// \brief angle unit
enum AngleUnit : int8_t
{
    AU_Radian = 0,
    AU_Degree = 1,
    AU_Centidegree = -2,
};

OPENRAVE_API const char* GetAngleUnitString(AngleUnit unit);

OPENRAVE_API AngleUnit GetAngleUnitFromString(const char* pAngleUnit, AngleUnit defaultAngleUnit);
OPENRAVE_API AngleUnit GetAngleUnitFromString(const std::string& pAngleUnit, AngleUnit defaultAngleUnit);

/// \brief time stamp unit
enum TimeStampUnit : int8_t
{
    TSU_SecondsFromLinuxEpoch = 0,
    TSU_MillisecondsFromLinuxEpoch = -3,
    TSU_MicrosecondsFromLinuxEpoch = -6,
    TSU_ISO8601 = 0x10,
};

OPENRAVE_API const char* GetTimeStampUnitString(TimeStampUnit unit);

OPENRAVE_API TimeStampUnit GetTimeStampUnitFromString(const char* pTimeStampUnit, TimeStampUnit defaultTimeStampUnit);
OPENRAVE_API TimeStampUnit GetTimeStampUnitFromString(const std::string& pTimeStampUnit, TimeStampUnit defaultTimeStampUnit);

/// \brief holds a struct of unit of the fundamental types so users know which they are working with.
class __attribute__((aligned(8))) OPENRAVE_API UnitInfo
{
public:
    inline bool operator==(const UnitInfo& rhs) const {
        return *reinterpret_cast<const uint64_t*>(this) == *reinterpret_cast<const uint64_t*>(&rhs);
    }
    inline bool operator!=(const UnitInfo& rhs) const {
        return *reinterpret_cast<const uint64_t*>(this) != *reinterpret_cast<const uint64_t*>(&rhs);
    }

    LengthUnit lengthUnit = LU_Millimeter; ///< standard in industrial applications
    MassUnit massUnit = MU_Kilogram; ///< SI unit
    TimeDurationUnit timeDurationUnit = TDU_Second; ///< SI unit
    AngleUnit angleUnit = AU_Degree; ///< easier to set for users
    TimeStampUnit timeStampUnit = TSU_MicrosecondsFromLinuxEpoch; ///< good default for integer storage
    uint8_t reserved5 = 0;
    uint8_t reserved6 = 0;
    uint8_t reserved7 = 0;
};
BOOST_STATIC_ASSERT(sizeof(UnitInfo) == sizeof(uint64_t));

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
constexpr inline T GetLengthUnitStandardValue(const LengthUnit unit)
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
    return T{};
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
    throw OPENRAVE_EXCEPTION_FORMAT("Unsupported mass unit '%s'", pUnit, ORE_MassUnitInvalid);
}

template <typename T>
inline T GetMassUnitStandardValue(const std::string& s) {
    return GetMassUnitStandardValue<T>(s.c_str());
}

// \brief how many units in a gram
template <typename T>
constexpr inline T GetMassUnitStandardValue(const MassUnit unit)
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

    throw OPENRAVE_EXCEPTION_FORMAT("Unsupported mass unit '%s'", GetMassUnitString(unit), ORE_MassUnitInvalid);
    return T{};
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
// Time Duration
//

// \brief how many units in a second
template <typename T>
inline T GetTimeDurationUnitStandardValue(const char* pUnit)
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
    throw OPENRAVE_EXCEPTION_FORMAT("Unsupported time duration unit '%s'", pUnit, ORE_TimeDurationUnitInvalid);
}

template <typename T>
inline T GetTimeDurationUnitStandardValue(const std::string& s) {
    return GetTimeDurationUnitStandardValue<T>(s.c_str());
}

// \brief how many units in a second
template <typename T>
constexpr inline T GetTimeDurationUnitStandardValue(const TimeDurationUnit unit)
{
    if( unit == OpenRAVE::TDU_Second ) {
        return T(1.0);
    }
    if( unit == OpenRAVE::TDU_Millisecond ) {
        return T(1e3);
    }
    if( unit == OpenRAVE::TDU_Microsecond ) {
        return T(1e6);
    }
    if( unit == OpenRAVE::TDU_Nanosecond ) {
        return T(1e9);
    }
    if( unit == OpenRAVE::TDU_Picosecond ) {
        return T(1e12);
    }

    throw OPENRAVE_EXCEPTION_FORMAT("Unsupported time duration unit '%s'", GetTimeDurationUnitString(unit), ORE_TimeDurationUnitInvalid);
    return T{};
}

template <typename T>
inline T GetTimeDurationUnitConversionScale(const std::string &sourceTimeDurationUnit, const std::string &targetTimeDurationUnit)
{
    if( sourceTimeDurationUnit == targetTimeDurationUnit ) {
        return T(1.0);
    }

    return GetTimeDurationUnitStandardValue<T>(targetTimeDurationUnit) / GetTimeDurationUnitStandardValue<T>(sourceTimeDurationUnit);
}

template <typename T>
inline T GetTimeDurationUnitConversionScale(const TimeDurationUnit sourceTimeDurationUnit, const TimeDurationUnit targetTimeDurationUnit)
{
    if( sourceTimeDurationUnit == targetTimeDurationUnit ) {
        return T(1.0);
    }

    return GetTimeDurationUnitStandardValue<T>(targetTimeDurationUnit) / GetTimeDurationUnitStandardValue<T>(sourceTimeDurationUnit);
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
    if (strcmp(pUnit, "cdeg") == 0 ) {
        return T(5729.577951308232);
    }
    throw OPENRAVE_EXCEPTION_FORMAT("Unsupported angle unit '%s'", pUnit, ORE_AngleUnitInvalid);
}

template <typename T>
inline T GetAngleUnitStandardValue(const std::string& s) {
    return GetAngleUnitStandardValue<T>(s.c_str());
}

// \brief how many units in a radian
template <typename T>
constexpr inline T GetAngleUnitStandardValue(const AngleUnit unit)
{
    if( unit == OpenRAVE::AU_Radian ) {
        return T(1.0);
    }
    if( unit == OpenRAVE::AU_Degree ) {
        return T(57.29577951308232);
    }
    if( unit == OpenRAVE::AU_Centidegree ) {
        return T(5729.577951308232);
    }

    throw OPENRAVE_EXCEPTION_FORMAT("Unsupported angle unit '%s'", GetAngleUnitString(unit), ORE_AngleUnitInvalid);
    return T{};
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
