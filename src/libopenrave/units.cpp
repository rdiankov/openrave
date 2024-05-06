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
#include <openrave/units.h>
#include <cstring>

namespace OpenRAVE {

const char* GetLengthUnitString(LengthUnit unit)
{
    switch(unit) {
    case LU_Meter: return "m";
    case LU_Millimeter: return "mm";
    case LU_Micrometer: return "um";
    case LU_Nanometer: return "nm";
    case LU_Centimeter: return "cm";
    case LU_Decimeter: return "dm";
    case LU_Inch: return "in";
    case LU_Foot: return "ft";
    case LU_DeciMillimeter: return "dmm";
    }
    return "(unknown)";
}

LengthUnit GetLengthUnitFromString(const char* pLengthUnit, LengthUnit defaultLengthUnit)
{
    if( pLengthUnit[0] == 0 ) {
        return defaultLengthUnit;
    }
    if( strcmp(pLengthUnit, "m") == 0 ) {
        return LU_Meter;
    }
    if( strcmp(pLengthUnit, "mm") == 0 ) {
        return LU_Millimeter;
    }
    if( strcmp(pLengthUnit, "um") == 0 ) {
        return LU_Micrometer;
    }
    if( strcmp(pLengthUnit, "nm") == 0 ) {
        return LU_Nanometer;
    }
    if( strcmp(pLengthUnit, "meter") == 0 ) { // rare so do later
        return LU_Meter;
    }
    if( strcmp(pLengthUnit, "cm") == 0 ) {
        return LU_Centimeter;
    }
    if( strcmp(pLengthUnit, "dm") == 0 ) {
        return LU_Decimeter;
    }
    if( strcmp(pLengthUnit, "in") == 0 || strcmp(pLengthUnit, "inch") == 0 ) {
        return LU_Inch;
    }
    if( strcmp(pLengthUnit, "ft") == 0 || strcmp(pLengthUnit, "foot") == 0 || strcmp(pLengthUnit, "feet") == 0 ) {
        return LU_Foot;
    }
    if( strcmp(pLengthUnit, "dmm") == 0 ) {
        return LU_DeciMillimeter;
    }
    if( strcmp(pLengthUnit, "millimeter") == 0 ) {
        return LU_Millimeter;
    }
    if( strcmp(pLengthUnit, "micrometer") == 0 ) {
        return LU_Micrometer;
    }
    throw OpenRAVEException(str(boost::format("Do not support LengthUnit '%s'")%pLengthUnit), ORE_LengthUnitInvalid);
}

LengthUnit GetLengthUnitFromString(const std::string& pLengthUnit, LengthUnit defaultLengthUnit)
{
    return GetLengthUnitFromString(pLengthUnit.c_str(), defaultLengthUnit);
}

const char* GetMassUnitString(MassUnit unit)
{
    switch(unit) {
    case MU_Gram: return "g";
    case MU_Milligram: return "mg";
    case MU_Kilogram: return "kg";
    case MU_Pound: return "lb";
    }
    return "(unknown)";
}

MassUnit GetMassUnitFromString(const char* pMassUnit, MassUnit defaultMassUnit)
{
    if( pMassUnit[0] == 0 ) {
        return defaultMassUnit;
    }
    if( strcmp(pMassUnit, "g") == 0 ) {
        return MU_Gram;
    }
    if( strcmp(pMassUnit, "mg") == 0 ) {
        return MU_Milligram;
    }
    if( strcmp(pMassUnit, "kg") == 0 ) {
        return MU_Kilogram;
    }
    if( strcmp(pMassUnit, "lb") == 0 ) {
        return MU_Pound;
    }
    throw OpenRAVEException(str(boost::format("Do not support MassUnit '%s'")%pMassUnit), ORE_MassUnitInvalid);
}

MassUnit GetMassUnitFromString(const std::string& pMassUnit, MassUnit defaultMassUnit)
{
    return GetMassUnitFromString(pMassUnit.c_str(), defaultMassUnit);
}

const char* GetTimeDurationUnitString(TimeDurationUnit unit)
{
    switch(unit) {
    case TDU_Second: return "s";
    case TDU_Millisecond: return "ms";
    case TDU_Microsecond: return "us";
    case TDU_Nanosecond: return "ns";
    case TDU_Picosecond: return "ps";
    }
    return "(unknown)";
}

TimeDurationUnit GetTimeDurationUnitFromString(const char* pTimeDurationUnit, TimeDurationUnit defaultTimeDurationUnit)
{
    if( pTimeDurationUnit[0] == 0 ) {
        return defaultTimeDurationUnit;
    }
    if( strcmp(pTimeDurationUnit, "s") == 0 ) {
        return TDU_Second;
    }
    if( strcmp(pTimeDurationUnit, "ms") == 0 ) {
        return TDU_Millisecond;
    }
    if( strcmp(pTimeDurationUnit, "us") == 0 ) {
        return TDU_Microsecond;
    }
    if( strcmp(pTimeDurationUnit, "ns") == 0 ) {
        return TDU_Nanosecond;
    }
    if( strcmp(pTimeDurationUnit, "ps") == 0 ) {
        return TDU_Picosecond;
    }
    throw OpenRAVEException(str(boost::format("Do not support TimeDurationUnit '%s'")%pTimeDurationUnit), ORE_TimeDurationUnitInvalid);
}

TimeDurationUnit GetTimeDurationUnitFromString(const std::string& pTimeDurationUnit, TimeDurationUnit defaultTimeDurationUnit)
{
    return GetTimeDurationUnitFromString(pTimeDurationUnit.c_str(), defaultTimeDurationUnit);
}

const char* GetAngleUnitString(AngleUnit unit)
{
    switch(unit) {
    case AU_Radian: return "rad";
    case AU_Degree: return "deg";
    case AU_Centidegree: return "cdeg";
    }
    return "(unknown)";
}

AngleUnit GetAngleUnitFromString(const char* pAngleUnit, AngleUnit defaultAngleUnit)
{
    if( pAngleUnit[0] == 0 ) {
        return defaultAngleUnit;
    }
    if( strcmp(pAngleUnit, "rad") == 0 ) {
        return AU_Radian;
    }
    if( strcmp(pAngleUnit, "deg") == 0 ) {
        return AU_Degree;
    }
    if( strcmp(pAngleUnit, "cdeg") == 0 ) {
        return AU_Centidegree;
    }
    throw OpenRAVEException(str(boost::format("Do not support AngleUnit '%s'")%pAngleUnit), ORE_AngleUnitInvalid);
}

AngleUnit GetAngleUnitFromString(const std::string& pAngleUnit, AngleUnit defaultAngleUnit)
{
    return GetAngleUnitFromString(pAngleUnit.c_str(), defaultAngleUnit);
}

const char* GetTimeStampUnitString(TimeStampUnit unit)
{
    switch(unit) {
    case TSU_SecondsFromLinuxEpoch: return "s";
    case TSU_MillisecondsFromLinuxEpoch: return "ms";
    case TSU_MicrosecondsFromLinuxEpoch: return "us";
    case TSU_ISO8601: return "iso8601";
    }
    return "(unknown)";
}

TimeStampUnit GetTimeStampUnitFromString(const char* pTimeStampUnit, TimeStampUnit defaultTimeStampUnit)
{
    if( pTimeStampUnit[0] == 0 ) {
        return defaultTimeStampUnit;
    }
    if( strcmp(pTimeStampUnit, "s") == 0 ) {
        return TSU_SecondsFromLinuxEpoch;
    }
    if( strcmp(pTimeStampUnit, "ms") == 0 ) {
        return TSU_MillisecondsFromLinuxEpoch;
    }
    if( strcmp(pTimeStampUnit, "us") == 0 ) {
        return TSU_MicrosecondsFromLinuxEpoch;
    }
    if( strcmp(pTimeStampUnit, "iso8601") == 0 ) {
        return TSU_ISO8601;
    }
    throw OpenRAVEException(str(boost::format("Do not support TimeStampUnit '%s'")%pTimeStampUnit), ORE_TimeStampUnitInvalid);
}

TimeStampUnit GetTimeStampUnitFromString(const std::string& pTimeStampUnit, TimeStampUnit defaultTimeStampUnit)
{
    return GetTimeStampUnitFromString(pTimeStampUnit.c_str(), defaultTimeStampUnit);
}

} // end namespace OpenRAVE
