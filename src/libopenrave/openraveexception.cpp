// -*- coding: utf-8 -*-
// Copyright (C) 2006-2023
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
#include <openrave/openraveexception.h>

namespace OpenRAVE {

OpenRAVEException::OpenRAVEException() : std::exception(), _s("unknown exception"), _error(ORE_Failed)
{
}

OpenRAVEException::OpenRAVEException(const std::string& s, OpenRAVEErrorCode error) : std::exception()
{
    _error = error;
    _s = "openrave (";
    _s += RaveGetErrorCodeString(_error);
    _s += "): ";
    _s += s;
}

char const* OpenRAVEException::what() const throw() {
    return _s.c_str();
}

const std::string& OpenRAVEException::message() const {
    return _s;
}

OpenRAVEErrorCode OpenRAVEException::GetCode() const {
    return _error;
}

const char* RaveGetErrorCodeString(OpenRAVEErrorCode error)
{
    switch(error) {
    case ORE_Failed: return "Failed";
    case ORE_InvalidArguments: return "InvalidArguments";
    case ORE_EnvironmentNotLocked: return "EnvironmentNotLocked";
    case ORE_CommandNotSupported: return "CommandNotSupported";
    case ORE_Assert: return "Assert";
    case ORE_InvalidPlugin: return "InvalidPlugin";
    case ORE_InvalidInterfaceHash: return "InvalidInterfaceHash";
    case ORE_NotImplemented: return "NotImplemented";
    case ORE_InconsistentConstraints: return "InconsistentConstraints";
    case ORE_NotInitialized: return "NotInitialized";
    case ORE_InvalidState: return "InvalidState";
    case ORE_Timeout: return "Timeout";
    case ORE_InvalidURI: return "InvalidURI";
    case ORE_BodyNameConflict: return "BodyNameConflict";
    case ORE_SensorNameConflict: return "SensorNameConflict";
    case ORE_BodyIdConflict: return "BodyIdConflict";

    case ORE_LengthUnitInvalid: return "LengthUnitInvalid";
    case ORE_MassUnitInvalid: return "MassUnitInvalid";
    case ORE_TimeDurationUnitInvalid: return "TimeDurationUnitInvalid";
    case ORE_AngleUnitInvalid: return "AngleUnitInvalid";
    case ORE_TimeStampUnitInvalid: return "TimeStampUnitInvalid";
    case ORE_EnvironmentBodyIndexConflict: return "EnvironmentBodyIndexConflict";

    case ORE_EnvironmentFormatUnrecognized: return "EnvironmentFormatUnrecognized";
    
    case ORE_CurlTimeout: return "CurlTimeout";
    case ORE_CurlInvalidHandle: return "CurlInvalidHandle";
    case ORE_CurlInvalidResponse: return "CurlInvalidResponse";

    }
    // should throw an exception?
    return "(unknown)";
}

} // end namespace OpenRAVE
