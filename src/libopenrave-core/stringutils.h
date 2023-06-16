// -*- coding: utf-8 -*-
// Copyright (C) 2023 Tan Li Boon <liboon.tan@mujin.co.jp>
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
#ifndef RAVE_STRINGUTILS
#define RAVE_STRINGUTILS

#include <algorithm>
#include <string>

#if  __cplusplus >= 201703L
#define OPENRAVE_NODISCARD [[nodiscard]]
#else
#define OPENRAVE_NODISCARD __attribute__((warn_unused_result))
#endif

namespace OpenRAVE {

// Returns true if a string begins with a matching prefix.
OPENRAVE_NODISCARD inline bool StringStartsWith(const std::string& input, const std::string& prefix, bool ignoreCase = true) {
    if (input.length() < prefix.length()) {
        return false;
    }
    if (ignoreCase) {
        return std::equal(input.begin(), input.begin() + prefix.length(), prefix.begin(), prefix.end(), [](char l, char r) -> bool {
            return std::tolower(l) == std::tolower(r);
        });
    } else {
        return std::equal(input.begin(), input.begin() + prefix.length(), prefix.begin(), prefix.end());
    }
}

// Returns true if a string ends with a matching suffix.
OPENRAVE_NODISCARD inline bool StringEndsWith(const std::string& input, const std::string& suffix, bool ignoreCase = true) {
    if (input.length() < suffix.length()) {
        return false;
    }
    if (ignoreCase) {
        return std::equal(input.end() - suffix.length(), input.end(), suffix.begin(), suffix.end(), [](char l, char r) -> bool {
            return std::tolower(l) == std::tolower(r);
        });
    } else {
        return std::equal(input.end() - suffix.length(), input.end(), suffix.begin(), suffix.end());
    }
}

// Attempt to remove a matching prefix from a string in-place. Nothing is done if the prefix does not match.
inline bool RemovePrefix(std::string& input, const std::string& prefix, bool ignoreCase = true) {
    if (!StringStartsWith(input, prefix, ignoreCase)) {
        return false;
    }
    input.erase(input.begin(), input.begin() + prefix.length());
    return true;
}

// Attempt to remove a matching suffix from a string in-place. Nothing is done if the suffix does not match.
inline bool RemoveSuffix(std::string& input, const std::string& suffix, bool ignoreCase = true) {
    if (!StringEndsWith(input, suffix, ignoreCase)) {
        return false;
    }
    input.erase(input.end() - suffix.length(), input.end());
    return true;
}

// Attempt to remove a matching prefix from a string, returning a copy. An empty string is returned if the prefix does not match.
OPENRAVE_NODISCARD inline std::string RemovePrefix(const std::string& input, const std::string& prefix, bool ignoreCase = true) {
    if (!StringStartsWith(input, prefix, ignoreCase)) {
        return "";
    }
    return input.substr(prefix.length());
}

// Attempt to remove a matching suffix from a string, returning a copy. An empty string is returned if the suffix does not match.
OPENRAVE_NODISCARD inline std::string RemoveSuffix(const std::string& input, const std::string& suffix, bool ignoreCase = true) {
    if (!StringEndsWith(input, suffix, ignoreCase)) {
        return "";
    }
    return input.substr(0, input.size() - suffix.length());
}

} // namespace OpenRAVE

#endif // RAVE_STRINGUTILS
