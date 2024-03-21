// -*- coding: utf-8 -*-
// Copyright (C) 2023 Tan Li Boon <undisputed.seraphim@gmail.com>
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

namespace OpenRAVE {

// Returns true if a string begins with a matching prefix.
inline bool StringStartsWith(string_view input, string_view prefix, bool ignoreCase = true)
{
    if (input.length() < prefix.length()) {
        return false;
    }
    if (ignoreCase) {
        return std::equal(prefix.begin(), prefix.end(), input.begin(), [](char l, char r) -> bool {
            return std::tolower(l) == std::tolower(r);
        });
    }
    return std::equal(prefix.begin(), prefix.end(), input.begin());
}

// Returns true if a string ends with a matching suffix.
inline bool StringEndsWith(string_view input, string_view suffix, bool ignoreCase = true)
{
    if (input.length() < suffix.length()) {
        return false;
    }
    if (ignoreCase) {
        return std::equal(suffix.rbegin(), suffix.rend(), input.rbegin(), [](char l, char r) -> bool {
            return std::tolower(l) == std::tolower(r);
        });
    }
    return std::equal(suffix.rbegin(), suffix.rend(), input.rbegin());
}

// Attempt to remove a matching prefix from a string in-place. Nothing is done if the prefix does not match.
inline bool RemovePrefix(std::string& input, string_view prefix, bool ignoreCase = true)
{
    if (!StringStartsWith(input, prefix, ignoreCase)) {
        return false;
    }
    input.erase(0, prefix.length());
    return true;
}

// Attempt to remove a matching suffix from a string in-place. Nothing is done if the suffix does not match.
inline bool RemoveSuffix(std::string& input, string_view suffix, bool ignoreCase = true)
{
    if (!StringEndsWith(input, suffix, ignoreCase)) {
        return false;
    }
    input.erase(input.length() - suffix.length());
    return true;
}

// Attempt to remove a matching prefix from a string, returning a copy. An empty string is returned if the prefix does not match.
inline std::string RemovePrefix(string_view input, string_view prefix, bool ignoreCase = true)
{
    if (!StringStartsWith(input, prefix, ignoreCase)) {
        return "";
    }
    return {input.begin() + prefix.length(), input.end()};
}

// Attempt to remove a matching suffix from a string, returning a copy. An empty string is returned if the suffix does not match.
inline std::string RemoveSuffix(string_view input, string_view suffix, bool ignoreCase = true)
{
    if (!StringEndsWith(input, suffix, ignoreCase)) {
        return "";
    }
    return {input.begin(), input.end() - suffix.length()};
}

/// \brief get the scheme of the uri, e.g. file: or openrave:
inline void ParseURI(const char* pUri, std::string& scheme, std::string& path, std::string& fragment)
{
    path = pUri;
    size_t hashindex = path.find_last_of('#');
    if (hashindex != std::string::npos) {
        fragment = path.substr(hashindex + 1);
        path = path.substr(0, hashindex);
    }

    size_t colonindex = path.find_first_of(':');
    if (colonindex != std::string::npos) {
        // notice: in python code, like realtimerobottask3.py, it pass scheme as {openravescene: mujin}. No colon,
        scheme = path.substr(0, colonindex);
        path = path.substr(colonindex + 1);
    }
}

} // namespace OpenRAVE

#endif // RAVE_STRINGUTILS
