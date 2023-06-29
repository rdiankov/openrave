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

#include <string>
#include <pcrecpp.h>

namespace OpenRAVE {

static const pcrecpp::RE regex("^(?:([\\w]+):[\\/]{0,2})?(\\/?[\\w.]+)([\\/\\w.]*)(?:\??([\\w=.&]*))?(?:\\#?([\\w]*))?");

bool DecomposeURI(const std::string& uri, std::string& scheme, std::string& authority, std::string& path, std::string& query, std::string& fragment) {
    return regex.FullMatch(uri, &scheme, &authority, &path, &query, &fragment);
}

} // namespace OpenRAVE
