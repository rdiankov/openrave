// -*- coding: utf-8 -*-
// Copyright (C) Rosen Diankov (rosen.diankov@gmail.com)
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
#include "openrave/hashcontext.h"
#include "md5.h"

namespace OpenRAVE {

HashContext::HashContext(const int precision)
    : _roundingScale(pow(10, precision))
{
    _md5_state = new md5_state_t();
    md5_init(_md5_state);
}

HashContext::~HashContext() {
    delete _md5_state;
}

const std::string& HashContext::HexDigest()
{
    if (!_hexDigest.empty()) {
        return _hexDigest;
    }

    md5_byte_t digest[16];
    md5_finish(_md5_state, digest);

    _hexDigest.resize(32);
    for (int di = 0; di < 16; ++di) {
        int n = (digest[di] & 0xf);
        _hexDigest[2 * di + 1] = n > 9 ? ('a' + n - 10) : ('0' + n);
        n = (digest[di] & 0xf0) >> 4;
        _hexDigest[2 * di + 0] = n > 9 ? ('a' + n - 10) : ('0' + n);
    }
    return _hexDigest;
}

void HashContext::_Append(const uint8_t* data, size_t len)
{
    md5_append(_md5_state, (const md5_byte_t*)data, len);
}

} // end namespace OpenRAVE
