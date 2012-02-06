// -*- coding: utf-8 -*-
// Copyright (C) 2006-2012 Rosen Diankov <rosen.diankov@gmail.com>
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
#include "libopenrave.h"
#include <openrave/utils.h>

#include "md5.h"

namespace OpenRAVE {
namespace utils {

std::string GetMD5HashString(const std::string& s)
{
    if( s.size() == 0 )
        return "";

    md5_state_t state;
    md5_byte_t digest[16];

    md5_init(&state);
    md5_append(&state, (const md5_byte_t *)s.c_str(), s.size());
    md5_finish(&state, digest);
    string hex_output;
    hex_output.resize(32);
    for (int di = 0; di < 16; ++di) {
        int n = (digest[di]&0xf);
        hex_output[2*di+1] = n > 9 ? ('a'+n-10) : ('0'+n);
        n = (digest[di]&0xf0)>>4;
        hex_output[2*di+0] = n > 9 ? ('a'+n-10) : ('0'+n);
    }
    return hex_output;
}

std::string GetMD5HashString(const std::vector<uint8_t>&v)
{
    if( v.size() == 0 )
        return "";

    md5_state_t state;
    md5_byte_t digest[16];

    md5_init(&state);
    md5_append(&state, (const md5_byte_t *)&v[0], v.size());
    md5_finish(&state, digest);
    string hex_output;
    hex_output.resize(32);
    for (int di = 0; di < 16; ++di) {
        int n = (digest[di]&0xf);
        hex_output[2*di+0] = n > 9 ? ('a'+n-10) : ('0'+n);
        n = (digest[di]&0xf0)>>4;
        hex_output[2*di+1] = n > 9 ? ('a'+n-10) : ('0'+n);
    }
    return hex_output;
}

bool PairStringLengthCompare(const std::pair<std::string, std::string>&p0, const std::pair<std::string, std::string>&p1)
{
    return p0.first.size() > p1.first.size();
}

std::string& SearchAndReplace(std::string& out, const std::string& in, const std::vector< std::pair<std::string, std::string> >&_pairs)
{
    BOOST_ASSERT(&out != &in);
    FOREACHC(itp,_pairs) {
        BOOST_ASSERT(itp->first.size()>0);
    }
    std::vector< std::pair<std::string, std::string> > pairs = _pairs;
    stable_sort(pairs.begin(),pairs.end(),PairStringLengthCompare);
    out.resize(0);
    size_t startindex = 0;
    while(startindex < in.size()) {
        size_t nextindex=std::string::npos;
        std::vector< std::pair<std::string, std::string> >::const_iterator itbestp;
        FOREACHC(itp,pairs) {
            size_t index = in.find(itp->first,startindex);
            if((nextindex == std::string::npos)|| ((index != std::string::npos)&&(index < nextindex)) ) {
                nextindex = index;
                itbestp = itp;
            }
        }
        if( nextindex == std::string::npos ) {
            out += in.substr(startindex);
            break;
        }
        out += in.substr(startindex,nextindex-startindex);
        out += itbestp->second;
        startindex = nextindex+itbestp->first.size();
    }
    return out;
}

std::string GetFilenameUntilSeparator(std::istream& sinput, char separator)
{
    std::string filename;
    if( !getline(sinput, filename, separator) ) {
        // just input directly
        sinput >> filename;
    }

    // trim leading spaces
    std::size_t startpos = filename.find_first_not_of(" \t");
    std::size_t endpos = filename.find_last_not_of(" \t");

    // if all spaces or empty return an empty string
    if( string::npos == startpos || string::npos == endpos ) {
        return "";
    }
    return filename.substr( startpos, endpos-startpos+1 );
}

} // utils
} // OpenRAVE
