// Copyright (C) 2006-2010 Rosen Diankov (rosen.diankov@gmail.com)
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
#ifndef OPENRAVE_PYTHON_DOCSTRINGS
#define OPENRAVE_PYTHON_DOCSTRINGS

#include <sstream>
#include <map>

#if BOOST_VERSION >= 103500
//#define DOXY_ENUM(name) ,".. doxygenenum:: "#name"\n"
#define DOXY_ENUM(name) ,openravepy::GetDoxygenComment("enum",#name)
#else
#define DOXY_ENUM(name) 
#endif

//#define DOXY_FN(class,name) ".. doxygenfunction:: "#name"\n\n"
#define DOXY_FN(class,name) openravepy::GetDoxygenComment("function",#name,#class)
#define DOXY_FN1(name) openravepy::GetDoxygenComment("function",#name)

// brief+detailed descriptions only
#define DOXY_CLASS(name) openravepy::GetDoxygenComment("class",#name)

namespace openravepy {

inline std::string InitializeLanguageCode()
{
    // only english and japanese are supports, default to english.
    return "en";
}

void InitializeComments(std::map<std::string,std::string>& comments);

inline const char* GetDoxygenComment(const char* type, const char* name, const char* classtype=NULL)
{
    static std::string s_langcode;
    static std::map<std::string, std::string> s_comments;
    if( s_langcode.size() == 0 ) {
        s_langcode = InitializeLanguageCode();
    }
    if( s_comments.size() == 0 ) {
        InitializeComments(s_comments);
    }
    std::stringstream ss;
    ss << s_langcode << " " << type;
    if( !!classtype ) {
        ss << " " << classtype;
    }
    ss << " " << name;
    std::map<std::string, std::string>::const_iterator it = s_comments.find(ss.str());
    if( it == s_comments.end() ) {
        return "";
    }
    return it->second.c_str();
}

} // end namespace openravepy

#endif
