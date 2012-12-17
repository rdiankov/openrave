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
/** \file configurationspecification.cpp
    \brief All definitions that involve ConfigurationSpecification
 */
#include "libopenrave.h"
#include <boost/lexical_cast.hpp>

namespace OpenRAVE {

ConfigurationSpecification RaveGetAffineConfigurationSpecification(int affinedofs,KinBodyConstPtr pbody,const std::string& interpolation)
{
    ConfigurationSpecification spec;
    spec._vgroups.resize(1);
    spec._vgroups[0].offset = 0;
    spec._vgroups[0].dof = RaveGetAffineDOF(affinedofs);
    spec._vgroups[0].interpolation = interpolation;
    if( !!pbody ) {
        spec._vgroups[0].name = str(boost::format("affine_transform %s %d")%pbody->GetName()%affinedofs);
    }
    else {
        spec._vgroups[0].name = str(boost::format("affine_transform __dummy__ %d")%affinedofs);
    }
    return spec;
}

ConfigurationSpecification::ConfigurationSpecification()
{
}

ConfigurationSpecification::ConfigurationSpecification(const ConfigurationSpecification::Group& g)
{
    BOOST_ASSERT(g.dof>=0);
    _vgroups.push_back(g);
    _vgroups.front().offset = 0;
}

ConfigurationSpecification::ConfigurationSpecification(const ConfigurationSpecification& c)
{
    _vgroups = c._vgroups;
}

int ConfigurationSpecification::GetDOF() const
{
    int maxdof = 0;
    FOREACHC(it,_vgroups) {
        maxdof = max(maxdof,it->offset+it->dof);
    }
    return maxdof;
}

bool ConfigurationSpecification::IsValid() const
{
    vector<uint8_t> occupied(GetDOF(),0);
    FOREACHC(it,_vgroups) {
        if(it->offset < 0 || it->dof <= 0 || it->offset+it->dof > (int)occupied.size()) {
            return false;
        }
        for(int i = it->offset; i < it->offset+it->dof; ++i) {
            if( occupied[i] ) {
                return false;
            }
            occupied[i] = 1;
        }
    }
    FOREACH(it,occupied) {
        if( *it == 0 ) {
            return false;
        }
    }
    // check for repeating names
    FOREACHC(it,_vgroups) {
        for(std::vector<Group>::const_iterator it2 = it+1; it2 != _vgroups.end(); ++it2) {
            if( it->name == it2->name ) {
                return false;
            }
        }
    }
    return true;
}

void ConfigurationSpecification::Validate() const
{
    vector<uint8_t> occupied(GetDOF(),0);
    FOREACHC(it,_vgroups) {
        OPENRAVE_ASSERT_OP(it->offset,>=, 0);
        OPENRAVE_ASSERT_OP(it->dof,>,0);
        OPENRAVE_ASSERT_OP(it->offset+it->dof,<=,(int)occupied.size());
        for(int i = it->offset; i < it->offset+it->dof; ++i) {
            OPENRAVE_ASSERT_FORMAT0(!occupied[i],"ocupied when it shoultn't be",ORE_Assert);
            occupied[i] = 1;
        }
    }
    FOREACH(it,occupied) {
        OPENRAVE_ASSERT_OP_FORMAT0(*it,!=,0,"found unocupied index",ORE_Assert);
    }
    // check for repeating names
    FOREACHC(it,_vgroups) {
        for(std::vector<Group>::const_iterator it2 = it+1; it2 != _vgroups.end(); ++it2) {
            OPENRAVE_ASSERT_OP_FORMAT0(it->name,!=,it2->name,"repeating names",ORE_Assert);
        }
    }
}

bool ConfigurationSpecification::operator==(const ConfigurationSpecification& r) const
{
    if( _vgroups.size() != r._vgroups.size() ) {
        return false;
    }
    // the groups could be out of order
    for(size_t i = 0; i < _vgroups.size(); ++i) {
        size_t j;
        for(j=0; j < r._vgroups.size(); ++j) {
            if( _vgroups[i].offset == r._vgroups[j].offset ) {
                if( _vgroups[i] != r._vgroups[j] ) {
                    return false;
                }
                break;
            }
        }
        if( j >= r._vgroups.size() ) {
            return false;
        }
    }
    return true;
}

bool ConfigurationSpecification::operator!=(const ConfigurationSpecification& r) const
{
    return !this->operator==(r);
}

const ConfigurationSpecification::Group& ConfigurationSpecification::GetGroupFromName(const std::string& name) const
{
    size_t bestmatch=0xffffffff;
    vector<Group>::const_iterator itbestgroup;
    FOREACHC(itgroup,_vgroups) {
        if( itgroup->name.size() >= name.size() ) {
            if( itgroup->name.size() == name.size() ) {
                if( itgroup->name == name ) {
                    return *itgroup;
                }
            }
            else {
                if( itgroup->name.substr(0,name.size()) == name ) {
                    size_t match = itgroup->name.size()-name.size();
                    if( match < bestmatch ) {
                        itbestgroup = itgroup;
                        bestmatch = match;
                    }
                }
            }
        }
    }
    if(bestmatch==0xffffffff) {
        throw OPENRAVE_EXCEPTION_FORMAT("failed to find group %s",name,ORE_InvalidArguments);
    }
    return *itbestgroup;
}

ConfigurationSpecification::Group& ConfigurationSpecification::GetGroupFromName(const std::string& name)
{
    size_t bestmatch=0xffffffff;
    vector<Group>::iterator itbestgroup;
    FOREACH(itgroup,_vgroups) {
        if( itgroup->name.size() >= name.size() ) {
            if( itgroup->name.size() == name.size() ) {
                if( itgroup->name == name ) {
                    return *itgroup;
                }
            }
            else {
                if( itgroup->name.substr(0,name.size()) == name ) {
                    size_t match = itgroup->name.size()-name.size();
                    if( match < bestmatch ) {
                        itbestgroup = itgroup;
                        bestmatch = match;
                    }
                }
            }
        }
    }
    if(bestmatch==0xffffffff) {
        throw OPENRAVE_EXCEPTION_FORMAT("failed to find group %s",name,ORE_InvalidArguments);
    }
    return *itbestgroup;
}

std::vector<ConfigurationSpecification::Group>::const_iterator ConfigurationSpecification::FindCompatibleGroup(const ConfigurationSpecification::Group& g, bool exactmatch) const
{
    std::vector<ConfigurationSpecification::Group>::const_iterator itcompatgroup = FindCompatibleGroup(g.name,exactmatch);
    if( itcompatgroup != _vgroups.end() && exactmatch ) {
        if( itcompatgroup->dof != g.dof ) {
            return _vgroups.end();
        }
    }
    return itcompatgroup;
}

std::vector<ConfigurationSpecification::Group>::const_iterator ConfigurationSpecification::FindCompatibleGroup(const std::string& name, bool exactmatch) const
{
    std::vector<ConfigurationSpecification::Group>::const_iterator itsemanticmatch = _vgroups.end();
    uint32_t bestmatchscore = 0;
    stringstream ss(name);
    std::vector<std::string> tokens((istream_iterator<std::string>(ss)), istream_iterator<std::string>());
    if( tokens.size() == 0 ) {
        return _vgroups.end();
    }
    FOREACHC(itgroup,_vgroups) {
        if( itgroup->name == name ) {
            return itgroup;
        }
        if( exactmatch ) {
            continue;
        }
        ss.clear();
        ss.str(itgroup->name);
        std::vector<std::string> curtokens((istream_iterator<std::string>(ss)), istream_iterator<std::string>());
        if( curtokens.size() == 0 ) {
            continue;
        }
        if( curtokens.at(0) == tokens.at(0) ) {
            uint32_t matchscore=1;
            if( curtokens.size() > 1 && tokens.size() > 1 ) {
                if( curtokens.at(1) != tokens.at(1) ) {
                    // both names exist and do not match so cannot go any further!
                    continue;
                }
                if( curtokens.size() > 2 && tokens.size() > 2 ) {
                    for(size_t i = 2; i < tokens.size(); ++i) {
                        if( find(curtokens.begin()+2,curtokens.end(),tokens[i]) != curtokens.end() ) {
                            matchscore += 1;
                        }
                    }
                }
            }
            if( bestmatchscore < matchscore ) {
                itsemanticmatch = itgroup;
                bestmatchscore = matchscore;
            }
        }
    }
    return itsemanticmatch;
}

std::vector<ConfigurationSpecification::Group>::const_iterator ConfigurationSpecification::FindTimeDerivativeGroup(const ConfigurationSpecification::Group& g, bool exactmatch) const
{
    std::vector<ConfigurationSpecification::Group>::const_iterator itcompatgroup = FindTimeDerivativeGroup(g.name,exactmatch);
    if( itcompatgroup != _vgroups.end() ) {
        if( itcompatgroup->dof != g.dof ) {
            return _vgroups.end();
        }
    }
    return itcompatgroup;
}

std::vector<ConfigurationSpecification::Group>::const_iterator ConfigurationSpecification::FindTimeDerivativeGroup(const std::string& name, bool exactmatch) const
{
    string derivativename;
    if( name.size() >= 12 && name.substr(0,12) == "joint_values" ) {
        derivativename = string("joint_velocities") + name.substr(12);
    }
    else if( name.size() >= 16 && name.substr(0,16) == "joint_velocities" ) {
        derivativename = string("joint_accelerations") + name.substr(16);
    }
    else if( name.size() >= 16 && name.substr(0,16) == "affine_transform" ) {
        derivativename = string("affine_velocities") + name.substr(16);
    }
    else if( name.size() >= 17 && name.substr(0,17) == "affine_velocities" ) {
        derivativename = string("affine_accelerations") + name.substr(17);
    }
    else if( name.size() >= 14 && name.substr(0,14) == "ikparam_values" ) {
        derivativename = string("ikparam_velocities") + name.substr(14);
    }
    else {
        return _vgroups.end();
    }
    return FindCompatibleGroup(derivativename,exactmatch);
}

void ConfigurationSpecification::AddDerivativeGroups(int deriv, bool adddeltatime)
{
    static const boost::array<string,3> s_GroupsJointValues = {{"joint_values","joint_velocities", "joint_accelerations"}};
    static const boost::array<string,3> s_GroupsAffine = {{"affine_transform","affine_velocities","ikparam_accelerations"}};
    static const boost::array<string,3> s_GroupsIkparam = {{"ikparam_values","ikparam_velocities","affine_accelerations"}};
    if( _vgroups.size() == 0 ) {
        return;
    }
    std::list<std::vector<ConfigurationSpecification::Group>::iterator> listtoremove;
    std::list<ConfigurationSpecification::Group> listadd;
    int offset = GetDOF();
    bool hasdeltatime = false;
    FOREACH(itgroup,_vgroups) {
        string replacename;
        int offset = -1;
        if( itgroup->name.size() >= 12 && itgroup->name.substr(0,12) == "joint_values" ) {
            replacename = s_GroupsJointValues.at(deriv);
            offset = 12;
        }
        else if( itgroup->name.size() >= 16 && itgroup->name.substr(0,16) == "affine_transform" ) {
            replacename = s_GroupsAffine.at(deriv);
            offset = 16;
        }
        else if( itgroup->name.size() >= 14 && itgroup->name.substr(0,14) == "ikparam_values" ) {
            replacename = s_GroupsIkparam.at(deriv);
            offset = 14;
        }

        if( offset > 0 ) {
            ConfigurationSpecification::Group g;
            g.name = replacename + itgroup->name.substr(offset);
            g.dof = itgroup->dof;
            g.interpolation = GetInterpolationDerivative(itgroup->interpolation,deriv);
            std::vector<ConfigurationSpecification::Group>::const_iterator itcompat = FindCompatibleGroup(g);
            if( itcompat != _vgroups.end() ) {
                if( itcompat->dof == g.dof ) {
                    ConfigurationSpecification::Group& gmodify = _vgroups.at(itcompat-_vgroups.begin());
                    gmodify.name = g.name;
                }
                else {
                    listtoremove.push_back(_vgroups.begin()+(itcompat-_vgroups.begin()));
                    listadd.push_back(g);
                }
            }
            else {
                listadd.push_back(g);
            }
        }
        else {
            if( !hasdeltatime ) {
                hasdeltatime = itgroup->name.size() >= 9 && itgroup->name.substr(0,9) == "deltatime";
            }
        }
    }
    if( listtoremove.size() > 0 ) {
        FOREACH(it,listtoremove) {
            _vgroups.erase(*it);
        }
        ResetGroupOffsets();
        offset = GetDOF();
    }
    FOREACH(itadd, listadd) {
        itadd->offset = offset;
        offset += itadd->dof;
        _vgroups.push_back(*itadd);
    }
    if( !hasdeltatime && adddeltatime ) {
        AddDeltaTimeGroup();
    }
}

ConfigurationSpecification ConfigurationSpecification::ConvertToVelocitySpecification() const
{
    ConfigurationSpecification vspec;
    vspec._vgroups = _vgroups;
    FOREACH(itgroup,vspec._vgroups) {
        if( itgroup->name.size() >= 12 && itgroup->name.substr(0,12) == "joint_values" ) {
            itgroup->name = string("joint_velocities") + itgroup->name.substr(12);
        }
        else if( itgroup->name.size() >= 16 && itgroup->name.substr(0,16) == "affine_transform" ) {
            itgroup->name = string("affine_velocities") + itgroup->name.substr(16);
        }
        else if( itgroup->name.size() >= 14 && itgroup->name.substr(0,14) == "ikparam_values" ) {
            itgroup->name = string("ikparam_velocities") + itgroup->name.substr(14);
        }

        itgroup->interpolation = GetInterpolationDerivative(itgroup->interpolation);
    }
    return vspec;
}

ConfigurationSpecification ConfigurationSpecification::GetTimeDerivativeSpecification(int timederivative) const
{
    ConfigurationSpecification vspec;
    const boost::array<string,3> posgroups = {{"joint_values","affine_transform","ikparam_values"}};
    const boost::array<string,3> velgroups = {{"joint_velocities","affine_velocities","ikparam_velocities"}};
    const boost::array<string,3> accgroups = {{"joint_accelerations","affine_accelerations","ikparam_accelerations"}};
    const boost::array<string,3>* pgroup=NULL;
    if( timederivative == 0 ) {
        pgroup = &posgroups;
    }
    else if( timederivative == 1 ) {
        pgroup = &velgroups;
    }
    else if( timederivative == 2 ) {
        pgroup = &accgroups;
    }
    else {
        throw OPENRAVE_EXCEPTION_FORMAT0("invalid timederivative",ORE_InvalidArguments);
    }

    FOREACHC(itgroup,_vgroups) {
        for(size_t i = 0; i < pgroup->size(); ++i) {
            const string& name = pgroup->at(i);
            if( itgroup->name.size() >= name.size() && itgroup->name.substr(0,name.size()) == name ) {
                vspec._vgroups.push_back(*itgroup);
                break;
            }
        }
    }
    vspec.ResetGroupOffsets();
    return vspec;
}

void ConfigurationSpecification::ResetGroupOffsets()
{
    int offset = 0;
    FOREACH(it,_vgroups) {
        it->offset = offset;
        offset += it->dof;
    }
}

int ConfigurationSpecification::AddDeltaTimeGroup()
{
    int dof = 0;
    for(size_t i = 0; i < _vgroups.size(); ++i) {
        dof = max(dof,_vgroups[i].offset+_vgroups[i].dof);
        if( _vgroups[i].name == "deltatime" ) {
            return _vgroups[i].offset;
        }
    }
    ConfigurationSpecification::Group g;
    g.name = "deltatime";
    g.offset = dof;
    g.dof = 1;
    _vgroups.push_back(g);
    return g.offset;
}

int ConfigurationSpecification::AddGroup(const std::string& name, int dof, const std::string& interpolation)
{
    BOOST_ASSERT(name.size()>0);
    stringstream ss(name);
    std::vector<std::string> tokens((istream_iterator<std::string>(ss)), istream_iterator<std::string>());
    int specdof = 0;
    for(size_t i = 0; i < _vgroups.size(); ++i) {
        specdof = max(specdof,_vgroups[i].offset+_vgroups[i].dof);
        if( _vgroups[i].name == name ) {
            BOOST_ASSERT(_vgroups[i].dof==dof);
            return _vgroups[i].offset;
        }
        else if( _vgroups[i].name.size() >= tokens[0].size() && _vgroups[i].name.substr(0,tokens[0].size()) == tokens[0] ) {
            // first token matches, check if the next token does also
            ss.clear();
            ss.str(_vgroups[i].name);
            std::vector<std::string> newtokens((istream_iterator<std::string>(ss)), istream_iterator<std::string>());
            if( newtokens.size() >= 2 && tokens.size() >= 2 ) {
                if( newtokens[1] == tokens[1] ) {
                    throw OPENRAVE_EXCEPTION_FORMAT("new group '%s' conflicts with existing group '%s'",name%_vgroups[i].name,ORE_InvalidArguments);
                }
            }
        }
    }
    ConfigurationSpecification::Group g;
    g.name = name;
    g.offset = specdof;
    g.dof = dof;
    g.interpolation = interpolation;
    _vgroups.push_back(g);
    return g.offset;
}

int ConfigurationSpecification::AddGroup(const ConfigurationSpecification::Group& g)
{
    return AddGroup(g.name,g.dof,g.interpolation);
}

ConfigurationSpecification& ConfigurationSpecification::operator+= (const ConfigurationSpecification& r)
{
    list< std::vector<Group>::const_iterator > listaddgroups;
    stringstream ss;
    vector<int> vindices;
    FOREACHC(itrgroup,r._vgroups) {
        std::vector<Group>::const_iterator itcompatgroupconst = FindCompatibleGroup(*itrgroup,false);
        if( itcompatgroupconst == _vgroups.end() ) {
            listaddgroups.push_back(itrgroup);
        }
        else {
            std::vector<Group>::iterator itcompatgroup = _vgroups.begin()+(itcompatgroupconst-_vgroups.begin());
            if( itcompatgroup->interpolation.size() == 0 ) {
                itcompatgroup->interpolation = itrgroup->interpolation;
            }
            else if( itrgroup->interpolation.size() > 0 && itrgroup->interpolation != itcompatgroup->interpolation ) {
                RAVELOG_WARN(str(boost::format("interpolation values of group %s differ: %s!=%s")%itcompatgroup->name%itcompatgroup->interpolation%itrgroup->interpolation));
            }

            if( itcompatgroup->name == itrgroup->name ) {
                BOOST_ASSERT(itrgroup->dof == itcompatgroup->dof);
            }
            else {
                // have to divide into tokens
                ss.clear();
                ss.str(itcompatgroup->name);
                std::vector<std::string> targettokens((istream_iterator<std::string>(ss)), istream_iterator<std::string>());
                ss.clear();
                ss.str(itrgroup->name);
                std::vector<std::string> sourcetokens((istream_iterator<std::string>(ss)), istream_iterator<std::string>());

                if( targettokens.at(0).size() >= 6 && targettokens.at(0).substr(0,6) == "joint_") {
                    if( targettokens.size() >= 2 && sourcetokens.size() >= 2 && targettokens.at(1) == sourcetokens.at(1) ) {
                        BOOST_ASSERT((int)targettokens.size()>=itcompatgroup->dof+2);
                        BOOST_ASSERT((int)sourcetokens.size()>=itrgroup->dof+2);
                        vindices.resize(itcompatgroup->dof);
                        for(size_t i = 0; i < vindices.size(); ++i) {
                            vindices[i] = boost::lexical_cast<int>(targettokens.at(i+2));
                        }
                        for(int i = 0; i < itrgroup->dof; ++i) {
                            int index = boost::lexical_cast<int>(sourcetokens.at(i+2));
                            if( find(vindices.begin(),vindices.end(),index) == vindices.end() ) {
                                itcompatgroup->name += string(" ");
                                itcompatgroup->name += sourcetokens.at(i+2);
                                itcompatgroup->dof += 1;
                                vindices.push_back(index);
                            }
                        }
                    }
                    else {
                        listaddgroups.push_back(itrgroup);
                    }
                }
                else if( targettokens.at(0).size() >= 7 && targettokens.at(0).substr(0,7) == "affine_") {
                    if( targettokens.size() >= 2 && sourcetokens.size() >= 2 && targettokens.at(1) == sourcetokens.at(1) ) {
                        int targetmask = boost::lexical_cast<int>(targettokens.at(2));
                        int sourcemask = boost::lexical_cast<int>(sourcetokens.at(2));
                        targetmask |= sourcemask;
                        if( targetmask & DOF_RotationQuat ) {
                            targetmask = (targetmask&~DOF_RotationMask)|DOF_RotationQuat;
                        }
                        if( targetmask & DOF_Rotation3D ) {
                            targetmask = (targetmask&~DOF_RotationMask)|DOF_Rotation3D;
                        }

                        targettokens[2] = boost::lexical_cast<string>(targetmask);
                        itcompatgroup->dof = RaveGetAffineDOF(targetmask);
                        itcompatgroup->name = targettokens.at(0);
                        for(size_t i = 1; i < targettokens.size(); ++i) {
                            itcompatgroup->name += string(" ");
                            itcompatgroup->name += targettokens[i];
                        }
                    }
                    else {
                        listaddgroups.push_back(itrgroup);
                    }
                }
                else if( targettokens.at(0).size() >= 8 && targettokens.at(0).substr(0,8) == "ikparam_") {
                    // no idea how to merge two different ikparams...
                    listaddgroups.push_back(itrgroup);
                }
                else if( targettokens.at(0).size() >= 4 && targettokens.at(0).substr(0,4) == "grab") {
                    if( targettokens.size() >= 2 && sourcetokens.size() >= 2 && targettokens.at(1) == sourcetokens.at(1) ) {
                        BOOST_ASSERT((int)targettokens.size()>=itcompatgroup->dof+2);
                        BOOST_ASSERT((int)sourcetokens.size()>=itrgroup->dof+2);
                        vindices.resize(itcompatgroup->dof);
                        for(size_t i = 0; i < vindices.size(); ++i) {
                            vindices[i] = boost::lexical_cast<int>(targettokens.at(i+2));
                        }
                        for(int i = 0; i < itrgroup->dof; ++i) {
                            int index = boost::lexical_cast<int>(sourcetokens.at(i+2));
                            if( find(vindices.begin(),vindices.end(),index) == vindices.end() ) {
                                itcompatgroup->name += string(" ");
                                itcompatgroup->name += sourcetokens.at(i+2);
                                itcompatgroup->dof += 1;
                                vindices.push_back(index);
                            }
                        }
                    }
                    else {
                        listaddgroups.push_back(itrgroup);
                    }
                }
                else {
                    throw OPENRAVE_EXCEPTION_FORMAT("do not know how to merge group '%s' into '%s'",itrgroup->name%itcompatgroup->name,ORE_InvalidArguments);
                }
            }
        }
    }
    if( _vgroups.capacity() < _vgroups.size()+listaddgroups.size() ) {
        _vgroups.reserve(_vgroups.size()+listaddgroups.size());
    }
    FOREACH(it,listaddgroups) {
        _vgroups.push_back(**it);
    }
    ResetGroupOffsets();
    return *this;
}

ConfigurationSpecification ConfigurationSpecification::operator+ (const ConfigurationSpecification& r) const
{
    ConfigurationSpecification spec = *this;
    spec += r;
    return spec;
}

bool ConfigurationSpecification::ExtractTransform(Transform& t, std::vector<dReal>::const_iterator itdata, KinBodyConstPtr pbody, int timederivative) const
{
    bool bfound = false;
    string searchname;
    switch( timederivative ) {
    case 0: searchname = "affine_transform"; break;
    case 1: searchname = "affine_velocities"; break;
    case 2: searchname = "affine_accelerations"; break;
    default:
        throw OPENRAVE_EXCEPTION_FORMAT("bad time derivative %d",timederivative,ORE_InvalidArguments);
    }
    FOREACHC(itgroup,_vgroups) {
        if( itgroup->name.size() >= searchname.size() && itgroup->name.substr(0,searchname.size()) == searchname ) {
            stringstream ss(itgroup->name.substr(searchname.size()));
            string bodyname;
            int affinedofs=0;
            ss >> bodyname >> affinedofs;
            if( !!ss ) {
                if( !!pbody && bodyname != pbody->GetName() ) {
                    continue;
                }
                Vector vaxis(0,0,1);
                if( affinedofs & DOF_RotationAxis ) {
                    ss >> vaxis.x >> vaxis.y >> vaxis.z;
                }
                RaveGetTransformFromAffineDOFValues(t,itdata+itgroup->offset,affinedofs, vaxis, timederivative==0);
                bfound = true;
            }
        }
    }
    return bfound;
}

bool ConfigurationSpecification::ExtractIkParameterization(IkParameterization& ikparam, std::vector<dReal>::const_iterator itdata, int timederivative) const
{
    bool bfound = false;
    string searchname;
    switch( timederivative ) {
    case 0: searchname = "ikparam_values"; break;
    case 1: searchname = "ikparam_velocities"; break;
    default:
        throw OPENRAVE_EXCEPTION_FORMAT("bad time derivative %d",timederivative,ORE_InvalidArguments);
    }
    FOREACHC(itgroup,_vgroups) {
        if( itgroup->name.size() >= searchname.size() && itgroup->name.substr(0,searchname.size()) == searchname ) {
            stringstream ss(itgroup->name.substr(searchname.size()));
            int iktype = IKP_None;
            ss >> iktype;
            if( !!ss ) {
                ikparam.Set(itdata+itgroup->offset,static_cast<IkParameterizationType>(iktype|(timederivative == 1 ? IKP_VelocityDataBit : 0)));
                bfound = true;
                if( timederivative == 0 ) {
                    // normalize parameterizations
                    switch(ikparam.GetType()) {
                    case IKP_Transform6D: {
                        Transform t = ikparam.GetTransform6D();
                        t.rot.normalize4();
                        ikparam.SetTransform6D(t);
                        break;
                    }
                    case IKP_Rotation3D: {
                        Vector quat = ikparam.GetRotation3D();
                        quat.normalize4();
                        ikparam.SetRotation3D(quat);
                        break;
                    }
                    case IKP_Direction3D: {
                        Vector dir = ikparam.GetDirection3D();
                        dir.normalize3();
                        ikparam.SetDirection3D(dir);
                        break;
                    }
                    case IKP_Ray4D: {
                        RAY r = ikparam.GetRay4D();
                        r.dir.normalize3();
                        ikparam.SetRay4D(r);
                        break;
                    }
                    case IKP_TranslationDirection5D: {
                        RAY r = ikparam.GetTranslationDirection5D();
                        r.dir.normalize3();
                        ikparam.SetTranslationDirection5D(r);
                        break;
                    }
                    default:
                        break;
                    }
                }
            }
        }
    }
    return bfound;
}

bool ConfigurationSpecification::ExtractAffineValues(std::vector<dReal>::iterator itvalues, std::vector<dReal>::const_iterator itdata, KinBodyConstPtr pbody, int affinedofs, int timederivative) const
{
    if( affinedofs == 0 ) {
        return false;
    }
    string searchname;
    switch( timederivative ) {
    case 0: searchname = "affine_values"; break;
    case 1: searchname = "affine_velocities"; break;
    case 2: searchname = "affine_accelerations"; break;
    default:
        throw OPENRAVE_EXCEPTION_FORMAT0("bad time derivative",ORE_InvalidArguments);
    };
    bool bfound = false;
    FOREACHC(itgroup,_vgroups) {
        if( itgroup->name.size() >= searchname.size() && itgroup->name.substr(0,searchname.size()) == searchname ) {
            stringstream ss(itgroup->name.substr(searchname.size()));
            string bodyname;
            int sourceaffinedofs=0;
            ss >> bodyname >> sourceaffinedofs;
            if( !!ss ) {
                if( !!pbody && bodyname != pbody->GetName() ) {
                    continue;
                }

                for(int index = 0; index < RaveGetAffineDOF(affinedofs); ++index) {
                    DOFAffine dof = RaveGetAffineDOFFromIndex(affinedofs,index);
                    int startindex = RaveGetIndexFromAffineDOF(affinedofs,dof);
                    if( sourceaffinedofs & dof ) {
                        int sourceindex = RaveGetIndexFromAffineDOF(sourceaffinedofs,dof);
                        *(itvalues+index) = *(itdata + sourceindex + (index-startindex));
                    }
                }
                bfound = true;
            }
        }
    }
    return bfound;
}

bool ConfigurationSpecification::ExtractJointValues(std::vector<dReal>::iterator itvalues, std::vector<dReal>::const_iterator itdata, KinBodyConstPtr pbody, const std::vector<int>& indices, int timederivative) const
{
    if( indices.size() == 0 ) {
        return false;
    }
    string searchname;
    switch( timederivative ) {
    case 0: searchname = "joint_values"; break;
    case 1: searchname = "joint_velocities"; break;
    case 2: searchname = "joint_accelerations"; break;
    default:
        throw OPENRAVE_EXCEPTION_FORMAT0("bad time derivative",ORE_InvalidArguments);
    };
    bool bfound = false;
    FOREACHC(itgroup,_vgroups) {
        if( itgroup->name.size() >= searchname.size() && itgroup->name.substr(0,searchname.size()) == searchname ) {
            stringstream ss(itgroup->name.substr(searchname.size()));
            string bodyname;
            ss >> bodyname;
            if( !!ss ) {
                if( !!pbody && bodyname != pbody->GetName() ) {
                    continue;
                }
                vector<int> vgroupindices((istream_iterator<int>(ss)), istream_iterator<int>());
                for(size_t i = 0; i < indices.size(); ++i) {
                    std::vector<int>::iterator it = find(vgroupindices.begin(),vgroupindices.end(),indices[i]);
                    if( it != vgroupindices.end() ) {
                        *(itvalues+i) = *(itdata+itgroup->offset+(it-vgroupindices.begin()));
                    }
                }
                bfound = true;
            }
        }
    }
    return bfound;
}

bool ConfigurationSpecification::ExtractDeltaTime(dReal& deltatime, std::vector<dReal>::const_iterator itdata) const
{
    FOREACHC(itgroup,_vgroups) {
        if( itgroup->name == "deltatime" ) {
            deltatime = *(itdata+itgroup->offset);
            return true;
        }
    }
    return false;
}

bool ConfigurationSpecification::InsertJointValues(std::vector<dReal>::iterator itdata, std::vector<dReal>::const_iterator itvalues, KinBodyConstPtr pbody, const std::vector<int>& indices, int timederivative) const
{
    if( indices.size() == 0 ) {
        return false;
    }
    string searchname;
    switch( timederivative ) {
    case 0: searchname = "joint_values"; break;
    case 1: searchname = "joint_velocities"; break;
    case 2: searchname = "joint_accelerations"; break;
    default:
        throw OPENRAVE_EXCEPTION_FORMAT0("bad time derivative",ORE_InvalidArguments);
    };
    bool bfound = false;
    FOREACHC(itgroup,_vgroups) {
        if( itgroup->name.size() >= searchname.size() && itgroup->name.substr(0,searchname.size()) == searchname ) {
            stringstream ss(itgroup->name.substr(searchname.size()));
            string bodyname;
            ss >> bodyname;
            if( !!ss ) {
                if( !!pbody && bodyname != pbody->GetName() ) {
                    continue;
                }
                vector<int> vgroupindices((istream_iterator<int>(ss)), istream_iterator<int>());
                for(size_t i = 0; i < vgroupindices.size(); ++i) {
                    std::vector<int>::const_iterator it = find(indices.begin(),indices.end(),vgroupindices[i]);
                    if( it != indices.end() ) {
                        *(itdata+itgroup->offset+i) = *(itvalues+static_cast<size_t>(it-indices.begin()));
                    }
                }
                bfound = true;
            }
        }
    }
    return bfound;
}

bool ConfigurationSpecification::InsertDeltaTime(std::vector<dReal>::iterator itdata, dReal deltatime) const
{
    bool bfound = false;
    FOREACHC(itgroup,_vgroups) {
        if( itgroup->name == "deltatime" ) {
            *(itdata+itgroup->offset) = deltatime;
            bfound = true;
        }
    }
    return bfound;
}

/** sets the body transform
    \param[in] values the affine dof values
    \param[in] affinedofs the affine dof mask
    \param[in] vaxis optional rotation axis if affinedofs specified \ref DOF_RotationAxis
    \param[in] if true will normalize rotations, should set to false if extracting velocity data
 */
void SetBodyTransformFromAffineDOFValues(const std::vector<dReal>& values, KinBodyPtr pbody, int affinedofs, const Vector& vaxis)
{
    Transform t;
    if( affinedofs != DOF_Transform ) {
        t = pbody->GetTransform();
    }
    RaveGetTransformFromAffineDOFValues(t,values.begin(),affinedofs, vaxis, true);
    pbody->SetTransform(t);
}

void GetAffineDOFValuesFromBodyTransform(std::vector<dReal>& values, KinBodyPtr pbody, int affinedofs, const Vector& vaxis)
{
    values.resize(RaveGetAffineDOF(affinedofs));
    RaveGetAffineDOFValuesFromTransform(values.begin(), pbody->GetTransform(), affinedofs, vaxis);
}

void SetBodyVelocityFromAffineDOFVelocities(const std::vector<dReal>& values, KinBodyPtr pbody, int affinedofs, const Vector& vaxis)
{
    Vector linearvel, angularvel;
    Vector quatrotation;
    if( affinedofs != DOF_Transform && affinedofs != (DOF_XYZ|DOF_Rotation3D) ) {
        pbody->GetLinks().at(0)->GetVelocity(linearvel, angularvel);
    }
    if( affinedofs & DOF_RotationQuat ) {
        quatrotation = pbody->GetTransform().rot;
    }
    RaveGetVelocityFromAffineDOFVelocities(linearvel, angularvel, values.begin(), affinedofs, vaxis, quatrotation);
    pbody->SetVelocity(linearvel, angularvel);
}

void GetAffineDOFVelocitiesFromBodyVelocity(std::vector<dReal>& values, KinBodyPtr pbody, int affinedofs, const Vector& vaxis)
{
    values.resize(RaveGetAffineDOF(affinedofs));
    Vector linearvel, angularvel;
    Vector quatrotation;
    pbody->GetLinks().at(0)->GetVelocity(linearvel, angularvel);
    if( affinedofs & DOF_RotationQuat ) {
        quatrotation = pbody->GetTransform().rot;
    }
    RaveGetAffineDOFValuesFromVelocity(values.begin(), linearvel, angularvel, quatrotation, affinedofs, vaxis);
}

boost::shared_ptr<ConfigurationSpecification::SetConfigurationStateFn> ConfigurationSpecification::GetSetFn(EnvironmentBasePtr penv) const
{
    boost::shared_ptr<SetConfigurationStateFn> fn;
    Validate();
    std::vector< std::pair<SetConfigurationStateFn, int> > setstatefns(_vgroups.size());
    string bodyname;
    stringstream ss, ssout;
    // order the groups depending on offset
    int nMaxDOFForGroup = 0;
    std::vector< std::pair<int, int> > vgroupoffsets(_vgroups.size());
    for(size_t igroup = 0; igroup < _vgroups.size(); ++igroup) {
        vgroupoffsets[igroup].first = _vgroups[igroup].offset;
        vgroupoffsets[igroup].second = igroup;
        nMaxDOFForGroup = max(nMaxDOFForGroup,_vgroups[igroup].dof);
    }
    std::sort(vgroupoffsets.begin(),vgroupoffsets.end());
    for(size_t igroup = 0; igroup < _vgroups.size(); ++igroup) {
        const ConfigurationSpecification::Group& g = _vgroups[igroup];
        int isavegroup = vgroupoffsets[igroup].second;
        if( g.name.size() >= 12 && g.name.substr(0,12) == "joint_values" ) {
            ss.clear(); ss.str(g.name.substr(12));
            ss >> bodyname;
            BOOST_ASSERT(!!ss);
            KinBodyPtr pbody = penv->GetKinBody(bodyname);
            OPENRAVE_ASSERT_FORMAT(!!pbody,"body %s not found",bodyname,ORE_InvalidArguments);
            std::vector<int> dofindices((istream_iterator<int>(ss)), istream_iterator<int>());
            if( dofindices.size() == 0 ) {
                OPENRAVE_ASSERT_OP((int)dofindices.size(),==,pbody->GetDOF());
            }
            void (KinBody::*setdofvaluesptr)(const std::vector<dReal>&, uint32_t, const std::vector<int>&) = &KinBody::SetDOFValues;
            setstatefns[isavegroup].first = boost::bind(setdofvaluesptr, pbody, _1, KinBody::CLA_CheckLimits, dofindices);
            setstatefns[isavegroup].second = g.dof;
        }
        else if( g.name.size() >= 16 && g.name.substr(0,16) == "joint_velocities" ) {
            ss.clear(); ss.str(g.name.substr(12));
            ss >> bodyname;
            BOOST_ASSERT(!!ss);
            KinBodyPtr pbody = penv->GetKinBody(bodyname);
            OPENRAVE_ASSERT_FORMAT(!!pbody,"body %s not found",bodyname,ORE_InvalidArguments);
            std::vector<int> dofindices((istream_iterator<int>(ss)), istream_iterator<int>());
            if( dofindices.size() == 0 ) {
                OPENRAVE_ASSERT_OP((int)dofindices.size(),==,pbody->GetDOF());
            }
            void (KinBody::*setdofvelocitiesptr)(const std::vector<dReal>&, uint32_t, const std::vector<int>&) = &KinBody::SetDOFVelocities;
            setstatefns[isavegroup].first = boost::bind(setdofvelocitiesptr, pbody, _1, KinBody::CLA_CheckLimits, dofindices);
            setstatefns[isavegroup].second = g.dof;
        }
        else if( g.name.size() >= 16 && g.name.substr(0,16) == "affine_transform" ) {
            ss.clear(); ss.str(g.name.substr(12));
            int affinedofs=0;
            ss >> bodyname >> affinedofs;
            BOOST_ASSERT(!!ss);
            KinBodyPtr pbody = penv->GetKinBody(bodyname);
            OPENRAVE_ASSERT_FORMAT(!!pbody,"body %s not found",bodyname,ORE_InvalidArguments);
            Vector vaxis(0,0,1);
            if( affinedofs & DOF_RotationAxis ) {
                ss >> vaxis.x >> vaxis.y >> vaxis.z;
            }
            setstatefns[isavegroup].first = boost::bind(SetBodyTransformFromAffineDOFValues, _1, pbody, affinedofs, vaxis);
            setstatefns[isavegroup].second = g.dof;
        }
        else if( g.name.size() >= 17 && g.name.substr(0,17) == "affine_velocities" ) {
            ss.clear(); ss.str(g.name.substr(12));
            int affinedofs=0;
            ss >> bodyname >> affinedofs;
            BOOST_ASSERT(!!ss);
            KinBodyPtr pbody = penv->GetKinBody(bodyname);
            OPENRAVE_ASSERT_FORMAT(!!pbody,"body %s not found",bodyname,ORE_InvalidArguments);
            Vector vaxis(0,0,1);
            if( affinedofs & DOF_RotationAxis ) {
                ss >> vaxis.x >> vaxis.y >> vaxis.z;
            }
            setstatefns[isavegroup].first = boost::bind(SetBodyVelocityFromAffineDOFVelocities, _1, pbody, affinedofs, vaxis);
            setstatefns[isavegroup].second = g.dof;
        }
//        else if( g.name.size() >= 4 && g.name.substr(0,4) == "grabbody" ) {
//        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT("group %s not supported for for planner parameters configuration",g.name,ORE_InvalidArguments);
        }
    }
    fn.reset(new SetConfigurationStateFn(boost::bind(CallSetStateFns,setstatefns, GetDOF(), nMaxDOFForGroup, _1)));
    return fn;
}

boost::shared_ptr<ConfigurationSpecification::GetConfigurationStateFn> ConfigurationSpecification::GetGetFn(EnvironmentBasePtr penv) const
{
    boost::shared_ptr<GetConfigurationStateFn> fn;
    Validate();
    std::vector< std::pair<GetConfigurationStateFn, int> > getstatefns(_vgroups.size());
    string bodyname;
    stringstream ss, ssout;
    // order the groups depending on offset
    int nMaxDOFForGroup = 0;
    std::vector< std::pair<int, int> > vgroupoffsets(_vgroups.size());
    for(size_t igroup = 0; igroup < _vgroups.size(); ++igroup) {
        vgroupoffsets[igroup].first = _vgroups[igroup].offset;
        vgroupoffsets[igroup].second = igroup;
        nMaxDOFForGroup = max(nMaxDOFForGroup,_vgroups[igroup].dof);
    }
    std::sort(vgroupoffsets.begin(),vgroupoffsets.end());
    for(size_t igroup = 0; igroup < _vgroups.size(); ++igroup) {
        const ConfigurationSpecification::Group& g = _vgroups[igroup];
        int isavegroup = vgroupoffsets[igroup].second;
        if( g.name.size() >= 12 && g.name.substr(0,12) == "joint_values" ) {
            ss.clear(); ss.str(g.name.substr(12));
            ss >> bodyname;
            BOOST_ASSERT(!!ss);
            KinBodyPtr pbody = penv->GetKinBody(bodyname);
            OPENRAVE_ASSERT_FORMAT(!!pbody,"body %s not found",bodyname,ORE_InvalidArguments);
            std::vector<int> dofindices((istream_iterator<int>(ss)), istream_iterator<int>());
            if( dofindices.size() == 0 ) {
                OPENRAVE_ASSERT_OP((int)dofindices.size(),==,pbody->GetDOF());
            }
            void (KinBody::*getdofvaluesptr)(std::vector<dReal>&, const std::vector<int>&) const = &KinBody::GetDOFValues;
            getstatefns[isavegroup].first = boost::bind(getdofvaluesptr, pbody, _1, dofindices);
            getstatefns[isavegroup].second = g.dof;
        }
        else if( g.name.size() >= 16 && g.name.substr(0,16) == "joint_velocities" ) {
            ss.clear(); ss.str(g.name.substr(12));
            ss >> bodyname;
            BOOST_ASSERT(!!ss);
            KinBodyPtr pbody = penv->GetKinBody(bodyname);
            OPENRAVE_ASSERT_FORMAT(!!pbody,"body %s not found",bodyname,ORE_InvalidArguments);
            std::vector<int> dofindices((istream_iterator<int>(ss)), istream_iterator<int>());
            if( dofindices.size() == 0 ) {
                OPENRAVE_ASSERT_OP((int)dofindices.size(),==,pbody->GetDOF());
            }
            void (KinBody::*getdofvelocitiesptr)(std::vector<dReal>&, const std::vector<int>&) const = &KinBody::GetDOFVelocities;
            getstatefns[isavegroup].first = boost::bind(getdofvelocitiesptr, pbody, _1, dofindices);
            getstatefns[isavegroup].second = g.dof;
        }
        else if( g.name.size() >= 16 && g.name.substr(0,16) == "affine_transform" ) {
            ss.clear(); ss.str(g.name.substr(12));
            int affinedofs=0;
            ss >> bodyname >> affinedofs;
            BOOST_ASSERT(!!ss);
            KinBodyPtr pbody = penv->GetKinBody(bodyname);
            OPENRAVE_ASSERT_FORMAT(!!pbody,"body %s not found",bodyname,ORE_InvalidArguments);
            Vector vaxis(0,0,1);
            if( affinedofs & DOF_RotationAxis ) {
                ss >> vaxis.x >> vaxis.y >> vaxis.z;
            }
            getstatefns[isavegroup].first = boost::bind(GetAffineDOFValuesFromBodyTransform, _1, pbody, affinedofs, vaxis);
            getstatefns[isavegroup].second = g.dof;
        }
        else if( g.name.size() >= 17 && g.name.substr(0,17) == "affine_velocities" ) {
            ss.clear(); ss.str(g.name.substr(12));
            int affinedofs=0;
            ss >> bodyname >> affinedofs;
            BOOST_ASSERT(!!ss);
            KinBodyPtr pbody = penv->GetKinBody(bodyname);
            OPENRAVE_ASSERT_FORMAT(!!pbody,"body %s not found",bodyname,ORE_InvalidArguments);
            Vector vaxis(0,0,1);
            if( affinedofs & DOF_RotationAxis ) {
                ss >> vaxis.x >> vaxis.y >> vaxis.z;
            }
            getstatefns[isavegroup].first = boost::bind(GetAffineDOFVelocitiesFromBodyVelocity, _1, pbody, affinedofs, vaxis);
            getstatefns[isavegroup].second = g.dof;
        }
//        else if( g.name.size() >= 4 && g.name.substr(0,4) == "grabbody" ) {
//        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT("group %s not supported for for planner parameters configuration",g.name,ORE_InvalidArguments);
        }
    }
    fn.reset(new GetConfigurationStateFn(boost::bind(CallGetStateFns,getstatefns, GetDOF(), nMaxDOFForGroup, _1)));
    return fn;
}


static void ConvertDOFRotation_AxisFrom3D(std::vector<dReal>::iterator ittarget, std::vector<dReal>::const_iterator itsource, const Vector& vaxis)
{
    Vector axisangle(*(itsource+0),*(itsource+1),*(itsource+2));
    *ittarget = normalizeAxisRotation(vaxis,quatFromAxisAngle(axisangle)).first;
}

static void ConvertDOFRotation_AxisFromQuat(std::vector<dReal>::iterator ittarget, std::vector<dReal>::const_iterator itsource, const Vector& vaxis)
{
    Vector quat(*(itsource+0),*(itsource+1),*(itsource+2),*(itsource+3));
    *ittarget = normalizeAxisRotation(vaxis,quat).first;
}

static void ConvertDOFRotation_3DFromAxis(std::vector<dReal>::iterator ittarget, std::vector<dReal>::const_iterator itsource, const Vector& vaxis)
{
    *(ittarget+0) = vaxis[0]* *itsource;
    *(ittarget+1) = vaxis[1]* *itsource;
    *(ittarget+2) = vaxis[2]* *itsource;
}
static void ConvertDOFRotation_3DFromQuat(std::vector<dReal>::iterator ittarget, std::vector<dReal>::const_iterator itsource)
{
    Vector quat(*(itsource+0),*(itsource+1),*(itsource+2),*(itsource+3));
    Vector axisangle = quatFromAxisAngle(quat);
    *(ittarget+0) = axisangle[0];
    *(ittarget+1) = axisangle[1];
    *(ittarget+2) = axisangle[2];
}
static void ConvertDOFRotation_QuatFromAxis(std::vector<dReal>::iterator ittarget, std::vector<dReal>::const_iterator itsource, const Vector& vaxis)
{
    Vector axisangle = vaxis * *itsource;
    Vector quat = quatFromAxisAngle(axisangle);
    *(ittarget+0) = quat[0];
    *(ittarget+1) = quat[1];
    *(ittarget+2) = quat[2];
    *(ittarget+3) = quat[3];
}

static void ConvertDOFRotation_QuatFrom3D(std::vector<dReal>::iterator ittarget, std::vector<dReal>::const_iterator itsource)
{
    Vector axisangle(*(itsource+0),*(itsource+1),*(itsource+2));
    Vector quat = quatFromAxisAngle(axisangle);
    *(ittarget+0) = quat[0];
    *(ittarget+1) = quat[1];
    *(ittarget+2) = quat[2];
    *(ittarget+3) = quat[3];
}

void ConfigurationSpecification::ConvertGroupData(std::vector<dReal>::iterator ittargetdata, size_t targetstride, const ConfigurationSpecification::Group& gtarget, std::vector<dReal>::const_iterator itsourcedata, size_t sourcestride, const ConfigurationSpecification::Group& gsource, size_t numpoints, EnvironmentBaseConstPtr penv, bool filluninitialized)
{
    if( numpoints > 1 ) {
        BOOST_ASSERT(targetstride != 0 && sourcestride != 0 );
    }
    if( gsource.name == gtarget.name ) {
        BOOST_ASSERT(gsource.dof==gtarget.dof);
        for(size_t i = 0; i < numpoints; ++i) {
            if( i != 0 ) {
                itsourcedata += sourcestride;
                ittargetdata += targetstride;
            }
            std::copy(itsourcedata,itsourcedata+gsource.dof,ittargetdata);
        }
    }
    else {
        stringstream ss(gtarget.name);
        std::vector<std::string> targettokens((istream_iterator<std::string>(ss)), istream_iterator<std::string>());
        ss.clear();
        ss.str(gsource.name);
        std::vector<std::string> sourcetokens((istream_iterator<std::string>(ss)), istream_iterator<std::string>());

        BOOST_ASSERT(targettokens.at(0) == sourcetokens.at(0));
        vector<int> vtransferindices; vtransferindices.reserve(gtarget.dof);
        std::vector<dReal> vdefaultvalues;
        if( targettokens.at(0).size() >= 6 && targettokens.at(0).substr(0,6) == "joint_") {
            std::vector<int> vsourceindices(gsource.dof), vtargetindices(gtarget.dof);
            if( (int)sourcetokens.size() < gsource.dof+2 ) {
                RAVELOG_DEBUG(str(boost::format("source tokens '%s' do not have %d dof indices, guessing....")%gsource.name%gsource.dof));
                for(int i = 0; i < gsource.dof; ++i) {
                    vsourceindices[i] = i;
                }
            }
            else {
                for(int i = 0; i < gsource.dof; ++i) {
                    vsourceindices[i] = boost::lexical_cast<int>(sourcetokens.at(i+2));
                }
            }
            if( (int)targettokens.size() < gtarget.dof+2 ) {
                RAVELOG_WARN(str(boost::format("target tokens '%s' do not match dof '%d', guessing....")%gtarget.name%gtarget.dof));
                for(int i = 0; i < gtarget.dof; ++i) {
                    vtargetindices[i] = i;
                }
            }
            else {
                for(int i = 0; i < gtarget.dof; ++i) {
                    vtargetindices[i] = boost::lexical_cast<int>(targettokens.at(i+2));
                }
            }

            bool bUninitializedData=false;
            FOREACH(ittargetindex,vtargetindices) {
                std::vector<int>::iterator it = find(vsourceindices.begin(),vsourceindices.end(),*ittargetindex);
                if( it == vsourceindices.end() ) {
                    bUninitializedData = true;
                    vtransferindices.push_back(-1);
                }
                else {
                    vtransferindices.push_back(static_cast<int>(it-vsourceindices.begin()));
                }
            }

            if( bUninitializedData && filluninitialized ) {
                KinBodyPtr pbody;
                if( targettokens.size() > 1 ) {
                    pbody = penv->GetKinBody(targettokens.at(1));
                }
                if( !pbody && sourcetokens.size() > 1 ) {
                    pbody = penv->GetKinBody(sourcetokens.at(1));
                }
                if( !pbody ) {
                    RAVELOG_WARN(str(boost::format("could not find body '%s' or '%s'")%gtarget.name%gsource.name));
                    vdefaultvalues.resize(vtargetindices.size(),0);
                }
                else {
                    std::vector<dReal> vbodyvalues;
                    vdefaultvalues.resize(vtargetindices.size(),0);
                    if( targettokens[0] == "joint_values" ) {
                        pbody->GetDOFValues(vbodyvalues);
                    }
                    else if( targettokens[0] == "joint_velocities" ) {
                        pbody->GetDOFVelocities(vbodyvalues);
                    }
                    if( vbodyvalues.size() > 0 ) {
                        for(size_t i = 0; i < vdefaultvalues.size(); ++i) {
                            vdefaultvalues[i] = vbodyvalues.at(vtargetindices[i]);
                        }
                    }
                }
            }
        }
        else if( targettokens.at(0).size() >= 7 && targettokens.at(0).substr(0,7) == "affine_") {
            int affinesource = 0, affinetarget = 0;
            Vector sourceaxis(0,0,1), targetaxis(0,0,1);
            if( sourcetokens.size() < 3 ) {
                if( targettokens.size() < 3 && gsource.dof == gtarget.dof ) {
                    for(size_t i = 0; i < targettokens.size(); ++i) {
                        vtransferindices[i] = i;
                    }
                }
                else {
                    throw OPENRAVE_EXCEPTION_FORMAT("source affine information not present '%s'\n",gsource.name,ORE_InvalidArguments);
                }
            }
            else {
                affinesource = boost::lexical_cast<int>(sourcetokens.at(2));
                BOOST_ASSERT(RaveGetAffineDOF(affinesource) == gsource.dof);
                if( (affinesource & DOF_RotationAxis) && sourcetokens.size() >= 6 ) {
                    sourceaxis.x = boost::lexical_cast<dReal>(sourcetokens.at(3));
                    sourceaxis.y = boost::lexical_cast<dReal>(sourcetokens.at(4));
                    sourceaxis.z = boost::lexical_cast<dReal>(sourcetokens.at(5));
                }
            }
            if( vtransferindices.size() == 0 ) {
                if( targettokens.size() < 3 ) {
                    throw OPENRAVE_EXCEPTION_FORMAT("target affine information not present '%s'\n",gtarget.name,ORE_InvalidArguments);
                }
                else {
                    affinetarget = boost::lexical_cast<int>(targettokens.at(2));
                    BOOST_ASSERT(RaveGetAffineDOF(affinetarget) == gtarget.dof);
                    if( (affinetarget & DOF_RotationAxis) && targettokens.size() >= 6 ) {
                        targetaxis.x = boost::lexical_cast<dReal>(targettokens.at(3));
                        targetaxis.y = boost::lexical_cast<dReal>(targettokens.at(4));
                        targetaxis.z = boost::lexical_cast<dReal>(targettokens.at(5));
                    }
                }

                int commondata = affinesource&affinetarget;
                int uninitdata = affinetarget&(~commondata);
                int sourcerotationstart = -1, targetrotationstart = -1, targetrotationend = -1;
                boost::function< void(std::vector<dReal>::iterator, std::vector<dReal>::const_iterator) > rotconverterfn;
                if( (uninitdata & DOF_RotationMask) && (affinetarget & DOF_RotationMask) && (affinesource & DOF_RotationMask) ) {
                    // both hold rotations, but need to convert
                    uninitdata &= ~DOF_RotationMask;
                    sourcerotationstart = RaveGetIndexFromAffineDOF(affinesource,DOF_RotationMask);
                    targetrotationstart = RaveGetIndexFromAffineDOF(affinetarget,DOF_RotationMask);
                    targetrotationend = targetrotationstart+RaveGetAffineDOF(affinetarget&DOF_RotationMask);
                    if( affinetarget & DOF_RotationAxis ) {
                        if( affinesource & DOF_Rotation3D ) {
                            rotconverterfn = boost::bind(ConvertDOFRotation_AxisFrom3D,_1,_2,targetaxis);
                        }
                        else if( affinesource & DOF_RotationQuat ) {
                            rotconverterfn = boost::bind(ConvertDOFRotation_AxisFromQuat,_1,_2,targetaxis);
                        }
                    }
                    else if( affinetarget & DOF_Rotation3D ) {
                        if( affinesource & DOF_RotationAxis ) {
                            rotconverterfn = boost::bind(ConvertDOFRotation_3DFromAxis,_1,_2,sourceaxis);
                        }
                        else if( affinesource & DOF_RotationQuat ) {
                            rotconverterfn = ConvertDOFRotation_3DFromQuat;
                        }
                    }
                    else if( affinetarget & DOF_RotationQuat ) {
                        if( affinesource & DOF_RotationAxis ) {
                            rotconverterfn = boost::bind(ConvertDOFRotation_QuatFromAxis,_1,_2,sourceaxis);
                        }
                        else if( affinesource & DOF_Rotation3D ) {
                            rotconverterfn = ConvertDOFRotation_QuatFrom3D;
                        }
                    }
                    BOOST_ASSERT(!!rotconverterfn);
                }
                if( uninitdata && filluninitialized ) {
                    // initialize with the current body values
                    KinBodyPtr pbody;
                    if( targettokens.size() > 1 ) {
                        pbody = penv->GetKinBody(targettokens.at(1));
                    }
                    if( !pbody && sourcetokens.size() > 1 ) {
                        pbody = penv->GetKinBody(sourcetokens.at(1));
                    }
                    if( !pbody ) {
                        RAVELOG_WARN(str(boost::format("could not find body '%s' or '%s'")%gtarget.name%gsource.name));
                        vdefaultvalues.resize(gtarget.dof,0);
                    }
                    else {
                        vdefaultvalues.resize(gtarget.dof);
                        RaveGetAffineDOFValuesFromTransform(vdefaultvalues.begin(),pbody->GetTransform(),affinetarget);
                    }
                }

                for(int index = 0; index < gtarget.dof; ++index) {
                    DOFAffine dof = RaveGetAffineDOFFromIndex(affinetarget,index);
                    int startindex = RaveGetIndexFromAffineDOF(affinetarget,dof);
                    if( affinesource & dof ) {
                        int sourceindex = RaveGetIndexFromAffineDOF(affinesource,dof);
                        vtransferindices.push_back(sourceindex + (index-startindex));
                    }
                    else {
                        vtransferindices.push_back(-1);
                    }
                }

                for(size_t i = 0; i < numpoints; ++i, itsourcedata += sourcestride, ittargetdata += targetstride) {
                    for(int j = 0; j < (int)vtransferindices.size(); ++j) {
                        if( vtransferindices[j] >= 0 ) {
                            *(ittargetdata+j) = *(itsourcedata+vtransferindices[j]);
                        }
                        else {
                            if( j >= targetrotationstart && j < targetrotationend ) {
                                if( j == targetrotationstart ) {
                                    // only convert when at first index
                                    rotconverterfn(ittargetdata+targetrotationstart,itsourcedata+sourcerotationstart);
                                }
                            }
                            else if( filluninitialized ) {
                                *(ittargetdata+j) = vdefaultvalues.at(j);
                            }
                        }
                    }
                }
                return;
            }
        }
        else if( targettokens.at(0).size() >= 8 && targettokens.at(0).substr(0,8) == "ikparam_") {
            IkParameterizationType iktypesource, iktypetarget;
            if( sourcetokens.size() >= 2 ) {
                iktypesource = static_cast<IkParameterizationType>(boost::lexical_cast<int>(sourcetokens[1]));
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT("ikparam type not present '%s'\n",gsource.name,ORE_InvalidArguments);
            }
            if( targettokens.size() >= 2 ) {
                iktypetarget = static_cast<IkParameterizationType>(boost::lexical_cast<int>(targettokens[1]));
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT("ikparam type not present '%s'\n",gtarget.name,ORE_InvalidArguments);
            }

            if( iktypetarget == iktypesource ) {
                vtransferindices.resize(IkParameterization::GetDOF(iktypetarget));
                for(size_t i = 0; i < vtransferindices.size(); ++i) {
                    vtransferindices[i] = i;
                }
            }
            else {
                RAVELOG_WARN("ikparam types do not match");
            }
        }
        else if( targettokens.at(0).size() >= 4 && targettokens.at(0).substr(0,4) == "grab") {
            std::vector<int> vsourceindices(gsource.dof), vtargetindices(gtarget.dof);
            if( (int)sourcetokens.size() < gsource.dof+2 ) {
                throw OPENRAVE_EXCEPTION_FORMAT("source tokens '%s' do not have %d dof indices, guessing....", gsource.name%gsource.dof, ORE_InvalidArguments);
            }
            else {
                for(int i = 0; i < gsource.dof; ++i) {
                    vsourceindices[i] = boost::lexical_cast<int>(sourcetokens.at(i+2));
                }
            }
            if( (int)targettokens.size() < gtarget.dof+2 ) {
                throw OPENRAVE_EXCEPTION_FORMAT("target tokens '%s' do not match dof '%d', guessing....", gtarget.name%gtarget.dof, ORE_InvalidArguments);
            }
            else {
                for(int i = 0; i < gtarget.dof; ++i) {
                    vtargetindices[i] = boost::lexical_cast<int>(targettokens.at(i+2));
                }
            }

            bool bUninitializedData=false;
            FOREACH(ittargetindex,vtargetindices) {
                std::vector<int>::iterator it = find(vsourceindices.begin(),vsourceindices.end(),*ittargetindex);
                if( it == vsourceindices.end() ) {
                    bUninitializedData = true;
                    vtransferindices.push_back(-1);
                }
                else {
                    vtransferindices.push_back(static_cast<int>(it-vsourceindices.begin()));
                }
            }

            if( bUninitializedData && filluninitialized ) {
                vdefaultvalues.resize(vtargetindices.size(),0);
            }
        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT("unsupported token conversion: %s",gtarget.name,ORE_InvalidArguments);
        }

        for(size_t i = 0; i < numpoints; ++i, itsourcedata += sourcestride, ittargetdata += targetstride) {
            for(size_t j = 0; j < vtransferindices.size(); ++j) {
                if( vtransferindices[j] >= 0 ) {
                    *(ittargetdata+j) = *(itsourcedata+vtransferindices[j]);
                }
                else if( filluninitialized ) {
                    *(ittargetdata+j) = vdefaultvalues.at(j);
                }
            }
        }
    }
}

void ConfigurationSpecification::ConvertData(std::vector<dReal>::iterator ittargetdata, const ConfigurationSpecification &targetspec, std::vector<dReal>::const_iterator itsourcedata, const ConfigurationSpecification &sourcespec, size_t numpoints, EnvironmentBaseConstPtr penv, bool filluninitialized)
{
    for(size_t igroup = 0; igroup < targetspec._vgroups.size(); ++igroup) {
        std::vector<ConfigurationSpecification::Group>::const_iterator itcompatgroup = sourcespec.FindCompatibleGroup(targetspec._vgroups[igroup]);
        if( itcompatgroup != sourcespec._vgroups.end() ) {
            ConfigurationSpecification::ConvertGroupData(ittargetdata+targetspec._vgroups[igroup].offset, targetspec.GetDOF(), targetspec._vgroups[igroup], itsourcedata+itcompatgroup->offset, sourcespec.GetDOF(), *itcompatgroup,numpoints,penv,filluninitialized);
        }
        else if( filluninitialized ) {
            vector<dReal> vdefaultvalues(targetspec._vgroups[igroup].dof,0);
            const string& name = targetspec._vgroups[igroup].name;
            if( name.size() >= 12 && name.substr(0,12) == "joint_values" ) {
                string bodyname;
                stringstream ss(name.substr(12));
                ss >> bodyname;
                if( !!ss ) {
                    if( !!penv ) {
                        KinBodyPtr body = penv->GetKinBody(bodyname);
                        if( !!body ) {
                            vector<dReal> values;
                            body->GetDOFValues(values);
                            std::vector<int> indices((istream_iterator<int>(ss)), istream_iterator<int>());
                            for(size_t i = 0; i < indices.size(); ++i) {
                                vdefaultvalues.at(i) = values.at(indices[i]);
                            }
                        }
                    }
                }
            }
            else if( name.size() >= 16 && name.substr(0,16) == "affine_transform" ) {
                string bodyname;
                int affinedofs;
                stringstream ss(name.substr(16));
                ss >> bodyname >> affinedofs;
                if( !!ss ) {
                    Transform tdefault;
                    if( !!penv ) {
                        KinBodyPtr body = penv->GetKinBody(bodyname);
                        if( !!body ) {
                            tdefault = body->GetTransform();
                        }
                    }
                    BOOST_ASSERT((int)vdefaultvalues.size() == RaveGetAffineDOF(affinedofs));
                    RaveGetAffineDOFValuesFromTransform(vdefaultvalues.begin(),tdefault,affinedofs);
                }
            }
            else if( name != "deltatime" ) {
                // messages are too frequent
                //RAVELOG_VERBOSE(str(boost::format("cannot initialize unknown group '%s'")%name));
            }
            int offset = targetspec._vgroups[igroup].offset;
            for(size_t i = 0; i < numpoints; ++i, offset += targetspec.GetDOF()) {
                for(size_t j = 0; j < vdefaultvalues.size(); ++j) {
                    *(ittargetdata+offset+j) = vdefaultvalues[j];
                }
            }
        }
    }
}

std::string ConfigurationSpecification::GetInterpolationDerivative(const std::string& interpolation, int deriv)
{
    const static boost::array<std::string,6> s_InterpolationOrder = {{"next","linear","quadratic","cubic","quadric","quintic"}};
    for(int i = 0; i < (int)s_InterpolationOrder.size(); ++i) {
        if( interpolation == s_InterpolationOrder[i] ) {
            if( i < deriv ) {
                return s_InterpolationOrder.at(0);
            }
            else {
                return s_InterpolationOrder.at(i-deriv);
            }
        }
    }
    return "";
}

ConfigurationSpecification::Reader::Reader(ConfigurationSpecification& spec) : _spec(spec)
{
    _spec = ConfigurationSpecification(); // reset
}

BaseXMLReader::ProcessElement ConfigurationSpecification::Reader::startElement(const std::string& name, const AttributesList &atts)
{
    if( !!_preader ) {
        if( _preader->startElement(name, atts) == PE_Support )
            return PE_Support;
        return PE_Ignore;
    }
    _ss.str(""); // have to clear the string
    if( name == "group" ) {
        _spec._vgroups.resize(_spec._vgroups.size()+1);
        ConfigurationSpecification::Group& g = _spec._vgroups.back();
        FOREACHC(itatt,atts) {
            if( itatt->first == "name" ) {
                g.name = itatt->second;
            }
            else if( itatt->first == "interpolation" ) {
                g.interpolation = itatt->second;
            }
            else if( itatt->first == "offset" ) {
                g.offset = boost::lexical_cast<int>(itatt->second);
            }
            else if( itatt->first == "dof" ) {
                g.dof = boost::lexical_cast<int>(itatt->second);
            }
        }
        return PE_Support;
    }
    else if( name == "configuration" ) {
        _preader.reset(new ConfigurationSpecification::Reader(_spec));
        return PE_Support;
    }
    return PE_Pass;
}

bool ConfigurationSpecification::Reader::endElement(const std::string& name)
{
    if( !!_preader ) {
        if( _preader->endElement(name) ) {
            _preader.reset();
        }
        return false;
    }
    if( name == "configuration" ) {
        return true;
    }
    return false;
}

void ConfigurationSpecification::Reader::characters(const std::string& ch)
{
    if( !_preader ) {
        _ss.clear();
        _ss << ch;
    }
    else {
        _preader->characters(ch);
    }
}

std::ostream& operator<<(std::ostream& O, const ConfigurationSpecification &spec)
{
    O << "<configuration>" << endl;
    FOREACHC(it,spec._vgroups) {
        O << "<group name=\"" << it->name << "\" offset=\"" << it->offset << "\" dof=\"" << it->dof << "\" interpolation=\"" << it->interpolation << "\"/>" << endl;
    }
    O << "</configuration>" << endl;
    return O;
}

std::istream& operator>>(std::istream& I, ConfigurationSpecification& spec)
{
    if( !!I) {
        stringbuf buf;
        stringstream::streampos pos = I.tellg();
        I.get(buf, 0); // get all the data, yes this is inefficient, not sure if there anyway to search in streams

        string pbuf = buf.str();
        const char* p = strcasestr(pbuf.c_str(), "</configuration>");
        int ppsize=-1;
        if( p != NULL ) {
            I.clear();
            ppsize=(p-pbuf.c_str())+20;
            I.seekg((size_t)pos+ppsize);
        }
        else {
            throw OPENRAVE_EXCEPTION_FORMAT("error, failed to find </configuration> in %s",buf.str(),ORE_InvalidArguments);
        }
        ConfigurationSpecification::Reader reader(spec);
        LocalXML::ParseXMLData(BaseXMLReaderPtr(&reader,utils::null_deleter()), pbuf.c_str(), ppsize);
        BOOST_ASSERT(spec.IsValid());
    }

    return I;
}

}
