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
#include "ravep.h"
#include <boost/lambda/lambda.hpp>
#include <boost/lexical_cast.hpp>
#include <openrave/xmlreaders.h>

namespace OpenRAVE {

static const dReal g_fEpsilonLinear = RavePow(g_fEpsilon,0.9);
static const dReal g_fEpsilonQuadratic = RavePow(g_fEpsilon,0.45); // should be 0.6...perhaps this is related to parabolic smoother epsilons?

class GenericTrajectory : public TrajectoryBase
{
    std::map<string,int> _maporder;
public:
    GenericTrajectory(EnvironmentBasePtr penv, std::istream& sinput) : TrajectoryBase(penv), _timeoffset(-1)
    {
        _maporder["deltatime"] = 0;
        _maporder["joint_snaps"] = 1;
        _maporder["affine_snaps"] = 2;
        _maporder["joint_jerks"] = 3;
        _maporder["affine_jerks"] = 4;
        _maporder["joint_accelerations"] = 5;
        _maporder["affine_accelerations"] = 6;
        _maporder["joint_velocities"] = 7;
        _maporder["affine_velocities"] = 8;
        _maporder["joint_values"] = 9;
        _maporder["affine_transform"] = 10;
        _maporder["joint_torques"] = 11;
        _bInit = false;
        _bSamplingVerified = false;
    }

    bool SortGroups(const ConfigurationSpecification::Group& g1, const ConfigurationSpecification::Group& g2)
    {
        size_t index1 = g1.name.find_first_of(' ');
        if( index1 == string::npos ) {
            index1 = g1.name.size();
        }
        size_t index2 = g2.name.find_first_of(' ');
        if( index2 == string::npos ) {
            index2 = g2.name.size();
        }
        std::map<string,int>::iterator it1 = _maporder.find(g1.name.substr(0,index1));
        std::map<string,int>::iterator it2 = _maporder.find(g2.name.substr(0,index2));
        if( it1 == _maporder.end() ) {
            return it2 == _maporder.end();
        }
        if( it2 == _maporder.end()) {
            return true;
        }
        return it1->second < it2->second;
    }

    void Init(const ConfigurationSpecification& spec)
    {
        if( _bInit  && _spec == spec ) {
            // already init
        }
        else {
            BOOST_ASSERT(spec.GetDOF()>0 && spec.IsValid());
            _bInit = false;
            _vgroupinterpolators.resize(0);
            _vgroupvalidators.resize(0);
            _vderivoffsets.resize(0);
            _vddoffsets.resize(0);
            _vdddoffsets.resize(0);
            _vintegraloffsets.resize(0);
            _spec = spec;
            // order the groups based on computation order
            stable_sort(_spec._vgroups.begin(),_spec._vgroups.end(),boost::bind(&GenericTrajectory::SortGroups,this,_1,_2));
            _timeoffset = -1;
            FOREACH(itgroup,_spec._vgroups) {
                if( itgroup->name == "deltatime" ) {
                    _timeoffset = itgroup->offset;
                }
            }
            _InitializeGroupFunctions();
        }
        _vtrajdata.resize(0);
        _vaccumtime.resize(0);
        _vdeltainvtime.resize(0);
        _bChanged = true;
        _bSamplingVerified = false;
        _bInit = true;
    }

    void Insert(size_t index, const std::vector<dReal>& data, bool bOverwrite)
    {
        BOOST_ASSERT(_bInit);
        if( data.size() == 0 ) {
            return;
        }
        BOOST_ASSERT(_spec.GetDOF()>0);
        OPENRAVE_ASSERT_FORMAT((data.size()%_spec.GetDOF()) == 0, "%d does not divide dof %d", data.size()%_spec.GetDOF(), ORE_InvalidArguments);
        OPENRAVE_ASSERT_OP(index*_spec.GetDOF(),<=,_vtrajdata.size());
        if( bOverwrite && index*_spec.GetDOF() < _vtrajdata.size() ) {
            size_t copysize = min(data.size(),_vtrajdata.size()-index*_spec.GetDOF());
            std::copy(data.begin(),data.begin()+copysize,_vtrajdata.begin()+index*_spec.GetDOF());
            if( copysize < data.size() ) {
                _vtrajdata.insert(_vtrajdata.end(),data.begin()+copysize,data.end());
            }
        }
        else {
            _vtrajdata.insert(_vtrajdata.begin()+index*_spec.GetDOF(),data.begin(),data.end());
        }
        _bChanged = true;
    }

    void Insert(size_t index, const std::vector<dReal>& data, const ConfigurationSpecification& spec, bool bOverwrite)
    {
        BOOST_ASSERT(_bInit);
        if( data.size() == 0 ) {
            return;
        }
        BOOST_ASSERT(spec.GetDOF()>0);
        OPENRAVE_ASSERT_FORMAT((data.size()%spec.GetDOF()) == 0, "%d does not divide dof %d", data.size()%spec.GetDOF(), ORE_InvalidArguments);
        OPENRAVE_ASSERT_OP(index*_spec.GetDOF(),<=,_vtrajdata.size());
        if( _spec == spec ) {
            Insert(index,data,bOverwrite);
        }
        else {
            std::vector< std::vector<ConfigurationSpecification::Group>::const_iterator > vconvertgroups(_spec._vgroups.size());
            for(size_t i = 0; i < vconvertgroups.size(); ++i) {
                vconvertgroups[i] = spec.FindCompatibleGroup(_spec._vgroups[i]);
            }
            size_t numpoints = data.size()/spec.GetDOF();
            size_t sourceindex = 0;
            std::vector<dReal>::iterator ittargetdata;
            std::vector<dReal>::const_iterator itsourcedata;
            if( bOverwrite && index*_spec.GetDOF() < _vtrajdata.size() ) {
                size_t copyelements = min(numpoints,_vtrajdata.size()/_spec.GetDOF()-index);
                ittargetdata = _vtrajdata.begin()+index*_spec.GetDOF();
                itsourcedata = data.begin();
                _ConvertData(ittargetdata,itsourcedata,vconvertgroups,spec,copyelements,false);
                sourceindex = copyelements*spec.GetDOF();
                index += copyelements;
            }
            if( sourceindex < data.size() ) {
                size_t numelements = (data.size()-sourceindex)/spec.GetDOF();
                std::vector<dReal> vtemp(numelements*_spec.GetDOF());
                ittargetdata = vtemp.begin();
                itsourcedata = data.begin()+sourceindex;
                _ConvertData(ittargetdata,itsourcedata,vconvertgroups,spec,numelements,true);
                _vtrajdata.insert(_vtrajdata.begin()+index*_spec.GetDOF(),vtemp.begin(),vtemp.end());
            }
            _bChanged = true;
        }
    }

    void Remove(size_t startindex, size_t endindex)
    {
        BOOST_ASSERT(_bInit);
        if( startindex == endindex ) {
            return;
        }
        BOOST_ASSERT(startindex*_spec.GetDOF() <= _vtrajdata.size() && endindex*_spec.GetDOF() <= _vtrajdata.size());
        OPENRAVE_ASSERT_OP(startindex,<,endindex);
        _vtrajdata.erase(_vtrajdata.begin()+startindex*_spec.GetDOF(),_vtrajdata.begin()+endindex*_spec.GetDOF());
        _bChanged = true;
    }

    void Sample(std::vector<dReal>& data, dReal time) const
    {
        BOOST_ASSERT(_bInit);
        BOOST_ASSERT(_timeoffset>=0);
        BOOST_ASSERT(time >= 0);
        _ComputeInternal();
        if( IS_DEBUGLEVEL(Level_Verbose) || (RaveGetDebugLevel() & Level_VerifyPlans) ) {
            _VerifySampling();
        }
        data.resize(0);
        data.resize(_spec.GetDOF(),0);
        if( time >= GetDuration() ) {
            std::copy(_vtrajdata.end()-_spec.GetDOF(),_vtrajdata.end(),data.begin());
        }
        else {
            std::vector<dReal>::iterator it = std::lower_bound(_vaccumtime.begin(),_vaccumtime.end(),time);
            if( it == _vaccumtime.begin() ) {
                std::copy(_vtrajdata.begin(),_vtrajdata.begin()+_spec.GetDOF(),data.begin());
            }
            else {
                size_t index = it-_vaccumtime.begin();
                dReal deltatime = time-_vaccumtime.at(index-1);
                for(size_t i = 0; i < _vgroupinterpolators.size(); ++i) {
                    if( !!_vgroupinterpolators[i] ) {
                        _vgroupinterpolators[i](index-1,deltatime,data);
                    }
                }
                // should return the sample time relative to the last endpoint so it is easier to re-insert in the trajectory
                data.at(_timeoffset) = deltatime;
            }
        }
    }

    void Sample(std::vector<dReal>& data, dReal time, const ConfigurationSpecification& spec) const
    {
        BOOST_ASSERT(_bInit);
        BOOST_ASSERT(_timeoffset>=0);
        BOOST_ASSERT(time >= -g_fEpsilon);
        _ComputeInternal();
        if( IS_DEBUGLEVEL(Level_Verbose) || (RaveGetDebugLevel() & Level_VerifyPlans) ) {
            _VerifySampling();
        }
        data.resize(0);
        data.resize(spec.GetDOF(),0);
        if( time >= GetDuration() ) {
            ConfigurationSpecification::ConvertData(data.begin(),spec,_vtrajdata.end()-_spec.GetDOF(),_spec,1,GetEnv());
        }
        else {
            std::vector<dReal>::iterator it = std::lower_bound(_vaccumtime.begin(),_vaccumtime.end(),time);
            if( it == _vaccumtime.begin() ) {
                ConfigurationSpecification::ConvertData(data.begin(),spec,_vtrajdata.begin(),_spec,1,GetEnv());
            }
            else {
                // could be faster
                vector<dReal> vinternaldata(_spec.GetDOF(),0);
                size_t index = it-_vaccumtime.begin();
                dReal deltatime = time-_vaccumtime.at(index-1);
                for(size_t i = 0; i < _vgroupinterpolators.size(); ++i) {
                    if( !!_vgroupinterpolators[i] ) {
                        _vgroupinterpolators[i](index-1,deltatime,vinternaldata);
                    }
                }
                ConfigurationSpecification::ConvertData(data.begin(),spec,vinternaldata.begin(),_spec,1,GetEnv());
            }
        }
    }

    const ConfigurationSpecification& GetConfigurationSpecification() const
    {
        return _spec;
    }

    size_t GetNumWaypoints() const
    {
        BOOST_ASSERT(_bInit);
        return _vtrajdata.size()/_spec.GetDOF();
    }

    void GetWaypoints(size_t startindex, size_t endindex, std::vector<dReal>& data) const
    {
        BOOST_ASSERT(_bInit);
        BOOST_ASSERT(startindex<=endindex && startindex*_spec.GetDOF() <= _vtrajdata.size() && endindex*_spec.GetDOF() <= _vtrajdata.size());
        data.resize((endindex-startindex)*_spec.GetDOF(),0);
        std::copy(_vtrajdata.begin()+startindex*_spec.GetDOF(),_vtrajdata.begin()+endindex*_spec.GetDOF(),data.begin());
    }

    void GetWaypoints(size_t startindex, size_t endindex, std::vector<dReal>& data, const ConfigurationSpecification& spec) const
    {
        BOOST_ASSERT(_bInit);
        BOOST_ASSERT(startindex<=endindex && startindex*_spec.GetDOF() <= _vtrajdata.size() && endindex*_spec.GetDOF() <= _vtrajdata.size());
        data.resize(spec.GetDOF()*(endindex-startindex),0);
        if( startindex < endindex ) {
            ConfigurationSpecification::ConvertData(data.begin(),spec,_vtrajdata.begin()+startindex*_spec.GetDOF(),_spec,endindex-startindex,GetEnv());
        }
    }

    size_t GetFirstWaypointIndexAfterTime(dReal time) const
    {
        BOOST_ASSERT(_bInit);
        BOOST_ASSERT(_timeoffset>=0);
        _ComputeInternal();
        if( _vaccumtime.size() == 0 ) {
            return 0;
        }
        if( time < _vaccumtime.at(0) ) {
            return 0;
        }
        if( time >= _vaccumtime.at(_vaccumtime.size()-1) ) {
            return GetNumWaypoints();
        }
        std::vector<dReal>::const_iterator itaccum = std::lower_bound(_vaccumtime.begin(), _vaccumtime.end(), time);
        return itaccum-_vaccumtime.begin();
    }

    dReal GetDuration() const
    {
        BOOST_ASSERT(_bInit);
        _ComputeInternal();
        return _vaccumtime.size() > 0 ? _vaccumtime.back() : 0;
    }

    void serialize(std::ostream& O, int options) const
    {
        O << "<trajectory>" << endl << _spec;
        O << "<data count=\"" << GetNumWaypoints() << "\">" << endl;
        FOREACHC(it,_vtrajdata) {
            O << *it << " ";
        }
        O << "</data>" << endl;
        if( GetDescription().size() > 0 ) {
            O << "<description><![CDATA[" << GetDescription() << "]]></description>" << endl;
        }
        if( GetReadableInterfaces().size() > 0 ) {
            xmlreaders::StreamXMLWriterPtr writer(new xmlreaders::StreamXMLWriter("readable"));
            FOREACHC(it, GetReadableInterfaces()) {
                BaseXMLWriterPtr newwriter = writer->AddChild(it->first);
                it->second->Serialize(newwriter,options);
            }
            writer->Serialize(O);
        }
        O << "</trajectory>" << endl;
    }

    void Clone(InterfaceBaseConstPtr preference, int cloningoptions)
    {
        InterfaceBase::Clone(preference,cloningoptions);
        TrajectoryBaseConstPtr r = RaveInterfaceConstCast<TrajectoryBase>(preference);
        Init(r->GetConfigurationSpecification());
        r->GetWaypoints(0,r->GetNumWaypoints(),_vtrajdata);
        _bChanged = true;
    }

    void Swap(TrajectoryBasePtr rawtraj)
    {
        OPENRAVE_ASSERT_OP(GetXMLId(),==,rawtraj->GetXMLId());
        boost::shared_ptr<GenericTrajectory> traj = boost::dynamic_pointer_cast<GenericTrajectory>(rawtraj);
        _spec.Swap(traj->_spec);
        _vderivoffsets.swap(traj->_vderivoffsets);
        _vddoffsets.swap(traj->_vddoffsets);
        _vdddoffsets.swap(traj->_vdddoffsets);
        _vintegraloffsets.swap(traj->_vintegraloffsets);
        std::swap(_timeoffset, traj->_timeoffset);
        std::swap(_bInit, traj->_bInit);
        std::swap(_vtrajdata, traj->_vtrajdata);
        std::swap(_vaccumtime, traj->_vaccumtime);
        std::swap(_vdeltainvtime, traj->_vdeltainvtime);
        std::swap(_bChanged, traj->_bChanged);
        std::swap(_bSamplingVerified, traj->_bSamplingVerified);
        _InitializeGroupFunctions();
    }

protected:
    void _ConvertData(std::vector<dReal>::iterator ittargetdata, std::vector<dReal>::const_iterator itsourcedata, const std::vector< std::vector<ConfigurationSpecification::Group>::const_iterator >& vconvertgroups, const ConfigurationSpecification& spec, size_t numelements, bool filluninitialized)
    {
        for(size_t igroup = 0; igroup < vconvertgroups.size(); ++igroup) {
            if( vconvertgroups[igroup] != spec._vgroups.end() ) {
                ConfigurationSpecification::ConvertGroupData(ittargetdata+_spec._vgroups[igroup].offset, _spec.GetDOF(), _spec._vgroups[igroup], itsourcedata+vconvertgroups[igroup]->offset, spec.GetDOF(), *vconvertgroups[igroup],numelements,GetEnv(),filluninitialized);
            }
            else if( filluninitialized ) {
                vector<dReal> vdefaultvalues(_spec._vgroups[igroup].dof,0);
                const string& groupname = _spec._vgroups[igroup].name;
                if( groupname.size() >= 16 && groupname.substr(0,16) == "affine_transform" ) {
                    stringstream ss(groupname.substr(16));
                    string robotname;
                    int affinedofs=0;
                    ss >> robotname >> affinedofs;
                    if( !!ss ) {
                        BOOST_ASSERT((int)vdefaultvalues.size()==RaveGetAffineDOF(affinedofs));
                        RaveGetAffineDOFValuesFromTransform(vdefaultvalues.begin(),Transform(),affinedofs);
                    }
                }
                int offset = _spec._vgroups[igroup].offset;
                for(size_t ielement = 0; ielement < numelements; ++ielement, offset += _spec.GetDOF()) {
                    for(int j = 0; j < _spec._vgroups[igroup].dof; ++j) {
                        *(ittargetdata+offset+j) = vdefaultvalues[j];
                    }
                }
            }
        }
    }

    void _ComputeInternal() const
    {
        if( !_bChanged ) {
            return;
        }
        if( _timeoffset < 0 ) {
            _vaccumtime.resize(0);
            _vdeltainvtime.resize(0);
        }
        else {
            _vaccumtime.resize(GetNumWaypoints());
            _vdeltainvtime.resize(_vaccumtime.size());
            if( _vaccumtime.size() == 0 ) {
                return;
            }
            _vaccumtime.at(0) = _vtrajdata.at(_timeoffset);
            _vdeltainvtime.at(0) = 1/_vtrajdata.at(_timeoffset);
            for(size_t i = 1; i < _vaccumtime.size(); ++i) {
                dReal deltatime = _vtrajdata[_spec.GetDOF()*i+_timeoffset];
                if( deltatime < 0 ) {
                    throw OPENRAVE_EXCEPTION_FORMAT("deltatime (%.15e) is < 0 at point %d/%d", deltatime%i%_vaccumtime.size(), ORE_InvalidState);
                }
                _vdeltainvtime[i] = 1/deltatime;
                _vaccumtime[i] = _vaccumtime[i-1] + deltatime;
            }
        }
        _bChanged = false;
        _bSamplingVerified = false;
    }

    /// \brief assumes _ComputeInternal has finished
    void _VerifySampling() const
    {
        BOOST_ASSERT(!_bChanged);
        BOOST_ASSERT(_bInit);
        if( _bSamplingVerified ) {
            return;
        }
        for(size_t i = 0; i < _vgroupinterpolators.size(); ++i) {
            if( _spec._vgroups.at(i).offset != _timeoffset ) {
                if( !_vgroupinterpolators[i] ) {
                    RAVELOG_WARN(str(boost::format("unknown interpolation method '%s' for group '%s'")%_spec._vgroups.at(i).interpolation%_spec._vgroups.at(i).name));
                }
            }
        }

        for(size_t i = 0; i < _spec._vgroups.size(); ++i) {
            const string& interpolation = _spec._vgroups[i].interpolation;
            const string& name = _spec._vgroups[i].name;
            for(int j = 0; j < _spec._vgroups[i].dof; ++j) {
                if( _vderivoffsets.at(_spec._vgroups[i].offset+j) < -2 && _vintegraloffsets.at(_spec._vgroups[i].offset+j) < -2 ) {
                    throw OPENRAVE_EXCEPTION_FORMAT(_("%s interpolation group '%s' needs derivatives/integrals for sampling"),interpolation%name,ORE_InvalidArguments);
                }
            }
        }

        if( IS_DEBUGLEVEL(Level_Debug) || (RaveGetDebugLevel() & Level_VerifyPlans) ) {
            // go through all the points
            for(size_t ipoint = 0; ipoint+1 < _vaccumtime.size(); ++ipoint) {
                dReal deltatime = _vaccumtime[ipoint+1] - _vaccumtime[ipoint];
                for(size_t i = 0; i < _vgroupvalidators.size(); ++i) {
                    if( !!_vgroupvalidators[i] ) {
                        _vgroupvalidators[i](ipoint,deltatime);
                    }
                }
            }
        }
        _bSamplingVerified = true;
    }

    /// \brief called in order to initialize _vgroupinterpolators and _vgroupvalidators, _vderivoffsets, _vintegraloffsets
    void _InitializeGroupFunctions()
    {
        // first set sizes to 0
        _vgroupinterpolators.resize(0);
        _vgroupvalidators.resize(0);
        _vderivoffsets.resize(0);
        _vddoffsets.resize(0);
        _vdddoffsets.resize(0);
        _vintegraloffsets.resize(0);
        _vgroupinterpolators.resize(_spec._vgroups.size());
        _vgroupvalidators.resize(_spec._vgroups.size());
        _vderivoffsets.resize(_spec.GetDOF(),-1);
        _vddoffsets.resize(_spec.GetDOF(),-1);
        _vdddoffsets.resize(_spec.GetDOF(),-1);
        _vintegraloffsets.resize(_spec.GetDOF(),-1);
        for(size_t i = 0; i < _spec._vgroups.size(); ++i) {
            const string& interpolation = _spec._vgroups[i].interpolation;
            int nNeedNeighboringInfo = 0;
            if( interpolation == "previous" ) {
                _vgroupinterpolators[i] = boost::bind(&GenericTrajectory::_InterpolatePrevious,this,boost::ref(_spec._vgroups[i]),_1,_2,_3);
            }
            else if( interpolation == "next" ) {
                _vgroupinterpolators[i] = boost::bind(&GenericTrajectory::_InterpolateNext,this,boost::ref(_spec._vgroups[i]),_1,_2,_3);
            }
            else if( interpolation == "linear" ) {
                if( _spec._vgroups[i].name.size() >= 14 && _spec._vgroups[i].name.substr(0,14) == "ikparam_values" ) {
                    stringstream ss(_spec._vgroups[i].name.substr(14));
                    int niktype=0;
                    ss >> niktype;
                    _vgroupinterpolators[i] = boost::bind(&GenericTrajectory::_InterpolateLinearIk,this,boost::ref(_spec._vgroups[i]),_1,_2,_3,static_cast<IkParameterizationType>(niktype));
                    // TODO add validation for ikparam until
                }
                else {
                    _vgroupinterpolators[i] = boost::bind(&GenericTrajectory::_InterpolateLinear,this,boost::ref(_spec._vgroups[i]),_1,_2,_3);
                    _vgroupvalidators[i] = boost::bind(&GenericTrajectory::_ValidateLinear,this,boost::ref(_spec._vgroups[i]),_1,_2);
                }
                nNeedNeighboringInfo = 2;
            }
            else if( interpolation == "quadratic" ) {
                if( _spec._vgroups[i].name.size() >= 14 && _spec._vgroups[i].name.substr(0,14) == "ikparam_values" ) {
                    stringstream ss(_spec._vgroups[i].name.substr(14));
                    int niktype=0;
                    ss >> niktype;
                    _vgroupinterpolators[i] = boost::bind(&GenericTrajectory::_InterpolateQuadraticIk,this,boost::ref(_spec._vgroups[i]),_1,_2,_3,static_cast<IkParameterizationType>(niktype));
                    // TODO add validation for ikparam until
                }
                else {
                    _vgroupinterpolators[i] = boost::bind(&GenericTrajectory::_InterpolateQuadratic,this,boost::ref(_spec._vgroups[i]),_1,_2,_3);
                    _vgroupvalidators[i] = boost::bind(&GenericTrajectory::_ValidateQuadratic,this,boost::ref(_spec._vgroups[i]),_1,_2);
                }
                nNeedNeighboringInfo = 3;
            }
            else if( interpolation == "cubic" ) {
                _vgroupinterpolators[i] = boost::bind(&GenericTrajectory::_InterpolateCubic,this,boost::ref(_spec._vgroups[i]),_1,_2,_3);
                _vgroupvalidators[i] = boost::bind(&GenericTrajectory::_ValidateCubic,this,boost::ref(_spec._vgroups[i]),_1,_2);
                nNeedNeighboringInfo = 3;
            }
            else if( interpolation == "quartic" ) {
                _vgroupinterpolators[i] = boost::bind(&GenericTrajectory::_InterpolateQuartic,this,boost::ref(_spec._vgroups[i]),_1,_2,_3);
                _vgroupvalidators[i] = boost::bind(&GenericTrajectory::_ValidateQuartic,this,boost::ref(_spec._vgroups[i]),_1,_2);
                nNeedNeighboringInfo = 3;
            }
            else if( interpolation == "quintic" ) {
                _vgroupinterpolators[i] = boost::bind(&GenericTrajectory::_InterpolateQuintic,this,boost::ref(_spec._vgroups[i]),_1,_2,_3);
                _vgroupvalidators[i] = boost::bind(&GenericTrajectory::_ValidateQuintic,this,boost::ref(_spec._vgroups[i]),_1,_2);
                nNeedNeighboringInfo = 3;
            }
            else if( interpolation == "sextic" ) {
                _vgroupinterpolators[i] = boost::bind(&GenericTrajectory::_InterpolateSextic,this,boost::ref(_spec._vgroups[i]),_1,_2,_3);
                _vgroupvalidators[i] = boost::bind(&GenericTrajectory::_ValidateSextic,this,boost::ref(_spec._vgroups[i]),_1,_2);
                nNeedNeighboringInfo = 3;
            }
            else if( interpolation == "" ) {
                // if there is no interpolation, default to "next". deltatime is such a group, but that is overwritten
                _vgroupinterpolators[i] = boost::bind(&GenericTrajectory::_InterpolateNext,this,boost::ref(_spec._vgroups[i]),_1,_2,_3);
            }


            if( nNeedNeighboringInfo ) {
                std::vector<ConfigurationSpecification::Group>::const_iterator itderiv = _spec.FindTimeDerivativeGroup(_spec._vgroups[i]);
                if( itderiv == _spec._vgroups.end() ) {
                    // don't throw an error here since it is unknown if the trajectory will be sampled
                    for(int j = 0; j < _spec._vgroups[i].dof; ++j) {
                        _vderivoffsets[_spec._vgroups[i].offset+j] = -nNeedNeighboringInfo;
                    }
                }
                else {
                    for(int j = 0; j < _spec._vgroups[i].dof; ++j) {
                        _vderivoffsets[_spec._vgroups[i].offset+j] = itderiv->offset+j;
                    }
                    std::vector<ConfigurationSpecification::Group>::const_iterator itdd = _spec.FindTimeDerivativeGroup(*itderiv);
                    if( itdd == _spec._vgroups.end() ) {
                        // don't throw an error here since it is unknown if the trajectory will be sampled
                        for(int j = 0; j < _spec._vgroups[i].dof; ++j) {
                            _vddoffsets[_spec._vgroups[i].offset+j] = -nNeedNeighboringInfo;
                        }
                    }
                    else {
                        for(int j = 0; j < _spec._vgroups[i].dof; ++j) {
                            _vddoffsets[_spec._vgroups[i].offset+j] = itdd->offset+j;
                        }
                        std::vector<ConfigurationSpecification::Group>::const_iterator itddd = _spec.FindTimeDerivativeGroup(*itdd);
                        if( itddd == _spec._vgroups.end() ) {
                            // don't throw an error here since it is unknown if the trajectory will be sampled
                            for(int j = 0; j < _spec._vgroups[i].dof; ++j) {
                                _vdddoffsets[_spec._vgroups[i].offset+j] = -nNeedNeighboringInfo;
                            }
                        }
                        else {
                            for(int j = 0; j < _spec._vgroups[i].dof; ++j) {
                                _vdddoffsets[_spec._vgroups[i].offset+j] = itddd->offset+j;
                            }
                        }
                    }
                }
                std::vector<ConfigurationSpecification::Group>::const_iterator itintegral = _spec.FindTimeIntegralGroup(_spec._vgroups[i]);
                if( itintegral == _spec._vgroups.end() ) {
                    // don't throw an error here since it is unknown if the trajectory will be sampled
                    for(int j = 0; j < _spec._vgroups[i].dof; ++j) {
                        _vintegraloffsets[_spec._vgroups[i].offset+j] = -nNeedNeighboringInfo;
                    }
                }
                else {
                    for(int j = 0; j < _spec._vgroups[i].dof; ++j) {
                        _vintegraloffsets[_spec._vgroups[i].offset+j] = itintegral->offset+j;
                    }
                }
            }
        }
    }

    void _InterpolatePrevious(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime, std::vector<dReal>& data)
    {
        size_t offset = ipoint*_spec.GetDOF()+g.offset;
        if( (ipoint+1)*_spec.GetDOF() < _vtrajdata.size() ) {
            // if point is so close the previous, then choose the next
            dReal f = _vdeltainvtime.at(ipoint+1)*deltatime;
            if( f > 1-g_fEpsilon ) {
                offset += _spec.GetDOF();
            }
        }
        std::copy(_vtrajdata.begin()+offset,_vtrajdata.begin()+offset+g.dof,data.begin()+g.offset);
    }

    void _InterpolateNext(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime, std::vector<dReal>& data)
    {
        if( (ipoint+1)*_spec.GetDOF() < _vtrajdata.size() ) {
            ipoint += 1;
        }
        size_t offset = ipoint*_spec.GetDOF() + g.offset;
        if( deltatime <= g_fEpsilon && ipoint > 0 ) {
            // if point is so close the previous, then choose the previous
            offset -= _spec.GetDOF();
        }
        std::copy(_vtrajdata.begin()+offset,_vtrajdata.begin()+offset+g.dof,data.begin()+g.offset);
    }

    void _InterpolateLinear(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime, std::vector<dReal>& data)
    {
        size_t offset = ipoint*_spec.GetDOF();
        int derivoffset = _vderivoffsets[g.offset];
        if( derivoffset < 0 ) {
            // expected derivative offset, interpolation can be wrong for circular joints
            dReal f = _vdeltainvtime.at(ipoint+1)*deltatime;
            for(int i = 0; i < g.dof; ++i) {
                data[g.offset+i] = _vtrajdata[offset+g.offset+i]*(1-f) + f*_vtrajdata[_spec.GetDOF()+offset+g.offset+i];
            }
        }
        else {
            for(int i = 0; i < g.dof; ++i) {
                dReal deriv0 = _vtrajdata[_spec.GetDOF()+offset+derivoffset+i];
                data[g.offset+i] = _vtrajdata[offset+g.offset+i] + deltatime*deriv0;
            }
        }
    }

    void _InterpolateLinearIk(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime, std::vector<dReal>& data, IkParameterizationType iktype)
    {
        _InterpolateLinear(g,ipoint,deltatime,data);
        if( deltatime > g_fEpsilon ) {
            size_t offset = ipoint*_spec.GetDOF();
            dReal f = _vdeltainvtime.at(ipoint+1)*deltatime;
            switch(iktype) {
            case IKP_Rotation3D:
            case IKP_Transform6D: {
                Vector q0, q1;
                q0.Set4(&_vtrajdata[offset+g.offset]);
                q1.Set4(&_vtrajdata[_spec.GetDOF()+offset+g.offset]);
                Vector q = quatSlerp(q0,q1,f);
                data[g.offset+0] = q[0];
                data[g.offset+1] = q[1];
                data[g.offset+2] = q[2];
                data[g.offset+3] = q[3];
                break;
            }
            case IKP_TranslationDirection5D: {
                Vector dir0(_vtrajdata[offset+g.offset+0],_vtrajdata[offset+g.offset+1],_vtrajdata[offset+g.offset+2]);
                Vector dir1(_vtrajdata[_spec.GetDOF()+offset+g.offset+0],_vtrajdata[_spec.GetDOF()+offset+g.offset+1],_vtrajdata[_spec.GetDOF()+offset+g.offset+2]);
                Vector axisangle = dir0.cross(dir1);
                dReal fsinangle = RaveSqrt(axisangle.lengthsqr3());
                if( fsinangle > g_fEpsilon ) {
                    axisangle *= f*RaveAsin(min(dReal(1),fsinangle))/fsinangle;
                    Vector newdir = quatRotate(quatFromAxisAngle(axisangle),dir0);
                    data[g.offset+0] = newdir[0];
                    data[g.offset+1] = newdir[1];
                    data[g.offset+2] = newdir[2];
                }
                break;
            }
            default:
                break;
            }
        }
    }

    void _InterpolateQuadratic(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime, std::vector<dReal>& data)
    {
        size_t offset = ipoint*_spec.GetDOF();
        if( deltatime > g_fEpsilon ) {
            int derivoffset = _vderivoffsets[g.offset];
            if( derivoffset >= 0 ) {
                for(int i = 0; i < g.dof; ++i) {
                    // coeff*t^2 + deriv0*t + pos0
                    dReal deriv0 = _vtrajdata[offset+derivoffset+i];
                    dReal deriv1 = _vtrajdata[_spec.GetDOF()+offset+derivoffset+i];
                    dReal coeff = 0.5*_vdeltainvtime.at(ipoint+1)*(deriv1-deriv0);
                    data[g.offset+i] = _vtrajdata[offset+g.offset+i] + deltatime*(deriv0 + deltatime*coeff);
                }
            }
            else {
                dReal ideltatime = _vdeltainvtime.at(ipoint+1);
                dReal ideltatime2 = ideltatime*ideltatime;
                int integraloffset = _vintegraloffsets[g.offset];
                for(int i = 0; i < g.dof; ++i) {
                    // c2*t**2 + c1*t + v0
                    // c2*deltatime**2 + c1*deltatime + v0 = v1
                    // integral: c2/3*deltatime**3 + c1/2*deltatime**2 + v0*deltatime = p1-p0
                    // mult by (3/deltatime): c2*deltatime**2 + 3/2*c1*deltatime + 3*v0 = 3*(p1-p0)/deltatime
                    // subtract by original: 0.5*c1*deltatime + 2*v0 - 3*(p1-p0)/deltatime + v1 = 0
                    // c1*deltatime = 6*(p1-p0)/deltatime - 4*v0 - 2*v1
                    dReal integral0 = _vtrajdata[offset+integraloffset+i];
                    dReal integral1 = _vtrajdata[_spec.GetDOF()+offset+integraloffset+i];
                    dReal value0 = _vtrajdata[offset+g.offset+i];
                    dReal value1 = _vtrajdata[_spec.GetDOF()+offset+g.offset+i];
                    dReal c1TimesDelta = 6*(integral1-integral0)*ideltatime - 4*value0 - 2*value1;
                    dReal c1 = c1TimesDelta*ideltatime;
                    dReal c2 = (value1 - value0 - c1TimesDelta)*ideltatime2;
                    data[g.offset+i] = value0 + deltatime * (c1 + deltatime*c2);
                }
            }
        }
        else {
            for(int i = 0; i < g.dof; ++i) {
                data[g.offset+i] = _vtrajdata[offset+g.offset+i];
            }
        }
    }

    void _InterpolateQuadraticIk(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime, std::vector<dReal>& data, IkParameterizationType iktype)
    {
        _InterpolateQuadratic(g, ipoint, deltatime, data);
        if( deltatime > g_fEpsilon ) {
            int derivoffset = _vderivoffsets[g.offset];
            size_t offset = ipoint*_spec.GetDOF();
            Vector q0, q0vel, q1, q1vel;
            switch(iktype) {
            case IKP_Rotation3D:
            case IKP_Transform6D: {
                q0.Set4(&_vtrajdata[offset+g.offset]);
                q0vel.Set4(&_vtrajdata[offset+derivoffset]);
                q1.Set4(&_vtrajdata[_spec.GetDOF()+offset+g.offset]);
                q1vel.Set4(&_vtrajdata[_spec.GetDOF()+offset+derivoffset]);
                Vector angularvelocity0 = quatMultiply(q0vel,quatInverse(q0))*2;
                Vector angularvelocity1 = quatMultiply(q1vel,quatInverse(q1))*2;
                Vector coeff = (angularvelocity1-angularvelocity0)*(0.5*_vdeltainvtime.at(ipoint+1));
                Vector vtotaldelta = angularvelocity0*deltatime + coeff*(deltatime*deltatime);
                Vector q = quatMultiply(quatFromAxisAngle(Vector(vtotaldelta.y,vtotaldelta.z,vtotaldelta.w)),q0);
                data[g.offset+0] = q[0];
                data[g.offset+1] = q[1];
                data[g.offset+2] = q[2];
                data[g.offset+3] = q[3];
                break;
            }
            case IKP_TranslationDirection5D: {
                Vector dir0, dir1, angularvelocity0, angularvelocity1;
                dir0.Set3(&_vtrajdata[offset+g.offset]);
                dir1.Set3(&_vtrajdata[_spec.GetDOF()+offset+g.offset]);
                Vector axisangle = dir0.cross(dir1);
                if( axisangle.lengthsqr3() > g_fEpsilon ) {
                    angularvelocity0.Set3(&_vtrajdata[offset+derivoffset]);
                    angularvelocity1.Set3(&_vtrajdata[_spec.GetDOF()+offset+derivoffset]);
                    Vector coeff = (angularvelocity1-angularvelocity0)*(0.5*_vdeltainvtime.at(ipoint+1));
                    Vector vtotaldelta = angularvelocity0*deltatime + coeff*(deltatime*deltatime);
                    Vector newdir = quatRotate(quatFromAxisAngle(vtotaldelta),dir0);
                    data[g.offset+0] = newdir[0];
                    data[g.offset+1] = newdir[1];
                    data[g.offset+2] = newdir[2];
                }
                break;
            }
            default:
                break;
            }
        }
    }

    void _InterpolateCubic(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime, std::vector<dReal>& data)
    {
        // p = c3*t**3 + c2*t**2 + c1*t + c0
        // c3 = (v1*dt + v0*dt - 2*px)/(dt**3)
        // c2 = (3*px - 2*v0*dt - v1*dt)/(dt**2)
        // c1 = v0
        // c0 = p0
        size_t offset = ipoint*_spec.GetDOF();
        if( deltatime > g_fEpsilon ) {
            int derivoffset = _vderivoffsets[g.offset];
            if( derivoffset >= 0 ) {
                dReal ideltatime = _vdeltainvtime.at(ipoint+1);
                dReal ideltatime2 = ideltatime*ideltatime;
                dReal ideltatime3 = ideltatime2*ideltatime;
                for(int i = 0; i < g.dof; ++i) {
                    // coeff*t^2 + deriv0*t + pos0
                    dReal deriv0 = _vtrajdata[offset+derivoffset+i];
                    dReal deriv1 = _vtrajdata[_spec.GetDOF()+offset+derivoffset+i];
                    dReal px = _vtrajdata.at(_spec.GetDOF()+offset+g.offset+i) - _vtrajdata[offset+g.offset+i];
                    dReal c3 = (deriv1+deriv0)*ideltatime2 - 2*px*ideltatime3;
                    dReal c2 = 3*px*ideltatime2 - (2*deriv0+deriv1)*ideltatime;
                    data[g.offset+i] = _vtrajdata[offset+g.offset+i] + deltatime*(deriv0 + deltatime*(c2 + deltatime*c3));
                }
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT0(_("cubic interpolation does not have all data"),ORE_InvalidArguments);
            }
        }
        else {
            for(int i = 0; i < g.dof; ++i) {
                data[g.offset+i] = _vtrajdata[offset+g.offset+i];
            }
        }
    }

    void _InterpolateQuartic(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime, std::vector<dReal>& data)
    {
        // p = c4*t**4 + c3*t**3 + c2*t**2 + c1*t + c0
        //
        // v1 = 4*c4*dt**3 + 3*c3*dt**2 + a0*dt + v0
        // a1 = 12*c4*dt**2 + 6*c3*dt + a0
        //
        // c4 = (-2*(v1-v0) + (a0 + a1)*dt)/(4*dt**3)
        // c3 = ((v1-v0)*3 - (2*a0+a1)*dt)/(3*dt**2)
        // c2 = a0/2
        // c1 = v0
        // c0 = p0
        size_t offset = ipoint*_spec.GetDOF();
        if( deltatime > g_fEpsilon ) {
            int derivoffset = _vderivoffsets[g.offset];
            int ddoffset = _vddoffsets[g.offset];
            if( derivoffset >= 0 && ddoffset >= 0 ) {
                dReal ideltatime = _vdeltainvtime.at(ipoint+1);
                dReal ideltatime2 = ideltatime*ideltatime;
                dReal ideltatime3 = ideltatime2*ideltatime;
                for(int i = 0; i < g.dof; ++i) {
                    dReal deriv0 = _vtrajdata[offset+derivoffset+i];
                    dReal deriv1 = _vtrajdata[_spec.GetDOF()+offset+derivoffset+i];
                    dReal dd0 = _vtrajdata[offset+ddoffset+i];
                    dReal dd1 = _vtrajdata[_spec.GetDOF()+offset+ddoffset+i];
                    dReal c4 = -0.5*(deriv1-deriv0)*ideltatime3 + (dd0 + dd1)*ideltatime2*0.25;
                    dReal c3 = (deriv1-deriv0)*ideltatime2 - (2*dd0+dd1)*ideltatime/3.0;
                    data[g.offset+i] = _vtrajdata[offset+g.offset+i] + deltatime*(deriv0 + deltatime*(0.5*dd0 + deltatime*(c3 + deltatime*c4)));
                }
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT0(_("cubic interpolation does not have all data"),ORE_InvalidArguments);
            }
        }
        else {
            for(int i = 0; i < g.dof; ++i) {
                data[g.offset+i] = _vtrajdata[offset+g.offset+i];
            }
        }
    }

    void _InterpolateQuintic(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime, std::vector<dReal>& data)
    {
        // p0, p1, v0, v1, a0, a1, dt, t, c5, c4, c3 = symbols('p0, p1, v0, v1, a0, a1, dt, t, c5, c4, c3')
        // p = c5*t**5 + c4*t**4 + c3*t**3 + c2*t**2 + c1*t + c0
        //
        // c5*dt**5 + c4*dt**4 + c3*dt**3 + a0/2*t**2 + v0*t + p0 - p1 = 0
        // 5*c5*t**4 + 4*c4*dt**3 + 3*c3*dt**2 + a0*dt + v0 - v1 = 0
        // 20*c5*t**3 + 12*c4*dt**2 + 6*c3*dt + a0 - a1 = 0
        //
        // A = Matrix(3,3,[dt**5, dt**4, dt**3, 5*dt**4, 4*dt**3, 3*dt**2, 20*dt**3, 12*dt**2, 6*dt])
        // b = Matrix(3, 1, [p1 -a0/2*dt**2 - v0*dt - p0, v1 -a0*dt - v0, a1 - a0])
        // c5 = -0.5*a0/dt**3 + a1/(2*dt**3) - 3*v0/dt**4 - 3*v1/dt**4 - 6*p0/dt**5 + 6*p1/dt**5
        // c4 = 1.5*a0/dt**2 - a1/dt**2 + 8*v0/dt**3 + 7*v1/dt**3 + 15*p0/dt**4 - 15*p1/dt**4
        // c3 = -1.5*a0/dt + a1/(2*dt) - 6*v0/dt**2 - 4*v1/dt**2 - 10*p0/dt**3 + 10*p1/dt**3
        // c2 = a0/2
        // c1 = v0
        // c0 = p0
        size_t offset = ipoint*_spec.GetDOF();
        if( deltatime > g_fEpsilon ) {
            int derivoffset = _vderivoffsets[g.offset];
            int ddoffset = _vddoffsets[g.offset];
            if( derivoffset >= 0 && ddoffset >= 0 ) {
                dReal ideltatime = _vdeltainvtime.at(ipoint+1);
                dReal ideltatime2 = ideltatime*ideltatime;
                dReal ideltatime3 = ideltatime2*ideltatime;
                dReal ideltatime4 = ideltatime2*ideltatime2;
                dReal ideltatime5 = ideltatime4*ideltatime;
                for(int i = 0; i < g.dof; ++i) {
                    dReal p0 = _vtrajdata[offset+g.offset+i];
                    dReal px = _vtrajdata[_spec.GetDOF()+offset+g.offset+i] - p0;
                    dReal deriv0 = _vtrajdata[offset+derivoffset+i];
                    dReal deriv1 = _vtrajdata[_spec.GetDOF()+offset+derivoffset+i];
                    dReal dd0 = _vtrajdata[offset+ddoffset+i];
                    dReal dd1 = _vtrajdata[_spec.GetDOF()+offset+ddoffset+i];
                    dReal c5 = (-0.5*dd0 + dd1*0.5)*ideltatime3 - (3*deriv0 + 3*deriv1)*ideltatime4 + px*6*ideltatime5;
                    dReal c4 = (1.5*dd0 - dd1)*ideltatime2 + (8*deriv0 + 7*deriv1)*ideltatime3 - px*15*ideltatime4;
                    dReal c3 = (-1.5*dd0 + dd1*0.5)*ideltatime + (- 6*deriv0 - 4*deriv1)*ideltatime2 + px*10*ideltatime3;
                    data[g.offset+i] = p0 + deltatime*(deriv0 + deltatime*(0.5*dd0 + deltatime*(c3 + deltatime*(c4 + deltatime*c5))));
                }
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT0(_("cubic interpolation does not have all data"),ORE_InvalidArguments);
            }
        }
        else {
            for(int i = 0; i < g.dof; ++i) {
                data[g.offset+i] = _vtrajdata[offset+g.offset+i];
            }
        }
    }

    void _InterpolateSextic(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime, std::vector<dReal>& data)
    {
        // p = c6*t**6 + c5*t**5 + c4*t**4 + c3*t**3 + c2*t**2 + c1*t + c0
        //
        // v1 = 6*c6*dt**5 + 5*c5*dt**4 + 4*c4*dt**3 + j0/2*dt**2 + a0*dt + v0
        // a1 = 30*c6*dt**4 + 20*c5*dt**3 + 12*c4*dt**2 + j0*dt + a0
        // j1 = 120*c6*dt**3 + 60*c5*dt**2 + 24*c4*dt + j0
        //
        // c6 = -a0/(2*dt**4) - a1/(2*dt**4) - j0/(12*dt**3) + j1/(12*dt**3) - v0/dt**5 + v1/dt**5
        // c5 = 8*a0/(5*dt**3) + 7*a1/(5*dt**3) + 3*j0/(10*dt**2) - j1/(5*dt**2) + 3*v0/dt**4 - 3*v1/dt**4
        // c4 = -3*a0/(2*dt**2) - a1/dt**2 - 3*j0/(8*dt) + j1/(8*dt) - 5*v0/(2*dt**3) + 5*v1/(2*dt**3)
        // c3 = j0/6
        // c2 = a0/2
        // c1 = v0
        // c0 = p0
        size_t offset = ipoint*_spec.GetDOF();
        if( deltatime > g_fEpsilon ) {
            int derivoffset = _vderivoffsets[g.offset];
            int ddoffset = _vddoffsets[g.offset];
            int dddoffset = _vdddoffsets[g.offset];
            if( derivoffset >= 0 && ddoffset >= 0 && dddoffset >= 0 ) {
                dReal ideltatime = _vdeltainvtime.at(ipoint+1);
                dReal ideltatime2 = ideltatime*ideltatime;
                dReal ideltatime3 = ideltatime2*ideltatime;
                dReal ideltatime4 = ideltatime2*ideltatime2;
                dReal ideltatime5 = ideltatime4*ideltatime;
                //dReal deltatime2 = deltatime*deltatime;
                //dReal deltatime3 = deltatime2*deltatime;
                //dReal deltatime4 = deltatime2*deltatime2;
                //dReal deltatime5 = deltatime4*deltatime;
                for(int i = 0; i < g.dof; ++i) {
                    dReal p0 = _vtrajdata[offset+g.offset+i];
                    //dReal px = _vtrajdata[_spec.GetDOF()+offset+g.offset+i] - p0;
                    dReal deriv0 = _vtrajdata[offset+derivoffset+i];
                    dReal deriv1 = _vtrajdata[_spec.GetDOF()+offset+derivoffset+i];
                    dReal dd0 = _vtrajdata[offset+ddoffset+i];
                    dReal dd1 = _vtrajdata[_spec.GetDOF()+offset+ddoffset+i];
                    dReal ddd0 = _vtrajdata[offset+dddoffset+i];
                    dReal ddd1 = _vtrajdata[_spec.GetDOF()+offset+dddoffset+i];
                    // matrix inverse is slow but at least it will work for now
                    // A=Matrix(3,3,[6*dt**5, 5*dt**4, 4*dt**3, 30*dt**4, 20*dt**3, 12*dt**2, 120*dt**3, 60*dt**2, 24*dt])
                    // A.inv() = [   dt**(-5), -1/(2*dt**4), 1/(12*dt**3)]
                    //           [   -3/dt**4,  7/(5*dt**3), -1/(5*dt**2)]
                    //           [5/(2*dt**3),     -1/dt**2,     1/(8*dt)]
                    // b = Matrix(3,1,[v1 -j0/2*dt**2 - a0*dt - v0, a1 -j0*dt - a0, j1 -j0])
                    //dReal A[9], b[3]; // A*[c6,c5,c4] = b
                    //A[0] = 6*deltatime5;   A[1] = 5*deltatime4;  A[2] = 4*deltatime3;  b[0] = -ddd0*0.5*deltatime2 - dd0*deltatime - deriv0;
                    //A[3] = 30*deltatime4;  A[4] = 20*deltatime3; A[5] = 12*deltatime2; b[1] = -ddd0*deltatime - dd0;
                    //A[6] = 120*deltatime3; A[7] = 60*deltatime2; A[8] = 24*deltatime;  b[2] = -ddd0;
                    //mathextra::inv3(A, deltatime5, NULL, 3);
                    //dReal c6 = A[0]*b[0] + A[1]*b[1] + A[2]*b[2];
                    //dReal c5 = A[3]*b[0] + A[4]*b[1] + A[5]*b[2];
                    //dReal c4 = A[6]*b[0] + A[7]*b[1] + A[8]*b[2];
                    dReal c6 = (-dd0 - dd1)*0.5*ideltatime4 + (- ddd0 + ddd1)/12.0*ideltatime3 + (-deriv0 + deriv1)*ideltatime5;
                    dReal c5 = (1.6*dd0 + 1.4*dd1)*ideltatime3 + (0.3*ddd0 - ddd1*0.2)*ideltatime2 + (3*deriv0 - 3*deriv1)*ideltatime4;
                    dReal c4 = (-1.5*dd0 - dd1)*ideltatime2 + (- 0.375*ddd0 + ddd1*0.125)*ideltatime + (-2.5*deriv0 + 2.5*deriv1)*ideltatime3;
                    data[g.offset+i] = p0 + deltatime*(deriv0 + deltatime*(0.5*dd0 + deltatime*(ddd0/6.0 + deltatime*(c4 + deltatime*(c5 + deltatime*c6)))));
                }
            }
            else {
                throw OPENRAVE_EXCEPTION_FORMAT0(_("cubic interpolation does not have all data"),ORE_InvalidArguments);
            }
        }
        else {
            for(int i = 0; i < g.dof; ++i) {
                data[g.offset+i] = _vtrajdata[offset+g.offset+i];
            }
        }
    }

    void _ValidateLinear(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime)
    {
        size_t offset = ipoint*_spec.GetDOF();
        int derivoffset = _vderivoffsets[g.offset];
        if( derivoffset >= 0 ) {
            for(int i = 0; i < g.dof; ++i) {
                dReal deriv0 = _vtrajdata[_spec.GetDOF()+offset+derivoffset+i];
                dReal expected = _vtrajdata[offset+g.offset+i] + deltatime*deriv0;
                dReal error = RaveFabs(_vtrajdata[_spec.GetDOF()+offset+g.offset+i] - expected);
                if( RaveFabs(error-2*PI) > g_fEpsilonLinear ) { // TODO, officially track circular joints
                    OPENRAVE_ASSERT_OP_FORMAT(error,<=,g_fEpsilonLinear, "trajectory segment for group %s interpolation %s points %d-%d dof %d is invalid", g.name%g.interpolation%ipoint%(ipoint+1)%i, ORE_InvalidState);
                }
            }
        }
    }

    void _ValidateQuadratic(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime)
    {
        if( deltatime > g_fEpsilon ) {
            size_t offset = ipoint*_spec.GetDOF();
            int derivoffset = _vderivoffsets[g.offset];
            if( derivoffset >= 0 ) {
                for(int i = 0; i < g.dof; ++i) {
                    // coeff*t^2 + deriv0*t + pos0
                    dReal deriv0 = _vtrajdata[offset+derivoffset+i];
                    dReal coeff = 0.5*_vdeltainvtime.at(ipoint+1)*(_vtrajdata[_spec.GetDOF()+offset+derivoffset+i]-deriv0);
                    dReal expected = _vtrajdata[offset+g.offset+i] + deltatime*(deriv0 + deltatime*coeff);
                    dReal error = RaveFabs(_vtrajdata.at(_spec.GetDOF()+offset+g.offset+i)-expected);
                    if( RaveFabs(error-2*PI) > 1e-5 ) { // TODO, officially track circular joints
                        OPENRAVE_ASSERT_OP_FORMAT(error,<=,1e-4, "trajectory segment for group %s interpolation %s time %f points %d-%d dof %d is invalid", g.name%g.interpolation%deltatime%ipoint%(ipoint+1)%i, ORE_InvalidState);
                    }
                }
            }
            else {
                int integraloffset = _vintegraloffsets[g.offset];
                BOOST_ASSERT(integraloffset>=0);
                // cannot verify since there's not enough constraints
            }
        }
    }

    void _ValidateCubic(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime)
    {
        // TODO, need 3 groups to verify
    }

    void _ValidateQuartic(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime)
    {
    }

    void _ValidateQuintic(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime)
    {
    }

    void _ValidateSextic(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime)
    {
    }

    ConfigurationSpecification _spec;
    std::vector< boost::function<void(size_t,dReal,std::vector<dReal>&)> > _vgroupinterpolators;
    std::vector< boost::function<void(size_t,dReal)> > _vgroupvalidators;
    std::vector<int> _vderivoffsets, _vddoffsets, _vdddoffsets; ///< for every group that relies on other info to compute its position, this will point to the derivative offset. -1 if invalid and not needed, -2 if invalid and needed
    std::vector<int> _vintegraloffsets; ///< for every group that relies on other info to compute its position, this will point to the integral offset (ie the position for a velocity group). -1 if invalid and not needed, -2 if invalid and needed
    int _timeoffset;

    std::vector<dReal> _vtrajdata;
    mutable std::vector<dReal> _vaccumtime, _vdeltainvtime;
    bool _bInit;
    mutable bool _bChanged; ///< if true, then _ComputeInternal() has to be called in order to compute _vaccumtime and _vdeltainvtime
    mutable bool _bSamplingVerified; ///< if false, then _VerifySampling() has not be called yet to verify that all points can be sampled.
};

TrajectoryBasePtr CreateGenericTrajectory(EnvironmentBasePtr penv, std::istream& sinput)
{
    return TrajectoryBasePtr(new GenericTrajectory(penv,sinput));
}

}
