// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
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

namespace OpenRAVE {

class GenericTrajectory : public TrajectoryBase
{
    std::map<string,int> _maporder;
public:
    GenericTrajectory(EnvironmentBasePtr penv, std::istream& sinput) : TrajectoryBase(penv), _timeoffset(-1)
    {
        _maporder["deltatime"] = 0;
        _maporder["joint_accelerations"] = 1;
        _maporder["affine_accelerations"] = 2;
        _maporder["joint_velocities"] = 3;
        _maporder["affine_velocities"] = 4;
        _maporder["joint_values"] = 5;
        _maporder["affine_transform"] = 6;
        _maporder["joint_torques"] = 7;
        _bInit = false;
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
            _spec = spec;
            // order the groups based on computation order
            stable_sort(_spec._vgroups.begin(),_spec._vgroups.end(),boost::bind(&GenericTrajectory::SortGroups,this,_1,_2));
            _timeoffset = -1;
            FOREACH(itgroup,_spec._vgroups) {
                if( itgroup->name == "deltatime" ) {
                    _timeoffset = itgroup->offset;
                }
            }
            _vgroupinterpolators.resize(_spec._vgroups.size());
            _vderivoffsets.resize(0); _vderivoffsets.resize(_spec.GetDOF(),-1);
            for(size_t i = 0; i < _spec._vgroups.size(); ++i) {
                const string& interpolation = _spec._vgroups[i].interpolation;
                const string& name = _spec._vgroups[i].name;
                bool bNeedDerivatives = false;
                if( interpolation == "previous" ) {
                    _vgroupinterpolators[i] = boost::bind(&GenericTrajectory::_InterpolatePrevious,this,boost::ref(_spec._vgroups[i]),_1,_2,_3);
                }
                else if( interpolation == "next" ) {
                    _vgroupinterpolators[i] = boost::bind(&GenericTrajectory::_InterpolateNext,this,boost::ref(_spec._vgroups[i]),_1,_2,_3);
                }
                else if( interpolation == "linear" ) {
                    _vgroupinterpolators[i] = boost::bind(&GenericTrajectory::_InterpolateLinear,this,boost::ref(_spec._vgroups[i]),_1,_2,_3);
                }
                else if( interpolation == "quadratic" ) {
                    _vgroupinterpolators[i] = boost::bind(&GenericTrajectory::_InterpolateQuadratic,this,boost::ref(_spec._vgroups[i]),_1,_2,_3);
                    bNeedDerivatives = true;
                }
                else if( interpolation == "cubic" ) {
                    _vgroupinterpolators[i] = boost::bind(&GenericTrajectory::_InterpolateCubic,this,boost::ref(_spec._vgroups[i]),_1,_2,_3);
                    bNeedDerivatives = true;
                }
                else if( interpolation == "quadric" ) {
                    _vgroupinterpolators[i] = boost::bind(&GenericTrajectory::_InterpolateQuadric,this,boost::ref(_spec._vgroups[i]),_1,_2,_3);
                    bNeedDerivatives = true;
                }
                else if( interpolation == "quintic" ) {
                    _vgroupinterpolators[i] = boost::bind(&GenericTrajectory::_InterpolateQuintic,this,boost::ref(_spec._vgroups[i]),_1,_2,_3);
                    bNeedDerivatives = true;
                }
                else if( name != "deltatime" ) {
                    RAVELOG_DEBUG(str(boost::format("unknown interpolation method '%s' for group '%s'")%interpolation%name));
                }


                if( bNeedDerivatives ) {
                    std::vector<ConfigurationSpecification::Group>::const_iterator itderiv = _spec.FindTimeDerivativeGroup(_spec._vgroups[i]);
                    if( itderiv == _spec._vgroups.end() ) {
                        throw OPENRAVE_EXCEPTION_FORMAT("%s interpolation group '%s' needs derivatives",interpolation%name,ORE_InvalidArguments);
                    }
                    for(int j = 0; j < _spec._vgroups[i].dof; ++j) {
                        _vderivoffsets[_spec._vgroups[i].offset+j] = itderiv->offset+j;
                    }
                }
            }
        }
        _vtrajdata.resize(0);
        _vaccumtime.resize(0);
        _vdeltainvtime.resize(0);
        _bChanged = true;
        _bInit = true;
    }

    void Insert(size_t index, const std::vector<dReal>& data, bool bOverwrite)
    {
        BOOST_ASSERT(_bInit);
        if( data.size() == 0 ) {
            return;
        }
        BOOST_ASSERT((data.size()%_spec.GetDOF()) == 0);
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
        BOOST_ASSERT((data.size()%spec.GetDOF()) == 0);
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
                _ConvertData(ittargetdata,itsourcedata,vconvertgroups,spec,copyelements);
                sourceindex = copyelements*spec.GetDOF();
                index += copyelements;
            }
            if( sourceindex < data.size() ) {
                size_t numelements = (data.size()-sourceindex)/spec.GetDOF();
                std::vector<dReal> vtemp(numelements*_spec.GetDOF());
                ittargetdata = vtemp.begin();
                itsourcedata = data.begin()+sourceindex;
                _ConvertData(ittargetdata,itsourcedata,vconvertgroups,spec,numelements);
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
        _vtrajdata.erase(_vtrajdata.begin()+startindex*_spec.GetDOF(),_vtrajdata.begin()+endindex*_spec.GetDOF());
        _bChanged = true;
    }

    void Sample(std::vector<dReal>& data, dReal time) const
    {
        BOOST_ASSERT(_bInit);
        BOOST_ASSERT(_timeoffset>=0);
        BOOST_ASSERT(time >= 0);
        _ComputeInternal();
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
            }
        }
    }

    void Sample(std::vector<dReal>& data, dReal time, const ConfigurationSpecification& spec) const
    {
        BOOST_ASSERT(_bInit);
        BOOST_ASSERT(_timeoffset>=0);
        BOOST_ASSERT(time >= 0);
        _ComputeInternal();
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

    dReal GetDuration() const
    {
        BOOST_ASSERT(_bInit);
        _ComputeInternal();
        return _vaccumtime.size() > 0 ? _vaccumtime.back() : 0;
    }

    void serialize(std::ostream& O, int options) const
    {
        O << "<trajectory>" << endl << _spec << endl;
        O << "<data count=\"" << GetNumWaypoints() << "\">" << endl;
        FOREACHC(it,_vtrajdata) {
            O << *it << " ";
        }
        O << "</data>" << endl;
        //O << "<description><![CDATA[" << GetDescription() << "]]></description>" << endl;
        O << "<description>" << GetDescription() << "</description>" << endl;
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

protected:
    void _ConvertData(std::vector<dReal>::iterator ittargetdata, std::vector<dReal>::const_iterator itsourcedata, const std::vector< std::vector<ConfigurationSpecification::Group>::const_iterator >& vconvertgroups, const ConfigurationSpecification& spec, size_t numelements)
    {
        for(size_t igroup = 0; igroup < vconvertgroups.size(); ++igroup) {
            if( vconvertgroups[igroup] != spec._vgroups.end() ) {
                ConfigurationSpecification::ConvertGroupData(ittargetdata+_spec._vgroups[igroup].offset, _spec.GetDOF(), _spec._vgroups[igroup], itsourcedata+vconvertgroups[igroup]->offset, spec.GetDOF(), *vconvertgroups[igroup],numelements,GetEnv());
            }
            else {
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
                BOOST_ASSERT(deltatime>=0);
                _vdeltainvtime[i] = 1/deltatime;
                _vaccumtime[i] = _vaccumtime[i-1] + deltatime;
            }
        }
        _bChanged = false;
    }

    void _InterpolatePrevious(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime, std::vector<dReal>& data)
    {
        size_t offset= ipoint*_spec.GetDOF()+g.offset;
        std::copy(_vtrajdata.begin()+offset,_vtrajdata.begin()+offset+g.dof,data.begin()+g.offset);
    }

    void _InterpolateNext(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime, std::vector<dReal>& data)
    {
        if( (ipoint+1)*_spec.GetDOF() < _vtrajdata.size() ) {
            ipoint += 1;
        }
        size_t offset= ipoint*_spec.GetDOF()+g.offset;
        std::copy(_vtrajdata.begin()+offset,_vtrajdata.begin()+offset+g.dof,data.begin()+g.offset);
    }

    void _InterpolateLinear(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime, std::vector<dReal>& data)
    {
        size_t offset= ipoint*_spec.GetDOF()+g.offset;
        dReal f = _vdeltainvtime.at(ipoint+1)*deltatime;
        for(int i = 0; i < g.dof; ++i) {
            data[g.offset+i] = (1-f)*_vtrajdata[offset+i] + f*_vtrajdata[_spec.GetDOF()+offset+i];
        }
    }

    void _InterpolateQuadratic(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime, std::vector<dReal>& data)
    {
        size_t offset= ipoint*_spec.GetDOF();
        int derivoffset = _vderivoffsets[g.offset];
        for(int i = 0; i < g.dof; ++i) {
            // coeff*t^2 + deriv0*t + pos0
            dReal deriv0 = _vtrajdata[offset+derivoffset+i];
            dReal coeff = 0.5*_vdeltainvtime.at(ipoint+1)*(_vtrajdata[_spec.GetDOF()+offset+derivoffset+i]-deriv0);
            data[g.offset+i] = _vtrajdata[offset+g.offset+i] + deltatime*(deriv0 + deltatime*coeff);
        }
    }

    void _InterpolateCubic(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime, std::vector<dReal>& data)
    {
        throw OPENRAVE_EXCEPTION_FORMAT0("cubic interpolation not supported",ORE_InvalidArguments);
    }

    void _InterpolateQuadric(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime, std::vector<dReal>& data)
    {
        throw OPENRAVE_EXCEPTION_FORMAT0("quadric interpolation not supported",ORE_InvalidArguments);
    }

    void _InterpolateQuintic(const ConfigurationSpecification::Group& g, size_t ipoint, dReal deltatime, std::vector<dReal>& data)
    {
        throw OPENRAVE_EXCEPTION_FORMAT0("quintic interpolation not supported",ORE_InvalidArguments);
    }

    ConfigurationSpecification _spec;
    std::vector< boost::function<void(size_t,dReal,std::vector<dReal>&)> > _vgroupinterpolators;
    std::vector<int> _vderivoffsets; ///< for every group that relies on derivatives, this will point to the offset (-1 if invalid)
    int _timeoffset;
    bool _bInit;

    std::vector<dReal> _vtrajdata;
    mutable std::vector<dReal> _vaccumtime, _vdeltainvtime;
    mutable bool _bChanged;
};

TrajectoryBasePtr CreateGenericTrajectory(EnvironmentBasePtr penv, std::istream& sinput)
{
    return TrajectoryBasePtr(new GenericTrajectory(penv,sinput));
}

}
