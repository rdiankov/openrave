// -*- coding: utf-8 --*
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
#include <boost/bind.hpp>

class BodyConfigurationSampler : public SpaceSamplerBase
{
public:
    BodyConfigurationSampler(EnvironmentBasePtr penv, std::istream& sinput) : SpaceSamplerBase(penv)
    {
        __description = ":Interface Author: Rosen Diankov\n\n\
Samples a kinbody configuration space, treats revolute and circular joints appropriately. When creating pass the following parameters::\n\n\
  RobotConfiguration [robot name] [sampler name]\n\n\
The sampler needs to return values in the range [0,1]. Default sampler is 'mt19937'.\n\
By default will sample the entire body space, can you 'SetDOFs' command to set a new set of dof indices\n\
";
        RegisterCommand("SetDOFs",boost::bind(&BodyConfigurationSampler::SetDOFsCommand,this,_1,_2),
                        "set new indices to sample from.");
        string name;
        sinput >> name;
        _pbody = GetEnv()->GetKinBody(name);
        BOOST_ASSERT(!!_pbody);
        _dofindices.resize(_pbody->GetDOF());
        for(int i = 0; i < _pbody->GetDOF(); ++i) {
            _dofindices[i] = i;
        }
        string samplername;
        sinput >> samplername;
        if( samplername.size() == 0 ) {
            samplername = "mt19937";
        }
        _psampler = RaveCreateSpaceSampler(penv,samplername);
        if( !!_psampler && !!_pbody ) {
            _UpdateDOFs();
            vector<dReal> vsamplerlower, vsamplerupper;
            _psampler->GetLimits(vsamplerlower, vsamplerupper);
            for(int i = 0; i < GetDOF(); ++i) {
                BOOST_ASSERT(vsamplerlower[i] == 0 && vsamplerupper[i] == 1);
            }
        }
    }

    void SetSeed(uint32_t seed) {
        _psampler->SetSeed(seed);
    }

    void SetSpaceDOF(int dof) {
        BOOST_ASSERT(dof==(int)_lower.size());
    }
    int GetDOF() const {
        return (int)_lower.size();
    }
    int GetNumberOfValues() const {
        return (int)_lower.size();
    }
    bool Supports(SampleDataType type) const {
        return !!_pbody && !!_psampler && type==SDT_Real;
    }

    void GetLimits(std::vector<dReal>& vLowerLimit, std::vector<dReal>& vUpperLimit) const
    {
        vLowerLimit = _lower;
        vUpperLimit = _upper;
    }

    void SampleSequence(std::vector<dReal>& samples, size_t num=1,IntervalType interval=IT_Closed)
    {
        _psampler->SampleSequence(samples,num,interval);
        for (size_t inum = 0; inum < num*_lower.size(); inum += _lower.size()) {
            for (size_t i = 0; i < _lower.size(); i++) {
                if( _viscircular[i] ) {
                    samples[inum+i] = -PI + 2*PI*samples[inum+i];
                }
                else {
                    samples[inum+i] = _lower[i] + samples[inum+i]*_range[i];
                }
            }
        }
    }

protected:

    bool SetDOFsCommand(ostream& sout, istream& sinput)
    {
        std::vector<int> dofindices((istream_iterator<int>(sinput)), istream_iterator<int>());
        for(std::vector<int>::iterator it = dofindices.begin(); it != dofindices.end(); ++it) {
            if( *it < 0 || *it >= _pbody->GetDOF() ) {
                return false;
            }
        }
        _dofindices = dofindices;
        _UpdateDOFs();
        return true;
    }

    void _UpdateDOFs()
    {
        _pbody->GetDOFLimits(_lower, _upper,_dofindices);
        _range.resize(_lower.size());
        for(size_t i = 0; i < _range.size(); ++i) {
            _range[i] = _upper[i] - _lower[i];
        }
        _psampler->SetSpaceDOF(_dofindices.size());
        _viscircular.resize(_dofindices.size(),0);
        for(size_t i = 0; i < _dofindices.size(); ++i) {
            KinBody::JointPtr pjoint = _pbody->GetJointFromDOFIndex(_dofindices[i]);
            _viscircular[i] = pjoint->IsCircular(_dofindices[i]-pjoint->GetDOFIndex());
        }
    }

    SpaceSamplerBasePtr _psampler;
    KinBodyPtr _pbody;
    std::vector<int> _dofindices;
    std::vector<dReal> _lower, _upper, _range, _rangescaled;
    std::vector<dReal> _tempsamples;
    std::vector<uint8_t> _viscircular;
};
