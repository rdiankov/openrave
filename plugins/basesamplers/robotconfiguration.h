// -*- coding: utf-8 --*
// Copyright (C) 2006-2020 Rosen Diankov <rosen.diankov@gmail.com>
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

class RobotConfigurationSampler : public SpaceSamplerBase
{
public:
    RobotConfigurationSampler(EnvironmentBasePtr penv, std::istream& sinput) : SpaceSamplerBase(penv)
    {
        __description = ":Interface Author: Rosen Diankov\n\n\
Samples the robot active configuration space, treats revolute and circular joints appropriately. When creating pass the following parameters::\n\n\
  RobotConfiguration [robot name] [sampler name]\n\n\
The sampler needs to return values in the range [0,1]. Default sampler is 'mt19937'.\n\
If the robot active DOFs change, can use the 'TrackActiveSpace' command to automatically update the sampling configuration space. By default this is true.\n\
";
        RegisterCommand("TrackActiveSpace",boost::bind(&RobotConfigurationSampler::TrackActiveSpaceCommand,this,_1,_2),
                        "Enable/disable the automating updating of the active configuration space. Disabled by default.");
        string robotname;
        sinput >> robotname;
        _probot = GetEnv()->GetRobot(robotname);
        string samplername;
        sinput >> samplername;
        if( samplername.size() == 0 ) {
            samplername = "mt19937";
        }
        _psampler = RaveCreateSpaceSampler(penv,samplername);
        if( !!_psampler && !!_probot ) {
            _UpdateDOFs();
            vector<dReal> vsamplerlower, vsamplerupper;
            _psampler->GetLimits(vsamplerlower, vsamplerupper);
            for(int i = 0; i < GetDOF(); ++i) {
                BOOST_ASSERT(vsamplerlower[i] == 0 && vsamplerupper[i] == 1);
            }
        }
        // Disable active space tracking by default. When RobotConfigurationSampler is used as a sampler in a planner
        // parameters' sample function, it should not automatically update robot's lower/upper limits and only use the
        // lower/upper limits as initialized in the constructor, which are supposed to be the same as parameters'
        // _vConfigLowerLimit and _vConfigUpperLimit. When parameters' limits change, should recreate a sampler.
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
        return !!_probot && !!_psampler && type==SDT_Real;
    }

    void GetLimits(std::vector<dReal>& vLowerLimit, std::vector<dReal>& vUpperLimit) const
    {
        vLowerLimit = _lower;
        vUpperLimit = _upper;
    }

    int SampleSequence(std::vector<dReal>& samples, size_t num=1,IntervalType interval=IT_Closed)
    {
        _psampler->SampleSequence(samples,num,interval);
        for (size_t inum = 0; inum < num*_lower.size(); inum += _lower.size()) {
            for (size_t i = 0; i < _lower.size(); i++) {
                if( _viscircular[i] || (int)i == _affinerotaxis ) {
                    samples[inum+i] = -PI + 2*PI*samples[inum+i];
                }
                else if( _affinerot3d >= 0 && (int)i >= _affinerot3d && (int)i < _affinerot3d+3 ) {
                    if( (int)i == _affinerot3d ) {
                        Vector axisangle = axisAngleFromQuat(_SampleQuaternion());
                        samples[inum+i+0] = axisangle[0];
                        samples[inum+i+1] = axisangle[1];
                        samples[inum+i+2] = axisangle[2];
                    }
                }
                else if( _affinequat >= 0 && (int)i >= _affinequat && (int)i < _affinequat+3 ) {
                    if( (int)i == _affinequat ) {
                        Vector quat = _SampleQuaternion();
                        samples[inum+i+0] = quat[0];
                        samples[inum+i+1] = quat[1];
                        samples[inum+i+2] = quat[2];
                        samples[inum+i+3] = quat[3];
                    }
                }
                else {
                    samples[inum+i] = _lower[i] + samples[inum+i]*_range[i];
                }
            }
        }
        return (int)num;
    }

protected:

    bool TrackActiveSpaceCommand(ostream& sout, istream& sinput)
    {
        bool btrack = false;
        sinput >> btrack;
        if( !sinput ) {
            return false;
        }

        if( !!_probot && btrack ) {
            if( !_updatedofscallback ) {
                _updatedofscallback = _probot->RegisterChangeCallback(RobotBase::Prop_RobotActiveDOFs,boost::bind(&RobotConfigurationSampler::_UpdateDOFs,this));
            }
        }
        else {
            _updatedofscallback.reset();
        }
        return true;
    }

    Vector _SampleQuaternion()
    {
        _tempsamples.resize(4);
        Vector v;
        while(1) {
            _psampler->SampleSequence(_tempsamples,4,IT_Closed);
            v.x = v.y = v.z = v.w = 2*_tempsamples[0]-1;
            dReal flen = v.lengthsqr4();
            if( flen > 1 ) {
                continue;
            }
            flen = 1.0f/RaveSqrt(flen);
            return v*(1.0f/RaveSqrt(flen));
        }
        return Vector();
    }

    void _UpdateDOFs()
    {
        _probot->GetActiveDOFLimits(_lower, _upper);
        _range.resize(_lower.size());
        for(size_t i = 0; i < _range.size(); ++i) {
            _range[i] = _upper[i] - _lower[i];
        }
        _viscircular.resize(0); _viscircular.resize(GetDOF(),0);
        const std::vector<int>& activeDOFIndices = _probot->GetActiveDOFIndices();
        for(size_t i = 0; i < activeDOFIndices.size(); ++i) {
            int dof = activeDOFIndices[i];
            KinBody::JointPtr pjoint = _probot->GetJointFromDOFIndex(dof);
            _viscircular[i] = pjoint->IsCircular(dof-pjoint->GetDOFIndex());
        }
        _affinerotaxis = -1;
        _affinerot3d = -1;
        _affinequat = -1;

        if( _probot->GetAffineDOF() & DOF_RotationAxis) {
            _affinerotaxis = _probot->GetActiveDOFIndices().size()+RaveGetIndexFromAffineDOF(_probot->GetAffineDOF(),DOF_RotationAxis);
        }
        if( _probot->GetAffineDOF() & DOF_Rotation3D) {
            _affinerot3d = _probot->GetActiveDOFIndices().size()+RaveGetIndexFromAffineDOF(_probot->GetAffineDOF(),DOF_Rotation3D);
        }
        if( _probot->GetAffineDOF() & DOF_RotationQuat) {
            _affinequat = _probot->GetActiveDOFIndices().size()+RaveGetIndexFromAffineDOF(_probot->GetAffineDOF(),DOF_RotationQuat);
        }

        if( _lower.size() > 0 ) {
            _psampler->SetSpaceDOF(_lower.size());
        }
    }

    SpaceSamplerBasePtr _psampler;
    RobotBasePtr _probot;
    UserDataPtr _updatedofscallback;
    std::vector<dReal> _lower, _upper, _range, _rangescaled;
    std::vector<dReal> _tempsamples;
    std::vector<uint8_t> _viscircular;
    int _affinerotaxis, _affinerot3d, _affinequat;
};
