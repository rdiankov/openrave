// -*- coding: utf-8 --*
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

class RobotConfigurationSampler : public SpaceSamplerBase
{
public:
    RobotConfigurationSampler(EnvironmentBasePtr penv, std::istream& sinput) : SpaceSamplerBase(penv)
    {
        __description = ":Interface Author: Rosen Diankov\n\n\
Samples the robot active configuration space, treats revolute and circular joints appropriately. When creating pass the following parameters::\n\n\
  RobotConfiguration [robot name] [sampler name]\n\n\
The default sampler is 'halton'.\n\
";
        string robotname;
        sinput >> robotname;
        _probot = GetEnv()->GetRobot(robotname);
        if( !!_probot ) {
            _probot->GetActiveDOFWeights(_weights);
            _probot->GetActiveDOFLimits(_lower, _upper);
            _range.resize(_lower.size());
            for(int i = 0; i < (int)_range.size(); ++i) {
                _range[i] = _upper[i] - _lower[i];
            }
        }
        else {
            RAVELOG_WARN(str(boost::format("failed to find robot '%s'\n")%robotname));
        }
        string samplername;
        sinput >> samplername;
        if( samplername.size() == 0 ) {
            samplername = "MT19937";
        }
        _psampler = RaveCreateSpaceSampler(penv,samplername);
        if( !!_psampler && !!_probot ) {
            _psampler->SetSpaceDOF(_lower.size());
        }
    }

    void SetSeed(uint32_t seed) {
        _psampler->SetSeed(seed);
    }

    void SetSpaceDOF(int dof) { BOOST_ASSERT(dof==(int)_lower.size()); }
    int GetDOF() const { return (int)_lower.size(); }
    int GetNumberOfValues() const { return (int)_lower.size(); }

    void SampleSequence(std::vector<dReal>& samples, size_t num=1,IntervalType interval=IT_Closed)
    {
        _psampler->SampleSequence(samples,num,interval);
        if( samples.size() > 0 ) {
            BOOST_ASSERT(samples.size()==num*_lower.size());
            for (size_t inum = 0; inum < num*_lower.size(); inum += _lower.size()) {
                for (size_t i = 0; i < _lower.size(); i++) {
                    samples[inum+i] = _lower[i] + samples[inum+i]*_range[i];
                }
            }
        }
    }

    void SampleComplete(std::vector<dReal>& samples, size_t num,IntervalType interval=IT_Closed)
    {
        _psampler->SampleComplete(samples,num,interval);
        if( samples.size() > 0 ) {
            BOOST_ASSERT(samples.size()==num*_lower.size());
            for (size_t inum = 0; inum < num*_lower.size(); inum += _lower.size()) {
                for (size_t i = 0; i < _lower.size(); i++) {
                    samples[inum+i] = _lower[i] + samples[inum+i]*_range[i];
                }
            }
        }
    }

 protected:
    SpaceSamplerBasePtr _psampler;
    RobotBasePtr _probot;
    vector<dReal> _weights;
    vector<dReal> _lower, _upper, _range;
};
