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
#include <openrave/openrave.h>

using namespace OpenRAVE;

class BodyConfigurationSampler : public SpaceSamplerBase
{
public:
    BodyConfigurationSampler(EnvironmentBasePtr penv, std::istream& sinput);

    void SetSeed(uint32_t seed) override;

    void SetSpaceDOF(int dof) override;
    int GetDOF() const override;
    int GetNumberOfValues() const override;
    bool Supports(SampleDataType type) const override;

    void GetLimits(std::vector<dReal>& vLowerLimit, std::vector<dReal>& vUpperLimit) const override;

    int SampleSequence(std::vector<dReal>& samples, size_t num=1,IntervalType interval=IT_Closed) override;

protected:

    bool SetDOFsCommand(std::ostream& sout, std::istream& sinput);

    void _UpdateDOFs();

    SpaceSamplerBasePtr _psampler;
    KinBodyPtr _pbody;
    std::vector<int> _dofindices;
    std::vector<dReal> _lower, _upper, _range, _rangescaled;
    std::vector<dReal> _tempsamples;
    std::vector<uint8_t> _viscircular;
};
