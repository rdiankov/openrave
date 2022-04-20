// -*- coding: utf-8 -*-
// Copyright (C) 2006-2022 Rosen Diankov <rosen.diankov@gmail.com>
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
#ifndef OPENRAVEPY_INTERNAL_SPACESAMPLER_H
#define OPENRAVEPY_INTERNAL_SPACESAMPLER_H

#define NO_IMPORT_ARRAY
#include <openravepy/openravepy_int.h>

namespace openravepy {
using py::object;

class OPENRAVEPY_API PySpaceSamplerBase : public PyInterfaceBase
{
protected:
    SpaceSamplerBasePtr _pspacesampler;
public:
    PySpaceSamplerBase(SpaceSamplerBasePtr pspacesampler, PyEnvironmentBasePtr pyenv);
    virtual ~PySpaceSamplerBase();

    SpaceSamplerBasePtr GetSpaceSampler();

    void SetSeed(uint32_t seed);
    void SetSpaceDOF(int dof);
    int GetDOF();
    int GetNumberOfValues();
    bool Supports(SampleDataType type);
    object GetLimits(SampleDataType type);
    object SampleSequence(SampleDataType type, size_t num, int interval = IntervalType::IT_Closed);
    object SampleSequence2D(SampleDataType type, size_t num, int interval = IntervalType::IT_Closed);
    dReal SampleSequenceOneReal(int interval = IntervalType::IT_Closed);
    uint32_t SampleSequenceOneUInt32();
    object SampleComplete(SampleDataType type, size_t num, int interval = IntervalType::IT_Closed);
    object SampleComplete2D(SampleDataType type, size_t num, int interval = IntervalType::IT_Closed);
protected:
    object _ReturnSamples2D(const std::vector<dReal>&samples);
    object _ReturnSamples2D(const std::vector<uint32_t>&samples);
};


} // namespace openravepy

#endif // OPENRAVEPY_INTERNAL_SPACESAMPLER_H
