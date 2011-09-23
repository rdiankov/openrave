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
/** \file spacesampler.h
    \brief Sampling definitions.
 */
#ifndef OPENRAVE_SPACESAMPLER_H
#define OPENRAVE_SPACESAMPLER_H

namespace OpenRAVE {

/// \brief Specifies the boundary conditions of intervals for sampling
enum IntervalType {
    IT_Open=0, ///< (a,b)
    IT_OpenStart=1, ///< (a,b]
    IT_OpenEnd=2, ///< [a,b)
    IT_Closed=3, ///< [a,b]
};

enum SampleDataType {
    SDT_Real=1,
    SDT_Uint32=2,
};

/** \brief <b>[interface]</b> Contains space samplers commonly used in planners. <b>If not specified, method is not multi-thread safe.</b> See \ref arch_spacesampler.
    \ingroup interfaces
 */
class OPENRAVE_API SpaceSamplerBase :  public InterfaceBase
{
public:
    SpaceSamplerBase(EnvironmentBasePtr penv) : InterfaceBase(PT_SpaceSampler, penv) {
    }
    virtual ~SpaceSamplerBase() {
    }

    /// \brief return the static interface type this class points to (used for safe casting)
    static inline InterfaceType GetInterfaceTypeStatic() {
        return PT_SpaceSampler;
    }

    /// \brief sets a new seed. For sequence samplers, the seed describes the n^th sample to begin at.
    virtual void SetSeed(uint32_t seed) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \brief Sets the degrees of freedom of the space (note this is different from the parameterization dimension)
    virtual void SetSpaceDOF(int dof) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \brief returns the degrees of freedom of the sampling space
    virtual int GetDOF() const = 0;

    /** \brief Dimension of the return samples.

        Number of values used to represent the parameterization of the space (>= dof).
        For example, let a quaternion describe a 3D rotation. The DOF of the space is 3, while the dimension of the returned samples is 4.
     */
    virtual int GetNumberOfValues() const = 0;

    virtual bool Supports(SampleDataType type) const = 0;

    /// \brief returns the minimum and maximum values returned for each dimension (size is GetNumberOfValues())
    ///
    ///  By default the limits should be in [0,1]^N.
    virtual void GetLimits(std::vector<dReal>& vLowerLimit, std::vector<dReal>& vUpperLimit) const OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \brief returns the minimum and maximum values returned for each dimension (size is GetNumberOfValues())
    ///
    /// By default the limits should be [0,2^32-1]
    virtual void GetLimits(std::vector<uint32_t>& vLowerLimit, std::vector<uint32_t>& vUpperLimit) const OPENRAVE_DUMMY_IMPLEMENTATION;

    /** \brief sequentially sampling returning the next 'num' samples

        The sampler can fail by returning an array of size 0.
        \param sample the values of the samples. This is a num*GetNumberOfValues() array.
        \param num number of samples to return
        \param interval the sampling intervel for each of the dimensions.
     */
    virtual void SampleSequence(std::vector<dReal>& samples, size_t num=1,IntervalType interval=IT_Closed) OPENRAVE_DUMMY_IMPLEMENTATION;

    /** \brief sequentially sampling returning the next 'num' samples

        The sampler can fail by returning an array of size 0.
        \param sample the values of the samples. This is a num*GetNumberOfValues() array.
        \param num number of samples to return
     */
    virtual void SampleSequence(std::vector<uint32_t>& sample, size_t num=1) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \brief returns N samples that best approximate the entire sampling space.
    ///
    /// The sampler can fail by returning an array of size 0.
    virtual void SampleComplete(std::vector<dReal>& samples, size_t num,IntervalType interval=IT_Closed) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \brief returns N samples that best approximate the entire sampling space.
    ///
    /// The sampler can fail by returning an array of size 0.
    virtual void SampleComplete(std::vector<uint32_t>& samples, size_t num) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \brief Sets a distance metric for measuring samples. Used when computing neighborhood sampling
    //virtual void SetDistanceMetric(const boost::function<dReal(const std::vector<dReal>&, const std::vector<dReal>&)>& distmetricfn) OPENRAVE_DUMMY_IMPLEMENTATION;


private:
    virtual const char* GetHash() const {
        return OPENRAVE_SPACESAMPLER_HASH;
    }
};

} // end namespace OpenRAVE

#endif
