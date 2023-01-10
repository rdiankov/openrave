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

    Automatically included with \ref openrave.h
 */
#ifndef OPENRAVE_SPACESAMPLER_H
#define OPENRAVE_SPACESAMPLER_H

namespace OpenRAVE {

/// \brief The lower sixteen bits specify the boundary confitions of intervals for sampling. The upper sixteen bits
/// specify interpolation mode (currently specifically for the Check function).
enum IntervalType {
    // Interval type (lower sixteen bits)
    IT_Open=0, ///< (a,b)
    IT_OpenStart=1, ///< (a,b]
    IT_OpenEnd=2, ///< [a,b)
    IT_Closed=3, ///< [a,b]
    IT_IntervalMask=0xffff,
    // Interpolation mode (higher sixteen bits)
    IT_Default=0x00000000,
    IT_AllLinear=0x00010000,
    IT_Cubic=0x00020000,
    IT_Quintic=0x00040000,
    IT_InterpolationMask=0xffff0000,
};

enum SampleDataType {
    SDT_Real=1,
    SDT_Uint32=2,
};

typedef boost::function<int (std::vector<dReal>&,const std::vector<dReal>&, int)> NeighStateFn;

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
        \return the number of samples completed or an error code. Error codes are <= 0.
     */
    virtual int SampleSequence(std::vector<dReal>& samples, size_t num=1,IntervalType interval=IT_Closed) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \brief samples the real next value on the sequence, only valid for 1 DOF sequences.
    ///
    /// \throw openrave_exception throw if could not be sampled
    virtual dReal SampleSequenceOneReal(IntervalType interval=IT_Closed)
    {
        OPENRAVE_ASSERT_OP_FORMAT0(GetDOF(),==,1,"sample can only be 1 dof", ORE_InvalidState);
        std::vector<dReal> samples(1);
        // by default, use SampleSequence
        SampleSequence(samples,1,interval);
        return samples.at(0);
    }

    /** \brief sequentially sampling returning the next 'num' samples

        The sampler can fail by returning an array of size 0.
        \param sample the values of the samples. This is a num*GetNumberOfValues() array.
        \param num number of samples to return
        \return the number of samples completed or an error code. Error codes are <= 0.
     */
    virtual int SampleSequence(std::vector<uint32_t>& sample, size_t num=1) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \brief samples the unsigned integer next value on the sequence, only valid for 1 DOF sequences.
    ///
    /// \throw openrave_exception throw if could not be sampled
    virtual uint32_t SampleSequenceOneUInt32()
    {
        OPENRAVE_ASSERT_OP_FORMAT0(GetDOF(),==,1,"sample can only be 1 dof", ORE_InvalidState);
        std::vector<uint32_t> samples(1);
        // by default, use SampleSequence
        SampleSequence(samples,1);
        return samples.at(0);
    }

    /// \brief returns N samples that best approximate the entire sampling space.
    ///
    /// The sampler can fail by returning an array of size 0.
    /// \return the number of samples completed or an error code. Error codes are <= 0.
    virtual int SampleComplete(std::vector<dReal>& samples, size_t num,IntervalType interval=IT_Closed) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \brief returns N samples that best approximate the entire sampling space.
    ///
    /// The sampler can fail by returning an array of size 0.
    /// \return the number of samples completed or an error code. Error codes are <= 0.
    virtual int SampleComplete(std::vector<uint32_t>& samples, size_t num) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \brief Sets a distance metric for measuring samples. Used when computing neighborhood sampling
    //virtual void SetDistanceMetric(const boost::function<dReal(const std::vector<dReal>&, const std::vector<dReal>&)>& distmetricfn) OPENRAVE_DUMMY_IMPLEMENTATION;

    /// \brief Sets a function for computing a neighboring state of a given sample that satisfies constraints.
    virtual void SetNeighStateFn(const NeighStateFn& neighstatefn) OPENRAVE_DUMMY_IMPLEMENTATION;

    /** \brief Callback function during sampling

        \param sampleiteration the sampling iteration of the planner (shows how far the planner has gone)
        \return return value can be processed by the sampler to modify its behavior. the meaning is user defined.
     */
    typedef boost::function<int (int)> StatusCallbackFn;

    /** \brief register a function that is called periodically during sampling

        Callback can be used to periodically send status messages from the sampler's thread. It can also throw an exception to "cancel" the sampler if it takes too long.
     */
    virtual UserDataPtr RegisterStatusCallback(const StatusCallbackFn& callbackfn);

protected:
    inline SpaceSamplerBasePtr shared_sampler() {
        return boost::static_pointer_cast<SpaceSamplerBase>(shared_from_this());
    }
    inline SpaceSamplerBaseConstPtr shared_sampler_const() const {
        return boost::static_pointer_cast<SpaceSamplerBase const>(shared_from_this());
    }

    /// \brief Calls the registered callbacks in order, returns the bitwise OR of all the functions.
    virtual int _CallStatusFunctions(int sampleiteration);

private:
    std::list<UserDataWeakPtr> __listRegisteredCallbacks; ///< internally managed callbacks

    friend class CustomSamplerCallbackData;
};

} // end namespace OpenRAVE

#endif
