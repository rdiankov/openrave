// -*- coding: utf-8 --*
// Copyright (C) 2020 Puttichai Lertkultanon <puttichai.lertkultanon@mujin.co.jp>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <openraveplugindefs.h>

#ifdef OPENRAVE_HAS_LAPACK
// for jacobians
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/vector_proxy.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/triangular.hpp>
#include <boost/numeric/bindings/traits/ublas_matrix.hpp>
#include <boost/numeric/bindings/lapack/gesdd.hpp>
#endif

#include "configurationjitterer.h"

namespace configurationcache {

class WorkspaceConfigurationJitterer : public SpaceSamplerBase
{
public:
    /// \param parameters The planner parameters used to define the configuration space to jitter. The following fields are required: _getstatefn, _setstatefn, _vConfigUpperLimit, _vConfigLowerLimit, _checkpathvelocityconstraintsfn, _diffstatefn, _nRandomGeneratorSeed, _samplefn. The following are used and optional : _neighstatefn (used for constraining on manifolds)
    WorkspaceConfigurationJitterer(EnvironmentBasePtr penv, std::istream& is) : SpaceSamplerBase(penv)
    {
        __description = ":Interface Author: Puttichai Lertkultanon\n\n\
If the current robot configuration is in collision, then jitters the robot until it is out of collision.\n\
By default will sample the robot's active DOFs. Parameters part of the interface name::\n\
\n\
  [robotname] [samplername]\n\
\n\
";
        RegisterCommand("SetMaxJitter", boost::bind(&WorkspaceConfigurationJitterer::SetMaxJitterCommand, this, _1, _2),
                        "sets a new max jitter");
        RegisterCommand("SetMaxIterations", boost::bind(&WorkspaceConfigurationJitterer::SetMaxIterationsCommand, this, _1, _2),
                        "sets a new max iterations");
        RegisterCommand("SetMaxLinkDistThresh", boost::bind(&WorkspaceConfigurationJitterer::SetMaxLinkDistThreshCommand, this, _1, _2),
                        "sets a new max link dist threshold");
        RegisterCommand("SetPerturbation", boost::bind(&WorkspaceConfigurationJitterer::SetPerturbationCommand, this, _1, _2),
                        "sets a new perturbation");
        RegisterCommand("SetResultOnRobot", boost::bind(&WorkspaceConfigurationJitterer::SetResultOnRobotCommand, this, _1, _2),
                        "sets a new result on a robot");
        RegisterCommand("SetNeighDistThresh", boost::bind(&WorkspaceConfigurationJitterer::SetNeighDistThreshCommand, this, _1, _2),
                        "sets the minimum distance that nodes can be with respect to each other for the cache");
        RegisterCommand("SetConstraintToolDirection", boost::bind(&WorkspaceConfigurationJitterer::SetConstraintToolDirectionCommand, this, _1, _2),
                        "constrains an axis of the manipulator around a cone. manipname + 7 values: vManipDir, vGlobalDir, fCosAngleThresh.");
        RegisterCommand("SetConstraintToolPosition", boost::bind(&WorkspaceConfigurationJitterer::SetConstraintToolPositionCommand, this, _1, _2),
                        "constrains the position of the manipulator around an obb: right, up, dir, pos, extents");
        RegisterCommand("SetResetIterationsOnSample",boost::bind(&WorkspaceConfigurationJitterer::SetResetIterationsOnSampleCommand, this, _1, _2),
                        "sets the _bResetIterationsOnSample: whether or not to reset _nNumIterations every time Sample is called.");
        RegisterCommand("SetNullSpaceSamplingProb",boost::bind(&WorkspaceConfigurationJitterer::SetNullSpaceSamplingProbCommand, this, _1, _2),
                        "sets the probability to add perturbations from the nullspace of the Jacobian.");
        RegisterCommand("SetNeighStateOptions", boost::bind(&WorkspaceConfigurationJitterer::SetNeighStateOptionsCommand, this, _1, _2),
                        "sets a flag to use with NeighStateFn");
        RegisterJSONCommand("GetFailuresCount", boost::bind(&WorkspaceConfigurationJitterer::GetFailuresCountCommand, this, _1, _2, _3),
                            "Gets the numbers of failing jittered configurations from the latest call categorized based on the failure reasons.");
        RegisterJSONCommand("GetCurrentParameters", boost::bind(&WorkspaceConfigurationJitterer::GetCurrentParametersCommand, this, _1, _2, _3),
                            "Gets the current values of parameters.");

#ifndef OPENRAVE_HAS_LAPACK
        throw OPENRAVE_EXCEPTION_FORMAT0(_("cannot use WorkspaceConfigurationJitterer since lapack is not supported"), ORE_CommandNotSupported);
#else
        bool bUseCache = false;
        std::string robotname, manipname, samplername = "MT19937";
        is >> robotname >> manipname >> samplername >> bUseCache;
        _probot = GetEnv()->GetRobot(robotname);
        OPENRAVE_ASSERT_FORMAT(!!_probot, "env=%s, could not find robot %s", GetEnv()->GetNameId()%robotname, ORE_InvalidArguments);
        _pmanip = _probot->GetManipulator(manipname);
        OPENRAVE_ASSERT_FORMAT(!!_pmanip, "env=%s, could not find manip %s in robot %s", GetEnv()->GetNameId()%manipname%robotname, ORE_InvalidArguments);

        _vActiveIndices = _probot->GetActiveDOFIndices();
        _nActiveAffineDOFs = _probot->GetAffineDOF();
        _vActiveAffineAxis = _probot->GetAffineRotationAxis();
        _vLinks.reserve(_probot->GetLinks().size());
        if( _nActiveAffineDOFs == 0 ) {
            for (size_t ilink = 0; ilink < _probot->GetLinks().size(); ++ilink) {
                if( _probot->GetLinks()[ilink]->GetGeometries().empty() ) {
                    // Links that don't have geometries (virtual links) don't matter for jittering. Their AABBs can interfere with the results.
                    continue;
                }
                for (int dofindex : _vActiveIndices) {
                    if( _probot->DoesAffect(_probot->GetJointFromDOFIndex(dofindex)->GetJointIndex(), ilink)) {
                        _vLinks.push_back(_probot->GetLinks()[ilink]);
                        break;
                    }
                }
            }
        }
        else {
            for (const KinBody::LinkPtr& plink: _probot->GetLinks()) {
                if( plink->GetGeometries().empty() ) {
                    // Links that don't have geometries (virtual links) don't matter for jittering. Their AABBs can interfere with the results.
                    continue;
                }
                _vLinks.push_back(plink);
            }
        }
        _vLinkAABBs.resize(_vLinks.size());
        for(size_t i = 0; i < _vLinks.size(); ++i) {
            _vLinkAABBs[i] = _vLinks[i]->ComputeLocalAABB();
        }

        std::vector<dReal> vweights;
        _probot->GetActiveDOFResolutions(vweights);

        // if weights are zero, used a default value
        FOREACH(itweight, vweights) {
            if( *itweight > 0 ) {
                *itweight = 1 / *itweight;
            }
            else {
                *itweight = 100;
            }
        }

        if( bUseCache ) {
            _cache.reset(new CacheTree(_probot, _probot->GetActiveDOF()));
            _cache->Init(vweights, 1);
        }

        _bSetResultOnRobot = true;
        _bResetIterationsOnSample = true;

        // for selecting sampling modes
        if( samplername.size() == 0 ) {
            samplername = "mt19937";
        }
        _ssampler = RaveCreateSpaceSampler(penv, samplername);
        OPENRAVE_ASSERT_FORMAT(!!_ssampler, "env=%s, sampler %s not found", GetEnv()->GetNameId()%samplername, ORE_InvalidArguments);
        _ssampler->SetSpaceDOF(1);

        size_t dof = _probot->GetActiveDOF();

        // use for sampling, perturbations
        _curdof.resize(dof,0);
        _newdof2.resize(dof);
        _deltadof.resize(dof);
        _deltadof2.resize(dof);
        _nRandomGeneratorSeed = 0;
        _nNumIterations = 0;

        _report.reset(new CollisionReport());
        _maxiterations = 5000;
        _maxjitter = 0.02;
        _fPerturbation = 1e-5;
        _linkdistthresh = 0.02;
        _linkdistthresh2 = _linkdistthresh*_linkdistthresh;
        _neighdistthresh = 1;
        _neighstateoptions = 0;

        _UpdateLimits();
        _limitscallback = _probot->RegisterChangeCallback(RobotBase::Prop_JointLimits, boost::bind(&WorkspaceConfigurationJitterer::_UpdateLimits, this));
        _UpdateGrabbed();
        _grabbedcallback = _probot->RegisterChangeCallback(RobotBase::Prop_RobotGrabbed, boost::bind(&WorkspaceConfigurationJitterer::_UpdateGrabbed, this));

        if( !!_cache ) {
            _SetCacheMaxDistance();
        }

        _mults = {0, -1, 1};

#endif // OPENRAVE_HAS_LAPACK
    }

    virtual ~WorkspaceConfigurationJitterer(){
    }

    virtual void SetSeed(uint32_t seed) {
        _nRandomGeneratorSeed = seed;
        _nNumIterations = 0;
        _ssampler->SetSeed(seed);
    }

    virtual int GetDOF() const {
        return (int)_lower.size();
    }

    virtual int GetNumberOfValues() const {
        return (int)_lower.size();
    }

    virtual bool Supports(SampleDataType type) const {
        return type == SDT_Real;
    }

    virtual void GetLimits(std::vector<dReal>& vLowerLimit, std::vector<dReal>& vUpperLimit) const {
        vLowerLimit = _lower;
        vUpperLimit = _upper;
    }

    virtual void GetLimits(std::vector<uint32_t>& vLowerLimit, std::vector<uint32_t>& vUpperLimit) const {
        BOOST_ASSERT(0);
    }

    virtual std::string GetManipulatorName() const {
        return _pmanip->GetName();
    }

    virtual bool SetMaxJitterCommand(std::ostream& sout, std::istream& sinput)
    {
        dReal maxjitter=0;
        sinput >> maxjitter;
        if( !sinput || maxjitter < 0 ) {
            return false;
        }
        _maxjitter = maxjitter;
        if( !!_cache ) {
            _SetCacheMaxDistance();
        }
        return true;
    }

    bool SetMaxIterationsCommand(std::ostream& sout, std::istream& sinput)
    {
        int maxiterations=0;
        sinput >> maxiterations;
        if( !sinput || maxiterations < 0 ) {
            return false;
        }
        _maxiterations = maxiterations;
        return true;
    }

    bool SetMaxLinkDistThreshCommand(std::ostream& sout, std::istream& sinput)
    {
        dReal linkdistthresh=0;
        sinput >> linkdistthresh;
        if( linkdistthresh < 0 || !sinput ) {
            return false;
        }
        _linkdistthresh = linkdistthresh;
        _linkdistthresh2 = _linkdistthresh*_linkdistthresh;
        return true;
    }

    bool SetPerturbationCommand(std::ostream& sout, std::istream& sinput)
    {
        dReal fPerturbation=0;
        sinput >> fPerturbation;
        if( fPerturbation < 0 ) {
            return false;
        }
        _fPerturbation = fPerturbation;
        return true;
    }

    bool SetResultOnRobotCommand(std::ostream& sout, std::istream& sinput)
    {
        int bSetResultOnRobot = 0;
        sinput >> bSetResultOnRobot;
        if( bSetResultOnRobot < 0 ) {
            return false;
        }
        _bSetResultOnRobot = bSetResultOnRobot;
        return true;
    }

    bool SetNeighDistThreshCommand(std::ostream& sout, std::istream& sinput)
    {
        dReal neighdistthresh = 0;
        sinput >> neighdistthresh;
        if( neighdistthresh <= 0 ) {
            return false;
        }
        _neighdistthresh = neighdistthresh;
        return true;
    }

    bool SetConstraintToolDirectionCommand(std::ostream& sout, std::istream& sinput)
    {
        std::string manipname;
        ManipDirectionThreshPtr thresh(new ManipDirectionThresh());
        sinput >> manipname;

        if( manipname.size() == 0 ) {
            if( !!_pConstraintToolDirection ) {
                if( !!_cache ) {
                    _cache->Reset(); // need this here in order to invalidate cache.
                }
            }
            _pConstraintToolDirection.reset();
            return true;
        }
        if( manipname != _pmanip->GetName() ) {
            RAVELOG_DEBUG_FORMAT("env=%s, the given manipname=\"%s\" is different from the currently set name \"%s\"", GetEnv()->GetNameId()%manipname%_pmanip->GetName());
            RobotBase::ManipulatorConstPtr pmanip = _probot->GetManipulator(manipname);
            if( !pmanip ) {
                RAVELOG_WARN_FORMAT("env=%s, cannot find manip %s in robot %s", GetEnv()->GetNameId()%manipname%_probot->GetName());
                return false;
            }
            _pmanip = pmanip;
        }
        sinput >> thresh->vManipDir.x >> thresh->vManipDir.y >> thresh->vManipDir.z >> thresh->vGlobalDir.x >> thresh->vGlobalDir.y >> thresh->vGlobalDir.z >> thresh->fCosAngleThresh;
        if( !sinput ) {
            RAVELOG_WARN_FORMAT("env=%s, bad command input", GetEnv()->GetNameId());
            return false;
        }

        _pConstraintToolDirection = thresh;
        if( !!_cache ) {
            _cache->Reset(); // need this here in order to invalidate cache.
        }
        return true;
    }

    bool SetConstraintToolPositionCommand(std::ostream& sout, std::istream& sinput)
    {
        std::string manipname;
        ManipPositionConstraintsPtr constraint(new ManipPositionConstraints());
        sinput >> manipname;
        if( manipname.size() == 0 ) {
            if( !!_pConstraintToolPosition ) {
                if( !!_cache ) {
                    _cache->Reset(); // need this here in order to invalidate cache.
                }
            }
            _pConstraintToolPosition.reset();
            return true;
        }
        if( manipname != _pmanip->GetName() ) {
            RAVELOG_DEBUG_FORMAT("env=%s, the given manipname=\"%s\" is different from the currently set name \"%s\"", GetEnv()->GetNameId()%manipname%_pmanip->GetName());
            RobotBase::ManipulatorConstPtr pmanip = _probot->GetManipulator(manipname);
            if( !pmanip ) {
                RAVELOG_WARN_FORMAT("env=%s, cannot find manip %s in robot %s", GetEnv()->GetNameId()%manipname%_probot->GetName());
                return false;
            }
            _pmanip = pmanip;
        }
        sinput >> constraint->obb.right.x >> constraint->obb.right.y >> constraint->obb.right.z
        >> constraint->obb.up.x >> constraint->obb.up.y >> constraint->obb.up.z
        >> constraint->obb.dir.x >> constraint->obb.dir.y >> constraint->obb.dir.z
        >> constraint->obb.pos.x >> constraint->obb.pos.y >> constraint->obb.pos.z
        >> constraint->obb.extents.x >> constraint->obb.extents.y >> constraint->obb.extents.z;
        if( !sinput ) {
            RAVELOG_WARN_FORMAT("env=%s, bad command input", GetEnv()->GetNameId());
            return false;
        }

        _pConstraintToolPosition = constraint;
        if( !!_cache ) {
            _cache->Reset(); // need this here in order to invalidate cache.
        }
        return true;
    }

    bool SetResetIterationsOnSampleCommand(std::ostream& sout, std::istream& sinput)
    {
        sinput >> _bResetIterationsOnSample;
        return !!sinput;
    }

    virtual int SampleSequence(std::vector<dReal>& samples, size_t num=1,IntervalType interval=IT_Closed)
    {
        samples.resize(0);
        for(size_t i = 0; i < num; ++i) {
            int ret = Sample(_vonesample, interval);
            if( ret == 1 ) {
                samples.insert(samples.end(), _vonesample.begin(), _vonesample.end());
            }
            else {
                return ret;
            }
        }
        return (int)num;
    }

    virtual dReal SampleSequenceOneReal(IntervalType interval) {
        BOOST_ASSERT(0);
        return 0;
    }

    virtual uint32_t SampleSequenceOneUInt32()
    {
        BOOST_ASSERT(0);
        return 0;
    }

    virtual int SampleComplete(std::vector<dReal>& samples, size_t num, IntervalType interval=IT_Closed) {
        // have to reset the seed
        _ssampler->SetSeed(_nRandomGeneratorSeed);
        _nNumIterations = 0;
        return SampleSequence(samples, num, interval);
    }

    virtual int SampleComplete(std::vector<uint32_t>& samples, size_t num) {
        BOOST_ASSERT(0);
        return 0;
    }

    virtual bool SetNullSpaceSamplingProbCommand(std::ostream& sout, std::istream& sinput)
    {
        dReal nullsampleprob = 0.60;
        sinput >> nullsampleprob;
        if( !sinput ) {
            RAVELOG_WARN_FORMAT("env=%s, bad command input", GetEnv()->GetNameId());
            return false;
        }
        _nullsampleprob = nullsampleprob;
        return true;
    }

    void SetNeighStateFn(const OpenRAVE::NeighStateFn& neighstatefn)
    {
        _neighstatefn = neighstatefn;
    }

    bool SetNeighStateOptionsCommand(std::ostream& sout, std::istream& sinput)
    {
        int neighstateoptions = 0;
        sinput >> neighstateoptions;
        if( neighstateoptions < 0 ) {
            return false;
        }
        _neighstateoptions = neighstateoptions;
        return true;
    }

    virtual bool GetFailuresCountCommand(const rapidjson::Value& input, rapidjson::Value& output, rapidjson::Document::AllocatorType& alloc)
    {
        _counter.SaveToJson(output, alloc);
        return true;
    }

    virtual bool GetCurrentParametersCommand(const rapidjson::Value& input, rapidjson::Value& output, rapidjson::Document::AllocatorType& alloc)
    {
        output.SetObject();

        orjson::SetJsonValueByKey(output, "nullSampleProb", _nullsampleprob, alloc);
        orjson::SetJsonValueByKey(output, "currentJointValues", _fulldof, alloc);
        orjson::SetJsonValueByKey(output, "maxJitter", _maxjitter, alloc);
        orjson::SetJsonValueByKey(output, "maxJitterIterations", _maxiterations, alloc);
        orjson::SetJsonValueByKey(output, "maxJitterLinkDist", _linkdistthresh, alloc);
        orjson::SetJsonValueByKey(output, "jitterPerturbation", _fPerturbation, alloc);
        orjson::SetJsonValueByKey(output, "jitterNeighDistThresh", _neighdistthresh, alloc);
        orjson::SetJsonValueByKey(output, "resetIterationsOnSample", _bResetIterationsOnSample, alloc);

        orjson::SetJsonValueByKey(output, "manipName", _pmanip->GetName(), alloc);
        rapidjson::Value rTransform;
        rTransform.SetArray();
        rTransform.Reserve(7, alloc);
        rTransform.PushBack(_tLocalTool.rot[0], alloc);
        rTransform.PushBack(_tLocalTool.rot[1], alloc);
        rTransform.PushBack(_tLocalTool.rot[2], alloc);
        rTransform.PushBack(_tLocalTool.rot[3], alloc);
        rTransform.PushBack(_tLocalTool.trans[0], alloc);
        rTransform.PushBack(_tLocalTool.trans[1], alloc);
        rTransform.PushBack(_tLocalTool.trans[2], alloc);
        orjson::SetJsonValueByKey(output, "localToolPose", rTransform, alloc);
        if( !!_pConstraintToolDirection ) {
            rapidjson::Value rConstraintToolDirection;
            _pConstraintToolDirection->SaveToJson(rConstraintToolDirection, alloc);
            orjson::SetJsonValueByKey(output, "constraintToolDirection", rConstraintToolDirection, alloc);
        }
        if( !!_pConstraintToolPosition ) {
            rapidjson::Value rConstraintToolPosition;
            _pConstraintToolPosition->SaveToJson(rConstraintToolPosition, alloc);
            orjson::SetJsonValueByKey(output, "constraintToolPosition", rConstraintToolPosition, alloc);
        }
        return true;
    }

    /// \brief Jitters the current configuration and sets a new configuration on the environment. The jittered
    ///        configuration will also be checked with small perturbations to make sure that it is not too close to
    ///        boundaries of collision constraints and tool direction constraints.
    ///
    /// \return  0 if jittering fails.
    ///          1 if a jittered configuration is produced successfully.
    ///         -1 if the original configuration does not need jittering.
    int Sample(std::vector<dReal>& vnewdof, IntervalType interval=IT_Closed)
    {
        RobotBase::RobotStateSaver robotsaver(_probot, KinBody::Save_LinkTransformation|KinBody::Save_ActiveDOF);
        if( !_InitRobotState() ) {
            return 0;
        }

        const dReal linkdistthresh = _linkdistthresh;
        const dReal linkdistthresh2 = _linkdistthresh2;

        if( _bResetIterationsOnSample ) {
            _nNumIterations = 0;
        }

        bool bCollision = false;
        bool bConstraintFailed = false;
        const bool bHasNeighStateFn = !!_neighstatefn;

        std::vector<dReal> vPerturbations; // for testing with perturbed configurations
        if( _fPerturbation > 0 ) {
            vPerturbations.resize(3, 0);
            vPerturbations[0] = _fPerturbation;
            vPerturbations[1] = -_fPerturbation;
        }
        else {
            vPerturbations.resize(1, 0);
        }

        vnewdof.resize(GetDOF());

        // count of types of failures to better give user that info
        _counter.Reset();

        if( _nNumIterations == 0 ) {
            // Check _curdof + perturbation
            FOREACH(itperturbation, vPerturbations) {
                // Perturbation is added to a config to make sure that the config is not too close to collision and tool
                // direction/position constraint boundaries. So we do not use _neighstatefn to compute perturbed
                // configurations.
                for( size_t idof = 0; idof < vnewdof.size(); ++idof ) {
                    vnewdof[idof] = _curdof[idof] + (*itperturbation);
                    if( vnewdof[idof] > _upper.at(idof) ) {
                        vnewdof[idof] = _upper[idof];
                    }
                    else if( vnewdof[idof] < _lower.at(idof) ) {
                        vnewdof[idof] = _lower[idof];
                    }
                }

                // don't need to set state since CheckPathAllConstraints does it
                _probot->SetActiveDOFValues(vnewdof);

                if( !!_pConstraintToolDirection ) {
                    if( !_pConstraintToolDirection->IsInConstraints(_pmanip->GetTransform()) ) {
                        _counter.nConstraintToolDirFailure++;
                        bConstraintFailed = true;
                        break;

                    }
                }
                if( !!_pConstraintToolPosition ) {
                    if( !_pConstraintToolPosition->IsInConstraints(_pmanip->GetTransform()) ) {
                        _counter.nConstraintToolPositionFailure++;
                        bConstraintFailed = true;
                        break;

                    }
                }
                if( GetEnv()->CheckCollision(_probot, _report) ) {
                    if( IS_DEBUGLEVEL(Level_Verbose) ) {
                        stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
                        ss << "env=" << GetEnv()->GetNameId() << ", original env collision failed colvalues=[";
                        for( size_t idof = 0; idof < vnewdof.size(); ++idof ) {
                            ss << vnewdof[idof] << ",";
                        }
                        ss << "]; report=" << _report->__str__();
                        RAVELOG_VERBOSE(ss.str());
                    }
                    _counter.nEnvCollisionFailure++;
                    bCollision = true;
                    break;
                }

                if( _probot->CheckSelfCollision(_report) ) {
                    if( IS_DEBUGLEVEL(Level_Verbose) ) {
                        stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
                        ss << "env=" << GetEnv()->GetNameId() << ", original self collision failed colvalues=[";
                        for( size_t idof = 0; idof < vnewdof.size(); ++idof ) {
                            ss << vnewdof[idof] << ",";
                        }
                        ss << "]; report=" << _report->__str__();
                        RAVELOG_VERBOSE(ss.str());
                    }
                    _counter.nSelfCollisionFailure++;
                    bCollision = true;
                    break;
                }
            }

            if( (!bCollision && !bConstraintFailed) || _maxjitter <= 0 ) {
                if( _counter.nNeighStateFailure > 0 ) {
                    RAVELOG_DEBUG_FORMAT("env=%s, initial point configuration is good, but neigh state failed %d times", GetEnv()->GetNameId()%_counter.nNeighStateFailure);
                }
                return -1;
            }

            _nNumIterations++;
        }
        else {
            //RAVELOG_VERBOSE_FORMAT("env=%s, skipping checks of the original configuration", GetEnv()->GetNameId());
            RAVELOG_DEBUG_FORMAT("env=%s, skipping checks of the original configuration", GetEnv()->GetNameId());
        }

        if( !!_cache ) {
            _cache->InsertNode(_curdof, CollisionReportPtr(), _neighdistthresh);
            _cachehit = 0;
        }

        const int nMaxIterRadiusThresh = _maxiterations/2;
        const dReal imaxiterations = 2.0/dReal(_maxiterations);

        uint64_t starttime = utils::GetNanoPerformanceTime();
        for( int iter = 0; iter < _maxiterations; ++iter ) {
            if( (iter % 10) == 0 ) {
                _CallStatusFunctions(iter);
            }
            _nNumIterations++;

            size_t idirection = _ssampler->SampleSequenceOneUInt32()%(_vvbiasdofdirections.size());
            int ix = (idirection + 1) / 9;
            int iy = ((idirection + 1)%9) / 3;
            int iz = ((idirection + 1)%9) % 3;
            Vector vCurBiasDirection = _mults[ix]*_vBasisBiasDirections[0] + _mults[iy]*_vBasisBiasDirections[1] + _mults[iz]*_vBasisBiasDirections[2];
            vCurBiasDirection /= RaveSqrt(vCurBiasDirection.lengthsqr3()); // normalize
            const std::vector<dReal>& vCurBiasDOFDirection = _vvbiasdofdirections[idirection];
            const std::vector<std::vector<dReal> >& vCurBiasNullSpace = _vvbiasnullspace[idirection];

            // Compute vnewdof = curdof + delta
            {
                dReal jitter = _maxjitter;
                // Start with lower magnitude of jitter. Keep increasing it as the number of iterations increase.
                if( iter < nMaxIterRadiusThresh ) {
                    jitter = _maxjitter*dReal(iter + 1)*imaxiterations;
                }

                bool bSampleNull = false;
                if( _ssampler->SampleSequenceOneReal() < _nullsampleprob ) {
                    bSampleNull = true;
                }

                dReal fNullSpaceMultiplier = 2*linkdistthresh;
                if( fNullSpaceMultiplier <= 0 ) {
                    fNullSpaceMultiplier = jitter;
                }

                dReal fCurJitter = jitter * _ssampler->SampleSequenceOneReal();
                for( size_t idof = 0; idof < vnewdof.size(); ++idof ) {
                    vnewdof[idof] = _curdof[idof] + fCurJitter * vCurBiasDOFDirection[idof];

                    if( bSampleNull ) {
                        for( size_t idim = 0; idim < vCurBiasNullSpace.size(); ++idim ) {
                            dReal nullx = (_ssampler->SampleSequenceOneReal()*2 - 1)*fNullSpaceMultiplier;
                            vnewdof[idof] += nullx * vCurBiasNullSpace[idim][idof];
                        }
                    }
                }
            } // computing vnewdof

            // Clamp values with limits
            for( size_t idof = 0; idof < vnewdof.size(); ++idof ) {
                if( vnewdof[idof] > _upper.at(idof) ) {
                    vnewdof[idof] = _upper[idof];
                }
                else if( vnewdof[idof] < _lower.at(idof) ) {
                    vnewdof[idof] = _lower[idof];
                }
            }

            // Compute a neighbor of _curdof that satisfies constraints.
            // If _neighstatefn is not initialized, then the neighbor is vnewdof itself.
            if( bHasNeighStateFn ) {
                // Obtain the delta dof values computed from the jittering above.
                for( size_t idof = 0; idof < _deltadof.size(); ++idof ) {
                    _deltadof[idof] = vnewdof[idof] - _curdof[idof];
                }
                vnewdof = _curdof;
                _probot->SetActiveDOFValues(vnewdof); // need to set robot configuration before calling _neighstatefn
                if( _neighstatefn(vnewdof, _deltadof, _neighstateoptions) == NSS_Failed) {
                    _counter.nNeighStateFailure++;
                    continue;
                }
            }

            if( !!_cache ) {
                if( !!_cache->FindNearestNode(vnewdof, _neighdistthresh).first ) {
                    _cachehit++;
                    _counter.nCacheHitSamples++;
                    continue;
                }
            }

            _probot->SetActiveDOFValues(vnewdof);

            bool bSuccess = true;

#ifdef _DEBUG
            dReal fmaxtransdist = 0;
#endif
            if( linkdistthresh > 0 ) {
                for (size_t ilink = 0; ilink < _vLinkAABBs.size(); ++ilink) {
                    // check for an elipse
                    // L^2 (b*v)^2 + |v|^2|b|^4 - (b*v)^2 |b|^2 <= |b|^4 * L^2
                    Transform tnewlink = _vLinks[ilink]->GetTransform();
                    TransformMatrix projdelta = _vOriginalInvTransforms[ilink] * tnewlink;
                    projdelta.m[0] -= 1;
                    projdelta.m[5] -= 1;
                    projdelta.m[10] -= 1;
                    Vector projextents = _vLinkAABBs[ilink].extents;
                    Vector projboxright(projdelta.m[0]*projextents.x, projdelta.m[4]*projextents.x, projdelta.m[8]*projextents.x);
                    Vector projboxup(projdelta.m[1]*projextents.y, projdelta.m[5]*projextents.y, projdelta.m[9]*projextents.y);
                    Vector projboxdir(projdelta.m[2]*projextents.z, projdelta.m[6]*projextents.z, projdelta.m[10]*projextents.z);
                    Vector projboxpos = projdelta * _vLinkAABBs[ilink].pos;

                    Vector b = _vOriginalInvTransforms[ilink].rotate( vCurBiasDirection ); // inside link coordinate system

                    dReal blength2 = b.lengthsqr3();
                    dReal blength4 = blength2*blength2;
                    dReal rhs = blength4 * linkdistthresh2;
                    //dReal rhs = (b.lengthsqr3()) * linkdistthresh;
                    dReal ellipdist = 0;
                    // now figure out what is the max distance
                    for( ix = 0; ix < 2; ++ix ) {
                        Vector projvx = ix > 0 ? projboxpos + projboxright : projboxpos - projboxright;
                        for( iy = 0; iy < 2; ++iy ) {
                            Vector projvy = iy > 0 ? projvx + projboxup : projvx - projboxup;
                            for( iz = 0; iz < 2; ++iz ) {
                                Vector projvz = iz > 0 ? projvy + projboxdir : projvy - projboxdir;
                                Vector v = projvz; // inside link coordinate system
                                dReal bv = (v.dot3(b));
                                dReal bv2 = bv*bv;
                                dReal flen2 = (linkdistthresh2 - blength2) * bv2 + v.lengthsqr3()*blength4;

                                if( ellipdist < flen2 ) {
                                    ellipdist = flen2;
#ifdef _DEBUG
                                    fmaxtransdist = flen2;
#endif
                                    if (ellipdist > rhs) {
                                        bSuccess = false;
                                        break;
                                    }
                                }
                            } // end for iz
                            if (ellipdist > rhs) {
                                bSuccess = false;
                                break;
                            }
                        } // end for iy
                        if (ellipdist > rhs) {
                            bSuccess = false;
                            break;
                        }
                    } // end for ix
                    if( !bSuccess ) {
                        if( IS_DEBUGLEVEL(Level_Verbose) ) {
                            stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
                            ss << "dofvalues=[";
                            for(size_t i = 0; i < vnewdof.size(); ++i ) {
                                ss << vnewdof[i];
                                if( i < vnewdof.size() - 1 ) {
                                    ss << ", ";
                                }
                            }
                            ss << "]";
                            RAVELOG_VERBOSE_FORMAT("env=%s, link %s exceeded linkdisthresh. ellipdist[%e] > rhs[%e], %s", GetEnv()->GetNameId()%_vLinks[ilink]->GetName()%ellipdist%rhs%ss.str());
                        }
                        break;
                    }
                }

                if (!bSuccess) {
                    _counter.nLinkDistThreshRejections++;
                    continue;
                }
            }

            // check perturbation
            bCollision = false;
            bConstraintFailed = false;
            FOREACH(itperturbation,vPerturbations) {
                // Perturbation is added to a config to make sure that the config is not too close to collision and tool
                // direction/position constraint boundaries. So we do not use _neighstatefn to compute perturbed
                // configurations.
                _newdof2 = vnewdof;
                for(size_t idof = 0; idof < _newdof2.size(); ++idof) {
                    _newdof2[idof] += *itperturbation;
                    if( _newdof2[idof] > _upper.at(idof) ) {
                        _newdof2[idof] = _upper.at(idof);
                    }
                    else if( _newdof2[idof] < _lower.at(idof) ) {
                        _newdof2[idof] = _lower.at(idof);
                    }
                }
                _probot->SetActiveDOFValues(_newdof2);
                if( !!_pConstraintToolDirection ) {
                    if( !_pConstraintToolDirection->IsInConstraints(_pmanip->GetTransform()) ) {
                        bConstraintFailed = true;
                        _counter.nConstraintToolDirFailure++;
                        if( IS_DEBUGLEVEL(Level_Verbose) ) {
                            stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
                            ss << "env=" << GetEnv()->GetNameId() << ", direction constraints failed colvalues=[";
                            for( size_t idof = 0; idof < _newdof2.size(); ++idof ) {
                                ss << _newdof2[idof] << ",";
                            }
                            ss << "]; cosangle=" << _pConstraintToolDirection->ComputeCosAngle(_pmanip->GetTransform());
                            ss << "; quat=[" << _pmanip->GetTransform().rot.x << ", " << _pmanip->GetTransform().rot.y << ", " << _pmanip->GetTransform().rot.z << ", " << _pmanip->GetTransform().rot.w << "]";
                            RAVELOG_VERBOSE(ss.str());
                        }
                        break;
                    }
                }
                if( !!_pConstraintToolPosition ) {
                    if( !_pConstraintToolPosition->IsInConstraints(_pmanip->GetTransform()) ) {
                        bConstraintFailed = true;
                        _counter.nConstraintToolPositionFailure++;
                        if( IS_DEBUGLEVEL(Level_Verbose) ) {
                            stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
                            ss << "env=" << GetEnv()->GetNameId() << ", position constraints failed colvalues=[";
                            for( size_t idof = 0; idof < _newdof2.size(); ++idof ) {
                                ss << _newdof2[idof] << ",";
                            }
                            ss << "]; trans=[" << _pmanip->GetTransform().trans.x << ", " << _pmanip->GetTransform().trans.y << ", " << _pmanip->GetTransform().trans.z << "]";
                            RAVELOG_VERBOSE(ss.str());
                        }
                        break;
                    }
                }

                if( GetEnv()->CheckCollision(_probot, _report) ) {
                    bCollision = true;
                    _counter.nEnvCollisionFailure++;
                }
                if( !bCollision && _probot->CheckSelfCollision(_report)) {
                    bCollision = true;
                    _counter.nSelfCollisionFailure++;
                }

                if( bCollision ) {
                    if( IS_DEBUGLEVEL(Level_Verbose) ) {
                        stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
                        ss << "env=" << GetEnv()->GetNameId() << ", collision failed colvalues=[";
                        for( size_t idof = 0; idof < _newdof2.size(); ++idof ) {
                            ss << _newdof2[idof] << ",";
                        }
                        ss << "]; report=" << _report->__str__();
                        RAVELOG_VERBOSE(ss.str());
                    }
                    break;
                }
            }

            if( !bCollision && !bConstraintFailed ) {
                // the last perturbation is 0, so state is already set to the correct jittered value
                if( IS_DEBUGLEVEL(Level_Verbose) ) {
                    _probot->GetActiveDOFValues(vnewdof);
                    stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
                    ss << "env=" << GetEnv()->GetNameId() << ", jitter iter=" << iter;
#ifdef _DEBUG
                    ss << "; maxtrans=" << fmaxtransdist;
#endif
                    ss << "; jitteredvalues=[";
                    for( size_t idof = 0; idof < vnewdof.size(); ++idof ) {
                        ss << vnewdof[idof] << ",";
                    }
                    ss << "]";
                    RAVELOG_VERBOSE(ss.str());
                }

                if( _bSetResultOnRobot ) {
                    // have to release the saver so it does not restore the old configuration
                    robotsaver.Release();
                }

                RAVELOG_DEBUG_FORMAT("env=%s, succeeded iterations=%d, computation=%fs, bHasNeighStateFn=%d, neighstate=%d, constraintToolDir=%d, constraintToolPos=%d, envCollision=%d, selfCollision=%d, nLinkDistThreshRejections=%d", GetEnv()->GetNameId()%iter%(1e-9*(utils::GetNanoPerformanceTime() - starttime))%bHasNeighStateFn%_counter.nNeighStateFailure%_counter.nConstraintToolDirFailure%_counter.nConstraintToolPositionFailure%_counter.nEnvCollisionFailure%_counter.nSelfCollisionFailure%_counter.nLinkDistThreshRejections);
                return 1;
            } // end if( !bCollision && !bConstraintFailed )

        } // end for iter

        RAVELOG_INFO_FORMAT("env=%s, failed iterations=%d (max=%d), computation=%fs, bHasNeighStateFn=%d, neighstate=%d, constraintToolDir=%d, constraintToolPos=%d, envCollision=%d, selfCollision=%d, cachehit=%d, samesamples=%d, nLinkDistThreshRejections=%d", GetEnv()->GetNameId()%_nNumIterations%_maxiterations%(1e-9*(utils::GetNanoPerformanceTime() - starttime))%bHasNeighStateFn%_counter.nNeighStateFailure%_counter.nConstraintToolDirFailure%_counter.nConstraintToolPositionFailure%_counter.nEnvCollisionFailure%_counter.nSelfCollisionFailure%_counter.nCacheHitSamples%_counter.nSameSamples%_counter.nLinkDistThreshRejections);
        return 0;
    }

protected:

    /// \brief extracts all used bodies from the configurationspecification and computes AABBs, transforms, and limits for links
    bool _InitRobotState()
    {
        _probot->SetActiveDOFs(_vActiveIndices, _nActiveAffineDOFs, _vActiveAffineAxis);
        _probot->GetActiveDOFValues(_curdof);
        _probot->GetDOFValues(_fulldof);
        _tLocalTool = _pmanip->GetLocalToolTransform();

        _vOriginalTransforms.resize(_vLinks.size());
        _vOriginalInvTransforms.resize(_vLinks.size());
        for(size_t i = 0; i < _vLinks.size(); ++i) {
            _vOriginalTransforms[i] = _vLinks[i]->GetTransform();
            _vOriginalInvTransforms[i] = _vOriginalTransforms[i].inverse();
        }

        namespace ublas = boost::numeric::ublas;

        const dReal zerothresh = 1e-7;
        size_t numdof = _pmanip->GetArmDOF();

        // First compute the Jacobian at the current configuration
        _pmanip->CalculateJacobian(_mjacobian);
        boost::numeric::ublas::matrix<double, ublas::column_major> J(3, numdof);
        for(size_t i = 0; i < 3; ++i) {
            for(size_t j = 0; j < _pmanip->GetArmIndices().size(); ++j) {
                J(i, j) = _mjacobian[i][j]; // *_viweights.at(j), will have to scale output also?, needs testing
            }
        }

        size_t workspacedim = 3;
        ublas::vector<double> S(workspacedim);
        ublas::matrix<double, ublas::column_major> U(workspacedim, workspacedim), V(numdof, numdof);
        ublas::vector<double> P(workspacedim), P2(workspacedim), P3(numdof);

        // J * dofvelocities = P
        // compute single value decomposition: Jacobian = U*diag(S)*transpose(V)
        int ret = boost::numeric::bindings::lapack::gesdd('O', 'A', J, S, U, V);
        if( ret != 0 ) {
            RAVELOG_WARN_FORMAT("env=%s, failed to compute SVD for jacobian, disabling bias", GetEnv()->GetNameId());
            return false;
        }
        bool bNullSpace = false;
        size_t istart = 0;
        for( istart = 0; istart < S.size(); ++istart ) {
            if( RaveFabs(S(istart)) < zerothresh ) {
                break;
            }
        }
        if( istart < numdof ) {
            bNullSpace = true;
        }

        Transform tmanip = _pmanip->GetTransform();
        _vBasisBiasDirections[0] = ExtractAxisFromQuat(tmanip.rot, 0);
        _vBasisBiasDirections[1] = ExtractAxisFromQuat(tmanip.rot, 1);
        _vBasisBiasDirections[2] = ExtractAxisFromQuat(tmanip.rot, 2);

        // Each of x, y, and z has 3 possible perturbations: 0, -1, 1. Excluding the all-zeros perturbation gives 26 cases.
        _vvbiasdofdirections.resize(26);
        _vvbiasnullspace.resize(26);

        for( int ix = 0; ix < 3; ++ix ) {
            for( int iy = 0; iy < 3; ++iy ) {
                for( int iz = 0; iz < 3; ++iz ) {
                    if( ix == 0 && iy == 0 && iz == 0 ) {
                        continue;
                    }
                    int idirection = (9*ix + 3*iy + iz) - 1;

                    Vector vCurDirection = _mults[ix]*_vBasisBiasDirections[0] + _mults[iy]*_vBasisBiasDirections[1] + _mults[iz]*_vBasisBiasDirections[2];
                    vCurDirection /= RaveSqrt(vCurDirection.lengthsqr3());
                    P(0) = vCurDirection.x;
                    P(1) = vCurDirection.y;
                    P(2) = vCurDirection.z;

                    // diag(S) * transpose(V) * dofvelocities = transpose(U) * P = P2
                    // transpose(V) * dofvelocities = diag(1/S) * P2 = P3
                    P2 = ublas::prod(ublas::trans(U), P);
                    // Compute P3
                    for( size_t idof = 0; idof < numdof; ++idof ) {
                        if( idof < S.size() ) {
                            if( RaveFabs(S(idof)) < zerothresh ) {
                                P3(idof) = 0;
                            }
                            else {
                                P3(idof) = P2(idof)/S(idof);
                            }
                        }
                        else {
                            P3(idof) = 0;
                        }
                    }
                    // dofvelocities = P3
                    P3 = ublas::prod(ublas::trans(V), P3);

                    std::vector<dReal>& vBiasDOFDirection = _vvbiasdofdirections[idirection];
                    vBiasDOFDirection.resize(numdof);
                    for( size_t idof = 0; idof < numdof; ++idof ) {
                        vBiasDOFDirection[idof] = P3(idof);
                    }

                    std::vector<std::vector<dReal> >& vBiasNullSpace = _vvbiasnullspace[idirection];
                    if( bNullSpace ) {
                        vBiasNullSpace.resize(numdof - istart);
                        for( size_t idim = istart; idim < numdof; ++idim ) {
                            vBiasNullSpace[idim - istart].resize(numdof);
                            for( size_t jdof = 0; jdof < numdof; ++jdof ) {
                                vBiasNullSpace[idim - istart][jdof] = V(idim, jdof);
                            }
                        }
                    }
                    else {
                        vBiasNullSpace.resize(0);
                    }
                } // end iz
            } // end iy
        } // end ix

        // update all the links (since geometry could have changed)
        _vLinkAABBs.resize(_vLinks.size());
        for(size_t i = 0; i < _vLinks.size(); ++i) {
            _vLinkAABBs[i] = _vLinks[i]->ComputeLocalAABB();
        }

        if( !!_cache ) {
            _cache->Reset(); // need this here in order to invalidate cache.
        }

        return true;
    }

    void _UpdateGrabbed()
    {
        vector<KinBodyPtr> vgrabbedbodies;
        _probot->GetGrabbed(vgrabbedbodies);
        // robot itself might have changed?
        _vLinks.resize(0);
        _vLinks.reserve(_probot->GetLinks().size());
        if( _nActiveAffineDOFs == 0 ) {
            for (size_t ilink = 0; ilink < _probot->GetLinks().size(); ++ilink) {
                if( _probot->GetLinks()[ilink]->GetGeometries().empty() ) {
                    // Links that don't have geometries (virtual links) don't matter for jittering. Their AABBs can interfere with the results.
                    continue;
                }
                for(int dofindex : _vActiveIndices) {
                    if( _probot->DoesAffect(_probot->GetJointFromDOFIndex(dofindex)->GetJointIndex(), ilink)) {
                        _vLinks.push_back(_probot->GetLinks()[ilink]);
                        break;
                    }
                }
            }
        }
        else {
            for (const KinBody::LinkPtr& plink: _probot->GetLinks()) {
                if( plink->GetGeometries().empty() ) {
                    // Links that don't have geometries (virtual links) don't matter for jittering. Their AABBs can interfere with the results.
                    continue;
                }
                _vLinks.push_back(plink);
            }
        }
        FOREACHC(itgrabbed, vgrabbedbodies) {
            FOREACHC(itlink2, (*itgrabbed)->GetLinks()) {
                _vLinks.push_back(*itlink2);
            }
        }

        // update all the grabbed links
        _vLinkAABBs.resize(_vLinks.size());
        for(size_t i = 0; i < _vLinks.size(); ++i) {
            _vLinkAABBs[i] = _vLinks[i]->ComputeLocalAABB();
        }

        //_SetCacheMaxDistance();
    }

    void _UpdateLimits()
    {
        RobotBase::RobotStateSaver robotsaver(_probot, KinBody::Save_ActiveDOF);
        _probot->SetActiveDOFs(_vActiveIndices, _nActiveAffineDOFs, _vActiveAffineAxis);
        _probot->GetActiveDOFLimits(_lower, _upper);
        _range.resize(_lower.size());
        // even though jitter is limited, if bias is enabled
        for(size_t i = 0; i < _range.size(); ++i) {
            _range[i] = _upper[i] - _lower[i];
        }

        //_SetCacheMaxDistance();
    }

    /// sets the cache's max configuration distance
    void _SetCacheMaxDistance()
    {
        dReal maxdistance=0;
        for(size_t i = 0; i < _cache->GetWeights().size(); ++i) {
            dReal f = _range[i] * _cache->GetWeights()[i];
            maxdistance += f*f;
        }
        maxdistance = RaveSqrt(maxdistance);
        if( maxdistance > _cache->GetMaxDistance()+g_fEpsilonLinear ) {
            _cache->SetMaxDistance(maxdistance);
        }
    }

    RobotBasePtr _probot;
    std::vector<dReal> _lower, _upper, _range, _rangescaled;
    std::vector<int> _vActiveIndices;
    int _nActiveAffineDOFs;
    Vector _vActiveAffineAxis;
    std::vector<KinBody::LinkPtr> _vLinks; ///< links tracking for linkdistthresh
    std::vector<AABB> _vLinkAABBs; ///< indexed according to _vLinks
    std::vector<Transform> _vOriginalTransforms, _vOriginalInvTransforms; ///< indexed according to _vLinks
    CollisionReportPtr _report;

    OpenRAVE::NeighStateFn _neighstatefn; ///< if initialized, then use this function to get nearest neighbor
    ///< Advantage of using neightstatefn is that user constraints can be met like maintaining a certain orientation of the gripper.
    int _neighstateoptions; ///< a flag to supply to _neighstatefn indicating how the neighbor should be computed.

    UserDataPtr _limitscallback, _grabbedcallback; ///< limits,grabbed change handles

    FailureCounter _counter;

    /// \return Return 0 if jitter failed and constraints are not satisfied. -1 if constraints are originally satisfied. 1 if jitter succeeded, configuration is different, and constraints are satisfied.

    uint32_t _nRandomGeneratorSeed;
    uint32_t _nNumIterations; ///< maintains the iteration count from start of SetSeed to how many iterations Sample has undergone. Used to consecutively call Sample without re-sampling the same _curdof. When > 0, then will skip some commonly tested configurations not randomized
    int _maxiterations; ///< number of different configurations to test
    dReal _maxjitter; ///< The max deviation of a dof value to jitter. value +- maxjitter
    dReal _fPerturbation; ///< Test with vPerturbations since very small changes in angles can produce collision inconsistencies
    dReal _linkdistthresh, _linkdistthresh2; ///< the maximum distance to allow a link to move. If 0, then will disable checking

    std::vector<dReal> _curdof, _newdof2, _deltadof, _deltadof2, _vonesample;
    std::vector<dReal> _fulldof; ///< full robot dof values

    CacheTreePtr _cache; ///< caches the visisted configurations
    int _cachehit;
    dReal _neighdistthresh; ///< the minimum distance that nodes can be with respect to each other for the cache

    // for biasing
    SpaceSamplerBasePtr _ssampler;
    dReal _nullsampleprob;
    dReal _nullbiassampleprob;
    dReal _deltasampleprob;
    RobotBase::ManipulatorConstPtr _pmanip;
    boost::multi_array<OpenRAVE::dReal,2> _mjacobian;
    Vector _vbiasdirection; // direction to bias in workspace. magnitude is the max bias distance
    std::vector<dReal> _vbiasdofdirection; // direction to bias in configuration space (from jacobian)
    std::vector< std::vector<dReal> > _vbiasnullspace; // configuration nullspace that does not constraint rotation. vectors are unit

    std::array<Vector, 3> _vBasisBiasDirections;
    std::vector<std::vector<dReal> > _vvbiasdofdirections;
    std::vector< std::vector<std::vector<dReal> > > _vvbiasnullspace;

    // manip constraints
    ManipDirectionThreshPtr _pConstraintToolDirection; ///< constrain direction
    ManipPositionConstraintsPtr _pConstraintToolPosition; ///< constraint position
    Transform _tLocalTool; ///< manipulator local tool pose

    //Vector vManipConstraintBoxMin, vManipConstraintBoxMax; // constraint position

    bool _bSetResultOnRobot; ///< if true, will set the final result on the robot DOF values
    bool _bResetIterationsOnSample; ///< if true, when Sample or SampleSequence is called, will reset the _nNumIterations to 0. O

    std::array<dReal, 3> _mults;
};

SpaceSamplerBasePtr CreateWorkspaceConfigurationJitterer(EnvironmentBasePtr penv, std::istream& sinput)
{
    return SpaceSamplerBasePtr(new WorkspaceConfigurationJitterer(penv, sinput));
}

} // end namespace configurationcache
