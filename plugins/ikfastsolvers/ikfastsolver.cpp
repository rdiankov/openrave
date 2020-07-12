// -*- coding: utf-8 -*-
// Copyright (C) 2006-2016 Rosen Diankov <rosen.diankov@gmail.com>
//
// This program is free software: you can redistribute it and/or modify
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
#include "plugindefs.h"

#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/lexical_cast.hpp>

#ifdef OPENRAVE_HAS_LAPACK
#include "jacobianinverse.h"
#endif

template <typename IkReal>
class IkFastSolver : public IkSolverBase
{
    class SolutionInfo
    {
public:
        SolutionInfo() : dist(1e30) {
        }
        dReal dist;
        IkReturnPtr ikreturn;
    };

public:
    IkFastSolver(EnvironmentBasePtr penv, std::istream& sinput, boost::shared_ptr<ikfast::IkFastFunctions<IkReal> > ikfunctions, const vector<dReal>& vfreeinc, dReal ikthreshold=1e-4) : IkSolverBase(penv), _ikfunctions(ikfunctions), _vFreeInc(vfreeinc), _ikthreshold(ikthreshold) {
        OPENRAVE_ASSERT_OP(ikfunctions->_GetIkRealSize(),==,sizeof(IkReal));

        _bEmptyTransform6D = false;
        std::stringstream sversion(ikfunctions->_GetIkFastVersion());
        uint32_t ikfastversion = 0;
        sversion >> std::hex >> ikfastversion;
        if( ikfastversion & 0x10000000 ) {
            _bEmptyTransform6D = true;
        }

        _fRefineWithJacobianInverseAllowedError = -1;
        _vfreeparams.resize(ikfunctions->_GetNumFreeParameters());
        _fFreeIncRevolute = PI/8; // arbitrary
        _fFreeIncPrismaticNum = 10.0; // arbitrary
        for(size_t i = 0; i < _vfreeparams.size(); ++i) {
            _vfreeparams[i] = ikfunctions->_GetFreeIndices()[i];
        }
        _nTotalDOF = ikfunctions->_GetNumJoints();
        _iktype = static_cast<IkParameterizationType>(ikfunctions->_GetIkType());
        _kinematicshash = ikfunctions->_GetKinematicsHash();
        __description = ":Interface Author: Rosen Diankov\n\nAn OpenRAVE wrapper for the ikfast generated files.\nIf 6D IK is used, will check if the end effector and other independent links are in collision before manipulator link collisions. If they are, the IK will terminate with failure immediately.\nBecause checking collisions is the slowest part of the IK, the custom filter function run before collision checking.";
        RegisterCommand("SetIkThreshold",boost::bind(&IkFastSolver<IkReal>::_SetIkThresholdCommand,this,_1,_2),
                        "sets the ik threshold for validating returned ik solutions");
        RegisterCommand("SetJacobianRefine",boost::bind(&IkFastSolver<IkReal>::_SetJacobianRefineCommand,this,_1,_2),
                        "sets the allowed workspace error, if ik solver returns above that, then use jacobian inverse to refine.");
        RegisterCommand("GetJacobianRefine",boost::bind(&IkFastSolver<IkReal>::_GetJacobianRefineCommand,this,_1,_2),
                        "returns the jaocbian refinement error threshold and max iterations.");
        RegisterCommand("SetWorkspaceDiscretizedRotationAngle",boost::bind(&IkFastSolver<IkReal>::_SetWorkspaceDiscretizedRotationAngleCommand,this,_1,_2),
                        "sets the workspace discretization value when using 6D iksolvers to solve for 5D.");
        RegisterCommand("SetDefaultIncrements",boost::bind(&IkFastSolver<IkReal>::_SetDefaultIncrementsCommand,this,_1,_2),
                        "Specify four values (2 pairs). Each pair is the free increment for revolute joint and second is the number of segment to divide free prismatic joints. The first pair is for structural free joints, the second pair is for solutions where axes align");
        RegisterCommand("GetFreeIndices",boost::bind(&IkFastSolver<IkReal>::_GetFreeIndicesCommand,this,_1,_2),
                        "returns the free increments for all the free joints.");
        RegisterCommand("SetFreeIncrements",boost::bind(&IkFastSolver<IkReal>::_SetFreeIncrementsCommand,this,_1,_2),
                        "sets the free increments for all the free joints");
        RegisterCommand("GetFreeIncrements",boost::bind(&IkFastSolver<IkReal>::_GetFreeIncrementsCommand,this,_1,_2),
                        "returns the free increments for all the free joints.");
        RegisterCommand("GetSolutionIndices",boost::bind(&IkFastSolver<IkReal>::_GetSolutionIndicesCommand,this,_1,_2),
                        "**Can only be called by a custom filter during a Solve function call.** Gets the indices of the current solution being considered. if large-range joints wrap around, (index>>16) holds the index. So (index&0xffff) is unique to robot link pose, while (index>>16) describes the repetition.");
        RegisterCommand("GetRobotLinkStateRepeatCount", boost::bind(&IkFastSolver<IkReal>::_GetRobotLinkStateRepeatCountCommand,this,_1,_2),
                        "**Can only be called by a custom filter during a Solve function call.**. Returns 1 if the filter was called already with the same robot link positions, 0 otherwise. This is useful in saving computation. ");
        RegisterCommand("SetBackTraceSelfCollisionLinks",boost::bind(&IkFastSolver<IkReal>::_SetBackTraceSelfCollisionLinksCommand,this,_1,_2),
                        "format: int int\n\n\
for numBacktraceLinksForSelfCollisionWithNonMoving numBacktraceLinksForSelfCollisionWithFree, when pruning self collisions, the number of links to look at. If the tip of the manip self collides with the base, then can safely quit the IK.");
        _numBacktraceLinksForSelfCollisionWithNonMoving = 2;
        _numBacktraceLinksForSelfCollisionWithFree = 0;
    }
    virtual ~IkFastSolver() {
    }

    inline boost::shared_ptr<IkFastSolver<IkReal> > shared_solver() {
        return boost::static_pointer_cast<IkFastSolver<IkReal> >(shared_from_this());
    }
    inline boost::shared_ptr<IkFastSolver<IkReal> const> shared_solver_const() const {
        return boost::static_pointer_cast<IkFastSolver<IkReal> const>(shared_from_this());
    }
    inline boost::weak_ptr<IkFastSolver<IkReal> > weak_solver() {
        return shared_solver();
    }

    bool _SetIkThresholdCommand(ostream& sout, istream& sinput)
    {
        sinput >> _ikthreshold;
        return !!sinput;
    }

    bool _SetJacobianRefineCommand(ostream& sout, istream& sinput)
    {
#ifdef OPENRAVE_HAS_LAPACK
        dReal f = 0;
        int nMaxIterations = -1;
        sinput >> f >> nMaxIterations;
        _SetJacobianRefine(f, nMaxIterations);
        return true;
#else
        return false;
#endif
    }

    void _SetJacobianRefine(dReal f, int nMaxIterations)
    {
#ifdef OPENRAVE_HAS_LAPACK
        _fRefineWithJacobianInverseAllowedError = f;
        _jacobinvsolver.SetErrorThresh(_fRefineWithJacobianInverseAllowedError);
        if( nMaxIterations >= 0 ) {
            _jacobinvsolver.SetMaxIterations(nMaxIterations);
        }
#endif
    }

    bool _SetWorkspaceDiscretizedRotationAngleCommand(ostream& sout, istream& sinput)
    {
        dReal workspaceDiscretizedRotationAngle = 0;
        sinput >> workspaceDiscretizedRotationAngle;
        return !!sinput;
    }

    bool _SetDefaultIncrementsCommand(ostream& sout, istream& sinput)
    {
        dReal fFreeIncRevolute=0.1, fFreeIncPrismaticNum=100;
        sinput >> fFreeIncRevolute >> fFreeIncPrismaticNum >> _fFreeIncRevolute >> _fFreeIncPrismaticNum;
        _vFreeInc.resize(_vfreeparams.size());
        for(size_t i = 0; i < _vFreeInc.size(); ++i) {
            if( _vfreerevolute.at(i) ) {
                _vFreeInc[i] = fFreeIncRevolute;
            }
            else {
                _vFreeInc[i] = (_qupper.at(_vfreeparams[i])-_qlower.at(_vfreeparams[i]))/fFreeIncPrismaticNum;
            }
        }
        //RAVELOG_VERBOSE(str(boost::format("SetFreeIncrements: %f %f %f %f")%fFreeIncRevolute%fFreeIncPrismaticNum%_fFreeIncRevolute%_fFreeIncPrismaticNum));
        return !!sinput;
    }

    bool _SetFreeIncrementsCommand(ostream& sout, istream& sinput)
    {
        if( _vFreeInc.size() == 0 ) {
            return true;
        }
        FOREACHC(it, _vFreeInc) {
            sinput >> *it;
        }
        return !!sinput;
    }

    bool _GetFreeIndicesCommand(ostream& sout, istream& sinput)
    {
        FOREACHC(it, _vfreeparams) {
            sout << *it << " ";
        }
        return true;
    }

    bool _GetFreeIncrementsCommand(ostream& sout, istream& sinput)
    {
        FOREACHC(it, _vFreeInc) {
            sout << *it << " ";
        }
        return true;
    }

    bool _GetJacobianRefineCommand(ostream& sout, istream& sinput)
    {
#ifdef OPENRAVE_HAS_LAPACK
        sout << _jacobinvsolver.GetErrorThresh() << " " << _jacobinvsolver.GetMaxIterations();
        return true;
#else
        return false;
#endif
    }

    bool _GetSolutionIndicesCommand(ostream& sout, istream& sinput)
    {
        sout << _vsolutionindices.size() << " ";
        FOREACHC(it,_vsolutionindices) {
            sout << *it << " ";
        }
        return true;
    }

    bool _GetRobotLinkStateRepeatCountCommand(ostream& sout, istream& sinput)
    {
        sout << _nSameStateRepeatCount;
        return true;
    }

    bool _SetBackTraceSelfCollisionLinksCommand(ostream& sout, istream& sinput)
    {
        sinput >> _numBacktraceLinksForSelfCollisionWithNonMoving >> _numBacktraceLinksForSelfCollisionWithFree;
        return true;
    }

    virtual IkReturnAction CallFilters(const IkParameterization& param, IkReturnPtr ikreturn, int minpriority, int maxpriority) {
        // have to convert to the manipulator's base coordinate system
        RobotBase::ManipulatorPtr pmanip = _pmanip.lock();
        if( !pmanip ) {
            RAVELOG_WARN_FORMAT("env=%d iksolver points to removed manip '%s', passing through", GetEnv()->GetId()%_manipname);
            return IKRA_Success; // pass through
        }
        std::vector<dReal> vsolution;
        pmanip->GetRobot()->GetDOFValues(vsolution, pmanip->GetArmIndices());
        // do sanity check to make sure that current robot manip is consistent with param
        dReal ikworkspacedist = pmanip->GetIkParameterization(param.GetType(), false).ComputeDistanceSqr(param);
        if( ikworkspacedist > _ikthreshold ) {
            stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
            ss << "ignoring bad ik for " << pmanip->GetRobot()->GetName() << ":" << pmanip->GetName() << " dist=" << RaveSqrt(ikworkspacedist) << ", param=[" << param << "], sol=[";
            FOREACHC(itvalue,vsolution) {
                ss << *itvalue << ", ";
            }
            ss << "]" << endl;
            throw OPENRAVE_EXCEPTION_FORMAT0(ss.str(), ORE_InvalidArguments);
        }

        return _CallFilters(vsolution, pmanip, param, ikreturn, minpriority, maxpriority);
    }

    virtual void SetJointLimits()
    {
        RobotBase::ManipulatorPtr pmanip = _pmanip.lock();
        if( !pmanip ) {
            RAVELOG_WARN_FORMAT("env=%d iksolver points to removed manip '%s'", GetEnv()->GetId()%_manipname);
            return;
        }

        RobotBasePtr probot = pmanip->GetRobot();
        probot->GetDOFLimits(_qlower,_qupper,pmanip->GetArmIndices());
        _qmid.resize(_qlower.size());
        _qbigrangeindices.resize(0);
        _qbigrangemaxsols.resize(0);
        _qbigrangemaxcumprod.resize(0); _qbigrangemaxcumprod.push_back(1);
        for(size_t i = 0; i < _qmid.size(); ++i) {
            _qmid[i] = 0.5*(_qlower[i]*_qupper[i]);
            if( _qupper[i]-_qlower[i] > 2*PI ) {
                int dofindex = pmanip->GetArmIndices().at(i);
                KinBody::JointPtr pjoint = probot->GetJointFromDOFIndex(dofindex);
                int iaxis = dofindex-pjoint->GetDOFIndex();
                if( pjoint->IsRevolute(iaxis) && !pjoint->IsCircular(iaxis) ) {
                    _qbigrangeindices.push_back(i);
                    _qbigrangemaxsols.push_back( 1+int((_qupper[i]-_qlower[i])/(2*PI))); // max redundant solutions
                    _qbigrangemaxcumprod.push_back(_qbigrangemaxcumprod.back() * _qbigrangemaxsols.back());
                }
            }
        }
        _vfreeparamscales.resize(0);
        FOREACH(itfree, _vfreeparams) {
            if( *itfree < 0 || *itfree >= (int)_qlower.size() ) {
                throw openrave_exception(str(boost::format(_("free parameter idx %d out of bounds\n"))%*itfree));
            }
            if( _qupper[*itfree] > _qlower[*itfree] ) {
                _vfreeparamscales.push_back(1.0f/(_qupper[*itfree]-_qlower[*itfree]));
            }
            else {
                _vfreeparamscales.push_back(0.0f);
            }
        }
    }

    const std::string& GetKinematicsStructureHash() const {
        return _kinematicshash;
    }

    virtual bool Init(RobotBase::ManipulatorConstPtr pmanip)
    {
        if( _kinematicshash.size() > 0 && pmanip->GetInverseKinematicsStructureHash(_iktype) != _kinematicshash ) {
            RAVELOG_ERROR_FORMAT("env=%d, inverse kinematics hashes do not match for manip %s:%s. IK will not work!  manip (%s) != loaded (%s)", pmanip->GetRobot()->GetEnv()->GetId()%pmanip->GetRobot()->GetName()%pmanip->GetName()%pmanip->GetInverseKinematicsStructureHash(_iktype)%_kinematicshash);
            return false;
        }
        RobotBasePtr probot = pmanip->GetRobot();
        bool bfound = false;
        _pmanip.reset();
        _manipname.clear();
        FOREACHC(itmanip,probot->GetManipulators()) {
            if( *itmanip == pmanip ) {
                _pmanip = *itmanip;
                _manipname = (*itmanip)->GetName();
                bfound = true;
            }
        }
        if( !bfound ) {
            std::stringstream smanipnames;
            FOREACH(itmanip, probot->GetManipulators()) {
                smanipnames << (*itmanip)->GetName() << ", ";
            }
            throw OPENRAVE_EXCEPTION_FORMAT(_("manipulator '%s' not found in robot '%s' (%d) with manips [%s]"), pmanip->GetName()%probot->GetName()%probot->GetEnvironmentId()%smanipnames.str(), ORE_InvalidArguments);
        }

        _cblimits = probot->RegisterChangeCallback(KinBody::Prop_JointLimits,boost::bind(&IkFastSolver<IkReal>::SetJointLimits,boost::bind(&utils::sptr_from<IkFastSolver<IkReal> >, weak_solver())));

        if( _nTotalDOF != (int)pmanip->GetArmIndices().size() ) {
            RAVELOG_ERROR(str(boost::format("ik %s configured with different number of joints than robot manipulator (%d!=%d)\n")%GetXMLId()%pmanip->GetArmIndices().size()%_nTotalDOF));
            return false;
        }

        _vfreerevolute.resize(0);
        FOREACH(itfree, _vfreeparams) {
            int index = pmanip->GetArmIndices().at(*itfree);
            KinBody::JointPtr pjoint = probot->GetJointFromDOFIndex(index);
            if( pjoint->IsRevolute(index-pjoint->GetDOFIndex()) ) {
                _vfreerevolute.push_back(pjoint->IsCircular(index-pjoint->GetDOFIndex()) ? 2 : 1);
            }
            else {
                _vfreerevolute.push_back(0);
            }
        }

        _vjointrevolute.resize(0);
        FOREACHC(it,pmanip->GetArmIndices()) {
            KinBody::JointPtr pjoint = probot->GetJointFromDOFIndex(*it);
            if( pjoint->IsRevolute(*it-pjoint->GetDOFIndex()) ) {
                _vjointrevolute.push_back(pjoint->IsCircular(*it-pjoint->GetDOFIndex()) ? 2 : 1);
            }
            else {
                _vjointrevolute.push_back(0);
            }
        }

        if( _vFreeInc.size() != _vfreeparams.size() ) {
            if( _vFreeInc.size() != 0 ) {
                RAVELOG_WARN(str(boost::format("_vFreeInc not correct size: %d != %d\n")%_vFreeInc.size()%_vfreeparams.size()));
            }
            _vFreeInc.resize(_vfreeparams.size());
            //stringstream ss;
            //ss << "robot " << probot->GetName() << ":" << pmanip->GetName() << " setting free increment to: ";
            for(size_t i = 0; i < _vFreeInc.size(); ++i) {
                if( _vfreerevolute.at(i) ) {
                    _vFreeInc[i] = 0.1;
                }
                else {
                    _vFreeInc[i] = 0.01;
                }
                //ss << _vFreeInc[i] << " ";
            }
            //RAVELOG_DEBUG(ss.str());
        }

        pmanip->GetChildLinks(_vchildlinks);
        _vchildlinkindices.resize(_vchildlinks.size());
        for(size_t i = 0; i < _vchildlinks.size(); ++i) {
            _vchildlinkindices[i] = _vchildlinks[i]->GetIndex();
        }
        pmanip->GetIndependentLinks(_vindependentlinks);

        if( _vfreeparams.size() == 0 ) {
            _vIndependentLinksIncludingFreeJoints = _vindependentlinks;
        }
        else {
            std::vector<dReal> vactiveindices; vactiveindices.reserve(pmanip->GetArmDOF() - _vfreeparams.size());
            for(size_t i = 0; i < pmanip->GetArmIndices().size(); ++i) {
                if( find(_vfreeparams.begin(), _vfreeparams.end(), i) == _vfreeparams.end() ) {
                    vactiveindices.push_back(pmanip->GetArmIndices()[i]);
                }
            }

            _vIndependentLinksIncludingFreeJoints.resize(0);
            FOREACHC(itlink, probot->GetLinks()) {
                bool bAffected = false;
                FOREACHC(itindex,vactiveindices) {
                    if( probot->DoesAffect(probot->GetJointFromDOFIndex(*itindex)->GetJointIndex(),(*itlink)->GetIndex()) ) {
                        bAffected = true;
                        break;
                    }
                }
                FOREACHC(itindex,pmanip->GetGripperIndices()) {
                    if( probot->DoesAffect(probot->GetJointFromDOFIndex(*itindex)->GetJointIndex(),(*itlink)->GetIndex()) ) {
                        bAffected = true;
                        break;
                    }
                }

                if( !bAffected ) {
                    _vIndependentLinksIncludingFreeJoints.push_back(*itlink);
                }
            }
        }

#ifdef OPENRAVE_HAS_LAPACK
        if( !!pmanip ) {
            _jacobinvsolver.Init(*pmanip);
        }
#endif

        SetJointLimits();
        return true;
    }

    virtual bool Supports(IkParameterizationType iktype) const
    {
        if( iktype == _iktype ) {
            return true;
        }
        // auto-conversion
        if( _nTotalDOF-GetNumFreeParameters() == 4 ) {
            if( _iktype == IKP_Transform6D ) {
                // not always true! sometimes 4D robots can only support Transform6D (fanuc 4 axis depalletizing)
                //return iktype == IKP_TranslationXAxisAngleZNorm4D || iktype == IKP_TranslationYAxisAngleXNorm4D || iktype == IKP_TranslationZAxisAngleYNorm4D || iktype == IKP_TranslationZAxisAngle4D;
            }
        }
        else if( _nTotalDOF-GetNumFreeParameters() == 5 ) {
            // not always true! sometimes 5D robots can only support Transform6D
//            if( _iktype == IKP_Transform6D ) {
//                return iktype == IKP_TranslationDirection5D;
//            }
        }
        return false;
    }

    /// \brief manages the enabling and disabling of the end effector links depending on the filter options
    class StateCheckEndEffector
    {
public:
        StateCheckEndEffector(RobotBasePtr probot, const std::vector<KinBody::LinkPtr>& vchildlinks, const std::vector<KinBody::LinkPtr>& vindependentlinks, int filteroptions) : _vchildlinks(vchildlinks), _vindependentlinks(vindependentlinks) {
            _probot = probot;
            _bCheckEndEffectorEnvCollision = !(filteroptions & IKFO_IgnoreEndEffectorEnvCollisions);
            _bCheckEndEffectorSelfCollision = !(filteroptions & (IKFO_IgnoreEndEffectorSelfCollisions|IKFO_IgnoreSelfCollisions));
            _bCheckSelfCollision = !(filteroptions & IKFO_IgnoreSelfCollisions);
            _bDisabled = false;
            numImpossibleSelfCollisions = 0;
        }
        virtual ~StateCheckEndEffector() {
            // restore the link states
            if( _vlinkenabled.size() == _vchildlinks.size() ) {
                for(size_t i = 0; i < _vchildlinks.size(); ++i) {
                    _vchildlinks[i]->Enable(!!_vlinkenabled[i]);
                }
            }
        }

        void SetEnvironmentCollisionState()
        {
            if( !_bDisabled && (!_bCheckEndEffectorEnvCollision || !_bCheckEndEffectorSelfCollision) ) {
                _InitSavers();
                for(size_t i = 0; i < _vchildlinks.size(); ++i) {
                    _vchildlinks[i]->Enable(false);
                }
                FOREACH(it, _listGrabbedSavedStates) {
                    it->GetBody()->Enable(false);
                }
                _bDisabled = true;
            }
        }
        void SetSelfCollisionState()
        {
            if( _bDisabled ) {
                _InitSavers();
                for(size_t i = 0; i < _vchildlinks.size(); ++i) {
                    _vchildlinks[i]->Enable(!!_vlinkenabled[i]);
                }
                FOREACH(it, _listGrabbedSavedStates) {
                    it->Restore();
                }
                _bDisabled = false;
            }
            if( (!_bCheckEndEffectorEnvCollision || !_bCheckEndEffectorSelfCollision) && !_callbackhandle ) {
                _InitSavers();
                // have to register a handle if we're ignoring end effector collisions
                _callbackhandle = _probot->GetEnv()->RegisterCollisionCallback(boost::bind(&StateCheckEndEffector::_CollisionCallback,this,_1,_2));
            }
        }

        bool NeedCheckEndEffectorEnvCollision() {
            return _bCheckEndEffectorEnvCollision;
        }

        // set collision state to not check for end effector collisions
        void ResetCheckEndEffectorEnvCollision() {
            _bCheckEndEffectorEnvCollision = false;
            SetEnvironmentCollisionState();
        }

        void RestoreCheckEndEffectorEnvCollision() {
            _bCheckEndEffectorEnvCollision = true;
            if( _bDisabled ) {
                _InitSavers();
                for(size_t i = 0; i < _vchildlinks.size(); ++i) {
                    _vchildlinks[i]->Enable(!!_vlinkenabled[i]);
                }
                FOREACH(it, _listGrabbedSavedStates) {
                    it->Restore();
                }
                _bDisabled = false;
            }
        }

        /// \brief check if end effector is colliding if there's a colliding transform within some angle (this is used for TranslationDirection5D IK where end effector differs by a rotation around an axis)
        ///
        /// \param coshalfdeltaangle cos(0.5*deltaangle) where deltaangle is the angle between the two poses
        /// \return 1 if in collision, 0 if not, -1 if unknown (there's no enough data)
        int IsCollidingEndEffector(const Transform& tendeffector, dReal coshalfdeltaangle=0.99968751627570263) {
            //Transform tendeffectorinv = tendeffectorinv.inverse();
            FOREACHC(it, _listCollidingTransforms) {
                //Transform t = tendeffectorinv*it->first;
                dReal quatcosangle = tendeffector.rot.dot(it->first.rot);
                if( quatcosangle >= coshalfdeltaangle ) {
                    return (int)it->second;
                }
            }
            return -1; // don't know
        }

        void RegisterCollidingEndEffector(const Transform& t, bool bcolliding)
        {
            _listCollidingTransforms.emplace_back(t,  bcolliding);
        }

        int numImpossibleSelfCollisions; ///< a count of the number of self-collisions that most likely mean that the IK itself will fail.
protected:
        void _InitSavers()
        {
            if( _vlinkenabled.size() > 0 ) {
                return; // already initialized
            }
            _vlinkenabled.resize(_vchildlinks.size());
            for(size_t i = 0; i < _vchildlinks.size(); ++i) {
                _vlinkenabled[i] = _vchildlinks[i]->IsEnabled();
            }
            _listGrabbedSavedStates.clear();
            std::vector<KinBodyPtr> vgrabbedbodies;
            _probot->GetGrabbed(vgrabbedbodies);
            FOREACH(itbody,vgrabbedbodies) {
                if( find(_vchildlinks.begin(),_vchildlinks.end(),_probot->IsGrabbing(**itbody)) != _vchildlinks.end() ) {
                    _listGrabbedSavedStates.push_back(KinBody::KinBodyStateSaver(*itbody, KinBody::Save_LinkEnable));
                }
            }
        }

        CollisionAction _CollisionCallback(CollisionReportPtr report, bool IsCalledFromPhysicsEngine)
        {
            if( !_bCheckEndEffectorEnvCollision || !_bCheckEndEffectorSelfCollision ) {
                bool bChildLink1 = find(_vchildlinks.begin(),_vchildlinks.end(),report->plink1) != _vchildlinks.end();
                bool bChildLink2 = find(_vchildlinks.begin(),_vchildlinks.end(),report->plink2) != _vchildlinks.end();
                bool bIndependentLink1 = find(_vindependentlinks.begin(),_vindependentlinks.end(),report->plink1) != _vindependentlinks.end();
                bool bIndependentLink2 = find(_vindependentlinks.begin(),_vindependentlinks.end(),report->plink2) != _vindependentlinks.end();
                if( !_bCheckEndEffectorEnvCollision ) {
                    if( (bChildLink1 && !bIndependentLink2) || (bChildLink2 && bIndependentLink1) ) {
                        // end effector colliding with environment
                        return CA_Ignore;
                    }
                }
                if( !_bCheckEndEffectorSelfCollision ) {
                    if( (bChildLink1&&bIndependentLink2) || (bChildLink2&&bIndependentLink1) ) {
                        // child+independent link when should be ignore end-effector collisions
                        return CA_Ignore;
                    }
                }

                // check for attached bodies of the child links
                if( !bIndependentLink2 && !bChildLink2 && !!report->plink2 ) {
                    KinBodyPtr pcolliding = report->plink2->GetParent();
                    FOREACH(it,_listGrabbedSavedStates) {
                        if( it->GetBody() == pcolliding ) {
                            if( !_bCheckEndEffectorEnvCollision ) {
                                // if plink1 is not part of the robot, then ignore.
                                if( !report->plink1 || report->plink1->GetParent() != _probot ) {
                                    return CA_Ignore;
                                }
                            }
                            // otherwise counted as self-collision since a part of this end effector
                            if( !_bCheckEndEffectorSelfCollision ) {
                                return CA_Ignore;
                            }
                        }
                    }
                }
                if( !bIndependentLink1 && !bChildLink1 && !!report->plink1 ) {
                    KinBodyPtr pcolliding = report->plink1->GetParent();
                    FOREACH(it,_listGrabbedSavedStates) {
                        if( it->GetBody() == pcolliding ) {
                            if( !_bCheckEndEffectorEnvCollision ) {
                                // if plink2 is not part of the robot, then ignore. otherwise it needs to be counted as self-collision
                                if( !report->plink2 || report->plink2->GetParent() != _probot ) {
                                    return CA_Ignore;
                                }
                            }
                            // otherwise counted as self-collision since a part of this end effector
                            if( !_bCheckEndEffectorSelfCollision ) {
                                return CA_Ignore;
                            }
                        }
                    }
                }
            }
            return CA_DefaultAction;
        }

        RobotBasePtr _probot;
        std::list<KinBody::KinBodyStateSaver> _listGrabbedSavedStates;
        vector<uint8_t> _vlinkenabled;
        UserDataPtr _callbackhandle;
        const std::vector<KinBody::LinkPtr>& _vchildlinks, &_vindependentlinks;
        std::list<std::pair<Transform, bool> > _listCollidingTransforms;
        bool _bCheckEndEffectorEnvCollision, _bCheckEndEffectorSelfCollision, _bCheckSelfCollision, _bDisabled;
    };

    virtual bool Solve(const IkParameterization& rawparam, const std::vector<dReal>& q0, int filteroptions, boost::shared_ptr< std::vector<dReal> > result)
    {
        std::vector<dReal> q0local = q0; // copy in case result points to q0
        if( !!result ) {
            result->resize(0);
        }
        IkReturn ikreturn(IKRA_Success);
        IkReturnPtr pikreturn(&ikreturn,utils::null_deleter());
        if( !Solve(rawparam,q0local,filteroptions,pikreturn) ) {
            return false;
        }
        if( !!result ) {
            *result = ikreturn._vsolution;
        }
        return true;
    }

    virtual bool SolveAll(const IkParameterization& rawparam, int filteroptions, std::vector< std::vector<dReal> >& qSolutions)
    {
        std::vector<IkReturnPtr> vikreturns;
        qSolutions.resize(0);
        if( !SolveAll(rawparam,filteroptions,vikreturns) ) {
            return false;
        }
        qSolutions.resize(vikreturns.size());
        for(size_t i = 0; i < vikreturns.size(); ++i) {
            qSolutions[i] = vikreturns[i]->_vsolution;
        }
        return qSolutions.size()>0;
    }

    virtual bool Solve(const IkParameterization& param, const std::vector<dReal>& q0, const std::vector<dReal>& vFreeParameters, int filteroptions, boost::shared_ptr< std::vector<dReal> > result)
    {
        std::vector<dReal> q0local = q0; // copy in case result points to q0
        if( !!result ) {
            result->resize(0);
        }
        IkReturn ikreturn(IKRA_Success);
        IkReturnPtr pikreturn(&ikreturn,utils::null_deleter());
        if( !Solve(param,q0local,vFreeParameters,filteroptions,pikreturn) ) {
            return false;
        }
        if( !!result ) {
            *result = ikreturn._vsolution;
        }
        return true;
    }

    virtual bool SolveAll(const IkParameterization& param, const std::vector<dReal>& vFreeParameters, int filteroptions, std::vector< std::vector<dReal> >& qSolutions)
    {
        std::vector<IkReturnPtr> vikreturns;
        qSolutions.resize(0);
        if( !SolveAll(param,vFreeParameters,filteroptions,vikreturns) ) {
            return false;
        }
        qSolutions.resize(vikreturns.size());
        for(size_t i = 0; i < vikreturns.size(); ++i) {
            qSolutions[i] = vikreturns[i]->_vsolution;
        }
        return qSolutions.size()>0;
    }

    virtual bool Solve(const IkParameterization& rawparam, const std::vector<dReal>& q0, int filteroptions, IkReturnPtr ikreturn)
    {
        IkParameterization ikparamdummy;
        const IkParameterization& param = _ConvertIkParameterization(rawparam, ikparamdummy);
        if( !!ikreturn ) {
            ikreturn->Clear();
        }
        RobotBase::ManipulatorPtr pmanip(_pmanip);
        RobotBasePtr probot = pmanip->GetRobot();
        RobotBase::RobotStateSaver saver(probot);
        probot->SetActiveDOFs(pmanip->GetArmIndices());
        std::vector<IkReal> vfree(_vfreeparams.size());
        StateCheckEndEffector stateCheck(probot,_vchildlinks,_vindependentlinks,filteroptions);
        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);
        IkReturnAction retaction = ComposeSolution(_vfreeparams, vfree, 0, q0, boost::bind(&IkFastSolver::_SolveSingle,shared_solver(), boost::ref(param),boost::ref(vfree),boost::ref(q0),filteroptions,ikreturn,boost::ref(stateCheck)), _vFreeInc);
        if( !!ikreturn ) {
            ikreturn->_action = retaction;
        }
        return retaction == IKRA_Success;
    }

    virtual bool SolveAll(const IkParameterization& rawparam, int filteroptions, std::vector<IkReturnPtr>& vikreturns)
    {
        vikreturns.resize(0);
        IkParameterization ikparamdummy;
        const IkParameterization& param = _ConvertIkParameterization(rawparam, ikparamdummy);
        RobotBase::ManipulatorPtr pmanip(_pmanip);
        RobotBasePtr probot = pmanip->GetRobot();
        RobotBase::RobotStateSaver saver(probot);
        probot->SetActiveDOFs(pmanip->GetArmIndices());
        std::vector<IkReal> vfree(_vfreeparams.size());
        StateCheckEndEffector stateCheck(probot,_vchildlinks,_vindependentlinks,filteroptions);
        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);
        IkReturnAction retaction = ComposeSolution(_vfreeparams, vfree, 0, vector<dReal>(), boost::bind(&IkFastSolver::_SolveAll,shared_solver(), param,boost::ref(vfree),filteroptions,boost::ref(vikreturns), boost::ref(stateCheck)), _vFreeInc);
        if( retaction & IKRA_Quit ) {
            return false;
        }
        _SortSolutions(probot, vikreturns);
        return vikreturns.size()>0;
    }

    virtual bool Solve(const IkParameterization& rawparam, const std::vector<dReal>& q0, const std::vector<dReal>& vFreeParameters, int filteroptions, IkReturnPtr ikreturn)
    {
        IkParameterization ikparamdummy;
        const IkParameterization& param = _ConvertIkParameterization(rawparam, ikparamdummy);
        if( vFreeParameters.size() != _vfreeparams.size() ) {
            throw openrave_exception(_("free parameters not equal"),ORE_InvalidArguments);
        }
        if( !!ikreturn ) {
            ikreturn->Clear();
        }
        RobotBase::ManipulatorPtr pmanip(_pmanip);
        RobotBasePtr probot = pmanip->GetRobot();
        RobotBase::RobotStateSaver saver(probot);
        probot->SetActiveDOFs(pmanip->GetArmIndices());
        std::vector<IkReal> vfree(_vfreeparams.size());
        for(size_t i = 0; i < _vfreeparams.size(); ++i) {
            vfree[i] = vFreeParameters[i]*(_qupper[_vfreeparams[i]]-_qlower[_vfreeparams[i]]) + _qlower[_vfreeparams[i]];
        }
        StateCheckEndEffector stateCheck(probot,_vchildlinks,_vindependentlinks,filteroptions);
        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);
        IkReturnAction retaction = _SolveSingle(param,vfree,q0,filteroptions,ikreturn,stateCheck);
        if( !!ikreturn ) {
            ikreturn->_action = retaction;
        }
        return retaction==IKRA_Success;
    }

    virtual bool SolveAll(const IkParameterization& rawparam, const std::vector<dReal>& vFreeParameters, int filteroptions, std::vector<IkReturnPtr>& vikreturns)
    {
        vikreturns.resize(0);
        IkParameterization ikparamdummy;
        const IkParameterization& param = _ConvertIkParameterization(rawparam, ikparamdummy);
        if( vFreeParameters.size() != _vfreeparams.size() ) {
            throw openrave_exception(_("free parameters not equal"),ORE_InvalidArguments);
        }
        RobotBase::ManipulatorPtr pmanip(_pmanip);
        RobotBasePtr probot = pmanip->GetRobot();
        RobotBase::RobotStateSaver saver(probot);
        probot->SetActiveDOFs(pmanip->GetArmIndices());
        std::vector<IkReal> vfree(_vfreeparams.size());
        for(size_t i = 0; i < _vfreeparams.size(); ++i) {
            vfree[i] = vFreeParameters[i]*(_qupper[_vfreeparams[i]]-_qlower[_vfreeparams[i]]) + _qlower[_vfreeparams[i]];
        }
        StateCheckEndEffector stateCheck(probot,_vchildlinks,_vindependentlinks,filteroptions);
        CollisionOptionsStateSaver optionstate(GetEnv()->GetCollisionChecker(),GetEnv()->GetCollisionChecker()->GetCollisionOptions()|CO_ActiveDOFs,false);
        IkReturnAction retaction = _SolveAll(param,vfree,filteroptions,vikreturns, stateCheck);
        if( retaction & IKRA_Quit ) {
            return false;
        }
        _SortSolutions(probot, vikreturns);
        return vikreturns.size()>0;
    }

    virtual int GetNumFreeParameters() const
    {
        return (int)_vfreeparams.size();
    }

    virtual bool GetFreeParameters(std::vector<dReal>& pFreeParameters) const
    {
        RobotBase::ManipulatorPtr pmanip(_pmanip);
        RobotBasePtr probot = pmanip->GetRobot();
        std::vector<dReal> values;
        std::vector<dReal>::const_iterator itscale = _vfreeparamscales.begin();
        probot->GetDOFValues(values);
        pFreeParameters.resize(_vfreeparams.size());
        for(size_t i = 0; i < _vfreeparams.size(); ++i) {
            pFreeParameters[i] = (values.at(pmanip->GetArmIndices().at(_vfreeparams[i]))-_qlower.at(_vfreeparams[i])) * *itscale++;
        }
        return true;
    }

    virtual bool GetFreeIndices(std::vector<int>& vFreeIndices) const
    {
        vFreeIndices = _vfreeparams;
        return true;
    }

    virtual RobotBase::ManipulatorPtr GetManipulator() const {
        return _pmanip.lock();
    }

    virtual void Clone(InterfaceBaseConstPtr preference, int cloningoptions)
    {
        IkSolverBase::Clone(preference, cloningoptions);
        boost::shared_ptr< IkFastSolver<IkReal> const > r = boost::dynamic_pointer_cast<IkFastSolver<IkReal> const>(preference);

        _pmanip.reset();
        _manipname.clear();
        _cblimits.reset();
        _vchildlinks.resize(0);
        _vchildlinkindices.resize(0);
        _vindependentlinks.resize(0);
        RobotBase::ManipulatorPtr rmanip = r->_pmanip.lock();
        if( !!rmanip ) {
            RobotBasePtr probot = GetEnv()->GetRobot(rmanip->GetRobot()->GetName());
            if( !!probot ) {
                RobotBase::ManipulatorPtr pmanip = probot->GetManipulator(rmanip->GetName());
                _pmanip = pmanip;
                if( !!pmanip ) {
                    _manipname = pmanip->GetName();
                }
                _cblimits = probot->RegisterChangeCallback(KinBody::Prop_JointLimits,boost::bind(&IkFastSolver<IkReal>::SetJointLimits,boost::bind(&utils::sptr_from<IkFastSolver<IkReal> >, weak_solver())));

                if( !!pmanip ) {
                    pmanip->GetChildLinks(_vchildlinks);
                    _vchildlinkindices.resize(_vchildlinks.size());
                    for(size_t i = 0; i < _vchildlinks.size(); ++i) {
                        _vchildlinkindices[i] = _vchildlinks[i]->GetIndex();
                    }
                    pmanip->GetIndependentLinks(_vindependentlinks);
                }
            }
        }
        _vfreeparams = r->_vfreeparams;
        _vfreerevolute = r->_vfreerevolute;
        _vjointrevolute = r->_vjointrevolute;
        _vfreeparamscales = r->_vfreeparamscales;
        _ikfunctions = r->_ikfunctions; // probably not necessary, but not setting it here could create inconsistency problems later on
        _vFreeInc = r->_vFreeInc;
        _fFreeIncRevolute = r->_fFreeIncRevolute;
        _fFreeIncPrismaticNum = r->_fFreeIncPrismaticNum;
        _nTotalDOF = r->_nTotalDOF;
        _qlower = r->_qlower;
        _qupper = r->_qupper;
        _qmid = r->_qmid;
        _qbigrangeindices = r->_qbigrangeindices;
        _qbigrangemaxsols = r->_qbigrangemaxsols;
        _qbigrangemaxcumprod = r->_qbigrangemaxcumprod;
        _iktype = r->_iktype;

        _kinematicshash = r->_kinematicshash;
        _numBacktraceLinksForSelfCollisionWithNonMoving = r->_numBacktraceLinksForSelfCollisionWithNonMoving;
        _numBacktraceLinksForSelfCollisionWithFree = r->_numBacktraceLinksForSelfCollisionWithFree;
        _ikthreshold = r->_ikthreshold;
#ifdef OPENRAVE_HAS_LAPACK
        _SetJacobianRefine(r->_fRefineWithJacobianInverseAllowedError, r->_jacobinvsolver._nMaxIterations);
#endif

        _bEmptyTransform6D = r->_bEmptyTransform6D;
    }

protected:
    IkReturnAction ComposeSolution(const std::vector<int>& vfreeparams, vector<IkReal>& vfree, int freeindex, const vector<dReal>& q0, const boost::function<IkReturnAction()>& fn, const std::vector<dReal>& vFreeInc)
    {
        if( freeindex >= (int)vfreeparams.size()) {
            return fn();
        }

        // start searching for phi close to q0, as soon as a solution is found for the curphi, return it
        dReal startphi = q0.size() == _qlower.size() ? q0.at(vfreeparams.at(freeindex)) : 0;
        dReal upperphi = _qupper.at(vfreeparams.at(freeindex)), lowerphi = _qlower.at(vfreeparams.at(freeindex)), deltaphi = 0;
        dReal lowerChecked = startphi; // lowest value checked
        dReal upperChecked = startphi; // uppermost value checked
        int isJointRevolute = _vjointrevolute.at(vfreeparams.at(freeindex));
        if( isJointRevolute == 2 ) {
            startphi = utils::NormalizeCircularAngle(startphi, -PI, PI);
            lowerphi = startphi-PI;
            upperphi = startphi+PI;
        }

        bool bIsZeroTested = false;
        int iter = 0;
        dReal fFreeInc = vFreeInc.at(freeindex);
        int allres = IKRA_Reject;
        while(1) {
            dReal curphi = startphi;
            if( iter & 1 ) { // increment
                curphi += deltaphi;
                if( curphi > upperphi ) {
                    if( startphi-deltaphi < lowerphi) {
                        break; // reached limit
                    }
                    ++iter;
                    continue;
                }
                upperChecked = curphi;
                if( isJointRevolute != 0 && upperChecked - lowerChecked >= 2*PI-g_fEpsilonJointLimit ) {
                    // reached full circle
                    break;
                }
            }
            else { // decrement
                curphi -= deltaphi;
                if( curphi < lowerphi ) {
                    if( startphi+deltaphi > upperphi ) {
                        break; // reached limit
                    }
                    deltaphi += fFreeInc; // increment
                    ++iter;
                    continue;
                }
                lowerChecked = curphi;
                if( isJointRevolute != 0 && upperChecked - lowerChecked >= 2*PI-fFreeInc*0.1 ) {
                    // reached full circle
                    break;
                }

                deltaphi += fFreeInc; // increment
            }

            iter++;

            if( RaveFabs(curphi) <= g_fEpsilonJointLimit ) {
                bIsZeroTested = true;
            }
            //RAVELOG_VERBOSE_FORMAT("index=%d curphi=%.16e, range=%.16e", freeindex%curphi%(upperChecked - lowerChecked));
            vfree.at(freeindex) = curphi;
            IkReturnAction res = ComposeSolution(vfreeparams, vfree, freeindex+1,q0, fn, vFreeInc);
            if( !(res & IKRA_Reject) ) {
                return res;
            }
            if( res & IKRA_Quit ) {
                return res;
            }
            allres |= res;
        }

        // explicitly test 0 since many edge cases involve 0s
        if( !bIsZeroTested && _qlower[vfreeparams[freeindex]] <= 0 && _qupper[vfreeparams[freeindex]] >= 0 ) {
            vfree.at(freeindex) = 0;
            IkReturnAction res = ComposeSolution(vfreeparams, vfree, freeindex+1,q0, fn, vFreeInc);
            if( !(res & IKRA_Reject) ) {
                return res;
            }
            allres |= res;
        }

        return static_cast<IkReturnAction>(allres);
    }

    /// \param tLocalTool _pmanip->GetLocalToolTransform()
    inline bool _CallIk(const IkParameterization& param, const vector<IkReal>& vfree, const Transform& tLocalTool, ikfast::IkSolutionList<IkReal>& solutions)
    {
        bool bsuccess = false;
        if( !!_ikfunctions->_ComputeIk2 ) {
            bsuccess = _CallIk2(param, vfree, tLocalTool, solutions);
        }
        else {
            bsuccess = _CallIk1(param, vfree, tLocalTool, solutions);
        }
        return bsuccess;
    }

    bool _CallIk1(const IkParameterization& param, const vector<IkReal>& vfree, const Transform& tLocalTool, ikfast::IkSolutionList<IkReal>& solutions)
    {
        try {
            switch(param.GetType()) {
            case IKP_Transform6D: {
                TransformMatrix t = param.GetTransform6D();
                if( _bEmptyTransform6D ) {
                    t = t * tLocalTool.inverse();
                }
                IkReal eetrans[3] = {(IkReal)t.trans.x, (IkReal)t.trans.y, (IkReal)t.trans.z};
                IkReal eerot[9] = {(IkReal)t.m[0],(IkReal)t.m[1],(IkReal)t.m[2],(IkReal)t.m[4],(IkReal)t.m[5],(IkReal)t.m[6],(IkReal)t.m[8],(IkReal)t.m[9],(IkReal)t.m[10]};
//                stringstream ss; ss << "./ik " << std::setprecision(16);
//                ss << eerot[0]  << " " << eerot[1]  << " " << eerot[2]  << " " << eetrans[0]  << " " << eerot[3]  << " " << eerot[4]  << " " << eerot[5]  << " " << eetrans[1]  << " " << eerot[6]  << " " << eerot[7]  << " " << eerot[8]  << " " << eetrans[2] << " ";
//                FOREACH(itfree,vfree) {
//                    ss << *itfree << " ";
//                }
//                ss << endl;
//                RAVELOG_INFO(ss.str());
                return _ikfunctions->_ComputeIk(eetrans, eerot, vfree.size()>0 ? &vfree[0] : NULL, solutions);
            }
            case IKP_Rotation3D: {
                TransformMatrix t(Transform(param.GetRotation3D(),Vector()));
                IkReal eerot[9] = {(IkReal)t.m[0],(IkReal)t.m[1],(IkReal)t.m[2],(IkReal)t.m[4],(IkReal)t.m[5],(IkReal)t.m[6],(IkReal)t.m[8],(IkReal)t.m[9],(IkReal)t.m[10]};
                return _ikfunctions->_ComputeIk(NULL, eerot, vfree.size()>0 ? &vfree[0] : NULL, solutions);
            }
            case IKP_Translation3D: {
                Vector v = param.GetTranslation3D();
                IkReal eetrans[3] = {(IkReal)v.x, (IkReal)v.y, (IkReal)v.z};
                //                stringstream ss; ss << "./ik " << std::setprecision(16);
                //                ss << eetrans[0]  << " " << eetrans[1]  << " " << eetrans[2] << " ";
                //                FOREACH(itfree,vfree) {
                //                    ss << *itfree << " ";
                //                }
                //                ss << endl;
                //                RAVELOG_INFO(ss.str());
                return _ikfunctions->_ComputeIk(eetrans, NULL, vfree.size()>0 ? &vfree[0] : NULL, solutions);
            }
            case IKP_Direction3D: {
                Vector v = param.GetDirection3D();
                IkReal eerot[9] = {(IkReal)v.x, (IkReal)v.y, (IkReal)v.z, 0,0,0,0,0,0};
                return _ikfunctions->_ComputeIk(NULL, eerot, vfree.size()>0 ? &vfree[0] : NULL, solutions);
            }
            case IKP_Ray4D: {
                RAY r = param.GetRay4D();
                IkReal eetrans[3] = {(IkReal)r.pos.x, (IkReal)r.pos.y, (IkReal)r.pos.z};
                IkReal eerot[9] = {(IkReal)r.dir.x, (IkReal)r.dir.y, (IkReal)r.dir.z, 0,0,0,0,0,0};
                //RAVELOG_INFO("ray: %f %f %f %f %f %f\n",eerot[0],eerot[1],eerot[2],eetrans[0],eetrans[1],eetrans[2]);
                if( !_ikfunctions->_ComputeIk(eetrans, eerot, vfree.size()>0 ? &vfree[0] : NULL, solutions) ) {
                    return false;
                }
                return true;
            }
            case IKP_Lookat3D: {
                Vector v = param.GetLookat3D();
                IkReal eetrans[3] = {(IkReal)v.x, (IkReal)v.y, (IkReal)v.z};
                return _ikfunctions->_ComputeIk(eetrans, NULL, vfree.size()>0 ? &vfree[0] : NULL, solutions);
            }
            case IKP_TranslationDirection5D: {
                RAY r = param.GetTranslationDirection5D();
                IkReal eetrans[3] = {(IkReal)r.pos.x, (IkReal)r.pos.y, (IkReal)r.pos.z};
                IkReal eerot[9] = {(IkReal)r.dir.x, (IkReal)r.dir.y, (IkReal)r.dir.z, 0,0,0,0,0,0};
                if( !_ikfunctions->_ComputeIk(eetrans, eerot, vfree.size()>0 ? &vfree[0] : NULL, solutions) ) {
                    return false;
                }
                return true;
            }
            case IKP_TranslationXY2D: {
                Vector v = param.GetTranslationXY2D();
                IkReal eetrans[3] = {(IkReal)v.x, (IkReal)v.y, 0};
                return _ikfunctions->_ComputeIk(eetrans, NULL, vfree.size()>0 ? &vfree[0] : NULL, solutions);
            }
            case IKP_TranslationXYOrientation3D: {
                //Vector v = param.GetTranslationXYOrientation3D();
//                IkReal eetrans[3] = {v.x, v.y,v.z};
//                return _ikfunctions->_ComputeIk(eetrans, NULL, vfree.size()>0 ? &vfree[0] : NULL, solutions);
                //std::pair<Vector,dReal> p = param.GetTranslationXAxisAngleZNorm4D();
                Vector v = param.GetTranslationXYOrientation3D();
                IkReal eetrans[3] = {(IkReal)v.x, (IkReal)v.y, 0};
                IkReal eerot[9] = {(IkReal)v.z, 0, 0, 0, 0, 0, 0, 0, 0};
                return _ikfunctions->_ComputeIk(eetrans, eerot, vfree.size()>0 ? &vfree[0] : NULL, solutions);
            }
            case IKP_TranslationLocalGlobal6D: {
                std::pair<Vector,Vector> p = param.GetTranslationLocalGlobal6D();
                IkReal eetrans[3] = {(IkReal)p.second.x, (IkReal)p.second.y, (IkReal)p.second.z};
                IkReal eerot[9] = {(IkReal)p.first.x, 0, 0, 0, (IkReal)p.first.y, 0, 0, 0, (IkReal)p.first.z};
                return _ikfunctions->_ComputeIk(eetrans, eerot, vfree.size()>0 ? &vfree[0] : NULL, solutions);
            }
            case IKP_TranslationXAxisAngle4D: {
                std::pair<Vector,dReal> p = param.GetTranslationXAxisAngle4D();
                IkReal eetrans[3] = {(IkReal)p.first.x, (IkReal)p.first.y, (IkReal)p.first.z};
                IkReal eerot[9] = {(IkReal)p.second, 0, 0, 0, 0, 0, 0, 0, 0};
                return _ikfunctions->_ComputeIk(eetrans, eerot, vfree.size()>0 ? &vfree[0] : NULL, solutions);
            }
            case IKP_TranslationYAxisAngle4D: {
                std::pair<Vector,dReal> p = param.GetTranslationYAxisAngle4D();
                IkReal eetrans[3] = {(IkReal)p.first.x, (IkReal)p.first.y, (IkReal)p.first.z};
                IkReal eerot[9] = {(IkReal)p.second, 0, 0, 0, 0, 0, 0, 0, 0};
                return _ikfunctions->_ComputeIk(eetrans, eerot, vfree.size()>0 ? &vfree[0] : NULL, solutions);
            }
            case IKP_TranslationZAxisAngle4D: {
                std::pair<Vector,dReal> p = param.GetTranslationZAxisAngle4D();
                IkReal eetrans[3] = {(IkReal)p.first.x, (IkReal)p.first.y, (IkReal)p.first.z};
                IkReal eerot[9] = {(IkReal)p.second, 0, 0, 0, 0, 0, 0, 0, 0};
                return _ikfunctions->_ComputeIk(eetrans, eerot, vfree.size()>0 ? &vfree[0] : NULL, solutions);
            }
            case IKP_TranslationXAxisAngleZNorm4D: {
                std::pair<Vector,dReal> p = param.GetTranslationXAxisAngleZNorm4D();
                IkReal eetrans[3] = {(IkReal)p.first.x, (IkReal)p.first.y, (IkReal)p.first.z};
                IkReal eerot[9] = {(IkReal)p.second, 0, 0, 0, 0, 0, 0, 0, 0};
                return _ikfunctions->_ComputeIk(eetrans, eerot, vfree.size()>0 ? &vfree[0] : NULL, solutions);
            }
            case IKP_TranslationYAxisAngleXNorm4D: {
                std::pair<Vector,dReal> p = param.GetTranslationYAxisAngleXNorm4D();
                IkReal eetrans[3] = {(IkReal)p.first.x, (IkReal)p.first.y, (IkReal)p.first.z};
                IkReal eerot[9] = {(IkReal)p.second, 0, 0, 0, 0, 0, 0, 0, 0};
                return _ikfunctions->_ComputeIk(eetrans, eerot, vfree.size()>0 ? &vfree[0] : NULL, solutions);
            }
            case IKP_TranslationZAxisAngleYNorm4D: {
                std::pair<Vector,dReal> p = param.GetTranslationZAxisAngleYNorm4D();
                IkReal eetrans[3] = {(IkReal)p.first.x, (IkReal)p.first.y, (IkReal)p.first.z};
                IkReal eerot[9] = {(IkReal)p.second, 0, 0, 0, 0, 0, 0, 0, 0};
                //TODO Code review! using &vfree[0] fails 100% of the time for hashimoto, NULL works normally. I don't know why!
                return _ikfunctions->_ComputeIk(eetrans, eerot, NULL, solutions);
            }
            default:
                BOOST_ASSERT(0);
                break;
            }
        }
        catch(const std::exception& e) {
            RAVELOG_WARN(str(boost::format("ik call failed for ik %s:0x%x: %s")%GetXMLId()%param.GetType()%e.what()));
            return false;
        }

        throw openrave_exception(str(boost::format(_("don't support ik parameterization 0x%x"))%param.GetType()),ORE_InvalidArguments);
    }

    bool _CallIk2(const IkParameterization& param, const vector<IkReal>& vfree, const Transform& tLocalTool, ikfast::IkSolutionList<IkReal>& solutions)
    {
        RobotBase::ManipulatorPtr pmanip = _pmanip.lock();
        try {
            switch(param.GetType()) {
            case IKP_Transform6D: {
                TransformMatrix t = param.GetTransform6D();
                if( _bEmptyTransform6D ) {
                    t = t * tLocalTool.inverse();
                }
                IkReal eetrans[3] = {(IkReal)t.trans.x, (IkReal)t.trans.y, (IkReal)t.trans.z};
                IkReal eerot[9] = {(IkReal)t.m[0],(IkReal)t.m[1],(IkReal)t.m[2],(IkReal)t.m[4],(IkReal)t.m[5],(IkReal)t.m[6],(IkReal)t.m[8],(IkReal)t.m[9],(IkReal)t.m[10]};
//                RobotBase::ManipulatorPtr pmanip(_pmanip);
//                stringstream ss; ss << pmanip->GetRobot()->GetName() << ":" << pmanip->GetName() << " ./ik " << std::setprecision(17);
//                ss << eerot[0]  << " " << eerot[1]  << " " << eerot[2]  << " " << eetrans[0]  << " " << eerot[3]  << " " << eerot[4]  << " " << eerot[5]  << " " << eetrans[1]  << " " << eerot[6]  << " " << eerot[7]  << " " << eerot[8]  << " " << eetrans[2] << " ";
//                FOREACH(itfree,vfree) {
//                    ss << *itfree << " ";
//                }
//                ss << endl;
                bool bret = _ikfunctions->_ComputeIk2(eetrans, eerot, vfree.size()>0 ? &vfree[0] : NULL, solutions, &pmanip);
                if( !bret ) {
#ifdef OPENRAVE_HAS_LAPACK
                    if( _fRefineWithJacobianInverseAllowedError > 0 ) {
                        // since will be refining, can add a little error to see if IK gets recomputed
                        eetrans[0] += 0.001;
                        eetrans[1] += 0.001;
                        eetrans[2] += 0.001;
                        bret = _ikfunctions->_ComputeIk2(eetrans, eerot, vfree.size()>0 ? &vfree[0] : NULL, solutions, &pmanip);
                        if( !bret ) {
                            eetrans[0] -= 0.002;
                            eetrans[1] -= 0.002;
                            eetrans[2] -= 0.002;
                            bret = _ikfunctions->_ComputeIk2(eetrans, eerot, vfree.size()>0 ? &vfree[0] : NULL, solutions, &pmanip);
                        }
                        RAVELOG_VERBOSE("ik failed, trying with slight jitter, ret=%d", (int)bret);
                    }
#endif
                }
//                ss << "ret=" << bret << " numsols=" << solutions.GetNumSolutions();
//                RAVELOG_INFO(ss.str());
                return bret;
            }
            case IKP_Rotation3D: {
                TransformMatrix t(Transform(param.GetRotation3D(),Vector()));
                IkReal eerot[9] = {(IkReal)t.m[0],(IkReal)t.m[1],(IkReal)t.m[2],(IkReal)t.m[4],(IkReal)t.m[5],(IkReal)t.m[6],(IkReal)t.m[8],(IkReal)t.m[9],(IkReal)t.m[10]};
                return _ikfunctions->_ComputeIk2(NULL, eerot, vfree.size()>0 ? &vfree[0] : NULL, solutions, &pmanip);
            }
            case IKP_Translation3D: {
                Vector v = param.GetTranslation3D();
                IkReal eetrans[3] = {(IkReal)v.x, (IkReal)v.y, (IkReal)v.z};
                //                stringstream ss; ss << "./ik " << std::setprecision(16);
                //                ss << eetrans[0]  << " " << eetrans[1]  << " " << eetrans[2] << " ";
                //                FOREACH(itfree,vfree) {
                //                    ss << *itfree << " ";
                //                }
                //                ss << endl;
                //                RAVELOG_INFO(ss.str());
                return _ikfunctions->_ComputeIk2(eetrans, NULL, vfree.size()>0 ? &vfree[0] : NULL, solutions, &pmanip);
            }
            case IKP_Direction3D: {
                Vector v = param.GetDirection3D();
                IkReal eerot[9] = {(IkReal)v.x, (IkReal)v.y, (IkReal)v.z, 0,0,0,0,0,0};
                return _ikfunctions->_ComputeIk2(NULL, eerot, vfree.size()>0 ? &vfree[0] : NULL, solutions, &pmanip);
            }
            case IKP_Ray4D: {
                RAY r = param.GetRay4D();
                IkReal eetrans[3] = {(IkReal)r.pos.x, (IkReal)r.pos.y, (IkReal)r.pos.z};
                IkReal eerot[9] = {(IkReal)r.dir.x, (IkReal)r.dir.y, (IkReal)r.dir.z, 0,0,0,0,0,0};
                //RAVELOG_INFO("ray: %f %f %f %f %f %f\n",eerot[0],eerot[1],eerot[2],eetrans[0],eetrans[1],eetrans[2]);
                if( !_ikfunctions->_ComputeIk2(eetrans, eerot, vfree.size()>0 ? &vfree[0] : NULL, solutions, &pmanip) ) {
                    return false;
                }
                return true;
            }
            case IKP_Lookat3D: {
                Vector v = param.GetLookat3D();
                IkReal eetrans[3] = {(IkReal)v.x, (IkReal)v.y, (IkReal)v.z};
                return _ikfunctions->_ComputeIk2(eetrans, NULL, vfree.size()>0 ? &vfree[0] : NULL, solutions, &pmanip);
            }
            case IKP_TranslationDirection5D: {
                RAY r = param.GetTranslationDirection5D();
                IkReal eetrans[3] = {(IkReal)r.pos.x, (IkReal)r.pos.y, (IkReal)r.pos.z};
                IkReal eerot[9] = {(IkReal)r.dir.x, (IkReal)r.dir.y, (IkReal)r.dir.z, 0,0,0,0,0,0};
                bool bret = _ikfunctions->_ComputeIk2(eetrans, eerot, vfree.size()>0 ? &vfree[0] : NULL, solutions, &pmanip);
                if( !bret ) {
#ifdef OPENRAVE_HAS_LAPACK
                    if( _fRefineWithJacobianInverseAllowedError > 0 ) {
                        // since will be refining, can add a little error to see if IK gets recomputed
                        eerot[0] = r.dir.x+0.01;
                        eerot[1] = r.dir.y+0.01;
                        eerot[2] = r.dir.z+0.01;
                        dReal fnorm = RaveSqrt(eerot[0]*eerot[0] + eerot[1]*eerot[1] + eerot[2]*eerot[2]);
                        eerot[0] /= fnorm;
                        eerot[1] /= fnorm;
                        eerot[2] /= fnorm;
                        bret = _ikfunctions->_ComputeIk2(eetrans, eerot, vfree.size()>0 ? &vfree[0] : NULL, solutions, &pmanip);
                        if( !bret ) {
                            eerot[0] = r.dir.x-0.01;
                            eerot[1] = r.dir.y-0.01;
                            eerot[2] = r.dir.z-0.01;
                            dReal fnorm = RaveSqrt(eerot[0]*eerot[0] + eerot[1]*eerot[1] + eerot[2]*eerot[2]);
                            eerot[0] /= fnorm;
                            eerot[1] /= fnorm;
                            eerot[2] /= fnorm;
                            bret = _ikfunctions->_ComputeIk2(eetrans, eerot, vfree.size()>0 ? &vfree[0] : NULL, solutions, &pmanip);
                            if( !bret ) {
                                eerot[0] = r.dir.x-0.04;
                                eerot[1] = r.dir.y-0.04;
                                eerot[2] = r.dir.z;
                                dReal fnorm = RaveSqrt(eerot[0]*eerot[0] + eerot[1]*eerot[1] + eerot[2]*eerot[2]);
                                eerot[0] /= fnorm;
                                eerot[1] /= fnorm;
                                eerot[2] /= fnorm;
                                bret = _ikfunctions->_ComputeIk2(eetrans, eerot, vfree.size()>0 ? &vfree[0] : NULL, solutions, &pmanip);
                                if( !bret ) {
                                    eerot[0] = r.dir.x+0.04;
                                    eerot[1] = r.dir.y+0.04;
                                    eerot[2] = r.dir.z;
                                    dReal fnorm = RaveSqrt(eerot[0]*eerot[0] + eerot[1]*eerot[1] + eerot[2]*eerot[2]);
                                    eerot[0] /= fnorm;
                                    eerot[1] /= fnorm;
                                    eerot[2] /= fnorm;
                                    bret = _ikfunctions->_ComputeIk2(eetrans, eerot, vfree.size()>0 ? &vfree[0] : NULL, solutions, &pmanip);
                                }
                            }
                        }
                        RAVELOG_VERBOSE("ik failed, trying with slight jitter, ret=%d", (int)bret);
                    }
#endif
                }

                return bret;
            }
            case IKP_TranslationXY2D: {
                Vector v = param.GetTranslationXY2D();
                IkReal eetrans[3] = {(IkReal)v.x, (IkReal)v.y, 0};
                return _ikfunctions->_ComputeIk2(eetrans, NULL, vfree.size()>0 ? &vfree[0] : NULL, solutions, &pmanip);
            }
            case IKP_TranslationXYOrientation3D: {
                //Vector v = param.GetTranslationXYOrientation3D();
//                IkReal eetrans[3] = {v.x, v.y,v.z};
//                return _ikfunctions->_ComputeIk2(eetrans, NULL, vfree.size()>0 ? &vfree[0] : NULL, solutions, &pmanip);
                //std::pair<Vector,dReal> p = param.GetTranslationXAxisAngleZNorm4D();
                Vector v = param.GetTranslationXYOrientation3D();
                IkReal eetrans[3] = {(IkReal)v.x, (IkReal)v.y, 0};
                IkReal eerot[9] = {(IkReal)v.z, 0, 0, 0, 0, 0, 0, 0, 0};
                return _ikfunctions->_ComputeIk2(eetrans, eerot, vfree.size()>0 ? &vfree[0] : NULL, solutions, &pmanip);
            }
            case IKP_TranslationLocalGlobal6D: {
                std::pair<Vector,Vector> p = param.GetTranslationLocalGlobal6D();
                IkReal eetrans[3] = {(IkReal)p.second.x, (IkReal)p.second.y, (IkReal)p.second.z};
                IkReal eerot[9] = {(IkReal)p.first.x, 0, 0, 0, (IkReal)p.first.y, 0, 0, 0, (IkReal)p.first.z};
                return _ikfunctions->_ComputeIk2(eetrans, eerot, vfree.size()>0 ? &vfree[0] : NULL, solutions, &pmanip);
            }
            case IKP_TranslationXAxisAngle4D: {
                std::pair<Vector,dReal> p = param.GetTranslationXAxisAngle4D();
                IkReal eetrans[3] = {(IkReal)p.first.x, (IkReal)p.first.y, (IkReal)p.first.z};
                IkReal eerot[9] = {(IkReal)p.second, 0, 0, 0, 0, 0, 0, 0, 0};
                return _ikfunctions->_ComputeIk2(eetrans, eerot, vfree.size()>0 ? &vfree[0] : NULL, solutions, &pmanip);
            }
            case IKP_TranslationYAxisAngle4D: {
                std::pair<Vector,dReal> p = param.GetTranslationYAxisAngle4D();
                IkReal eetrans[3] = {(IkReal)p.first.x, (IkReal)p.first.y, (IkReal)p.first.z};
                IkReal eerot[9] = {(IkReal)p.second, 0, 0, 0, 0, 0, 0, 0, 0};
                return _ikfunctions->_ComputeIk2(eetrans, eerot, vfree.size()>0 ? &vfree[0] : NULL, solutions, &pmanip);
            }
            case IKP_TranslationZAxisAngle4D: {
                std::pair<Vector,dReal> p = param.GetTranslationZAxisAngle4D();
                IkReal eetrans[3] = {(IkReal)p.first.x, (IkReal)p.first.y, (IkReal)p.first.z};
                IkReal eerot[9] = {(IkReal)p.second, 0, 0, 0, 0, 0, 0, 0, 0};
                return _ikfunctions->_ComputeIk2(eetrans, eerot, vfree.size()>0 ? &vfree[0] : NULL, solutions, &pmanip);
            }
            case IKP_TranslationXAxisAngleZNorm4D: {
                std::pair<Vector,dReal> p = param.GetTranslationXAxisAngleZNorm4D();
                IkReal eetrans[3] = {(IkReal)p.first.x, (IkReal)p.first.y, (IkReal)p.first.z};
                IkReal eerot[9] = {(IkReal)p.second, 0, 0, 0, 0, 0, 0, 0, 0};
                return _ikfunctions->_ComputeIk2(eetrans, eerot, vfree.size()>0 ? &vfree[0] : NULL, solutions, &pmanip);
            }
            case IKP_TranslationYAxisAngleXNorm4D: {
                std::pair<Vector,dReal> p = param.GetTranslationYAxisAngleXNorm4D();
                IkReal eetrans[3] = {(IkReal)p.first.x, (IkReal)p.first.y, (IkReal)p.first.z};
                IkReal eerot[9] = {(IkReal)p.second, 0, 0, 0, 0, 0, 0, 0, 0};
                return _ikfunctions->_ComputeIk2(eetrans, eerot, vfree.size()>0 ? &vfree[0] : NULL, solutions, &pmanip);
            }
            case IKP_TranslationZAxisAngleYNorm4D: {
                std::pair<Vector,dReal> p = param.GetTranslationZAxisAngleYNorm4D();
                IkReal eetrans[3] = {(IkReal)p.first.x, (IkReal)p.first.y, (IkReal)p.first.z};
                IkReal eerot[9] = {(IkReal)p.second, 0, 0, 0, 0, 0, 0, 0, 0};
                //TODO Code review! using NULL fails crash the slave for hashimoto, &vfree[0] works normally. I don't know why!
                return _ikfunctions->_ComputeIk2(eetrans, eerot, vfree.size()>0 ? &vfree[0] : NULL, solutions, &pmanip);
            }
            default:
                BOOST_ASSERT(0);
                break;
            }
        }
        catch(const std::exception& e) {
            RAVELOG_WARN(str(boost::format("ik call failed for ik %s:0x%x: %s")%GetXMLId()%param.GetType()%e.what()));
            return false;
        }

        throw openrave_exception(str(boost::format(_("don't support ik parameterization 0x%x"))%param.GetType()),ORE_InvalidArguments);
    }

    static bool SortSolutionDistances(const pair<size_t,dReal>& p1, const pair<size_t,dReal>& p2)
    {
        return p1.second < p2.second;
    }

    IkReturnAction _SolveSingle(const IkParameterization& param, const vector<IkReal>& vfree, const vector<dReal>& q0, int filteroptions, IkReturnPtr ikreturn, StateCheckEndEffector& stateCheck)
    {
        RobotBase::ManipulatorPtr pmanip(_pmanip);
        ikfast::IkSolutionList<IkReal> solutions;
        if( !_CallIk(param,vfree, pmanip->GetLocalToolTransform(), solutions) ) {
            return IKRA_RejectKinematics;
        }

        RobotBasePtr probot = pmanip->GetRobot();
        SolutionInfo bestsolution;
        std::vector<dReal> vravesol(pmanip->GetArmIndices().size());
        std::vector<IkReal> sol(pmanip->GetArmIndices().size()), vsolfree;
        // find the first valid solution that satisfies joint constraints and collisions
        boost::tuple<const vector<IkReal>&, const vector<dReal>&,int> textra(vsolfree, q0, filteroptions);

        vector<int> vsolutionorder(solutions.GetNumSolutions());
        if( vravesol.size() == q0.size() ) {
            // sort the solutions from closest to farthest
            vector<pair<size_t,dReal> > vdists; vdists.reserve(solutions.GetNumSolutions());
            for(size_t isolution = 0; isolution < solutions.GetNumSolutions(); ++isolution) {
                const ikfast::IkSolution<IkReal>& iksol = dynamic_cast<const ikfast::IkSolution<IkReal>& >(solutions.GetSolution(isolution));
                iksol.Validate();
                vsolfree.resize(iksol.GetFree().size());
                for(size_t ifree = 0; ifree < iksol.GetFree().size(); ++ifree) {
                    vsolfree[ifree] = q0.at(iksol.GetFree()[ifree]);
                }
                iksol.GetSolution(sol,vsolfree);
                for(int i = 0; i < iksol.GetDOF(); ++i) {
                    vravesol.at(i) = (dReal)sol[i];
                }
                vdists.emplace_back(vdists.size(),_ComputeGeometricConfigDistSqr(probot,vravesol,q0, true));
            }

            std::stable_sort(vdists.begin(),vdists.end(),SortSolutionDistances);
            for(size_t i = 0; i < vsolutionorder.size(); ++i) {
                vsolutionorder[i] = vdists[i].first;
            }
        }
        else {
            for(size_t i = 0; i < vsolutionorder.size(); ++i) {
                vsolutionorder[i] = i;
            }
        }

        int allres = IKRA_Reject;
        IkParameterization paramnewglobal; // needs to be initialized by _ValidateSolutionSingle so we get most accurate result back
        FOREACH(itindex,vsolutionorder) {
            const ikfast::IkSolution<IkReal>& iksol = dynamic_cast<const ikfast::IkSolution<IkReal>& >(solutions.GetSolution(*itindex));
            IkReturnAction res;
            if( iksol.GetFree().size() > 0 ) {
                // have to search over all the free parameters of the solution!
                vsolfree.resize(iksol.GetFree().size());
                std::vector<dReal> vFreeInc(_GetFreeIncFromIndices(iksol.GetFree()));
                res = ComposeSolution(iksol.GetFree(), vsolfree, 0, q0, boost::bind(&IkFastSolver::_ValidateSolutionSingle,shared_solver(), boost::ref(iksol), boost::ref(textra), boost::ref(sol), boost::ref(vravesol), boost::ref(bestsolution), boost::ref(param), boost::ref(stateCheck), boost::ref(paramnewglobal)), vFreeInc);
            }
            else {
                vsolfree.resize(0);
                res = _ValidateSolutionSingle(iksol, textra, sol, vravesol, bestsolution, param, stateCheck, paramnewglobal);
            }
            allres |= res;
            if( res & IKRA_Quit ) {
                // return the accumulated errors
                return static_cast<IkReturnAction>(allres);
            }
            // stop if there is no solution we are attempting to get close to
            if( res == IKRA_Success && q0.size() != pmanip->GetArmIndices().size() ) {
                break;
            }
        }

        // return as soon as a solution is found, since we're visiting phis starting from q0, we are guaranteed
        // that the solution will be close (ie, phi's dominate in the search). This is to speed things up
        if( !!bestsolution.ikreturn ) {
            if( !!ikreturn ) {
                *ikreturn = *bestsolution.ikreturn;
            }
            _CallFinishCallbacks(bestsolution.ikreturn, pmanip, paramnewglobal);
            return bestsolution.ikreturn->_action;
        }
        return static_cast<IkReturnAction>(allres);
    }

    void _CheckRefineSolution(const IkParameterization& param, const RobotBase::Manipulator& manip, std::vector<dReal>& vsolution, bool bIgnoreJointLimits)
    {
#ifdef OPENRAVE_HAS_LAPACK
        IkParameterization paramnew = manip.GetIkParameterization(param,false);
        dReal ikworkspacedist = param.ComputeDistanceSqr(paramnew);
        if( _fRefineWithJacobianInverseAllowedError > 0 && ikworkspacedist > _fRefineWithJacobianInverseAllowedError*_fRefineWithJacobianInverseAllowedError ) {
            if( param.GetType() == IKP_Transform6D ) { // only 6d supported for now
                int ret = _jacobinvsolver.ComputeSolution(param.GetTransform6D(), manip, vsolution, bIgnoreJointLimits);
                if( ret == 2 ) {
                    RAVELOG_VERBOSE("did not converge, try to prioritize translation at least\n");
                    ret = _jacobinvsolver.ComputeSolutionTranslation(param.GetTransform6D(), manip, vsolution, bIgnoreJointLimits);
                }
                if( ret == 0 ) {
                    stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
                    ss << "IkParameterization('" << param << "'); sol=[";
                    FOREACH(it, vsolution) {
                        ss << *it << ",";
                    }
                    ss << "]";
                    RAVELOG_WARN_FORMAT("failed to refine solution lasterror=%f, %s", RaveSqrt(_jacobinvsolver._lasterror2)%ss.str());
                }
            }
            else if( param.GetType() == IKP_TranslationDirection5D ) { // only 6d supported for now
                // have to project the current manip into the direction constraints so that direction aligns perfectly
                Transform tgoal;
                Transform tnewmanip = manip.GetBase()->GetTransform().inverse()*manip.GetTransform();
                tgoal.rot = quatRotateDirection(manip.GetLocalToolDirection(), param.GetTranslationDirection5D().dir);

                dReal frotangle0 = normalizeAxisRotation(param.GetTranslationDirection5D().dir, tnewmanip.rot).first;
                dReal frotanglegoal = normalizeAxisRotation(param.GetTranslationDirection5D().dir, tgoal.rot).first;
                tgoal.rot = quatMultiply(quatFromAxisAngle(param.GetTranslationDirection5D().dir, frotanglegoal - frotangle0), tgoal.rot);

                tgoal.trans = param.GetTranslationDirection5D().pos;
                int ret = _jacobinvsolver.ComputeSolution(tgoal, manip, vsolution, bIgnoreJointLimits);
                if( ret == 2 ) {
                    RAVELOG_VERBOSE("did not converge, try to prioritize translation at least\n");
                    ret = _jacobinvsolver.ComputeSolutionTranslation(tgoal, manip, vsolution, bIgnoreJointLimits);
                }
                if( ret == 0 ) {
                    stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
                    ss << "IkParameterization('" << param << "'); sol=[";
                    FOREACH(it, vsolution) {
                        ss << *it << ",";
                    }
                    ss << "]";
                    RAVELOG_WARN_FORMAT("failed to refine solution lasterror=%f, %s", RaveSqrt(_jacobinvsolver._lasterror2)%ss.str());
                }
            }
        }
#endif
    }


    /// validate a solution
    /// \param paramnewglobal[out]
    IkReturnAction _ValidateSolutionSingle(const ikfast::IkSolution<IkReal>& iksol, boost::tuple<const vector<IkReal>&, const vector<dReal>&, int>& freeq0check, std::vector<IkReal>& sol, std::vector<dReal>& vravesol, SolutionInfo& bestsolution, const IkParameterization& param, StateCheckEndEffector& stateCheck, IkParameterization& paramnewglobal)
    {
        const vector<IkReal>& vfree = boost::get<0>(freeq0check);
        //BOOST_ASSERT(sol.size()== iksol.basesol.size() && vfree.size() == iksol.GetFree().size());
        iksol.GetSolution(sol,vfree);
        for(int i = 0; i < (int)sol.size(); ++i) {
            vravesol.at(i) = dReal(sol[i]);
        }

        int nSameStateRepeatCount = 0;
        _nSameStateRepeatCount = 0;
        std::vector<unsigned int> vsolutionindices;
        iksol.GetSolutionIndices(vsolutionindices);

        RobotBase::ManipulatorPtr pmanip(_pmanip);
        RobotBasePtr probot = pmanip->GetRobot();

        std::vector< std::pair<std::vector<dReal>, int> > vravesols;
        list<IkReturnPtr> listlocalikreturns; // orderd with respect to vravesols

        /// if have to check for closest solution, make sure this new solution is closer than best found so far
        //dReal d = dReal(1e30);

        int filteroptions = boost::get<2>(freeq0check);
        if( !(filteroptions&IKFO_IgnoreJointLimits) ) {
            _ComputeAllSimilarJointAngles(vravesols, vravesol);
            if( boost::get<1>(freeq0check).size() == vravesol.size() ) {
                std::vector< std::pair<std::vector<dReal>, int> > vravesols2;
                // if all the solutions are worse than the best, then ignore everything
                vravesols2.reserve(vravesols.size());
                FOREACH(itravesol, vravesols) {
                    dReal d = _ComputeGeometricConfigDistSqr(probot,itravesol->first,boost::get<1>(freeq0check));
                    if( !(bestsolution.dist <= d) ) {
                        vravesols2.push_back(*itravesol);
                    }
                }
                vravesols.swap(vravesols2);
            }
            if( vravesols.size() == 0 ) {
                return IKRA_RejectJointLimits;
            }
        }
        else {
            if( boost::get<1>(freeq0check).size() == vravesol.size() ) {
                dReal d = _ComputeGeometricConfigDistSqr(probot,vravesol,boost::get<1>(freeq0check));
                if( bestsolution.dist <= d ) {
                    return IKRA_Reject;
                }
            }
            vravesols.emplace_back(vravesol, 0);
        }

        IkParameterization paramnew;

        int retactionall = IKRA_Reject;
        if( !(filteroptions & IKFO_IgnoreCustomFilters) && _HasFilterInRange(1, IKSP_MaxPriority) ) {
//            unsigned int maxsolutions = 1;
//            for(size_t i = 0; i < iksol.basesol.size(); ++i) {
//                unsigned char m = iksol.basesol[i].maxsolutions;
//                if( m != (unsigned char)-1 && m > 1) {
//                    maxsolutions *= m;
//                }
//            }
            // TODO: iterating vravesols would call the filters vravesols.size() times even if a valid solution is found
            // figure out a way to do short-curcuit the code to check the final solutions
            FOREACH(itravesol, vravesols) {
                _vsolutionindices = vsolutionindices;
                FOREACH(it,_vsolutionindices) {
                    *it += itravesol->second<<16;
                }
                probot->SetActiveDOFValues(itravesol->first,false);
                _CheckRefineSolution(param, *pmanip, itravesol->first, !!(filteroptions&IKFO_IgnoreJointLimits));

                // due to floating-point precision, vravesol and param will not necessarily match anymore. The filters require perfectly matching pair, so compute a new param
                paramnew = pmanip->GetIkParameterization(param,false); // custom data is copied!
                paramnewglobal = pmanip->GetBase()->GetTransform() * paramnew;
                _nSameStateRepeatCount = nSameStateRepeatCount; // could be overwritten by _CallFilters call!
                IkReturnPtr localret(new IkReturn(IKRA_Success));
                localret->_mapdata["solutionindices"] = std::vector<dReal>(_vsolutionindices.begin(),_vsolutionindices.end());

                bool bNeedCheckEndEffectorEnvCollision = stateCheck.NeedCheckEndEffectorEnvCollision();
                if( !(filteroptions & IKFO_IgnoreEndEffectorEnvCollisions) ) {
                    // have to make sure end effector collisions are set, regardless if stateCheck.ResetCheckEndEffectorEnvCollision has been called
                    stateCheck.RestoreCheckEndEffectorEnvCollision();
                }
                IkReturnAction retaction = _CallFilters(itravesol->first, pmanip, paramnew,localret, 1, IKSP_MaxPriority);
                if( !(filteroptions & IKFO_IgnoreEndEffectorEnvCollisions) && !bNeedCheckEndEffectorEnvCollision ) {
                    stateCheck.ResetCheckEndEffectorEnvCollision();
                }
                nSameStateRepeatCount++;
                _nSameStateRepeatCount = nSameStateRepeatCount;
                retactionall |= retaction;
                if( retactionall & IKRA_Quit ) {
                    return static_cast<IkReturnAction>(retactionall|IKRA_RejectCustomFilter);
                }
                else if( retaction == IKRA_Success ) {
                    localret->_vsolution.swap(itravesol->first);
                    listlocalikreturns.push_back(localret);
                }
            }
            if( listlocalikreturns.size() == 0 ) {
                return static_cast<IkReturnAction>(retactionall|IKRA_RejectCustomFilter);
            }
        }
        else {
            FOREACH(itravesol, vravesols) {
                _vsolutionindices = vsolutionindices;
                FOREACH(it,_vsolutionindices) {
                    *it += itravesol->second<<16;
                }
                probot->SetActiveDOFValues(itravesol->first,false);
                _CheckRefineSolution(param, *pmanip, itravesol->first, !!(filteroptions&IKFO_IgnoreJointLimits));

                // due to floating-point precision, vravesol and param will not necessarily match anymore. The filters require perfectly matching pair, so compute a new param
                paramnew = pmanip->GetIkParameterization(param,false);
                paramnewglobal = pmanip->GetBase()->GetTransform() * paramnew;
                IkReturnPtr localret(new IkReturn(IKRA_Success));
                localret->_mapdata["solutionindices"] = std::vector<dReal>(_vsolutionindices.begin(),_vsolutionindices.end());
                localret->_vsolution.swap(itravesol->first);
                listlocalikreturns.push_back(localret);
            }
        }

        CollisionReport report;
        CollisionReportPtr ptempreport;
        if( !(filteroptions&IKFO_IgnoreSelfCollisions) || IS_DEBUGLEVEL(Level_Verbose) || paramnewglobal.GetType() == IKP_TranslationDirection5D ) { // 5D is necessary for tracking end effector collisions
            ptempreport = boost::shared_ptr<CollisionReport>(&report,utils::null_deleter());
        }
        if( !(filteroptions&IKFO_IgnoreSelfCollisions) ) {
            // check for self collisions
            stateCheck.SetSelfCollisionState();
            if( probot->CheckSelfCollision(ptempreport) ) {
                if( !!ptempreport ) {
                    if( !!ptempreport->plink1 && !!ptempreport->plink2 && (paramnewglobal.GetType() == IKP_Transform6D || paramnewglobal.GetDOF() >= pmanip->GetArmDOF()) ) {
                        // ik constraints the robot pretty well, so any self-collisions might mean the IK itself is impossible.
                        // check if self-colliding with non-moving part and a link that is pretty high in the chain, then perhaps we should give up...?
                        KinBody::LinkConstPtr potherlink;
                        if( find(_vindependentlinks.begin(), _vindependentlinks.end(), ptempreport->plink1) != _vindependentlinks.end() ) {
                            potherlink = ptempreport->plink2;
                        }
                        else if( find(_vindependentlinks.begin(), _vindependentlinks.end(), ptempreport->plink2) != _vindependentlinks.end() ) {
                            potherlink = ptempreport->plink1;
                        }

                        if( !!potherlink && _numBacktraceLinksForSelfCollisionWithNonMoving > 0 ) {
                            for(int itestdof = (int)pmanip->GetArmIndices().size()-_numBacktraceLinksForSelfCollisionWithNonMoving; itestdof < (int)pmanip->GetArmIndices().size(); ++itestdof) {
                                KinBody::JointPtr pjoint = probot->GetJointFromDOFIndex(pmanip->GetArmIndices()[itestdof]);
                                if( !!pjoint->GetHierarchyParentLink() && pjoint->GetHierarchyParentLink()->IsRigidlyAttached(*potherlink) ) {

                                    stateCheck.numImpossibleSelfCollisions++;
                                    RAVELOG_VERBOSE_FORMAT("self-collision with links %s and %s most likely means IK itself will not succeed, attempts=%d", ptempreport->plink1->GetName()%ptempreport->plink2->GetName()%stateCheck.numImpossibleSelfCollisions);
                                    if( stateCheck.numImpossibleSelfCollisions > 16 ) { // not sure what a good threshold is here
                                        RAVELOG_DEBUG_FORMAT("self-collision with links %s and %s most likely means IK itself will not succeed, giving up after %d attempts", ptempreport->plink1->GetName()%ptempreport->plink2->GetName()%stateCheck.numImpossibleSelfCollisions);
                                        return static_cast<IkReturnAction>(retactionall|IKRA_RejectSelfCollision|IKRA_Quit);
                                    }
                                    else {
                                        break;
                                    }
                                }
                            }
                        }

                        if( _vindependentlinks.size() != _vIndependentLinksIncludingFreeJoints.size() && _numBacktraceLinksForSelfCollisionWithFree > 0 ) {
                            // check
                            potherlink.reset();
                            if( find(_vIndependentLinksIncludingFreeJoints.begin(), _vIndependentLinksIncludingFreeJoints.end(), ptempreport->plink1) != _vIndependentLinksIncludingFreeJoints.end() ) {
                                potherlink = ptempreport->plink2;
                            }
                            else if( find(_vIndependentLinksIncludingFreeJoints.begin(), _vIndependentLinksIncludingFreeJoints.end(), ptempreport->plink2) != _vIndependentLinksIncludingFreeJoints.end() ) {
                                potherlink = ptempreport->plink1;
                            }

                            if( !!potherlink ) {
                                //bool bRigidlyAttached = false;
                                for(int itestdof = (int)pmanip->GetArmIndices().size()-_numBacktraceLinksForSelfCollisionWithFree; itestdof < (int)pmanip->GetArmIndices().size(); ++itestdof) {
                                    KinBody::JointPtr pjoint = probot->GetJointFromDOFIndex(pmanip->GetArmIndices()[itestdof]);
                                    if( !!pjoint->GetHierarchyParentLink() && pjoint->GetHierarchyParentLink()->IsRigidlyAttached(*potherlink) ) {

                                        stateCheck.numImpossibleSelfCollisions++;
                                        RAVELOG_VERBOSE_FORMAT("self-collision with links %s and %s most likely means IK itself will not succeed, attempts=%d", ptempreport->plink1->GetName()%ptempreport->plink2->GetName()%stateCheck.numImpossibleSelfCollisions);
                                        if( stateCheck.numImpossibleSelfCollisions > 16 ) { // not sure what a good threshold is here
                                            RAVELOG_DEBUG_FORMAT("self-collision with links %s and %s most likely means IK itself will not succeed for these free parameters, giving up after %d attempts", ptempreport->plink1->GetName()%ptempreport->plink2->GetName()%stateCheck.numImpossibleSelfCollisions);
                                            return static_cast<IkReturnAction>(retactionall|IKRA_RejectSelfCollision|IKRA_Quit);
                                        }
                                        else {
                                            break;
                                        }
                                    }
                                }
                            }
                        }

                    }
                }
                return static_cast<IkReturnAction>(retactionall|IKRA_RejectSelfCollision);
            }
        }
        if( filteroptions&IKFO_CheckEnvCollisions ) {
            stateCheck.SetEnvironmentCollisionState();
            if( stateCheck.NeedCheckEndEffectorEnvCollision() ) {
                // only check if the end-effector position is fully determined from the ik
                if( paramnewglobal.GetType() == IKP_Transform6D ) {// || (int)pmanip->GetArmIndices().size() <= paramnewglobal.GetDOF() ) {
                    // if gripper is colliding, solutions will always fail, so completely stop solution process
                    if( pmanip->CheckEndEffectorCollision(pmanip->GetTransform(), ptempreport) ) {
                        if( !!ptempreport ) {
                            stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
                            ss << "ikfast collision " << report.__str__() << " colvalues=[";
                            std::vector<dReal> vallvalues;
                            probot->GetDOFValues(vallvalues);
                            for(size_t i = 0; i < vallvalues.size(); ++i ) {
                                if( i > 0 ) {
                                    ss << "," << vallvalues[i];
                                }
                                else {
                                    ss << vallvalues[i];
                                }
                            }
                            ss << "]";
                            RAVELOG_VERBOSE(ss.str());
                        }
                        if( paramnewglobal.GetType() == IKP_Transform6D ) {
                            return static_cast<IkReturnAction>(retactionall|IKRA_QuitEndEffectorCollision); // stop the search
                        }
                        else {
                            // end effector could change depending on the solution
                            return static_cast<IkReturnAction>(retactionall|IKRA_RejectEnvCollision); // stop the search
                        }
                    }
                    stateCheck.ResetCheckEndEffectorEnvCollision();
                }
                else if( paramnewglobal.GetType() == IKP_TranslationDirection5D ) {
                    int colliding = stateCheck.IsCollidingEndEffector(pmanip->GetTransform());
                    if( colliding == 1 ) {
                        // end effector could change depending on the solution
                        return static_cast<IkReturnAction>(retactionall|IKRA_RejectEnvCollision); // stop the search
                    }
                }
            }
            if( GetEnv()->CheckCollision(KinBodyConstPtr(probot), ptempreport) ) {
                if( paramnewglobal.GetType() == IKP_TranslationDirection5D ) {
                    // colliding and 5d,so check if colliding with end effector. If yes, then register as part of the stateCheck
                    bool bIsEndEffectorCollision = false;
                    FOREACHC(itcollidingpairs, ptempreport->vLinkColliding) {
                        if( itcollidingpairs->first->GetParent() == probot ) {
                            if( find(_vchildlinkindices.begin(), _vchildlinkindices.end(), itcollidingpairs->first->GetIndex()) != _vchildlinkindices.end() ) {
                                bIsEndEffectorCollision = true;
                                break;
                            }
                        }
                        if( itcollidingpairs->second->GetParent() == probot ) {
                            if( find(_vchildlinkindices.begin(), _vchildlinkindices.end(), itcollidingpairs->second->GetIndex()) != _vchildlinkindices.end() ) {
                                bIsEndEffectorCollision = true;
                                break;
                            }
                        }
                    }
                    if( bIsEndEffectorCollision ) {
                        // only really matters if in collision
                        stateCheck.RegisterCollidingEndEffector(pmanip->GetTransform(), bIsEndEffectorCollision);
                    }
                }

                if( !!ptempreport ) {
                    stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
                    ss << "env=" << GetEnv()->GetId() << ", ikfast collision " << probot->GetName() << ":" << pmanip->GetName() << " " << report.__str__() << " colvalues=[";
                    std::vector<dReal> vallvalues;
                    probot->GetDOFValues(vallvalues);
                    for(size_t i = 0; i < vallvalues.size(); ++i ) {
                        if( i > 0 ) {
                            ss << "," << vallvalues[i];
                        }
                        else {
                            ss << vallvalues[i];
                        }
                    }
                    ss << "]";
                    RAVELOG_VERBOSE(ss.str());
                }
                return static_cast<IkReturnAction>(retactionall|IKRA_RejectEnvCollision);
            }
        }

        // check that end effector moved in the correct direction
        dReal ikworkspacedist = param.ComputeDistanceSqr(paramnew);
        if( ikworkspacedist > _ikthreshold ) {
            BOOST_ASSERT(listlocalikreturns.size()>0);
            stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
            ss << "ignoring bad ik for " << probot->GetName() << ":" << pmanip->GetName() << " dist=" << RaveSqrt(ikworkspacedist) << ", param=[" << param << "], sol=[";
            FOREACHC(itvalue,listlocalikreturns.front()->_vsolution) {
                ss << *itvalue << ", ";
            }
            ss << "]" << endl;
            RAVELOG_ERROR(ss.str());
            return static_cast<IkReturnAction>(retactionall|IKRA_RejectKinematicsPrecision);
        }

        if( (int)boost::get<1>(freeq0check).size() == _nTotalDOF ) {
            // order the listlocalikreturns depending on the distance to boost::get<1>(freeq0check), that way first solution is prioritized
            std::vector< std::pair<size_t, dReal> > vdists; vdists.reserve(listlocalikreturns.size());
            std::vector<IkReturnPtr> vtempikreturns; vtempikreturns.reserve(listlocalikreturns.size());
            FOREACH(itikreturn, listlocalikreturns) {
                dReal soldist = _ComputeGeometricConfigDistSqr(probot,(*itikreturn)->_vsolution,boost::get<1>(freeq0check));
                if( !(bestsolution.dist <= soldist) ) {
                    vdists.emplace_back(vdists.size(),  soldist);
                    vtempikreturns.push_back(*itikreturn);
                }
            }
            if( vdists.size() == 0 ) {
                return IKRA_Reject; // none could pass already computed solution
            }

            std::stable_sort(vdists.begin(),vdists.end(),SortSolutionDistances);
            listlocalikreturns.clear();
            for(size_t i = 0; i < vdists.size(); ++i) {
                std::vector<dReal>& vdata = vtempikreturns.at(vdists[i].first)->_mapdata["__distancetosol__"];
                vdata.resize(1);
                vdata[0] = vdists[i].second;
                listlocalikreturns.push_back(vtempikreturns.at(vdists[i].first));
            }
        }

        if( listlocalikreturns.size() == 0 ) {
            return IKRA_Reject;
        }

        // check ones with filter <= 0
        if( !(filteroptions & IKFO_IgnoreCustomFilters) && _HasFilterInRange(IKSP_MinPriority, 0) ) {
//            unsigned int maxsolutions = 1;
//            for(size_t i = 0; i < iksol.basesol.size(); ++i) {
//                unsigned char m = iksol.basesol[i].maxsolutions;
//                if( m != (unsigned char)-1 && m > 1) {
//                    maxsolutions *= m;
//                }
//            }
            // TODO: iterating vravesols would call the filters vravesols.size() times even if a valid solution is found
            // figure out a way to do short-curcuit the code to check the final solutions

            int nSameStateRepeatCount = 0;
            _nSameStateRepeatCount = 0;

            list<IkReturnPtr> listtestikreturns;
            listtestikreturns.swap(listlocalikreturns);
            FOREACH(ittestreturn, listtestikreturns) {//itravesol, vravesols) {
                IkReturnPtr localret = *ittestreturn;
                _vsolutionindices.resize(0);
                FOREACH(it, localret->_mapdata["solutionindices"]) {
                    _vsolutionindices.push_back((unsigned int)(*it+0.5)); // round
                }

                probot->SetActiveDOFValues(localret->_vsolution,false);
                _CheckRefineSolution(param, *pmanip, localret->_vsolution, !!(filteroptions&IKFO_IgnoreJointLimits));

                // due to floating-point precision, vravesol and param will not necessarily match anymore. The filters require perfectly matching pair, so compute a new param
                paramnew = pmanip->GetIkParameterization(param,false);
                paramnewglobal = pmanip->GetBase()->GetTransform() * paramnew;
                _nSameStateRepeatCount = nSameStateRepeatCount; // could be overwritten by _CallFilters call!

                bool bNeedCheckEndEffectorEnvCollision = stateCheck.NeedCheckEndEffectorEnvCollision();
                if( !(filteroptions & IKFO_IgnoreEndEffectorEnvCollisions) ) {
                    // have to make sure end effector collisions are set, regardless if stateCheck.ResetCheckEndEffectorEnvCollision has been called
                    stateCheck.RestoreCheckEndEffectorEnvCollision();
                }
                IkReturnAction retaction = _CallFilters(localret->_vsolution, pmanip, paramnew,localret, IKSP_MinPriority, 0);
                if( !(filteroptions & IKFO_IgnoreEndEffectorEnvCollisions) && !bNeedCheckEndEffectorEnvCollision ) {
                    stateCheck.ResetCheckEndEffectorEnvCollision();
                }
                nSameStateRepeatCount++;
                _nSameStateRepeatCount = nSameStateRepeatCount;
                retactionall |= retaction;
                if( retactionall & IKRA_Quit ) {
                    return static_cast<IkReturnAction>(retactionall|IKRA_RejectCustomFilter);
                }
                else if( retaction == IKRA_Success ) {
                    // success and the ikreturns are already ordered so that first one is with least distance.
                    bestsolution.ikreturn = localret;
                    dReal d = 1e30;
                    if( (int)boost::get<1>(freeq0check).size() == _nTotalDOF ) {
                        d = _ComputeGeometricConfigDistSqr(probot,localret->_vsolution,boost::get<1>(freeq0check));
                    }
                    bestsolution.dist = d;
                    listlocalikreturns.push_back(localret);
                    return IKRA_Success;
                }
            }
            if( listlocalikreturns.size() == 0 ) {
                return static_cast<IkReturnAction>(retactionall|IKRA_RejectCustomFilter);
            }
        }

        OPENRAVE_ASSERT_OP(listlocalikreturns.size(),>,0);
        bestsolution.ikreturn = listlocalikreturns.front();
        IkReturn::CustomData::iterator itdist = bestsolution.ikreturn->_mapdata.find("__distancetosol__");
        if( itdist != bestsolution.ikreturn->_mapdata.end() ) {
            bestsolution.dist = itdist->second.at(0);
        }
        else {
            bestsolution.dist = 1e30;
        }
        return IKRA_Success;
//        // solution is valid, so replace the best
//        size_t index = 0;
//        FOREACH(itikreturn, listlocalikreturns) {
//            if( (int)boost::get<1>(freeq0check).size() == _nTotalDOF ) {
//                d = _ComputeGeometricConfigDistSqr(probot,(*itikreturn)->_vsolution,boost::get<1>(freeq0check));
//                if( !(bestsolution.dist <= d) ) {
//                    bestsolution.ikreturn = *itikreturn;
//                    bestsolution.dist = d;
//                }
//            }
//            else {
//                // cannot compute distance, so quit once first solution is set to best
//                bestsolution.ikreturn = *itikreturn;
//                bestsolution.dist = d;
//                break;
//            }
//            ++index;
//        }
//        return IKRA_Success;
    }

    IkReturnAction _SolveAll(const IkParameterization& param, const vector<IkReal>& vfree, int filteroptions, std::vector<IkReturnPtr>& vikreturns, StateCheckEndEffector& stateCheck)
    {
        RobotBase::ManipulatorPtr pmanip(_pmanip);
        RobotBasePtr probot = pmanip->GetRobot();
        ikfast::IkSolutionList<IkReal> solutions;
        if( _CallIk(param,vfree, pmanip->GetLocalToolTransform(), solutions) ) {
            vector<IkReal> vsolfree;
            std::vector<IkReal> sol(pmanip->GetArmIndices().size());
            for(size_t isolution = 0; isolution < solutions.GetNumSolutions(); ++isolution) {
                const ikfast::IkSolution<IkReal>& iksol = dynamic_cast<const ikfast::IkSolution<IkReal>& >(solutions.GetSolution(isolution));
                iksol.Validate();
                //RAVELOG_VERBOSE_FORMAT("ikfast solution %d/%d (free=%d)", isolution%solutions.GetNumSolutions()%iksol.GetFree().size());
                if( iksol.GetFree().size() > 0 ) {
                    // have to search over all the free parameters of the solution!
                    vsolfree.resize(iksol.GetFree().size());
                    std::vector<dReal> vFreeInc(_GetFreeIncFromIndices(iksol.GetFree()));
                    IkReturnAction retaction = ComposeSolution(iksol.GetFree(), vsolfree, 0, vector<dReal>(), boost::bind(&IkFastSolver::_ValidateSolutionAll,shared_solver(), boost::ref(param), boost::ref(iksol), boost::ref(vsolfree), filteroptions, boost::ref(sol), boost::ref(vikreturns), boost::ref(stateCheck)), vFreeInc);
                    if( retaction & IKRA_Quit) {
                        return retaction;
                    }
                }
                else {
                    IkReturnAction retaction = _ValidateSolutionAll(param, iksol, vector<IkReal>(), filteroptions, sol, vikreturns, stateCheck);
                    if( retaction & IKRA_Quit ) {
                        return retaction;
                    }
                }
            }
        }
        return IKRA_Reject; // signals to continue
    }

    IkReturnAction _ValidateSolutionAll(const IkParameterization& param, const ikfast::IkSolution<IkReal>& iksol, const vector<IkReal>& vfree, int filteroptions, std::vector<IkReal>& sol, std::vector<IkReturnPtr>& vikreturns, StateCheckEndEffector& stateCheck)
    {
        iksol.GetSolution(sol,vfree);
        std::vector<dReal> vravesol(sol.size());
        std::copy(sol.begin(),sol.end(),vravesol.begin());

        int nSameStateRepeatCount = 0;
        _nSameStateRepeatCount = 0;
        std::vector< pair<std::vector<dReal>,int> > vravesols;
        list< std::pair<IkReturnPtr, IkParameterization> > listlocalikreturns; // orderd with respect to vravesols

        // find the first valid solutino that satisfies joint constraints and collisions
        if( !(filteroptions&IKFO_IgnoreJointLimits) ) {
            _ComputeAllSimilarJointAngles(vravesols, vravesol);
            if( vravesols.size() == 0 ) {
                return IKRA_RejectJointLimits;
            }
        }
        else {
            vravesols.emplace_back(vravesol, 0);
        }

        std::vector<unsigned int> vsolutionindices;
        iksol.GetSolutionIndices(vsolutionindices);

        // check for self collisions
        RobotBase::ManipulatorPtr pmanip(_pmanip);
        RobotBasePtr probot = pmanip->GetRobot();

//        if( IS_DEBUGLEVEL(Level_Verbose) ) {
//            stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
//            ss << "ikfast solution (free=" << iksol.GetFree().size() << "); iksol=[";
//            FOREACHC(it, vravesol) {
//                ss << *it << ", ";
//            }
//            ss << "]";
//            RAVELOG_VERBOSE(ss.str());
//        }

        IkParameterization paramnewglobal, paramnew;

        int retactionall = IKRA_Reject;
        if( !(filteroptions & IKFO_IgnoreCustomFilters) && _HasFilterInRange(1, IKSP_MaxPriority) ) {
//            unsigned int maxsolutions = 1;
//            for(size_t i = 0; i < iksol.basesol.size(); ++i) {
//                unsigned char m = iksol.basesol[i].maxsolutions;
//                if( m != (unsigned char)-1 && m > 1) {
//                    maxsolutions *= m;
//                }
//            }
            FOREACH(itravesol, vravesols) {
                _vsolutionindices = vsolutionindices;
                FOREACH(it,_vsolutionindices) {
                    *it += itravesol->second<<16;
                }
                probot->SetActiveDOFValues(itravesol->first,false);
                _CheckRefineSolution(param, *pmanip, itravesol->first, !!(filteroptions&IKFO_IgnoreJointLimits));

                // due to floating-point precision, vravesol and param will not necessarily match anymore. The filters require perfectly matching pair, so compute a new param
                paramnew = pmanip->GetIkParameterization(param,false);
                paramnewglobal = pmanip->GetBase()->GetTransform() * paramnew;
                _nSameStateRepeatCount = nSameStateRepeatCount; // could be overwritten by _CallFilters call!
                IkReturnPtr localret(new IkReturn(IKRA_Success));
                localret->_mapdata["solutionindices"] = std::vector<dReal>(_vsolutionindices.begin(),_vsolutionindices.end());

                bool bNeedCheckEndEffectorEnvCollision = stateCheck.NeedCheckEndEffectorEnvCollision();
                if( !(filteroptions & IKFO_IgnoreEndEffectorEnvCollisions) ) {
                    // have to make sure end effector collisions are set, regardless if stateCheck.ResetCheckEndEffectorEnvCollision has been called
                    stateCheck.RestoreCheckEndEffectorEnvCollision();
                }
                IkReturnAction retaction = _CallFilters(itravesol->first, pmanip, paramnew,localret, 1, IKSP_MaxPriority);
                if( !(filteroptions & IKFO_IgnoreEndEffectorEnvCollisions) && !bNeedCheckEndEffectorEnvCollision ) {
                    stateCheck.ResetCheckEndEffectorEnvCollision();
                }
                nSameStateRepeatCount++;
                _nSameStateRepeatCount = nSameStateRepeatCount;
                retactionall |= retaction;
                if( retactionall & IKRA_Quit ) {
                    return static_cast<IkReturnAction>(retactionall|IKRA_RejectCustomFilter);
                }
                else if( retaction == IKRA_Success ) {
                    localret->_vsolution.swap(itravesol->first);
                    listlocalikreturns.emplace_back(localret,  paramnew);
                }
            }
            if( listlocalikreturns.size() == 0 ) {
                return static_cast<IkReturnAction>(retactionall|IKRA_RejectCustomFilter);
            }
        }
        else {
            FOREACH(itravesol, vravesols) {
                _vsolutionindices = vsolutionindices;
                FOREACH(it,_vsolutionindices) {
                    *it += itravesol->second<<16;
                }
                probot->SetActiveDOFValues(itravesol->first,false);
                _CheckRefineSolution(param, *pmanip, itravesol->first, !!(filteroptions&IKFO_IgnoreJointLimits));

                // due to floating-point precision, vravesol and param will not necessarily match anymore. The filters require perfectly matching pair, so compute a new param
                paramnew = pmanip->GetIkParameterization(param,false);
                paramnewglobal = pmanip->GetBase()->GetTransform() * paramnew;
                IkReturnPtr localret(new IkReturn(IKRA_Success));
                localret->_mapdata["solutionindices"] = std::vector<dReal>(_vsolutionindices.begin(),_vsolutionindices.end());
                localret->_vsolution.swap(itravesol->first);
                listlocalikreturns.emplace_back(localret,  paramnew);
            }
        }

        CollisionReport report;
        CollisionReportPtr ptempreport;
        if( IS_DEBUGLEVEL(Level_Verbose) ) {
            ptempreport = boost::shared_ptr<CollisionReport>(&report,utils::null_deleter());
        }
        if( !(filteroptions&IKFO_IgnoreSelfCollisions) ) {
            stateCheck.SetSelfCollisionState();
            if( probot->CheckSelfCollision(ptempreport) ) {
                if( !!ptempreport ) {
                    if( !!ptempreport->plink1 && !!ptempreport->plink2 && (paramnewglobal.GetType() == IKP_Transform6D || paramnewglobal.GetDOF() >= pmanip->GetArmDOF()) ) {
                        // ik constraints the robot pretty well, so any self-collisions might mean the IK itself is impossible.
                        // check if self-colliding with non-moving part and a link that is pretty high in the chain, then perhaps we should give up...?
                        KinBody::LinkConstPtr potherlink;
                        if( find(_vindependentlinks.begin(), _vindependentlinks.end(), ptempreport->plink1) != _vindependentlinks.end() ) {
                            potherlink = ptempreport->plink2;
                        }
                        else if( find(_vindependentlinks.begin(), _vindependentlinks.end(), ptempreport->plink2) != _vindependentlinks.end() ) {
                            potherlink = ptempreport->plink1;
                        }

                        if( !!potherlink && _numBacktraceLinksForSelfCollisionWithNonMoving > 0 ) {
                            for(int itestdof = (int)pmanip->GetArmIndices().size()-_numBacktraceLinksForSelfCollisionWithNonMoving; itestdof < (int)pmanip->GetArmIndices().size(); ++itestdof) {
                                KinBody::JointPtr pjoint = probot->GetJointFromDOFIndex(pmanip->GetArmIndices()[itestdof]);
                                if( !!pjoint->GetHierarchyParentLink() && pjoint->GetHierarchyParentLink()->IsRigidlyAttached(*potherlink) ) {

                                    stateCheck.numImpossibleSelfCollisions++;
                                    RAVELOG_VERBOSE_FORMAT("self-collision with links %s and %s most likely means IK itself will not succeed, attempts=%d", ptempreport->plink1->GetName()%ptempreport->plink2->GetName()%stateCheck.numImpossibleSelfCollisions);
                                    if( stateCheck.numImpossibleSelfCollisions > 16 ) { // not sure what a good threshold is here
                                        RAVELOG_DEBUG_FORMAT("self-collision with links %s and %s most likely means IK itself will not succeed, giving up after %d attempts", ptempreport->plink1->GetName()%ptempreport->plink2->GetName()%stateCheck.numImpossibleSelfCollisions);
                                        return static_cast<IkReturnAction>(retactionall|IKRA_RejectSelfCollision|IKRA_Quit);
                                    }
                                    else {
                                        break;
                                    }
                                }
                            }
                        }

                        if( _vindependentlinks.size() != _vIndependentLinksIncludingFreeJoints.size() && _numBacktraceLinksForSelfCollisionWithFree > 0 ) {
                            // check
                            potherlink.reset();
                            if( find(_vIndependentLinksIncludingFreeJoints.begin(), _vIndependentLinksIncludingFreeJoints.end(), ptempreport->plink1) != _vIndependentLinksIncludingFreeJoints.end() ) {
                                potherlink = ptempreport->plink2;
                            }
                            else if( find(_vIndependentLinksIncludingFreeJoints.begin(), _vIndependentLinksIncludingFreeJoints.end(), ptempreport->plink2) != _vIndependentLinksIncludingFreeJoints.end() ) {
                                potherlink = ptempreport->plink1;
                            }

                            if( !!potherlink ) {
                                //bool bRigidlyAttached = false;
                                for(int itestdof = (int)pmanip->GetArmIndices().size()-_numBacktraceLinksForSelfCollisionWithFree; itestdof < (int)pmanip->GetArmIndices().size(); ++itestdof) {
                                    KinBody::JointPtr pjoint = probot->GetJointFromDOFIndex(pmanip->GetArmIndices()[itestdof]);
                                    if( !!pjoint->GetHierarchyParentLink() && pjoint->GetHierarchyParentLink()->IsRigidlyAttached(*potherlink) ) {

                                        stateCheck.numImpossibleSelfCollisions++;
                                        RAVELOG_VERBOSE_FORMAT("self-collision with links %s and %s most likely means IK itself will not succeed, attempts=%d", ptempreport->plink1->GetName()%ptempreport->plink2->GetName()%stateCheck.numImpossibleSelfCollisions);
                                        if( stateCheck.numImpossibleSelfCollisions > 16 ) { // not sure what a good threshold is here
                                            RAVELOG_DEBUG_FORMAT("self-collision with links %s and %s most likely means IK itself will not succeed for these free parameters, giving up after %d attempts", ptempreport->plink1->GetName()%ptempreport->plink2->GetName()%stateCheck.numImpossibleSelfCollisions);
                                            return static_cast<IkReturnAction>(retactionall|IKRA_RejectSelfCollision|IKRA_Quit);
                                        }
                                        else {
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                return static_cast<IkReturnAction>(retactionall|IKRA_RejectSelfCollision);
            }
        }
        if( (filteroptions&IKFO_CheckEnvCollisions) ) {
            stateCheck.SetEnvironmentCollisionState();
            if( stateCheck.NeedCheckEndEffectorEnvCollision() ) {
                // only check if the end-effector position is fully determined from the ik
                if( paramnewglobal.GetType() == IKP_Transform6D ) {// || (int)pmanip->GetArmIndices().size() <= paramnewglobal.GetDOF() ) {
                    if( pmanip->CheckEndEffectorCollision(pmanip->GetTransform(), ptempreport) ) {
                        if( !!ptempreport ) {
                            stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
                            ss << "env=" << GetEnv()->GetId() << ", ikfast collision " << ptempreport->__str__() << " ";
                            if( !!ptempreport->plink1 ) {
                                ss << "num1=" << ptempreport->plink1->GetCollisionData().vertices.size() << " ";
                            }
                            if( !!ptempreport->plink2 ) {
                                ss << "num2=" << ptempreport->plink2->GetCollisionData().vertices.size() << " ";
                            }
                            ss << "; colvalues=[";
                            std::vector<dReal> vallvalues;
                            probot->GetDOFValues(vallvalues);
                            for(size_t i = 0; i < vallvalues.size(); ++i ) {
                                if( i > 0 ) {
                                    ss << "," << vallvalues[i];
                                }
                                else {
                                    ss << vallvalues[i];
                                }
                            }
                            ss << "], manippose=[";
                            SerializeTransform(ss, pmanip->GetTransform());
                            ss << "]; localpose=[";
                            SerializeTransform(ss, pmanip->GetLocalToolTransform());
                            ss << "]";
                            RAVELOG_VERBOSE(ss.str());
                        }
                        if( paramnewglobal.GetType() == IKP_Transform6D ) {
                            // 6D so end effector is determined
                            return static_cast<IkReturnAction>(retactionall|IKRA_QuitEndEffectorCollision); // stop the search
                        }
                        else {
                            // end effector could change depending on the solution
                            return static_cast<IkReturnAction>(retactionall|IKRA_RejectEnvCollision); // stop the search
                        }
                    }
                    stateCheck.ResetCheckEndEffectorEnvCollision();
                }
            }
            if( GetEnv()->CheckCollision(KinBodyConstPtr(probot), ptempreport) ) {
                if( !!ptempreport ) {
                    stringstream ss; ss << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1);
                    ss << "ikfast collision " << report.__str__() << " colvalues=[";
                    std::vector<dReal> vallvalues;
                    probot->GetDOFValues(vallvalues);
                    for(size_t i = 0; i < vallvalues.size(); ++i ) {
                        if( i > 0 ) {
                            ss << "," << vallvalues[i];
                        }
                        else {
                            ss << vallvalues[i];
                        }
                    }
                    ss << "]";
                    RAVELOG_VERBOSE(ss.str());
                }
                return static_cast<IkReturnAction>(retactionall|IKRA_RejectEnvCollision);
            }
        }

        if( !(filteroptions & IKFO_IgnoreCustomFilters) && _HasFilterInRange(IKSP_MinPriority, 0) ) {
            int nSameStateRepeatCount = 0;
            _nSameStateRepeatCount = 0;

            list< std::pair<IkReturnPtr, IkParameterization> >::iterator ittestreturn = listlocalikreturns.begin();
            while(ittestreturn != listlocalikreturns.end()) {
                IkReturnPtr localret = ittestreturn->first;
                _vsolutionindices.resize(0);
                FOREACH(it, localret->_mapdata["solutionindices"]) {
                    _vsolutionindices.push_back((unsigned int)(*it+0.5)); // round
                }

                probot->SetActiveDOFValues(localret->_vsolution,false);
                _nSameStateRepeatCount = nSameStateRepeatCount; // could be overwritten by _CallFilters call!

                bool bNeedCheckEndEffectorEnvCollision = stateCheck.NeedCheckEndEffectorEnvCollision();
                if( !(filteroptions & IKFO_IgnoreEndEffectorEnvCollisions) ) {
                    // have to make sure end effector collisions are set, regardless if stateCheck.ResetCheckEndEffectorEnvCollision has been called
                    stateCheck.RestoreCheckEndEffectorEnvCollision();
                }
                IkReturnAction retaction = _CallFilters(localret->_vsolution, pmanip, ittestreturn->second, localret, IKSP_MinPriority, 0);
                if( !(filteroptions & IKFO_IgnoreEndEffectorEnvCollisions) && !bNeedCheckEndEffectorEnvCollision ) {
                    stateCheck.ResetCheckEndEffectorEnvCollision();
                }
                nSameStateRepeatCount++;
                _nSameStateRepeatCount = nSameStateRepeatCount;
                retactionall |= retaction;
                if( retactionall & IKRA_Quit ) {
                    return static_cast<IkReturnAction>(retactionall|IKRA_RejectCustomFilter);
                }
                else if( retaction == IKRA_Success ) {
                    ++ittestreturn;
                }
                else {
                    ittestreturn = listlocalikreturns.erase(ittestreturn);
                }
            }
        }

        // check that end effector moved in the correct direction
        dReal ikworkspacedist = param.ComputeDistanceSqr(paramnew);
        if( ikworkspacedist > _ikthreshold ) {
            stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
            ss << "ignoring bad ik for " << probot->GetName() << ":" << pmanip->GetName() << " dist=" << RaveSqrt(ikworkspacedist) << ", param=[" << param << "]";
            if( listlocalikreturns.size() > 0 ) {
                ss << ", sol=[";
                FOREACHC(itvalue,listlocalikreturns.front().first->_vsolution) {
                    ss << *itvalue << ", ";
                }
                ss << "]";
            }
            ss << endl;
            RAVELOG_ERROR(ss.str());
            return static_cast<IkReturnAction>(retactionall); // signals to continue
        }

        if( listlocalikreturns.size() > 0 ) {
            bool bNeedCheckEndEffectorEnvCollision = stateCheck.NeedCheckEndEffectorEnvCollision();
            if( !(filteroptions & IKFO_IgnoreEndEffectorEnvCollisions) ) {
                // have to make sure end effector collisions are set, regardless if stateCheck.ResetCheckEndEffectorEnvCollision has been called
                stateCheck.RestoreCheckEndEffectorEnvCollision();
            }
            FOREACH(itlocalikreturn, listlocalikreturns) {
                _CallFinishCallbacks(itlocalikreturn->first, pmanip, paramnewglobal);
            }
            if( !(filteroptions & IKFO_IgnoreEndEffectorEnvCollisions) && !bNeedCheckEndEffectorEnvCollision ) {
                stateCheck.ResetCheckEndEffectorEnvCollision();
            }
        }
        //RAVELOG_VERBOSE("add %d solutions", listlocalikreturns.size());
        FOREACH(itlocalikreturn, listlocalikreturns) {
            vikreturns.push_back(itlocalikreturn->first);
        }
        return static_cast<IkReturnAction>(retactionall); // signals to continue
    }

    bool _CheckJointAngles(std::vector<dReal>& vravesol) const
    {
        for(int j = 0; j < (int)_qlower.size(); ++j) {
            if( _vjointrevolute.at(j) == 2 ) {
                while( vravesol.at(j) > PI ) {
                    vravesol[j] -= 2*PI;
                }
                while( vravesol[j] < -PI ) {
                    vravesol[j] += 2*PI;
                }
            }
            else if( _vjointrevolute.at(j) == 1 ) {
                while( vravesol.at(j) > _qupper[j] ) {
                    vravesol[j] -= 2*PI;
                }
                while( vravesol[j] < _qlower[j] ) {
                    vravesol[j] += 2*PI;
                }
            }
            // due to error propagation, give error bounds for lower and upper limits
            if( vravesol[j] < _qlower[j]-g_fEpsilonJointLimit || vravesol[j] > _qupper[j]+g_fEpsilonJointLimit ) {
                return false;
            }
        }
        return true;
    }

    /// \brief configuraiton distance
    ///
    /// \param bNormalizeRevolute if true, then compute difference mod 2*PI
    dReal _ComputeGeometricConfigDistSqr(RobotBasePtr probot, const vector<dReal>& q1, const vector<dReal>& q2, bool bNormalizeRevolute=false) const
    {
        vector<dReal> q = q1;
        probot->SubtractActiveDOFValues(q,q2);
        vector<dReal>::iterator itq = q.begin();
        std::vector<uint8_t>::const_iterator itrevolute = _vjointrevolute.begin();
        dReal dist = 0;
        FOREACHC(it, probot->GetActiveDOFIndices()) {
            KinBody::JointPtr pjoint = probot->GetJointFromDOFIndex(*it);
            dReal fweight = pjoint->GetWeight(*it-pjoint->GetDOFIndex());
            // have to do the
            if( bNormalizeRevolute && *itrevolute ) {
                *itq = utils::NormalizeCircularAngle(*itq, -PI, PI);
            }
            dist += *itq**itq * fweight * fweight;
            ++itq;
            ++itrevolute;
        }
        return dist;
    }

    void _ComputeAllSimilarJointAngles(std::vector< std::pair<std::vector<dReal>, int> >& vravesols, std::vector<dReal>& vravesol)
    {
        vravesols.resize(0);
        if( !_CheckJointAngles(vravesol) ) {
            return;
        }
        vravesols.emplace_back(vravesol, 0);
        if( _qbigrangeindices.size() > 0 ) {
            std::vector< std::list<dReal> > vextravalues(_qbigrangeindices.size());
            std::vector< std::list<dReal> >::iterator itextra = vextravalues.begin();
            std::vector< size_t > vcumproduct; vcumproduct.reserve(_qbigrangeindices.size());
            size_t nTotal = 1, k = 0;
            FOREACH(itindex, _qbigrangeindices) {
                dReal foriginal = vravesol.at(*itindex);
                itextra->push_back(foriginal);
                dReal f = foriginal-2*PI;
                while(f >= _qlower[*itindex]) {
                    itextra->push_back(f);
                    f -= 2*PI;
                }
                f = foriginal+2*PI;
                while(f <= _qupper[*itindex]) {
                    itextra->push_back(f);
                    f += 2*PI;
                }
                vcumproduct.push_back(nTotal);
                OPENRAVE_ASSERT_OP_FORMAT(itextra->size(),<=,_qbigrangemaxsols.at(k),"exceeded max possible redundant solutions for manip arm index %d",_qbigrangeindices.at(k),ORE_InconsistentConstraints);
                nTotal *= itextra->size();
                ++itextra;
                ++k;
            }

            if( nTotal > 1 ) {
                vravesols.resize(nTotal);
                // copy the first point and save the cross product of all values in vextravalues
                for(size_t i = 1; i < vravesols.size(); ++i) {
                    vravesols[i].first.resize(vravesols[0].first.size());
                    int solutionindex = 0; // use to count which range has overflown on which joint index
                    for(size_t j = 0; j < vravesols[0].first.size(); ++j) {
                        vravesols[i].first[j] = vravesols[0].first[j];
                    }
                    for(size_t k = 0; k < _qbigrangeindices.size(); ++k) {
                        if( vextravalues[k].size() > 1 ) {
                            size_t repeat = vcumproduct.at(k);
                            int j = _qbigrangeindices[k];
                            size_t valueindex = (i/repeat)%vextravalues[k].size();
                            std::list<dReal>::const_iterator itvalue = vextravalues[k].begin();
                            advance(itvalue,valueindex);
                            vravesols[i].first[j] = *itvalue;
                            solutionindex += valueindex*_qbigrangemaxcumprod.at(k); // assumes each range
                        }
                    }
                    vravesols[i].second = solutionindex;
                }
            }
        }
    }

    void _SortSolutions(RobotBasePtr probot, std::vector<IkReturnPtr>& vikreturns)
    {
        // sort with respect to how far it is from limits
        vector< pair<size_t, dReal> > vdists; vdists.resize(vikreturns.size());
        vector<dReal> v;
        vector<dReal> viweights; viweights.reserve(probot->GetActiveDOF());
        FOREACHC(it, probot->GetActiveDOFIndices()) {
            KinBody::JointPtr pjoint = probot->GetJointFromDOFIndex(*it);
            viweights.push_back(1/pjoint->GetWeight(*it-pjoint->GetDOFIndex()));
        }

        for(size_t i = 0; i < vdists.size(); ++i) {
            v = vikreturns[i]->_vsolution;
            probot->SubtractActiveDOFValues(v,_qlower);
            dReal distlower = 1e30;
            for(size_t j = 0; j < v.size(); ++j) {
                distlower = min(distlower, RaveFabs(v[j])*viweights[j]);
            }
            v = vikreturns[i]->_vsolution;
            probot->SubtractActiveDOFValues(v,_qupper);
            dReal distupper = 1e30;
            for(size_t j = 0; j < v.size(); ++j) {
                distupper = min(distupper, RaveFabs(v[j])*viweights[j]);
            }
            vdists[i].first = i;
            vdists[i].second = -min(distupper,distlower);
        }

        std::stable_sort(vdists.begin(),vdists.end(),SortSolutionDistances);
        for(size_t i = 0; i < vdists.size(); ++i) {
            if( i != vdists[i].first )  {
                std::swap(vikreturns[i], vikreturns[vdists[i].first]);
                std::swap(vdists[i], vdists[vdists[i].first]);
            }
        }
    }

    /// \brief return incremental values for indices in the chain
    std::vector<dReal> _GetFreeIncFromIndices(const std::vector<int>& vindices)
    {
        std::vector<dReal> vFreeInc(vindices.size());
        for(size_t i = 0; i < vindices.size(); ++i) {
            if( _vjointrevolute.at(vindices[i]) ) {
                vFreeInc[i] = _fFreeIncRevolute;
            }
            else {
                // get the range of the axis and divide by 16..?
                vFreeInc[i] = (_qupper.at(i)-_qlower.at(i))/_fFreeIncPrismaticNum;
            }
        }
        return vFreeInc;
    }

    /// \brief convert ikparam to another type.
    /// \param param original ik param represented in manipulator's base link coordinate
    /// \param ikdummy ik param represented as _iktype in manipulator's base link coordinate
    /// \return constant reference to ikdummy
    inline const IkParameterization& _ConvertIkParameterization(const IkParameterization& param, IkParameterization& ikdummy)
    {
        if( param.GetType() == _iktype ) {
            return param;
        }

        // try to convert localgoal into a different goal suitable for the IK
        if( param.GetType() == IKP_Transform6D ) {
            if( _nTotalDOF-GetNumFreeParameters() == 4 ) {
                ikdummy = param; // copy the custom data!
                RobotBase::ManipulatorPtr pmanip(_pmanip);
                Vector vglobaldirection = param.GetTransform6D().rotate(pmanip->GetLocalToolDirection());
                if( _iktype == IKP_TranslationYAxisAngleXNorm4D ) {
                    ikdummy.SetTranslationYAxisAngleXNorm4D(param.GetTransform6D().trans,RaveAtan2(vglobaldirection.z,vglobaldirection.y));
                }
                else if( _iktype == IKP_TranslationZAxisAngleYNorm4D ) {
                    ikdummy.SetTranslationZAxisAngleYNorm4D(param.GetTransform6D().trans,RaveAtan2(vglobaldirection.x,vglobaldirection.z));
                }
                else if( _iktype == IKP_TranslationZAxisAngle4D ) {
                    ikdummy.SetTranslationZAxisAngle4D(param.GetTransform6D().trans,RaveAcos(vglobaldirection.z));
                }
                else if( _iktype == IKP_TranslationXAxisAngleZNorm4D ) {
                    ikdummy.SetTranslationXAxisAngleZNorm4D(param.GetTransform6D().trans,RaveAtan2(vglobaldirection.y,vglobaldirection.x));
                }
                else{
                    throw OPENRAVE_EXCEPTION_FORMAT(_("ik solver %s (dof=%d) does not support iktype 0x%x"), GetXMLId()%_nTotalDOF%_iktype, ORE_InvalidArguments);
                }
                return ikdummy;
            }
            else if( _nTotalDOF-GetNumFreeParameters() == 5 ) {
                ikdummy = param; // copy the custom data!
                RobotBase::ManipulatorPtr pmanip(_pmanip);
                ikdummy.SetTranslationDirection5D(RAY(param.GetTransform6D().trans, quatRotate(param.GetTransform6D().rot, pmanip->GetLocalToolDirection())));
                // have to copy the custom data!
                return ikdummy;
            }
        }
        throw OPENRAVE_EXCEPTION_FORMAT(_("ik solver %s (dof=%d) does not support iktype 0x%x"), GetXMLId()%_nTotalDOF%param.GetType(), ORE_InvalidArguments);
    }

    RobotBase::ManipulatorWeakPtr _pmanip;
    std::string _manipname; ///< name of the manipluator being set, this is for book keeping purposes
    std::vector<int> _vfreeparams; ///< the indices into _pmanip->GetArmIndices() for the free indices of this IK
    std::vector<uint8_t> _vfreerevolute, _vjointrevolute; // 0 if not revolute, 1 if revolute and not circular, 2 if circular
    std::vector<dReal> _vfreeparamscales;
    UserDataPtr _cblimits;
    std::vector<KinBody::LinkPtr> _vchildlinks; ///< the child links of the manipulator
    std::vector<KinBody::LinkPtr> _vindependentlinks; ///< independent links of the manipulator
    std::vector<KinBody::LinkPtr> _vIndependentLinksIncludingFreeJoints; ///< independent links of the ik chain without free joints
    std::vector<int> _vchildlinkindices; ///< indices of the links at _vchildlinks
    boost::shared_ptr<ikfast::IkFastFunctions<IkReal> > _ikfunctions;
    std::vector<dReal> _vFreeInc;
    dReal _fFreeIncRevolute; ///< default increment for revolute joints
    dReal _fFreeIncPrismaticNum; ///< default number of segments to divide a free slider axis
    int _nTotalDOF;
    std::vector<dReal> _qlower, _qupper, _qmid;
    std::vector<int> _qbigrangeindices; ///< indices into _qlower/_qupper of joints that are revolute, not circular, and have ranges > 360
    std::vector<size_t> _qbigrangemaxsols, _qbigrangemaxcumprod;
    IkParameterizationType _iktype;
    std::string _kinematicshash;
    int _numBacktraceLinksForSelfCollisionWithNonMoving, _numBacktraceLinksForSelfCollisionWithFree; ///< when pruning self collisions, the number of links to look at. If the tip of the manip self collides with the base, then can safely quit the IK. this is used purely for optimization purposes and by default it is mostly disabled. For more complex robots with a lot of joints, can use these parameters to speed up searching for IK.
    dReal _ikthreshold; ///< workspace distance threshold sanity checking between desired workspace goal and the workspace position with the returned ik values.
    dReal _fRefineWithJacobianInverseAllowedError; ///< if > 0, then use jacobian inverse numerical method to refine the results until workspace error drops down this much. By default it is disabled (=-1)

#ifdef OPENRAVE_HAS_LAPACK
    ikfastsolvers::JacobianInverseSolver<double> _jacobinvsolver; ///< jacobian inverse solver if _fRefineWithJacobianInverseAllowedError is > 0
#endif

    //@{
    // cache for current Solve call. This has to be saved/restored if any user functions are called (like filters) since the filters themselves can potentially call into this ik solver.
    std::vector<unsigned int> _vsolutionindices; ///< holds the indices of the current solution, this is not multi-thread safe
    int _nSameStateRepeatCount;
    //@}

    bool _bEmptyTransform6D; ///< if true, then the iksolver has been built with identity of the manipulator transform. Only valid for Transform6D IKs.

};

#ifdef OPENRAVE_IKFAST_FLOAT32
IkSolverBasePtr CreateIkFastSolver(EnvironmentBasePtr penv, std::istream& sinput, boost::shared_ptr<ikfast::IkFastFunctions<float> > ikfunctions, const vector<dReal>& vfreeinc, dReal ikthreshold)

{
    return IkSolverBasePtr(new IkFastSolver<float>(penv,sinput,ikfunctions,vfreeinc,ikthreshold));
}
#endif

IkSolverBasePtr CreateIkFastSolver(EnvironmentBasePtr penv, std::istream& sinput, boost::shared_ptr<ikfast::IkFastFunctions<double> > ikfunctions, const vector<dReal>& vfreeinc, dReal ikthreshold)
{
    return IkSolverBasePtr(new IkFastSolver<double>(penv,sinput,ikfunctions,vfreeinc,ikthreshold));
}
