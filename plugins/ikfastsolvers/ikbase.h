// -*- coding: utf-8 -*-
// Copyright (C) 2006-2010 Rosen Diankov (rdiankov@cs.cmu.edu)
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
#ifndef  IKFASTSOLVERBASE_H
#define  IKFASTSOLVERBASE_H

#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>

template <typename IKReal, typename Solution>
class IkFastSolver : public IkSolverBase
{
    enum SolutionResults
    {
        SR_Continue = 0, ///< go onto next set of parameters
        SR_Success = 1, ///< found solution
        SR_Quit = 2,  ///< failed due to collisions or other reasons that requires immediate failure
    };

 public:
    typedef bool (*IkFn)(const IKReal* eetrans, const IKReal* eerot, const IKReal* pfree, std::vector<Solution>& vsolutions);
    typedef bool (*FkFn)(const IKReal* j, IKReal* eetrans, IKReal* eerot);
    
 IkFastSolver(IkFn pfnik, const std::vector<int>& vfreeparams, const vector<dReal>& vFreeInc, int nTotalDOF, IkParameterization::Type iktype, boost::shared_ptr<void> resource, const std::string kinematicshash, EnvironmentBasePtr penv) : IkSolverBase(penv), _vfreeparams(vfreeparams), _pfnik(pfnik), _vFreeInc(vFreeInc), _nTotalDOF(nTotalDOF), _iktype(iktype), _resource(resource), _kinematicshash(kinematicshash) {
        __description = ":Interface Author: Rosen Diankov\n\nAn OpenRAVE wrapper for the ikfast generated files.\nIf 6D IK is used, will check if the end effector and other independent links are in collision before manipulator link collisions. If they are, the IK will terminate with failure immediately.\nBecause checking collisions is the slowest part of the IK, the custom filter function run before collision checking.";
    }
    virtual ~IkFastSolver() {}

    inline boost::shared_ptr<IkFastSolver<IKReal,Solution> > shared_solver() { return boost::static_pointer_cast<IkFastSolver<IKReal,Solution> >(shared_from_this()); }
    inline boost::shared_ptr<IkFastSolver<IKReal,Solution> const> shared_solver_const() const { return boost::static_pointer_cast<IkFastSolver<IKReal,Solution> const>(shared_from_this()); }
    inline boost::weak_ptr<IkFastSolver<IKReal,Solution> > weak_solver() { return shared_solver(); }

    virtual void SetCustomFilter(const IkFilterCallbackFn& filterfn) { _filterfn = filterfn; }

    virtual void SetJointLimits()
    {
        RobotBase::ManipulatorPtr pmanip(_pmanip);
        RobotBasePtr probot = pmanip->GetRobot();
        RobotBase::RobotStateSaver saver(probot);
        probot->SetActiveDOFs(pmanip->GetArmIndices());
        probot->GetActiveDOFLimits(_qlower,_qupper);
        _vfreeparamscales.resize(0);
        FOREACH(itfree, _vfreeparams) {
            if( *itfree < 0 || *itfree >= (int)_qlower.size() ) {
                throw openrave_exception(str(boost::format("free parameter idx %d out of bounds\n")%*itfree));
            }
            if( _qupper[*itfree] > _qlower[*itfree] ) {
                _vfreeparamscales.push_back(1.0f/(_qupper[*itfree]-_qlower[*itfree]));
            }
            else {
                _vfreeparamscales.push_back(0.0f);
            }
        }
    }

    virtual bool Init(RobotBase::ManipulatorPtr pmanip)
    {
        if( _kinematicshash.size() > 0 && pmanip->GetKinematicsStructureHash() != _kinematicshash ) {
            RAVELOG_ERROR(str(boost::format("inverse kinematics hashes do not match for manip %s:%s. IK will not work! %s!=%s\n")%pmanip->GetRobot()->GetName()%pmanip->GetName()%pmanip->GetKinematicsStructureHash()%_kinematicshash));
        }
        _pmanip = pmanip;
        RobotBasePtr probot = pmanip->GetRobot();
        _cblimits = probot->RegisterChangeCallback(KinBody::Prop_JointLimits,boost::bind(&IkFastSolver<IKReal,Solution>::SetJointLimits,boost::bind(&sptr_from<IkFastSolver<IKReal,Solution> >, weak_solver())));

        if( _nTotalDOF != (int)pmanip->GetArmIndices().size() ) {
            RAVELOG_ERROR(str(boost::format("ik %s configured with different number of joints than robot manipulator (%d!=%d)\n")%GetXMLId()%pmanip->GetArmIndices().size()%_nTotalDOF));
            return false;
        }

        _vfreerevolute.resize(0);
        FOREACH(itfree, _vfreeparams) {
            int index = pmanip->GetArmIndices().at(*itfree);
            KinBody::JointPtr pjoint = probot->GetJointFromDOFIndex(index);
            _vfreerevolute.push_back(pjoint->IsRevolute(index-pjoint->GetDOFIndex()));
        }
        
        _vjointrevolute.resize(0);
        FOREACHC(it,pmanip->GetArmIndices()) {
            KinBody::JointPtr pjoint = probot->GetJointFromDOFIndex(*it);
            _vjointrevolute.push_back(pjoint->IsRevolute(*it-pjoint->GetDOFIndex()));
        }

        if( _vFreeInc.size() != _vfreeparams.size() ) {
            _vFreeInc.resize(_vfreeparams.size());
            stringstream ss;
            ss << "robot " << probot->GetName() << ":" << pmanip->GetName() << " setting free increment to: ";
            for(size_t i = 0; i < _vFreeInc.size(); ++i) {
                if( _vfreerevolute[i] ) {
                    _vFreeInc[i] = 0.1;
                }
                else {
                    _vFreeInc[i] = 0.01;
                }
                ss << _vFreeInc[i] << " ";
            }
            RAVELOG_DEBUG(ss.str());
        }

        // get the joint limits
        RobotBase::RobotStateSaver saver(probot);
        probot->SetActiveDOFs(pmanip->GetArmIndices());
        SetJointLimits();
        return true;
    }

    virtual bool Supports(IkParameterization::Type iktype) const
    {
        return iktype == _iktype;
    }

    virtual bool Solve(const IkParameterization& param, const std::vector<dReal>& q0, int filteroptions, boost::shared_ptr< std::vector<dReal> > result)
    {
        if( param.GetType() != _iktype ) {
            RAVELOG_WARN(str(boost::format("ik solver only supports type 0x%x, given 0x%x")%_iktype%param.GetType()));
            return false;
        }

        RobotBase::ManipulatorPtr pmanip(_pmanip);
        if( (filteroptions&IKFO_CheckEnvCollisions) && _CheckIndependentCollision(pmanip) ) {
            return false;
        }
        
        RobotBasePtr probot = pmanip->GetRobot();
        RobotBase::RobotStateSaver saver(probot);
        probot->SetActiveDOFs(pmanip->GetArmIndices());
        std::vector<IKReal> vfree(_vfreeparams.size());
        bool bCheckEndEffector = true;
        return ComposeSolution(_vfreeparams, vfree, 0, q0, boost::bind(&IkFastSolver::_SolveSingle,shared_solver(), boost::ref(param),boost::ref(vfree),boost::ref(q0),filteroptions,result,boost::ref(bCheckEndEffector))) == SR_Success;
    }

    virtual bool Solve(const IkParameterization& param, int filteroptions, std::vector< std::vector<dReal> >& qSolutions)
    {
        qSolutions.resize(0);
        if( param.GetType() != _iktype ) {
            RAVELOG_WARN(str(boost::format("ik solver only supports type 0x%x, given 0x%x\n")%_iktype%param.GetType()));
            return false;
        }
        RobotBase::ManipulatorPtr pmanip(_pmanip);
        if( (filteroptions&IKFO_CheckEnvCollisions) && _CheckIndependentCollision(pmanip) ) {
            return false;
        }

        RobotBasePtr probot = pmanip->GetRobot();
        RobotBase::RobotStateSaver saver(probot);
        probot->SetActiveDOFs(pmanip->GetArmIndices());
        std::vector<IKReal> vfree(_vfreeparams.size());
        bool bCheckEndEffector = true;
        if( ComposeSolution(_vfreeparams, vfree, 0, vector<dReal>(), boost::bind(&IkFastSolver::_SolveAll,shared_solver(), param,boost::ref(vfree),filteroptions,boost::ref(qSolutions),boost::ref(bCheckEndEffector))) == SR_Quit ) {
            return false;
        }
        return qSolutions.size()>0;
    }

    virtual bool Solve(const IkParameterization& param, const std::vector<dReal>& q0, const std::vector<dReal>& vFreeParameters, int filteroptions, boost::shared_ptr< std::vector<dReal> > result)
    {
        if( param.GetType() != _iktype ) {
            RAVELOG_WARN(str(boost::format("ik solver only supports type 0x%x, given 0x%x\n")%_iktype%param.GetType()));
            return false;
        }
        if( vFreeParameters.size() != _vfreeparams.size() ) {
            throw openrave_exception("free parameters not equal",ORE_InvalidArguments);
        }
        RobotBase::ManipulatorPtr pmanip(_pmanip);
        if( (filteroptions&IKFO_CheckEnvCollisions) && _CheckIndependentCollision(pmanip) ) {
            return false;
        }
        RobotBasePtr probot = pmanip->GetRobot();
        RobotBase::RobotStateSaver saver(probot);
        probot->SetActiveDOFs(pmanip->GetArmIndices());
        std::vector<IKReal> vfree(_vfreeparams.size());
        for(size_t i = 0; i < _vfreeparams.size(); ++i) {
            vfree[i] = vFreeParameters[i]*(_qupper[_vfreeparams[i]]-_qlower[_vfreeparams[i]]) + _qlower[_vfreeparams[i]];
        }
        bool bCheckEndEffector = true;
        return _SolveSingle(param,vfree,q0,filteroptions,result,bCheckEndEffector)==SR_Success;
    }
    virtual bool Solve(const IkParameterization& param, const std::vector<dReal>& vFreeParameters, int filteroptions, std::vector< std::vector<dReal> >& qSolutions)
    {
        qSolutions.resize(0);
        if( param.GetType() != _iktype ) {
            RAVELOG_WARN(str(boost::format("ik solver only supports type 0x%x, given 0x%x")%_iktype%param.GetType()));
            return false;
        }
        if( vFreeParameters.size() != _vfreeparams.size() ) {
            throw openrave_exception("free parameters not equal",ORE_InvalidArguments);
        }
        RobotBase::ManipulatorPtr pmanip(_pmanip);
        if( (filteroptions&IKFO_CheckEnvCollisions) && _CheckIndependentCollision(pmanip) ) {
            return false;
        }

        RobotBasePtr probot = pmanip->GetRobot();
        RobotBase::RobotStateSaver saver(probot);
        probot->SetActiveDOFs(pmanip->GetArmIndices());
        std::vector<IKReal> vfree(_vfreeparams.size());
        for(size_t i = 0; i < _vfreeparams.size(); ++i) {
            vfree[i] = vFreeParameters[i]*(_qupper[_vfreeparams[i]]-_qlower[_vfreeparams[i]]) + _qlower[_vfreeparams[i]];
        }
        bool bCheckEndEffector = true;
        if( _SolveAll(param,vfree,filteroptions,qSolutions,bCheckEndEffector) == SR_Quit ) {
            return false;
        }
        return qSolutions.size()>0;
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

    virtual RobotBase::ManipulatorPtr GetManipulator() const { return RobotBase::ManipulatorPtr(_pmanip); }

private:
    SolutionResults ComposeSolution(const std::vector<int>& vfreeparams, vector<IKReal>& vfree, int freeindex, const vector<dReal>& q0, const boost::function<SolutionResults()>& fn)
    {
        if( freeindex >= (int)vfreeparams.size()) {
            return fn();
        }

        // start searching for phi close to q0, as soon as a solution is found for the curphi, return it
        dReal startphi = q0.size() == _qlower.size() ? q0.at(vfreeparams.at(freeindex)) : 0;
        dReal upperphi = _qupper.at(vfreeparams.at(freeindex)), lowerphi = _qlower.at(vfreeparams.at(freeindex)), deltaphi = 0;
        int iter = 0;
        dReal fFreeInc = _vFreeInc.at(freeindex);
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

                deltaphi += fFreeInc; // increment
            }

            iter++;

            vfree.at(freeindex) = curphi;
            SolutionResults res = ComposeSolution(vfreeparams, vfree, freeindex+1,q0, fn);
            if( res != SR_Continue ) {
                return res;
            }
        }

        // explicitly test 0 since many edge cases involve 0s
        if( _qlower[vfreeparams[freeindex]] <= 0 && _qupper[vfreeparams[freeindex]] >= 0 ) {
            vfree.at(freeindex) = 0;
            SolutionResults res = ComposeSolution(vfreeparams, vfree, freeindex+1,q0, fn);
            if( res != SR_Continue ) {
                return res;
            }
        }

        return SR_Continue;
    }

    bool _CallIK(const IkParameterization& param, const vector<IKReal>& vfree, std::vector<Solution>& vsolutions)
    {
        try {
            switch(param.GetType()) {
            case IkParameterization::Type_Transform6D: {
                TransformMatrix t = param.GetTransform6D();
                IKReal eetrans[3] = {t.trans.x, t.trans.y, t.trans.z};
                IKReal eerot[9] = {t.m[0],t.m[1],t.m[2],t.m[4],t.m[5],t.m[6],t.m[8],t.m[9],t.m[10]};
//                stringstream ss; ss << "./ik " << std::setprecision(16);
//                ss << eerot[0]  << " " << eerot[1]  << " " << eerot[2]  << " " << eetrans[0]  << " " << eerot[3]  << " " << eerot[4]  << " " << eerot[5]  << " " << eetrans[1]  << " " << eerot[6]  << " " << eerot[7]  << " " << eerot[8]  << " " << eetrans[2] << " ";
//                FOREACH(itfree,vfree) {
//                    ss << *itfree << " ";
//                }
//                ss << endl;
                //RAVELOG_INFO(ss.str());
                return _pfnik(eetrans, eerot, vfree.size()>0?&vfree[0]:NULL, vsolutions);
            }
            case IkParameterization::Type_Rotation3D: {
                TransformMatrix t(Transform(param.GetRotation3D(),Vector()));
                IKReal eerot[9] = {t.m[0],t.m[1],t.m[2],t.m[4],t.m[5],t.m[6],t.m[8],t.m[9],t.m[10]};
                return _pfnik(NULL, eerot, vfree.size()>0?&vfree[0]:NULL, vsolutions);
            }
            case IkParameterization::Type_Translation3D: {
                Vector v = param.GetTranslation3D();
                IKReal eetrans[3] = {v.x, v.y, v.z};
//                stringstream ss; ss << "./ik " << std::setprecision(16);
//                ss << eetrans[0]  << " " << eetrans[1]  << " " << eetrans[2] << " ";
//                FOREACH(itfree,vfree) {
//                    ss << *itfree << " ";
//                }
//                ss << endl;
//                RAVELOG_INFO(ss.str());
                return _pfnik(eetrans, NULL, vfree.size()>0?&vfree[0]:NULL, vsolutions);
            }
            case IkParameterization::Type_Direction3D: {
                Vector v = param.GetDirection3D();
                IKReal eerot[9] = {v.x, v.y, v.z,0,0,0,0,0,0};
                return _pfnik(NULL, eerot, vfree.size()>0?&vfree[0]:NULL, vsolutions);
            }
            case IkParameterization::Type_Ray4D: {
                RAY r = param.GetRay4D();
                IKReal eetrans[3] = {r.pos.x,r.pos.y,r.pos.z};
                IKReal eerot[9] = {r.dir.x, r.dir.y, r.dir.z,0,0,0,0,0,0};
                //RAVELOG_INFO("ray: %f %f %f %f %f %f\n",eerot[0],eerot[1],eerot[2],eetrans[0],eetrans[1],eetrans[2]);
                if( !_pfnik(eetrans, eerot, vfree.size()>0?&vfree[0]:NULL, vsolutions) ) {
                    return false;
                }
                return true;
            }
            case IkParameterization::Type_Lookat3D: {
                Vector v = param.GetLookat3D();
                IKReal eetrans[3] = {v.x, v.y, v.z};
                return _pfnik(eetrans, NULL, vfree.size()>0?&vfree[0]:NULL, vsolutions);
            }
            case IkParameterization::Type_TranslationDirection5D: {
                RAY r = param.GetTranslationDirection5D();
                IKReal eetrans[3] = {r.pos.x,r.pos.y,r.pos.z};
                IKReal eerot[9] = {r.dir.x, r.dir.y, r.dir.z,0,0,0,0,0,0};
                if( !_pfnik(eetrans, eerot, vfree.size()>0?&vfree[0]:NULL, vsolutions) ) {
                    return false;
                }
                return true;
            }
            case IkParameterization::Type_TranslationXY2D: {
                Vector v = param.GetTranslationXY2D();
                IKReal eetrans[3] = {v.x, v.y,0};
                return _pfnik(eetrans, NULL, vfree.size()>0?&vfree[0]:NULL, vsolutions);
            }
            case IkParameterization::Type_TranslationXYOrientation3D: {
                Vector v = param.GetTranslationXYOrientation3D();
                IKReal eetrans[3] = {v.x, v.y,v.z};
                return _pfnik(eetrans, NULL, vfree.size()>0?&vfree[0]:NULL, vsolutions);
            }
            case IkParameterization::Type_TranslationLocalGlobal6D: {
                std::pair<Vector,Vector> p = param.GetTranslationLocalGlobal6D();
                IKReal eetrans[3] = {p.second.x, p.second.y, p.second.z};
                IKReal eerot[9] = {p.first.x, 0, 0, 0, p.first.y, 0, 0, 0, p.first.z};
                return _pfnik(eetrans, eerot, vfree.size()>0?&vfree[0]:NULL, vsolutions);
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

        throw openrave_exception(str(boost::format("don't support ik parameterization 0x%x")%param.GetType()),ORE_InvalidArguments);
    }

    static bool SortSolutionDistances(const pair<int,dReal>& p1, const pair<int,dReal>& p2)
    {
        return p1.second < p2.second;
    }

    SolutionResults _SolveSingle(const IkParameterization& param, const vector<IKReal>& vfree, const vector<dReal>& q0, int filteroptions, boost::shared_ptr< std::vector<dReal> > result, bool& bCheckEndEffector)
    {
        std::vector<Solution> vsolutions;
        if( !_CallIK(param,vfree,vsolutions) ) {
            return SR_Continue;
        }

        RobotBase::ManipulatorPtr pmanip(_pmanip);
        RobotBasePtr probot = pmanip->GetRobot();
        dReal bestdist = 1e30;
        std::vector<dReal> vravesol(pmanip->GetArmIndices().size());
        std::vector<dReal> vbest;
        std::vector<IKReal> sol(pmanip->GetArmIndices().size()), vsolfree;
        // find the first valid solution that satisfies joint constraints and collisions
        boost::tuple<const vector<IKReal>&, const vector<dReal>&,int> textra(vsolfree, q0, filteroptions);

        vector<int> vsolutionorder(vsolutions.size());
        if( vravesol.size() == q0.size() ) {
            // sort the solutions from closest to farthest
            vector<pair<int,dReal> > vdists; vdists.reserve(vsolutions.size());
            FOREACH(itsol, vsolutions) {
                vsolfree.resize(itsol->GetFree().size());
                for(size_t ifree = 0; ifree < itsol->GetFree().size(); ++ifree) {
                    vsolfree[ifree] = q0.at(itsol->GetFree()[ifree]);
                }
                itsol->GetSolution(&sol[0],vsolfree.size()>0?&vsolfree[0]:NULL);
                for(int i = 0; i < (int)sol.size(); ++i) {
                    vravesol[i] = (dReal)sol[i];
                }
                vdists.push_back(make_pair((int)vdists.size(),_configdist2(probot,vravesol,q0)));
            }

            std::sort(vdists.begin(),vdists.end(),SortSolutionDistances);
            for(size_t i = 0; i < vsolutionorder.size(); ++i) {
                vsolutionorder[i] = vdists[i].first;
            }
        }
        else {
            for(size_t i = 0; i < vsolutionorder.size(); ++i) {
                vsolutionorder[i] = i;
            }
        }

        FOREACH(itindex,vsolutionorder) {
            Solution& iksol = vsolutions.at(*itindex);
            SolutionResults res;
            if( iksol.GetFree().size() > 0 ) {
                // have to search over all the free parameters of the solution!
                vsolfree.resize(iksol.GetFree().size());
                res = ComposeSolution(iksol.GetFree(), vsolfree, 0, q0, boost::bind(&IkFastSolver::_ValidateSolutionSingle,shared_solver(), boost::ref(iksol), boost::ref(textra), boost::ref(sol), boost::ref(vravesol), boost::ref(vbest), boost::ref(bestdist), boost::ref(param), boost::ref(bCheckEndEffector)));
            }
            else {
                vsolfree.resize(0);
                res = _ValidateSolutionSingle(iksol, textra, sol, vravesol, vbest, bestdist, param, bCheckEndEffector);
            }

            if( res == SR_Quit ) {
                return SR_Quit;
            }
            // stop if there is no solution we are attempting to get close to
            if( res == SR_Success && q0.size() != pmanip->GetArmIndices().size() ) {
                break;
            }
        }

        // return as soon as a solution is found, since we're visiting phis starting from q0, we are guaranteed
        // that the solution will be close (ie, phi's dominate in the search). This is to speed things up
        if( vbest.size() == pmanip->GetArmIndices().size() ) {
            if( !!result ) {
                *result = vbest;
            }
            return SR_Success;
        }

        return SR_Continue;
    }
    
    // validate a solution
    SolutionResults _ValidateSolutionSingle(const Solution& iksol, boost::tuple<const vector<IKReal>&, const vector<dReal>&,int>& freeq0check, std::vector<IKReal>& sol, std::vector<dReal>& vravesol, std::vector<dReal>& vbest, dReal& bestdist, const IkParameterization& param, bool& bCheckEndEffector)
    {
        const vector<IKReal>& vfree = boost::get<0>(freeq0check);
        //BOOST_ASSERT(sol.size()== iksol.basesol.size() && vfree.size() == iksol.GetFree().size());
        iksol.GetSolution(&sol[0],vfree.size()>0?&vfree[0]:NULL);
        for(int i = 0; i < (int)sol.size(); ++i) {
            vravesol[i] = dReal(sol[i]);
        }

        RobotBase::ManipulatorPtr pmanip(_pmanip);
        RobotBasePtr probot = pmanip->GetRobot();

        /// if have to check for closest solution, make sure this new solution is closer than best found so far
        dReal d = dReal(1e30);
        if( boost::get<1>(freeq0check).size() == vravesol.size() ) {
            d = _configdist2(probot,vravesol,boost::get<1>(freeq0check));
            if( bestdist <= d ) {
                return SR_Continue;
            }
        }

        int filteroptions = boost::get<2>(freeq0check);
        if( !(filteroptions&IKFO_IgnoreJointLimits) ) {
            if( !_checkjointangles(vravesol) ) {
                //            stringstream ss; ss << "bad joint angles: ";
                //            FOREACH(it,vravesol)
                //                ss << *it << " ";
                //            ss << endl;
                //            RAVELOG_INFO(ss.str().c_str());
                return SR_Continue;
            }
        }

        // check for self collisions
        probot->SetActiveDOFValues(vravesol);
        if( !!_filterfn ) {
            switch(_filterfn(vravesol, pmanip, param)) {
            case IKFR_Reject: return SR_Continue;
            case IKFR_Quit: return SR_Quit;
            case IKFR_Success:
                break;
            }
        }

        CollisionReport report;
        if( !(filteroptions&IKFO_IgnoreSelfCollisions) ) {
            if( IS_DEBUGLEVEL(Level_Verbose) ) {
                if( probot->CheckSelfCollision(boost::shared_ptr<CollisionReport>(&report,null_deleter())) ) {
                    return SR_Continue;
                }
            }
            else {
                if( probot->CheckSelfCollision() ) {
                    return SR_Continue;
                }
            }
        }
        if( (filteroptions&IKFO_CheckEnvCollisions) && GetEnv()->CheckCollision(KinBodyConstPtr(probot), boost::shared_ptr<CollisionReport>(&report,null_deleter())) ) {
            if( !!report.plink1 && !!report.plink2 ) {
                RAVELOG_VERBOSE(str(boost::format("IKFastSolver: collision %s:%s with %s:%s\n")%report.plink1->GetParent()->GetName()%report.plink1->GetName()%report.plink2->GetParent()->GetName()%report.plink2->GetName()));
            }
            else {
                RAVELOG_VERBOSE("ik collision, no link\n");
            }

            // if gripper is colliding, solutions will always fail, so completely stop solution process
            if( bCheckEndEffector && param.GetType() == IkParameterization::Type_Transform6D && pmanip->CheckEndEffectorCollision(pmanip->GetBase()->GetTransform()*param.GetTransform6D()) ) {
                return SR_Quit; // stop the search
            }
            
            bCheckEndEffector = false;
            return SR_Continue;
        }

        // solution is valid
        vbest = vravesol;
        bestdist = d;
        return SR_Success;
    }

    SolutionResults _SolveAll(const IkParameterization& param, const vector<IKReal>& vfree, int filteroptions, std::vector< std::vector<dReal> >& qSolutions, bool& bCheckEndEffector)
    {
        RobotBase::ManipulatorPtr pmanip(_pmanip);
        RobotBasePtr probot = pmanip->GetRobot();        
        std::vector<Solution> vsolutions;
        if( _CallIK(param,vfree,vsolutions) ) {
            vector<IKReal> vsolfree;
            vector<dReal> vravesol(pmanip->GetArmIndices().size());
            std::vector<IKReal> sol(pmanip->GetArmIndices().size());
            FOREACH(itsol, vsolutions) {
                if( itsol->GetFree().size() > 0 ) {
                    // have to search over all the free parameters of the solution!
                    vsolfree.resize(itsol->GetFree().size());                    
                    if( ComposeSolution(itsol->GetFree(), vsolfree, 0, vector<dReal>(), boost::bind(&IkFastSolver::_ValidateSolutionAll,shared_solver(), boost::ref(param), boost::ref(*itsol), boost::ref(vsolfree), filteroptions, boost::ref(sol), boost::ref(vravesol), boost::ref(qSolutions), boost::ref(bCheckEndEffector))) == SR_Quit) {
                        return SR_Quit;
                    }
                }
                else {
                    if( _ValidateSolutionAll(param, *itsol, vector<IKReal>(), filteroptions, sol, vravesol, qSolutions, bCheckEndEffector) == SR_Quit ) {
                        return SR_Quit;
                    }
                }
            }
        }
        return SR_Continue;
    }

    SolutionResults _ValidateSolutionAll(const IkParameterization& param, const Solution& iksol, const vector<IKReal>& vfree, int filteroptions, std::vector<IKReal>& sol, std::vector<dReal>& vravesol, std::vector< std::vector<dReal> >& qSolutions, bool& bCheckEndEffector)
    {
        iksol.GetSolution(&sol[0],vfree.size()>0?&vfree[0]:NULL);
        for(int i = 0; i < (int)sol.size(); ++i) {
            vravesol[i] = (dReal)sol[i];
        }
        // find the first valid solutino that satisfies joint constraints and collisions
        if( !(filteroptions&IKFO_IgnoreJointLimits) ) {
            if( !_checkjointangles(vravesol) ) {
                return SR_Continue;
            }
        }

        // check for self collisions
        RobotBase::ManipulatorPtr pmanip(_pmanip);
        RobotBasePtr probot = pmanip->GetRobot();
        probot->SetActiveDOFValues(vravesol);

        if( !!_filterfn ) {
            switch(_filterfn(vravesol, pmanip, param)) {
            case IKFR_Reject: return SR_Continue;
            case IKFR_Quit: return SR_Quit;
            case IKFR_Success:
                break;
            }
        }

        if( !(filteroptions&IKFO_IgnoreSelfCollisions) ) {
            if( IS_DEBUGLEVEL(Level_Verbose) ) {
                CollisionReport report;
                if( probot->CheckSelfCollision(boost::shared_ptr<CollisionReport>(&report,null_deleter())) ) {
                    return SR_Continue;
                }
            }
            else {
                if( probot->CheckSelfCollision() ) {
                    return SR_Continue;
                }
            }
        }
        if( (filteroptions&IKFO_CheckEnvCollisions) ) {
            if( bCheckEndEffector && param.GetType() == IkParameterization::Type_Transform6D ) {
                if( pmanip->CheckEndEffectorCollision(pmanip->GetBase()->GetTransform()*param.GetTransform6D()) ) {
                    return SR_Quit; // stop the search
                }
                bCheckEndEffector = false;
            }
            if( GetEnv()->CheckCollision(KinBodyConstPtr(probot)) ) {
                return SR_Continue;
            }
        }

        qSolutions.push_back(vravesol);
        return SR_Continue;
    }

    bool _checkjointangles(std::vector<dReal>& vravesol) const
    {
        for(int j = 0; j < (int)_qlower.size(); ++j) {
            if( _vjointrevolute.at(j) ) {
                if( _qlower[j] < -PI && vravesol[j] > _qupper[j] ) {
                    vravesol[j] -= 2*PI;
                }
                if( _qupper[j] > PI && vravesol[j] < _qlower[j] ) {
                    vravesol[j] += 2*PI;
                }
            }
            // due to error propagation, give error bounds for lower and upper limits
            if( vravesol[j] < _qlower[j]-10*g_fEpsilon || vravesol[j] > _qupper[j]+10*g_fEpsilon ) {
                return false;
            }
        }
        return true;
    }

    bool _CheckIndependentCollision(RobotBase::ManipulatorPtr pmanip)
    {
        CollisionReport report;
        if( pmanip->CheckIndependentCollision(boost::shared_ptr<CollisionReport>(&report,null_deleter())) ) {
            if( !!report.plink1 && !!report.plink2 ) {
                RAVELOG_VERBOSE(str(boost::format("IKFastSolver: indep collision %s:%s with %s:%s\n")%report.plink1->GetParent()->GetName()%report.plink1->GetName()%report.plink2->GetParent()->GetName()%report.plink2->GetName()));
            }

            return true;
        }

        return false;
    }

    dReal _configdist2(RobotBasePtr probot, const vector<dReal>& q1, const vector<dReal>& q2) const
    {
        vector<dReal> q = q1;
        probot->SubtractActiveDOFValues(q,q2);
        vector<dReal>::iterator itq = q.begin();
        dReal dist = 0;
        FOREACHC(it, probot->GetActiveDOFIndices()) {
            KinBody::JointPtr pjoint = probot->GetJointFromDOFIndex(*it);
            dist += *itq**itq * pjoint->GetWeight(*it-pjoint->GetDOFIndex());
        }
        return dist;
    }

    template <typename U> U SQR(U t) { return t*t; }

    RobotBase::ManipulatorWeakPtr _pmanip;
    std::vector<int> _vfreeparams;
    std::vector<uint8_t> _vfreerevolute, _vjointrevolute;
    std::vector<dReal> _vfreeparamscales;
    boost::shared_ptr<void> _cblimits;
    IkFn _pfnik;
    std::vector<dReal> _vFreeInc;
    int _nTotalDOF;
    std::vector<dReal> _qlower, _qupper;
    IkFilterCallbackFn _filterfn;
    IkParameterization::Type _iktype;
    boost::shared_ptr<void> _resource;
    std::string _kinematicshash;
};

#endif
