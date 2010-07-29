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
    
 IkFastSolver(IkFn pfnik, const std::vector<int>& vfreeparams, dReal fFreeInc, int nTotalDOF, IkParameterization::Type iktype, EnvironmentBasePtr penv) : IkSolverBase(penv), _vfreeparams(vfreeparams), _pfnik(pfnik), _fFreeInc(fFreeInc), _nTotalDOF(nTotalDOF), _iktype(iktype) {}
    virtual ~IkFastSolver() {}

    inline boost::shared_ptr<IkFastSolver<IKReal,Solution> > shared_solver() { return boost::static_pointer_cast<IkFastSolver<IKReal,Solution> >(shared_from_this()); }
    inline boost::shared_ptr<IkFastSolver<IKReal,Solution> const> shared_solver_const() const { return boost::static_pointer_cast<IkFastSolver<IKReal,Solution> const>(shared_from_this()); }
    inline boost::weak_ptr<IkFastSolver<IKReal,Solution> > weak_solver() { return shared_solver(); }

    virtual void SetFilter(const IkFilterCallbackFn& filterfn) { _filterfn = filterfn; }

    virtual void SetJointLimits()
    {
        RobotBase::ManipulatorPtr pmanip(_pmanip);
        RobotBasePtr probot = pmanip->GetRobot();
        RobotBase::RobotStateSaver saver(probot);
        probot->SetActiveDOFs(pmanip->GetArmIndices());
        probot->GetActiveDOFLimits(_qlower,_qupper);
        _vfreeparamscales.resize(0);
        FOREACH(itfree, _vfreeparams) {
            if( *itfree < 0 || *itfree >= (int)_qlower.size() )
                throw openrave_exception(str(boost::format("free parameter idx %d out of bounds\n")%*itfree));

            if( _qupper[*itfree] > _qlower[*itfree] )
                _vfreeparamscales.push_back(1.0f/(_qupper[*itfree]-_qlower[*itfree]));
            else
                _vfreeparamscales.push_back(0.0f);
        }
    }

    virtual bool Init(RobotBase::ManipulatorPtr pmanip)
    {
        _pmanip = pmanip;
        RobotBasePtr probot = pmanip->GetRobot();
        _cblimits = probot->RegisterChangeCallback(KinBody::Prop_JointLimits,boost::bind(&IkFastSolver<IKReal,Solution>::SetJointLimits,boost::bind(&sptr_from<IkFastSolver<IKReal,Solution> >, weak_solver())));

        if( _nTotalDOF != (int)pmanip->GetArmIndices().size() ) {
            RAVELOG_ERRORA(str(boost::format("ik %s configured with different number of joints than robot manipulator (%d!=%d)\n")%GetXMLId()%pmanip->GetArmIndices().size()%_nTotalDOF));
            return false;
        }

        _vfreetypes.resize(0);
        FOREACH(itfree, _vfreeparams) {
            _vfreetypes.push_back(probot->GetJoints().at(pmanip->GetArmIndices().at(*itfree))->GetType());
        }

        _vjointtypes.resize(0);
        FOREACHC(it,pmanip->GetArmIndices()) {
            _vjointtypes.push_back(probot->GetJoints().at(*it)->GetType());
        }

        // get the joint limits
        RobotBase::RobotStateSaver saver(probot);
        probot->SetActiveDOFs(pmanip->GetArmIndices());
        SetJointLimits();
        return true;
    }

    virtual bool Solve(const IkParameterization& param, const std::vector<dReal>& q0, int filteroptions, boost::shared_ptr< std::vector<dReal> > result)
    {
        if( param.GetType() != _iktype ) {
            RAVELOG_WARNA(str(boost::format("ik solver only supports type %d, given %d")%_iktype%param.GetType()));
            return false;
        }

        RobotBase::ManipulatorPtr pmanip(_pmanip);
        if( (filteroptions&IKFO_CheckEnvCollisions) && _CheckIndependentCollision(pmanip) )
            return false;
        
        RobotBasePtr probot = pmanip->GetRobot();
        RobotBase::RobotStateSaver saver(probot);
        probot->SetActiveDOFs(pmanip->GetArmIndices());
        std::vector<IKReal> vfree(_vfreeparams.size());
        bool bCheckEndEffector = true;
        return ComposeSolution(_vfreeparams, vfree, 0, q0, boost::bind(&IkFastSolver::_SolveSingle,shared_solver(), boost::ref(param),boost::ref(vfree),boost::ref(q0),filteroptions,result,boost::ref(bCheckEndEffector))) == SR_Success;
    }

    virtual bool Solve(const IkParameterization& param, int filteroptions, std::vector< std::vector<dReal> >& qSolutions)
    {
        if( param.GetType() != _iktype ) {
            RAVELOG_WARNA(str(boost::format("ik solver only supports type %d, given %d")%_iktype%param.GetType()));
            return false;
        }

        RobotBase::ManipulatorPtr pmanip(_pmanip);
        if( (filteroptions&IKFO_CheckEnvCollisions) && _CheckIndependentCollision(pmanip) )
            return false;

        RobotBasePtr probot = pmanip->GetRobot();
        RobotBase::RobotStateSaver saver(probot);
        probot->SetActiveDOFs(pmanip->GetArmIndices());
        std::vector<IKReal> vfree(_vfreeparams.size());
        qSolutions.resize(0);
        bool bCheckEndEffector = true;
        if( ComposeSolution(_vfreeparams, vfree, 0, vector<dReal>(), boost::bind(&IkFastSolver::_SolveAll,shared_solver(), param,boost::ref(vfree),filteroptions,boost::ref(qSolutions),boost::ref(bCheckEndEffector))) == SR_Quit )
            return false;
        return qSolutions.size()>0;
    }

    virtual bool Solve(const IkParameterization& param, const std::vector<dReal>& q0, const std::vector<dReal>& vFreeParameters, int filteroptions, boost::shared_ptr< std::vector<dReal> > result)
    {
        if( param.GetType() != _iktype ) {
            RAVELOG_WARNA(str(boost::format("ik solver only supports type %d, given %d")%_iktype%param.GetType()));
            return false;
        }

        if( vFreeParameters.size() != _vfreeparams.size() )
            throw openrave_exception("free parameters not equal",ORE_InvalidArguments);

        RobotBase::ManipulatorPtr pmanip(_pmanip);
        if( (filteroptions&IKFO_CheckEnvCollisions) && _CheckIndependentCollision(pmanip) )
            return false;

        RobotBasePtr probot = pmanip->GetRobot();
        RobotBase::RobotStateSaver saver(probot);
        probot->SetActiveDOFs(pmanip->GetArmIndices());
        std::vector<IKReal> vfree(_vfreeparams.size());
        for(size_t i = 0; i < _vfreeparams.size(); ++i)
            vfree[i] = vFreeParameters[i]*(_qupper[_vfreeparams[i]]-_qlower[_vfreeparams[i]]) + _qlower[_vfreeparams[i]];
        bool bCheckEndEffector = true;
        return _SolveSingle(param,vfree,q0,filteroptions,result,bCheckEndEffector)==SR_Success;
    }
    virtual bool Solve(const IkParameterization& param, const std::vector<dReal>& vFreeParameters, int filteroptions, std::vector< std::vector<dReal> >& qSolutions)
    {
        if( param.GetType() != _iktype ) {
            RAVELOG_WARNA(str(boost::format("ik solver only supports type %d, given %d")%_iktype%param.GetType()));
            return false;
        }

        if( vFreeParameters.size() != _vfreeparams.size() )
            throw openrave_exception("free parameters not equal",ORE_InvalidArguments);

        RobotBase::ManipulatorPtr pmanip(_pmanip);
        if( (filteroptions&IKFO_CheckEnvCollisions) && _CheckIndependentCollision(pmanip) )
            return false;

        RobotBasePtr probot = pmanip->GetRobot();
        RobotBase::RobotStateSaver saver(probot);
        probot->SetActiveDOFs(pmanip->GetArmIndices());
        std::vector<IKReal> vfree(_vfreeparams.size());
        for(size_t i = 0; i < _vfreeparams.size(); ++i)
            vfree[i] = vFreeParameters[i]*(_qupper[_vfreeparams[i]]-_qlower[_vfreeparams[i]]) + _qlower[_vfreeparams[i]];
        qSolutions.resize(0);
        bool bCheckEndEffector = true;
        if( _SolveAll(param,vfree,filteroptions,qSolutions,bCheckEndEffector) == SR_Quit )
            return false;
        return qSolutions.size()>0;
    }

    virtual int GetNumFreeParameters() const { return (int)_vfreeparams.size(); }
    virtual bool GetFreeParameters(std::vector<dReal>& pFreeParameters) const
    {
        RobotBase::ManipulatorPtr pmanip(_pmanip);
        RobotBasePtr probot = pmanip->GetRobot();
        std::vector<dReal> values;
        std::vector<dReal>::const_iterator itscale = _vfreeparamscales.begin();
        probot->GetDOFValues(values);
        pFreeParameters.resize(_vfreeparams.size());
        for(size_t i = 0; i < _vfreeparams.size(); ++i)
            pFreeParameters[i] = (values[pmanip->GetArmIndices()[_vfreeparams[i]]]-_qlower[_vfreeparams[i]]) * *itscale++;

        return true;
    }

    virtual RobotBase::ManipulatorPtr GetManipulator() const { return RobotBase::ManipulatorPtr(_pmanip); }

private:
    SolutionResults ComposeSolution(const std::vector<int>& vfreeparams, vector<IKReal>& vfree, int freeindex, const vector<dReal>& q0, const boost::function<SolutionResults()>& fn)
    {
        if( freeindex >= (int)vfreeparams.size())
            return fn();

        // start searching for phi close to q0, as soon as a solution is found for the curphi, return it
        dReal startphi = q0.size() == _qlower.size() ? q0.at(vfreeparams.at(freeindex)) : 0;
        dReal upperphi = _qupper.at(vfreeparams.at(freeindex)), lowerphi = _qlower.at(vfreeparams.at(freeindex)), deltaphi = 0;
        int iter = 0;
        dReal fFreeInc = _fFreeInc;
                // if joint is a slider, make increments 5 times less (this makes it possible to have free joints that are both revolume and prismatic)
        // (actually this should be fixed so that there is a different increment per free joint). can use max radius
        if( &vfreeparams == &_vfreeparams && _vfreetypes.at(freeindex) == KinBody::Joint::JointPrismatic )
            fFreeInc *= 0.2f;
        while(1) {

            dReal curphi = startphi;
            if( iter & 1 ) { // increment
                curphi += deltaphi;
                if( curphi > upperphi ) {

                    if( startphi-deltaphi < lowerphi)
                        break; // reached limit
                    ++iter;
                    continue;
                }
            }
            else { // decrement
                curphi -= deltaphi;
                if( curphi < lowerphi ) {

                    if( startphi+deltaphi > upperphi )
                        break; // reached limit
                    deltaphi += fFreeInc; // increment
                    ++iter;
                    continue;
                }

                deltaphi += fFreeInc; // increment
            }

            iter++;

            vfree.at(freeindex) = curphi;
            SolutionResults res = ComposeSolution(vfreeparams, vfree, freeindex+1,q0, fn);
            if( res != SR_Continue )
                return res;
        }

        // explicitly test 0 since many edge cases involve 0s
        if( _qlower[vfreeparams[freeindex]] <= 0 && _qupper[vfreeparams[freeindex]] >= 0 ) {
            vfree.at(freeindex) = 0;
            SolutionResults res = ComposeSolution(vfreeparams, vfree, freeindex+1,q0, fn);
            if( res != SR_Continue )
                return res;
        }

        return SR_Continue;
    }

    bool _CallIK(const IkParameterization& param, const vector<IKReal>& vfree, std::vector<Solution>& vsolutions)
    {
        if( param.GetType() == IkParameterization::Type_Transform6D ) {
            TransformMatrix t = param.GetTransform();
            IKReal eetrans[3] = {t.trans.x, t.trans.y, t.trans.z};
            IKReal eerot[9] = {t.m[0],t.m[1],t.m[2],t.m[4],t.m[5],t.m[6],t.m[8],t.m[9],t.m[10]};
//            stringstream ss; ss << "./ik ";
//            ss << eerot[0]  << " " << eerot[1]  << " " << eerot[2]  << " " << eetrans[0]  << " " << eerot[3]  << " " << eerot[4]  << " " << eerot[5]  << " " << eetrans[1]  << " " << eerot[6]  << " " << eerot[7]  << " " << eerot[8]  << " " << eetrans[2] << " ";
//            FOREACH(itfree,vfree) {
//                ss << *itfree << " ";
//            }
//            ss << endl;
//            RAVELOG_INFO(ss.str());
            return _pfnik(eetrans, eerot, vfree.size()>0?&vfree[0]:NULL, vsolutions);
        }
        else if( param.GetType() == IkParameterization::Type_Rotation3D ) {
            TransformMatrix t(Transform(param.GetRotation(),Vector()));
            IKReal eerot[9] = {t.m[0],t.m[1],t.m[2],t.m[4],t.m[5],t.m[6],t.m[8],t.m[9],t.m[10]};
            return _pfnik(NULL, eerot, vfree.size()>0?&vfree[0]:NULL, vsolutions);
        }
        else if( param.GetType() == IkParameterization::Type_Translation3D ) {
            Vector v = param.GetTranslation();
            IKReal eetrans[3] = {v.x, v.y, v.z};
            return _pfnik(eetrans, NULL, vfree.size()>0?&vfree[0]:NULL, vsolutions);
        }
        else if( param.GetType() == IkParameterization::Type_Direction3D ) {
            Vector v = param.GetDirection();
            IKReal eerot[9] = {v.x, v.y, v.z,0,0,0,0,0,0};
            return _pfnik(NULL, eerot, vfree.size()>0?&vfree[0]:NULL, vsolutions);
        }
        else if( param.GetType() == IkParameterization::Type_Ray4D ) {
            Vector pos = param.GetRay().pos;
            Vector dir = param.GetRay().dir;
            IKReal eetrans[3] = {pos.x,pos.y,pos.z};
            IKReal eerot[9] = {dir.x, dir.y, dir.z,0,0,0,0,0,0};
            //RAVELOG_INFO("ray: %f %f %f %f %f %f\n",eerot[0],eerot[1],eerot[2],eetrans[0],eetrans[1],eetrans[2]);
            if( !_pfnik(eetrans, eerot, vfree.size()>0?&vfree[0]:NULL, vsolutions) ) {
                return false;
            }
            //?
            FOREACH(itsol,vsolutions) {
                FOREACH(it,itsol->basesol) {
                    it->freeind=-1;
                }
                itsol->vfree.resize(0);
            }
            return true;
//            IKReal s[4];
//            IKReal free[10];
//            vsolutions[0].GetSolution(s,free);
//            RAVELOG_INFO("sol %d: %f %f %f %f\n",(int)vsolutions[0].GetFree().size(),s[0],s[1],s[2],s[3]);
        }

        throw openrave_exception(str(boost::format("don't support ik parameterization %d")%param.GetType()));
    }

    SolutionResults _SolveSingle(const IkParameterization& param, const vector<IKReal>& vfree, const vector<dReal>& q0, int filteroptions, boost::shared_ptr< std::vector<dReal> > result, bool& bCheckEndEffector)
    {
        std::vector<Solution> vsolutions;
        if( !_CallIK(param,vfree,vsolutions) ) {
            return SR_Continue;
        }

        vector<IKReal> vsolfree;

        RobotBase::ManipulatorPtr pmanip(_pmanip);
        dReal bestdist = 1e30;
        std::vector<dReal> vravesol(pmanip->GetArmIndices().size());
        std::vector<dReal> vbest;
        std::vector<IKReal> sol(pmanip->GetArmIndices().size());
        // find the first valid solution that satisfies joint constraints and collisions
        boost::tuple<const vector<IKReal>&, const vector<dReal>&,int> textra(vsolfree, q0, filteroptions);

        FOREACH(itsol, vsolutions) {
            SolutionResults res;
            if( itsol->GetFree().size() > 0 ) {
                // have to search over all the free parameters of the solution!
                vsolfree.resize(itsol->GetFree().size());
                res = ComposeSolution(itsol->GetFree(), vsolfree, 0, q0, boost::bind(&IkFastSolver::_ValidateSolutionSingle,shared_solver(), boost::ref(*itsol), boost::ref(textra), boost::ref(sol), boost::ref(vravesol), boost::ref(vbest), boost::ref(bestdist), boost::ref(param), boost::ref(bCheckEndEffector)));
            }
            else {
                vsolfree.resize(0);
                res = _ValidateSolutionSingle(*itsol, textra, sol, vravesol, vbest, bestdist, param, bCheckEndEffector);
            }

            if( res == SR_Quit )
                return SR_Quit;

            // stop if there is no solution we are attempting to get close to
            if( res == SR_Success && q0.size() != pmanip->GetArmIndices().size() )
                break;
        }

        // return as soon as a solution is found, since we're visiting phis starting from q0, we are guaranteed
        // that the solution will be close (ie, phi's dominate in the search). This is to speed things up
        if( vbest.size() == pmanip->GetArmIndices().size() ) {
            if( !!result )
                *result = vbest;
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

        /// if have to check for closest solution, make sure this new solution is closer than best found so far
        dReal d = dReal(1e30);
        if( boost::get<1>(freeq0check).size() == vravesol.size() ) {
            dReal d = 0;
            for(size_t k = 0; k < vravesol.size(); ++k) {
                d += SQR(vravesol[k]-boost::get<1>(freeq0check)[k]);
            }
            if( bestdist <= d ) {
                return SR_Continue;
            }
        }

        int filteroptions = boost::get<2>(freeq0check);
        if( !(filteroptions&IKFO_IgnoreJointLimits) ) {
            if( !checkjointangles(vravesol) ) {
                //            stringstream ss; ss << "bad joint angles: ";
                //            FOREACH(it,vravesol)
                //                ss << *it << " ";
                //            ss << endl;
                //            RAVELOG_INFO(ss.str().c_str());
                return SR_Continue;
            }
        }

        // check for self collisions
        RobotBase::ManipulatorPtr pmanip(_pmanip);
        RobotBasePtr probot = pmanip->GetRobot();
        probot->SetActiveDOFValues(vravesol);
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
                RAVELOG_VERBOSEA(str(boost::format("IKFastSolver: collision %s:%s with %s:%s\n")%report.plink1->GetParent()->GetName()%report.plink1->GetName()%report.plink2->GetParent()->GetName()%report.plink2->GetName()));
            }
            else
                RAVELOG_VERBOSEA("ik collision, no link\n");

            // if gripper is colliding, solutions will always fail, so completely stop solution process
            if( bCheckEndEffector && param.GetType() == IkParameterization::Type_Transform6D && pmanip->CheckEndEffectorCollision(pmanip->GetBase()->GetTransform()*param.GetTransform()*pmanip->GetGraspTransform()) ) {
                return SR_Quit; // stop the search
            }
            
            bCheckEndEffector = false;
            return SR_Continue;
        }
        if( !!_filterfn ) {
            switch(_filterfn(vravesol, pmanip, param)) {
            case IKFR_Reject: return SR_Continue;
            case IKFR_Quit: return SR_Quit;
            case IKFR_Success:
                break;
            }
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
                    
                    if( ComposeSolution(itsol->GetFree(), vsolfree, 0, vector<dReal>(), boost::bind(&IkFastSolver::_ValidateSolutionAll,shared_solver(), boost::ref(param), boost::ref(*itsol), boost::ref(vsolfree), filteroptions, boost::ref(sol), boost::ref(vravesol), boost::ref(qSolutions), boost::ref(bCheckEndEffector))) == SR_Quit)
                        return SR_Quit;
                }
                else {
                    if( _ValidateSolutionAll(param, *itsol, vector<IKReal>(), filteroptions, sol, vravesol, qSolutions, bCheckEndEffector) == SR_Quit )
                        return SR_Quit;
                }
            }
        }
        return SR_Continue;
    }

    SolutionResults _ValidateSolutionAll(const IkParameterization& param, const Solution& iksol, const vector<IKReal>& vfree, int filteroptions, std::vector<IKReal>& sol, std::vector<dReal>& vravesol, std::vector< std::vector<dReal> >& qSolutions, bool& bCheckEndEffector)
    {
        iksol.GetSolution(&sol[0],vfree.size()>0?&vfree[0]:NULL);

        for(int i = 0; i < (int)sol.size(); ++i)
            vravesol[i] = (dReal)sol[i];
            
        // find the first valid solutino that satisfies joint constraints and collisions
        if( !(filteroptions&IKFO_IgnoreJointLimits) ) {
            if( !checkjointangles(vravesol) )
                return SR_Continue;
        }

        // check for self collisions
        RobotBase::ManipulatorPtr pmanip(_pmanip);
        RobotBasePtr probot = pmanip->GetRobot();
        probot->SetActiveDOFValues(vravesol);
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
                if( pmanip->CheckEndEffectorCollision(pmanip->GetBase()->GetTransform()*param.GetTransform()*pmanip->GetGraspTransform()) )
                    return SR_Quit; // stop the search
                bCheckEndEffector = false;
            }
            if( GetEnv()->CheckCollision(KinBodyConstPtr(probot)) ) {
                return SR_Continue;
            }
        }
        if( !!_filterfn ) {
            switch(_filterfn(vravesol, pmanip, param)) {
            case IKFR_Reject: return SR_Continue;
            case IKFR_Quit: return SR_Quit;
            case IKFR_Success:
                break;
            }
        }
        qSolutions.push_back(vravesol);
        return SR_Continue;
    }

    bool checkjointangles(std::vector<dReal>& vravesol)
    {
        for(int j = 0; j < (int)_qlower.size(); ++j) {
            if( _vjointtypes.at(j) != KinBody::Joint::JointPrismatic ) {
                if( _qlower[j] < -PI && vravesol[j] > _qupper[j] )
                    vravesol[j] -= 2*PI;
                if( _qupper[j] > PI && vravesol[j] < _qlower[j] )
                    vravesol[j] += 2*PI;
            }
            if( vravesol[j] < _qlower[j]-1e-5 || vravesol[j] > _qupper[j]+1e-5 ) {
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
                RAVELOG_VERBOSEA(str(boost::format("IKFastSolver: indep collision %s:%s with %s:%s\n")%report.plink1->GetParent()->GetName()%report.plink1->GetName()%report.plink2->GetParent()->GetName()%report.plink2->GetName()));
            }

            return true;
        }

        return false;
    }

    template <typename U> U SQR(U t) { return t*t; }

    RobotBase::ManipulatorWeakPtr _pmanip;
    std::vector<int> _vfreeparams;
    std::vector<KinBody::Joint::JointType> _vfreetypes, _vjointtypes;
    std::vector<dReal> _vfreeparamscales;
    boost::shared_ptr<void> _cblimits;
    IkFn _pfnik;
    dReal _fFreeInc;
    int _nTotalDOF;
    std::vector<dReal> _qlower, _qupper;
    IkFilterCallbackFn _filterfn;
    IkParameterization::Type _iktype;
};

#endif
