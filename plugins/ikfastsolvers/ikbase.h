// Copyright (C) 2006-2009 Rosen Diankov (rdiankov@cs.cmu.edu)
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

template <typename IKReal, IkSolverBase::Parameterization::Type IKType, typename Solution>
class IkFastSolver : public IkSolverBase
{
 public:
    typedef bool (*IkFn)(const IKReal* eetrans, const IKReal* eerot, const IKReal* pfree, std::vector<Solution>& vsolutions);
    
 IkFastSolver(IkFn pfnik, const std::vector<int>& vfreeparams, dReal fFreeInc, int nTotalJoints, EnvironmentBasePtr penv) : IkSolverBase(penv), _vfreeparams(vfreeparams), _pfnik(pfnik), _fFreeInc(fFreeInc), _nTotalJoints(nTotalJoints) {}

    inline boost::shared_ptr<IkFastSolver<IKReal,IKType,Solution> > shared_solver() { return boost::static_pointer_cast<IkFastSolver<IKReal,IKType, Solution> >(shared_from_this()); }
    inline boost::shared_ptr<IkFastSolver<IKReal,IKType,Solution> const> shared_solver_const() const { return boost::static_pointer_cast<IkFastSolver<IKReal,IKType,Solution> const>(shared_from_this()); }
    inline boost::weak_ptr<IkFastSolver<IKReal,IKType,Solution> > weak_solver() { return shared_solver(); }

    virtual void SetJointLimits()
    {
        RobotBase::ManipulatorPtr pmanip(_pmanip);
        RobotBasePtr probot = pmanip->GetRobot();
        RobotBase::RobotStateSaver saver(probot);
        probot->SetActiveDOFs(pmanip->GetArmJoints());
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
        _cblimits = probot->RegisterChangeCallback(KinBody::Prop_JointLimits,boost::bind(&IkFastSolver<IKReal,IKType,Solution>::SetJointLimits,boost::bind(&sptr_from<IkFastSolver<IKReal,IKType,Solution> >, weak_solver())));

        if( _nTotalJoints != (int)pmanip->GetArmJoints().size() ) {
            RAVELOG_ERRORA(str(boost::format("ik %s configured with different number of joints than robot manipulator (%d!=%d)\n")%GetXMLId()%pmanip->GetArmJoints().size()%_nTotalJoints));
            return false;
        }

        // get the joint limits
        RobotBase::RobotStateSaver saver(probot);
        probot->SetActiveDOFs(pmanip->GetArmJoints());
        SetJointLimits();
        return true;
    }

    virtual bool Solve(const Parameterization& param, const std::vector<dReal>& q0, bool bCheckEnvCollision, boost::shared_ptr< std::vector<dReal> > result)
    {
        if( param.GetType() != IKType ) {
            RAVELOG_WARNA("ik solver only supports type %d",IKType);
            return false;
        }

        RobotBase::ManipulatorPtr pmanip(_pmanip);
        if( bCheckEnvCollision && _CheckIndependentCollision(pmanip) )
            return false;
        
        RobotBasePtr probot = pmanip->GetRobot();
        RobotBase::RobotStateSaver saver(probot);
        probot->SetActiveDOFs(pmanip->GetArmJoints());
        std::vector<IKReal> vfree(_vfreeparams.size());
        return ComposeSolution(_vfreeparams, vfree, 0, q0, boost::bind(&IkFastSolver::_SolveSingle,shared_solver(), boost::ref(param),boost::ref(vfree),boost::ref(q0),bCheckEnvCollision,result));
    }

    virtual bool Solve(const Parameterization& param, bool bCheckEnvCollision, std::vector< std::vector<dReal> >& qSolutions)
    {
        if( param.GetType() != IKType ) {
            RAVELOG_WARNA("ik solver only supports type %d",IKType);
            return false;
        }

        RobotBase::ManipulatorPtr pmanip(_pmanip);
        if( bCheckEnvCollision && _CheckIndependentCollision(pmanip) )
            return false;

        RobotBasePtr probot = pmanip->GetRobot();
        RobotBase::RobotStateSaver saver(probot);
        probot->SetActiveDOFs(pmanip->GetArmJoints());
        std::vector<IKReal> vfree(_vfreeparams.size());
        qSolutions.resize(0);
        ComposeSolution(_vfreeparams, vfree, 0, vector<dReal>(), boost::bind(&IkFastSolver::_SolveAll,shared_solver(), param,boost::ref(vfree),bCheckEnvCollision,boost::ref(qSolutions)));
        return qSolutions.size()>0;
    }

    virtual bool Solve(const Parameterization& param, const std::vector<dReal>& q0, const std::vector<dReal>& vFreeParameters, bool bCheckEnvCollision, boost::shared_ptr< std::vector<dReal> > result)
    {
        if( param.GetType() != IKType ) {
            RAVELOG_WARNA("ik solver only supports type %d",IKType);
            return false;
        }

        if( vFreeParameters.size() != _vfreeparams.size() )
            throw openrave_exception("free parameters not equal",ORE_InvalidArguments);

        RobotBase::ManipulatorPtr pmanip(_pmanip);
        if( bCheckEnvCollision && _CheckIndependentCollision(pmanip) )
            return false;

        RobotBasePtr probot = pmanip->GetRobot();
        RobotBase::RobotStateSaver saver(probot);
        probot->SetActiveDOFs(pmanip->GetArmJoints());
        std::vector<IKReal> vfree(_vfreeparams.size());
        for(size_t i = 0; i < _vfreeparams.size(); ++i)
            vfree[i] = vFreeParameters[i]*(_qupper[_vfreeparams[i]]-_qlower[_vfreeparams[i]]) + _qlower[_vfreeparams[i]];
        return _SolveSingle(param,vfree,q0,bCheckEnvCollision,result);
    }
    virtual bool Solve(const Parameterization& param, const std::vector<dReal>& vFreeParameters, bool bCheckEnvCollision, std::vector< std::vector<dReal> >& qSolutions)
    {
        if( param.GetType() != IKType ) {
            RAVELOG_WARNA("ik solver only supports type %d",IKType);
            return false;
        }

        if( vFreeParameters.size() != _vfreeparams.size() )
            throw openrave_exception("free parameters not equal",ORE_InvalidArguments);

        RobotBase::ManipulatorPtr pmanip(_pmanip);
        if( bCheckEnvCollision && _CheckIndependentCollision(pmanip) )
            return false;

        RobotBasePtr probot = pmanip->GetRobot();
        RobotBase::RobotStateSaver saver(probot);
        probot->SetActiveDOFs(pmanip->GetArmJoints());
        std::vector<IKReal> vfree(_vfreeparams.size());
        for(size_t i = 0; i < _vfreeparams.size(); ++i)
            vfree[i] = vFreeParameters[i]*(_qupper[_vfreeparams[i]]-_qlower[_vfreeparams[i]]) + _qlower[_vfreeparams[i]];
        qSolutions.resize(0);
        _SolveAll(param,vfree,bCheckEnvCollision,qSolutions);
        return qSolutions.size()>0;
    }

    virtual int GetNumFreeParameters() const { return (int)_vfreeparams.size(); }
    virtual bool GetFreeParameters(std::vector<dReal>& pFreeParameters) const
    {
        RobotBase::ManipulatorPtr pmanip(_pmanip);
        RobotBasePtr probot = pmanip->GetRobot();
        std::vector<dReal> values;
        std::vector<dReal>::const_iterator itscale = _vfreeparamscales.begin();
        probot->GetJointValues(values);
        pFreeParameters.resize(_vfreeparams.size());
        for(size_t i = 0; i < _vfreeparams.size(); ++i)
            pFreeParameters[i] = (values[pmanip->GetArmJoints()[_vfreeparams[i]]]-_qlower[_vfreeparams[i]]) * *itscale++;

        return true;
    }

    virtual RobotBase::ManipulatorPtr GetManipulator() const { return RobotBase::ManipulatorPtr(_pmanip); }

private:
    bool ComposeSolution(const std::vector<int>& vfreeparams, vector<IKReal>& vfree, int freeindex, const vector<dReal>& q0, const boost::function<bool()>& fn)
    {
        if( freeindex >= (int)vfreeparams.size())
            return fn();

        // start searching for phi close to q0, as soon as a solution is found for the curphi, return it
        dReal startphi = q0.size() == _qlower.size() ? q0[vfreeparams[freeindex]] : 0;
        dReal upperphi = _qupper[vfreeparams[freeindex]], lowerphi = _qlower[vfreeparams[freeindex]], deltaphi = 0;
        int iter = 0;

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
                    deltaphi += _fFreeInc; // increment
                    ++iter;
                    continue;
                }

                deltaphi += _fFreeInc; // increment
            }

            iter++;

            vfree[freeindex] = curphi;
            if( ComposeSolution(_vfreeparams, vfree, freeindex+1,q0, fn) )
                return true;
        }

        // explicitly test 0 since many edge cases involve 0s
        if( _qlower[vfreeparams[freeindex]] <= 0 && _qupper[vfreeparams[freeindex]] >= 0 ) {
            vfree[freeindex] = 0;
            if( ComposeSolution(_vfreeparams, vfree, freeindex+1,q0, fn) )
                return true;
        }

        return false;
    }

    bool _SolveSingle(const Parameterization& param, const vector<IKReal>& vfree, const vector<dReal>& q0, bool bCheckEnvCollision, boost::shared_ptr< std::vector<dReal> > result)
    {
        TransformMatrix t = param.GetTransform();
        IKReal eetrans[3] = {t.trans.x, t.trans.y, t.trans.z};
        IKReal eerot[9] = {t.m[0],t.m[1],t.m[2],t.m[4],t.m[5],t.m[6],t.m[8],t.m[9],t.m[10]};

        std::vector<Solution> vsolutions;
        bool bStopSearching = false;

        if( _pfnik(eetrans, eerot, vfree.size()>0?&vfree[0]:NULL, vsolutions) ) {
            vector<IKReal> vsolfree;

            RobotBase::ManipulatorPtr pmanip(_pmanip);
            dReal bestdist = 1000;
            std::vector<dReal> vravesol(pmanip->GetArmJoints().size());
            std::vector<dReal> vbest;
            std::vector<IKReal> sol(pmanip->GetArmJoints().size());
            // find the first valid solution that satisfies joint constraints and collisions
            FOREACH(itsol, vsolutions) {
                if( itsol->GetFree().size() > 0 ) {
                    // have to search over all the free parameters of the solution!
                    vsolfree.resize(itsol->GetFree().size());

                    if( ComposeSolution(itsol->GetFree(), vsolfree, 0, q0, boost::bind(&IkFastSolver::_ValidateSolutionSingle,shared_solver(), boost::ref(*itsol), boost::make_tuple(vsolfree, q0, bCheckEnvCollision), boost::ref(sol), boost::ref(vravesol), boost::ref(vbest), boost::ref(bestdist), boost::ref(param), boost::ref(bStopSearching))) )
                        break;
                }
                else {
                    if( _ValidateSolutionSingle(*itsol, boost::make_tuple(vector<IKReal>(),q0,bCheckEnvCollision), sol, vravesol, vbest, bestdist, param, bStopSearching) )
                        break;
                }
            }

            if( bStopSearching )
                return true; // return true to stop the search

            // return as soon as a solution is found, since we're visiting phis starting from q0, we are guaranteed
            // that the solution will be close (ie, phi's dominate in the search). This is to speed things up
            if( vbest.size() == _qlower.size() ) {
                if( !!result )
                    *result = vbest;
                return true;
            }
        }

        return false;
    }
    
    // validate a solution
    bool _ValidateSolutionSingle(const Solution& iksol, boost::tuple<const vector<IKReal>&, const vector<dReal>&,bool> freeq0check, std::vector<IKReal>& sol, std::vector<dReal>& vravesol, std::vector<dReal>& vbest, dReal& bestdist, const Parameterization& param, bool& bStopSearching)
    {
        const vector<IKReal>& vfree = boost::get<0>(freeq0check);
        iksol.GetSolution(&sol[0],vfree.size()>0?&vfree[0]:NULL);

        for(int i = 0; i < (int)sol.size(); ++i)
            vravesol[i] = (dReal)sol[i];

        if( !checkjointangles(vravesol) ) {
//            stringstream ss; ss << "bad joint angles: ";
//            FOREACH(it,vravesol)
//                ss << *it << " ";
//            ss << endl;
//            RAVELOG_VERBOSEA(ss.str().c_str());
            return false;
        }

        // check for self collisions
        RobotBase::ManipulatorPtr pmanip(_pmanip);
        RobotBasePtr probot = pmanip->GetRobot();
        probot->SetActiveDOFValues(vravesol);
        if( probot->CheckSelfCollision() )
            return false;

        COLLISIONREPORT report;
        if( boost::get<2>(freeq0check) && GetEnv()->CheckCollision(KinBodyConstPtr(probot), boost::shared_ptr<COLLISIONREPORT>(&report,null_deleter())) ) {
            if( !!report.plink1 && !!report.plink2 ) {
                RAVELOG_VERBOSEA(str(boost::format("IKFastSolver: collision %s:%s with %s:%s\n")%report.plink1->GetParent()->GetName()%report.plink1->GetName()%report.plink2->GetParent()->GetName()%report.plink2->GetName()));
            }
            else
                RAVELOG_VERBOSEA("ik collision, no link\n");

            // if gripper is colliding, solutions will always fail, so completely stop solution process
            if( param.GetType() == Parameterization::Type_Transform6D && pmanip->CheckEndEffectorCollision(param.GetTransform()*pmanip->GetGraspTransform()) ) {
                bStopSearching = true;
                return true; // return true to stop the search
            }

            return false;
        }

        // solution is valid, check with q0
        if( boost::get<1>(freeq0check).size() == pmanip->GetArmJoints().size() ) {
            dReal d = 0;
            for(int k = 0; k < (int)pmanip->GetArmJoints().size(); ++k)
                d += SQR(vravesol[k]-boost::get<1>(freeq0check)[k]);

            if( bestdist > d ) {
                vbest = vravesol;
                bestdist = d;
            }
        }
        else
            vbest = vravesol;
        return true;
    }

    bool _SolveAll(const Parameterization& param, const vector<IKReal>& vfree, bool bCheckEnvCollision, std::vector< std::vector<dReal> >& qSolutions)
    {
        TransformMatrix t = param.GetTransform();
        IKReal eetrans[3] = {t.trans.x, t.trans.y, t.trans.z};
        IKReal eerot[9] = {t.m[0],t.m[1],t.m[2],t.m[4],t.m[5],t.m[6],t.m[8],t.m[9],t.m[10]};

        RobotBase::ManipulatorPtr pmanip(_pmanip);
        RobotBasePtr probot = pmanip->GetRobot();
        
        std::vector<Solution> vsolutions;
        if( _pfnik(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, vsolutions) ) {
            vector<IKReal> vsolfree;
            vector<dReal> vravesol(pmanip->GetArmJoints().size());
            std::vector<IKReal> sol(pmanip->GetArmJoints().size());
            FOREACH(itsol, vsolutions) {
                if( itsol->GetFree().size() > 0 ) {
                    // have to search over all the free parameters of the solution!
                    vsolfree.resize(itsol->GetFree().size());
                    
                    ComposeSolution(itsol->GetFree(), vsolfree, 0, vector<dReal>(), boost::bind(&IkFastSolver::_ValidateSolutionAll,shared_solver(),probot, boost::ref(*itsol), boost::ref(vsolfree), bCheckEnvCollision, boost::ref(sol), boost::ref(vravesol), boost::ref(qSolutions)));
                }
                else {
                    if( _ValidateSolutionAll(probot, *itsol, vector<IKReal>(), bCheckEnvCollision, sol, vravesol, qSolutions) )
                        break;
                }
            }
        }

        return false;
    }

    bool _ValidateSolutionAll(RobotBasePtr probot, const Solution& iksol, const vector<IKReal>& vfree, bool bCheckEnvCollision, std::vector<IKReal>& sol, std::vector<dReal>& vravesol, std::vector< std::vector<dReal> >& qSolutions)
    {
        iksol.GetSolution(&sol[0],vfree.size()>0?&vfree[0]:NULL);

        for(int i = 0; i < (int)sol.size(); ++i)
            vravesol[i] = (dReal)sol[i];
            
        // find the first valid solutino that satisfies joint constraints and collisions
        if( !checkjointangles(vravesol) )
            return false;

        // check for self collisions
        probot->SetActiveDOFValues(vravesol);
        if( probot->CheckSelfCollision() )
            return false;

        if( bCheckEnvCollision && GetEnv()->CheckCollision(KinBodyConstPtr(probot)) )
            return false;

        qSolutions.push_back(vravesol);
        return false;
    }

    bool checkjointangles(std::vector<dReal>& vravesol)
    {
        for(int j = 0; j < (int)_qlower.size(); ++j) {
            if( _qlower[j] < -PI && vravesol[j] > _qupper[j] )
                vravesol[j] -= 2*PI;
            if( _qupper[j] > PI && vravesol[j] < _qlower[j] )
                vravesol[j] += 2*PI;
            if( vravesol[j] < _qlower[j] || vravesol[j] > _qupper[j] ) {
                return false;
            }
        }

        return true;
    }

    bool _CheckIndependentCollision(RobotBase::ManipulatorPtr pmanip)
    {
        COLLISIONREPORT report;
        if( pmanip->CheckIndependentCollision(boost::shared_ptr<COLLISIONREPORT>(&report,null_deleter())) ) {
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
    std::vector<dReal> _vfreeparamscales;
    boost::shared_ptr<void> _cblimits;
    IkFn _pfnik;
    dReal _fFreeInc;
    int _nTotalJoints;
    std::vector<dReal> _qlower, _qupper;
};

#endif
