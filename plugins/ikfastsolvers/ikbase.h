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

template <typename IKReal, typename Solution>
class IkFastSolver : public IkSolverBase
{
public:
    typedef bool (*IkFn)(const IKReal* eetrans, const IKReal* eerot, const IKReal* pfree, std::vector<Solution>& vsolutions);

    IkFastSolver(IkFn pfnik, const std::vector<int>& vfreeparams, dReal fFreeInc, int nTotalJoints, EnvironmentBase* penv) : IkSolverBase(penv), _probot(NULL), _vfreeparams(vfreeparams), _pfnik(pfnik), _fFreeInc(fFreeInc)
    {
        _varmjoints.resize(nTotalJoints);
    }

    virtual bool Init(RobotBase* probot, const RobotBase::Manipulator* pmanip, int options)
    {
        assert( probot != NULL );
        _probot = probot;
        if( _probot == NULL || pmanip == NULL )
            return false;

        if( _varmjoints.size() != pmanip->_vecarmjoints.size() ) {
            RAVELOG_ERRORA("ik %s configured with different number of joints than robot manipulator (%"PRIdS"!=%"PRIdS")\n", GetXMLId(), _varmjoints.size(), pmanip->_vecarmjoints.size());
            return false;
        }

        _varmjoints = pmanip->_vecarmjoints;

        // get the joint limits
        std::vector<dReal> qlower, qupper;
        _probot->GetJointLimits(qlower, qupper);
        _qlower.resize(_varmjoints.size()); _qupper.resize(_varmjoints.size());

        for(int i = 0; i < (int)_varmjoints.size(); ++i) {
            _qlower[i] = qlower[_varmjoints[i]];
            _qupper[i] = qupper[_varmjoints[i]];
        }

        _vfreeparamscales.clear();
        FOREACH(itfree, _vfreeparams) {
            if( *itfree < 0 || *itfree >= (int)_qlower.size() ) {
                RAVELOG_ERRORA("free parameter idx %d out of bounds\n", *itfree);
                return false;
            }

            if( _qupper[*itfree] > _qlower[*itfree] )
                _vfreeparamscales.push_back(1.0f/(_qupper[*itfree]-_qlower[*itfree]));
            else
                _vfreeparamscales.push_back(0.0f);
        }

        return true;
    }
    
    virtual bool Solve(const Transform &transEE, const dReal* q0, bool bCheckEnvCollision, dReal* qResult)
    {
        assert( _probot != NULL );

        RobotBase::RobotStateSaver saver(_probot);
        _probot->SetActiveDOFs(_varmjoints);

        std::vector<IKReal> vfree(_vfreeparams.size());
        
        TransformMatrix t = transEE;
        IKReal eetrans[3] = {t.trans.x, t.trans.y, t.trans.z};
        IKReal eerot[9] = {t.m[0],t.m[1],t.m[2],t.m[4],t.m[5],t.m[6],t.m[8],t.m[9],t.m[10]};

        return ComposeSolution(_vfreeparams, vfree.size()>0?&vfree[0]:NULL, 0, q0, boost::bind(&IkFastSolver::_SolveSingle,this, eerot,eetrans,vfree.size()>0?&vfree[0]:NULL,q0,bCheckEnvCollision,qResult));
    }

    virtual bool Solve(const Transform &transEE, bool bCheckEnvCollision, std::vector< std::vector<dReal> >& qSolutions)
    {
        assert( _probot != NULL );

        RobotBase::RobotStateSaver saver(_probot);
        _probot->SetActiveDOFs(_varmjoints);

        std::vector<IKReal> vfree(_vfreeparams.size());
        
        TransformMatrix t = transEE;
        IKReal eetrans[3] = {t.trans.x, t.trans.y, t.trans.z};
        IKReal eerot[9] = {t.m[0],t.m[1],t.m[2],t.m[4],t.m[5],t.m[6],t.m[8],t.m[9],t.m[10]};

        qSolutions.resize(0);
        ComposeSolution(_vfreeparams, vfree.size()>0?&vfree[0]:NULL, 0, NULL, boost::bind(&IkFastSolver::_SolveAll,this, eerot,eetrans,vfree.size()>0?&vfree[0]:NULL,bCheckEnvCollision,boost::ref(qSolutions)));

        return qSolutions.size()>0;
    }

    virtual bool Solve(const Transform &transEE, const dReal* q0, const dReal* pFreeParameters, bool bCheckEnvCollision, dReal* qResult)
    {
        if( pFreeParameters == NULL )
            return Solve(transEE,q0,bCheckEnvCollision,qResult);

        assert( _probot != NULL );

        RobotBase::RobotStateSaver saver(_probot);
        _probot->SetActiveDOFs(_varmjoints);

        std::vector<IKReal> vfree(_vfreeparams.size());
        for(size_t i = 0; i < _vfreeparams.size(); ++i)
            vfree[i] = pFreeParameters[i]*(_qupper[_vfreeparams[i]]-_qlower[_vfreeparams[i]]) + _qlower[_vfreeparams[i]];

        TransformMatrix t = transEE;
        IKReal eetrans[3] = {t.trans.x, t.trans.y, t.trans.z};
        IKReal eerot[9] = {t.m[0],t.m[1],t.m[2],t.m[4],t.m[5],t.m[6],t.m[8],t.m[9],t.m[10]};
        
        return _SolveSingle(eerot,eetrans,vfree.size()>0?&vfree[0]:NULL,q0,bCheckEnvCollision,qResult);
    }

    virtual bool Solve(const Transform &transEE, const dReal* pFreeParameters, bool bCheckEnvCollision, std::vector< std::vector<dReal> >& qSolutions)
    {
        if( pFreeParameters == NULL )
            return Solve(transEE,bCheckEnvCollision,qSolutions);

        assert( _probot != NULL );

        RobotBase::RobotStateSaver saver(_probot);
        _probot->SetActiveDOFs(_varmjoints);

        std::vector<IKReal> vfree(_vfreeparams.size());
        for(size_t i = 0; i < _vfreeparams.size(); ++i)
            vfree[i] = pFreeParameters[i]*(_qupper[_vfreeparams[i]]-_qlower[_vfreeparams[i]]) + _qlower[_vfreeparams[i]];
        
        TransformMatrix t = transEE;
        IKReal eetrans[3] = {t.trans.x, t.trans.y, t.trans.z};
        IKReal eerot[9] = {t.m[0],t.m[1],t.m[2],t.m[4],t.m[5],t.m[6],t.m[8],t.m[9],t.m[10]};

        qSolutions.resize(0);
        _SolveAll(eerot,eetrans,vfree.size()>0?&vfree[0]:NULL,bCheckEnvCollision,qSolutions);

        return qSolutions.size()>0;
    }

    virtual int GetNumFreeParameters() const { return (int)_vfreeparams.size(); }
    virtual bool GetFreeParameters(dReal* pFreeParameters) const
    {
        assert( _probot != NULL && pFreeParameters != NULL );

        std::vector<dReal> values;
        std::vector<dReal>::const_iterator itscale = _vfreeparamscales.begin();
        _probot->GetJointValues(values);
        FOREACHC(itfree, _vfreeparams)
            *pFreeParameters++ = (values[_varmjoints[*itfree]]-_qlower[*itfree]) * *itscale++;

        return true;
    }
    virtual RobotBase* GetRobot() const { return _probot; }

private:
    template <class SolveFn>
    bool ComposeSolution(const std::vector<int>& vfreeparams, IKReal* pfree, int freeindex, const dReal* q0, const SolveFn& fn)
    {
        if( freeindex >= (int)vfreeparams.size())
            return fn();

        // start searching for phi close to q0, as soon as a solution is found for the curphi, return it
        dReal startphi = q0 != NULL ? q0[vfreeparams[freeindex]] : 0;
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

            pfree[freeindex] = curphi;
            if( ComposeSolution(_vfreeparams, pfree, freeindex+1,q0, fn) )
                return true;
        }

        return false;
    }

    bool _SolveSingle(const IKReal* eerot, const IKReal* eetrans, const IKReal* pfree, const dReal* q0, bool bCheckEnvCollision, dReal* qResult)
    {
        std::vector<Solution> vsolutions;
        if( _pfnik(eetrans, eerot, pfree, vsolutions) ) {
            vector<IKReal> vsolfree;

            dReal bestdist = 1000; // only valid if q0 != NULL
            std::vector<dReal> vravesol(_varmjoints.size());
            std::vector<dReal> vbest;
            std::vector<IKReal> sol(_varmjoints.size());
            // find the first valid solution that satisfies joint constraints and collisions
            FOREACH(itsol, vsolutions) {
                if( itsol->GetFree().size() > 0 ) {
                    // have to search over all the free parameters of the solution!
                    vsolfree.resize(itsol->GetFree().size());

                    ComposeSolution(itsol->GetFree(), &vsolfree[0], 0, q0, boost::bind(&IkFastSolver::_ValidateSolutionSingle,this, boost::ref(*itsol), &vsolfree[0], q0, bCheckEnvCollision, boost::ref(sol), boost::ref(vravesol), boost::ref(vbest), boost::ref(bestdist)));
                }
                else {
                    if( _ValidateSolutionSingle(*itsol, NULL,q0,bCheckEnvCollision, sol, vravesol, vbest, bestdist) )
                        break;
                }
            }

            // return as soon as a solution is found, since we're visiting phis starting from q0, we are guaranteed
            // that the solution will be close (ie, phi's dominate in the search). This is to speed things up
            if( vbest.size() == _qlower.size() && qResult != NULL ) {
                memcpy(&qResult[0], &vbest[0], sizeof(dReal)*_qlower.size());
                return true;
            }
        }

        return false;
    }

    // validate a solution
    bool _ValidateSolutionSingle(const Solution& iksol, const IKReal* pfree, const dReal* q0, bool bCheckEnvCollision, std::vector<IKReal>& sol, std::vector<dReal>& vravesol, std::vector<dReal>& vbest, dReal& bestdist)
    {
        iksol.GetSolution(&sol[0],pfree);

        for(int i = 0; i < (int)sol.size(); ++i)
            vravesol[i] = (dReal)sol[i];

        if( !checkjointangles(vravesol) )
            return false;

        // check for self collisions
        _probot->SetActiveDOFValues(NULL, &vravesol[0]);

        if( _probot->CheckSelfCollision() )
            return false;

        COLLISIONREPORT report;
        if( bCheckEnvCollision && GetEnv()->CheckCollision(_probot, &report) ) {
            if( report.plink1 != NULL && report.plink2 != NULL ) {
                RAVELOG_VERBOSEA("WAMIK: collision %S:%S with %S:%S\n", report.plink1->GetParent()->GetName(), report.plink1->GetName(), report.plink2->GetParent()->GetName(), report.plink2->GetName());
            }
            return false;
        }

        // solution is valid, check with q0
        if( q0 != NULL ) {

            dReal d = 0;
            for(int k = 0; k < (int)_varmjoints.size(); ++k)
                d += SQR(vravesol[k]-q0[k]);

            if( bestdist > d ) {
                vbest = vravesol;
                bestdist = d;
            }
        }
        else
            vbest = vravesol;
        return true;
    }

    bool _SolveAll(const IKReal* eerot, const IKReal* eetrans, const IKReal* pfree, bool bCheckEnvCollision, std::vector< std::vector<dReal> >& qSolutions)
    {
        std::vector<Solution> vsolutions;
        if( _pfnik(eetrans, eerot, pfree, vsolutions) ) {
            vector<IKReal> vsolfree;
            vector<dReal> vravesol(_varmjoints.size());
            std::vector<IKReal> sol(_varmjoints.size());
            FOREACH(itsol, vsolutions) {
                if( itsol->GetFree().size() > 0 ) {
                    // have to search over all the free parameters of the solution!
                    vsolfree.resize(itsol->GetFree().size());
                    
                    ComposeSolution(itsol->GetFree(), &vsolfree[0], 0, NULL, boost::bind(&IkFastSolver::_ValidateSolutionAll,this, boost::ref(*itsol), &vsolfree[0], bCheckEnvCollision, boost::ref(sol), boost::ref(vravesol), boost::ref(qSolutions)));
                }
                else {
                    if( _ValidateSolutionAll(*itsol, NULL, bCheckEnvCollision, sol, vravesol, qSolutions) )
                        break;
                }
            }
        }

        return false;
    }

    bool _ValidateSolutionAll(const Solution& iksol, const IKReal* pfree, bool bCheckEnvCollision, std::vector<IKReal>& sol, std::vector<dReal>& vravesol, std::vector< std::vector<dReal> >& qSolutions)
    {
        iksol.GetSolution(&sol[0],pfree);

        for(int i = 0; i < (int)sol.size(); ++i)
            vravesol[i] = (dReal)sol[i];
            
        // find the first valid solutino that satisfies joint constraints and collisions
        if( !checkjointangles(vravesol) )
            return false;

        // check for self collisions
        _probot->SetActiveDOFValues(NULL, &vravesol[0]);

        if( _probot->CheckSelfCollision() )
            return false;

        if( bCheckEnvCollision && GetEnv()->CheckCollision(_probot) )
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
            if( vravesol[j] < _qlower[j] || vravesol[j] > _qupper[j] )
                return false;
        }

        return true;
    }
    
    template <typename U> U SQR(U t) { return t*t; }

    RobotBase* _probot;
    std::vector<int> _vfreeparams, _varmjoints;
    std::vector<dReal> _vfreeparamscales;
    IkFn _pfnik;
    dReal _fFreeInc;
    std::vector<dReal> _qlower, _qupper;
};

#endif
