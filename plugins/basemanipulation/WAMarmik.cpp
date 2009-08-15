// Copyright (C) 2006-2008 Rosen Diankov (rdiankov@cs.cmu.edu)
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

#include "WAMarmik.h"

#ifndef SQR
template <class T>
inline T SQR(T x) { return x * x; }
#endif

WAMArmIK::WAMArmIK(EnvironmentBase* penv, bool bFullDOF) : IkSolverBase(penv)
{
    _bFullDOF = bFullDOF;
}

/// WAMIK author: Rosen Diankov (email rdiankov@cs.cmu.edu for bugs, comments, etc)
bool WAMArmIK::Init(RobotBase* probot, const RobotBase::Manipulator* pmanip, int options)
{
    assert( probot != NULL );
    _probot = probot;
    if( _probot == NULL || pmanip == NULL )
        return false;

    _bUseGraspTrans = _bFullDOF ? false : !!(options&WAMOPT_UseGraspTrans);

    // get the joint limits
    const vector<int>& vjoints = pmanip->_vecarmjoints;

    vector<dReal> qlower, qupper;
    _probot->GetJointLimits(qlower, qupper);
    _qlower.resize(vjoints.size()); _qupper.resize(vjoints.size());

    for(int i = 0; i < (int)vjoints.size(); ++i) {
        _qlower[i] = qlower[vjoints[i]];
        _qupper[i] = qupper[vjoints[i]];
    }

    if( _bUseGraspTrans ) {
        vElbowToGrasp = pmanip->tGrasp.trans+WAMVector(-0.045f, 0, 0.3f);
        alength2 = lengthsqr3(vElbowToGrasp);
        WAMReal totallength2 = lengthsqr3(pmanip->tGrasp.trans+Vector(0,0,0.85f));
        abdenom = (WAMReal)(1 / (2.0f * RaveSqrt(alength2) * (0.55184f)));
        elbowoffset = (WAMReal)acos( (totallength2 - alength2 - (WAMReal)0.30453f) * abdenom);
        fElbowOffset2 = 2 * elbowoffset;
        elbowoffset = PI - elbowoffset;
    }
    else {
        // a = 0.3033562, b = 0.551837838
        vElbowToGrasp = WAMVector(-0.045f, 0, 0.3f);
        alength2 = lengthsqr3(vElbowToGrasp);
        WAMReal totallength2 = lengthsqr3(Vector(0,0,0.85f));
        abdenom = (WAMReal)(1 / (2.0f * RaveSqrt(alength2) * (0.55184f)));

        elbowoffset = (WAMReal)acos( (totallength2 - alength2 - (WAMReal)0.30453f) * abdenom);
        fElbowOffset2 = 2 * elbowoffset;
        elbowoffset = PI - elbowoffset;

        fElbowOffset2 = 0.461f;
    }

    if( _qlower.size() > 0 )
        fiFreeParam = 1.0f / (_qupper[0]-_qlower[0]);
    else fiFreeParam = 1;

    return true;
}

// end eff transform is the transform of the wrist with respect to the base arm link
bool WAMArmIK::Solve(const Transform &_T, const dReal* q0, bool bCheckEnvCollision, dReal* qResult)
{
    assert( _probot != NULL );
    
    const RobotBase::Manipulator* pmanip = _probot->GetActiveManipulator();
    assert( pmanip != NULL );
    
    // the world coordinate system is at the origin of the intersection of the first 3 joint axes
    Transform T;
    if( _bUseGraspTrans )
        T = _T * pmanip->tGrasp;
    else
        T = _T;
    T.trans -= Vector(0.22f, 0.14f, 0.346f);

    assert( pmanip->_vecarmjoints.size() == _qlower.size() );

    RobotBase::RobotStateSaver saver(_probot);

    _probot->SetActiveDOFs(pmanip->_vecarmjoints);

    //RAVEPRINT(L"IKgoal: %f %f %f, %f %f %f %f\n", T.trans.x, T.trans.y, T.trans.z, T.rot.x, T.rot.y, T.rot.z, T.rot.w);

    int i, j;

    // start searching for phi close to q0, as soon as a solution is found for the curphi, return it
    dReal startphi = q0 != NULL ? q0[0] : 0;
    dReal upperphi = _qupper[0], lowerphi = _qlower[0], deltaphi = 0;
    int iter = 0;
    bool bsuccess = false;

    Transform tWorld = pmanip->pBase->GetTransform() * _T * pmanip->tGrasp; // used for comparison

    dReal bestdist = 1000; // only valid if q0 != NULL

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
                deltaphi += GetPhiInc(); // increment
                ++iter;
                continue;
            }

            deltaphi += GetPhiInc(); // increment
        }

        iter++;

        int sols = _Solve(T, curphi, q0);
        dReal* pbest = NULL;

        for(i = 0; i < sols; ++i) {

//            _probot->SetActiveDOFValues(NULL, _psolutions[i], false);
//            Transform t = GetBodyTransform(_probot->GetLinks()[7]->GetBody());
//            dReal* pf = _psolutions[i];
//            RAVEPRINT(L"sol: %f %f %f %f %f %f %f,\n    p: %f %f %f, r: %f %f %f %f\n", pf[0], pf[1], pf[2], pf[3], pf[4], pf[5], pf[6],
//                t.trans.x, t.trans.y, t.trans.z, t.rot.x, t.rot.y, t.rot.z, t.rot.w);

            // find the first valid solutino that satisfies joint constraints and collisions
            for(j = 0; j < (int)pmanip->_vecarmjoints.size(); ++j) {
                if( _psolutions[i][j] < _qlower[j] || _psolutions[i][j] > _qupper[j] )
                    break;
            }

            if( j < (int)pmanip->_vecarmjoints.size() )
                continue; // out of bounds

            // check for self collisions (does WAM ever self-collide?)
            _probot->SetActiveDOFValues(NULL, _psolutions[i]);

            if( _probot->CheckSelfCollision() )
                continue;

            COLLISIONREPORT report;
            if( bCheckEnvCollision && GetEnv()->CheckCollision(_probot, &report) ) {
                if( report.plink1 != NULL && report.plink2 != NULL ) {
                    RAVELOG(L"WAMIK: collision %S:%S with %S:%S\n", report.plink1->GetParent()->GetName(), report.plink1->GetName(), report.plink2->GetParent()->GetName(), report.plink2->GetName());
                }
                continue;
            }

            // solution is valid, check with q0
            if( q0 != NULL ) {

                dReal d = 0;

                if( _bFullDOF ) {
                    for(int k = 0; k < (int)pmanip->_vecarmjoints.size(); ++k)
                        d += SQR(_psolutions[i][k]-q0[k]);
                }
                else {
                    // get the closest pose in terms of rotation
                    //Vector vee = pmanip->GetEndEffectorTransform().trans;
                    //RAVEPRINT(L"tr: %f %f %f : %f %f %f\n", vee.x, vee.y, vee.z, T.trans.x, T.trans.y, T.trans.z);
                    d = lengthsqr4(pmanip->GetEndEffectorTransform().rot-tWorld.rot);
                    dReal d2 = lengthsqr4(pmanip->GetEndEffectorTransform().rot+tWorld.rot);
                    if( d2 < d ) d = d2;
                }

                if( bestdist > d ) {
                    pbest = _psolutions[i];
                    bestdist = d;
                }
            }
            else {
                pbest = _psolutions[i];
                break;
            }
        }

        // return as soon as a solution is found, since we're visiting phis starting from q0[0], we are guaranteed
        // that the solution will be close (ie, phi's dominate in the search). This is to speed things up
        if( pbest != NULL ) {

            if( qResult != NULL )
                memcpy(qResult, pbest, pmanip->_vecarmjoints.size()*sizeof(dReal));
            bsuccess = true;

            if( _bFullDOF )
                break;
        }
    }

    return bsuccess;
}

bool WAMArmIK::Solve(const Transform &_T, bool bCheckEnvCollision, std::vector< std::vector<dReal> >& qSolutions)
{
    assert( _probot != NULL );
    const RobotBase::Manipulator* pmanip = _probot->GetActiveManipulator();

    assert( pmanip != NULL );
    
    qSolutions.resize(0);

    // the world coordinate system is at the origin of the intersection of the first 3 joint axes
    Transform T;
    if( _bUseGraspTrans )
        T = _T * pmanip->tGrasp;
    else
        T = _T;
    T.trans -= Vector(0.22f, 0.14f, 0.346f);

    assert( pmanip->_vecarmjoints.size() && _qlower.size() );

    RobotBase::RobotStateSaver saver(_probot);

    _probot->SetActiveDOFs(pmanip->_vecarmjoints);

    int i, j;

    // start searching for phi close to q0, as soon as a solution is found for the curphi, return it
    WAMReal startphi = 0;
    WAMReal upperphi = _qupper[0], lowerphi = _qlower[0], deltaphi = 0;
    int iter = 0;

    while(1) {

        WAMReal curphi = startphi;
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
                ++iter;
                deltaphi += GetPhiInc(); // increment
                continue;
            }

            deltaphi += GetPhiInc(); // increment
        }

        iter++;

        int sols = _Solve(T, curphi, NULL);

        for(i = 0; i < sols; ++i) {
            // find the first valid solutino that satisfies joint constraints and collisions
            for(j = 0; j < (int)pmanip->_vecarmjoints.size(); ++j) {
                if( _psolutions[i][j] < _qlower[j] || _psolutions[i][j] > _qupper[j] )
                    break;
            }

            if( j < (int)pmanip->_vecarmjoints.size() )
                continue; // out of bounds

            // check for self collisions (does WAM ever self-collide?)
            _probot->SetActiveDOFValues(NULL, _psolutions[i]);

            if( _probot->CheckSelfCollision() )
                continue;

            if( bCheckEnvCollision && GetEnv()->CheckCollision(_probot) )
                continue;

            qSolutions.push_back(vector<dReal>());
            qSolutions.back().resize(pmanip->_vecarmjoints.size());
            memcpy(&qSolutions.back()[0], _psolutions[i], sizeof(dReal)*pmanip->_vecarmjoints.size());
        }
    }

    return qSolutions.size()>0;
}

bool WAMArmIK::Solve(const Transform &_T, const dReal* q0, const dReal* pFreeParameters,
                     bool bCheckEnvCollision, dReal* qResult)
{
    if( pFreeParameters == NULL )
        return Solve(_T, q0, bCheckEnvCollision, qResult);

    assert( _probot != NULL );
    
    const RobotBase::Manipulator* pmanip = _probot->GetActiveManipulator();
    assert( pmanip != NULL );
    
    // the world coordinate system is at the origin of the intersection of the first 3 joint axes
    Transform T;
    if( _bUseGraspTrans )
        T = _T * pmanip->tGrasp;
    else
        T = _T;
    T.trans -= Vector(0.22f, 0.14f, 0.346f);

    assert( pmanip->_vecarmjoints.size() == _qlower.size() );

    RobotBase::RobotStateSaver saver(_probot);

    _probot->SetActiveDOFs(pmanip->_vecarmjoints);

    int i, j;

    // start searching for phi close to q0, as soon as a solution is found for the curphi, return it
    Transform tWorld = pmanip->pBase->GetTransform() * _T * pmanip->tGrasp; // used for comparison

    dReal bestdist = 1000; // only valid if q0 != NULL
    int sols = _Solve(T, _qlower[0] + (_qupper[0]-_qlower[0])*pFreeParameters[0], q0, true);
    dReal* pbest = NULL;
    
    for(i = 0; i < sols; ++i) {
        
        // find the first valid solutino that satisfies joint constraints and collisions
        for(j = 0; j < (int)pmanip->_vecarmjoints.size(); ++j) {
            if( _psolutions[i][j] < _qlower[j] || _psolutions[i][j] > _qupper[j] )
                break;
        }
        
        if( j < (int)pmanip->_vecarmjoints.size() )
            continue; // out of bounds
        
        // check for self collisions (does WAM ever self-collide?)
        _probot->SetActiveDOFValues(NULL, _psolutions[i]);
        
        if( _probot->CheckSelfCollision() )
            continue;
        
        COLLISIONREPORT report;
        if( bCheckEnvCollision && GetEnv()->CheckCollision(_probot, &report) ) {
            if( report.plink1 != NULL && report.plink2 != NULL ) {
                RAVELOG(L"WAMIK: collision %S:%S with %S:%S\n", report.plink1->GetParent()->GetName(), report.plink1->GetName(), report.plink2->GetParent()->GetName(), report.plink2->GetName());
            }
            continue;
        }
        
        // solution is valid, check with q0
        if( q0 != NULL ) {
            
            dReal d = 0;
            
            if( _bFullDOF ) {
                for(int k = 0; k < (int)pmanip->_vecarmjoints.size(); ++k)
                    d += SQR(_psolutions[i][k]-q0[k]);
            }
            else {
                // get the closest pose in terms of rotation
                //Vector vee = pmanip->GetEndEffectorTransform().trans;
                    //RAVEPRINT(L"tr: %f %f %f : %f %f %f\n", vee.x, vee.y, vee.z, T.trans.x, T.trans.y, T.trans.z);
                d = lengthsqr4(pmanip->GetEndEffectorTransform().rot-tWorld.rot);
                dReal d2 = lengthsqr4(pmanip->GetEndEffectorTransform().rot+tWorld.rot);
                if( d2 < d ) d = d2;
            }
            
            if( bestdist > d ) {
                pbest = _psolutions[i];
                bestdist = d;
            }
        }
        else {
            pbest = _psolutions[i];
            break;
        }
    }

    // return as soon as a solution is found, since we're visiting phis starting from q0[0], we are guaranteed
    // that the solution will be close (ie, phi's dominate in the search). This is to speed things up
    if( pbest != NULL ) {
        if( qResult != NULL )
            memcpy(qResult, pbest, pmanip->_vecarmjoints.size()*sizeof(dReal));
        return true;
    }
    
    return false;
}

bool WAMArmIK::Solve(const Transform &_T, const dReal* pFreeParameters,
                     bool bCheckEnvCollision, std::vector< std::vector<dReal> >& qSolutions)
{
    if( pFreeParameters == NULL )
        return Solve(_T, bCheckEnvCollision, qSolutions);

        assert( _probot != NULL );
    const RobotBase::Manipulator* pmanip = _probot->GetActiveManipulator();

    assert( pmanip != NULL );
    
    qSolutions.resize(0);

    // the world coordinate system is at the origin of the intersection of the first 3 joint axes
    Transform T;
    if( _bUseGraspTrans )
        T = _T * pmanip->tGrasp;
    else
        T = _T;
    T.trans -= Vector(0.22f, 0.14f, 0.346f);

    assert( pmanip->_vecarmjoints.size() && _qlower.size() );

    RobotBase::RobotStateSaver saver(_probot);

    _probot->SetActiveDOFs(pmanip->_vecarmjoints);

    int i, j;

    // start searching for phi close to q0, as soon as a solution is found for the curphi, return it
    int sols = _Solve(T, _qlower[0] + (_qupper[0]-_qlower[0])*pFreeParameters[0], NULL, true);
    
    for(i = 0; i < sols; ++i) {
        // find the first valid solutino that satisfies joint constraints and collisions
        for(j = 0; j < (int)pmanip->_vecarmjoints.size(); ++j) {
            if( _psolutions[i][j] < _qlower[j] || _psolutions[i][j] > _qupper[j] )
                break;
        }
        
        if( j < (int)pmanip->_vecarmjoints.size() )
            continue; // out of bounds
        
        // check for self collisions (does WAM ever self-collide?)
        _probot->SetActiveDOFValues(NULL, _psolutions[i]);
        
        if( _probot->CheckSelfCollision() )
            continue;
        
        if( bCheckEnvCollision && GetEnv()->CheckCollision(_probot) )
            continue;
        
        qSolutions.push_back(vector<dReal>());
        qSolutions.back().resize(pmanip->_vecarmjoints.size());
        memcpy(&qSolutions.back()[0], _psolutions[i], sizeof(dReal)*pmanip->_vecarmjoints.size());
    }

    return qSolutions.size()>0;
}

int WAMArmIK::_Solve(const Transform &T, WAMReal phi, const dReal* q0, bool bStrictPhi)
{
    // use law of cosines to calculate elbow angle
    WAMReal elbow;
    WAMReal c2 = lengthsqr3(T.trans);
    WAMReal celbow = ((WAMReal)0.30453f + alength2 - c2) * abdenom;
    
    if( celbow > 1 ) {
        if( celbow > 1.0001f )
            return 0;
        celbow = 1;
    }
    else if( celbow < -1 ) {
        // have to do extra checks for floating point precision
        if( celbow < -1.0001f )
            return 0;
        celbow = -1;
    }
        
    elbow = acos(celbow)-elbowoffset; // need the offset
    assert(!isnan(elbow));

    int n = _SolveElbow(T, phi, elbow, 0, q0, bStrictPhi);
    n += _SolveElbow(T, phi, fElbowOffset2-elbow, n, q0, bStrictPhi);

    return n;
}

int WAMArmIK::_SolveElbow(const Transform &T, WAMReal phi, WAMReal elbow, int startindex, const dReal* q0, bool bStrictPhi)
{
    WAMReal joints[4];
    WAMReal cphi = cos(phi), sphi = sin(phi), ctheta3, stheta3;
    WAMReal celbow = cos(elbow), selbow = sin(elbow);

    if( fabs(elbow) < 0.01f && _bFullDOF ) { // ~1 degree

        // special case when arm is all the way up
        joints[0] = atan2(T.trans.y,T.trans.x);
        joints[1] = atan2(sqrt(T.trans.x*T.trans.x+T.trans.y*T.trans.y), T.trans.z);
        
        // if joints[0] is out of range, flip 180
        if( joints[0] <= _qlower[0] ) {
            joints[0] += PI;
            joints[1] = -joints[1];
        }
        else if( joints[0] >= _qupper[0] ) {
            joints[0] -= PI;
            joints[1] = -joints[1];
        }
        
        if( joints[1] < _qlower[1] ) {
            if( -joints[1] > _qupper[1] )
                return 0;
            joints[1] = -joints[1];
            if( joints[0] < 0 ) {
                joints[0] += PI;
                if( joints[0] >= _qupper[0] )
                    return 0;
            }
            else {
                joints[0] -= PI;
                if( joints[0] <= _qlower[0] )
                    return 0;
            }
        }

        if( bStrictPhi ) {
            if( RaveFabs(phi-bStrictPhi) > 0.001f )
                return 0;
        }

        joints[2] = q0 != NULL ? q0[2] : 0; // for lack of better values
        joints[3] = elbow;

        dReal stheta2 = (dReal)sin(joints[1]), ctheta2 = (dReal)cos(joints[1]);
        dReal stheta3 = (dReal)sin(joints[2]), ctheta3 = (dReal)cos(joints[2]);

        TransformMatrix mrot12, mrot34, mrotall;
        cphi = cos(joints[0]); sphi = sin(joints[0]);
        mrot12.m[0] = dReal(cphi * ctheta2);     mrot12.m[1] = -(dReal)sphi;  mrot12.m[2] = dReal(cphi * stheta2);
        mrot12.m[4] = dReal(sphi * ctheta2);     mrot12.m[5] = (dReal)cphi;  mrot12.m[6] = dReal(sphi * stheta2);
        mrot12.m[8] = -(dReal)stheta2;           mrot12.m[9] = 0;      mrot12.m[10] = (dReal)ctheta2;
        mrot34.m[0] = dReal(ctheta3 * celbow);   mrot34.m[1] = -(dReal)stheta3;  mrot34.m[2] = dReal(ctheta3 * selbow);
        mrot34.m[4] = dReal(stheta3 * celbow);   mrot34.m[5] = (dReal)ctheta3;  mrot34.m[6] = dReal(stheta3 * selbow);
        mrot34.m[8] = -(dReal)selbow;            mrot34.m[9] = 0;      mrot34.m[10] = (dReal)celbow;
        mult3_s4(mrotall.m, mrot12.m, mrot34.m);
                
        return _SolveRotation(T, joints, mrotall, startindex, q0);
    }

    WAMReal tx, ty, tz;
    tx = (WAMReal)0.045f + vElbowToGrasp.x * celbow + vElbowToGrasp.z * selbow;
    ty = vElbowToGrasp.y;
    tz = (WAMReal)0.55f - vElbowToGrasp.x * selbow + vElbowToGrasp.z * celbow;

    // note there are two solution for elbow
    WAMReal trad = tx*tx + ty*ty;

    //assert( trad > 1e-5f );
    if( trad <= 1e-5f ) {

        if( !_bFullDOF ) {
            WAMReal g1 = T.trans.x * cphi + T.trans.y * sphi;
            WAMReal g2 = T.trans.x * -sphi + T.trans.y * cphi;

            if( RaveFabs(g2) > 0.001f )
                return 0;
            
            WAMReal theta2 = (dReal)atan2((WAMReal)g1,(WAMReal)T.trans.z);
            WAMReal ctheta2 = cos(theta2), stheta2 = sin(theta2);

            TransformMatrix mrot12, mirot4;
            mrot12.m[0] = dReal(cphi * ctheta2);     mrot12.m[1] = -(dReal)sphi;  mrot12.m[2] = dReal(cphi * stheta2);
            mrot12.m[4] = dReal(sphi * ctheta2);     mrot12.m[5] = (dReal)cphi;  mrot12.m[6] = dReal(sphi * stheta2);
            mrot12.m[8] = -(dReal)stheta2;           mrot12.m[9] = 0;      mrot12.m[10] = (dReal)ctheta2;
            
            mirot4.m[0] = celbow;     mirot4.m[1] = 0;  mirot4.m[2] = -selbow;
            mirot4.m[4] = 0;     mirot4.m[5] = 1;  mirot4.m[6] = 0;
            mirot4.m[8] = (dReal)selbow;            mirot4.m[9] = 0;      mirot4.m[10] = (dReal)celbow;

            Transform tLocalT = mrot12.inverse() * T * mirot4;

            // find the closest ang such that
            // |tLocalT.rot - (cos(ang/2),0,0,sin(ang/2))| is minimized
            WAMReal stheta3 = (tLocalT.rot.x+tLocalT.rot.w)*(tLocalT.rot.x+tLocalT.rot.w) - 1.0f;
            WAMReal theta3;
            if( stheta3 < -1 )
                theta3 = -PI/2;
            else if( stheta3 > 1 )
                theta3 = PI/2;
            else
                theta3 = asin(stheta3);
            
            _psolutions[startindex][0] = (dReal)phi;
            _psolutions[startindex][1] = theta2;
            _psolutions[startindex][2] = (dReal)theta3; // pick closest to current desired rotation
            _psolutions[startindex][3] = (dReal)elbow;

            _psolutions[startindex+1][0] = (dReal)phi;
            _psolutions[startindex+1][1] = theta2;
            _psolutions[startindex+1][2] = PI-(dReal)theta3; // pick closest to current desired rotation
            _psolutions[startindex+1][3] = (dReal)elbow;

            return 1;
        }

        return 0;
    }

    WAMReal itrad = 1/trad, tz2 = tz*tz;
    
    WAMReal g1 = T.trans.x * cphi + T.trans.y * sphi;
    WAMReal f1sqr[2];

    WAMReal ftemp = g1*g1 + T.trans.z*T.trans.z;
    int n = solvequad((WAMReal)1, 2*tz2 - ftemp, tz2*tz2 - ftemp * tz2, f1sqr[0], f1sqr[1]);
    if( n == 0 )
        return 0;

    // only accept positive roots
    if( n == 1 ) {
        if( f1sqr[0] < 0 )
        return 0;
    }
    else { // n == 2
        if( f1sqr[0] < 0 ) {
            n = 1;
            f1sqr[0] = f1sqr[1];

            if( f1sqr[0] < 0 )
                return 0;
        }
        if( f1sqr[1] < 0 )
            n = 1;
    }

    int orgindex = startindex;

    joints[0] = phi;
    
    for(int i = 0; i < n; ++i) {

        WAMReal f1 = sqrt(f1sqr[i]);
        WAMReal denom = 1 / (tz2+f1sqr[i]);

        for(int j = 0; j < 2; ++j) {

            if( j == 1 )
                f1 = -f1;

            WAMReal stheta2, ctheta2 = (g1 * f1 + T.trans.z * tz) * denom;

            if( ctheta2 > 1 ) {
                if( ctheta2 > 1.0001f )
                    break;
                ctheta2 = 1;
                stheta2 = 0;
            }
            else if( ctheta2 < -1 ) {
                if( ctheta2 < -1.0001f )
                    break;
                ctheta2 = -1;
                stheta2 = 0;
            }
            else
                stheta2 = (g1 * tz - T.trans.z * f1) * denom;

            // intermediate values
            WAMReal f2 = -T.trans.x * sphi + T.trans.y * cphi; // =g2
        
            // estimate theta3
            ctheta3 = (tx * f1 + ty * f2) * itrad;
            if( ctheta3 > 1 ) {
                if( ctheta3 > 1.0001f )
                    continue;
                ctheta3 = 1;
                stheta3 = 0;
            }
            else if( ctheta3 < -1 ) {
                if( ctheta3 < -1.0001f )
                    continue;
                ctheta3 = -1;
                stheta3 = 0;
            }
            else {
                stheta3 = (tx * f2 - ty * f1) * itrad;
                if( stheta3 > 1 ) {
                    if( stheta3 > 1.0001f )
                        continue;
                }
                else if( stheta3 < -1 ) {
                    if( stheta3 < -1.0001f )
                        continue;
                }
            }

            joints[1] = atan2(stheta2, ctheta2);
            joints[2] = atan2(stheta3, ctheta3);
            joints[3] = elbow;

            TransformMatrix mrot12, mrot34, mrotall;
            mrot12.m[0] = dReal(cphi * ctheta2);     mrot12.m[1] = -(dReal)sphi;  mrot12.m[2] = dReal(cphi * stheta2);
            mrot12.m[4] = dReal(sphi * ctheta2);     mrot12.m[5] = (dReal)cphi;  mrot12.m[6] = dReal(sphi * stheta2);
            mrot12.m[8] = -(dReal)stheta2;            mrot12.m[9] = 0;      mrot12.m[10] = (dReal)ctheta2;
            mrot34.m[0] = dReal(ctheta3 * celbow);     mrot34.m[1] = -(dReal)stheta3;  mrot34.m[2] = dReal(ctheta3 * selbow);
            mrot34.m[4] = dReal(stheta3 * celbow);     mrot34.m[5] = (dReal)ctheta3;  mrot34.m[6] = dReal(stheta3 * selbow);
            mrot34.m[8] = -(dReal)selbow;            mrot34.m[9] = 0;      mrot34.m[10] = (dReal)celbow;
            mult3_s4(mrotall.m, mrot12.m, mrot34.m);

            startindex += _SolveRotation(T, joints, mrotall, startindex, q0);
        }
    }

    return startindex-orgindex;
}

int WAMArmIK::_SolveRotation(const Transform &T, const WAMReal* pjoints, const TransformMatrix& mBaseRot, int startindex, const dReal* q0)
{
    if( _bFullDOF ) {

        // get the rotation induced by the 4 joint values
        TransformMatrix mT = mBaseRot.inverse() * TransformMatrix(T);

        assert( startindex < 15 );
        _psolutions[startindex][0] = (dReal)pjoints[0];
        _psolutions[startindex][1] = (dReal)pjoints[1];
        _psolutions[startindex][2] = (dReal)pjoints[2];
        _psolutions[startindex][3] = (dReal)pjoints[3];

        if( mT.m[6]*mT.m[6] + mT.m[2]*mT.m[2] < 1e-7 )
            _psolutions[startindex][4] = q0 != NULL ? q0[4] : 0;
        else
            _psolutions[startindex][4] = atan2(mT.m[6], mT.m[2]);

        dReal c1 = cos(_psolutions[startindex][4]);
        dReal s1 = sin(_psolutions[startindex][4]);

        _psolutions[startindex][5] = atan2(c1 * mT.m[2] + s1 * mT.m[6], mT.m[10]);
        _psolutions[startindex][6] = atan2(-s1 * mT.m[0] + c1 * mT.m[4], -s1 * mT.m[1] + c1 * mT.m[5]);

    //    dReal c2 = cos(_psolutions[startindex][5]);
    //    dReal s2 = sin(_psolutions[startindex][5]);
    //    dReal c3 = cos(_psolutions[startindex][6]);
    //    dReal s3 = sin(_psolutions[startindex][6]);
    //    TransformMatrix m1, m2, m3;
    //    m1.m[0] = c1; m1.m[1] = -s1; m1.m[4] = s1; m1.m[5] = c1;
    //    m2.m[0] = c2; m2.m[2] = s2; m2.m[8] = -s2; m2.m[10] = c2;
    //    m3.m[0] = c3; m3.m[1] = -s3; m3.m[4] = s3; m3.m[5] = c3;
    //    TransformMatrix mall = m1 * m2 * m3;

        // add another solution
        _psolutions[startindex+1][0] = (dReal)pjoints[0];
        _psolutions[startindex+1][1] = (dReal)pjoints[1];
        _psolutions[startindex+1][2] = (dReal)pjoints[2];
        _psolutions[startindex+1][3] = (dReal)pjoints[3];

        if( _psolutions[startindex][4] > 0 )
            _psolutions[startindex+1][4] = _psolutions[startindex][4] - PI;
        else
            _psolutions[startindex+1][4] = _psolutions[startindex][4] + PI;

        _psolutions[startindex+1][5] = atan2(-c1 * mT.m[2] - s1 * mT.m[6], mT.m[10]);
        _psolutions[startindex+1][6] = atan2(s1 * mT.m[0] - c1 * mT.m[4], s1 * mT.m[1] - c1 * mT.m[5]);
        return 2;
    }
    else {
        _psolutions[startindex][0] = (dReal)pjoints[0];
        _psolutions[startindex][1] = (dReal)pjoints[1];
        _psolutions[startindex][2] = (dReal)pjoints[2];
        _psolutions[startindex][3] = (dReal)pjoints[3];
        return 1;
    }
}

bool WAMArmIK::GetFreeParameters(dReal* pFreeParameters) const
{
    assert( _probot != NULL && pFreeParameters != NULL );

    const RobotBase::Manipulator* pmanip = _probot->GetActiveManipulator();
    if( pmanip == NULL )
        return false;
    assert( pmanip->_vecarmjoints.size() > 0 && pmanip->_vecarmjoints[0] < _probot->GetDOF());
    assert( _qlower.size() > 0 && _qupper.size() > 0 );

    dReal values[3];
    _probot->GetJointFromDOFIndex(pmanip->_vecarmjoints[0])->GetValues(values);
    pFreeParameters[0] = (values[0]-_qlower[0])*fiFreeParam;
    return true;
}
