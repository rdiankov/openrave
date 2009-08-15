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
#ifndef  WAMARMIK_H
#define  WAMARMIK_H

/// WAMIK author: Rosen Diankov (email rdiankov@cs.cmu.edu for bugs, comments, etc)
class WAMArmIK : public IkSolverBase
{
public:
    enum WAMOptions
    {
        WAMOPT_UseGraspTrans = 1, // only applies to 4DOF IK
    };
    
    WAMArmIK(EnvironmentBase* penv, bool bFullDOF);

    virtual bool Init(RobotBase* probot, const RobotBase::Manipulator* pmanip, int options);

    virtual bool Solve(const Transform &transEE, const dReal* q0, bool bCheckEnvCollision, dReal* qResult);
    virtual bool Solve(const Transform &transEE, bool bCheckEnvCollision, std::vector< std::vector<dReal> >& qSolutions);

    virtual bool Solve(const Transform &endEffTransform, const dReal* q0, const dReal* pFreeParameters,
                       bool bCheckEnvCollision, dReal* qResult);
    virtual bool Solve(const Transform &endEffTransform, const dReal* pFreeParameters,
                       bool bCheckEnvCollision, std::vector< std::vector<dReal> >& qSolutions);

    virtual int GetNumFreeParameters() const { return _bFullDOF?1:0; }
    virtual bool GetFreeParameters(dReal* pFreeParameters) const;
    virtual RobotBase* GetRobot() const { return _probot; }

private:

    typedef double WAMReal; ///< doubles are about a magnitude more precise than floats for WAM IK
    typedef RaveVector<WAMReal> WAMVector;

    /// phi is the first joint angle of the robot
    /// \return number of valid solutions in _psolutions
    int _Solve(const Transform &transEE, WAMReal phi, const dReal* q0, bool bStrictPhi = false);
    int _SolveElbow(const Transform &transEE, WAMReal phi, WAMReal elbow, int startindex, const dReal* q0, bool bStrictPhi);

    /// solves for the last 3 joint values given the first 4. mBaseRot is the rotation matrix induced by the first 4 joint values
    int _SolveRotation(const Transform &transEE, const WAMReal* pjoints, const TransformMatrix& mBaseRot, int startindex, const dReal* q0);

    dReal GetPhiInc() { return _bFullDOF ? 0.05f : 0.01f; }

    RobotBase* _probot;
    WAMReal alength2, abdenom, elbowoffset, fElbowOffset2;
    WAMVector vElbowToGrasp; // vector
    bool _bFullDOF;
    bool _bUseGraspTrans; // adds the translation component of tGrasp
                                 // when computing the elbow angle
    dReal _psolutions[16][7]; // a max of 16 solutions with 7 dofs
    std::vector<dReal> _qlower, _qupper;
    dReal fiFreeParam;
};

#endif
