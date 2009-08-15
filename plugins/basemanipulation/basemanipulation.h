#ifndef OPENRAVE_BASEMANIPULATION_H
#define OPENRAVE_BASEMANIPULATION_H

inline bool JitterActiveDOF(RobotBase* robot)
{
    assert( robot != NULL );
    RAVELOG_VERBOSEA("starting jitter active dof...\n");
    vector<dReal> curdof, newdof, lower, upper;
    robot->GetActiveDOFValues(curdof);
    robot->GetActiveDOFLimits(lower, upper);
    newdof = curdof;

    dReal fRand = 0.03f;
    int iter = 0;

    if(robot->CheckSelfCollision())
        RAVEPRINT(L"JitterActiveDOFs: initial config in self collision!\n");

    while(robot->GetEnv()->CheckCollision(robot) || robot->CheckSelfCollision() ) {
        if( iter > 5000 ) {
            RAVEPRINT(L"Failed to find noncolliding position for robot\n");

            robot->SetActiveDOFValues(NULL, &curdof[0]);

            // display collision report
            COLLISIONREPORT report;
            if( robot->GetEnv()->CheckCollision(robot, &report) ) {
                if( report.plink1 != NULL && report.plink2 != NULL ) {
                    RAVELOG(L"Jitter collision %S:%S with %S:%S\n", report.plink1->GetParent()->GetName(), report.plink1->GetName(), report.plink2->GetParent()->GetName(), report.plink2->GetName());
                }
            }
            
            return false;        
        }

        for(int j = 0; j < robot->GetActiveDOF(); j++)
            newdof[j] = CLAMP_ON_RANGE(curdof[j] + fRand * (RANDOM_FLOAT()-0.5f), lower[j], upper[j]);

        /// dangerous
//        if( (iter%1000) == 499 )
//            fRand *= 2;

        robot->SetActiveDOFValues(NULL, &newdof[0]);
        ++iter;
    }
    
    return true;
}

inline bool JitterTransform(KinBody* pbody, float fJitter)
{
    assert( pbody != NULL );
    RAVELOG_VERBOSEA("starting jitter transform...\n");

    // randomly add small offset to the body until it stops being in collision
    Transform transorig = pbody->GetTransform();
    Transform transnew = transorig;
    int iter = 0;
    while(pbody->GetEnv()->CheckCollision(pbody) ) {

        if( iter > 1000 )
            return false;

        transnew.trans = transorig.trans + fJitter * Vector(RANDOM_FLOAT()-0.5f, RANDOM_FLOAT()-0.5f, RANDOM_FLOAT()-0.5f);
        pbody->SetTransform(transnew);
        ++iter;
    }

    return true;
}

/// Samples numsamples of solutions and each solution to vsolutions
/// \return number of ik solutions sampled
inline int SampleIkSolutions(RobotBase* robot, const Transform& tgrasp, int numsamples, vector<dReal>& vsolutions)
{
    const RobotBase::Manipulator* pmanip = robot->GetActiveManipulator();

    if( numsamples <= 0 )
        return 0;

    int _numsamples = numsamples;

    vector< vector<dReal> > viksolutions;
    vector<dReal> vfree(pmanip->GetNumFreeParameters());
    for(int iter = 0; iter < 50*numsamples; ++iter) {
        for(int i = 0; i < (int)vfree.size(); ++i)
            vfree[i] = robot->GetEnv()->RandomFloat();
            
        if( pmanip->FindIKSolutions(tgrasp, vfree.size() > 0 ? &vfree[0] : NULL, viksolutions, true) ) {

            FOREACH(itsol, viksolutions) {
                vsolutions.insert(vsolutions.end(), itsol->begin(), itsol->end());
                if( --_numsamples <= 0 )
                    return numsamples;
            }
        }
    }

    bool bSuccess = pmanip->FindIKSolutions(tgrasp, viksolutions, true);
    if( !bSuccess || viksolutions.size() == 0 )
        return false;

    while(1) {
        FOREACH(itsol, viksolutions) {
            vsolutions.insert(vsolutions.end(), itsol->begin(), itsol->end());
            if( --_numsamples <= 0 )
                return numsamples;
        }
    }

    return numsamples-_numsamples;
}

class MoveUnsyncGoalFunction : public PlannerBase::GoalFunction
{
public:
    virtual void SetRobot(RobotBase* robot) { _robot = robot; thresh = 0; }
    virtual float GetGoalThresh() { return thresh; }
    
    virtual float Eval(const void* pConfiguration) {
        
        assert( _robot != NULL );

        // check if there's a collision when hand moves to final config
        RobotBase::RobotStateSaver saver(_robot);
            
        _robot->SetActiveDOFValues(NULL, (dReal*)pConfiguration);
            
        _robot->SetActiveDOFs(vhandjoints);
        _robot->GetActiveDOFValues(vhandvalues);
            
        int numiter = 10;
        vhanddelta.resize(vhandjoints.size());
        for(size_t i = 0; i < vhandjoints.size(); ++i)
            vhanddelta[i] = (vhandgoal[i]-vhandvalues[i])/(dReal)numiter;
            
        while(numiter-- > 0) {
                
            for(size_t i = 0; i < vhandjoints.size(); ++i)
                vhandvalues[i] += vhanddelta[i];
                
            _robot->SetActiveDOFValues(NULL, &vhandvalues[0]);
                
            // don't check self collisions!!!!
            if( _robot->GetEnv()->CheckCollision(_robot))
                return 1000;
        }

        // check self collision with the final config
        if( _robot->CheckSelfCollision() )
            return 1000;

        // check that final goal is away from obstacles
        newvalues.resize(vhandgoal.size());
        numiter = 10;
        while(numiter > 0) {
            for(size_t i = 0; i < newvalues.size(); ++i)
                newvalues[i] = vhandgoal[i] + 0.2f*(_robot->GetEnv()->RandomFloat()-0.5f);
            _robot->SetActiveDOFValues(NULL, &newvalues[0], true);
            if( _robot->CheckSelfCollision() )
                continue;

            if( _robot->GetEnv()->CheckCollision(_robot) )
                return 1000;
            --numiter;
        }

        // not in collision so returnt true
        return 0;
    }
    
    vector<dReal> vhandgoal;
    vector<int> vhandjoints;
    float thresh;

    static bool _MoveUnsyncJoints(EnvironmentBase* penv, RobotBase* robot, Trajectory* ptraj, const vector<int>& vhandjoints, const vector<dReal>& vhandgoal, const wchar_t* pplannername)
    {
        if( vhandjoints.size() == 0 || vhandjoints.size() != vhandgoal.size() || ptraj == NULL )
            return false;
    
        MoveUnsyncGoalFunction goalfn;
        goalfn.thresh = 0;
        goalfn.vhandjoints = vhandjoints;
        goalfn.vhandgoal = vhandgoal;
        goalfn.SetRobot(robot);

        PlannerBase::PlannerParameters params;
        params.pgoalfn = &goalfn;
        params.nMaxIterations = 20000;
        robot->GetActiveDOFValues(params.vinitialconfig);
        params.vParameters.push_back(0.04f);
        params.vnParameters.push_back(0); // one step

        if( goalfn.Eval(&params.vinitialconfig[0]) <= goalfn.GetGoalThresh() ) {
            // already done
            Trajectory::TPOINT pt; pt.q = params.vinitialconfig;
            ptraj->AddPoint(pt);
            return true;
        }

        if( pplannername == NULL )
            pplannername = L"BasicRRT";
        boost::shared_ptr<PlannerBase> planner(penv->CreatePlanner(pplannername));
        if( planner == NULL ) {
            RAVEPRINT(L"failed to find planner %S\n", pplannername);
            return false;
        }
    
        if( !planner->InitPlan(robot, &params) )
            return false;
        if( !planner->PlanPath(ptraj) )
            return false;

        //ptraj->CalcTrajTiming(robot, ptraj->GetInterpMethod(), true, true);
        return true;
    }

protected:
    vector<dReal> vhandvalues, vhanddelta, values, newvalues;
    RobotBase* _robot;
};

#endif
