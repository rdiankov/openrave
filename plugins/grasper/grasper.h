// Copyright (C) 2006-2008 Carnegie Mellon University (rdiankov@cs.cmu.edu)
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
#ifndef  GRASPER_PLANNER_H
#define  GRASPER_PLANNER_H

// plans a grasp
class GrasperPlanner:  public PlannerBase
{
public:
    GrasperPlanner(EnvironmentBase* penv) : PlannerBase(penv) {}
    bool InitPlan(RobotBase* pbase, const PlannerParameters* pparams);
    bool PlanPath(Trajectory* ptraj, std::ostream* pOutStream);

    virtual RobotBase* GetRobot() const {return robot;};
    virtual const wchar_t* GetDescription() const { return L"Used to simulate grasping."; }

    void UpdateDependents(int ifing, dReal * dofvals);

    RobotBase* robot;
    std::vector<dReal> _initconfig;
    bool _bInit;

protected:
    PlannerParameters _parameters;
};

// very simple interface to use the GrasperPlanner
class GrasperProblem : public ProblemInstance
{
public:
    GrasperProblem(EnvironmentBase* penv);
    virtual ~GrasperProblem();
    virtual void Destroy();
    virtual int main(const char* cmd);
    virtual void SetActiveRobots(const std::vector<RobotBase*>& robots);
    
    virtual bool SendCommand(const char* cmd, std::string& response);

private:

    struct SURFACEPOINT
    {
        SURFACEPOINT(){}
        SURFACEPOINT(dReal px, dReal py, dReal pz, dReal nx, dReal ny, dReal nz){pos.x = px; pos.y = py; pos.z = pz; norm.x = nx; norm.y = ny; norm.z = nz;}

        Vector pos, norm;
        dReal dist;
        std::vector<dReal> dists;
        void PrintToFile(ofstream& of){ of << dist << " " << pos.x <<" " << pos.y <<" " << pos.z <<" " << norm.x <<" " << norm.y <<" " << norm.z << "\n";}
    };


//    bool TriTriContact(const Vector& u1, const Vector& u2, const Vector& u3, const Vector& v1, const Vector& v2, const Vector& v3, Vector& contactpos);

    void SampleObject(KinBody* pbody, vector<SURFACEPOINT>& vpoints, int N, Vector graspcenter);

    // generates samples across a geodesic sphere (the higher the level, the higher the number of points
    void DeterministicallySample(KinBody* pbody, vector<SURFACEPOINT>& vpoints, int levels, Vector graspcenter);

    void BoxSample(KinBody* pbody, vector<SURFACEPOINT>& vpoints, int num_samples, Vector center);

    // computes a distance map. For every point, samples many vectors around the point's normal such that angle
    // between normal and sampled vector doesn't exceeed fTheta. Returns the minimum distance.
    // vpoints needs to already be initialized
    void ComputeDistanceMap(const vector<KinBody*>& bodies, vector<SURFACEPOINT>& vpoints, dReal fTheta);
    void ComputeMultiResMap(vector<KinBody*>& vbodies, KinBody* pbody, vector<SURFACEPOINT>& vpoints, dReal stepsize,dReal maxradius);
    
    void GetStableContacts(std::stringstream* colfile, KinBody* ptarget,Vector& direction, dReal mu);
    
    void UpdateDependentJoints(RobotBase* robot,int ifing, dReal * dofvals);
    
    PlannerBase* planner;
    RobotBase* robot;
    KinBody* pbody;
    Vector normalplane; // guess direction of palm
};

#endif   // BIRRT_PLANNER_H
