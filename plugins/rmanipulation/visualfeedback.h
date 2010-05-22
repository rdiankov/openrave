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
#ifndef OPENRAVE_VISUALSERVOING_PROBLEM
#define OPENRAVE_VISUALSERVOING_PROBLEM

struct FRUSTUM
{
    Vector right, up, dir, pos;
    dReal fnear, ffar;
    dReal ffovx,ffovy;
    dReal fcosfovx,fsinfovx,fcosfovy,fsinfovy;

    void Init(const SensorBase::CameraIntrinsics& KK,int width, int height)
    {
        ffovx = atanf(0.5f*width/KK.fx);
        fcosfovx = cosf(ffovx); fsinfovx = sinf(ffovx);
        ffovy = atanf(0.5f*height/KK.fy);
        fcosfovy = cosf(ffovy); fsinfovy = sinf(ffovy);
    }
};

inline bool IsOBBinFrustum(const OBB& o, const FRUSTUM& fr)
{
	// check OBB against all 6 planes
	Vector v = o.pos - fr.pos;

	// if v lies on the left or bottom sides of the frustrum
	// then freflect about the planes to get it on the right and 
	// top sides

	// side planes
	Vector vNorm = fr.fcosfovx * fr.right - fr.fsinfovx * fr.dir;
    if( dot3(v,vNorm) > -o.extents.x * RaveFabs(dot3(vNorm, o.right)) - 
        o.extents.y * RaveFabs(dot3(vNorm, o.up)) - 
        o.extents.z * RaveFabs(dot3(vNorm, o.dir)))
        return false;
    
	vNorm = -fr.fcosfovx * fr.right - fr.fsinfovx * fr.dir;
	if(dot3(v, vNorm) > -o.extents.x * RaveFabs(dot3(vNorm, o.right)) -
				o.extents.y * RaveFabs(dot3(vNorm, o.up)) -
				o.extents.z * RaveFabs(dot3(vNorm, o.dir))) return false;

	vNorm = fr.fcosfovy * fr.up - fr.fsinfovy * fr.dir;
	if(dot3(v, vNorm) > -o.extents.x * RaveFabs(dot3(vNorm, o.right)) -
				o.extents.y * RaveFabs(dot3(vNorm, o.up)) -
				o.extents.z * RaveFabs(dot3(vNorm, o.dir))) return false;

	vNorm = -fr.fcosfovy * fr.up - fr.fsinfovy * fr.dir;
	if(dot3(v, vNorm) > -o.extents.x * RaveFabs(dot3(vNorm, o.right)) -
				o.extents.y * RaveFabs(dot3(vNorm, o.up)) -
				o.extents.z * RaveFabs(dot3(vNorm, o.dir))) return false;

	vNorm.x = dot3(v, fr.dir);
	vNorm.y = o.extents.x * RaveFabs(dot3(fr.dir, o.right)) + 
					o.extents.y * RaveFabs(dot3(fr.dir, o.up)) + 
					o.extents.z * RaveFabs(dot3(fr.dir, o.dir));

	if( (vNorm.x < fr.fnear + vNorm.y) || (vNorm.x > fr.ffar - vNorm.y) ) return false;

	return true;
}

/// returns true if all points on the oriented bounding box are inside the convex hull
/// planes should be facing inside
inline bool IsOBBinConvexHull(const OBB& o, const vector<Vector>& vplanes)
{
    FOREACHC(itplane, vplanes) {
        // side planes
        if( dot3(o.pos,*itplane)+itplane->w < o.extents.x * RaveFabs(dot3(*itplane, o.right))
            + o.extents.y * RaveFabs(dot3(*itplane, o.up))
            + o.extents.z * RaveFabs(dot3(*itplane, o.dir)))
            return false;
    }

    return true;
}

/// samples rays from the projected OBB and returns true if the test function returns true
/// for all the rays. Otherwise, returns false
/// allowableoutliers - specifies the % of allowable outliying rays
bool SampleProjectedOBBWithTest(const OBB& obb, dReal delta, const boost::function<bool(const Vector&)>& testfn,dReal allowableocclusion=0)
{
    dReal fscalefactor = 0.95f; // have to make box smaller or else rays might miss
    Vector vpoints[8] = {obb.pos + fscalefactor*(obb.right*obb.extents.x + obb.up*obb.extents.y + obb.dir*obb.extents.z),
                         obb.pos + fscalefactor*(obb.right*obb.extents.x + obb.up*obb.extents.y - obb.dir*obb.extents.z),
                         obb.pos + fscalefactor*(obb.right*obb.extents.x - obb.up*obb.extents.y + obb.dir*obb.extents.z),
                         obb.pos + fscalefactor*(obb.right*obb.extents.x - obb.up*obb.extents.y - obb.dir*obb.extents.z),
                         obb.pos + fscalefactor*(-obb.right*obb.extents.x + obb.up*obb.extents.y + obb.dir*obb.extents.z),
                         obb.pos + fscalefactor*(-obb.right*obb.extents.x + obb.up*obb.extents.y - obb.dir*obb.extents.z),
                         obb.pos + fscalefactor*(-obb.right*obb.extents.x - obb.up*obb.extents.y + obb.dir*obb.extents.z),
                         obb.pos + fscalefactor*(-obb.right*obb.extents.x - obb.up*obb.extents.y - obb.dir*obb.extents.z)};
//    Vector vpoints3d[8];
//    for(int j = 0; j < 8; ++j) vpoints3d[j] = tcamera*vpoints[j];

    for(int i =0; i < 8; ++i) {
        dReal fz = 1.0f/vpoints[i].z;
        vpoints[i].x *= fz;
        vpoints[i].y *= fz;
        vpoints[i].z = 1;
    }

    int faceindices[3][4];
    if( obb.right.z >= 0 ) {
        faceindices[0][0] = 4; faceindices[0][1] = 5; faceindices[0][2] = 6; faceindices[0][3] = 7;
    }
    else {
        faceindices[0][0] = 0; faceindices[0][1] = 1; faceindices[0][2] = 2; faceindices[0][3] = 3;
    }
    if( obb.up.z >= 0 ) {
        faceindices[1][0] = 2; faceindices[1][1] = 3; faceindices[1][2] = 6; faceindices[1][3] = 7;
    }
    else {
        faceindices[1][0] = 0; faceindices[1][1] = 1; faceindices[1][2] = 4; faceindices[1][3] = 5;
    }
    if( obb.dir.z >= 0 ) {
        faceindices[2][0] = 1; faceindices[2][1] = 3; faceindices[2][2] = 5; faceindices[2][3] = 7;
    }
    else {
        faceindices[2][0] = 0; faceindices[2][1] = 2; faceindices[2][2] = 4; faceindices[2][3] = 6;
    }

    int nallowableoutliers=0;
    if( allowableocclusion > 0 ) {
        // have to compute the area of all the faces!
        dReal farea=0;
        for(int i = 0; i < 3; ++i) {
            Vector v0 = vpoints[faceindices[i][0]];
            Vector v1 = vpoints[faceindices[i][1]]-v0;
            Vector v2 = vpoints[faceindices[i][2]]-v0;
            Vector v;v.Cross(v1,v2);
            farea += v.lengthsqr3();
        }
        nallowableoutliers = (int)(allowableocclusion*farea*0.5/(delta*delta));
    }

    for(int i = 0; i < 3; ++i) {
        Vector v0 = vpoints[faceindices[i][0]];
        Vector v1 = vpoints[faceindices[i][1]]-v0;
        Vector v2 = vpoints[faceindices[i][2]]-v0;
        Vector v3 = vpoints[faceindices[i][3]]-v0;
        dReal f3length = RaveSqrt(v3.lengthsqr2());
        Vector v3norm = v3 * (1.0f/f3length);
        Vector v3perp(-v3norm.y,v3norm.x,0,0);
        dReal f1proj = RaveFabs(dot2(v3perp,v1)), f2proj = RaveFabs(dot2(v3perp,v2));
        
        int n1 = (int)(f1proj/delta);
        dReal n1scale = 1.0f/n1;
        Vector vdelta1 = v1*n1scale;
        Vector vdelta2 = (v1-v3)*n1scale;
        dReal fdeltalen = (RaveFabs(dot2(v3norm,v1)) + RaveFabs(dot2(v3norm,v1-v3)))*n1scale;
        dReal ftotalen = f3length;
        Vector vcur1 = v0, vcur2 = v0+v3;
        for(int j = 0; j <= n1; ++j, vcur1 += vdelta1, vcur2 += vdelta2, ftotalen -= fdeltalen ) {
            int numsteps = (int)(ftotalen/delta);
            Vector vdelta = (vcur2-vcur1)*(1.0f/numsteps), vcur = vcur1;
            for(int k = 0; k <= numsteps; ++k, vcur += vdelta) {
                if( !testfn(vcur) ) {
                    if( nallowableoutliers-- <= 0 )
                        return false;
                }
            }
        }
        
//        Vector vtripoints[6] = {vpoints3d[faceindices[i][0]], vpoints3d[faceindices[i][3]], vpoints3d[faceindices[i][1]],
//                                vpoints3d[faceindices[i][0]], vpoints3d[faceindices[i][1]], vpoints3d[faceindices[i][3]]};
//        penv->drawtrimesh(vtripoints[0], 16, NULL, 2);

        int n2 = (int)(f2proj/delta);
        if( n2 == 0 )
            continue;

        dReal n2scale = 1.0f/n2;
        vdelta1 = v2*n2scale;
        vdelta2 = (v2-v3)*n2scale;
        fdeltalen = (RaveFabs(dot2(v3norm,v2)) + RaveFabs(dot2(v3norm,v2-v3)))*n2scale;
        ftotalen = f3length;
        vcur1 = v0; vcur2 = v0+v3;
        vcur1 += vdelta1; vcur2 += vdelta2; ftotalen -= fdeltalen; // do one step
        for(int j = 0; j < n2; ++j, vcur1 += vdelta1, vcur2 += vdelta2, ftotalen -= fdeltalen ) {
            int numsteps = (int)(ftotalen/delta);
            Vector vdelta = (vcur2-vcur1)*(1.0f/numsteps), vcur = vcur1;
            for(int k = 0; k <= numsteps; ++k, vcur += vdelta) {
                if( !testfn(vcur) ) {
                    if( nallowableoutliers-- <= 0 )
                        return false;
                }
            }
        }
    }

    return true;
}

class VisualFeedbackProblem : public ProblemInstance
{
public:
    inline boost::shared_ptr<VisualFeedbackProblem> shared_problem() { return boost::static_pointer_cast<VisualFeedbackProblem>(shared_from_this()); }
    inline boost::shared_ptr<VisualFeedbackProblem const> shared_problem_const() const { return boost::static_pointer_cast<VisualFeedbackProblem const>(shared_from_this()); }
    friend class VisibilityConstraintFunction;

    class VisibilityConstraintFunction
    {
    public:
    VisibilityConstraintFunction(boost::shared_ptr<VisualFeedbackProblem> vf, KinBodyPtr ptarget) : _vf(vf), _ptarget(ptarget) {
            _report.reset(new COLLISIONREPORT());

            _vTargetOBBs.reserve(_ptarget->GetLinks().size());
            FOREACHC(itlink, _ptarget->GetLinks())
                _vTargetOBBs.push_back(OBBFromAABB((*itlink)->GetCollisionData().ComputeAABB(),(*itlink)->GetTransform()));
            _abTarget = _ptarget->ComputeAABB();
            _fAllowableOcclusion = 0.1;
            _fRayMinDist = 0.05f;

            // create the dummy box
            {
                KinBody::KinBodyStateSaver saver(_ptarget);
                _ptarget->SetTransform(Transform());
                vector<AABB> vboxes; vboxes.push_back(_ptarget->ComputeAABB());

                _ptargetbox = _ptarget->GetEnv()->CreateKinBody();
                _ptargetbox->InitFromBoxes(vboxes,false);
                _ptargetbox->SetName("__visualfeedbacktest__");
                _ptargetbox->GetEnv()->AddKinBody(_ptargetbox);
                _ptargetbox->Enable(false);
                _ptargetbox->SetTransform(_ptarget->GetTransform());
            }
        }
        virtual ~VisibilityConstraintFunction() {
            _ptargetbox->GetEnv()->RemoveKinBody(_ptargetbox);
        }
        
        virtual bool Constraint(const vector<dReal>& pSrcConf, vector<dReal>& pDestConf, int settings)
        {
            TransformMatrix tcamera = _vf->_psensor->GetSensor()->GetTransform();
            if( !InConvexHull(tcamera) )
                return false;
            // no need to check gripper collision

            bool bOcclusion = IsOccluded(tcamera);
            if( bOcclusion )
                return false;
            
            pDestConf = pSrcConf;
            return true;
        }

        bool SampleWithCamera(const TransformMatrix& tcamera, vector<dReal>& pNewSample)
        {
            if( !InConvexHull(tcamera) )
                return false;

            // object is inside, find an ik solution
            Transform tgoalee = tcamera*_vf->_tcameratogripper;
            if( !_vf->_pmanip->FindIKSolution(tgoalee,_vsolution,true) ) {
                RAVELOG_VERBOSEA("no valid ik\n");
                return false;
            }

            // convert the solution into active dofs
            _vf->_robot->GetActiveDOFValues(pNewSample);
            FOREACHC(itarm,_vf->_pmanip->GetArmJoints()) {
                vector<int>::const_iterator itactive = find(_vf->_robot->GetActiveJointIndices().begin(),_vf->_robot->GetActiveJointIndices().end(),*itarm);
                if( itactive != _vf->_robot->GetActiveJointIndices().end() )
                    pNewSample.at((int)(itactive-_vf->_robot->GetActiveJointIndices().begin())) = _vsolution.at((int)(itarm-_vf->_pmanip->GetArmJoints().begin()));
            }
            _vf->_robot->SetActiveDOFValues(pNewSample);
            
            return !IsOccluded(tcamera);
        }

        bool InConvexHull(const TransformMatrix& tcamera)
        {
            Vector vitrans(-tcamera.m[0]*tcamera.trans.x - tcamera.m[4]*tcamera.trans.y - tcamera.m[8]*tcamera.trans.z,
                           -tcamera.m[1]*tcamera.trans.x - tcamera.m[5]*tcamera.trans.y - tcamera.m[9]*tcamera.trans.z,
                           -tcamera.m[2]*tcamera.trans.x - tcamera.m[6]*tcamera.trans.y - tcamera.m[10]*tcamera.trans.z);
            _vconvexplanes3d.resize(_vf->_vconvexplanes.size());
            for(size_t i = 0; i < _vf->_vconvexplanes.size(); ++i) {
                _vconvexplanes3d[i] = tcamera.rotate(_vf->_vconvexplanes[i]);
                _vconvexplanes3d[i].w = dot3(vitrans,_vf->_vconvexplanes[i]);
            }

            FOREACH(itobb,_vTargetOBBs) {
                if( !IsOBBinConvexHull(*itobb,_vconvexplanes3d) ) {
                    RAVELOG_VERBOSEA("box not in camera vision hull\n");
                    return false;
                }
            }
            
            return true;
        }

        /// check if any part of the environment or robot is in front of the camera blocking the object
        /// sample object's surface and shoot rays
        bool IsOccluded(const TransformMatrix& tcamera)
        {
            TransformMatrix tcamerainv = tcamera.inverse();
            _ptargetbox->Enable(true);
            _ptarget->Enable(false);
            _ptargetbox->SetTransform(_ptarget->GetTransform());
            bool bOcclusion = false;
            FOREACH(itobb,_vTargetOBBs) {
                OBB cameraobb = TransformOBB(*itobb,tcamerainv);
                if( !SampleProjectedOBBWithTest(cameraobb, _vf->_fSampleRayDensity, boost::bind(&VisibilityConstraintFunction::TestRay, this, _1, boost::ref(tcamera)),_fAllowableOcclusion) ) {
                    bOcclusion = true;
                    RAVELOG_VERBOSEA("box is occluded\n");
                    break;
                }
            }
            _ptargetbox->Enable(false);
            _ptarget->Enable(true);
            return bOcclusion;
        }

        bool TestRay(const Vector& v, const TransformMatrix& tcamera)
        {
            RAY r;
            r.dir = tcamera.rotate(2.0f*v);
            r.pos = tcamera.trans + 0.05f*r.dir; // move the rays a little forward
            if( !_vf->_robot->GetEnv()->CheckCollision(r,_report) ) {
                RAVELOG_DEBUGA("no collision!?\n");
                return true; // not supposed to happen, but it is OK
            }

//            RaveVector<float> vpoints[2];
//            vpoints[0] = r.pos;
//            BOOST_ASSERT(_report.contacts.size() == 1 );
//            vpoints[1] = _report.contacts[0].pos;
//            _vf->_robot->GetEnv()->drawlinestrip(vpoints[0],2,16,1.0f,Vector(0,0,1));
            if( !(!!_report->plink1 && _report->plink1->GetParent() == _ptargetbox) )
                RAVELOG_DEBUGA(str(boost::format("bad collision: %s\n")%_report->__str__()));
            return !!_report->plink1 && _report->plink1->GetParent() == _ptargetbox;
        }

    private:
        boost::shared_ptr<VisualFeedbackProblem> _vf;
        KinBodyPtr _ptarget;
        KinBodyPtr _ptargetbox; ///< box to represent the target for simulating ray collisions

        vector<OBB> _vTargetOBBs; // object links local AABBs
        vector<dReal> _vsolution;
        CollisionReportPtr _report;
        AABB _abTarget; // target aabb
        vector<Vector> _vconvexplanes3d;
        dReal _fRayMinDist, _fAllowableOcclusion;
    };

    class GoalSampleFunction
    {
    public:
    GoalSampleFunction(boost::shared_ptr<VisualFeedbackProblem> vf, KinBodyPtr ptarget, const vector<Transform>& visibilitytransforms) : _vconstraint(vf,ptarget), _fSampleGoalProb(1.0f), _vf(vf), _visibilitytransforms(visibilitytransforms)
        {
            RAVELOG_DEBUGA(str(boost::format("have %d detection extents hypotheses\n")%_visibilitytransforms.size()));
            _ttarget = ptarget->GetTransform();
            _sphereperms.PermuteStart(_visibilitytransforms.size());
        }
        virtual ~GoalSampleFunction() {}

        virtual bool Sample(vector<dReal>& pNewSample)
        {
            if( RaveRandomFloat() > _fSampleGoalProb )
                return false;
            RobotBase::RobotStateSaver state(_vf->_robot);
            _sphereperms._fn = boost::bind(&GoalSampleFunction::SampleWithParameters,this,_1,boost::ref(pNewSample));
            if( _sphereperms.PermuteContinue() >= 0 )
                return true;

//            // start from the beginning, if nothing, throw
//            _sphereperms.PermuteStart(_visibilitytransforms.size());
//            if( _sphereperms.PermuteContinue() >= 0 )
//                return true;

            return false;
        }

        bool SampleWithParameters(int isample, vector<dReal>& pNewSample)
        {
            TransformMatrix tcamera = _ttarget*_visibilitytransforms.at(isample);
            return _vconstraint.SampleWithCamera(tcamera,pNewSample);
        }

        VisibilityConstraintFunction _vconstraint;
        dReal _fSampleGoalProb;
    private:
        boost::shared_ptr<VisualFeedbackProblem> _vf;
        const vector<Transform>& _visibilitytransforms;

        Transform _ttarget; ///< transform of target
        Vector _vTargetLocalCenter;
        RandomPermuationExecutor _sphereperms;
        vector<Transform> _vcameras; ///< camera transformations in local coord systems
    };

    VisualFeedbackProblem(EnvironmentBasePtr penv) : ProblemInstance(penv)
    {
        __description = "Planning with Visibility Constraints - Rosen Diankov";
        _nManipIndex = -1;
        _fMaxVelMult=1;
        _fSampleRayDensity = 0.001;
        RegisterCommand("SetCamera",boost::bind(&VisualFeedbackProblem::SetCamera,this,_1,_2),
                        "Sets the camera index from the robot and its convex hull");
        RegisterCommand("ProcessVisibilityExtents",boost::bind(&VisualFeedbackProblem::ProcessVisibilityExtents,this,_1,_2),
                        "Converts 3D extents to full 6D camera transforms and prunes bad transforms");
        RegisterCommand("SetCameraTransforms",boost::bind(&VisualFeedbackProblem::SetCameraTransforms,this,_1,_2),
                        "Processes the visibility directions for containment of the object inside the gripper mask");
        RegisterCommand("ComputeVisibility",boost::bind(&VisualFeedbackProblem::ComputeVisibility,this,_1,_2),
                        "Computes the visibility of the current robot configuration");
        RegisterCommand("SampleVisibilityGoal",boost::bind(&VisualFeedbackProblem::SampleVisibilityGoal,this,_1,_2),
                        "Samples a goal with the current manipulator maintaining camera visibility constraints");
        RegisterCommand("MoveToObserveTarget",boost::bind(&VisualFeedbackProblem::MoveToObserveTarget,this,_1,_2),
                        "Approaches a target object while choosing a goal such that the robot's camera sensor sees the object ");
        RegisterCommand("VisualFeedbackGrasping",boost::bind(&VisualFeedbackProblem::VisualFeedbackGrasping,this,_1,_2),
                        "Stochastic greedy grasp planner considering visibility");
    }

    virtual ~VisualFeedbackProblem() {}

    void Destroy()
    {
        ProblemInstance::Destroy();
    }

    int main(const string& args)
    {
        stringstream ss(args);
        string robotname;
        _fMaxVelMult=1;
        ss >> robotname;
        string cmd;
        while(!ss.eof()) {
            ss >> cmd;
            if( !ss )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
            if( cmd == "maxvelmult" )
                ss >> _fMaxVelMult;

            if( ss.fail() || !ss )
                break;
        }
        _robot = GetEnv()->GetRobot(robotname);
        return 0;
    }

    virtual bool SendCommand(std::ostream& sout, std::istream& sinput)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        return ProblemInstance::SendCommand(sout,sinput);
    }

    bool SetCamera(ostream& sout, istream& sinput)
    {
        _pmanip.reset();
        _psensor.reset();
        _vconvexplanes.resize(0);
        _pcamerageom.reset();
        RobotBase::AttachedSensorPtr psensor;
        RobotBase::ManipulatorPtr pmanip;
        vector<Vector> vconvexplanes;

        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "sensorindex" ) {
                int sensorindex=-1;
                sinput >> sensorindex;
                psensor = _robot->GetSensors().at(sensorindex);
            }
            else if( cmd == "sensorname" ) {
                string sensorname;
                sinput >> sensorname;
                FOREACH(itsensor,_robot->GetSensors()) {
                    if( (*itsensor)->GetName() == sensorname ) {
                        psensor = *itsensor;
                        break;
                    }
                }
            }
            else if( cmd == "manipname" ) {
                string manipname;
                sinput >> manipname;
                _nManipIndex = 0;
                FOREACH(itmanip,_robot->GetManipulators()) {
                    if( (*itmanip)->GetName() == manipname ) {
                        pmanip = *itmanip;
                        break;
                    }
                    _nManipIndex++;
                }
            }
            else if( cmd == "convexdata" ) {
                int numpoints=0;
                sinput >> numpoints;
                vector<dReal> vconvexdata(2*numpoints);
                FOREACH(it, vconvexdata)
                    sinput >> *it;
                BOOST_ASSERT(vconvexdata.size() > 2 );
                vector<Vector> vpoints;
                Vector vprev,vprev2,v,vdir,vnorm,vcenter;
                for(size_t i = 0; i < vconvexdata.size(); i += 2 ){ 
                    vpoints.push_back(Vector((vconvexdata[i]-_pcamerageom->KK.cx)/_pcamerageom->KK.fx,(vconvexdata[i+1]-_pcamerageom->KK.cy)/_pcamerageom->KK.fy,0,0));
                    vcenter += vpoints.back();
                }

                if( vpoints.size() > 2 ) {
                    vcenter *= 1.0f/vpoints.size();

                    // get the planes
                    _vconvexplanes.reserve(vpoints.size());
                    vprev = vpoints.back();
                    FOREACH(itv, vpoints) {
                        vdir = *itv-vprev;
                        vnorm.x = vdir.y;
                        vnorm.y = -vdir.x;
                        // normal has to be facing inside
                        if( dot2(vnorm,vcenter-vprev) < 0 ) {
                            vnorm = -vnorm;
                        }
                        vnorm.z = -(vnorm.x*vprev.x+vnorm.y*vprev.y);
                        if( vnorm.lengthsqr3() > 1e-10 )
                            _vconvexplanes.push_back(vnorm.normalize3());
                        vprev = *itv;
                    }

                    // get the center of the convex hull
                    dReal totalarea = 0;
                    _vcenterconvex = Vector(0,0,0);
                    for(size_t i = 2; i < vpoints.size(); ++i) {
                        Vector v0 = vpoints[i-1]-vpoints[0];
                        Vector v1 = vpoints[i]-vpoints[0];
                        dReal area = RaveFabs(v0.x*v1.y-v0.y*v1.x);
                        _vcenterconvex += area*(vpoints[0]+vpoints[i-1]+vpoints[i-1]);
                        totalarea += area;
                    }
                    _vcenterconvex /= 3.0f*totalarea; _vcenterconvex.z = 1;
                }
                else
                    RAVELOG_WARNA(str(boost::format("convex data does not have enough points %d\n")%vconvexdata.size()));
            }
            else {
                RAVELOG_WARNA(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERRORA(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        if( !psensor )
            return false;

        if( !psensor->GetSensor() ) {
            RAVELOG_WARN(str(boost::format("attached sensor %s does not have sensor interface\n")%psensor->GetName()));
            return false;
        }

        if( psensor->GetSensor()->GetSensorGeometry()->GetType() != SensorBase::ST_Camera) {
            RAVELOG_WARN(str(boost::format("attached sensor %s is not a camera")%psensor->GetName()));
            return false;
        }

        // check if there is a manipulator with the same end effector as camera
        std::vector<KinBody::LinkPtr> vattachedlinks;
        _robot->GetRigidlyAttachedLinks(psensor->GetAttachingLink()->GetIndex(), vattachedlinks);
        if( !!pmanip ) {
            if( pmanip->GetEndEffector() != psensor->GetAttachingLink() && find(vattachedlinks.begin(),vattachedlinks.end(),pmanip->GetEndEffector()) == vattachedlinks.end() ) {
                RAVELOG_WARN(str(boost::format("specified manipulator %s end effector not attached to specified sensor %s\n")%pmanip->GetName()%psensor->GetName()));
                return false;
            }
        }
        else {
            _nManipIndex = 0;
            FOREACHC(itmanip,_robot->GetManipulators()) {
                if( (*itmanip)->GetEndEffector() == psensor->GetAttachingLink() || find(vattachedlinks.begin(),vattachedlinks.end(),(*itmanip)->GetEndEffector()) != vattachedlinks.end() ) {
                    pmanip = *itmanip;
                    break;
                }
                _nManipIndex++;
            }

            if( !pmanip ) {
                RAVELOG_WARN(str(boost::format("failed to find manipulator with end effector rigidly attached to sensor %s.\n")%psensor->GetName()));
                return false;
            }
        }

        _tcameratogripper = psensor->GetTransform().inverse()*pmanip->GetEndEffectorTransform();
        _pcamerageom = boost::static_pointer_cast<SensorBase::CameraGeomData>(psensor->GetSensor()->GetSensorGeometry());
        _fSampleRayDensity = 20.0f/_pcamerageom->KK.fx;

        if( _vconvexplanes.size() == 0 ) {
            // pick the camera boundaries
            _vconvexplanes.push_back(Vector(1,0,_pcamerageom->KK.cx/_pcamerageom->KK.fx,0).normalize3()); // -x
            _vconvexplanes.push_back(Vector(-1,0,-(_pcamerageom->width-_pcamerageom->KK.cx)/_pcamerageom->KK.fx,0).normalize3()); // +x
            _vconvexplanes.push_back(Vector(1,0,_pcamerageom->KK.cy/_pcamerageom->KK.fy,0).normalize3()); // -y
            _vconvexplanes.push_back(Vector(-1,0,-(_pcamerageom->height-_pcamerageom->KK.cy)/_pcamerageom->KK.fy,0).normalize3()); // +y
            _vcenterconvex = Vector(0,0,1);
        }

        _pmanip = pmanip;
        _psensor = psensor;
        _vconvexplanes = vconvexplanes;
        sout << _pmanip->GetName();
        return true;
    }

    TransformMatrix ComputeCameraMatrix(const Vector& vdir,dReal fdist,dReal froll)
    {
        Vector vright, vup = Vector(0,1,0) - vdir * vdir.y;
        dReal uplen = vup.lengthsqr3();
        if( uplen < 0.001 ) {
            vup = Vector(0,0,1) - vdir * vdir.z;
            uplen = vup.lengthsqr3();
        }
            
        vup *= (dReal)1.0/RaveSqrt(uplen);
        cross3(vright,vup,vdir);
        TransformMatrix tcamera;
        tcamera.m[2] = vdir.x; tcamera.m[6] = vdir.y; tcamera.m[10] = vdir.z;

        dReal fcosroll = RaveCos(froll), fsinroll = RaveSin(froll);
        tcamera.m[0] = vright.x*fcosroll+vup.x*fsinroll; tcamera.m[1] = -vright.x*fsinroll+vup.x*fcosroll;
        tcamera.m[4] = vright.y*fcosroll+vup.y*fsinroll; tcamera.m[5] = -vright.y*fsinroll+vup.y*fcosroll;
        tcamera.m[8] = vright.z*fcosroll+vup.z*fsinroll; tcamera.m[9] = -vright.z*fsinroll+vup.z*fcosroll;
        tcamera.trans = - fdist * tcamera.rotate(_vcenterconvex);
        return tcamera;
    }

    bool ProcessVisibilityExtents(ostream& sout, istream& sinput)
    {
        string cmd;
        KinBodyPtr ptarget;
        Vector vTargetLocalCenter;
        bool bSetTargetCenter = false;
        int numrolls=8;
        vector<Transform> vtransforms;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "target" ) {
                string name; sinput >> name;
                ptarget = GetEnv()->GetKinBody(name);
            }
            else if( cmd == "localtargetcenter" ) {
                sinput >> vTargetLocalCenter.x >> vTargetLocalCenter.y >> vTargetLocalCenter.z;
                bSetTargetCenter = true;
            }
            else if( cmd == "transforms" ) {
                int numtrans=0;
                sinput >> numtrans;
                vtransforms.resize(numtrans);
                FOREACH(it,vtransforms)
                    sinput >> *it;
            }
            else if( cmd == "numrolls" )
                sinput >> numrolls;
            else if( cmd == "extents" ) {
                if( !bSetTargetCenter && !!ptarget ) {
                    KinBody::KinBodyStateSaver saver(ptarget);
                    ptarget->SetTransform(Transform());
                    vTargetLocalCenter = ptarget->ComputeAABB().pos;
                }
                int numtrans=0;
                sinput >> numtrans;
                dReal deltaroll = PI*2.0f/(dReal)numrolls;
                vtransforms.resize(0); vtransforms.reserve(numtrans*numrolls);
                for(int i = 0; i < numtrans; ++i) {
                    Vector v;
                    sinput >> v.x >> v.y >> v.z;
                    dReal fdist = RaveSqrt(v.lengthsqr3());
                    v *= 1/fdist;
                    dReal froll = 0;
                    for(int iroll = 0; iroll < numrolls; ++iroll, froll += deltaroll) {
                        Transform t = ComputeCameraMatrix(v,fdist,froll); t.trans += vTargetLocalCenter;
                        vtransforms.push_back(t);
                    }
                }
            }
            else if( cmd == "sphere" ) {
                if( !bSetTargetCenter && !!ptarget ) {
                    KinBody::KinBodyStateSaver saver(ptarget);
                    ptarget->SetTransform(Transform());
                    vTargetLocalCenter = ptarget->ComputeAABB().pos;
                }

                KinBody::Link::TRIMESH spheremesh;
                CM::GenerateSphereTriangulation(spheremesh,3);
                int spherelevel = 3, numdists = 0;
                sinput >> spherelevel >> numdists;
                vector<dReal> vdists(numdists);
                FOREACH(it,vdists)
                    sinput >> *it;
                dReal deltaroll = PI*2.0f/(dReal)numrolls;
                vtransforms.resize(spheremesh.vertices.size()*numdists*numrolls);
                vector<Transform>::iterator itcamera = vtransforms.begin();
                for(size_t j = 0; j < spheremesh.vertices.size(); ++j) {
                    Vector v = spheremesh.vertices[j];
                    for(int i = 0; i < numdists; ++i) {
                        dReal froll = 0;
                        for(int iroll = 0; iroll < numrolls; ++iroll, froll += deltaroll) {
                            *itcamera = ComputeCameraMatrix(spheremesh.vertices[j],vdists[i],froll);
                            itcamera->trans += vTargetLocalCenter;
                            ++itcamera;
                        }
                    }
                }
            }
            else {
                RAVELOG_WARNA(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERRORA(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        if( !ptarget )
            return false;

        KinBody::KinBodyStateSaver saver(ptarget);
        ptarget->SetTransform(Transform());
        boost::shared_ptr<GoalSampleFunction> pgoalfn(new GoalSampleFunction(shared_problem(), ptarget,vtransforms));

        // get all the camera positions and test them
        FOREACHC(itcamera, vtransforms) {
            if( pgoalfn->_vconstraint.InConvexHull(*itcamera) ) {
                if( !_pmanip->CheckEndEffectorCollision(*itcamera*_tcameratogripper) )
                    sout << *itcamera << " ";
            }
        }

        return true;
    }

    bool SetCameraTransforms(ostream& sout, istream& sinput)
    {
        string cmd;
        KinBodyPtr ptarget;
        _visibilitytransforms.resize(0);
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "target" ) {
                string name; sinput >> name;
                ptarget = GetEnv()->GetKinBody(name);
            }
            else if( cmd == "transforms" ) {
                int numtrans=0;
                sinput >> numtrans;
                _visibilitytransforms.resize(numtrans);
                FOREACH(it,_visibilitytransforms)
                    sinput >> *it;
            }
            else {
                RAVELOG_WARNA(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERRORA(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        bool bSuccess = true;
        if( !!ptarget ) {
            KinBody::KinBodyStateSaver saver(ptarget);
            ptarget->SetTransform(Transform());
            boost::shared_ptr<GoalSampleFunction> pgoalfn(new GoalSampleFunction(shared_problem(), ptarget,_visibilitytransforms));

            // get all the camera positions and test them
            int i = 0;
            FOREACHC(itcamera, _visibilitytransforms) {
                if( !pgoalfn->_vconstraint.InConvexHull(*itcamera) ) {
                    RAVELOG_WARN("camera transform %d fails constraints\n",i);
                    bSuccess = false;
                }
            }
        }
        
        return bSuccess;
    }

    bool ComputeVisibility(ostream& sout, istream& sinput)
    {
        string cmd;
        KinBodyPtr ptarget;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "target" ) {
                string name; sinput >> name;
                ptarget = GetEnv()->GetKinBody(name);
            }
            else {
                RAVELOG_WARNA(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERRORA(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        RobotBase::RobotStateSaver saver(_robot);
        _robot->SetActiveManipulator(_nManipIndex); BOOST_ASSERT(_robot->GetActiveManipulator()==_pmanip);
        _robot->SetActiveDOFs(_pmanip->GetArmJoints());

        boost::shared_ptr<VisibilityConstraintFunction> pconstraintfn(new VisibilityConstraintFunction(shared_problem(),ptarget));
        vector<dReal> v;
        _robot->GetActiveDOFValues(v);
        sout << pconstraintfn->Constraint(v,v,0);
        return true;
    }

    bool SampleVisibilityGoal(ostream& sout, istream& sinput)
    {
        string cmd;

        KinBodyPtr ptarget;
        int numsamples=1;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
        
            if( cmd == "target" ) {
                string name; sinput >> name;
                ptarget = GetEnv()->GetKinBody(name);
            }
            else if( cmd == "numsamples" )
                sinput >> numsamples;
            else {
                RAVELOG_WARNA(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERRORA(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        RobotBase::RobotStateSaver saver(_robot);
        _robot->SetActiveManipulator(_nManipIndex); BOOST_ASSERT(_robot->GetActiveManipulator()==_pmanip);
        _robot->SetActiveDOFs(_pmanip->GetArmJoints());

        boost::shared_ptr<GoalSampleFunction> pgoalsampler(new GoalSampleFunction(shared_problem(),ptarget,_visibilitytransforms));
     
        uint64_t starttime = GetMicroTime();
        vector<dReal> vsample;
        vector<dReal> vsamples(_robot->GetActiveDOF()*numsamples);
        int numsampled = 0;
        for(int i = 0; i < numsamples; ++i) {
            if( pgoalsampler->Sample(vsample) ) {
                std::copy(vsample.begin(), vsample.end(),vsamples.begin()+i*_robot->GetActiveDOF());
                numsampled++;
            }
        }

        if( numsampled == 0 )
            return false;
        float felapsed = (GetMicroTime()-starttime)*1e-6f;
        RAVELOG_INFO("total time for %d samples is %fs, %f avg\n", numsamples,felapsed,felapsed/numsamples);
        sout << numsampled << " ";
        for(int i = 0; i < numsampled*_robot->GetActiveDOF(); ++i)
            sout << vsamples[i] << " ";
        return true;
    }

    bool MoveToObserveTarget(ostream& sout, istream& sinput)
    {
        string strtrajfilename;
        bool bExecute = true;
        boost::shared_ptr<ostream> pOutputTrajStream;

        PlannerBase::PlannerParametersPtr params(new PlannerBase::PlannerParameters());
        params->_nMaxIterations = 4000;
        int affinedofs=0;
        KinBodyPtr ptarget;
        string cmd, plannername="BiRRT";
        dReal fSampleGoalProb = 0.001f;

        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "outputtraj" )
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,null_deleter());
            else if( cmd == "affinedofs" )
                sinput >> affinedofs;
            else if( cmd == "maxiter" )
                sinput >> params->_nMaxIterations;
            else if( cmd == "execute" )
                sinput >> bExecute;
            else if( cmd == "writetraj" )
                sinput >> strtrajfilename;
            else if( cmd == "smoothpath" )
                sinput >> params->_sPathOptimizationPlanner;
            else if( cmd == "target" ) {
                string name; sinput >> name;
                ptarget = GetEnv()->GetKinBody(name);
            }
            else if( cmd == "planner" )
                sinput >> plannername;
            else if( cmd == "sampleprob" )
                sinput >> fSampleGoalProb;
            else {
                RAVELOG_WARNA(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERRORA(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        if( !ptarget ) {
            RAVELOG_ERRORA("no target specified\n");
            return false;
        }

        RobotBase::RobotStateSaver saver(_robot);
        _robot->SetActiveManipulator(_nManipIndex); BOOST_ASSERT(_robot->GetActiveManipulator()==_pmanip);
        _robot->SetActiveDOFs(_pmanip->GetArmJoints(), affinedofs);

        boost::shared_ptr<GoalSampleFunction> pgoalsampler(new GoalSampleFunction(shared_problem(),ptarget,_visibilitytransforms));
        pgoalsampler->_fSampleGoalProb = fSampleGoalProb;
        _robot->RegrabAll();

        params->SetRobotActiveJoints(_robot);
        _robot->GetActiveDOFValues(params->vinitialconfig);

        params->_samplegoalfn = boost::bind(&GoalSampleFunction::Sample,pgoalsampler,_1);
        TrajectoryBasePtr ptraj = GetEnv()->CreateTrajectory(_robot->GetActiveDOF());

        Trajectory::TPOINT pt;
        pt.q = params->vinitialconfig;
        ptraj->AddPoint(pt);
    
        // jitter for initial collision
        if( !CM::JitterActiveDOF(_robot) ) {
            RAVELOG_WARNA("jitter failed for initial\n");
            return false;
        }
        _robot->GetActiveDOFValues(params->vinitialconfig);

        PlannerBasePtr planner = GetEnv()->CreatePlanner(plannername);
        if( !planner ) {
            RAVELOG_ERRORA("failed to create BiRRTs\n");
            return false;
        }
    
        bool bSuccess = false;
        RAVELOG_INFOA("starting planning\n");
        uint64_t starttime = GetMicroTime();
        for(int iter = 0; iter < 1; ++iter) {
            if( !planner->InitPlan(_robot, params) ) {
                RAVELOG_ERRORA("InitPlan failed\n");
                return false;
            }
            if( planner->PlanPath(ptraj) ) {
                bSuccess = true;
                RAVELOG_INFOA("finished planning\n");
                break;
            }
            else RAVELOG_WARNA("PlanPath failed\n");
        }

        float felapsed = (GetMicroTime()-starttime)*0.000001f;
        RAVELOG_INFOA("total planning time: %fs\n", felapsed);
        if( !bSuccess )
            return false;

        CM::SetActiveTrajectory(_robot, ptraj, bExecute, strtrajfilename, pOutputTrajStream,_fMaxVelMult);
        return true;
    }

    bool VisualFeedbackGrasping(ostream& sout, istream& sinput)
    {
        string strtrajfilename;
        bool bExecute = true;
        boost::shared_ptr<ostream> pOutputTrajStream;

        boost::shared_ptr<GraspSetParameters> params(new GraspSetParameters(GetEnv()));
        params->_nMaxIterations = 4000;
        KinBodyPtr ptarget;
        string cmd, plannername="GraspGradient";
        params->_fVisibiltyGraspThresh = 0.05f;
        bool bUseVisibility = false;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
        
            if( cmd == "outputtraj" )
                pOutputTrajStream = boost::shared_ptr<ostream>(&sout,null_deleter());
            else if( cmd == "maxiter" )
                sinput >> params->_nMaxIterations;
            else if( cmd == "visgraspthresh" )
                sinput >> params->_fVisibiltyGraspThresh;
            else if( cmd == "execute" )
                sinput >> bExecute;
            else if( cmd == "writetraj" )
                sinput >> strtrajfilename;
            else if( cmd == "target" ) {
                string name; sinput >> name;
                ptarget = GetEnv()->GetKinBody(name);
            }
            else if( cmd == "usevisibility" )
                sinput >> bUseVisibility;
            else if( cmd == "planner" )
                sinput >> plannername;
            else if( cmd == "graspdistthresh" )
                sinput >> params->_fGraspDistThresh;
            else if( cmd == "graspset" ) {
                int numgrasps=0;
                sinput >> numgrasps;
                params->_vgrasps.resize(numgrasps);
                TransformMatrix t;
                FOREACH(itgrasp,params->_vgrasps) {
                    sinput >> t;
                    *itgrasp = t;
                }
                RAVELOG_DEBUGA(str(boost::format("grasp set size = %d\n")%params->_vgrasps.size()));
            }
            else if( cmd == "numgradientsamples" )
                sinput >> params->_nGradientSamples;
            else {
                RAVELOG_WARNA(str(boost::format("unrecognized command: %s\n")%cmd));
                return false;
            }

            if( !sinput ) {
                RAVELOG_ERRORA(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        if( !ptarget ) {
            RAVELOG_ERRORA("no target specified\n");
            return false;
        }

        RobotBase::RobotStateSaver saver(_robot);
        _robot->SetActiveManipulator(_nManipIndex); BOOST_ASSERT(_robot->GetActiveManipulator()==_pmanip);
        _robot->SetActiveDOFs(_pmanip->GetArmJoints());
        params->SetRobotActiveJoints(_robot);

        if( bUseVisibility ) {
            RAVELOG_DEBUG("using visibility constraints\n");
            boost::shared_ptr<VisibilityConstraintFunction> pconstraint(new VisibilityConstraintFunction(shared_problem(),ptarget));
            params->_constraintfn = boost::bind(&VisibilityConstraintFunction::Constraint,pconstraint,_1,_2,_3);
        }

        params->_ptarget = ptarget;
        _robot->GetActiveDOFValues(params->vinitialconfig);

        TrajectoryBasePtr ptraj = GetEnv()->CreateTrajectory(_robot->GetActiveDOF());
        PlannerBasePtr planner = GetEnv()->CreatePlanner(plannername);
        if( !planner ) {
            RAVELOG_ERRORA("failed to create BiRRTs\n");
            return false;
        }
    
        bool bSuccess = false;
        RAVELOG_INFOA("starting planning\n");
        uint64_t starttime = GetMicroTime();
        if( !planner->InitPlan(_robot, params) ) {
            RAVELOG_ERRORA("InitPlan failed\n");
            return false;
        }
        
        if( planner->PlanPath(ptraj) ) {
            bSuccess = true;
        }
        else RAVELOG_WARNA("PlanPath failed\n");

        float felapsed = (GetMicroTime()-starttime)*0.000001f;
        RAVELOG_INFOA("total planning time: %fs\n", felapsed);
        if( !bSuccess )
            return false;

        CM::SetActiveTrajectory(_robot, ptraj, bExecute, strtrajfilename, pOutputTrajStream,_fMaxVelMult);
        return true;
    }

    bool SimulationStep(dReal fElapsedTime)
    {
        return false;
    }
    
protected:
    RobotBasePtr _robot;
    dReal _fMaxVelMult;
    RobotBase::AttachedSensorPtr _psensor;
    RobotBase::ManipulatorPtr _pmanip;
    int _nManipIndex;
    boost::shared_ptr<SensorBase::CameraGeomData> _pcamerageom;
    Transform _tcameratogripper; ///< transforms a camera coord system to the gripper coordsystem
    vector<Transform> _visibilitytransforms;
    dReal _fSampleRayDensity;

    vector<Vector> _vconvexplanes; ///< the planes defining the bounding visibility region (posive is inside)
    Vector _vcenterconvex; ///< center point on the z=1 plane of the convex region

};

#endif
