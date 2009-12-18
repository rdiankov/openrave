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
#ifndef  GRASPER_PROBLEM_H
#define  GRASPER_PROBLEM_H

#define GTS_M_ICOSAHEDRON_X /* sqrt(sqrt(5)+1)/sqrt(2*sqrt(5)) */ \
  (dReal)0.850650808352039932181540497063011072240401406
#define GTS_M_ICOSAHEDRON_Y /* sqrt(2)/sqrt(5+sqrt(5))         */ \
  (dReal)0.525731112119133606025669084847876607285497935
#define GTS_M_ICOSAHEDRON_Z (dReal)0.0

// very simple interface to use the GrasperPlanner
class GrasperProblem : public ProblemInstance
{
 public:
 GrasperProblem(EnvironmentBasePtr penv)  : ProblemInstance(penv) {
        RegisterCommand("Grasp",boost::bind(&GrasperProblem::Grasp,this,_1,_2),
                        "Performs a grasp and returns contact points");
        RegisterCommand("ComputeDistanceMap",boost::bind(&GrasperProblem::ComputeDistanceMap,this,_1,_2),
                        "Computes a distance map around a particular point in space");
        
    }
    
    virtual void Destroy()
    {
        _planner.reset();
        _robot.reset();
    }

    virtual int main(const std::string& args)
    {
        _planner = GetEnv()->CreatePlanner("Grasper");
        if( !_planner ) {
            RAVELOG_WARNA("Failed to create planner\n");
            return -1;
        }

        stringstream ss(args);
        ss >> _strRobotName;

        _report.reset(new COLLISIONREPORT());

        return 0;
    }

    virtual bool SendCommand(std::ostream& sout, std::istream& sinput)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _robot = GetEnv()->GetRobot(_strRobotName);
        return ProblemInstance::SendCommand(sout,sinput);
    }

    virtual bool Grasp(std::ostream& sout, std::istream& sinput)
    {
        bool randomize = false;
        string strsavetraj;
        bool bHasDirection = false;
        bool bNoise = false;
        bool bGetLinkCollisions = false;
        bool bExecute = true;

        dReal rotnoise=0;
        dReal transnoise=0;
        unsigned int random_seed=0;
    
        dReal mu = -1.0f;
        bool bcheck_stability = false;

        boost::shared_ptr<GraspParameters> params(new GraspParameters(GetEnv()));
        params->btransformrobot = true;
        params->bonlycontacttarget = true;
        params->btightgrasp = false;

        boost::shared_ptr<CollisionCheckerMngr> pcheckermngr;

        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "body" ) {
                string name; sinput >> name;
                params->targetbody = GetEnv()->GetKinBody(name);
            }
            else if( cmd == "bodyid" ) {
                int id = 0; sinput >> id;
                params->targetbody = GetEnv()->GetBodyFromNetworkId(id);
            }
            else if( cmd == "direction" ) {
                sinput >> params->vtargetdirection.x >> params->vtargetdirection.y >> params->vtargetdirection.z;
                bHasDirection = true;
                params->vtargetdirection.normalize3();
            }
            else if( cmd == "randomize" ) {
                sinput >> random_seed;
                randomize = true;
            }
            else if( cmd == "avoidlink" ) {
                string linkname;
                sinput >> linkname;
                params->vavoidlinkgeometry.push_back(linkname);
            }
            else if( cmd == "notrans" )
                params->btransformrobot = false;
            else if( cmd == "execute" )
                sinput >> bExecute;
            else if( cmd == "writetraj" )
                sinput >> strsavetraj;
            else if( cmd == "noise" ) {
                sinput >> rotnoise >> transnoise;
                bNoise = true;
            }
            else if( cmd == "roll" )
                sinput >> params->ftargetroll;
            else if( cmd == "centeroffset" || cmd == "position" )
                sinput >> params->vtargetposition.x >> params->vtargetposition.y >> params->vtargetposition.z;
            else if( cmd == "standoff" )
                sinput >> params->fstandoff;
            else if( cmd == "friction" ) {
                sinput >> mu;
                bcheck_stability = true;
            }
            else if( cmd == "getlinkcollisions" )
                bGetLinkCollisions = true;
            else if( cmd == "collision" )  {
                string name; sinput >> name;
                pcheckermngr.reset(new CollisionCheckerMngr(GetEnv(), name));
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
    
        boost::shared_ptr<KinBody::KinBodyStateSaver> bodysaver;
        if( !!params->targetbody )
            bodysaver.reset(new KinBody::KinBodyStateSaver(params->targetbody));

        RobotBase::RobotStateSaver saver(_robot);
        _robot->Enable(true);

        std::vector<KinBodyPtr> vecbodies;

        bool bremove_obstacles = false;
        if(bremove_obstacles) {
            GetEnv()->GetBodies(vecbodies);
            for(size_t i = 0; i < vecbodies.size(); i++)
                if( vecbodies[i] != params->targetbody && vecbodies[i] != _robot)
                    GetEnv()->RemoveKinBody(vecbodies[i]);
            
            params->targetbody->SetTransform(Transform());
        }

        if( !bHasDirection )
            params->vtargetdirection = Vector(0,0,1);

        if(randomize) {
            srand(random_seed);
            GetEnv()->GetBodies(vecbodies); 

            dReal variance;            
            do {
                srand(random_seed);
                for(int i = 0; i < (int)vecbodies.size(); i++) {
                    if( vecbodies[i] != params->targetbody && vecbodies[i] != _robot) {
                        if(i <= 8)
                            variance = 0.2f;
                        else
                            variance = 1.0f;
                        
                        Transform Trand;
                        Trand.trans.x = variance*(2*RaveRandomFloat()-1);
                        Trand.trans.y = variance*(2*RaveRandomFloat()-1);
                        Trand.trans.z = variance*(2*RaveRandomFloat()-1);
                        Trand.rot = GetRandomQuat();
                        Trand = vecbodies[i]->GetTransform()*Trand;
                        
                        vecbodies[i]->SetTransform(Trand);
                    }
                    
                }
            } while(GetEnv()->CheckCollision(KinBodyConstPtr(params->targetbody)));
        }
    
        if(bNoise) {
            Transform Trand;
            Trand.trans.x = transnoise*(2.0f*RaveRandomFloat()-1.0f);
            Trand.trans.y = transnoise*(2.0f*RaveRandomFloat()-1.0f);
            Trand.trans.z = transnoise*(2.0f*RaveRandomFloat()-1.0f);
            if(rotnoise > 0) {
                Vector rand;
                //get random axis
                while(1) {
                    rand.x = 2.0f*RaveRandomFloat()-1.0f;
                    rand.y = 2.0f*RaveRandomFloat()-1.0f;
                    rand.z = 2.0f*RaveRandomFloat()-1.0f;
                    if( rand.lengthsqr3() > 0 )
                        break;
                }
                
                Trand.rotfromaxisangle(rand.normalize3(),RaveRandomFloat()*rotnoise);
            }
            else
                RAVELOG_WARNA("Rot Noise below threshold, no rotation noise added\n");
            
            params->targetbody->SetTransform(Trand*params->targetbody->GetTransform());
        }

        params->SetRobotActiveJoints(_robot);
        _robot->GetActiveDOFValues(params->vinitialconfig);
    
        if( !_planner->InitPlan(_robot, params) ) {
            RAVELOG_WARNA("InitPlan failed\n");
            return false;
        }

        TrajectoryBasePtr ptraj = GetEnv()->CreateTrajectory(_robot->GetActiveDOF());
        if( !_planner->PlanPath(ptraj) || ptraj->GetPoints().size() == 0 ) {
            RAVELOG_WARNA("PlanPath failed\n");
            return false;
        }

        ptraj->CalcTrajTiming(_robot, ptraj->GetInterpMethod(), true, true);
        TrajectoryBasePtr pfulltraj = GetEnv()->CreateTrajectory(_robot->GetDOF());
        _robot->GetFullTrajectoryFromActive(pfulltraj,ptraj,false);

        if( strsavetraj.size() > 0 )
            pfulltraj->Write(strsavetraj, 0);

        std::vector<KinBodyConstPtr> vbodyexcluded;
        std::vector<KinBody::LinkConstPtr> vlinkexcluded;
        vbodyexcluded.push_back(params->targetbody);

        Transform transBodyOriginal = params->targetbody->GetTransform();

        //make sure the robot isn't colliding with anything except params->targetbody
        _robot->SetTransform(ptraj->GetPoints().back().trans);
        _robot->SetActiveDOFValues(ptraj->GetPoints().back().q);
    
        if( bNoise )
            params->targetbody->SetTransform(transBodyOriginal);

        bool get_contacts = !GetEnv()->CheckCollision(KinBodyConstPtr(_robot),vbodyexcluded,vlinkexcluded,_report) && !_robot->CheckSelfCollision();

        Vector vworlddirection = !params->targetbody ? params->vtargetdirection : params->targetbody->GetTransform().rotate(params->vtargetdirection);

        if( get_contacts ) {
            if(bcheck_stability)
                GetStableContacts(&sout, vworlddirection, mu);
            else {
                // calculate the contact normals
                GetEnv()->GetCollisionChecker()->SetCollisionOptions(CO_Contacts);
                vector<KinBody::Link*>::const_iterator itlink;
                int icont=0;
                FOREACHC(itlink, _robot->GetLinks()) {
                    if( GetEnv()->CheckCollision(KinBody::LinkConstPtr(*itlink), KinBodyConstPtr(params->targetbody), _report) ) {
                        RAVELOG_DEBUGA(str(boost::format("contact %s:%s with %s:%s\n")%_report->plink1->GetParent()->GetName()%_report->plink1->GetName()%_report->plink2->GetParent()->GetName()%_report->plink2->GetName()));

                        for(size_t i = 0; i < _report->contacts.size(); i++) {
                            Vector pos = _report->contacts[i].pos;
                            Vector norm = _report->contacts[i].norm;
                            if( _report->plink1 != *itlink )
                                norm = -norm;
                            sout << pos.x <<" " << pos.y <<" " << pos.z <<" " << norm.x <<" " << norm.y <<" " << norm.z <<" ";
                            if(bGetLinkCollisions)
                                sout << (*itlink)->GetIndex();
                            sout << endl;
                            icont++;
                            //GetEnv()->plot3(pos, 1, 0);
                        }
                    }
                }
                RAVELOG_DEBUGA("number of contacts: %d\n", icont);
                GetEnv()->GetCollisionChecker()->SetCollisionOptions(0);
            }
        }
        else if( !!_report->plink1 && !!_report->plink2 ) {
            RAVELOG_WARNA(str(boost::format("collision %s:%s with %s:%s\n")%_report->plink1->GetParent()->GetName()%_report->plink1->GetName()%_report->plink2->GetParent()->GetName()%_report->plink2->GetName()));
        }

        if( bExecute )
            _robot->SetMotion(pfulltraj);

        return true;
    }

    bool ComputeDistanceMap(ostream& sout, istream& sinput)
    {
        dReal conewidth = 45.0f;
        int nDistMapSamples = 60000;
        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "conewidth" )
                sinput >> conewidth;
            else if( cmd == "mapsamples" )
                sinput >> nDistMapSamples;
            else {
                RAVELOG_WARNA(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERRORA(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

//        // move the robot out of the way
//        std::vector<RobotBasePtr> robots;
//        GetEnv()->GetRobots(robots);
//        std::vector<Transform> Tbackup;
//        for(size_t i = 0; i < robots.size(); i++) {
//            Tbackup.push_back(robots[i]->GetTransform());
//            robots[i]->SetTransform(Transform(Vector(1,0,0,0),Vector(1000,1000,1000)));
//        }
//        
//        //get min distance data
//        vector<GrasperProblem::SURFACEPOINT> vpoints;
//        
//        Vector graspcenter = params->targetbody->GetTransform()*centeroffset;
//        BoxSample(params->targetbody,vpoints,nDistMapSamples,graspcenter);
//        //DeterministicallySample(params->targetbody, vpoints, 4, graspcenter);
//
//        vector<KinBodyPtr> vbodies;
//        GetEnv()->GetBodies(vbodies);
//        for(size_t i = 0; i < vbodies.size(); ++i) {
//            if( vbodies[i] == params->targetbody ) {
//                vbodies.erase(vbodies.begin()+i);
//                break;
//            }
//        }
//
//        //compute distance map, last argument is width of cone around each ray
//        ComputeDistanceMap(vbodies, vpoints, conewidth/180.0f*PI);
//
//        for(size_t i = 0; i < vpoints.size(); ++i) {
//            
//            sout << vpoints[i].dist << " " << vpoints[i].norm.x << " " << vpoints[i].norm.y << " " << vpoints[i].norm.z << " ";
//            sout << vpoints[i].pos.x - graspcenter.x << " " << vpoints[i].pos.y - graspcenter.y << " " << vpoints[i].pos.z - graspcenter.z << "\n";
//        }
//        
//        RAVELOG_DEBUGA("distance map computed\n");
//
//        // reenable
//        for(size_t i = 0; i < robots.size(); i++)
//            robots[i]->SetTransform(Tbackup[i]);

        return false;
    }

 protected:
    struct SURFACEPOINT
    {
        SURFACEPOINT(){}
        SURFACEPOINT(dReal px, dReal py, dReal pz, dReal nx, dReal ny, dReal nz){pos.x = px; pos.y = py; pos.z = pz; norm.x = nx; norm.y = ny; norm.z = nz;}

        Vector pos, norm;
        dReal dist;
        std::vector<dReal> dists;
    };


    //    bool TriTriContact(const Vector& u1, const Vector& u2, const Vector& u3, const Vector& v1, const Vector& v2, const Vector& v3, Vector& contactpos);

    void SampleObject(KinBodyPtr pbody, vector<SURFACEPOINT>& vpoints, int N, Vector graspcenter)
    {
        RAY r;
        Vector com = graspcenter;
        GetEnv()->GetCollisionChecker()->SetCollisionOptions(CO_Contacts|CO_Distance);
        
        vpoints.resize(N);
        int i = 0;

        while(i < N) {
            r.dir.z = 2*RaveRandomFloat()-1;
            dReal R = sqrtf(1 - r.dir.x * r.dir.x);
            dReal U2 = 2 * PI * RaveRandomFloat();
            r.dir.x = R * cos(U2);
            r.dir.y = R * sin(U2);

            r.pos = com - 10.0f*r.dir;
            r.dir *= 1000;

            if( GetEnv()->CheckCollision(r, KinBodyConstPtr(pbody), _report) ) {
                vpoints[i].norm = _report->contacts.front().norm;
                vpoints[i].pos = _report->contacts.front().pos + 0.001f * vpoints[i].norm; // extrude a little
                vpoints[i].dist = 0;
                i++;
            }
        }

        GetEnv()->GetCollisionChecker()->SetCollisionOptions(0);
    }

    // generates samples across a geodesic sphere (the higher the level, the higher the number of points
    void DeterministicallySample(KinBodyPtr pbody, vector<SURFACEPOINT>& vpoints, int levels, Vector graspcenter)
    {
        RAY r;
        KinBody::Link::TRIMESH tri;
        Vector com = graspcenter;
        GenerateSphereTriangulation(tri,levels);

        GetEnv()->GetCollisionChecker()->SetCollisionOptions(CO_Contacts|CO_Distance);

        // take the mean across every tri
        vpoints.reserve(tri.indices.size()/3);
        for(int i = 0; i < (int)tri.indices.size(); i += 3) {
            r.dir = 0.33333f * (tri.vertices[tri.indices[i]] + tri.vertices[tri.indices[i+1]] + tri.vertices[tri.indices[i+2]]);
            normalize3(r.dir, r.dir);
            r.dir *= 1000;
        
            r.pos = com - 10.0f*r.dir;
            SURFACEPOINT p;
            if( GetEnv()->CheckCollision(r, KinBodyConstPtr(pbody), _report) ) {
                p.norm = -_report->contacts.front().norm;//-r.dir//_report->contacts.front().norm1;
                p.pos = _report->contacts.front().pos + 0.001f * p.norm; // extrude a little
                p.dist = 0;
                vpoints.push_back(p);
            }
        }

        GetEnv()->GetCollisionChecker()->SetCollisionOptions(0);
    }

    // generate a sphere triangulation starting with an icosahedron
    // all triangles are oriented counter clockwise
    void GenerateSphereTriangulation(KinBody::Link::TRIMESH& tri, int levels)
    {
        KinBody::Link::TRIMESH temp, temp2;

        temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_Z, +GTS_M_ICOSAHEDRON_X, -GTS_M_ICOSAHEDRON_Y));
        temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_X, +GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z));
        temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z, -GTS_M_ICOSAHEDRON_X));
        temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z, +GTS_M_ICOSAHEDRON_X));
        temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_X, -GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z));
        temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_Z, +GTS_M_ICOSAHEDRON_X, +GTS_M_ICOSAHEDRON_Y));
        temp.vertices.push_back(Vector(-GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z, +GTS_M_ICOSAHEDRON_X));
        temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_Z, -GTS_M_ICOSAHEDRON_X, -GTS_M_ICOSAHEDRON_Y));
        temp.vertices.push_back(Vector(-GTS_M_ICOSAHEDRON_X, +GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z));
        temp.vertices.push_back(Vector(-GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z, -GTS_M_ICOSAHEDRON_X));
        temp.vertices.push_back(Vector(-GTS_M_ICOSAHEDRON_X, -GTS_M_ICOSAHEDRON_Y, +GTS_M_ICOSAHEDRON_Z));
        temp.vertices.push_back(Vector(+GTS_M_ICOSAHEDRON_Z, -GTS_M_ICOSAHEDRON_X, +GTS_M_ICOSAHEDRON_Y));

        const int nindices=60;
        int indices[nindices] = {
            0, 1, 2,
            1, 3, 4,
            3, 5, 6,
            2, 4, 7,
            5, 6, 8,
            2, 7, 9,
            0, 5, 8,
            7, 9, 10,
            0, 1, 5,
            7, 10, 11,
            1, 3, 5,
            6, 10, 11,
            3, 6, 11,
            9, 10, 8,
            3, 4, 11,
            6, 8, 10,
            4, 7, 11,
            1, 2, 4,
            0, 8, 9,
            0, 2, 9
        };

        Vector v[3];
    
        // make sure oriented CCW 
        for(int i = 0; i < nindices; i += 3 ) {
            v[0] = temp.vertices[indices[i]];
            v[1] = temp.vertices[indices[i+1]];
            v[2] = temp.vertices[indices[i+2]];
            if( dot3(v[0], (v[1]-v[0]).Cross(v[2]-v[0])) < 0 )
                swap(indices[i], indices[i+1]);
        }

        temp.indices.resize(nindices);
        std::copy(&indices[0],&indices[nindices],temp.indices.begin());

        KinBody::Link::TRIMESH* pcur = &temp;
        KinBody::Link::TRIMESH* pnew = &temp2;
        while(levels-- > 0) {

            pnew->vertices.resize(0);
            pnew->vertices.reserve(2*pcur->vertices.size());
            pnew->vertices.insert(pnew->vertices.end(), pcur->vertices.begin(), pcur->vertices.end());
            pnew->indices.resize(0);
            pnew->indices.reserve(4*pcur->indices.size());

            map< uint64_t, int > mapnewinds;
            map< uint64_t, int >::iterator it;

            for(size_t i = 0; i < pcur->indices.size(); i += 3) {
                // for ever tri, create 3 new vertices and 4 new triangles.
                v[0] = pcur->vertices[pcur->indices[i]];
                v[1] = pcur->vertices[pcur->indices[i+1]];
                v[2] = pcur->vertices[pcur->indices[i+2]];

                int inds[3];
                for(int j = 0; j < 3; ++j) {
                    uint64_t key = ((uint64_t)pcur->indices[i+j]<<32)|(uint64_t)pcur->indices[i + ((j+1)%3) ];
                    it = mapnewinds.find(key);

                    if( it == mapnewinds.end() ) {
                        inds[j] = mapnewinds[key] = mapnewinds[(key<<32)|(key>>32)] = (int)pnew->vertices.size();
                        pnew->vertices.push_back((v[j]+v[(j+1)%3 ]).normalize3());
                    }
                    else {
                        inds[j] = it->second;
                    }
                }

                pnew->indices.push_back(pcur->indices[i]);    pnew->indices.push_back(inds[0]);    pnew->indices.push_back(inds[2]);
                pnew->indices.push_back(inds[0]);    pnew->indices.push_back(pcur->indices[i+1]);    pnew->indices.push_back(inds[1]);
                pnew->indices.push_back(inds[2]);    pnew->indices.push_back(inds[0]);    pnew->indices.push_back(inds[1]);
                pnew->indices.push_back(inds[2]);    pnew->indices.push_back(inds[1]);    pnew->indices.push_back(pcur->indices[i+2]);
            }

            swap(pnew,pcur);
        }

        tri = *pcur;
    }

    void BoxSample(KinBodyPtr pbody, vector<SURFACEPOINT>& vpoints, int num_samples, Vector center)
    {
        RAY r;
        KinBody::Link::TRIMESH tri;
        SURFACEPOINT p;
        dReal ffar = 1.0f;

        GetEnv()->GetCollisionChecker()->SetCollisionOptions(CO_Contacts|CO_Distance);
        vpoints.reserve(num_samples);

        dReal counter = ffar/sqrt((dReal)num_samples/12);
        for(int k = 0; k < 6; k++) {
            for(dReal i = -ffar/2.0f; i < ffar/2.0f; i+=counter) {
                for(dReal j = -ffar/2.0f; j < ffar/2.0f; j+=counter) {
                    switch(k){
                    case 0:
                        r.pos = Vector(center.x-ffar,center.y+i,center.z+j);
                        r.dir = Vector(1000,0,0);
                        break;
                    case 1:
                        r.pos = Vector(center.x+ffar,center.y+i,center.z+j);
                        r.dir = Vector(-1000,0,0);
                        break;
                    case 2:
                        r.pos = Vector(center.x+i,center.y-ffar,center.z+j);
                        r.dir = Vector(0,1000,0);
                        break;
                    case 3:
                        r.pos = Vector(center.x+i,center.y+ffar,center.z+j);
                        r.dir = Vector(0,-1000,0);
                        break;
                    case 4:
                        r.pos = Vector(center.x+i,center.y+j,center.z-ffar);
                        r.dir = Vector(0,0,1000);
                        break;
                    case 5:
                        r.pos = Vector(center.x+i,center.y+j,center.z+ffar);
                        r.dir = Vector(0,0,-1000);
                        break;
                    }
                
                    if( GetEnv()->CheckCollision(r, KinBodyConstPtr(pbody), _report) ) {
                        p.norm = -_report->contacts.front().norm;//-r.dir//_report->contacts.front().norm1;
                        p.pos = _report->contacts.front().pos;// + 0.001f * p.norm; // extrude a little
                        p.dist = 0;
                        vpoints.push_back(p);
                    }
                }
            }
        }

        GetEnv()->GetCollisionChecker()->SetCollisionOptions(0);
    }

    // computes a distance map. For every point, samples many vectors around the point's normal such that angle
    // between normal and sampled vector doesn't exceeed fTheta. Returns the minimum distance.
    // vpoints needs to already be initialized
    void ComputeDistanceMap(const vector<KinBodyPtr>& vbodies, vector<SURFACEPOINT>& vpoints, dReal fTheta)
    {
        dReal fCosTheta = cosf(fTheta);
        int N;
        if(fTheta < 0.01)
            N = 1;
    
        RAY r;

        GetEnv()->GetCollisionChecker()->SetCollisionOptions(CO_Distance);

        // set number of rays to randomly sample
        if( fTheta < 1e-2 )
            N = 1;
        else
            N = (int)ceil(fTheta * (64.0f/(PI/12.0f))); // sample 64 points when at pi/12
        for(int i = 0; i < (int)vpoints.size(); ++i) {

            Vector vright = Vector(1,0,0);
            if( fabsf(vpoints[i].norm.x) > 0.9 ) vright.y = 1;
            vright -= vpoints[i].norm * dot3(vright,vpoints[i].norm);
            normalize3(vright,vright);
            Vector vup;
            cross3(vup, vpoints[i].norm, vright);

            dReal fMinDist = 2;
            for(int j = 0; j < N; ++j) {
                // sample around a cone
                dReal fAng = fCosTheta + (1-fCosTheta)*RaveRandomFloat();
                dReal R = sqrtf(1 - fAng * fAng);
                dReal U2 = 2 * PI * RaveRandomFloat();
                r.dir = 1000.0f*(fAng * vpoints[i].norm + R * RaveCos(U2) * vright + R * RaveSin(U2) * vup);

                r.pos = vpoints[i].pos;

                for(int k = 0; k < (int)vbodies.size(); ++k) {
                    if( GetEnv()->CheckCollision(r, KinBodyConstPtr(vbodies[k]), _report) ) {
                        if( _report->minDistance < fMinDist )
                            fMinDist = _report->minDistance;
                    }
                }
            }

            vpoints[i].dist = fMinDist;
        }

        GetEnv()->GetCollisionChecker()->SetCollisionOptions(0);
    }
    
    void GetStableContacts(std::ostream* colfile, Vector& direction, dReal mu)
    {
        RAVELOG_WARNA("Starting GetStableContacts...\n");
        stringstream s; 

        bool bdraw = false;

        if(!GetEnv()->CheckCollision(KinBodyConstPtr(_robot))) {
            RAVELOG_ERRORA("GrasperProblem::GetStableContacts - Error: Robot is not colliding with the target.\n");
            return;
        }

        if(mu < 0) {
            RAVELOG_ERRORA("GrasperProblem::GetStableContacts - Error: Friction coefficient is invalid.\n");
            return;
        }

        vector<dReal> closingdir(_robot->GetDOF(),1);

        //make sure we get the right closing direction and don't look at irrelevant joints
        FOREACH(itmanip,_robot->GetManipulators()) {
            vector<dReal>::const_iterator itclosing = (*itmanip)->GetClosingDirection().begin();
            FOREACHC(itgripper,(*itmanip)->GetGripperJoints()) {
                closingdir[*itgripper] = *itclosing++;
            }
        }

        // calculate the contact normals using the Jacobian
        int numdofs = _robot->GetDOF();
        std::vector<dReal> J;
        Vector deltaxyz;
        dReal temp;
        Vector vnormalpart;
        Vector vtangentpart;
    
        std::vector<KinBody::LinkPtr> vbodies = _robot->GetLinks();
        for(int ilink = 0; ilink < (int)vbodies.size(); ilink++) {
            if( GetEnv()->CheckCollision(KinBody::LinkConstPtr(vbodies[ilink]), _report) )  {
         
                RAVELOG_WARNA(str(boost::format("contact %s:%s with %s:%s\n")%_report->plink1->GetParent()->GetName()%_report->plink1->GetName()%_report->plink2->GetParent()->GetName()%_report->plink2->GetName()));

                Transform linkTm = vbodies[ilink]->GetTransform();
                Transform pointTm;
                int icont=0;
            
                for(size_t i = 0; i < _report->contacts.size(); i++) {  
                    icont++;
                    _report->contacts[i].norm = -_report->contacts[i].norm;
                    //check if this link is the base link, if so there will be no Jacobian
                    if(ilink == _robot->GetActiveManipulator()->GetBase()->GetIndex() )  {
                        deltaxyz = direction;
                    }
                    else {   
                        //calculate the jacobian for the contact point as if were part of the link
                        pointTm.trans = _report->contacts[i].pos;
                        _robot->CalculateJacobian(vbodies[ilink]->GetIndex(), pointTm.trans, J);

                        //get the vector of delta xyz induced by a small squeeze for all joints relevant manipulator joints
                        for(int j = 0; j < 3; j++) {   
                            temp = 0;
                            for(int k = 0; k < numdofs; k++)
                                temp += J[j*numdofs + k]*0.01f*closingdir[k];
                                                    
                            if( j == 0)
                                deltaxyz.x  = temp;
                            else if( j == 1)
                                deltaxyz.y = temp;
                            else if( j == 2)
                                deltaxyz.z = temp;
                        }
                    }
                
                    //if ilink is degenerate to base link (no joint between them), deltaxyz will be 0 0 0
                    //so treat it as if it were part of the base link
                    if(lengthsqr3(deltaxyz) < 0.000000001f)
                        deltaxyz = direction;
                
                    normalize3(deltaxyz,deltaxyz);

                    s.str().erase();
                    s << "link " << ilink << " delta XYZ: ";
                    for(int q = 0; q < 3; q++)
                        s << deltaxyz[q] << " ";
                    s << endl;
                    RAVELOG_DEBUGA(s.str());
                    RAVELOG_DEBUGA("number of contacts: %d\n", icont);

                    //determine if contact is stable
                    bool bstable = true;
                    //if angle is obtuse, can't be in friction cone
                    if (RaveAcos(dot3(_report->contacts[i].norm,deltaxyz)) > PI/2.0f)
                        bstable = false;
                    else {
                        vnormalpart = dot3(_report->contacts[i].norm,deltaxyz)*_report->contacts[i].norm;
                        vtangentpart = deltaxyz - vnormalpart;
                        //check if tangent force is outside friction cone
                        if( mu*sqrt(lengthsqr3(vnormalpart)) < sqrt(lengthsqr3(vtangentpart)) )
                            bstable = false;
  
                    }


                    if(bdraw) {
                        if(bstable)
                            GetEnv()->plot3( RaveVector<float>(_report->contacts[i].pos), 1, 0, 0.004f, RaveVector<float>(0,1,0) );
                        else
                            GetEnv()->plot3( RaveVector<float>(_report->contacts[i].pos), 1, 0, 0.004f, RaveVector<float>(1,0,0) );

                        GetEnv()->plot3(RaveVector<float>(_report->contacts[i].pos + 0.02f*_report->contacts[i].norm), 1, 0, 0.004f, RaveVector<float>(0,0,1) );
                        GetEnv()->plot3(RaveVector<float>(_report->contacts[i].pos + 0.02f*deltaxyz), 1, 0, 0.004f, RaveVector<float>(1,1,0) );
                    }
                
                    if(bstable)
                        *colfile << _report->contacts[i].pos.x <<"\t" << _report->contacts[i].pos.y <<"\t" << _report->contacts[i].pos.z <<"\t" << _report->contacts[i].norm.x <<"\t" << _report->contacts[i].norm.y <<"\t" << _report->contacts[i].norm.z << endl;
                }
            }
        }
    }
    
    string _strRobotName;
    PlannerBasePtr _planner;
    RobotBasePtr _robot;
    Vector normalplane; // guess direction of palm
    boost::shared_ptr<COLLISIONREPORT> _report;
    boost::mutex _mutex;
};

#endif
