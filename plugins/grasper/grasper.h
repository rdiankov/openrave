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
        normalplane = Vector(0,0,1);
    }
    
    virtual void Destroy()
    {
        planner.reset();
    }

    virtual int main(const std::string& args)
    {
        planner = GetEnv()->CreatePlanner("Grasper");
        if( !planner ) {
            RAVELOG_WARNA("Failed to create planner\n");
            return -1;
        }

        stringstream ss(args);
        ss >> _strRobotName;

        report.reset(new COLLISIONREPORT());

        return 0;
    }

    virtual bool SendCommand(std::ostream& sout, std::istream& sinput)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        robot = GetEnv()->GetRobot(_strRobotName);

        // parse the command line for a direction (or actual body)
        Transform Tstart;
        bool randomize = false;
        string pFilename; // write results to this file
        bool bHasDirection = false;
        bool bNoise = false;
        bool bComputeDistMap = false;
        bool bUsePoints = false;
        bool bGetLinkCollisions = false;
        bool bCenterOnManipulator = false;

        bool bExecutePlanner=false;
        bool bTransRobot=true; // if false, don't move the robot
        bool bMoveOut = false; //move until just barely in collision along the specified direction

        dReal standoff=0;
        dReal fingerspread=0;
        dReal rotnoise=0;
        dReal transnoise=0;
        dReal handroll=0;
        unsigned int random_seed=0;
        Vector direction;
        Vector centeroffset; // center offset when aiming for pbody
        dReal conewidth = 45.0f;
    
        dReal mu = -1.0f;
        bool bcheck_stability = false;
        int nDistMapSamples = 60000;

        normalplane = robot->GetActiveManipulator()->GetPalmDirection();
        std::vector<GrasperProblem::SURFACEPOINT> vpoints_in;

        boost::shared_ptr<CollisionCheckerMngr> pcheckermngr;

        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "body" ) {
                string name; sinput >> name;
                pbody = GetEnv()->GetKinBody(name);
            }
            if( cmd == "bodyid" ) {
                int id = 0; sinput >> id;
                pbody = GetEnv()->GetBodyFromNetworkId(id);
            }
            else if( cmd == "direction" ) {
                sinput >> direction.x >> direction.y >> direction.z;
                bHasDirection = true;
                direction.normalize3();
            }
            else if( cmd == "direction" ) {
                sinput >> random_seed;
                randomize = true;
            }
            else if( cmd == "computedistances") {
                bComputeDistMap = true;
            }
            else if( cmd == "usepoints" ) {
                bUsePoints = true;
                int numpoints = 0;
                sinput >> numpoints;
                for(int i = 0; i < numpoints;i++) {
                    SURFACEPOINT pt;
                    sinput >> pt.pos.x >> pt.pos.y >> pt.pos.z >> pt.norm.x >> pt.norm.y >> pt.norm.z;
                    vpoints_in.push_back(pt);
                }
            }
            else if( cmd == "mapsamples" )
                sinput >> nDistMapSamples;
            else if( cmd == "file" )
                sinput >> pFilename;
            else if( cmd == "noise" ) {
                sinput >> rotnoise >> transnoise;
                bNoise = true;
            }
            else if( cmd == "roll" )
                sinput >> handroll;
            else if( cmd == "palmdir" ) {
                sinput >> normalplane.x >> normalplane.y >> normalplane.z;
                normalplane.normalize3();
            }
            else if( cmd == "centeroffset" )
                sinput >> centeroffset.x >> centeroffset.y >> centeroffset.z;
            else if( cmd == "centermanip" )
                bCenterOnManipulator = true;
            else if( cmd == "fingerspread" )
                sinput >> fingerspread;
            else if( cmd == "standoff" )
                sinput >> standoff;
            else if( cmd == "exec" )
                bExecutePlanner = true;
            else if( cmd == "notrans" )
                bTransRobot = false;
            else if( cmd == "conewidth" )
                sinput >> conewidth;
            else if( cmd == "friction" ) {
                sinput >> mu;
                bcheck_stability = true;
            }
            else if( cmd == "moveout" ) {
                //should be used along with "notrans"
                bMoveOut = true;
            }
            else if( cmd == "getlinkcollisions" )
                bGetLinkCollisions = true;
            else if( cmd == "collision" )  {
                string name; sinput >> name;
                pcheckermngr.reset(new CollisionCheckerMngr(GetEnv(), name));
            }
            else
                break;

            if( !sinput ) {
                RAVELOG_ERRORA("failed\n");
                return false;
            }
        }
    
        boost::shared_ptr<KinBody::KinBodyStateSaver> bodysaver, robotsaver;
        if( !!pbody )
            bodysaver.reset(new KinBody::KinBodyStateSaver(pbody));
    
        if(bMoveOut && bTransRobot) {
            RAVELOG_WARNA("Error: 'Move until almost out of collision' and 'translate robot' cannot both be set. If you want to move out , use 'moveout notrans'.\n");       
            return false;
        }

        if( bComputeDistMap ) {
            // move the robot out of the way
            std::vector<RobotBasePtr> robots;
            GetEnv()->GetRobots(robots);
            std::vector<Transform> Tbackup;
            std::vector<Transform> vtrans;
            for(size_t i = 0; i < robots.size(); i++) {
                robots[i]->GetBodyTransformations(vtrans);
                Tbackup.push_back(vtrans[0]);
                robots[i]->SetTransform(Transform(Vector(1,0,0,0),Vector(1000,1000,1000)));
            }
        
            //get min distance data
            vector<GrasperProblem::SURFACEPOINT> vpoints;
        
            Vector graspcenter = centeroffset;
            graspcenter += pbody->GetCenterOfMass();

            BoxSample(pbody,vpoints,nDistMapSamples,graspcenter);
            //DeterministicallySample(pbody, vpoints, 4, graspcenter);

            vector<KinBodyPtr> vbodies;
            GetEnv()->GetBodies(vbodies);
            for(size_t i = 0; i < vbodies.size(); ++i) {
                if( vbodies[i] == pbody ) {
                    vbodies.erase(vbodies.begin()+i);
                    break;
                }
            }

            //compute distance map, last argument is width of cone around each ray
            ComputeDistanceMap(vbodies, vpoints, conewidth/180.0f*PI);

            for(size_t i = 0; i < vpoints.size(); ++i) {
            
                sout << vpoints[i].dist << " " << vpoints[i].norm.x << " " << vpoints[i].norm.y << " " << vpoints[i].norm.z << " ";
                sout << vpoints[i].pos.x - graspcenter.x << " " << vpoints[i].pos.y - graspcenter.y << " " << vpoints[i].pos.z - graspcenter.z << "\n";
            }
        
            RAVELOG_DEBUGA("distance map computed\n");

            // reenable
            for(size_t i = 0; i < robots.size(); i++)
                robots[i]->SetTransform(Tbackup[i]);
        }

        if(bMoveOut) {
            if(!bHasDirection) {
                RAVELOG_ERRORA("Error: Need to specify direction to move along!\n");
                return false;
            }
        
            if(GetEnv()->CheckCollision(KinBodyConstPtr(robot),KinBodyConstPtr(pbody))) {
                //backup the robot until it is no longer colliding with the object
                Transform Ti;
                Ti = robot->GetTransform();

                //GetEnv()->plot3(Ti.trans, 1, 0, 0.004f, Vector(0,1,0) );
                dReal step_size = 0.015f;
                while(1) {
                    robot->SetTransform(Ti);
                    if(!GetEnv()->CheckCollision(KinBodyConstPtr(robot)))
                        break;
                    Ti.trans = Ti.trans + step_size*direction; //note: moves positively along direction
                }               
                //move back into collision
                step_size = step_size/15.0f;
                while(1) {
                    robot->SetTransform(Ti);
                    if(GetEnv()->CheckCollision(KinBodyConstPtr(robot),KinBodyConstPtr(pbody)))
                        break;
                    Ti.trans = Ti.trans - step_size*direction; 
                }
            }

        }

        if( !bExecutePlanner )
            return true;

        robot->Enable(true);

        Vector graspcenter = pbody->GetCenterOfMass() + centeroffset;    
        Tstart.trans = Tstart.trans + graspcenter;

        if( bTransRobot ) {
            Tstart.rot = robot->GetTransform().rot;
            robot->SetTransform(Tstart);
        }

        std::vector<Transform > vtrans;
        std::vector<KinBodyPtr> vecbodies;

        bool bremove_obstacles = false;
        if(bremove_obstacles) {
            GetEnv()->GetBodies(vecbodies);
            for(size_t i = 0; i < vecbodies.size(); i++)
                if( vecbodies[i] != pbody && vecbodies[i] != robot)
                    GetEnv()->RemoveKinBody(vecbodies[i]);
            
            pbody->SetTransform(Transform());
        }

        if( !bHasDirection )
            direction = Vector(0,0,1);

        if(randomize) {
            srand(random_seed);
            GetEnv()->GetBodies(vecbodies); 

            dReal variance;            
            do {
                srand(random_seed);
                for(int i = 0; i < (int)vecbodies.size(); i++) {
                    if( vecbodies[i] != pbody && vecbodies[i] != robot) {
                        if(i <= 8)
                            variance = 0.2f;
                        else
                            variance = 1.0f;
                        
                        Transform Trand;
                        Trand.trans.x = variance*(2*RaveRandomFloat()-1);
                        Trand.trans.y = variance*(2*RaveRandomFloat()-1);
                        Trand.trans.z = variance*(2*RaveRandomFloat()-1);
                        Trand.rot = GetRandomQuat();
                        vecbodies[i]->GetBodyTransformations(vtrans);
                        
                        Trand = vtrans[0]*Trand;
                        
                        vecbodies[i]->SetTransform(Trand);
                    }
                    
                }
            } while(GetEnv()->CheckCollision(KinBodyConstPtr(pbody)));
        }

        if( bTransRobot ) {
            // There are some complications with the 0 angle of the roll
            // In order to make things consistent even when the body rotates, have to 
            // first take the direction with respect to the identity body transformation
            Transform tBodyTrans = pbody->GetTransform();
            tBodyTrans.trans = Vector(0,0,0);
            Vector vnewdir = tBodyTrans.inverse().rotate(direction);
        
            // set the robot so it always points in the direction of the object
            Vector vup = Vector(0,1,0);
            vup = vup - vnewdir  * dot3(vnewdir, vup);
            if( vup.y < 0.01 ) {
                vup.x = 1;
                vup = vup - vnewdir * dot3(vnewdir, vup);
            }
            normalize3(vup, vup);
        
            Vector vright; cross3(vright, vup, vnewdir);
        
            TransformMatrix mtrans;
            mtrans.m[0] = vright.x;     mtrans.m[1] = vup.x;     mtrans.m[2] = vnewdir.x;
            mtrans.m[4] = vright.y;     mtrans.m[5] = vup.y;     mtrans.m[6] = vnewdir.y;
            mtrans.m[8] = vright.z;     mtrans.m[9] = vup.z;     mtrans.m[10] = vnewdir.z;
            mtrans.trans = Tstart.trans;//robot->GetLinks().front()->GetCentroid();

            if( bCenterOnManipulator ) {
                Transform t = robot->GetActiveManipulator()->GetGraspTransform().inverse();
                mtrans = mtrans * robot->GetActiveManipulator()->GetGraspTransform().inverse();
            }
        
            Transform Tfinal(mtrans);
            Transform newrot;
            newrot.rotfromaxisangle(Vector(0,0,1),handroll);

            // rotate back to world by tBodyTrans
            Tfinal = tBodyTrans*Tfinal*newrot;
        
            // set the robot so that its palm is facing normalplane
            // find the closest rotation
            Vector vrot;
            cross3(vrot, normalplane, Vector(0,0,1));

            dReal fsin = sqrtf(lengthsqr3(vrot));
            dReal fcos = dot3(normalplane, Vector(0,0,1));

            if( fsin > 0.001f ) {
                vrot /= fsin;

                Transform talign;
                talign.rotfromaxisangle(vrot, RaveAtan2(fsin, fcos));
                Tfinal = Tfinal * talign;
            }
            else if( fcos < 0 ) {
                // hand is flipped 180, rotate around x axis
                vrot = Vector(1,0,0);
                //vrot -= vnewdir * dot3(vnewdir, vrot); // not necessary?
                vrot -= normalplane * dot3(normalplane, vrot);
                normalize3(vrot, vrot);

                Transform talign;
                talign.rotfromaxisangle(vrot, RaveAtan2(fsin, fcos));
                Tfinal = Tfinal * talign;
            }

            robot->SetTransform(Tfinal);    

            //backup the robot until it is no longer colliding with the object
            Transform Ti;
            Ti.trans = graspcenter;
            //GetEnv()->plot3(Ti.trans, 1, 0, 0.004f, Vector(0,1,0) );
            Ti.rot = Tfinal.rot;
            dReal step_size = 0.05f;
            while(1) {
                robot->SetTransform(Ti);
                if(!GetEnv()->CheckCollision(KinBodyConstPtr(robot),KinBodyConstPtr(pbody)))
                    break;
                Ti.trans = Ti.trans - step_size*direction;
            }
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
                    if( sqrt(rand.lengthsqr3()) <= 1.0f)
                        break;
                }
                
                
                Trand.rotfromaxisangle(rand,RaveRandomFloat()*rotnoise);
            }
            else
                RAVELOG_WARNA("Rot Noise below threshold, no rotation noise added\n");
            
            pbody->ApplyTransform(Trand);
        }

        boost::shared_ptr<GraspParameters> params(new GraspParameters());
        params->SetRobotActiveJoints(robot);
        params->stand_off = standoff;
        params->face_target = false;
        params->roll_hand = 0;
        params->direction = direction;
        params->palmnormal = normalplane;
        robot->GetActiveDOFValues(params->vinitialconfig);
    
        if( !planner->InitPlan(robot, params) ) {
            RAVELOG_WARNA("InitPlan failed\n");
            return false;
        }

        TrajectoryBasePtr ptraj = GetEnv()->CreateTrajectory(robot->GetActiveDOF());
        if( !planner->PlanPath(ptraj) || ptraj->GetPoints().size() == 0 ) {
            RAVELOG_WARNA("PlanPath failed\n");
            return false;
        }

        if( pFilename.size() > 0 )
            ptraj->Write(pFilename, 0);

        std::vector<KinBodyConstPtr> vbodyexcluded;
        std::vector<KinBody::LinkConstPtr> vlinkexcluded;
        vbodyexcluded.push_back(pbody);

        Transform transBodyOriginal = pbody->GetTransform();

        //make sure the robot isn't colliding with anything except pbody
        robot->SetActiveDOFValues(ptraj->GetPoints().back().q);
    
        if( bNoise )
            pbody->SetTransform(transBodyOriginal);

        bool get_contacts = !GetEnv()->CheckCollision(KinBodyConstPtr(robot),vbodyexcluded,vlinkexcluded,report) && !robot->CheckSelfCollision();

        if( get_contacts ) {
            if(bcheck_stability)
                GetStableContacts(&sout, pbody,direction, mu);
            else {
                // calculate the contact normals
                GetEnv()->GetCollisionChecker()->SetCollisionOptions(CO_Contacts);
                vector<KinBody::Link*>::const_iterator itlink;
                int icont=0;
                FOREACHC(itlink, robot->GetLinks()) {
                    if( GetEnv()->CheckCollision(KinBody::LinkConstPtr(*itlink), KinBodyConstPtr(pbody), report) ) {
                        RAVELOG_DEBUGA(str(boost::format("contact %s:%s with %s:%s\n")%report->plink1->GetParent()->GetName()%report->plink1->GetName()%report->plink2->GetParent()->GetName()%report->plink2->GetName()));

                        for(size_t i = 0; i < report->contacts.size(); i++) {
                            Vector pos = report->contacts[i].pos;
                            Vector norm = report->contacts[i].norm;
                            if( report->plink1 != *itlink )
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
        else if( !!report->plink1 && !!report->plink2 ) {
            RAVELOG_WARNA(str(boost::format("collision %s:%s with %s:%s\n")%report->plink1->GetParent()->GetName()%report->plink1->GetName()%report->plink2->GetParent()->GetName()%report->plink2->GetName()));
        }

        ptraj->CalcTrajTiming(robot, ptraj->GetInterpMethod(), true, true);
        robot->SetActiveMotion(ptraj);

        return true;
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

            if( GetEnv()->CheckCollision(r, KinBodyConstPtr(pbody), report) ) {
                vpoints[i].norm = report->contacts.front().norm;
                vpoints[i].pos = report->contacts.front().pos + 0.001f * vpoints[i].norm; // extrude a little
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
            if( GetEnv()->CheckCollision(r, KinBodyConstPtr(pbody), report) ) {
                p.norm = -report->contacts.front().norm;//-r.dir//report->contacts.front().norm1;
                p.pos = report->contacts.front().pos + 0.001f * p.norm; // extrude a little
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
                
                    if( GetEnv()->CheckCollision(r, KinBodyConstPtr(pbody), report) ) {
                        p.norm = -report->contacts.front().norm;//-r.dir//report->contacts.front().norm1;
                        p.pos = report->contacts.front().pos;// + 0.001f * p.norm; // extrude a little
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
                    if( GetEnv()->CheckCollision(r, KinBodyConstPtr(vbodies[k]), report) ) {
                        if( report->minDistance < fMinDist )
                            fMinDist = report->minDistance;
                    }
                }
            }

            vpoints[i].dist = fMinDist;
        }

        GetEnv()->GetCollisionChecker()->SetCollisionOptions(0);
    }
    
    void GetStableContacts(std::ostream* colfile, KinBodyPtr ptarget,Vector& direction, dReal mu)
    {
        RAVELOG_WARNA("Starting GetStableContacts...\n");
        stringstream s; 

        bool bdraw = false;

        if(!GetEnv()->CheckCollision(KinBodyConstPtr(robot),KinBodyConstPtr(ptarget))) {
            RAVELOG_ERRORA("GrasperProblem::GetStableContacts - Error: Robot is not colliding with the target.\n");
            return;
        }

        if(mu < 0) {
            RAVELOG_ERRORA("GrasperProblem::GetStableContacts - Error: Friction coefficient is invalid.\n");
            return;
        }

        vector<dReal> closingdir(robot->GetDOF(),1);

        //make sure we get the right closing direction and don't look at irrelevant joints
        FOREACH(itmanip,robot->GetManipulators()) {
            vector<dReal>::const_iterator itclosing = (*itmanip)->GetClosingDirection().begin();
            FOREACH(itgripper,(*itmanip)->GetGripperJoints()) {
                closingdir[*itgripper] = *itclosing++;
            }
        }

        // calculate the contact normals using the Jacobian
        int numdofs = robot->GetDOF();
        std::vector<dReal> J;
        Vector deltaxyz;
        dReal temp;
        Vector vnormalpart;
        Vector vtangentpart;
    
        std::vector<KinBody::LinkPtr> vbodies = robot->GetLinks();
        for(int ilink = 0; ilink < (int)vbodies.size(); ilink++) {
            if( GetEnv()->CheckCollision(KinBody::LinkConstPtr(vbodies[ilink]),KinBodyConstPtr(ptarget), report) )  {
         
                RAVELOG_WARNA(str(boost::format("contact %s:%s with %s:%s\n")%report->plink1->GetParent()->GetName()%report->plink1->GetName()%report->plink2->GetParent()->GetName()%report->plink2->GetName()));

                Transform linkTm = vbodies[ilink]->GetTransform();
                Transform pointTm;
                int icont=0;
            
                for(size_t i = 0; i < report->contacts.size(); i++) {  
                    icont++;
                    report->contacts[i].norm = -report->contacts[i].norm;
                    //check if this link is the base link, if so there will be no Jacobian
                    if(ilink == robot->GetActiveManipulator()->GetBase()->GetIndex() )  {
                        deltaxyz = direction;
                    }
                    else {   
                        //calculate the jacobian for the contact point as if were part of the link
                        pointTm.trans = report->contacts[i].pos;
                        robot->CalculateJacobian(vbodies[ilink]->GetIndex(), pointTm.trans, J);

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
                    if (RaveAcos(dot3(report->contacts[i].norm,deltaxyz)) > PI/2.0f)
                        bstable = false;
                    else {
                        vnormalpart = dot3(report->contacts[i].norm,deltaxyz)*report->contacts[i].norm;
                        vtangentpart = deltaxyz - vnormalpart;
                        //check if tangent force is outside friction cone
                        if( mu*sqrt(lengthsqr3(vnormalpart)) < sqrt(lengthsqr3(vtangentpart)) )
                            bstable = false;
  
                    }


                    if(bdraw) {
                        if(bstable)
                            GetEnv()->plot3( RaveVector<float>(report->contacts[i].pos), 1, 0, 0.004f, RaveVector<float>(0,1,0) );
                        else
                            GetEnv()->plot3( RaveVector<float>(report->contacts[i].pos), 1, 0, 0.004f, RaveVector<float>(1,0,0) );

                        GetEnv()->plot3(RaveVector<float>(report->contacts[i].pos + 0.02f*report->contacts[i].norm), 1, 0, 0.004f, RaveVector<float>(0,0,1) );
                        GetEnv()->plot3(RaveVector<float>(report->contacts[i].pos + 0.02f*deltaxyz), 1, 0, 0.004f, RaveVector<float>(1,1,0) );
                    }
                
                    if(bstable)
                        *colfile << report->contacts[i].pos.x <<"\t" << report->contacts[i].pos.y <<"\t" << report->contacts[i].pos.z <<"\t" << report->contacts[i].norm.x <<"\t" << report->contacts[i].norm.y <<"\t" << report->contacts[i].norm.z << endl;
                }
            }
        }
    }
    
    string _strRobotName;
    PlannerBasePtr planner;
    RobotBasePtr robot;
    KinBodyPtr pbody;
    Vector normalplane; // guess direction of palm
    boost::shared_ptr<COLLISIONREPORT> report;
    boost::mutex _mutex;
};

#endif
