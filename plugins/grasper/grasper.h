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

#ifdef QHULL_FOUND

extern "C"
{
#include <qhull/qhull.h>
#include <qhull/mem.h>
#include <qhull/qset.h>
#include <qhull/geom.h>
#include <qhull/merge.h>
#include <qhull/poly.h>
#include <qhull/io.h>
#include <qhull/stat.h>
}

#endif

#define GTS_M_ICOSAHEDRON_X /* sqrt(sqrt(5)+1)/sqrt(2*sqrt(5)) */ \
  (dReal)0.850650808352039932181540497063011072240401406
#define GTS_M_ICOSAHEDRON_Y /* sqrt(2)/sqrt(5+sqrt(5))         */ \
  (dReal)0.525731112119133606025669084847876607285497935
#define GTS_M_ICOSAHEDRON_Z (dReal)0.0

// very simple interface to use the GrasperPlanner
class GrasperProblem : public ProblemInstance
{
    struct GRASPANALYSIS
    {
    GRASPANALYSIS() : mindist(0), volume(0) {}
        dReal mindist;
        dReal volume;
    };

 public:
 GrasperProblem(EnvironmentBasePtr penv)  : ProblemInstance(penv), errfile(NULL) {
        RegisterCommand("Grasp",boost::bind(&GrasperProblem::Grasp,this,_1,_2),
                        "Performs a grasp and returns contact points");
        RegisterCommand("ComputeDistanceMap",boost::bind(&GrasperProblem::ComputeDistanceMap,this,_1,_2),
                        "Computes a distance map around a particular point in space");
        RegisterCommand("GetStableContacts",boost::bind(&GrasperProblem::GetStableContacts,this,_1,_2),
                        "Returns the stable contacts as defined by the closing direction");   
    }
    virtual ~GrasperProblem() {
        if( !!errfile )
            fclose(errfile);
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

        string strRobotName;
        stringstream ss(args);
        ss >> strRobotName;

        _report.reset(new COLLISIONREPORT());
        _robot = GetEnv()->GetRobot(strRobotName);

        return 0;
    }

    virtual bool SendCommand(std::ostream& sout, std::istream& sinput)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        return ProblemInstance::SendCommand(sout,sinput);
    }

    virtual bool Grasp(std::ostream& sout, std::istream& sinput)
    {
        string strsavetraj;
        bool bHasDirection = false;
        bool bNoise = false;
        bool bGetLinkCollisions = false;
        bool bExecute = true;
        bool bComputeStableContacts = false;
        bool bComputeForceClosure = false;
        bool bOutputFinal = false;
        dReal rotnoise=0;
        dReal transnoise=0;    
        dReal friction = 0;

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

            if( cmd == "body" || cmd == "target" ) {
                string name; sinput >> name;
                params->targetbody = GetEnv()->GetKinBody(name);
                if( !params->targetbody )                    RAVELOG_WARN(str(boost::format("failed to find target %s\n")%name));
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
            else if( cmd == "avoidlink" ) {
                string linkname;
                sinput >> linkname;
                params->vavoidlinkgeometry.push_back(linkname);
            }
            else if( cmd == "notrans" )
                params->btransformrobot = false;
            else if( cmd == "transformrobot" )
                sinput >> params->btransformrobot;
            else if( cmd == "onlycontacttarget" )
                sinput >> params->bonlycontacttarget;
            else if( cmd == "tightgrasp" )
                sinput >> params->btightgrasp;
            else if( cmd == "execute" )
                sinput >> bExecute;
            else if( cmd == "writetraj" )
                sinput >> strsavetraj;
            else if( cmd == "outputfinal" )
                sinput >> bOutputFinal;
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
            else if( cmd == "friction" )
                sinput >> friction;
            else if( cmd == "getlinkcollisions" )
                bGetLinkCollisions = true;
            else if( cmd == "stablecontacts" )
                sinput >> bComputeStableContacts;
            else if( cmd == "forceclosure" )
                sinput >> bComputeForceClosure;
            else if( cmd == "collision" ) {
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
        if( !_planner->PlanPath(ptraj) || ptraj->GetPoints().size() == 0 )
            return false;

        ptraj->CalcTrajTiming(_robot, ptraj->GetInterpMethod(), true, true);
        TrajectoryBasePtr pfulltraj = GetEnv()->CreateTrajectory(_robot->GetDOF());
        _robot->GetFullTrajectoryFromActive(pfulltraj,ptraj,false);

        if( strsavetraj.size() > 0 )
            pfulltraj->Write(strsavetraj, 0);

        bodysaver.reset(); // restore target
        _robot->SetTransform(ptraj->GetPoints().back().trans);
        _robot->SetActiveDOFValues(ptraj->GetPoints().back().q);

        if( bNoise ) {
            //make sure the robot isn't colliding with anything except params->targetbody
            std::vector<KinBodyConstPtr> vbodyexcluded;
            std::vector<KinBody::LinkConstPtr> vlinkexcluded;
            vbodyexcluded.push_back(params->targetbody);
            
            if( GetEnv()->CheckCollision(KinBodyConstPtr(_robot),vbodyexcluded,vlinkexcluded,_report) || _robot->CheckSelfCollision() ) {
                if( !!_report->plink1 && !!_report->plink2 )
                    RAVELOG_WARNA(str(boost::format("collision %s:%s with %s:%s\n")%_report->plink1->GetParent()->GetName()%_report->plink1->GetName()%_report->plink2->GetParent()->GetName()%_report->plink2->GetName()));
                return false;
            }
        }

        Vector vworlddirection = !params->targetbody ? params->vtargetdirection : params->targetbody->GetTransform().rotate(params->vtargetdirection);

        vector< pair<COLLISIONREPORT::CONTACT,int> > contacts;
        if( bComputeStableContacts ) {
            _GetStableContacts(contacts, vworlddirection, friction);
        }
        else {
            // calculate the contact normals
            GetEnv()->GetCollisionChecker()->SetCollisionOptions(CO_Contacts);
            vector<KinBody::Link*>::const_iterator itlink;
            FOREACHC(itlink, _robot->GetLinks()) {
                if( GetEnv()->CheckCollision(KinBody::LinkConstPtr(*itlink), KinBodyConstPtr(params->targetbody), _report) ) {
                    RAVELOG_VERBOSEA(str(boost::format("contact %s:%s with %s:%s\n")%_report->plink1->GetParent()->GetName()%_report->plink1->GetName()%_report->plink2->GetParent()->GetName()%_report->plink2->GetName()));
                    FOREACH(itcontact,_report->contacts) {
                        if( _report->plink1 != *itlink )
                            itcontact->norm = -itcontact->norm;
                        contacts.push_back(make_pair(*itcontact,(*itlink)->GetIndex()));
                    }
                }
            }
            GetEnv()->GetCollisionChecker()->SetCollisionOptions(0);
        }

        RAVELOG_VERBOSEA(str(boost::format("number of contacts: %d\n")%contacts.size()));
        FOREACH(itcontact,contacts) {
            Vector pos = itcontact->first.pos, norm = itcontact->first.norm;
            sout << pos.x <<" " << pos.y <<" " << pos.z <<" " << norm.x <<" " << norm.y <<" " << norm.z <<" ";
            if(bGetLinkCollisions)
                sout << itcontact->second << " ";
            sout << endl;
        }

        if( bOutputFinal ) {
            sout << pfulltraj->GetPoints().back().trans << " ";
            FOREACHC(it,pfulltraj->GetPoints().back().q)
                sout << *it << " ";
        }

        GRASPANALYSIS analysis;
        if( bComputeForceClosure ) {
            try {
                vector<COLLISIONREPORT::CONTACT> c(contacts.size());
                for(size_t i = 0; i < c.size(); ++i)
                    c[i] = contacts[i].first;
                analysis = _AnalyzeContacts3D(c,friction,8);
            }
            catch(const openrave_exception& ex) {
                RAVELOG_ERROR("AnalyzeContacts3D: %s\n",ex.what());
            }
            sout << analysis.mindist << " " << analysis.volume << " ";
        }

        if( bExecute )
            _robot->SetMotion(pfulltraj);
        
        return true;
    }

    virtual bool ComputeDistanceMap(std::ostream& sout, std::istream& sinput)
    {
        dReal conewidth = 0.25f*PI;
        int nDistMapSamples = 60000;
        string cmd;
        KinBodyPtr targetbody;
        Vector vmapcenter;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "conewidth" )
                sinput >> conewidth;
            else if( cmd == "mapsamples" )
                sinput >> nDistMapSamples;
            else if( cmd == "target" ) {
                string name; sinput >> name;
                targetbody = GetEnv()->GetKinBody(name);
            }
            else if( cmd == "center" )
                sinput >> vmapcenter.x >> vmapcenter.y >> vmapcenter.z;
            else {
                RAVELOG_WARNA(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERRORA(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        RobotBase::RobotStateSaver saver1(_robot);
        KinBody::KinBodyStateSaver saver2(targetbody);
        _robot->Enable(false);
        targetbody->Enable(true);

        vector<COLLISIONREPORT::CONTACT> vpoints;        
        BoxSample(targetbody,vpoints,nDistMapSamples,vmapcenter);
        //DeterministicallySample(targetbody, vpoints, 4, vmapcenter);

        targetbody->Enable(false);
        _ComputeDistanceMap(vpoints, conewidth);
        FOREACH(itpoint, vpoints) {
            sout << itpoint->depth << " " << itpoint->norm.x << " " << itpoint->norm.y << " " << itpoint->norm.z << " ";
            sout << itpoint->pos.x - vmapcenter.x << " " << itpoint->pos.y - vmapcenter.y << " " << itpoint->pos.z - vmapcenter.z << "\n";
        }

        return true;
    }

    virtual bool GetStableContacts(std::ostream& sout, std::istream& sinput)
    {
        string cmd;
        dReal mu=0;
        Vector direction;
        bool bGetLinkCollisions = false;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "direction" )
                sinput >> direction.x >> direction.y >> direction.z;
            else if( cmd == "friction" )
                sinput >> mu;
            else if( cmd == "getlinkcollisions" )
                bGetLinkCollisions = true;
            else {
                RAVELOG_WARNA(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERRORA(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        vector< pair<COLLISIONREPORT::CONTACT,int> > contacts;
        _GetStableContacts(contacts, direction, mu);
        FOREACH(itcontact,contacts) {
            Vector pos = itcontact->first.pos, norm = itcontact->first.norm;
            sout << pos.x <<" " << pos.y <<" " << pos.z <<" " << norm.x <<" " << norm.y <<" " << norm.z <<" ";
            if(bGetLinkCollisions)
                sout << itcontact->second << " ";
            sout << endl;
        }

        return true;
    }

 protected:
    void SampleObject(KinBodyPtr pbody, vector<COLLISIONREPORT::CONTACT>& vpoints, int N, Vector graspcenter)
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
                vpoints[i].depth = 0;
                i++;
            }
        }

        GetEnv()->GetCollisionChecker()->SetCollisionOptions(0);
    }

    // generates samples across a geodesic sphere (the higher the level, the higher the number of points
    void DeterministicallySample(KinBodyPtr pbody, vector<COLLISIONREPORT::CONTACT>& vpoints, int levels, Vector graspcenter)
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
            COLLISIONREPORT::CONTACT p;
            if( GetEnv()->CheckCollision(r, KinBodyConstPtr(pbody), _report) ) {
                p.norm = -_report->contacts.front().norm;//-r.dir//_report->contacts.front().norm1;
                p.pos = _report->contacts.front().pos + 0.001f * p.norm; // extrude a little
                p.depth = 0;
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

    void BoxSample(KinBodyPtr pbody, vector<COLLISIONREPORT::CONTACT>& vpoints, int num_samples, Vector center)
    {
        RAY r;
        KinBody::Link::TRIMESH tri;
        COLLISIONREPORT::CONTACT p;
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
                        p.depth = 0;
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
    void _ComputeDistanceMap(vector<COLLISIONREPORT::CONTACT>& vpoints, dReal fTheta)
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

                if( GetEnv()->CheckCollision(r, _report) ) {
                    if( _report->minDistance < fMinDist )
                        fMinDist = _report->minDistance;
                }
            }

            vpoints[i].depth = fMinDist;
        }

        GetEnv()->GetCollisionChecker()->SetCollisionOptions(0);
    }
    
    void _GetStableContacts(vector< pair<COLLISIONREPORT::CONTACT,int> >& contacts, const Vector& direction, dReal mu)
    {
        BOOST_ASSERT(mu>0);
        RAVELOG_DEBUGA("Starting GetStableContacts...\n");

        if(!GetEnv()->CheckCollision(KinBodyConstPtr(_robot))) {
            RAVELOG_ERRORA("GrasperProblem::GetStableContacts - Error: Robot is not colliding with the target.\n");
            return;
        }

        //make sure we get the right closing direction and don't look at irrelevant joints
        vector<dReal> closingdir(_robot->GetDOF(),0);
        FOREACH(itmanip,_robot->GetManipulators()) {
            vector<dReal>::const_iterator itclosing = (*itmanip)->GetClosingDirection().begin();
            FOREACHC(itgripper,(*itmanip)->GetGripperJoints()) {
                closingdir.at(*itgripper) = *itclosing++;
            }
        }

        // calculate the contact normals using the Jacobian
        std::vector<dReal> J;    
        FOREACHC(itlink,_robot->GetLinks()) {
            if( GetEnv()->CheckCollision(KinBody::LinkConstPtr(*itlink), _report) )  {
                RAVELOG_DEBUGA(str(boost::format("contact %s:%s with %s:%s\n")%_report->plink1->GetParent()->GetName()%_report->plink1->GetName()%_report->plink2->GetParent()->GetName()%_report->plink2->GetName()));
                FOREACH(itcontact, _report->contacts) {  
                    if( _report->plink1 != *itlink )
                        itcontact->norm = -itcontact->norm;

                    Vector deltaxyz;
                    //check if this link is the base link, if so there will be no Jacobian
                    if( *itlink == _robot->GetLinks().front() || (!!_robot->GetActiveManipulator() && *itlink == _robot->GetActiveManipulator()->GetBase()) )  {
                        deltaxyz = direction;
                    }
                    else {
                        //calculate the jacobian for the contact point as if were part of the link
                        Transform pointTm;
                        pointTm.trans = itcontact->pos;
                        _robot->CalculateJacobian((*itlink)->GetIndex(), pointTm.trans, J);

                        //get the vector of delta xyz induced by a small squeeze for all joints relevant manipulator joints
                        for(int j = 0; j < 3; j++) {
                            for(int k = 0; k < _robot->GetDOF(); k++)
                                deltaxyz[j] += J.at(j*_robot->GetDOF() + k)*closingdir[k];
                        }
                    }
                
                    //if ilink is degenerate to base link (no joint between them), deltaxyz will be 0 0 0
                    //so treat it as if it were part of the base link
                    if(deltaxyz.lengthsqr3() < 1e-7f) {
                        RAVELOG_WARNA(str(boost::format("degenerate link at %s")%(*itlink)->GetName()));
                        deltaxyz = direction;
                    }
                
                    deltaxyz.normalize3();

                    if( IS_DEBUGLEVEL(Level_Debug) ) {
                        stringstream ss; 
                        ss << "link " << (*itlink)->GetIndex() << " delta XYZ: ";
                        for(int q = 0; q < 3; q++)
                            ss << deltaxyz[q] << " ";
                        ss << endl;
                        RAVELOG_DEBUGA(ss.str());
                    }

                    // determine if contact is stable (if angle is obtuse, can't be in friction cone)
                    dReal fsin2 = Vector().Cross(itcontact->norm,deltaxyz).lengthsqr3();
                    dReal fcos = dot3(itcontact->norm,deltaxyz);
                    bool bstable = fcos > 0 && fsin2 <= fcos*fcos*mu*mu;
                    if(bstable)
                        contacts.push_back(make_pair(*itcontact,(*itlink)->GetIndex()));
                }
            }
        }
    }

    virtual GRASPANALYSIS _AnalyzeContacts3D(const vector<COLLISIONREPORT::CONTACT>& contacts, dReal mu, int Nconepoints)
    {
        if( mu == 0 )
            return _AnalyzeContacts3D(contacts);

        dReal fdeltaang = 2*PI/(dReal)Nconepoints;
        dReal fang = 0;
        vector<pair<dReal,dReal> > vsincos(Nconepoints);
        FOREACH(it,vsincos) {
            it->first = RaveSin(fang);
            it->second = RaveCos(fang);
            fang += fdeltaang;
        }
        
        vector<COLLISIONREPORT::CONTACT> newcontacts;
        newcontacts.reserve(contacts.size()*Nconepoints);
        FOREACH(itcontact,contacts) {
            // find a coordinate system where z is the normal
            Vector rottodirection;
            rottodirection.Cross(Vector(0,0,1), itcontact->norm);
            dReal fsin = RaveSqrt(rottodirection.lengthsqr3());
            dReal fcos = dot3(Vector(0,0,1), itcontact->norm);
            TransformMatrix torient;
            if( fsin > 1e-6f )
                torient.rotfromaxisangle(rottodirection*(1/fsin), RaveAtan2(fsin, fcos));
            else if( fcos < 0 )
                // flipped 180, rotate around x axis
                torient.rotfromaxisangle(Vector(1,0,0), RaveAtan2(fsin, fcos));

            Vector right(torient.m[0],torient.m[4],torient.m[8]);
            Vector up(torient.m[1],torient.m[5],torient.m[9]);
            FOREACH(it,vsincos)
                newcontacts.push_back(COLLISIONREPORT::CONTACT(itcontact->pos, (itcontact->norm + mu*it->first*right + mu*it->second*up).normalize3(),0));
        }
        
        return _AnalyzeContacts3D(newcontacts);
    }

    virtual GRASPANALYSIS _AnalyzeContacts3D(const vector<COLLISIONREPORT::CONTACT>& contacts)
    {
        if( contacts.size() < 7 )
            throw openrave_exception("need at least 7 contact wrenches to have force closure in 3D");

        GRASPANALYSIS analysis;
        vector<double> vpoints(6*contacts.size()), vconvexplanes;

        vector<double>::iterator itpoint = vpoints.begin();
        FOREACH(itcontact, contacts) {
            *itpoint++ = itcontact->norm.x;
            *itpoint++ = itcontact->norm.y;
            *itpoint++ = itcontact->norm.z;
            Vector v; v.Cross(itcontact->pos,itcontact->norm);
            *itpoint++ = v.x;
            *itpoint++ = v.y;
            *itpoint++ = v.z;
        }

        analysis.volume = _ComputeConvexHull(vpoints,vconvexplanes,6);

        boost::array<double,6> vmean;
        for(size_t i = 0; i < vpoints.size(); i += 6) {
            for(int j = 0; j < 6; ++j)
                vmean[j] += vpoints[i+j];
        }
        double fipoints = 1.0f/(double)contacts.size();
        for(int j = 0; j < 6; ++j)
            vmean[j] *= fipoints;
        
        // go through each of the faces and check if center is inside, and compute its distance
        double mindist = 1e30;
        for(size_t i = 0; i < vconvexplanes.size(); i += 7) {
            double dist = -vconvexplanes.at(i+6);
            double meandist = 0;
            for(int j = 0; j < 6; ++j)
                meandist += vconvexplanes[i+j]*vmean[j];
            
            if( dist < meandist )
                dist = -dist;

            if( dist < 0 || RaveFabs(dist-meandist) < 1e-15 )
                return analysis;
            mindist = min(mindist,dist);
        }
        analysis.mindist = mindist;
        return analysis;
    }

    /// Computes the convex hull of a set of points
    /// \param vpoints a set of points each of dimension dim
    /// \param vconvexplaces the places of the convex hull, dimension is dim+1
    virtual double _ComputeConvexHull(const vector<double>& vpoints, vector<double>& vconvexplanes, int dim)
    {
        vconvexplanes.resize(0);
#ifdef QHULL_FOUND
        vector<coordT> qpoints(vpoints.size());
        std::copy(vpoints.begin(),vpoints.end(),qpoints.begin());
        
        boolT ismalloc = 0;           // True if qhull should free points in qh_freeqhull() or reallocation
        char flags[]= "qhull Tv FA"; // option flags for qhull, see qh_opt.htm, output volume (FA)

        if( !errfile )
            errfile = tmpfile();    // stderr, error messages from qhull code  
        
        int exitcode= qh_new_qhull (dim, qpoints.size()/dim, &qpoints[0], ismalloc, flags, errfile, errfile);
        if (!exitcode) {
            vconvexplanes.reserve(1000);

            facetT *facet;	          // set by FORALLfacets 
            FORALLfacets { // 'qh facet_list' contains the convex hull
//                if( facet->isarea && facet->f.area < 1e-15 ) {
//                    RAVELOG_VERBOSE(str(boost::format("skipping area: %e\n")%facet->f.area));
//                    continue;
//                }
                for(int i = 0; i < dim; ++i)
                    vconvexplanes.push_back(facet->normal[i]);
                vconvexplanes.push_back(facet->offset);
            }
        }
        
        double totvol = qh totvol;
        qh_freeqhull(!qh_ALL);
        int curlong, totlong;	  // memory remaining after qh_memfreeshort 
        qh_memfreeshort (&curlong, &totlong);
        if (curlong || totlong)
            RAVELOG_ERROR("qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);
        if( exitcode )
            throw openrave_exception(str(boost::format("Qhull failed with error %d")%exitcode));
        return totvol; // return volume
#else
        throw openrave_exception(str(boost::format("QHull library not found, cannot compute convex hull of contact points")));
        return 0;
#endif
    }

    PlannerBasePtr _planner;
    RobotBasePtr _robot;
    boost::shared_ptr<COLLISIONREPORT> _report;
    boost::mutex _mutex;
    FILE *errfile;
};

#endif
