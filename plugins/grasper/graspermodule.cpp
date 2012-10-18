// -*- coding: utf-8 -*-
// Copyright (C) 2006-2010 Carnegie Mellon University (rdiankov@cs.cmu.edu)
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

#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>

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

#include <boost/thread/once.hpp>

static boost::mutex s_QhullMutex;

#define GTS_M_ICOSAHEDRON_X /* sqrt(sqrt(5)+1)/sqrt(2*sqrt(5)) */   \
    (dReal)0.850650808352039932181540497063011072240401406
#define GTS_M_ICOSAHEDRON_Y /* sqrt(2)/sqrt(5+sqrt(5))         */   \
    (dReal)0.525731112119133606025669084847876607285497935
#define GTS_M_ICOSAHEDRON_Z (dReal)0.0

template<class T1, class T2>
struct sort_pair_first {
    bool operator()(const std::pair<T1,T2>&left, const std::pair<T1,T2>&right) {
        return left.first < right.first;
    }
};

// very simple interface to use the GrasperPlanner
class GrasperModule : public ModuleBase
{
    struct GRASPANALYSIS
    {
        GRASPANALYSIS() : mindist(0), volume(0) {
        }
        dReal mindist;
        dReal volume;
    };

public:
    GrasperModule(EnvironmentBasePtr penv, std::istream& sinput)  : ModuleBase(penv), errfile(NULL) {
        __description = ":Interface Author: Rosen Diankov\n\nUsed to simulate a hand grasping an object by closing its fingers until collision with all links. ";
        RegisterCommand("Grasp",boost::bind(&GrasperModule::_GraspCommand,this,_1,_2),
                        "Performs a grasp and returns contact points");
        RegisterCommand("GraspThreaded",boost::bind(&GrasperModule::_GraspThreadedCommand,this,_1,_2),
                        "Parllelizes the computation of the grasp planning and force closure. Number of threads can be specified with 'numthreads'.");
        RegisterCommand("ComputeDistanceMap",boost::bind(&GrasperModule::_ComputeDistanceMapCommand,this,_1,_2),
                        "Computes a distance map around a particular point in space");
        RegisterCommand("GetStableContacts",boost::bind(&GrasperModule::_GetStableContactsCommand,this,_1,_2),
                        "Returns the stable contacts as defined by the closing direction");
        RegisterCommand("ConvexHull",boost::bind(&GrasperModule::_ConvexHullCommand,this,_1,_2),
                        "Given a point cloud, returns information about its convex hull like normal planes, vertex indices, and triangle indices. Computed planes point outside the mesh, face indices are not ordered, triangles point outside the mesh (counter-clockwise)");
    }
    virtual ~GrasperModule() {
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
        string strRobotName;
        stringstream ss(args);
        ss >> strRobotName;

        _report.reset(new CollisionReport());
        _robot = GetEnv()->GetRobot(strRobotName);

        string plannername = "Grasper";
        string cmd;
        while(!ss.eof()) {
            ss >> cmd;
            if( !ss ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "planner" ) {
                ss >> plannername;
            }
            if( ss.fail() || !ss ) {
                break;
            }
        }

        _planner = RaveCreatePlanner(GetEnv(),plannername);
        if( !_planner ) {
            RAVELOG_WARN("Failed to create planner\n");
            return -1;
        }

        _ComputeJointMaxLengths(_vjointmaxlengths);
        return 0;
    }

    virtual bool _GraspCommand(std::ostream& sout, std::istream& sinput)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());

        string strsavetraj;
        bool bGetLinkCollisions = false;
        bool bExecute = true;
        bool bComputeStableContacts = false;
        bool bComputeForceClosure = false;
        bool bOutputFinal = false;
        dReal friction = 0;

        GraspParametersPtr params(new GraspParameters(GetEnv()));
        params->btransformrobot = true;
        params->bonlycontacttarget = true;
        params->btightgrasp = false;
        params->vtargetdirection = Vector(0,0,1);
        params->vmanipulatordirection =  _robot->GetActiveManipulator()->GetDirection();
        boost::shared_ptr<CollisionCheckerMngr> pcheckermngr;

        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if(( cmd == "body") ||( cmd == "target") ) {
                // initialization
                string name; sinput >> name;
                params->targetbody = GetEnv()->GetKinBody(name);
                if( !params->targetbody ) {
                    RAVELOG_WARN(str(boost::format("failed to find target %s\n")%name));
                }
            }
            else if( cmd == "bodyid" ) {
                // initialization
                int id = 0; sinput >> id;
                params->targetbody = GetEnv()->GetBodyFromEnvironmentId(id);
            }
            else if( cmd == "direction" ) {
                // grasp
                sinput >> params->vtargetdirection.x >> params->vtargetdirection.y >> params->vtargetdirection.z;
                params->vtargetdirection.normalize3();
            }
            else if( cmd == "avoidlink" ) {
                // initialization
                string linkname;
                sinput >> linkname;
                params->vavoidlinkgeometry.push_back(linkname);
            }
            else if( cmd == "notrans" ) {
                // initialization
                params->btransformrobot = false;
            }
            else if( cmd == "transformrobot" ) {
                // initialization
                sinput >> params->btransformrobot;
            }
            else if( cmd == "onlycontacttarget" ) {
                // initialization
                sinput >> params->bonlycontacttarget;
            }
            else if( cmd == "tightgrasp" ) {
                // initialization
                sinput >> params->btightgrasp;
            }
            else if( cmd == "execute" ) {
                // ignore
                sinput >> bExecute;
            }
            else if( cmd == "writetraj" ) {
                // ignore
                sinput >> strsavetraj;
            }
            else if( cmd == "outputfinal" ) {
                // ignore
                sinput >> bOutputFinal;
            }
            else if( cmd == "graspingnoise" ) {
                // initialization
                sinput >> params->fgraspingnoise;
            }
            else if( cmd == "roll" ) {
                // grasp
                sinput >> params->ftargetroll;
            }
            else if( cmd == "centeroffset" || cmd == "position" ) {
                // initialization
                sinput >> params->vtargetposition.x >> params->vtargetposition.y >> params->vtargetposition.z;
            }
            else if(cmd == "manipulatordirection") {
                // initialization
                sinput >> params->vmanipulatordirection.x >> params->vmanipulatordirection.y >> params->vmanipulatordirection.z;
            }
            else if( cmd == "standoff" ) {
                // grasp
                sinput >> params->fstandoff;
            }
            else if( cmd == "friction" ) {
                // initialization
                sinput >> friction;
            }
            else if( cmd == "getlinkcollisions" ) {
                // ignore
                bGetLinkCollisions = true;
            }
            else if( cmd == "stablecontacts" ) {
                // ignore
                sinput >> bComputeStableContacts;
            }
            else if( cmd == "forceclosure" ) {
                // initialization
                sinput >> bComputeForceClosure;
            }
            else if( cmd == "collision" ) {
                // initialiation
                string name; sinput >> name;
                pcheckermngr.reset(new CollisionCheckerMngr(GetEnv(), name));
            }
            else if( cmd == "translationstepmult" ) {
                // initialization
                sinput >> params->ftranslationstepmult;
            }
            else if( cmd == "finestep" ) {
                sinput >> params->ffinestep;
            }
            else {
                RAVELOG_WARN(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERROR(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        RobotBase::RobotStateSaver saver(_robot);
        _robot->Enable(true);

        params->SetRobotActiveJoints(_robot);
        _robot->GetActiveDOFValues(params->vinitialconfig);

        if( !_planner->InitPlan(_robot, params) ) {
            RAVELOG_WARN("InitPlan failed\n");
            return false;
        }

        TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),"");
        if( !_planner->PlanPath(ptraj) || ptraj->GetNumWaypoints() == 0 ) {
            return false;
        }

        if( strsavetraj.size() > 0 ) {
            ofstream f(strsavetraj.c_str());
            ptraj->serialize(f);
        }

        BOOST_ASSERT(ptraj->GetNumWaypoints()>0);
        vector<dReal> vdata;
        ptraj->GetWaypoint(-1,vdata,_robot->GetConfigurationSpecification());
        _robot->SetConfigurationValues(vdata.begin(),true);

        vector< pair<CollisionReport::CONTACT,int> > contacts;
        if( bComputeStableContacts ) {
            Vector vworlddirection = !params->targetbody ? params->vtargetdirection : params->targetbody->GetTransform().rotate(params->vtargetdirection);
            _GetStableContacts(contacts, vworlddirection, friction);
        }
        else {
            // calculate the contact normals
            GetEnv()->GetCollisionChecker()->SetCollisionOptions(CO_Contacts);
            std::vector<KinBody::LinkPtr> vlinks;
            _robot->GetActiveManipulator()->GetChildLinks(vlinks);
            FOREACHC(itlink, vlinks) {
                if( GetEnv()->CheckCollision(KinBody::LinkConstPtr(*itlink), KinBodyConstPtr(params->targetbody), _report) ) {
                    RAVELOG_VERBOSE(str(boost::format("contact %s\n")%_report->__str__()));
                    FOREACH(itcontact,_report->contacts) {
                        if( _report->plink1 != *itlink ) {
                            itcontact->norm = -itcontact->norm;
                            itcontact->depth = -itcontact->depth;
                        }
                        contacts.push_back(make_pair(*itcontact,(*itlink)->GetIndex()));
                    }
                }
            }
            GetEnv()->GetCollisionChecker()->SetCollisionOptions(0);
        }

        RAVELOG_VERBOSE(str(boost::format("number of contacts: %d\n")%contacts.size()));
        FOREACH(itcontact,contacts) {
            Vector norm = itcontact->first.norm;
            Vector pos = itcontact->first.pos;    //-norm*itcontact->first.depth; //?
            sout << pos.x <<" " << pos.y <<" " << pos.z <<" " << norm.x <<" " << norm.y <<" " << norm.z <<" ";
            if(bGetLinkCollisions) {
                sout << itcontact->second << " ";
            }
            sout << endl;
        }

        if( bOutputFinal ) {
            BOOST_ASSERT(ptraj->GetNumWaypoints()>0);
            vector<dReal> vtrajpoint, q;
            ptraj->GetWaypoint(-1,vtrajpoint,_robot->GetConfigurationSpecification());
            _robot->SetConfigurationValues(vtrajpoint.begin(),true);
            sout << _robot->GetTransform() << " ";
            _robot->GetDOFValues(q);
            FOREACHC(it,q) {
                sout << *it << " ";
            }
        }

        GRASPANALYSIS analysis;
        if( bComputeForceClosure ) {
            try {
                vector<CollisionReport::CONTACT> c(contacts.size());
                for(size_t i = 0; i < c.size(); ++i) {
                    c[i] = contacts[i].first;
                }
                analysis = _AnalyzeContacts3D(c,friction,8);
            }
            catch(const std::exception& ex) {
                RAVELOG_WARN("AnalyzeContacts3D: %s\n",ex.what());
            }
            sout << analysis.mindist << " " << analysis.volume << " ";
        }

        if( bExecute && !!_robot->GetController() ) {
            _robot->GetController()->SetPath(ptraj);
        }
        return true;
    }

    virtual bool _ComputeDistanceMapCommand(std::ostream& sout, std::istream& sinput)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());

        dReal conewidth = 0.25f*PI;
        int nDistMapSamples = 60000;
        string cmd;
        KinBodyPtr targetbody;
        Vector vmapcenter;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "conewidth" ) {
                sinput >> conewidth;
            }
            else if( cmd == "mapsamples" ) {
                sinput >> nDistMapSamples;
            }
            else if( cmd == "target" ) {
                string name; sinput >> name;
                targetbody = GetEnv()->GetKinBody(name);
            }
            else if( cmd == "center" )
                sinput >> vmapcenter.x >> vmapcenter.y >> vmapcenter.z;
            else {
                RAVELOG_WARN(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERROR(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        RobotBase::RobotStateSaver saver1(_robot);
        KinBody::KinBodyStateSaver saver2(targetbody);
        _robot->Enable(false);
        targetbody->Enable(true);

        vector<CollisionReport::CONTACT> vpoints;
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

    virtual bool _GetStableContactsCommand(std::ostream& sout, std::istream& sinput)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());

        string cmd;
        dReal mu=0;
        Vector direction;
        bool bGetLinkCollisions = false;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "direction" ) {
                sinput >> direction.x >> direction.y >> direction.z;
            }
            else if( cmd == "friction" ) {
                sinput >> mu;
            }
            else if( cmd == "getlinkcollisions" ) {
                bGetLinkCollisions = true;
            }
            else {
                RAVELOG_WARN(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERROR(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        vector< pair<CollisionReport::CONTACT,int> > contacts;
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

    virtual bool _ConvexHullCommand(std::ostream& sout, std::istream& sinput)
    {
        string cmd;
        bool bReturnFaces = true, bReturnPlanes = true, bReturnTriangles = true;
        int dim=0;
        vector<double> vpoints;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "points" ) {
                int N=0;
                sinput >> N >> dim;
                vpoints.resize(N*dim);
                for(int i = 0; i < N*dim; ++i) {
                    sinput >> vpoints[i];
                }
            }
            else if( cmd == "returnplanes" ) {
                sinput >> bReturnPlanes;
            }
            else if( cmd == "returnfaces" ) {
                sinput >> bReturnFaces;
            }
            else if( cmd == "returntriangles" ) {
                sinput >> bReturnTriangles;
            }
            else {
                RAVELOG_WARN(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERROR(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        vector<double> vconvexplanes;
        boost::shared_ptr< vector<int> > vconvexfaces;
        if( bReturnFaces || bReturnTriangles ) {
            vconvexfaces.reset(new vector<int>);
        }
        if( _ComputeConvexHull(vpoints,vconvexplanes, vconvexfaces, dim) == 0 ) {
            return false;
        }
        if( bReturnPlanes ) {
            sout << vconvexplanes.size()/(dim+1) << " ";
            FOREACH(it,vconvexplanes) {
                sout << *it << " ";
            }
        }
        if( bReturnFaces ) {
            FOREACH(it,*vconvexfaces) {
                sout << *it << " ";
            }
        }
        if( bReturnTriangles ) {
            if( dim != 3 ) {
                RAVELOG_WARN(str(boost::format("cannot triangulate convex hulls of dimension %d\n")%dim));
                return false;
            }
            size_t faceindex = 1;
            int numtriangles = 0;
            while(faceindex < vconvexfaces->size()) {
                numtriangles += vconvexfaces->at(faceindex)-2;
                faceindex += vconvexfaces->at(faceindex)+1;
            }
            sout << numtriangles << " ";
            faceindex = 1;
            size_t planeindex = 0;
            vector<double> meanpoint(dim,0), point0(dim,0), point1(dim,0);
            vector<pair<double,int> > angles;
            while(faceindex < vconvexfaces->size()) {
                // have to first sort the vertices of the face before triangulating them
                // point* = point-mean
                // atan2(plane^T * (point0* x point1*), point0*^T * point1*) = angle <- sort
                int numpoints = vconvexfaces->at(faceindex);
                for(int j = 0; j < dim; ++j) {
                    meanpoint[j] = 0;
                    point0[j] = 0;
                    point1[j] = 0;
                }
                for(int i = 0; i < numpoints; ++i) {
                    int pointindex = vconvexfaces->at(faceindex+i+1);
                    for(int j = 0; j < dim; ++j) {
                        meanpoint[j] += vpoints[pointindex*dim+j];
                    }
                }
                int pointindex0 = vconvexfaces->at(faceindex+1);
                for(int j = 0; j < dim; ++j) {
                    meanpoint[j] /= numpoints;
                    point0[j] = vpoints[pointindex0*dim+j] - meanpoint[j];
                }
                angles.resize(numpoints); angles[0].first = 0; angles[0].second = 0;
                for(int i = 1; i < numpoints; ++i) {
                    int pointindex = vconvexfaces->at(faceindex+i+1);
                    for(int j = 0; j < dim; ++j) {
                        point1[j] = vpoints[pointindex*dim+j] - meanpoint[j];
                    }
                    dReal sinang = vconvexplanes[planeindex+0] * (point0[1]*point1[2] - point0[2]*point1[1]) + vconvexplanes[planeindex+1] * (point0[2]*point1[0] - point0[0]*point1[2]) + vconvexplanes[planeindex+2] * (point0[0]*point1[1] - point0[1]*point1[0]);
                    dReal cosang = point0[0]*point1[0] + point0[1]*point1[1] + point0[2]*point1[2];
                    angles[i].first = RaveAtan2(sinang,cosang);
                    if( angles[i].first < 0 ) {
                        angles[i].first += 2*PI;
                    }
                    angles[i].second = i;
                }
                sort(angles.begin(),angles.end(),sort_pair_first<double,int>());
                for(size_t i = 2; i < angles.size(); ++i) {
                    sout << vconvexfaces->at(faceindex+1+angles[0].second) << " " << vconvexfaces->at(faceindex+1+angles[i-1].second) << " " << vconvexfaces->at(faceindex+1+angles[i].second) << " ";
                }
                faceindex += numpoints+1;
                planeindex += dim+1;
            }
        }
        return true;
    }

    // initialization parameters
    struct WorkerParameters
    {
        WorkerParameters() {
            bonlycontacttarget = true;
            btightgrasp = false;
            fgraspingnoise = 0;
            bComputeForceClosure = false;
            friction = 0.4;
            ftranslationstepmult = 0.1;
            nGraspingNoiseRetries = 0;
            forceclosurethreshold = 0;
            ffinestep = 0.001f;
            bCheckGraspIK = false;
        }

        string targetname;
        vector<string> vavoidlinkgeometry;
        bool bonlycontacttarget;
        bool btightgrasp;
        int nGraspingNoiseRetries;
        dReal fgraspingnoise;
        bool bComputeForceClosure;
        dReal forceclosurethreshold;
        dReal friction;
        string collisionchecker;
        dReal ftranslationstepmult;
        dReal ffinestep;

        string manipname;
        vector<int> vactiveindices;
        int affinedofs;
        Vector affineaxis;

        bool bCheckGraspIK;
    };

    struct GraspParametersThread
    {
        size_t id;
        Vector vtargetdirection;
        Vector vtargetposition;
        Vector vmanipulatordirection;
        dReal ftargetroll;
        vector<dReal> preshape;
        dReal fstandoff;

        // results
        // contact points
        vector< pair<CollisionReport::CONTACT,int> > contacts;
        dReal mindist, volume;
        Transform transfinal;
        vector<dReal> finalshape;
    };
    typedef boost::shared_ptr<GraspParametersThread> GraspParametersThreadPtr;
    typedef boost::shared_ptr<WorkerParameters> WorkerParametersPtr;

    virtual bool _GraspThreadedCommand(std::ostream& sout, std::istream& sinput)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());

        WorkerParametersPtr worker_params(new WorkerParameters());
        int numthreads = 2;
        string cmd;
        vector< pair<Vector, Vector> > approachrays;
        vector<dReal> rolls;
        vector< vector<dReal> > preshapes;
        vector<Vector> manipulatordirections;
        vector<dReal> standoffs;
        size_t startindex = 0;
        size_t maxgrasps = 0;

        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "target" ) {
                sinput >> worker_params->targetname;
            }
            else if( cmd == "avoidlink" ) {
                string linkname;
                sinput >> linkname;
                worker_params->vavoidlinkgeometry.push_back(linkname);
            }
            else if( cmd == "startindex" ) {
                sinput >> startindex;
            }
            else if( cmd == "maxgrasps" ) {
                sinput >> maxgrasps;
            }
            else if( cmd == "onlycontacttarget" ) {
                sinput >> worker_params->bonlycontacttarget;
            }
            else if( cmd == "tightgrasp" ) {
                sinput >> worker_params->btightgrasp;
            }
            else if( cmd == "graspingnoise" ) {
                sinput >> worker_params->fgraspingnoise >> worker_params->nGraspingNoiseRetries;
            }
            else if( cmd == "friction" ) {
                sinput >> worker_params->friction;
            }
            else if( cmd == "forceclosure" ) {
                sinput >> worker_params->bComputeForceClosure >> worker_params->forceclosurethreshold;
            }
            else if( cmd == "collisionchecker" ) {
                sinput >> worker_params->collisionchecker;
            }
            else if( cmd == "translationstepmult" ) {
                sinput >> worker_params->ftranslationstepmult;
            }
            else if( cmd == "finestep" ) {
                sinput >> worker_params->ffinestep;
            }
            else if( cmd == "numthreads" ) {
                sinput >> numthreads;
            }
            // grasp specific
            else if( cmd == "approachrays" ) {
                int numapproachrays = 0;
                sinput >> numapproachrays;
                approachrays.resize(numapproachrays);
                FOREACH(it,approachrays) {
                    sinput >> it->first.x >> it->first.y >> it->first.z;
                    sinput >> it->second.x >> it->second.y >> it->second.z;
                }
            }
            else if( cmd == "rolls" ) {
                int numrolls=0;
                sinput >> numrolls;
                rolls.resize(numrolls);
                FOREACH(it,rolls) {
                    sinput >> *it;
                }
            }
            else if( cmd == "standoffs" ) {
                int numstandoffs = 0;
                sinput >> numstandoffs;
                standoffs.resize(numstandoffs);
                FOREACH(it,standoffs) {
                    sinput >> *it;
                }
            }
            else if( cmd == "preshapes" ) {
                int numpreshapes = 0;
                sinput >> numpreshapes;
                preshapes.resize(numpreshapes);
                FOREACH(itpreshape,preshapes) {
                    itpreshape->resize(_robot->GetActiveManipulator()->GetGripperIndices().size());
                    FOREACH(it,*itpreshape) {
                        sinput >> *it;
                    }
                }
            }
            else if( cmd == "manipulatordirections" ) {
                int nummanipulatordirections = 0;
                sinput >> nummanipulatordirections;
                manipulatordirections.resize(nummanipulatordirections);
                FOREACH(it,manipulatordirections) {
                    sinput >> it->x >> it->y >> it->z;
                }
            }
            else if( cmd == "checkik" ) {
                sinput >> worker_params->bCheckGraspIK;
            }
            else {
                RAVELOG_WARN(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERROR(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        worker_params->manipname = _robot->GetActiveManipulator()->GetName();
        worker_params->vactiveindices = _robot->GetActiveDOFIndices();
        worker_params->affinedofs = _robot->GetAffineDOF();
        worker_params->affineaxis = _robot->GetAffineRotationAxis();

        EnvironmentBasePtr pcloneenv = GetEnv()->CloneSelf(Clone_Bodies|Clone_Simulation);

        _bContinueWorker = true;
        // start worker threads
        vector<boost::shared_ptr<boost::thread> > listthreads(numthreads);
        FOREACH(itthread,listthreads) {
            itthread->reset(new boost::thread(boost::bind(&GrasperModule::_WorkerThread,this,worker_params,pcloneenv)));
        }

        _listGraspResults.clear();
        size_t numgrasps = approachrays.size()*rolls.size()*preshapes.size()*standoffs.size()*manipulatordirections.size();
        if( maxgrasps == 0 ) {
            maxgrasps = numgrasps;
        }
        RAVELOG_INFO(str(boost::format("number of grasps to test: %d\n")%numgrasps));
        size_t id = startindex;
        for(id=startindex; id < numgrasps; ++id) {
            size_t istandoff = id % standoffs.size();
            size_t ipreshape = (id / standoffs.size()) % preshapes.size();
            size_t iroll = (id / (preshapes.size() * standoffs.size())) % rolls.size();
            size_t iapproachray = (id / (rolls.size() * preshapes.size() * standoffs.size()))%approachrays.size();
            size_t imanipulatordirection = (id / (rolls.size() * preshapes.size() * standoffs.size()*approachrays.size()));

            boost::mutex::scoped_lock lock(_mutexGrasp);
            if( _listGraspResults.size() >= maxgrasps ) {
                break;
            }

            // initialize _graspParamsWork
            BOOST_ASSERT(!_graspParamsWork);
            _graspParamsWork.reset(new GraspParametersThread());
            _graspParamsWork->id = id;
            _graspParamsWork->vtargetposition = approachrays.at(iapproachray).first;
            _graspParamsWork->vtargetdirection = approachrays.at(iapproachray).second;
            _graspParamsWork->vmanipulatordirection = manipulatordirections.at(imanipulatordirection);
            _graspParamsWork->ftargetroll = rolls.at(iroll);
            _graspParamsWork->fstandoff = standoffs.at(istandoff);
            _graspParamsWork->preshape = preshapes.at(ipreshape);
            _condGraspHasWork.notify_one();     // notify there is work
            _condGraspReceivedWork.wait(lock);     // wait for more work
        }

        // wait for workers
        _bContinueWorker = false;
        FOREACH(itthread,listthreads) {
            _condGraspHasWork.notify_all();     // signal
            (*itthread)->join();
        }
        listthreads.clear();

        // parse results to output
        sout << id << " " << _listGraspResults.size() << " ";
        FOREACH(itresult, _listGraspResults) {
            sout << (*itresult)->vtargetposition.x << " " << (*itresult)->vtargetposition.y << " " << (*itresult)->vtargetposition.z << " ";
            sout << (*itresult)->vtargetdirection.x << " " << (*itresult)->vtargetdirection.y << " " << (*itresult)->vtargetdirection.z << " ";
            sout << (*itresult)->ftargetroll << " " << (*itresult)->fstandoff << " ";
            sout << (*itresult)->vmanipulatordirection.x << " " << (*itresult)->vmanipulatordirection.y << " " << (*itresult)->vmanipulatordirection.z << " ";
            sout << (*itresult)->mindist << " " << (*itresult)->volume << " ";
            FOREACH(itangle, (*itresult)->preshape) {
                sout << (*itangle) << " ";
            }
            sout << (*itresult)->transfinal.rot.x << " " << (*itresult)->transfinal.rot.y << " " << (*itresult)->transfinal.rot.z << " " << (*itresult)->transfinal.rot.w << " " << (*itresult)->transfinal.trans.x << " " << (*itresult)->transfinal.trans.y << " " << (*itresult)->transfinal.trans.z << " ";
            FOREACH(itangle, (*itresult)->finalshape) {
                sout << *itangle << " ";
            }
            sout << (*itresult)->contacts.size() << " ";
            FOREACH(itc, (*itresult)->contacts) {
                const CollisionReport::CONTACT& c = itc->first;
                sout << c.pos.x << " " << c.pos.y << " " << c.pos.z << " " << c.norm.x << " " << c.norm.y << " " << c.norm.z << " ";
            }
        }
        return true;
    }

    void _WorkerThread(const WorkerParametersPtr worker_params, EnvironmentBasePtr penv)
    {
        // clone environment
        EnvironmentBasePtr pcloneenv = penv->CloneSelf(Clone_Bodies|Clone_Simulation);
        {
            EnvironmentMutex::scoped_lock lock(pcloneenv->GetMutex());
            boost::shared_ptr<CollisionCheckerMngr> pcheckermngr(new CollisionCheckerMngr(pcloneenv, worker_params->collisionchecker));
            PlannerBasePtr planner = RaveCreatePlanner(pcloneenv,"Grasper");
            RobotBasePtr probot = pcloneenv->GetRobot(_robot->GetName());
            string strsavetraj;

            probot->SetActiveManipulator(worker_params->manipname);

            // setup parameters
            GraspParametersPtr params(new GraspParameters(pcloneenv));
            params->targetbody = pcloneenv->GetKinBody(worker_params->targetname);
            params->vavoidlinkgeometry = worker_params->vavoidlinkgeometry;
            params->btransformrobot = true;
            params->bonlycontacttarget = worker_params->bonlycontacttarget;
            params->btightgrasp = worker_params->btightgrasp;
            params->fgraspingnoise = 0;
            params->ftranslationstepmult = worker_params->ftranslationstepmult;

            CollisionReportPtr report(new CollisionReport());
            TrajectoryBasePtr ptraj = RaveCreateTrajectory(pcloneenv,"");
            GraspParametersThreadPtr grasp_params;

            // calculate the contact normals
            std::vector<KinBody::LinkPtr> vlinks, vindependentlinks;
            probot->GetActiveManipulator()->GetChildLinks(vlinks);
            probot->GetActiveManipulator()->GetIndependentLinks(vindependentlinks);
            Transform trobotstart = probot->GetTransform();

            vector<dReal> vtrajpoint;

            // use CO_ActiveDOFs since might be calling FindIKSolution
            int coloptions = GetEnv()->GetCollisionChecker()->GetCollisionOptions()|(worker_params->bCheckGraspIK ? CO_ActiveDOFs : 0);
            coloptions &= ~CO_Contacts;
            pcloneenv->GetCollisionChecker()->SetCollisionOptions(coloptions|CO_Contacts);

            while(_bContinueWorker) {
                {
                    // wait for work
                    boost::mutex::scoped_lock lock(_mutexGrasp);
                    if( !_graspParamsWork ) {
                        _condGraspHasWork.wait(lock);
                        // after signal
                        if( !_graspParamsWork ) {
                            continue;
                        }
                    }
                    grasp_params = _graspParamsWork;
                    _graspParamsWork.reset();
                    _condGraspReceivedWork.notify_all();
                }

                RAVELOG_DEBUG(str(boost::format("grasp %d: start")%grasp_params->id));

                // fill params
                params->vtargetdirection = grasp_params->vtargetdirection;
                params->ftargetroll = grasp_params->ftargetroll;
                params->vtargetposition = grasp_params->vtargetposition;
                params->vmanipulatordirection = grasp_params->vmanipulatordirection;
                params->fstandoff = grasp_params->fstandoff;
                probot->SetActiveDOFs(worker_params->vactiveindices);
                probot->SetActiveDOFValues(grasp_params->preshape);
                probot->SetActiveDOFs(worker_params->vactiveindices,worker_params->affinedofs,worker_params->affineaxis);
                params->SetRobotActiveJoints(probot);

                RobotBase::RobotStateSaver saver(probot);
                probot->Enable(true);

                params->fgraspingnoise = 0;
                ptraj->Init(probot->GetActiveConfigurationSpecification());

                // InitPlan/PlanPath
                if( !planner->InitPlan(probot, params) ) {
                    RAVELOG_DEBUG(str(boost::format("grasp %d: grasper planner failed")%grasp_params->id));
                    continue;
                }
                if( !planner->PlanPath(ptraj) ) {
                    RAVELOG_DEBUG(str(boost::format("grasp %d: grasper planner failed")%grasp_params->id));
                    continue;
                }

                BOOST_ASSERT(ptraj->GetNumWaypoints() > 0);
                vector<dReal> vtrajpoint;
                ptraj->GetWaypoint(-1,vtrajpoint,probot->GetConfigurationSpecification());
                probot->SetConfigurationValues(vtrajpoint.begin(),true);
                grasp_params->transfinal = probot->GetTransform();
                probot->GetDOFValues(grasp_params->finalshape);

                FOREACHC(itlink, vlinks) {
                    if( pcloneenv->CheckCollision(KinBody::LinkConstPtr(*itlink), KinBodyConstPtr(params->targetbody), report) ) {
                        RAVELOG_VERBOSE(str(boost::format("contact %s\n")%report->__str__()));
                        FOREACH(itcontact,report->contacts) {
                            if( report->plink1 != *itlink ) {
                                itcontact->norm = -itcontact->norm;
                                itcontact->depth = -itcontact->depth;
                            }
                            grasp_params->contacts.push_back(make_pair(*itcontact,(*itlink)->GetIndex()));
                        }
                    }
                }

                if ( worker_params->bCheckGraspIK ) {
                    CollisionOptionsStateSaver optionstate(pcloneenv->GetCollisionChecker(),coloptions,false); // remove contacts
                    Transform Tgoalgrasp = probot->GetActiveManipulator()->GetEndEffectorTransform();
                    RobotBase::RobotStateSaver linksaver(probot);
                    probot->SetTransform(trobotstart);
                    FOREACH(itlink,vlinks) {
                        (*itlink)->Enable(false);
                    }
                    probot->SetActiveDOFs(worker_params->vactiveindices);
                    probot->SetActiveDOFValues(grasp_params->preshape);
                    probot->SetActiveDOFs(probot->GetActiveManipulator()->GetArmIndices());
                    vector<dReal> solution;
                    if( !probot->GetActiveManipulator()->FindIKSolution(Tgoalgrasp, solution,IKFO_CheckEnvCollisions) ) {
                        RAVELOG_DEBUG(str(boost::format("grasp %d: ik failed")%grasp_params->id));
                        continue;     // ik failed
                    }

                    grasp_params->transfinal = trobotstart;
                    size_t index = 0;
                    FOREACHC(itarmindex,probot->GetActiveManipulator()->GetArmIndices()) {
                        grasp_params->finalshape.at(*itarmindex) = solution.at(index++);
                    }
                }

                GRASPANALYSIS analysis;
                if( worker_params->bComputeForceClosure ) {
                    try {
                        vector<CollisionReport::CONTACT> c(grasp_params->contacts.size());
                        for(size_t i = 0; i < c.size(); ++i) {
                            c[i] = grasp_params->contacts[i].first;
                        }
                        analysis = _AnalyzeContacts3D(c,worker_params->friction,8);
                        if( analysis.mindist < worker_params->forceclosurethreshold ) {
                            RAVELOG_DEBUG(str(boost::format("grasp %d: force closure failed")%grasp_params->id));
                            continue;
                        }
                        grasp_params->mindist = analysis.mindist;
                        grasp_params->volume = analysis.volume;
                    }
                    catch(const std::exception& ex) {
                        RAVELOG_DEBUG(str(boost::format("grasp %d: force closure failed: %s")%grasp_params->id%ex.what()));
                        continue;     // failed
                    }
                }

                if( worker_params->fgraspingnoise > 0 && worker_params->nGraspingNoiseRetries > 0 ) {
                    params->fgraspingnoise = worker_params->fgraspingnoise;
                    vector<Transform> vfinaltransformations; vfinaltransformations.reserve(worker_params->nGraspingNoiseRetries);
                    vector< vector<dReal> > vfinalvalues; vfinalvalues.reserve(worker_params->nGraspingNoiseRetries);
                    for(int igrasp = 0; igrasp < worker_params->nGraspingNoiseRetries; ++igrasp) {
                        probot->SetActiveDOFs(worker_params->vactiveindices);
                        probot->SetActiveDOFValues(grasp_params->preshape);
                        probot->SetActiveDOFs(worker_params->vactiveindices,worker_params->affinedofs,worker_params->affineaxis);
                        params->vinitialconfig.resize(0);
                        ptraj->Init(probot->GetActiveConfigurationSpecification());
                        if( !planner->InitPlan(probot, params) ) {
                            RAVELOG_VERBOSE(str(boost::format("grasp %d: grasping noise planner failed")%grasp_params->id));
                            break;
                        }
                        if( !planner->PlanPath(ptraj) ) {
                            RAVELOG_VERBOSE(str(boost::format("grasp %d: grasping noise planner failed")%grasp_params->id));
                            break;
                        }
                        BOOST_ASSERT(ptraj->GetNumWaypoints() > 0);

                        if ( worker_params->bCheckGraspIK ) {
                            CollisionOptionsStateSaver optionstate(pcloneenv->GetCollisionChecker(),coloptions,false); // remove contacts
                            RobotBase::RobotStateSaver linksaver(probot);
                            ptraj->GetWaypoint(-1,vtrajpoint);
                            Transform t = probot->GetTransform();
                            ptraj->GetConfigurationSpecification().ExtractTransform(t,vtrajpoint.begin(),probot);
                            probot->SetTransform(t);
                            Transform Tgoalgrasp = probot->GetActiveManipulator()->GetEndEffectorTransform();
                            probot->SetTransform(trobotstart);
                            FOREACH(itlink,vlinks) {
                                (*itlink)->Enable(false);
                            }
                            probot->SetActiveDOFs(worker_params->vactiveindices);
                            probot->SetActiveDOFValues(grasp_params->preshape);
                            probot->SetActiveDOFs(probot->GetActiveManipulator()->GetArmIndices());
                            vector<dReal> solution;
                            if( !probot->GetActiveManipulator()->FindIKSolution(Tgoalgrasp, solution,IKFO_CheckEnvCollisions) ) {
                                RAVELOG_VERBOSE(str(boost::format("grasp %d: grasping noise ik failed")%grasp_params->id));
                                break;
                            }
                        }

                        ptraj->GetWaypoint(-1,vtrajpoint,probot->GetConfigurationSpecification());
                        probot->SetConfigurationValues(vtrajpoint.begin(),true);
                        vfinalvalues.push_back(vector<dReal>());
                        probot->GetDOFValues(vfinalvalues.back());
                        vfinaltransformations.push_back(probot->GetActiveManipulator()->GetTransform());
                    }

                    if( (int)vfinaltransformations.size() != worker_params->nGraspingNoiseRetries ) {
                        RAVELOG_DEBUG(str(boost::format("grasp %d: grasping noise failed")%grasp_params->id));
                        continue;
                    }

                    // take statistics
                    Vector translationmean;
                    FOREACHC(ittrans,vfinaltransformations) {
                        translationmean += ittrans->trans;
                    }
                    translationmean *= (1.0/vfinaltransformations.size());
                    Vector translationstd;
                    FOREACHC(ittrans,vfinaltransformations) {
                        Vector v = ittrans->trans - translationmean;
                        translationstd += v*v;
                    }
                    translationstd *= (1.0/vfinaltransformations.size());
                    dReal ftranslationdisplacement = (RaveSqrt(translationstd.x)+RaveSqrt(translationstd.y)+RaveSqrt(translationstd.z))/3;
                    vector<dReal> jointvaluesstd(vfinalvalues.at(0).size());
                    for(size_t i = 0; i < jointvaluesstd.size(); ++i) {
                        dReal jointmean = 0;
                        FOREACHC(it, vfinalvalues) {
                            jointmean += it->at(i);
                        }
                        jointmean /= dReal(vfinalvalues.size());
                        dReal jointstd = 0;
                        FOREACHC(it, vfinalvalues) {
                            jointstd += (it->at(i)-jointmean)*(it->at(i)-jointmean);
                        }
                        jointvaluesstd[i] = _vjointmaxlengths.at(i) * RaveSqrt(jointstd / dReal(vfinalvalues.size()));
                    }
                    dReal fmaxjointdisplacement = 0;
                    FOREACHC(itlink, _robot->GetLinks()) {
                        dReal f = 0;
                        for(size_t ijoint = 0; ijoint < _robot->GetJoints().size(); ++ijoint) {
                            if( _robot->DoesAffect(ijoint, (*itlink)->GetIndex()) ) {
                                f += jointvaluesstd.at(ijoint);
                            }
                        }
                        fmaxjointdisplacement = max(fmaxjointdisplacement,f);
                    }

                    dReal graspthresh = 0.005*RaveSqrt(0.49+400*worker_params->fgraspingnoise)-0.0035;
                    if( graspthresh < worker_params->fgraspingnoise*0.1 ) {
                        graspthresh = worker_params->fgraspingnoise*0.1;
                    }
                    if( ftranslationdisplacement+fmaxjointdisplacement > graspthresh ) {
                        RAVELOG_DEBUG(str(boost::format("grasp %d: fragile grasp %f>%f\n")%grasp_params->id%(ftranslationdisplacement+fmaxjointdisplacement)%(0.7 * worker_params->fgraspingnoise)));
                        continue;
                    }
                }

                RAVELOG_DEBUG(str(boost::format("grasp %d: success")%grasp_params->id));

                boost::mutex::scoped_lock lock(_mutexGrasp);
                _listGraspResults.push_back(grasp_params);
            }
        }
        pcloneenv->Destroy();
    }

    bool _bContinueWorker;
    boost::mutex _mutexGrasp;
    GraspParametersThreadPtr _graspParamsWork;
    list<GraspParametersThreadPtr> _listGraspResults;
    boost::condition _condGraspHasWork, _condGraspReceivedWork;

protected:
    void _ComputeJointMaxLengths(vector<dReal>& vjointlengths)
    {
        TriMesh collisiondata;
        vector<Vector> vworldvertices; vworldvertices.reserve(10000);
        vjointlengths.resize(_robot->GetJoints().size(),0);
        FOREACHC(itjoint, _robot->GetJoints()) {
            if( !!(*itjoint)->GetHierarchyChildLink() ) {
                // todo: support multi-dof joints
                if( (*itjoint)->IsPrismatic(0) ) {
                    vjointlengths.at((*itjoint)->GetJointIndex()) = 1;
                }
                else if( (*itjoint)->IsRevolute(0) ) {
                    Transform t = (*itjoint)->GetHierarchyChildLink()->GetTransform();
                    t.trans -= (*itjoint)->GetAnchor();
                    vworldvertices.resize(0);
                    FOREACHC(itvertex, (*itjoint)->GetHierarchyChildLink()->GetCollisionData().vertices) {
                        vworldvertices.push_back(t * *itvertex);
                    }
                    FOREACHC(itchildjoint, _robot->GetJoints()) {
                        if( *itchildjoint != *itjoint ) {
                            if( (*itchildjoint)->GetFirstAttached() == (*itjoint)->GetHierarchyChildLink() || (*itchildjoint)->GetSecondAttached() == (*itjoint)->GetHierarchyChildLink() ) {
                                vworldvertices.push_back((*itchildjoint)->GetAnchor() - (*itjoint)->GetAnchor());
                            }
                        }
                    }
                    dReal maxlength = 0;
                    FOREACHC(itv,vworldvertices) {
                        dReal faxisdist = itv->dot3((*itjoint)->GetAxis(0));
                        maxlength = max(maxlength,itv->lengthsqr3() - faxisdist*faxisdist);
                    }
                    vjointlengths.at((*itjoint)->GetJointIndex()) = RaveSqrt(maxlength);
                }
            }
        }
    }

    void SampleObject(KinBodyPtr pbody, vector<CollisionReport::CONTACT>& vpoints, int N, Vector graspcenter)
    {
        RAY r;
        Vector com = graspcenter;
        GetEnv()->GetCollisionChecker()->SetCollisionOptions(CO_Contacts|CO_Distance);

        vpoints.resize(N);
        int i = 0;

        while(i < N) {
            r.dir.z = 2*RaveRandomFloat()-1;
            dReal R = RaveSqrt(1 - r.dir.x * r.dir.x);
            dReal U2 = 2 * PI * RaveRandomFloat();
            r.dir.x = R * RaveCos(U2);
            r.dir.y = R * RaveSin(U2);

            r.pos = com - 10.0f*r.dir;
            r.dir *= 1000;

            if( GetEnv()->CheckCollision(r, KinBodyConstPtr(pbody), _report) ) {
                vpoints[i].norm = _report->contacts.at(0).norm;
                vpoints[i].pos = _report->contacts.at(0).pos + 0.001f * vpoints[i].norm;     // extrude a little
                vpoints[i].depth = 0;
                i++;
            }
        }

        GetEnv()->GetCollisionChecker()->SetCollisionOptions(0);
    }

    // generates samples across a geodesic sphere (the higher the level, the higher the number of points
    void DeterministicallySample(KinBodyPtr pbody, vector<CollisionReport::CONTACT>& vpoints, int levels, Vector graspcenter)
    {
        RAY r;
        TriMesh tri;
        Vector com = graspcenter;
        GenerateSphereTriangulation(tri,levels);

        GetEnv()->GetCollisionChecker()->SetCollisionOptions(CO_Contacts|CO_Distance);

        // take the mean across every tri
        vpoints.reserve(tri.indices.size()/3);
        for(int i = 0; i < (int)tri.indices.size(); i += 3) {
            r.dir = 0.33333f * (tri.vertices[tri.indices[i]] + tri.vertices[tri.indices[i+1]] + tri.vertices[tri.indices[i+2]]);
            r.dir.normalize3();
            r.dir *= 1000;

            r.pos = com - 10.0f*r.dir;
            CollisionReport::CONTACT p;
            if( GetEnv()->CheckCollision(r, KinBodyConstPtr(pbody), _report) ) {
                p.norm = -_report->contacts.at(0).norm;    //-r.dir//_report->contacts.at(0).norm1;
                p.pos = _report->contacts.at(0).pos + 0.001f * p.norm;     // extrude a little
                p.depth = 0;
                vpoints.push_back(p);
            }
        }

        GetEnv()->GetCollisionChecker()->SetCollisionOptions(0);
    }

    // generate a sphere triangulation starting with an icosahedron
    // all triangles are oriented counter clockwise
    void GenerateSphereTriangulation(TriMesh& tri, int levels)
    {
        TriMesh temp, temp2;

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
            if( v[0].dot3((v[1]-v[0]).cross(v[2]-v[0])) < 0 )
                swap(indices[i], indices[i+1]);
        }

        temp.indices.resize(nindices);
        std::copy(&indices[0],&indices[nindices],temp.indices.begin());

        TriMesh* pcur = &temp;
        TriMesh* pnew = &temp2;
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

    void BoxSample(KinBodyPtr pbody, vector<CollisionReport::CONTACT>& vpoints, int num_samples, Vector center)
    {
        RAY r;
        TriMesh tri;
        CollisionReport::CONTACT p;
        dReal ffar = 1.0f;

        GetEnv()->GetCollisionChecker()->SetCollisionOptions(CO_Contacts|CO_Distance);
        vpoints.reserve(num_samples);

        dReal counter = ffar/sqrt((dReal)num_samples/12);
        for(int k = 0; k < 6; k++) {
            for(dReal i = -ffar/2.0f; i < ffar/2.0f; i+=counter) {
                for(dReal j = -ffar/2.0f; j < ffar/2.0f; j+=counter) {
                    switch(k) {
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
                        p.norm = -_report->contacts.at(0).norm;    //-r.dir//_report->contacts.at(0).norm1;
                        p.pos = _report->contacts.at(0).pos;    // + 0.001f * p.norm; // extrude a little
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
    void _ComputeDistanceMap(vector<CollisionReport::CONTACT>& vpoints, dReal fTheta)
    {
        dReal fCosTheta = RaveCos(fTheta);
        int N;
        if(fTheta < 0.01f) {
            N = 1;
        }
        RAY r;

        GetEnv()->GetCollisionChecker()->SetCollisionOptions(CO_Distance);

        // set number of rays to randomly sample
        if( fTheta < 0.01f ) {
            N = 1;
        }
        else {
            N = (int)ceil(fTheta * (64.0f/(PI/12.0f)));     // sample 64 points when at pi/12
        }
        for(int i = 0; i < (int)vpoints.size(); ++i) {
            Vector vright = Vector(1,0,0);
            if( RaveFabs(vpoints[i].norm.x) > 0.9 ) {
                vright.y = 1;
            }
            vright -= vpoints[i].norm * vright.dot3(vpoints[i].norm);
            vright.normalize3();
            Vector vup = vpoints[i].norm.cross(vright);

            dReal fMinDist = 2;
            for(int j = 0; j < N; ++j) {
                // sample around a cone
                dReal fAng = fCosTheta + (1-fCosTheta)*RaveRandomFloat();
                dReal R = RaveSqrt(1 - fAng * fAng);
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

    void _GetStableContacts(vector< pair<CollisionReport::CONTACT,int> >& contacts, const Vector& direction, dReal mu)
    {
        BOOST_ASSERT(mu>0);
        RAVELOG_DEBUG("Starting GetStableContacts...\n");

        if(!GetEnv()->CheckCollision(KinBodyConstPtr(_robot))) {
            RAVELOG_ERROR("GrasperModule::GetStableContacts - Error: Robot is not colliding with the target.\n");
            return;
        }

        //make sure we get the right closing direction and don't look at irrelevant joints
        vector<dReal> closingdir(_robot->GetDOF(),0);
        FOREACH(itmanip,_robot->GetManipulators()) {
            vector<dReal>::const_iterator itclosing = (*itmanip)->GetClosingDirection().begin();
            FOREACHC(itgripper,(*itmanip)->GetGripperIndices()) {
                closingdir.at(*itgripper) = *itclosing++;
            }
        }

        // calculate the contact normals using the Jacobian
        std::vector<dReal> J;
        FOREACHC(itlink,_robot->GetLinks()) {
            if( GetEnv()->CheckCollision(KinBody::LinkConstPtr(*itlink), _report) )  {
                RAVELOG_DEBUG(str(boost::format("contact %s:%s with %s:%s\n")%_report->plink1->GetParent()->GetName()%_report->plink1->GetName()%_report->plink2->GetParent()->GetName()%_report->plink2->GetName()));
                FOREACH(itcontact, _report->contacts) {
                    if( _report->plink1 != *itlink )
                        itcontact->norm = -itcontact->norm;

                    Vector deltaxyz;
                    //check if this link is the base link, if so there will be no Jacobian
                    if(( *itlink == _robot->GetLinks().at(0)) || (!!_robot->GetActiveManipulator() &&( *itlink == _robot->GetActiveManipulator()->GetBase()) ) ) {
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
                        RAVELOG_WARN(str(boost::format("degenerate link at %s")%(*itlink)->GetName()));
                        deltaxyz = direction;
                    }

                    deltaxyz.normalize3();

                    if( IS_DEBUGLEVEL(Level_Debug) ) {
                        stringstream ss;
                        ss << "link " << (*itlink)->GetIndex() << " delta XYZ: ";
                        for(int q = 0; q < 3; q++) {
                            ss << deltaxyz[q] << " ";
                        }
                        ss << endl;
                        RAVELOG_DEBUG(ss.str());
                    }

                    // determine if contact is stable (if angle is obtuse, can't be in friction cone)
                    dReal fsin2 = itcontact->norm.cross(deltaxyz).lengthsqr3();
                    dReal fcos = itcontact->norm.dot3(deltaxyz);
                    bool bstable = fcos > 0 && fsin2 <= fcos*fcos*mu*mu;
                    if(bstable) {
                        contacts.push_back(make_pair(*itcontact,(*itlink)->GetIndex()));
                    }
                }
            }
        }
    }

    virtual GRASPANALYSIS _AnalyzeContacts3D(const vector<CollisionReport::CONTACT>& contacts, dReal mu, int Nconepoints)
    {
        if( mu == 0 ) {
            return _AnalyzeContacts3D(contacts);
        }

        if( contacts.size() > 16 ) {
            // try reduce time by computing a subset of the points
            vector<CollisionReport::CONTACT> reducedcontacts;
            reducedcontacts.reserve(16);
            for(size_t i = 0; i < reducedcontacts.capacity(); ++i) {
                reducedcontacts.push_back( contacts.at((i*contacts.size())/reducedcontacts.capacity()) );
            }
            GRASPANALYSIS analysis = _AnalyzeContacts3D(reducedcontacts, mu, Nconepoints);
            if( analysis.mindist > 1e-9 ) {
                return analysis;
            }
        }

        dReal fdeltaang = 2*PI/(dReal)Nconepoints;
        dReal fang = 0;
        vector<pair<dReal,dReal> > vsincos(Nconepoints);
        FOREACH(it,vsincos) {
            it->first = RaveSin(fang);
            it->second = RaveCos(fang);
            fang += fdeltaang;
        }

        vector<CollisionReport::CONTACT> newcontacts;
        newcontacts.reserve(contacts.size()*Nconepoints);
        FOREACHC(itcontact,contacts) {
            // find a coordinate system where z is the normal
            TransformMatrix torient = matrixFromQuat(quatRotateDirection(Vector(0,0,1),itcontact->norm));
            Vector right(torient.m[0],torient.m[4],torient.m[8]);
            Vector up(torient.m[1],torient.m[5],torient.m[9]);
            FOREACH(it,vsincos) {
                newcontacts.push_back(CollisionReport::CONTACT(itcontact->pos, (itcontact->norm + mu*it->first*right + mu*it->second*up).normalize3(),0));
            }
        }
        return _AnalyzeContacts3D(newcontacts);
    }

    virtual GRASPANALYSIS _AnalyzeContacts3D(const vector<CollisionReport::CONTACT>& contacts)
    {
        if( contacts.size() < 7 ) {
            RAVELOG_DEBUG("need at least 7 contact wrenches to have force closure in 3D\n");
            return GRASPANALYSIS();
        }
        RAVELOG_DEBUG(str(boost::format("analyzing %d contacts for force closure\n")%contacts.size()));
        GRASPANALYSIS analysis;
        vector<double> vpoints(6*contacts.size()), vconvexplanes;
        vector<double>::iterator itpoint = vpoints.begin();
        FOREACHC(itcontact, contacts) {
            *itpoint++ = itcontact->norm.x;
            *itpoint++ = itcontact->norm.y;
            *itpoint++ = itcontact->norm.z;
            Vector v = itcontact->pos.cross(itcontact->norm);
            *itpoint++ = v.x;
            *itpoint++ = v.y;
            *itpoint++ = v.z;
        }

        analysis.volume = _ComputeConvexHull(vpoints,vconvexplanes,boost::shared_ptr< vector<int> >(),6);
        if( vconvexplanes.size() == 0 ) {
            return analysis;
        }
        // go through each of the faces and check if center is inside, and compute its distance
        double mindist = 1e30;
        for(size_t i = 0; i < vconvexplanes.size(); i += 7) {
            if(( vconvexplanes.at(i+6) > 0) ||( RaveFabs(vconvexplanes.at(i+6)) < 1e-15) ) {
                return analysis;
            }
            mindist = min(mindist,-vconvexplanes.at(i+6));
        }
        analysis.mindist = mindist;
        return analysis;
    }

    /// Computes the convex hull of a set of points
    /// \param vpoints a set of points each of dimension dim
    /// \param vconvexplaces the places of the convex hull, dimension is dim+1
    virtual double _ComputeConvexHull(const vector<double>& vpoints, vector<double>& vconvexplanes, boost::shared_ptr< vector<int> > vconvexfaces, int dim)
    {
        boost::mutex::scoped_lock lock(s_QhullMutex);
        vconvexplanes.resize(0);
#ifdef QHULL_FOUND
        vector<coordT> qpoints(vpoints.size());
        std::copy(vpoints.begin(),vpoints.end(),qpoints.begin());

        boolT ismalloc = 0;               // True if qhull should free points in qh_freeqhull() or reallocation
        char flags[]= "qhull Tv FA";     // option flags for qhull, see qh_opt.htm, output volume (FA)

        if( !errfile ) {
            errfile = tmpfile();        // stderr, error messages from qhull code
        }
        int exitcode= qh_new_qhull (dim, qpoints.size()/dim, &qpoints[0], ismalloc, flags, errfile, errfile);
        if (!exitcode) {
            vconvexplanes.reserve(1000);
            if( !!vconvexfaces ) {
                // fill with face indices
                vconvexfaces->resize(0);
                vconvexfaces->push_back(0);
            }
            facetT *facet;                    // set by FORALLfacets
            vertexT *vertex, **vertexp;     // set by FOREACHvdertex_
            FORALLfacets {     // 'qh facet_list' contains the convex hull
                //                if( facet->isarea && facet->f.area < 1e-15 ) {
                //                    RAVELOG_VERBOSE(str(boost::format("skipping area: %e\n")%facet->f.area));
                //                    continue;
                //                }
                if( !!vconvexfaces && !!facet->vertices ) {
                    size_t startindex = vconvexfaces->size();
                    vconvexfaces->push_back(0);
                    FOREACHvertex_(facet->vertices) {
                        int id = qh_pointid(vertex->point);
                        BOOST_ASSERT(id>=0);
                        vconvexfaces->push_back(id);
                    }
                    vconvexfaces->at(startindex) = vconvexfaces->size()-startindex-1;
                    vconvexfaces->at(0) += 1;
                }
                if( !!facet->normal ) {
                    for(int i = 0; i < dim; ++i) {
                        vconvexplanes.push_back(facet->normal[i]);
                    }
                    vconvexplanes.push_back(facet->offset);
                }
            }
        }

        double totvol = qh totvol;
        qh_freeqhull(!qh_ALL);
        int curlong, totlong;         // memory remaining after qh_memfreeshort
        qh_memfreeshort (&curlong, &totlong);
        if (curlong || totlong) {
            RAVELOG_ERROR("qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);
        }
        if( exitcode ) {
            RAVELOG_DEBUG(str(boost::format("Qhull failed with error %d")%exitcode));
            vconvexplanes.resize(0);
            if( !!vconvexfaces ) {
                vconvexfaces->resize(0);
            }
            return 0;
        }

        vector<double> vmean(dim,0);
        for(size_t i = 0; i < vpoints.size(); i += dim) {
            for(int j = 0; j < dim; ++j)
                vmean[j] += vpoints[i+j];
        }
        double fipoints = 1/(double)(vpoints.size()/dim);
        for(int j = 0; j < dim; ++j) {
            vmean[j] *= fipoints;
        }
        for(size_t i = 0; i < vconvexplanes.size(); i += dim+1) {
            double meandist = 0;
            for(int j = 0; j < dim; ++j) {
                meandist += vconvexplanes[i+j]*vmean[j];
            }
            meandist += vconvexplanes.at(i+dim);
            if( meandist > 0 ) {
                for(int j = 0; j < dim; ++j) {
                    vconvexplanes[i+j] = -vconvexplanes[i+j];
                }
            }
        }

        return totvol;     // return volume
#else
        throw openrave_exception(str(boost::format("QHull library not found, cannot compute convex hull of contact points")));
        return 0;
#endif
    }

    PlannerBasePtr _planner;
    RobotBasePtr _robot;
    CollisionReportPtr _report;
    boost::mutex _mutex;
    FILE *errfile;
    std::vector<dReal> _vjointmaxlengths;
};

ModuleBasePtr CreateGrasperModule(EnvironmentBasePtr penv, std::istream& sinput)
{
    return ModuleBasePtr(new GrasperModule(penv,sinput));
}
