// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.co>m
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
#ifndef OPENRAVE_LOGGING_H
#define OPENRAVE_LOGGING_H

/// used to log scene elements
class LoggingModule : public ModuleBase
{
    inline boost::shared_ptr<LoggingModule> shared_module() {
        return boost::static_pointer_cast<LoggingModule>(shared_from_this());
    }
    inline boost::shared_ptr<LoggingModule const> shared_module_const() const {
        return boost::static_pointer_cast<LoggingModule const>(shared_from_this());
    }

public:
    LoggingModule(EnvironmentBasePtr penv) : ModuleBase(penv)
    {
        __description = ":Interface Author: Rosen Diankov\n\nCan save the entire scene to an OpenRAVE XML file";
        RegisterCommand("savescene",boost::bind(&LoggingModule::SaveScene,this,_1,_2),
                        "Saves the entire scene in an xml file. If paths are relative,\n"
                        "should only be opened from the dirctory openrave was launched in\n"
                        "Usage: [filename %s] [absolute (default is relative)]");
        //        RegisterCommand("startreplay",boost::bind(&LoggingModule::StartReplay,this,_1,_2),
        //                        "Starts replaying a recording given a speed (can be negative).\n"
        //                        "Usage: [speed %f] [filename %s]");
        //        RegisterCommand("startrecording",boost::bind(&LoggingModule::StartRecording,this,_1,_2),
        //                        "Starts recording the scene given a realtime delta.\n"
        //                        "If a current recording is in progress, stop it.\n"
        //                        "Usage: [filename %s] [realtime %f]");
        //        RegisterCommand("stoprecording",boost::bind(&LoggingModule::StopRecording,this,_1,_2),
        //                        "Stop recording.\n"
        //                        "Usage: [speed %f] [filename %s]");

        bDestroyThread = false;
        bAbsoluteTiem = false;
        bDoLog = false;
    }

    virtual ~LoggingModule() {
    }
    virtual void Destroy()
    {
        bDestroyThread = true;
        bDoLog = false;
        bAbsoluteTiem = false;
        if( !!_threadlog ) {
            _threadlog->join();
        }
        _threadlog.reset();
    }

    virtual int main(const string& cmd)
    {
        Destroy();

        bDestroyThread = false;
        //_threadlog.reset(new boost::thread(boost::bind(&LoggingModule::_log_thread,shared_module())));
        return 0;
    }

    /// returns true when problem is solved
    virtual bool SimulationStep(dReal fElapsedTime)
    {
        return false;
    }

private:
    virtual bool SaveScene(ostream& sout, istream& sinput)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());

        string cmd, filename;
        bool bAbsolutePaths = false;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "filename" )
                sinput >> filename;
            else if( cmd == "absolute" )
                bAbsolutePaths = true;
            else break;

            if( !sinput ) {
                RAVELOG_ERROR("failed\n");
                break;
            }
        }

        if( filename.size() == 0 ) {
            RAVELOG_ERROR("bad filename\n");
            sout << "0" << endl;
            return false;
        }

        ofstream fout(filename.c_str());
        if( !fout ) {
            RAVELOG_ERROR("failed to open file %s\n", filename.c_str());
            sout << "0" << endl;
            return false;
        }

        fout << std::setprecision(std::numeric_limits<dReal>::digits10+1);
        fout << "<Environment>" << endl;

        int tabwidth = 2;
        TransformMatrix t;
        if( !!GetEnv()->GetViewer() ) {
            t = TransformMatrix(GetEnv()->GetViewer()->GetCameraTransform());
        }
        fout << setw(tabwidth) << " " << "<camtrans>" << t.trans.x << " " << t.trans.y << " " << t.trans.z << "</camtrans>" << endl;
        fout << setw(tabwidth) << " " << "<camrotmat>";
        for(int i = 0; i < 3; ++i) {
            fout << t.m[4*i+0] << " " << t.m[4*i+1] << " " << t.m[4*i+2] << " ";
        }
        fout << "</camrotmat>" << endl << endl;

        vector<KinBodyPtr> vecbodies;
        vector<dReal> values;
        GetEnv()->GetBodies(vecbodies);
        FOREACHC(itbody, vecbodies) {
            KinBodyPtr pbody = *itbody;

            fout << setw(tabwidth) << " " << (pbody->IsRobot() ? "<Robot" : "<KinBody") << " name=\"" << pbody->GetName() << "\"";
            Transform tbase;
            if( pbody->GetURI().size() > 0 ) {
                fout << " file=\"" << pbody->GetURI() << "\">" << endl;
                // open up the filename and extract the transformation of first link
                KinBodyPtr ptestbody;
                AttributesList atts;
                atts.push_back(make_pair("skipgeometry","1"));
                if( pbody->IsRobot() ) {
                    RobotBasePtr ptestrobot;
                    ptestbody = GetEnv()->ReadRobotURI(ptestrobot, pbody->GetURI(),atts);
                }
                else {
                    ptestbody = GetEnv()->ReadKinBodyURI(ptestbody, pbody->GetURI(),atts);
                }
                if( !!ptestbody ) {
                    tbase = ptestbody->GetTransform();
                }
                else {
                    RAVELOG_WARN(str(boost::format("failed to load %s")%pbody->GetURI()));
                }
            }
            else {
                fout << ">" << endl;
                RAVELOG_WARN(str(boost::format("object %s does not have a filename, so outputting at least links\n")%pbody->GetName()));
                // have to process each link/geometry file
                FOREACHC(itlink, pbody->GetLinks()) {
                    fout << "<body name=\"" << (*itlink)->GetName() << "\" type=\"" << ((*itlink)->IsStatic() ? "static" : "dynamic") << "\">" << endl;
                    t = pbody->GetTransform().inverse() * (*itlink)->GetTransform();
                    fout << setw(tabwidth*2) << " " << "<Translation>" << t.trans.x << " " << t.trans.y << " " << t.trans.z << "</Translation>" << endl;
                    fout << setw(tabwidth*2) << " " << "<rotationmat>";
                    for(int i = 0; i < 3; ++i) {
                        fout << t.m[4*i+0] << " " << t.m[4*i+1] << " " << t.m[4*i+2] << " ";
                    }
                    fout << "</rotationmat>" << endl;
                    FOREACHC(itgeom,(*itlink)->GetGeometries()) {
                        switch(itgeom->GetType()) {
                        case KinBody::Link::GEOMPROPERTIES::GeomBox:
                            fout << "<geom type=\"box\">" << endl;
                            fout << "<extents>" << itgeom->GetBoxExtents().x << " " << itgeom->GetBoxExtents().y << " "  << itgeom->GetBoxExtents().z << "</extents>";
                            break;
                        case KinBody::Link::GEOMPROPERTIES::GeomSphere:
                            fout << "<geom type=\"sphere\">" << endl;
                            fout << "<radius>" << itgeom->GetSphereRadius() << "</radius>";
                            break;
                        case KinBody::Link::GEOMPROPERTIES::GeomCylinder:
                            fout << "<geom type=\"cylinder\">" << endl;
                            fout << "<radius>" << itgeom->GetCylinderRadius() << "</radius>" << endl;
                            fout << "<height>" << itgeom->GetCylinderHeight() << "</height>" << endl;
                            break;
                        case KinBody::Link::GEOMPROPERTIES::GeomTrimesh:
                            fout << "<geom type=\"trimesh\">" << endl;
                            break;
                        case KinBody::Link::GEOMPROPERTIES::GeomNone:
                            fout << "<geom>" << endl;
                            break;
                        }
                        if( itgeom->GetRenderFilename().size() > 0 ) {
                            fout << "<render>" << itgeom->GetRenderFilename() << "</render>" << endl;
                        }
                        t = itgeom->GetTransform();
                        fout << setw(tabwidth*2) << " " << "<Translation>" << t.trans.x << " " << t.trans.y << " " << t.trans.z << "</Translation>" << endl;
                        fout << setw(tabwidth*2) << " " << "<rotationmat>";
                        for(int i = 0; i < 3; ++i) {
                            fout << t.m[4*i+0] << " " << t.m[4*i+1] << " " << t.m[4*i+2] << " ";
                        }
                        fout << "</rotationmat>" << endl;
                        fout << "<diffusecolor>" << itgeom->GetDiffuseColor().x << " " << itgeom->GetDiffuseColor().y << " " << itgeom->GetDiffuseColor().z << " " << itgeom->GetDiffuseColor().w << "</diffusecolor>";
                        fout << "<ambientcolor>" << itgeom->GetAmbientColor().x << " " << itgeom->GetAmbientColor().y << " " << itgeom->GetAmbientColor().z << " " << itgeom->GetAmbientColor().w << "</ambientcolor>";
                        fout << "</geom>" << endl;
                    }
                    fout << "</body>" << endl;
                }
            }

            t = pbody->GetTransform() * tbase.inverse();
            fout << setw(tabwidth*2) << " " << "<Translation>" << t.trans.x << " " << t.trans.y << " " << t.trans.z << "</Translation>" << endl;
            fout << setw(tabwidth*2) << " " << "<rotationmat>";
            for(int i = 0; i < 3; ++i) {
                fout << t.m[4*i+0] << " " << t.m[4*i+1] << " " << t.m[4*i+2] << " ";
            }
            fout << "</rotationmat>" << endl << endl;

            pbody->GetDOFValues(values);
            if( values.size() > 0 ) {
                fout << setw(tabwidth*2) << " " << "<jointvalues>";
                FOREACH(it, values) {
                    fout << *it << " ";
                }
                fout << "</jointvalues>" << endl;
            }
            fout << setw(tabwidth) << " ";
            fout << (pbody->IsRobot() ? "</Robot>" : "</KinBody>") << endl;
        }

        fout << "</Environment>" << endl;
        sout << "1" << endl;

        return true;
    }

    //    virtual bool StartReplay(ostream& sout, istream& sinput)
    //    {
    //        RAVELOG_ERROR("not implemented\n");
    //        return false;
    //    }
    //    virtual bool StartRecording(ostream& sout, istream& sinput)
    //    {
    //        RAVELOG_ERROR("not implemented\n");
    //        return false;
    //    }
    //    virtual bool StopRecording(ostream& sout, istream& sinput)
    //    {
    //        RAVELOG_ERROR("not implemented\n");
    //        return false;
    //    }

    void _log_thread()
    {
        //while(!bDestroyThread) {
        // record
        //        if( pfLog == NULL ) {
        //            pfLog = fopen("motion.txt", "wb");
        //        }
        //
        //        fTotalTime += fElapsedTime;
        //        if( pfLog != NULL && (fTotalTime-fLogTime) > 0.05f ) {
        //
        //            fwrite(&fTotalTime, sizeof(float), 1, pfLog);
        //
        //            int numobjs = (int)GetEnv()->GetBodies().size();
        //            fwrite(&numobjs, 4, 1, pfLog);
        //
        //            Transform t;
        //            vector<dReal> vjoints;
        //            std::vector<KinBody*>::const_iterator itbody;
        //            FORIT(itbody, GetEnv()->GetBodies()) {
        //                size_t len = wcslen((*itbody)->GetName());
        //                fwrite(&len, 4, 1, pfLog);
        //                fwrite((*itbody)->GetName(), len*sizeof((*itbody)->GetName()[0]), 1, pfLog);
        //
        //                t = (*itbody)->GetTransform();
        //                fwrite(&t, sizeof(t), 1, pfLog);
        //
        //                (*itbody)->GetDOFValues(vjoints);
        //
        //                len = vjoints.size();
        //                fwrite(&len, 4, 1, pfLog);
        //                if( len > 0 )
        //                    fwrite(&vjoints[0], sizeof(vjoints[0]) * vjoints.size(), 1, pfLog);
        //            }
        //
        //            fLogTime = fTotalTime;
        //        }
        //Sleep(1);
        //}
    }

    boost::shared_ptr<boost::thread> _threadlog;
    bool bDoLog, bDestroyThread, bAbsoluteTiem;
};

#endif
