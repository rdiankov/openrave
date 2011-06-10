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
    inline boost::shared_ptr<LoggingModule> shared_module() { return boost::static_pointer_cast<LoggingModule>(shared_from_this()); }
    inline boost::shared_ptr<LoggingModule const> shared_module_const() const { return boost::static_pointer_cast<LoggingModule const>(shared_from_this()); }

 public:
 LoggingModule(EnvironmentBasePtr penv) : ProblemInstance(penv)
    {
        __description = ":Interface Author: Rosen Diankov\n\nCan save the entire scene to an XML file";
        RegisterCommand("savescene",boost::bind(&LoggingModule::SaveScene,this,_1,_2),
                        "Saves the entire scene in an xml file. If paths are relative,\n"
                        "should only be opened from the dirctory openrave was launched in\n"
                        "Usage: [filename %s] [absolute (default is relative)]");
        RegisterCommand("startreplay",boost::bind(&LoggingModule::StartReplay,this,_1,_2),
                        "Starts replaying a recording given a speed (can be negative).\n"
                        "Usage: [speed %f] [filename %s]");
        RegisterCommand("startrecording",boost::bind(&LoggingModule::StartRecording,this,_1,_2),
                        "Starts recording the scene given a realtime delta.\n"
                        "If a current recording is in progress, stop it.\n"
                        "Usage: [filename %s] [realtime %f]");
        RegisterCommand("stoprecording",boost::bind(&LoggingModule::StopRecording,this,_1,_2),
                        "Stop recording.\n"
                        "Usage: [speed %f] [filename %s]");

        bDestroyThread = false;
        bAbsoluteTiem = false;
        bDoLog = false;
    }

    virtual ~LoggingModule() {}
    virtual void Destroy()
    {
        bDestroyThread = true;
        bDoLog = false;
        bAbsoluteTiem = false;
        if( !!_threadlog )
            _threadlog->join();
        _threadlog.reset();
    }

    virtual int main(const string& cmd)
    {
        Destroy();

        bDestroyThread = false;
        _threadlog.reset(new boost::thread(boost::bind(&LoggingModule::_log_thread,shared_module())));
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

        fout << "<Environment>" << endl;

        int tabwidth = 2;
        TransformMatrix t;
        if( !!GetEnv()->GetViewer() ) {
            t = TransformMatrix(GetEnv()->GetViewer()->GetCameraTransform());
        }
        fout << setw(tabwidth) << " "
             << "<camtrans>" << t.trans.x << " " << t.trans.y << " " << t.trans.z << "</camtrans>" << endl;
        fout << setw(tabwidth) << " " << "<camrotmat>";
        for(int i = 0; i < 3; ++i)
            fout << t.m[4*i+0] << " " << t.m[4*i+1] << " " << t.m[4*i+2] << " ";
        fout << "</camrotmat>" << endl << endl;

        vector<KinBodyPtr> vecbodies;
        GetEnv()->GetBodies(vecbodies);
        FOREACHC(itbody, vecbodies) {
            KinBodyPtr pbody = *itbody;

            if( pbody->GetXMLFilename().size() > 0 ) {
                fout << setw(tabwidth) << " "
                     << (pbody->IsRobot()?"<Robot":"<KinBody") << " name=\"" << pbody->GetName() << "\"";
                fout << " file=\"" << pbody->GetXMLFilename() << "\">" << endl;
            }
            else {
                // have to process each link/geometry file
                RAVELOG_ERROR(str(boost::format("cannot process object %s defined inside environment file\n")%pbody->GetName()));
                continue;
                //fout << ">" << endl;
            }
        
            t = pbody->GetTransform();
            fout << setw(tabwidth*2) << " "
                 << "<Translation>" << t.trans.x << " " << t.trans.y << " " << t.trans.z << "</Translation>" << endl;
            fout << setw(tabwidth*2) << " " << "<rotationmat>";
            for(int i = 0; i < 3; ++i)
                fout << t.m[4*i+0] << " " << t.m[4*i+1] << " " << t.m[4*i+2] << " ";
            fout << "</rotationmat>" << endl << endl;

            fout << setw(tabwidth*2) << " " << "<jointvalues>";

            vector<dReal> values;
            pbody->GetDOFValues(values);
            FOREACH(it, values)
                fout << *it << " ";
            fout << "</jointvalues>" << endl;
            fout << setw(tabwidth) << " ";
            fout << (pbody->IsRobot()?"</Robot>":"</KinBody>") << endl;
        }
    
        fout << "</Environment>" << endl;
        sout << "1" << endl;

        return true;
    }

    virtual bool StartReplay(ostream& sout, istream& sinput)
    {
        RAVELOG_ERROR("not implemented\n");
        return false;
    }
    virtual bool StartRecording(ostream& sout, istream& sinput)
    {
        RAVELOG_ERROR("not implemented\n");
        return false;
    }
    virtual bool StopRecording(ostream& sout, istream& sinput)
    {
        RAVELOG_ERROR("not implemented\n");
        return false;
    }

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
