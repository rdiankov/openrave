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
#include "plugindefs.h"

LoggingProblem::LoggingProblem(EnvironmentBase* penv) : CmdProblemInstance(penv)
{
    bDestroyThread = false;
    bAbsoluteTiem = false;
    bDoLog = false;

    RegisterCommand("savescene",(CommandFn)&LoggingProblem::SaveScene,
                    "Saves the entire scene in an xml file. If paths are relative,\n"
                    "should only be opened from the dirctory openrave was launched in\n"
                    "Usage: [filename %s] [absolute (default is relative)]");
    RegisterCommand("startreplay",(CommandFn)&LoggingProblem::StartReplay,
                    "Starts replaying a recording given a speed (can be negative).\n"
                    "Usage: [speed %f] [filename %s]");
    RegisterCommand("startrecording",(CommandFn)&LoggingProblem::StartRecording,
                    "Starts recording the scene given a realtime delta.\n"
                    "If a current recording is in progress, stop it.\n"
                    "Usage: [filename %s] [realtime %f]");
    RegisterCommand("stoprecording",(CommandFn)&LoggingProblem::StopRecording,
                    "Stop recording.\n"
                    "Usage: [speed %f] [filename %s]");
    RegisterCommand("help",(CommandFn)&LoggingProblem::Help, "display this message.");

    pthread_create(&_threadlog, NULL, log_thread, this);
}

LoggingProblem::~LoggingProblem()
{
    Destroy();

    void* retvalue;
    bDestroyThread = true;
    pthread_join(_threadlog, &retvalue);
}

void LoggingProblem::Destroy()
{
    bDoLog = false;
    bAbsoluteTiem = false;
}

int LoggingProblem::main(const char* cmd)
{
    return 0;
}

bool LoggingProblem::SimulationStep(dReal fElapsedTime)
{
    return false;
}

bool LoggingProblem::SaveScene(ostream& sout, istream& sinput)
{
    string cmd, filename;
    bool bAbsolutePaths = false;
    while(!sinput.eof()) {
        sinput >> cmd;
        if( !sinput )
            break;

        if( stricmp(cmd.c_str(), "filename") == 0 )
            sinput >> filename;
        else if( stricmp(cmd.c_str(), "absolute") == 0 )
            bAbsolutePaths = true;
        else break;

        if( !sinput ) {
            RAVELOG(L"failed\n");
            break;
        }
    }

    if( filename.size() == 0 ) {
        RAVELOG(L"bad filename\n");
        sout << "0" << endl;
        return false;
    }

    ofstream fout(filename.c_str());
    if( !fout ) {
        RAVEPRINT(L"failed to open file %s\n", filename.c_str());
        sout << "0" << endl;
        return false;
    }

    fout << "<Environment>" << endl;

    int tabwidth = 2;
    TransformMatrix t = TransformMatrix(GetEnv()->GetCameraTransform());
    fout << setw(tabwidth) << " "
         << "<camtrans>" << t.trans.x << " " << t.trans.y << " " << t.trans.z << "</camtrans>" << endl;
    fout << setw(tabwidth) << " " << "<camrotmat>";
    for(int i = 0; i < 3; ++i)
        fout << t.m[4*i+0] << " " << t.m[4*i+1] << " " << t.m[4*i+2] << " ";
    fout << "</camrotmat>" << endl << endl;

    char str[256];

    FOREACHC(itbody, GetEnv()->GetBodies()) {

        KinBody* pbody = *itbody;

        sprintf(str, "%S", pbody->GetName());
        if( pbody->GetXMLFilename().size() > 0 ) {
            fout << setw(tabwidth) << " "
                 << (pbody->IsRobot()?"<Robot":"<KinBody") << " name=\"" << str << "\"";
            fout << " file=\"" << pbody->GetXMLFilename() << "\">" << endl;
        }
        else {
            // have to process each link/geometry file
            RAVEPRINT(L"cannot process object %S defined inside environment file\n", pbody->GetName());
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
        pbody->GetJointValues(values);
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

bool LoggingProblem::StartReplay(ostream& sout, istream& sinput)
{
    RAVEPRINT(L"not implemented\n");
    return true;
}

bool LoggingProblem::StartRecording(ostream& sout, istream& sinput)
{
    RAVEPRINT(L"not implemented\n");
    return true;
}

bool LoggingProblem::StopRecording(ostream& sout, istream& sinput)
{
    RAVEPRINT(L"not implemented\n");
    return true;
}

void* LoggingProblem::log_thread(void* param)
{
    return ((LoggingProblem*)param)->_log_thread();
}

void* LoggingProblem::_log_thread()
{
    while(!bDestroyThread) {
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
//                (*itbody)->GetJointValues(vjoints);
//                
//                len = vjoints.size();
//                fwrite(&len, 4, 1, pfLog);
//                if( len > 0 )
//                    fwrite(&vjoints[0], sizeof(vjoints[0]) * vjoints.size(), 1, pfLog);
//            }
//            
//            fLogTime = fTotalTime;
//        }
        Sleep(1);
    }
    
    return NULL;
}

bool LoggingProblem::Help(ostream& sout, istream& sinput)
{
    sout << "LoggingProblem help:" << endl;
    GetCommandHelp(sout);
    return true;
}
