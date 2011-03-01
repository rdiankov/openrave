// Copyright (C) 2006-2009 Rosen Diankov (rdiankov@cs.cmu.edu)
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
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
#include "ravep.h"

namespace OpenRAVE {
    EnvironmentBasePtr RaveCreateEnvironment() {
        boost::shared_ptr<Environment> p(new Environment());
        p->Init();
        return p;
    }

    EnvironmentBasePtr CreateEnvironment(bool bLoadAllPlugins) { return RaveCreateEnvironment(); }
}

#include <streambuf>

#ifdef OPENRAVE_COLLADA_SUPPORT
#include "colladareader.h"
#include "colladawriter.h"

bool RaveParseColladaFile(EnvironmentBasePtr penv, const string& filename,const AttributesList& atts)
{
    ColladaReader reader(penv);
    boost::shared_ptr<pair<string,string> > filedata = OpenRAVEXMLParser::RaveFindFile(filename);
    if (!filedata || !reader.InitFromFile(filedata->second,atts)) {
        return false;
    }
    return reader.Extract();
}

bool RaveParseColladaFile(EnvironmentBasePtr penv, KinBodyPtr& pbody, const string& filename,const AttributesList& atts)
{
    ColladaReader reader(penv);
    boost::shared_ptr<pair<string,string> > filedata = OpenRAVEXMLParser::RaveFindFile(filename);
    if (!filedata || !reader.InitFromFile(filedata->second,atts)) {
        return false;
    }
    return reader.Extract(pbody);
}

bool RaveParseColladaFile(EnvironmentBasePtr penv, RobotBasePtr& probot, const string& filename,const AttributesList& atts)
{
    ColladaReader reader(penv);
    boost::shared_ptr<pair<string,string> > filedata = OpenRAVEXMLParser::RaveFindFile(filename);
    if (!filedata || !reader.InitFromFile(filedata->second,atts)) {
        return false;
    }
    return reader.Extract(probot);
}

bool RaveParseColladaData(EnvironmentBasePtr penv, const string& pdata,const AttributesList& atts) {
    ColladaReader reader(penv);
    if (!reader.InitFromData(pdata,atts)) {
        return false;
    }
    return reader.Extract();
}

bool RaveParseColladaData(EnvironmentBasePtr penv, KinBodyPtr& pbody, const string& pdata,const AttributesList& atts)
{
    ColladaReader reader(penv);
    if (!reader.InitFromData(pdata,atts)) {
        return false;
    }
    return reader.Extract(pbody);
}

bool RaveParseColladaData(EnvironmentBasePtr penv, RobotBasePtr& probot, const string& pdata,const AttributesList& atts)
{
    ColladaReader reader(penv);
    if (!reader.InitFromData(pdata,atts)) {
        return false;
    }
    return reader.Extract(probot);
}

void RaveWriteColladaFile(EnvironmentBasePtr penv, const string& filename)
{
    ColladaWriter writer(penv);
    if( !writer.Write(penv) ) {
        throw openrave_exception("ColladaWriter::Write(EnvironmentBasePtr) failed");
    }
    writer.Save(filename);
}

void RaveWriteColladaFile(KinBodyPtr pbody, const string& filename)
{
    ColladaWriter writer(pbody->GetEnv());
    if( !writer.Write(pbody) ) {
        throw openrave_exception("ColladaWriter::Write(KinBodyPtr) failed");
    }
    writer.Save(filename);
}

void RaveWriteColladaFile(RobotBasePtr probot, const string& filename)
{
    ColladaWriter writer(probot->GetEnv());
    if( !writer.Write(probot) ) {
        throw openrave_exception("ColladaWriter::Write(RobotBasePtr) failed");
    }
    writer.Save(filename);
}

#else

bool RaveParseColladaFile(EnvironmentBasePtr penv, const string& filename,const AttributesList& atts)
{
    RAVELOG_ERROR("collada files not supported\n");
    return false;
}

bool RaveParseColladaFile(EnvironmentBasePtr penv, KinBodyPtr& pbody, const string& filename,const AttributesList& atts)
{
    RAVELOG_ERROR("collada files not supported\n");
    return false;
}

bool RaveParseColladaFile(EnvironmentBasePtr penv, RobotBasePtr& probot, const string& filename,const AttributesList& atts)
{
    RAVELOG_ERROR("collada files not supported\n");
    return false;
}

bool RaveParseColladaData(EnvironmentBasePtr penv, const string& pdata,const AttributesList& atts)
{
    RAVELOG_ERROR("collada files not supported\n");
    return false;
}

bool RaveParseColladaData(EnvironmentBasePtr penv, KinBodyPtr& pbody, const string& pdata,const AttributesList& atts)
{
    RAVELOG_ERROR("collada files not supported\n");
    return false;
}

bool RaveParseColladaData(EnvironmentBasePtr penv, RobotBasePtr& probot, const string& pdata,const AttributesList& atts)
{
    RAVELOG_ERROR("collada files not supported\n");
    return false;
}


bool RaveWriteColladaFile(EnvironmentBasePtr penv, const string& filename)
{
    RAVELOG_ERROR("collada files not supported\n");
    return false;
}

bool RaveWriteColladaFile(KinBodyPtr pbody, const string& filename)
{
    RAVELOG_ERROR("collada files not supported\n");
    return false;
}

bool RaveWriteColladaFile(RobotBasePtr probot, const string& filename)
{
    RAVELOG_ERROR("collada files not supported\n");
    return false;
}

#endif
