// -*- coding: utf-8 -*-
// Copyright (C) 2006-2012 Rosen Diankov <rosen.diankov@gmail.com>
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
#include "libopenrave.h"
#include <boost/lexical_cast.hpp>
#include <openrave/plannerparameters.h>
#include <openrave/xmlreaders.h>

namespace OpenRAVE {

WorkspaceTrajectoryParameters::WorkspaceTrajectoryParameters(EnvironmentBasePtr penv) : maxdeviationangle(0.15*PI), maintaintiming(false), greedysearch(true), ignorefirstcollision(0), ignorefirstcollisionee(0), ignorelastcollisionee(0), minimumcompletetime(0), _penv(penv), _bProcessing(false) {
    _vXMLParameters.push_back("maxdeviationangle");
    _vXMLParameters.push_back("maintaintiming");
    _vXMLParameters.push_back("greedysearch");
    _vXMLParameters.push_back("ignorefirstcollision");
    _vXMLParameters.push_back("ignorefirstcollisionee");
    _vXMLParameters.push_back("ignorelastcollisionee");
    _vXMLParameters.push_back("minimumcompletetime");
    _vXMLParameters.push_back("workspacetraj"); // back-compat
    _vXMLParameters.push_back("workspacetrajectory");
}

// save the extra data to XML
bool WorkspaceTrajectoryParameters::serialize(std::ostream& O, int options) const
{
    if( !PlannerParameters::serialize(O, options&~1) ) {
        return false;
    }
    O << "<maxdeviationangle>" << maxdeviationangle << "</maxdeviationangle>" << std::endl;
    O << "<maintaintiming>" << maintaintiming << "</maintaintiming>" << std::endl;
    O << "<greedysearch>" << greedysearch << "</greedysearch>" << std::endl;
    O << "<ignorefirstcollision>" << ignorefirstcollision << "</ignorefirstcollision>" << std::endl;
    O << "<ignorefirstcollisionee>" << ignorefirstcollisionee << "</ignorefirstcollisionee>" << std::endl;
    O << "<ignorelastcollisionee>" << ignorelastcollisionee << "</ignorelastcollisionee>" << std::endl;
    O << "<minimumcompletetime>" << minimumcompletetime << "</minimumcompletetime>" << std::endl;
    if( !!workspacetraj ) {
        O << "<workspacetrajectory>";
        workspacetraj->serialize(O);
        O << "</workspacetrajectory>" << std::endl;
    }
    if( !(options & 1) ) {
        O << _sExtraParameters << endl;
    }
    return !!O;
}

BaseXMLReader::ProcessElement WorkspaceTrajectoryParameters::startElement(const std::string& name, const AttributesList& atts)
{
    if( _bProcessing ) {
        return PE_Ignore;
    }
    if( !!_pcurreader ) {
        if( _pcurreader->startElement(name,atts) == PE_Support ) {
            return PE_Support;
        }
        return PE_Ignore;
    }
    switch( PlannerBase::PlannerParameters::startElement(name,atts) ) {
    case PE_Pass: break;
    case PE_Support: return PE_Support;
    case PE_Ignore: return PE_Ignore;
    }
    if( name == "workspacetrajectory" ) {
        _pcurreader.reset(new xmlreaders::TrajectoryReader(_penv,workspacetraj,atts));
        _bProcessing = false;
        return PE_Support;
    }
    _bProcessing = name=="maxdeviationangle" || name=="maintaintiming" || name=="greedysearch" || name=="ignorefirstcollision" || name=="ignorefirstcollisionee" || name=="ignorelastcollisionee" || name=="minimumcompletetime" || name=="workspacetraj";
    return _bProcessing ? PE_Support : PE_Pass;
}

// called at the end of every XML tag, _ss contains the data
bool WorkspaceTrajectoryParameters::endElement(const std::string& name)
{
    // _ss is an internal stringstream that holds the data of the tag
    if( !!_pcurreader ) {
        if( _pcurreader->endElement(name) ) {
            xmlreaders::TrajectoryReaderPtr ptrajreader = boost::dynamic_pointer_cast<xmlreaders::TrajectoryReader>(_pcurreader);
            if( !!ptrajreader ) {
                workspacetraj = ptrajreader->GetTrajectory();
            }
            _pcurreader.reset();
        }
        return false;
    }
    if( _bProcessing ) {
        if( name == "maxdeviationangle") {
            _ss >> maxdeviationangle;
        }
        else if( name == "maintaintiming" ) {
            _ss >> maintaintiming;
        }
        else if( name == "greedysearch" ) {
            _ss >> greedysearch;
        }
        else if( name == "ignorefirstcollision" ) {
            _ss >> ignorefirstcollision;
        }
        else if( name == "ignorefirstcollisionee" ) {
            _ss >> ignorefirstcollisionee;
        }
        else if( name == "ignorelastcollisionee" ) {
            _ss >> ignorelastcollisionee;
        }
        else if( name == "minimumcompletetime" ) {
            _ss >> minimumcompletetime;
        }
        else if( name == "workspacetraj" ) {
            if( !workspacetraj ) {
                workspacetraj = RaveCreateTrajectory(_penv,"");
            }
            workspacetraj->deserialize(_ss);
        }
        else {
            RAVELOG_WARN(str(boost::format("unknown tag %s\n")%name));
        }
        _bProcessing = false;
        return false;
    }
    // give a chance for the default parameters to get processed
    return PlannerParameters::endElement(name);
}

void WorkspaceTrajectoryParameters::characters(const std::string& ch)
{
    if( !!_pcurreader ) {
        _pcurreader->characters(ch);
    }
    else {
        PlannerParameters::characters(ch);
    }
}

}
