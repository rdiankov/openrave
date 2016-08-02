// -*- coding: utf-8 --*
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
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
#include "plugindefs.h"

class CollisionMapRobot : public RobotBase
{
public:
    class XMLData : public XMLReadable
    {
public:
        /// specifies the free space of two joints
        template <int N>
        struct COLLISIONMAP
        {
            boost::multi_array<uint8_t,N> vfreespace;         // 1 for free space, 0 for collision
            boost::array<dReal,N> fmin, fmax, fidelta;
            boost::array<string,N> jointnames;
            boost::array<int,N> jointindices;
        };
        typedef COLLISIONMAP<2> COLLISIONPAIR;
        XMLData() : XMLReadable("collisionmap") {
        }
        list<COLLISIONPAIR> listmaps;
    };

    class CollisionMapXMLReader : public BaseXMLReader
    {
public:
        CollisionMapXMLReader(std::shared_ptr<XMLData> cmdata, const AttributesList& atts) {
            _cmdata = cmdata;
            if( !_cmdata ) {
                _cmdata.reset(new XMLData());
            }
        }

        virtual XMLReadablePtr GetReadable() {
            return _cmdata;
        }

        virtual ProcessElement startElement(const std::string& name, const AttributesList& atts) {
            _ss.str("");         // have to clear the string
            if( name == "pair" ) {
                _cmdata->listmaps.push_back(XMLData::COLLISIONPAIR());
                XMLData::COLLISIONPAIR& pair = _cmdata->listmaps.back();
                for(AttributesList::const_iterator itatt = atts.begin(); itatt != atts.end(); ++itatt) {
                    if( itatt->first == "dims" ) {
                        boost::array<size_t,2> dims={ { 0,0}};
                        stringstream ss(itatt->second);
                        ss >> dims[0] >> dims[1];
                        pair.vfreespace.resize(dims);
                    }
                    else if( itatt->first == "min" ) {
                        stringstream ss(itatt->second);
                        ss >> pair.fmin[0] >> pair.fmin[1];
                    }
                    else if( itatt->first == "max" ) {
                        stringstream ss(itatt->second);
                        ss >> pair.fmax[0] >> pair.fmax[1];
                    }
                    else if( itatt->first == "joints") {
                        stringstream ss(itatt->second);
                        ss >> pair.jointnames[0] >> pair.jointnames[1];
                    }
                }
                RAVELOG_VERBOSE(str(boost::format("creating self-collision pair: %s %s")%pair.jointnames[0]%pair.jointnames[1]));
                return PE_Support;
            }

            return PE_Pass;
        }

        virtual bool endElement(const std::string& name)
        {
            if( name == "pair" ) {
                BOOST_ASSERT(_cmdata->listmaps.size()>0);
                XMLData::COLLISIONPAIR& pair = _cmdata->listmaps.back();
                for(size_t i = 0; i < pair.vfreespace.shape()[0]; i++) {
                    for(size_t j = 0; j < pair.vfreespace.shape()[1]; j++) {
                        // have to read with an int, uint8_t gives bugs!
                        int freespace;
                        _ss >> freespace;
                        pair.vfreespace[i][j] = freespace;
                    }
                }
                if( !_ss ) {
                    RAVELOG_WARN("failed to read collision pair values\n");
                }
            }
            else if( name == "collisionmap" ) {
                return true;
            }
            else {
                RAVELOG_ERROR(str(boost::format("unknown field %s")%name));
            }
            return false;
        }

        virtual void characters(const std::string& ch)
        {
            _ss.clear();
            _ss << ch;
        }

protected:
        std::shared_ptr<XMLData> _cmdata;
        stringstream _ss;
    };

    static BaseXMLReaderPtr CreateXMLReader(InterfaceBasePtr ptr, const AttributesList& atts)
    {
        // ptr is the robot interface that this reader is being created for
        return BaseXMLReaderPtr(new CollisionMapXMLReader(std::shared_ptr<XMLData>(),atts));
    }

    CollisionMapRobot(EnvironmentBasePtr penv, std::istream& sinput) : RobotBase(penv) {
        __description = ":Interface Author: Rosen Diankov\n\nAllows user to specify regions of the robot configuration space that are in self-collision via lookup tables. This is most commonly used when two or more joints are coupled and their joint limits cannot be specified by simple min/max limits. A CollisionMap robot allows the user to specify self-collision regions indexed by the values of two joints.\n\n\
The map will be 1 if the values are in free space (allowed) or 0 if they are in self-collision. If the robot gets into a 0 region, it will get into self-collision.\n\n\
This is done by first creating a robot of type 'CollisionMapRobot' and using the **<collisionmap>** XML tag. Inside the **<collisionmap>** tag, multiple **<pair>** tags can be specified for coupled joints. For example, to specify a 181x181 2D map for joints J0, J1, J2, J3 where J0,J1 are paired and J2,J3 are paired, do: \n\n\
.. code-block:: xml\n\n\
  <robot type=\"CollisionMapRobot\">\n\
    ...\n\
    <collisionmap>\n\
      <pair dims=\"181 181\" min=\"-1.5708 -1.5708\" max=\"1.5708 1.5708\" joints=\"J0 J1\">\n\
        <!-- 181x181 values of 0s and 1s -->\n\
      </pair>\n\
      <pair dims=\"91 131\" min=\"-1 -2\" max=\"1 2\" joints=\"J2 J3\">\n\
        <!-- 91x131 values of 0s and 1s -->\n\
      </pair>\n\
    </collisionmap>\n\
  </robot>\n\n\
The first pair specifies a map where both joints J0 and J1 have range [-1.5708, 1.5708]. This is usually the joint limits specified in the **<joint>** definition.\n\nIn order to index into the two maps defined above, the following operation is performed::\n\n\
  pair_J0xJ1[ 180*(J0+1.57)/(1.57+1.57) ][ 180*(J1+1.57)/(1.57+1.57) ]\n\n\
For joints J2xJ3, the index operation is::\n\n\
  pair_J2xJ3[ 90*(J2+1)/(1+1) ][ 130*(J3+2)/(2+2) ]\n\n\
";
    }
    virtual ~CollisionMapRobot() {
    }

    virtual bool SetController(ControllerBasePtr controller, const std::vector<int>& jointindices, int nControlTransformation)
    {
        _pController = controller;
        if( !!_pController ) {
            if( !_pController->Init(shared_robot(),jointindices,nControlTransformation) ) {
                RAVELOG_WARN(str(boost::format("GenericRobot %s: Failed to init controller %s")%GetName()%controller->GetXMLId()));
                _pController.reset();
                return false;
            }
        }
        return true;
    }

    virtual ControllerBasePtr GetController() const {
        return _pController;
    }
    virtual void SimulationStep(dReal fElapsedTime)
    {
        RobotBase::SimulationStep(fElapsedTime);
        if( !!_pController ) {
            _pController->SimulationStep(fElapsedTime);
        }
    }

    virtual void _ComputeInternalInformation()
    {
        RobotBase::_ComputeInternalInformation();
        std::shared_ptr<XMLData> cmdata = std::dynamic_pointer_cast<XMLData>(GetReadableInterface("collisionmap"));
        if( !!cmdata ) {
            // process the collisionmap structures
            FOREACH(itmap,cmdata->listmaps) {
                for(size_t i = 0; i < itmap->jointnames.size(); ++i) {
                    JointPtr pjoint = GetJoint(itmap->jointnames[i]);
                    itmap->fidelta.at(i) = (dReal)itmap->vfreespace.shape()[i]/(itmap->fmax.at(i)-itmap->fmin.at(i));
                    if( !pjoint ) {
                        itmap->jointindices.at(i) = -1;
                        RAVELOG_WARN(str(boost::format("failed to find joint %s specified in collisionmap")%itmap->jointnames[i]));
                    }
                    else {
                        itmap->jointindices.at(i) = pjoint->GetJointIndex();
                    }
                }
            }
        }
    }

    virtual bool CheckSelfCollision(CollisionReportPtr report = CollisionReportPtr(), CollisionCheckerBasePtr collisionchecker=CollisionCheckerBasePtr()) const
    {
        if( RobotBase::CheckSelfCollision(report, collisionchecker) ) {
            return true;
        }
        // check if the current joint angles fall within the allowable range
        std::shared_ptr<XMLData> cmdata = std::dynamic_pointer_cast<XMLData>(GetReadableInterface("collisionmap"));
        if( !!cmdata ) {
            vector<dReal> values;
            boost::array<int,2> indices={ { 0,0}};
            FOREACHC(itmap,cmdata->listmaps) {
                size_t i=0;
                const XMLData::COLLISIONPAIR& curmap = *itmap;     // for debugging
                FOREACHC(itjindex,curmap.jointindices) {
                    if( *itjindex < 0 ) {
                        break;
                    }
                    GetJoints().at(*itjindex)->GetValues(values);
                    if( curmap.fmin[i] < curmap.fmax[i] ) {
                        int index = (int)((values.at(0)-curmap.fmin[i])*curmap.fidelta[i]);
                        if( index < 0 || index >= (int)curmap.vfreespace.shape()[i] ) {
                            break;
                        }
                        indices.at(i) = index;
                    }
                    ++i;
                }
                if( i != curmap.jointindices.size() ) {
                    continue;
                }
                if( !curmap.vfreespace(indices) ) {
                    // get all colliding links and check to make sure that at least two are enabled
                    vector< std::pair<LinkConstPtr, LinkConstPtr> > vLinkColliding;
                    FOREACHC(itjindex,curmap.jointindices) {
                        JointPtr pjoint = GetJoints().at(*itjindex);
                        if( !!pjoint->GetFirstAttached() && !!pjoint->GetSecondAttached() ) {
                            std::pair<LinkConstPtr, LinkConstPtr> links(pjoint->GetFirstAttached(), pjoint->GetSecondAttached());
                            if( links.first->IsEnabled() && links.second->IsEnabled() ) {
                                if( links.second->GetIndex() < links.first->GetIndex() ) {
                                    std::swap(links.first, links.second);
                                }
                                if( find(vLinkColliding.begin(),vLinkColliding.end(), links) == vLinkColliding.end() ) {
                                    vLinkColliding.push_back(links);
                                }
                            }
                        }
                    }
                    if( vLinkColliding.size() == 0 ) {
                        continue;
                    }
                    if( !!report ) {
                        report->vLinkColliding = vLinkColliding;
                        if( vLinkColliding.size() > 0 ) {
                            report->plink1 = vLinkColliding.at(0).first;
                            report->plink2 = vLinkColliding.at(0).second;
                        }
                    }
                    RAVELOG_VERBOSE_FORMAT("Self collision: joints %s(%d):%s(%d)", curmap.jointnames[0]%indices[0]%curmap.jointnames[1]%indices[1]);
                    return true;
                }
            }
        }
        return false;
    }

protected:
    TrajectoryBaseConstPtr _trajcur;
    ControllerBasePtr _pController;
};

RobotBasePtr CreateCollisionMapRobot(EnvironmentBasePtr penv, std::istream& sinput)
{
    return RobotBasePtr(new CollisionMapRobot(penv,sinput));
}

void RegisterCollisionMapRobotReaders(std::list< UserDataPtr >& listRegisteredReaders)
{
    listRegisteredReaders.push_back(RaveRegisterXMLReader(PT_Robot,"collisionmap",CollisionMapRobot::CreateXMLReader));
}
