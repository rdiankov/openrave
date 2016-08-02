// -*- coding: utf-8 --*
// Copyright (C) 2012-2013 Rosen Diankov <rosen.diankov@gmail.com>
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
#include <openrave/xmlreaders.h>

class Conveyor : public RobotBase
{
public:
    class ConveyorLink : public Link
    {
public:
        ConveyorLink(const std::string& name, Transform tinernal, KinBodyPtr parent) : Link(parent) {
            _info._t = tinernal;
            _info._mass = 0.01; // just an estimate
            _info._vinertiamoments = Vector(1,1,1);
            _info._name = name;
            _info._bStatic = false;
            _info._bIsEnabled = true;
        }
    };

    class ConveyorJoint : public Joint
    {
public:
        ConveyorJoint(const std::string& name, TrajectoryBasePtr trajfollow, std::shared_ptr<KinBody::Mimic> mimic, bool bIsCircular, KinBodyPtr parent) : Joint(parent, KinBody::JointTrajectory) {
            _info._name = name;
            _info._vlowerlimit[0] = 0;
            _info._vupperlimit[0] = trajfollow->GetDuration();
            _vmimic[0] = mimic;
            _info._bIsCircular[0] = bIsCircular;
            _info._trajfollow = trajfollow;
        }

        virtual void _ComputeInternalInformation(LinkPtr plink0, LinkPtr plink1, dReal currentvalue)
        {
            std::vector<dReal> vcurrentvalues(1); vcurrentvalues[0] = 0; // current values always 0
            Joint::_ComputeInternalInformation(plink0, plink1, Vector(), std::vector<Vector>(), vcurrentvalues);
        }
    };

    class ConveyorInfo : public XMLReadable
    {
public:
        ConveyorInfo() : XMLReadable("conveyorjoint"), _fLinkDensity(10), _bIsCircular(true), _bCreated(false) {
        }
        std::shared_ptr<KinBody::Mimic> _mimic; // always has to mimic
        KinBody::LinkPtr _linkParent; ///< base link attached
        TrajectoryBasePtr _trajfollow; ///< trajectory to following in base link's coordinate system
        int _fLinkDensity; ///< number of links to create per unit of time in the trajectory
        std::list<GeometryInfo> _listGeometries; ///< geometry to attach to each child link
        std::string _namebase; ///< base name of joint
        bool _bIsCircular;

        bool _bCreated;
    };
    typedef std::shared_ptr<ConveyorInfo> ConveyorInfoPtr;

    class ConveyorXMLReader : public BaseXMLReader
    {
public:
        ConveyorXMLReader(ConveyorInfoPtr cmdata, RobotBasePtr probot, const AttributesList& atts) {
            _probot = probot;
            _cmdata = cmdata;
            if( !_cmdata ) {
                _cmdata.reset(new ConveyorInfo());
            }
            _cmdata->_mimic.reset(new KinBody::Mimic());
            FOREACHC(itatt,atts) {
                if( itatt->first == "name" ) {
                    _cmdata->_namebase = itatt->second;
                }
            }
        }

        virtual XMLReadablePtr GetReadable() {
            return _cmdata;
        }

        virtual ProcessElement startElement(const std::string& name, const AttributesList& atts) {
            if( !!_pcurreader ) {
                if( _pcurreader->startElement(name,atts) == PE_Support ) {
                    return PE_Support;
                }
                return PE_Ignore;
            }
            if( name == "trajectory" ) {
                _pcurreader.reset(new xmlreaders::TrajectoryReader(_probot->GetEnv(),_cmdata->_trajfollow,atts));
                return PE_Support;
            }
            else if( name == "geometry" ) {
                _pcurreader.reset(new xmlreaders::GeometryInfoReader(Link::GeometryInfoPtr(),atts));
                return PE_Support;
            }

            static boost::array<string, 6> tags = {{ "mimic_pos", "mimic_vel", "mimic_accel", "parentlink", "linkdensity", "circular" }};
            if( find(tags.begin(),tags.end(),name) == tags.end() ) {
                return PE_Pass;
            }
            _ss.str(""); // have to clear the string
            return PE_Support;
        }

        virtual bool endElement(const std::string& name)
        {
            if( !!_pcurreader ) {
                if( _pcurreader->endElement(name) ) {
                    xmlreaders::TrajectoryReaderPtr ptrajreader = std::dynamic_pointer_cast<xmlreaders::TrajectoryReader>(_pcurreader);
                    if( !!ptrajreader ) {
                        _cmdata->_trajfollow = ptrajreader->GetTrajectory();
                    }
                    else {
                        xmlreaders::GeometryInfoReaderPtr pgeomreader = std::dynamic_pointer_cast<xmlreaders::GeometryInfoReader>(_pcurreader);
                        if( !!pgeomreader ) {
                            _cmdata->_listGeometries.push_back(*pgeomreader->GetGeometryInfo());
                        }
                    }
                    _pcurreader.reset();
                }
                return false;
            }
            if( name == "mimic_pos" ) {
                _cmdata->_mimic->_equations[0] = _ss.str();
            }
            else if( name == "mimic_vel" ) {
                _cmdata->_mimic->_equations[1] = _ss.str();
            }
            else if( name == "mimic_accel" ) {
                _cmdata->_mimic->_equations[2] = _ss.str();
            }
            else if( name == "parentlink" ) {
                string linkname;
                _ss >> linkname;
                _cmdata->_linkParent = _probot->GetLink(linkname);
                if( !_cmdata->_linkParent ) {
                    RAVELOG_WARN(str(boost::format("failed to find link %s")%linkname));
                }
            }
            else if( name == "linkdensity" ) {
                _ss >> _cmdata->_fLinkDensity;
            }
            else if( name == "circular" ) {
                string s; _ss >> s;
                _cmdata->_bIsCircular = !(s=="false" || s=="0");
            }
            else if( name == "conveyorjoint" ) {
                return true;
            }
            else {
                RAVELOG_ERROR(str(boost::format("unknown field %s")%name));
            }
            return false;
        }

        virtual void characters(const std::string& ch)
        {
            if( !!_pcurreader ) {
                _pcurreader->characters(ch);
            }
            else {
                _ss.clear();
                _ss << ch;
            }
        }

protected:
        BaseXMLReaderPtr _pcurreader;
        RobotBasePtr _probot;
        ConveyorInfoPtr _cmdata;
        stringstream _ss;
    };

    static BaseXMLReaderPtr CreateXMLReader(InterfaceBasePtr ptr, const AttributesList& atts)
    {
        // ptr is the robot interface that this reader is being created for
        return BaseXMLReaderPtr(new ConveyorXMLReader(ConveyorInfoPtr(),RaveInterfaceCast<RobotBase>(ptr), atts));
    }

    Conveyor(EnvironmentBasePtr penv, std::istream& is) : RobotBase(penv) {
        __description = ":Interface Author: Rosen Diankov\n\nParses conveyor joints as a trajectory and adds child links to form a full conveyor system. Use the <conveyorjoint> tag to specify the conveyor properties.";
    }
    virtual ~Conveyor() {
    }

    virtual bool SetController(ControllerBasePtr controller, const std::vector<int>& jointindices, int nControlTransformation)
    {
        _pController = controller;
        if( !!_pController ) {
            if( !_pController->Init(shared_robot(),jointindices,nControlTransformation) ) {
                RAVELOG_WARN(str(boost::format("GenericRobot %s : Failed to init controller %s\n")%GetName()%controller->GetXMLId()));
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
        // create extra joints for each conveyor joint
        ConveyorInfoPtr cmdata = std::dynamic_pointer_cast<ConveyorInfo>(GetReadableInterface("conveyorjoint"));
        if( !!cmdata && !cmdata->_bCreated ) {
            if( !_pController ) {
                _pController = RaveCreateController(GetEnv(), "IdealVelocityController");
            }
            // add conveyor joints and links
            dReal timestep = 1.0/cmdata->_fLinkDensity;
            int numchildlinks = static_cast<int>(cmdata->_trajfollow->GetDuration()*cmdata->_fLinkDensity);
            dReal curtime = 0;
            std::vector<dReal> vsampledata;
            Transform tparent = cmdata->_linkParent->GetTransform();
            for(int ichild = 0; ichild < numchildlinks; ++ichild, curtime += timestep) {
                std::shared_ptr<ConveyorLink> pchildlink(new ConveyorLink(str(boost::format("__moving__%s%d")%cmdata->_namebase%ichild), tparent, shared_kinbody()));
                pchildlink->InitGeometries(cmdata->_listGeometries);
                _veclinks.push_back(pchildlink);

                std::shared_ptr<KinBody::Mimic> mimic(new KinBody::Mimic());
                *mimic = *cmdata->_mimic;
                mimic->_equations[0] += str(boost::format(" +%.15e")%curtime); // have to add the offset inside the equations
                std::shared_ptr<ConveyorJoint> pchildjoint(new ConveyorJoint(pchildlink->GetName(), cmdata->_trajfollow, mimic, cmdata->_bIsCircular, shared_kinbody()));
                _vecjoints.push_back(pchildjoint);
                pchildjoint->_ComputeInternalInformation(cmdata->_linkParent, pchildlink, curtime);
            }
            cmdata->_bCreated = true;
        }

        RobotBase::_ComputeInternalInformation();

        std::vector<int> dofindices(GetDOF());
        for(int i = 0; i < GetDOF(); ++i) {
            dofindices[i] = i;
        }
        if( !_pController->Init(shared_robot(),dofindices,0) ) {
            RAVELOG_WARN(str(boost::format("GenericRobot %s : Failed to init controller %s\n")%GetName()%_pController->GetXMLId()));
            _pController.reset();
        }
    }

protected:
    TrajectoryBaseConstPtr _trajcur;
    ControllerBasePtr _pController;

    static UserDataPtr s_registeredhandle;
};

RobotBasePtr CreateConveyorRobot(EnvironmentBasePtr penv, std::istream& sinput)
{
    return RobotBasePtr(new Conveyor(penv,sinput));
}

void RegisterConveyorReaders(std::list< UserDataPtr >& listRegisteredReaders)
{
    listRegisteredReaders.push_back(RaveRegisterXMLReader(PT_Robot,"conveyorjoint",Conveyor::CreateXMLReader));
}
