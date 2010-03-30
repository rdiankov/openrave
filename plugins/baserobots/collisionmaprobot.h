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
#ifndef RAVE_COLLISIONMAP_ROBOT_H
#define RAVE_COLLISIONMAP_ROBOT_H

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
            boost::multi_array<uint8_t,N> vfreespace; // 1 for free space, 0 for collision
            boost::array<dReal,N> fmin, fmax, fidelta;
            boost::array<string,N> jointnames;
            boost::array<KinBody::JointPtr,N> joints;
        };
        typedef COLLISIONMAP<2> COLLISIONPAIR;
    XMLData() : XMLReadable("collisionmap") {}
        list<COLLISIONPAIR> listmaps;
    };

    class CollisionMapXMLReader : public BaseXMLReader
    {
    public:
        CollisionMapXMLReader(boost::shared_ptr<XMLData> cmdata, const std::list<std::pair<std::string,std::string> >& atts) {
            _cmdata = cmdata;
            if( !_cmdata )
                _cmdata.reset(new XMLData());
        }

        virtual XMLReadablePtr GetReadable() { return _cmdata; }
        
        virtual void startElement(const std::string& name, const std::list<std::pair<std::string,std::string> >& atts) {
            BaseXMLReader::startElement(name,atts);
            _ss.str(""); // have to clear the string

            if( name == "pair" ) {
                _cmdata->listmaps.push_back(XMLData::COLLISIONPAIR());
                XMLData::COLLISIONPAIR& pair = _cmdata->listmaps.back();
                for(std::list<std::pair<std::string,std::string> >::const_iterator itatt = atts.begin(); itatt != atts.end(); ++itatt) {
                    if( itatt->first == "dims" ) {
                        boost::array<size_t,2> dims={{0,0}};
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
                RAVELOG_VERBOSE(str(boost::format("creating self-collision pair: %s %s\n")%pair.jointnames[0]%pair.jointnames[1]));
            }
        }

        virtual bool endElement(const std::string& name)
        {
            BaseXMLReader::endElement(name);

            if( name == "collisionmap" )
                return true;
            else if( name == "pair" ) {
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
                if( !_ss )
                    RAVELOG_WARNA("failed to read collision pair values\n");
            }
            else
                RAVELOG_ERRORA("unknown field %s\n", name.c_str());

            return false;
        }

        virtual void characters(const std::string& ch)
        {
            BaseXMLReader::characters(ch);
            _ss.clear();
            _ss << ch;
        }

    protected:
        boost::shared_ptr<XMLData> _cmdata;
        stringstream _ss;
    };

    static BaseXMLReaderPtr CreateXMLReader(InterfaceBasePtr ptr, const std::list<std::pair<std::string,std::string> >& atts)
    {
        // ptr is the robot interface that this reader is being created for
        return BaseXMLReaderPtr(new CollisionMapXMLReader(boost::shared_ptr<XMLData>(),atts));
    }

    CollisionMapRobot(EnvironmentBasePtr penv) : RobotBase(penv) {
        __description = "Allows user to specify regions of the robot configuration space that are in self-collision via lookup tables. This is done by the <collisionmap> XML tag";
    }
    virtual ~CollisionMapRobot() {}

    virtual bool SetController(ControllerBasePtr p, const string& args)
    {
        _pController = p;
        if( !!_pController ) {
            if( !_pController->Init(shared_robot(),args) ) {
                RAVELOG_WARNA(str(boost::format("Failed to init controller\n")%GetName()));
                _pController.reset();
                return false;
            }
        }
        return true;
    }

    virtual void SetMotion(TrajectoryBaseConstPtr ptraj)
    {
        _trajcur = ptraj;
        if( _trajcur->GetPoints().size() == 0 ) {
            RAVELOG_WARNA("trajectory has no points\n");
            return;
        }

        if( _trajcur->GetDOF() != GetDOF() )
            throw openrave_exception(str(boost::format("trajectory of wrong dimension (traj dof=%d), needs to be %d dof\n")%_trajcur->GetDOF()%GetDOF()));
        _trajcur = ptraj;

        if( !!_pController )
            _pController->SetPath(_trajcur);
    }
 
    virtual void SetActiveMotion(TrajectoryBaseConstPtr ptraj)
    {
        if( ptraj->GetPoints().size() == 0 ) {
            RAVELOG_WARNA("trajectory has no points\n");
            return;
        }

        if( ptraj->GetDOF() != GetActiveDOF() ) {
            RAVELOG_WARNA("trajectory of wrong dimension (traj dof=%d), needs to be %d dof\n", ptraj->GetDOF(), GetActiveDOF());
            return;
        }
        BOOST_ASSERT( ptraj->GetDOF() == GetActiveDOF() );

        TrajectoryBasePtr pfulltraj = GetEnv()->CreateTrajectory(ptraj->GetDOF());
        GetFullTrajectoryFromActive(pfulltraj, ptraj);
        _trajcur = pfulltraj;

        if( !!_pController )
            _pController->SetPath(_trajcur);
    }

    virtual ControllerBasePtr GetController() const { return _pController; }
    virtual void SimulationStep(dReal fElapsedTime)
    {
        RobotBase::SimulationStep(fElapsedTime);
        if( !!_pController ) {
            _pController->SimulationStep(fElapsedTime);
        }
    }

    virtual void ComputeJointHierarchy()
    {
        RobotBase::ComputeJointHierarchy();
        boost::shared_ptr<XMLData> cmdata = boost::dynamic_pointer_cast<XMLData>(GetReadableInterface("collisionmap"));
        if( !!cmdata ) {
            // process the collisionmap structures
            FOREACH(itmap,cmdata->listmaps) {
                for(size_t i = 0; i < itmap->jointnames.size(); ++i) {
                    itmap->joints.at(i) = GetJoint(itmap->jointnames[i]);
                    itmap->fidelta.at(i) = (dReal)itmap->vfreespace.shape()[i]/(itmap->fmax.at(i)-itmap->fmin.at(i));
                    if( !itmap->joints[i] )
                        RAVELOG_WARN(str(boost::format("failed to find joint %s specified in collisionmap\n")%itmap->jointnames[i]));
                }
            }
        }
    }

    virtual bool CheckSelfCollision(CollisionReportPtr report = CollisionReportPtr()) const
    {
        if( RobotBase::CheckSelfCollision(report) )
            return true;

        // check if the current joint angles fall within the allowable range
        boost::shared_ptr<XMLData> cmdata = boost::dynamic_pointer_cast<XMLData>(GetReadableInterface("collisionmap"));
        if( !!cmdata ) {
            vector<dReal> values;
            boost::array<int,2> indices={{0,0}};
            FOREACHC(itmap,cmdata->listmaps) {
                size_t i=0;
                FOREACHC(itjoint,itmap->joints) {
                    if( !*itjoint )
                        break;
                    (*itjoint)->GetValues(values);
                    if( itmap->fmin[i] < itmap->fmax[i] ) {
                        int index = (int)((values.at(0)-itmap->fmin[i])*itmap->fidelta[i]);
                        if( index < 0 || index >= (int)itmap->vfreespace.shape()[i] )
                            break;
                        indices.at(i) = index;
                    }
                    ++i;
                }
                if( i != itmap->joints.size() )
                    continue;
                if( !itmap->vfreespace(indices) ) {
                    if( !!report ) {
                        report->numCols = 1;
                        report->vLinkColliding.resize(0);
                        FOREACHC(itjoint,itmap->joints) {
                            if( !!(*itjoint)->GetFirstAttached() && find(report->vLinkColliding.begin(),report->vLinkColliding.end(),(*itjoint)->GetFirstAttached())== report->vLinkColliding.end() )
                                report->vLinkColliding.push_back(KinBody::LinkConstPtr((*itjoint)->GetFirstAttached()));
                            if( !!(*itjoint)->GetSecondAttached() && find(report->vLinkColliding.begin(),report->vLinkColliding.end(),(*itjoint)->GetSecondAttached())== report->vLinkColliding.end() )
                                report->vLinkColliding.push_back(KinBody::LinkConstPtr((*itjoint)->GetSecondAttached()));
                        }
                    }
                    RAVELOG_VERBOSEA(str(boost::format("Self collision: joints %s:%s\n")%itmap->jointnames[0]%itmap->jointnames[1]));
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

#endif
