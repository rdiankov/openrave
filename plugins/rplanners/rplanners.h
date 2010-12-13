// -*- coding: utf-8 -*-
// Copyright (C) 2006-2010 Rosen Diankov (rosen.diankov@gmail.com)
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
#ifndef RAVE_PLANNERS_H
#define RAVE_PLANNERS_H

#include "plugindefs.h"
#include "plannerparameters.h"

enum ExtendType {
    ET_Failed=0,
    ET_Sucess=1,
    ET_Connected=2
};

class CollisionFunctions
{
 public:
    static bool CheckCollision(PlannerBase::PlannerParametersConstPtr params, RobotBasePtr robot, const vector<dReal>& pQ0, const vector<dReal>& pQ1, IntervalType interval, vector< vector<dReal> >* pvCheckedConfigurations = NULL)
    {
        // set the bounds based on the interval type
        int start=0;
        bool bCheckEnd=false;
        switch (interval) {
        case IT_Open:
            start = 1;  bCheckEnd = false;
            break;
        case IT_OpenStart:
            start = 1;  bCheckEnd = true;
            break;
        case IT_OpenEnd:
            start = 0;  bCheckEnd = false;
            break;
        case IT_Closed:
            start = 0;  bCheckEnd = true;
            break;
        default:
            BOOST_ASSERT(0);
        }

        // first make sure the end is free
        vector<dReal> vlastconfig(params->GetDOF()), vtempconfig(params->GetDOF());
        if (bCheckEnd) {
            params->_setstatefn(pQ1);
            if (robot->GetEnv()->CheckCollision(KinBodyConstPtr(robot)) || (robot->CheckSelfCollision()) ) {
                if( pvCheckedConfigurations != NULL ) {
                    pvCheckedConfigurations->push_back(pQ1);
                }
                return true;
            }
        }

        // compute  the discretization
        vector<dReal> dQ = pQ1;
        params->_diffstatefn(dQ,pQ0);
        int i, numSteps = 1;
        vector<dReal>::const_iterator itres = params->_vConfigResolution.begin();
        for (i = 0; i < params->GetDOF(); i++,itres++) {
            int steps;
            if( *itres != 0 )
                steps = (int)(fabs(dQ[i]) / *itres);
            else
                steps = (int)(fabs(dQ[i]) * 100);
            if (steps > numSteps)
                numSteps = steps;
        }
        dReal fisteps = dReal(1.0f)/numSteps;
        FOREACH(it,dQ)
            *it *= fisteps;

        if( !!params->_constraintfn )
            vlastconfig = pQ0;
        // check for collision along the straight-line path
        // NOTE: this does not check the end config, and may or may
        // not check the start based on the value of 'start'
        for (int f = start; f < numSteps; f++) {
            for (i = 0; i < params->GetDOF(); i++) {
                vtempconfig[i] = pQ0[i] + (dQ[i] * f);
            }
            params->_setstatefn(vtempconfig);
            if( !!params->_constraintfn ) {
                if( !params->_constraintfn(vlastconfig,vtempconfig,0) ) {
                    return true;
                }
                vlastconfig = pQ0;
            }
            if( pvCheckedConfigurations != NULL ) {
                params->_getstatefn(vtempconfig); // query again in order to get normalizations/joint limits
                pvCheckedConfigurations->push_back(vtempconfig);
            }
            if( robot->GetEnv()->CheckCollision(KinBodyConstPtr(robot)) || (robot->CheckSelfCollision()) ) {
                return true;
            }
        }

        if( bCheckEnd && pvCheckedConfigurations != NULL ) {
            pvCheckedConfigurations->push_back(pQ1);
        }
        return false;
    }

    /// check collision between body and environment
    static bool CheckCollision(PlannerBase::PlannerParametersConstPtr params, RobotBasePtr robot, const vector<dReal>& pConfig, CollisionReportPtr report=CollisionReportPtr())
    {
        params->_setstatefn(pConfig);
        bool bCol = robot->GetEnv()->CheckCollision(KinBodyConstPtr(robot), report) || (robot->CheckSelfCollision(report));
        if( bCol && !!report ) {
            RAVELOG_WARN(str(boost::format("fcollision %s\n")%report->__str__()));
        }
        return bCol;
    }
};

class SimpleCostMetric
{
 public:
    SimpleCostMetric(RobotBasePtr robot) {}
    virtual float Eval(const vector<dReal>& pConfiguration) { return 1; }
};

class SimpleGoalMetric
{
public:       
 SimpleGoalMetric(RobotBasePtr robot, dReal thresh=0.01f) : _robot(robot), _thresh(thresh) {}
    
    //checks if pConf is within this cone (note: only works in 3D)
    dReal Eval(const vector<dReal>& c1)
    {
        _robot->SetActiveDOFValues(c1);
        Transform cur = _robot->GetActiveManipulator()->GetEndEffectorTransform();
        dReal f = RaveSqrt((tgoal.trans - cur.trans).lengthsqr3());
        return f < _thresh ? 0 : f;
    }

    Transform tgoal; // workspace goal

 private:
    RobotBasePtr _robot;
    dReal _thresh;
};

struct SimpleNode
{
SimpleNode(int parent, const vector<dReal>& q) : parent(parent), q(q) {}
    int parent;
    vector<dReal> q; // the configuration immediately follows the struct
};

class SpatialTreeBase
{
 public:
    virtual int AddNode(int parent, const vector<dReal>& config) = 0;
    virtual int GetNN(const vector<dReal>& q) = 0;
    virtual const vector<dReal>& GetConfig(int inode) = 0;
    virtual ExtendType Extend(const vector<dReal>& pTargetConfig, int& lastindex, bool bOneStep=false) = 0;
    virtual int GetDOF() = 0;
};

template <typename Planner, typename Node>
class SpatialTree : public SpatialTreeBase
{
 public:
    SpatialTree() {
        _fStepLength = 0.04f;
        _dof = 0;
        _fBestDist = 0;
        _nodes.reserve(5000);
    }

    ~SpatialTree(){}
    
    virtual void Reset(boost::weak_ptr<Planner> planner, int dof=0)
    {
        _planner = planner;

        typename vector<Node*>::iterator it;
        FORIT(it, _nodes)
            delete *it;
        _nodes.resize(0);

        if( dof > 0 ) {
            _vNewConfig.resize(dof);
            _dof = dof;
        }   
    }

    virtual int AddNode(int parent, const vector<dReal>& config)
    {
        _nodes.push_back(new Node(parent,config));
        return (int)_nodes.size()-1;
    }

    ///< return the nearest neighbor
    virtual int GetNN(const vector<dReal>& q)
    {
        if( _nodes.size() == 0 ) {
            return -1;
        }
        int ibest = -1;
        dReal fbest = 0;
        FOREACH(itnode, _nodes) {
            dReal f = _distmetricfn(q, (*itnode)->q);
            if( ibest < 0 || f < fbest ) {
                ibest = (int)(itnode-_nodes.begin());
                fbest = f;
            }
        }

        _fBestDist = fbest;
        return ibest;
    }

    /// extends toward pNewConfig
    /// \return true if extension reached pNewConfig
    virtual ExtendType Extend(const vector<dReal>& pTargetConfig, int& lastindex, bool bOneStep=false)
    {
        // get the nearest neighbor
        lastindex = GetNN(pTargetConfig);
        if( lastindex < 0 ) {
            return ET_Failed;
        }
        Node* pnode = _nodes.at(lastindex);
        bool bHasAdded = false;
        boost::shared_ptr<Planner> planner(_planner);
        PlannerBase::PlannerParametersConstPtr params = planner->GetParameters();
        // extend
        while(1) {
            dReal fdist = _distmetricfn(pTargetConfig,pnode->q);
            if( fdist > _fStepLength ) {
                fdist = _fStepLength / fdist;
            }
            else if( fdist <= dReal(0.1) * _fStepLength ) { // return connect if the distance is very close
                return ET_Connected;
            }
        
            _vNewConfig = pTargetConfig;
            params->_diffstatefn(_vNewConfig,pnode->q);
            for(int i = 0; i < _dof; ++i)
                _vNewConfig[i] = pnode->q[i] + _vNewConfig[i]*fdist;
        
            // project to constraints
            if( !!params->_constraintfn ) {
                params->_setstatefn(_vNewConfig);
                if( !params->_constraintfn(pnode->q, _vNewConfig, 0) ) {
                    if(bHasAdded) {
                        return ET_Sucess;
                    }
                    return ET_Failed;
                }

                // it could be the case that the node didn't move anywhere, in which case we would go into an infinite loop
                if( _distmetricfn(pnode->q, _vNewConfig) <= dReal(0.01)*_fStepLength ) {
                    if(bHasAdded) {
                        return ET_Sucess;
                    }
                    return ET_Failed;
                }
            }

            if( CollisionFunctions::CheckCollision(params,planner->GetRobot(),pnode->q, _vNewConfig, IT_OpenStart) ) {
                if(bHasAdded) {
                    return ET_Sucess;
                }
                return ET_Failed;
            }

            lastindex = AddNode(lastindex, _vNewConfig);
            pnode = _nodes[lastindex];
            bHasAdded = true;
            if( bOneStep ) {
                return ET_Connected;
            }
        }
    
        return ET_Failed;
    }

    virtual const vector<dReal>& GetConfig(int inode) { return _nodes.at(inode)->q; }
    virtual int GetDOF() { return _dof; }

    vector<Node*> _nodes;
    boost::function<dReal(const std::vector<dReal>&, const std::vector<dReal>&)> _distmetricfn;

    dReal _fBestDist; ///< valid after a call to GetNN
    dReal _fStepLength;

 private:
    vector<dReal> _vNewConfig;
    boost::weak_ptr<Planner> _planner;
    int _dof;
};

class GraspSetParameters : public PlannerBase::PlannerParameters
{
public:
GraspSetParameters(EnvironmentBasePtr penv) : _nGradientSamples(5), _fVisibiltyGraspThresh(0), _fGraspDistThresh(1.4f), _penv(penv),_bProcessingGS(false) {
        _vXMLParameters.push_back("grasps");
        _vXMLParameters.push_back("target");
        _vXMLParameters.push_back("numgradsamples");
        _vXMLParameters.push_back("visgraspthresh");
        _vXMLParameters.push_back("graspdistthresh");
    }
    
    vector<Transform> _vgrasps; ///< grasps with respect to the target object
    KinBodyPtr _ptarget;
    int _nGradientSamples;
    dReal _fVisibiltyGraspThresh; ///< if current grasp is less than this threshold, then visibilty is not checked
    dReal _fGraspDistThresh; ///< target grasps beyond this distance are ignored

 protected:
    EnvironmentBasePtr _penv;
    bool _bProcessingGS;
    virtual bool serialize(std::ostream& O) const
    {
        if( !PlannerParameters::serialize(O) )
            return false;
        O << "<grasps>" << _vgrasps.size() << " ";
        FOREACHC(it, _vgrasps)
            O << *it << " ";
        O << "</grasps>" << endl;
        O << "<target>" << (!!_ptarget?_ptarget->GetEnvironmentId():0) << "</target>" << endl;
        O << "<numgradsamples>" << _nGradientSamples << "</numgradsamples>" << endl;
        O << "<visgraspthresh>" << _fVisibiltyGraspThresh << "</visgraspthresh>" << endl;
        O << "<graspdistthresh>" << _fGraspDistThresh << "</graspdistthresh>" << endl;
        return !!O;
    }

    ProcessElement startElement(const std::string& name, const std::list<std::pair<std::string,std::string> >& atts)
    {
        if( _bProcessingGS )
            return PE_Ignore;
        switch( PlannerBase::PlannerParameters::startElement(name,atts) ) {
            case PE_Pass: break;
            case PE_Support: return PE_Support;
            case PE_Ignore: return PE_Ignore;
        }
        
        _bProcessingGS = name=="grasps"||name=="target"||name=="numgradsamples"||name=="visgraspthresh"||name=="graspdistthresh";
        return _bProcessingGS ? PE_Support : PE_Pass;
    }
    
    virtual bool endElement(const string& name)
    {
        if( _bProcessingGS ) {
            if( name == "grasps" ) {
                int ngrasps=0;
                _ss >> ngrasps;
                _vgrasps.resize(ngrasps);
                FOREACH(it, _vgrasps) {
                    _ss >> *it;
                }
            }
            else if( name == "target" ) {
                int id = 0;
                _ss >> id;
                _ptarget = _penv->GetBodyFromEnvironmentId(id);
            }
            else if( name == "numgradsamples" )
                _ss >> _nGradientSamples;
            else if( name == "visgraspthresh" )
                _ss >> _fVisibiltyGraspThresh;
            else if( name == "graspdistthresh")
                _ss >> _fGraspDistThresh;
            else
                RAVELOG_WARN(str(boost::format("unknown tag %s\n")%name));
            _bProcessingGS = false;
            return false;
        }

        // give a chance for the default parameters to get processed
        return PlannerParameters::endElement(name);
    }
};

class BasicRRTParameters : public PlannerBase::PlannerParameters
{
public:
BasicRRTParameters() : _fGoalBiasProb(0.05f), _bProcessing(false) {
        _vXMLParameters.push_back("goalbias");
    }
    
    dReal _fGoalBiasProb; 

 protected:
    bool _bProcessing;
    virtual bool serialize(std::ostream& O) const
    {
        if( !PlannerParameters::serialize(O) ) {
            return false;
        }
        O << "<goalbias>" << _fGoalBiasProb << "</goalbias>" << endl;
        return !!O;
    }

    ProcessElement startElement(const std::string& name, const std::list<std::pair<std::string,std::string> >& atts)
    {
        if( _bProcessing ) {
            return PE_Ignore;
        }
        switch( PlannerBase::PlannerParameters::startElement(name,atts) ) {
            case PE_Pass: break;
            case PE_Support: return PE_Support;
            case PE_Ignore: return PE_Ignore;
        }
        
        _bProcessing = name=="goalbias";
        return _bProcessing ? PE_Support : PE_Pass;
    }
    
    virtual bool endElement(const string& name)
    {
        if( _bProcessing ) {
            if( name == "goalbias") {
                _ss >> _fGoalBiasProb;
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
};

inline dReal TransformDistance2(const Transform& t1, const Transform& t2, dReal frotweight=1, dReal ftransweight=1)
{
    //dReal facos = RaveAcos(min(dReal(1),RaveFabs(dot4(t1.rot,t2.rot))));
    dReal facos = min((t1.rot-t2.rot).lengthsqr4(),(t1.rot+t2.rot).lengthsqr4());
    return (t1.trans-t2.trans).lengthsqr3() + frotweight*facos;//*facos;
}

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(SimpleNode)
BOOST_TYPEOF_REGISTER_TYPE(SpatialTree)
#endif

#endif
