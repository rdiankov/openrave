// -*- coding: utf-8 -*-
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
#ifndef RAVE_PLANNERS_H
#define RAVE_PLANNERS_H

#include "plugindefs.h"

enum ExtendType {
    ET_Failed=0,
    ET_Sucess=1,
    ET_Connected=2
};

class SimpleCostMetric
{
public:
    SimpleCostMetric(RobotBasePtr robot) {
    }
    virtual float Eval(const vector<dReal>& pConfiguration) {
        return 1;
    }
};

class SimpleGoalMetric
{
public:
    SimpleGoalMetric(RobotBasePtr robot, dReal thresh=0.01f) : _robot(robot), _thresh(thresh) {
    }

    //checks if pConf is within this cone (note: only works in 3D)
    dReal Eval(const vector<dReal>& c1)
    {
        _robot->SetActiveDOFValues(c1);
        Transform cur = _robot->GetActiveManipulator()->GetTransform();
        dReal f = RaveSqrt((tgoal.trans - cur.trans).lengthsqr3());
        return f < _thresh ? 0 : f;
    }

    Transform tgoal;     // workspace goal

private:
    RobotBasePtr _robot;
    dReal _thresh;
};

struct SimpleNode
{
    SimpleNode(int parent, const vector<dReal>& q) : parent(parent), q(q) {
    }
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
    SpatialTree(int fromgoal) {
        _fromgoal = fromgoal;
        _fStepLength = 0.04f;
        _dof = 0;
        _fBestDist = 0;
        _nodes.reserve(5000);
    }

    ~SpatialTree(){
    }

    virtual void Reset(boost::weak_ptr<Planner> planner, int dof=0)
    {
        _planner = planner;
        typename vector<Node*>::iterator it;
        FORIT(it, _nodes) {
            delete *it;
        }
        _nodes.resize(0);

        if( dof > 0 ) {
            _vNewConfig.resize(dof);
            _vDeltaConfig.resize(dof);
            _dof = dof;
        }
    }

    virtual int AddNode(int parent, const vector<dReal>& config)
    {
        _nodes.push_back(new Node(parent,config));
        return (int)_nodes.size()-1;
    }

    /// deletes all nodes that have parentindex as their parent
    virtual void DeleteNodesWithParent(int parentindex)
    {
        if( _vchildindices.capacity() == 0 ) {
            _vchildindices.reserve(128);
        }
        _vchildindices.resize(0); _vchildindices.push_back(parentindex);
        for(size_t i = 0; i < _nodes.size(); ++i) {
            if( !!_nodes[i] && std::binary_search(_vchildindices.begin(),_vchildindices.end(),_nodes[i]->parent) ) {
                _vchildindices.push_back(i);
                delete _nodes[i]; _nodes[i] = NULL;
            }
        }
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
            if( !!*itnode && (*itnode)->parent != (int)0x80000000) {
                dReal f = _distmetricfn(q, (*itnode)->q);
                if(( ibest < 0) ||( f < fbest) ) {
                    ibest = (int)(itnode-_nodes.begin());
                    fbest = f;
                }
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
        for(int iter = 0; iter < 100; ++iter) {     // to avoid infinite loops

            dReal fdist = _distmetricfn(pTargetConfig,pnode->q);
            if( fdist > _fStepLength ) {
                fdist = _fStepLength / fdist;
            }
            else if( fdist <= dReal(0.01) * _fStepLength ) {
                // return connect if the distance is very close
                return ET_Connected;
            }
            else {
                fdist = 1;
            }

            _vNewConfig = pnode->q;
            _vDeltaConfig = pTargetConfig;
            params->_diffstatefn(_vDeltaConfig,pnode->q);
            for(int i = 0; i < _dof; ++i) {
                _vDeltaConfig[i] *= fdist;
            }
            params->_setstatefn(_vNewConfig);
            if( !params->_neighstatefn(_vNewConfig,_vDeltaConfig,_fromgoal) ) {
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

            if( !params->_checkpathconstraintsfn(_fromgoal ? _vNewConfig : pnode->q, _fromgoal ? pnode->q : _vNewConfig, _fromgoal ? IT_OpenEnd : IT_OpenStart, PlannerBase::ConfigurationListPtr()) ) {
                return bHasAdded ? ET_Sucess : ET_Failed;
            }

            lastindex = AddNode(lastindex, _vNewConfig);
            pnode = _nodes[lastindex];
            bHasAdded = true;
            if( bOneStep ) {
                return ET_Connected;
            }
        }

        return bHasAdded ? ET_Sucess : ET_Failed;
    }

    virtual const vector<dReal>& GetConfig(int inode) {
        return _nodes.at(inode)->q;
    }
    virtual int GetDOF() {
        return _dof;
    }

    vector<Node*> _nodes;
    boost::function<dReal(const std::vector<dReal>&, const std::vector<dReal>&)> _distmetricfn;

    dReal _fBestDist;     ///< valid after a call to GetNN
    dReal _fStepLength;

private:
    vector<int> _vchildindices;
    vector<dReal> _vNewConfig, _vDeltaConfig;
    boost::weak_ptr<Planner> _planner;
    int _dof, _fromgoal;
};

inline dReal TransformDistance2(const Transform& t1, const Transform& t2, dReal frotweight=1, dReal ftransweight=1)
{
    //dReal facos = RaveAcos(min(dReal(1),RaveFabs(dot4(t1.rot,t2.rot))));
    dReal facos = min((t1.rot-t2.rot).lengthsqr4(),(t1.rot+t2.rot).lengthsqr4());
    return (t1.trans-t2.trans).lengthsqr3() + frotweight*facos; //*facos;
}

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(SimpleNode)
BOOST_TYPEOF_REGISTER_TYPE(SpatialTree)
#endif

#endif
