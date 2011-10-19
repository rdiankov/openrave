// -*- coding: utf-8 -*-
// Copyright (C) 2006-2009 Rosen Diankov <rosen.diankov@gmail.com>
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
#include "rplanners.h"

class RandomizedAStarPlanner : public PlannerBase
{
    // sorted by increasing getvalue
    template <class T, class S> class BinarySearchTree
    {
public:
        BinarySearchTree() {
            Reset();
        }

        // other global definitions
        void Add(T& pex)
        {
            BOOST_ASSERT( pex != NULL );

            switch(blocks.size()) {
            case 0:
                blocks.push_back(pex);
                return;
            case 1:
                if( blocks.front()->getvalue() < pex->getvalue() ) {
                    blocks.push_back(pex);
                }
                else blocks.insert(blocks.begin(), pex);

                return;

            default: {
                int imin = 0, imax = (int)blocks.size(), imid;

                while(imin < imax) {
                    imid = (imin+imax)>>1;

                    if( blocks[imid]->getvalue() > pex->getvalue() ) imax = imid;
                    else imin = imid+1;
                }

                blocks.insert(blocks.begin()+imin, pex);
                return;
            }
            }
        }

        ///< returns the index into blocks
        int Get(S& s)
        {
            switch(blocks.size()) {
            case 1: return 0;
            case 2: return blocks.front()->getvalue() < s;
            default: {
                int imin = 0, imax = blocks.size()-1, imid;

                while(imin < imax) {
                    imid = (imin+imax)>>1;

                    if( blocks[imid]->getvalue() > s ) imax = imid;
                    else if( blocks[imid]->getvalue() == s ) return imid;
                    else imin = imid+1;
                }

                return imin;
            }
            }
        }

        void Reset()
        {
            blocks.clear();
            blocks.reserve(1<<16);
        }

        vector<T> blocks;
    };

public:
    class RAStarParameters : public PlannerBase::PlannerParameters {
public:
        RAStarParameters() : fRadius(0.1f), fDistThresh(0.03f), fGoalCoeff(1), nMaxChildren(5), nMaxSampleTries(10), _bProcessingRA(false) {
            _vXMLParameters.push_back("radius");
            _vXMLParameters.push_back("distthresh");
            _vXMLParameters.push_back("goalcoeff");
            _vXMLParameters.push_back("maxchildren");
            _vXMLParameters.push_back("maxsampletries");
        }

        dReal fRadius;              ///< _pDistMetric thresh is the radius that children must be within parents
        dReal fDistThresh;          ///< gamma * _pDistMetric->thresh is the sampling radius
        dReal fGoalCoeff;           ///< balancees exploratino vs cost
        int nMaxChildren;           ///< limit on number of children
        int nMaxSampleTries;         ///< max sample tries before giving up on creating a child
protected:
        bool _bProcessingRA;
        virtual bool serialize(std::ostream& O) const
        {
            if( !PlannerParameters::serialize(O) )
                return false;

            O << "<radius>" << fRadius << "</radius>" << endl;
            O << "<distthresh>" << fDistThresh << "</distthresh>" << endl;
            O << "<goalcoeff>" << fGoalCoeff << "</goalcoeff>" << endl;
            O << "<maxchildren>" << nMaxChildren << "</maxchildren>" << endl;
            O << "<maxsampletries>" << nMaxSampleTries << "</maxsampletries>" << endl;

            return !!O;
        }

        ProcessElement startElement(const std::string& name, const AttributesList& atts)
        {
            if( _bProcessingRA )
                return PE_Ignore;
            switch( PlannerBase::PlannerParameters::startElement(name,atts) ) {
            case PE_Pass: break;
            case PE_Support: return PE_Support;
            case PE_Ignore: return PE_Ignore;
            }
            _bProcessingRA = name=="radius"||name=="distthresh"||name=="goalcoeff"||name=="maxchildren"||name=="maxsampletries";
            return _bProcessingRA ? PE_Support : PE_Pass;
        }
        virtual bool endElement(const string& name)
        {
            if( _bProcessingRA ) {
                if( name == "radius")
                    _ss >> fRadius;
                else if( name == "distthresh")
                    _ss >> fDistThresh;
                else if( name == "goalcoeff")
                    _ss >> fGoalCoeff;
                else if( name == "maxchildren")
                    _ss >> nMaxChildren;
                else if( name == "maxsampletries")
                    _ss >> nMaxSampleTries;
                else
                    RAVELOG_WARN(str(boost::format("unknown tag %s\n")%name));
                _bProcessingRA = false;
                return false;
            }
            // give a chance for the default parameters to get processed
            return PlannerParameters::endElement(name);
        }
    };

    struct Node
    {
        Node() {
            parent = NULL; level = 0; numchildren = 0;
        }

        // negative because those are most likely to be popped first
        dReal getvalue() {
            return -ftotal;
        }

        bool compare(const Node* r) {
            BOOST_ASSERT(r != NULL); return ftotal < r->ftotal;
        }

        dReal fcost, ftotal;
        int level;
        Node* parent;
        int numchildren;
        vector<dReal> q;     // the configuration immediately follows the struct
    };

    static bool SortChildGoal(const RandomizedAStarPlanner::Node* p1, const RandomizedAStarPlanner::Node* p2)
    {
        return p1->ftotal < p2->ftotal;    //p1->ftotal-p1->fcost < p2->ftotal-p2->fcost;
    }

    // implement kd-tree or approx-nn in the future, deallocates memory from Node
    class SpatialTree
    {
public:
        SpatialTree() {
            _fBestDist = 0;
        }
        ~SpatialTree() {
            Destroy();
        }

        void Destroy()
        {
            list<Node*>::iterator it;
            FORIT(it, _nodes)
            delete *it;
            FORIT(it, _dead)
            delete *it;
            _nodes.clear();
        }

        inline void AddNode(Node* pnode) {
            _nodes.push_back(pnode);
        }
        Node* GetNN(const vector<dReal>& q)
        {
            if( _nodes.size() == 0 )
                return NULL;

            list<Node*>::iterator itnode = _nodes.begin(), itbest = _nodes.begin();
            dReal fbest = _pDistMetric(q, _nodes.front()->q);
            ++itnode;

            while(itnode != _nodes.end()) {
                dReal f = _pDistMetric(q, (*itnode)->q);
                if( f < fbest ) {
                    itbest = itnode;
                    fbest = f;
                }
                ++itnode;
            }

            _fBestDist = fbest;
            return *itbest;
        }
        inline void RemoveNode(Node* pnode) {
            _nodes.remove(pnode);
        }

        list<Node*> _nodes;
        list<Node*> _dead;
        boost::function<dReal(const std::vector<dReal>&, const std::vector<dReal>&)> _pDistMetric;
        dReal _fBestDist;         ///< valid after a call to GetNN
    };

    enum IntervalType {
        OPEN = 0,
        OPEN_START,
        OPEN_END,
        CLOSED
    };

    RandomizedAStarPlanner(EnvironmentBasePtr penv, std::istream& sinput) : PlannerBase(penv)
    {
        __description = ":Interface Author: Rosen Diankov\n\nRandomized A*. A continuous version of A*. See:\n\
Rosen Diankov, James Kuffner. \"Randomized Statistical Path Planning. Intl. Conf. on Intelligent Robots and Systems, October 2007.\"\n";
        bUseGauss = false;
        nIndex = 0;
    }

    virtual ~RandomizedAStarPlanner() {
    }

    virtual PlannerParametersConstPtr GetParameters() const {
        return _parameters;
    }

    void Destroy()
    {
        _spatialtree.Destroy();
        _sortedtree.Reset();

        //    for(size_t i = 0; i < _vdeadnodes.size(); ++i) {
        //        _vdeadnodes[i]->~Node();
        //        free(_vdeadnodes[i]);
        //    }
        _vdeadnodes.clear();
        _vdeadnodes.reserve(1<<16);
    }

    // Planning Methods
    ///< manipulator state is also set
    virtual bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr pparams)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset();
        Destroy();
        _robot = pbase;
        boost::shared_ptr<RAStarParameters> parameters(new RAStarParameters());
        parameters->copy(pparams);

        RobotBase::RobotStateSaver savestate(_robot);

        if( !parameters->_goalfn )
            parameters->_goalfn = boost::bind(&SimpleGoalMetric::Eval,boost::shared_ptr<SimpleGoalMetric>(new SimpleGoalMetric(_robot)),_1);
        if( !parameters->_costfn )
            parameters->_costfn = boost::bind(&SimpleCostMetric::Eval,boost::shared_ptr<SimpleCostMetric>(new SimpleCostMetric(_robot)),_1);

        _vSampleConfig.resize(GetDOF());
        _jointIncrement.resize(GetDOF());
        _vzero.resize(GetDOF(),0);
        _spatialtree._pDistMetric = parameters->_distmetricfn;

        _jointResolutionInv.resize(0);
        FOREACH(itj, parameters->_vConfigResolution) {
            if( *itj != 0 )
                _jointResolutionInv.push_back(1 / *itj);
            else {
                RAVELOG_WARN("resolution is 0!\n");
                _jointResolutionInv.push_back(100);
            }
        }

        _parameters=parameters;
        nIndex = 0;
        return true;
    }

    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj)
    {
        if( !_parameters ) {
            return PS_Failed;
        }
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        Destroy();

        RobotBase::RobotStateSaver saver(_robot);
        Node* pcurrent=NULL, *pbest = NULL;

        if( !_parameters->_checkpathconstraintsfn(_parameters->vinitialconfig,_parameters->vinitialconfig,IT_OpenStart,ConfigurationListPtr()) ) {
            return PS_Failed;
        }

        _parameters->_setstatefn(_parameters->vinitialconfig);
        pcurrent = CreateNode(0, NULL, _parameters->vinitialconfig);

        vector<dReal> tempconfig(GetDOF());
        int nMaxIter = _parameters->_nMaxIterations > 0 ? _parameters->_nMaxIterations : 8000;

        while(1) {
            if( _sortedtree.blocks.size() == 0 ) {
                break;
            }
            else {
                pcurrent = _sortedtree.blocks.back();
            }

            // delete from current lists
            _sortedtree.blocks.pop_back();
            _vdeadnodes.push_back(pcurrent);
            BOOST_ASSERT( pcurrent->numchildren < _parameters->nMaxChildren );

            if( pcurrent->ftotal - pcurrent->fcost < 1e-4f ) {
                pbest = pcurrent;
                break;
            }

            list<Node*> children;
            int i;

            for(i = 0; i < _parameters->nMaxChildren && pcurrent->numchildren < _parameters->nMaxChildren; ++i) {

                // keep on sampling until a valid config
                int sample;
                for(sample = 0; sample < _parameters->nMaxSampleTries; ++sample) {
                    if( !_parameters->_sampleneighfn(_vSampleConfig, pcurrent->q, _parameters->fRadius) ) {
                        sample = 1000;
                        break;
                    }

                    if ( _parameters->_checkpathconstraintsfn(pcurrent->q, _vSampleConfig, IT_OpenStart, ConfigurationListPtr())) {
                        continue;
                    }
                    _parameters->_setstatefn(_vSampleConfig);
                    break;
                }
                if( sample >= _parameters->nMaxSampleTries ) {
                    continue;
                }

                //while (getchar() != '\n') usleep(1000);

                Node* nearestnode = _spatialtree.GetNN(_vSampleConfig);
                if( _spatialtree._fBestDist > _parameters->fDistThresh ) {
                    dReal fdist = _parameters->_distmetricfn(pcurrent->q, _vSampleConfig);
                    CreateNode(nearestnode->fcost + fdist * _parameters->_costfn(_vSampleConfig), nearestnode, _vSampleConfig, true);
                    pcurrent->numchildren++;

                    if( (_spatialtree._nodes.size() % 50) == 0 ) {
                        //DumpNodes();
                        RAVELOG_VERBOSE(str(boost::format("trees at %d(%d) : to goal at %f,%f\n")%_sortedtree.blocks.size()%_spatialtree._nodes.size()%((pcurrent->ftotal-pcurrent->fcost)/_parameters->fGoalCoeff)%pcurrent->fcost));
                    }
                }
            }

            if( (int)_spatialtree._nodes.size() > nMaxIter ) {
                break;
            }
        }

        if( !pbest ) {
            return PS_Failed;
        }

        RAVELOG_DEBUG("Path found, final node: %f, %f\n", pbest->fcost, pbest->ftotal-pbest->fcost);
        if( !_parameters->_checkpathconstraintsfn(pbest->q,pbest->q,IT_OpenStart,ConfigurationListPtr()) ) {
            RAVELOG_WARN("RA* bad initial config\n");
        }

        stringstream ss;
        ss << endl << "Path found, final node: cost: " << pbest->fcost << ", goal: " << (pbest->ftotal-pbest->fcost)/_parameters->fGoalCoeff << endl;
        for(int i = 0; i < GetDOF(); ++i) {
            ss << pbest->q[i] << " ";
        }
        ss << "\n-------\n";
        RAVELOG_DEBUG(ss.str());

        list<Node*> vecnodes;

        while(pcurrent != NULL) {
            vecnodes.push_back(pcurrent);
            pcurrent = pcurrent->parent;
        }

        _SimpleOptimizePath(vecnodes);

        if( _parameters->_configurationspecification != ptraj->GetConfigurationSpecification() ) {
            ptraj->Init(_parameters->_configurationspecification);
        }
        ptraj->Insert(ptraj->GetNumWaypoints(),_parameters->vinitialconfig);

        list<Node*>::reverse_iterator itcur, itprev;
        itcur = vecnodes.rbegin();
        itprev = itcur++;
        while(itcur != vecnodes.rend() ) {
            _InterpolateNodes((*itprev)->q, (*itcur)->q, ptraj);
            itprev = itcur;
            ++itcur;
        }

        _ProcessPostPlanners(_robot,ptraj);
        return PS_HasSolution;
    }

    int GetTotalNodes() {
        return (int)_sortedtree.blocks.size();
    }

    bool bUseGauss;

private:

    Node* CreateNode(dReal fcost, Node* parent, const vector<dReal>& pfConfig, bool add = true)
    {
        Node* p = new Node();
        p->parent = parent;
        if( parent != NULL )
            p->level = parent->level + 1;

        p->q = pfConfig;
        p->fcost = fcost;
        p->ftotal = _parameters->fGoalCoeff*_parameters->_goalfn(pfConfig) + fcost;

        if( add ) {
            _spatialtree.AddNode(p);
            _sortedtree.Add(p);
        }
        return p;
    }

    void _InterpolateNodes(const vector<dReal>& pQ0, const vector<dReal>& pQ1, TrajectoryBasePtr ptraj)
    {
        // compute  the discretization
        int i, numSteps = 1;
        for (i = 0; i < GetDOF(); i++) {
            int steps = (int)(fabs(pQ1[i] - pQ0[i]) * _jointResolutionInv[i]);
            if (steps > numSteps)
                numSteps = steps;
        }

        // compute joint increments
        for (i = 0; i < GetDOF(); i++) {
            _jointIncrement[i] = (pQ1[i] - pQ0[i])/((dReal)numSteps);
        }
        vector<dReal> vtrajpoint(GetDOF());

        // compute the straight-line path
        for (int f = 1; f <= numSteps; f++) {
            for (i = 0; i < GetDOF(); i++) {
                vtrajpoint[i] = pQ0[i] + (_jointIncrement[i] * f);
            }
            ptraj->Insert(ptraj->GetNumWaypoints(),vtrajpoint);
        }
    }

    void _SimpleOptimizePath(list<Node*>& path)
    {
        if( path.size() <= 2 )
            return;

        list<Node*>::iterator startNode, endNode;

        for(int i =10; i > 0; --i) {
            // pick a random node on the path, and a random jump ahead
            int startIndex = RaveRandomInt()%((int)path.size() - 2);
            int endIndex   = startIndex + ((RaveRandomInt()%5) + 2);
            if (endIndex >= (int)path.size()) endIndex = (int)path.size() - 1;

            startNode = path.begin();
            advance(startNode, startIndex);
            endNode = startNode;
            advance(endNode, endIndex-startIndex);

            // check if the nodes can be connected by a straight line
            if( !_parameters->_checkpathconstraintsfn((*startNode)->q, (*endNode)->q, IT_Open,ConfigurationListPtr()) ) {
                continue;
            }

            // splice out in-between nodes in path
            path.erase(++startNode, endNode);

            if( path.size() <= 2 )
                return;
        }
    }

    void DumpNodes()
    {
        char filename[255];
        sprintf(filename, "matlab/nodes%d.m", nIndex++);
        FILE* f = fopen(filename, "w");
        if( f == NULL ) {
            return;
        }

        vector<Node*>::iterator it;

        vector<Node*>* allnodes[2] = { &_vdeadnodes, &_sortedtree.blocks };

        fprintf(f, "allnodes = [");

        for(int n = 0; n < 2; ++n) {

            FORIT(it, *allnodes[n]) {
                for(int i = 0; i < GetDOF(); ++i) {
                    fprintf(f, "%f ", (*it)->q[i]);
                }

                int index = 0;
                if( (*it)->parent != NULL ) {

                    index = 0;
                    for(size_t j = 0; j < 2; ++j) {
                        vector<Node*>::iterator itfound = find(allnodes[j]->begin(), allnodes[j]->end(), (*it)->parent);
                        if( itfound != allnodes[j]->end() ) {
                            index += (int)(itfound-allnodes[j]->begin());
                            break;
                        }
                        index += (int)_vdeadnodes.size();
                    }
                }

                fprintf(f, "%f %d\n", ((*it)->ftotal-(*it)->fcost)/_parameters->fGoalCoeff, index+1);
            }
        }

        fprintf(f,"];\r\n\r\n");
        fprintf(f, "%s", str(boost::format("startindex = %d")%(_vdeadnodes.size()+1)).c_str());

        fclose(f);
    }

    inline int GetDOF() const {
        return _parameters->GetDOF();
    }

    boost::shared_ptr<RAStarParameters> _parameters;
    SpatialTree _spatialtree;
    BinarySearchTree<Node*, dReal> _sortedtree;       // sorted by decreasing value

    RobotBasePtr _robot;

    vector<Node*> _vdeadnodes;     ///< dead nodes
    vector<dReal> _vSampleConfig;
    vector<dReal> _jointIncrement, _jointResolutionInv;
    vector<dReal> _vzero;

    vector<Transform> _vectrans;     ///< cache
    int nIndex;
};

PlannerBasePtr CreateRandomizedAStarPlanner(EnvironmentBasePtr penv, std::istream& sinput) {
    return PlannerBasePtr(new RandomizedAStarPlanner(penv, sinput));
}
