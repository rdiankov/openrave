// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov <rosen.diankov@gmail.com>
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

// Functions to plan with caging grasps. See
// Rosen Diankov, Siddhartha Srinivasa, Dave Ferguson, James Kuffner.
// Manipulation Planning with Caging Grasps. IEEE-RAS Intl. Conf. on Humanoid Robots, December 2008.
#include "commonmanipulation.h"

class TaskCaging : public ModuleBase
{
public:
    struct BODYTRAJ
    {
        BODYTRAJ() : time(0) {
        }
        dReal time;
        KinBodyPtr ptarget;
        boost::shared_ptr<Trajectory> ptraj;
    };

    // discrete algorithm
    class ConstrainedTaskData : public boost::enable_shared_from_this<ConstrainedTaskData> {
public:
        struct FEATURES
        {
            FEATURES() : ftotal(0),bSuccess(false) {
            }
            boost::array<dReal,3> features;
            dReal ftotal;         // final weighted result
            bool bSuccess;
        };

        typedef pair< vector<dReal>, FEATURES > IKSOL;

        struct IkSolutionCompare
        {
            bool operator()( const IKSOL& a, const IKSOL& b ) const {
                return a.second.ftotal < b.second.ftotal;
            }
        };

        struct GraspCompare
        {
            template <class T>
            bool operator()( const pair<T,dReal>& a, const pair<T,dReal>& b ) const {
                // always put the grasps with computed iksolutions first
                if( (a.first->iksolutions.size() > 0) == (b.first->iksolutions.size() > 0) )
                    return a.second > b.second;                                                                                                                                                                                                                                                                                                                                                        // minimum on top of stack
                else
                    return a.first->iksolutions.size() == 0;
            }
        };

        struct GRASP
        {
            GRASP(Transform t) {
                tgrasp = t;
            }

            Transform tgrasp;
            list<IKSOL> iksolutions;         // first is the iksolution, second is its goal heuristic
        };

        ConstrainedTaskData() : fGraspThresh(0.1f), fConfigThresh(0.2f), nMaxIkIterations(8), nMaxSamples(1), fGoalThresh(0.04f) {
            fWeights[0] = fWeights[1] = fWeights[2] = 0;
            bSortSolutions = false;
            bGenerateFeatures = false;
            bCheckFullCollision = false;
        }
        virtual ~ConstrainedTaskData() {
        }

        virtual void SetRobot(RobotBasePtr robot)
        {
            _robot = robot;
            _robot->GetActiveDOFLimits(_lower, _upper);
            _robot->GetActiveDOFResolutions(_resolution);
            _J.resize(3*_robot->GetActiveDOF());
            _JJt.resize(9);

            if( !!ptarget ) {
                vector<dReal> vl, vu;
                ptarget->GetDOFLimits(vl, vu);
                for(size_t i = 0; i < _vtargetjoints.size(); ++i) {
                    _lower.push_back(vl[_vtargetjoints[i]]);
                    _upper.push_back(vu[_vtargetjoints[i]]);
                    _resolution.push_back(0.02f);         //?
                }

                ptarget->GetDOFValues(vtargvalues);
            }

            _vsample.resize(GetDOF());
            _robot->GetDOFWeights(_vRobotWeights);
        }

        virtual void SetState(const vector<dReal>& pstate)
        {
            _robot->SetActiveDOFValues(vector<dReal>(pstate.begin(),pstate.begin()+_robot->GetActiveDOF()));
            vector<dReal>::const_iterator ittarget = pstate.begin()+_robot->GetActiveDOF();
            for(size_t i = 0; i < _vtargetjoints.size(); ++i)
                vtargvalues[_vtargetjoints[i]] = *ittarget++;
            ptarget->SetDOFValues(vtargvalues);
        }

        virtual void GetState(vector<dReal>& pstate)
        {
            _robot->GetActiveDOFValues(pstate);
            pstate.resize(GetDOF());
            vector<dReal>::iterator ittarget = pstate.begin()+_robot->GetActiveDOF();

            for(size_t i = 0; i < _vtargetjoints.size(); ++i)
                *ittarget++ = vtargvalues[_vtargetjoints[i]];
        }

        virtual int GetDOF() const {
            return _robot->GetActiveDOF()+_vtargetjoints.size();
        }

        bool CheckCollisionInterval(const vector<dReal>& pQ0, const vector<dReal>& pQ1, IntervalType interval)
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
            if (bCheckEnd) {
                SetState(pQ1);
                if (_robot->GetEnv()->CheckCollision(KinBodyConstPtr(_robot)) || _robot->CheckSelfCollision() )
                    return true;
            }

            // compute  the discretization
            int i, numSteps = 1;
            for (i = 0; i < GetDOF(); i++) {
                int steps = (int)(fabs(pQ1[i] - pQ0[i]) * 60.0f);
                if (steps > numSteps)
                    numSteps = steps;
            }

            // compute joint increments
            vector<dReal> _jointIncrement(GetDOF());
            for (i = 0; i < GetDOF(); i++)
                _jointIncrement[i] = (pQ1[i] - pQ0[i])/((dReal)numSteps);

            vector<dReal> v(GetDOF());

            // check for collision along the straight-line path
            // NOTE: this does not check the end config, and may or may
            // not check the start based on the value of 'start'
            for (int f = start; f < numSteps; f++) {
                for (i = 0; i < GetDOF(); i++)
                    v[i] = pQ0[i] + (_jointIncrement[i] * f);

                SetState(v);
                if( _robot->GetEnv()->CheckCollision(KinBodyConstPtr(_robot)) || _robot->CheckSelfCollision() )
                    return true;
            }

            return false;
        }

        virtual void GenerateFeatures(const vector<dReal>& q, boost::array<dReal,3>& pfeatures)
        {
            dReal f = 0;
            dReal fdiff = 0.3f;
            for(int i = 0; i < _robot->GetActiveDOF(); ++i) {
                dReal flower = _lower[i]+fdiff-q[i];
                if( flower > 0 )
                    f += flower;
                else {
                    dReal fupper = q[i]-_upper[i]+fdiff;
                    if( fupper > 0 )
                        f += fupper;
                }
            }
            pfeatures[0] = f;

            f = 0;
            int linkindex = _robot->GetActiveManipulator()->GetEndEffector()->GetIndex();
            Transform tEE = _robot->GetActiveManipulator()->GetTransform();
            _robot->CalculateActiveJacobian(linkindex, tEE.trans, _J);
            //_robot->CalculateActiveRotationalJacobian(linkindex, tEE.trans, &_J[3*robot->GetActiveDOF()]);

            mathextra::multtrans_to2<dReal, dReal, dReal>(&_J[0], &_J[0], 3, _robot->GetActiveDOF(), 3, &_JJt[0], false);

            // find the determinant
            pfeatures[1] = RaveSqrt(RaveFabs(mathextra::matrixdet3(&_JJt[0], 3)));

            f = 0;
            for(size_t i = 0; i < vtargvalues.size(); ++i)
                f += (vtargvalues[i]-vtargettraj.at(0)[i])*(vtargvalues[i]-vtargettraj.at(0)[i]);
            pfeatures[2] = RaveSqrt(f);
        }

        virtual FEATURES EvalWithFeatures(const vector<dReal>& pConfiguration)
        {
            ptarget->GetDOFValues(vtargvalues);
            _robot->SetActiveDOFValues(pConfiguration);

            FEATURES f;
            GenerateFeatures(pConfiguration, f.features);
            f.ftotal = expf(fWeights[0] *f.features[0]) + expf(fWeights[1] * f.features[1]) + expf(fWeights[2] * f.features[2]);
            return f;
        }

        virtual float GetGoalThresh() {
            return fGoalThresh;
        }

        dReal Eval(const vector<dReal>& pConfiguration)
        {
            vector<dReal>::const_iterator ittarget = pConfiguration.begin()+_robot->GetActiveDOF();
            dReal f = 0;
            for(size_t i = 0; i < _vtargetjoints.size(); ++i) {
                dReal diff = *ittarget++-vtargettraj.at(0)[_vtargetjoints[i]];
                f += diff*diff;
            }

            return RaveSqrt(f);        //expf(fWeights[0]*sqrtf(f));
        }

        virtual float DistMetric(const vector<dReal>& c0, const vector<dReal>& c1)
        {
            BOOST_ASSERT( GetDOF() == (int)_vRobotWeights.size() );
            dReal out = 0;
            for(int i=0; i < GetDOF(); i++)
                out += _vRobotWeights[i] * (c0[i]-c1[i])*(c0[i]-c1[i]);
            return RaveSqrt(out);
        }

        virtual dReal GraspDist(const Transform& tprev, const vector<dReal>& preshapeprev, const Transform& tnew)
        {
            dReal frotweight = 0.4f;
            dReal frotdist1 = (tprev.rot - tnew.rot).lengthsqr4();
            dReal frotdist2 = (tprev.rot + tnew.rot).lengthsqr4();
            return (tprev.trans-tnew.trans).lengthsqr3() + frotweight * min(frotdist1,frotdist2);
        }

        virtual bool AcceptConfig(const vector<dReal>& qprev, const vector<dReal>& qnew)
        {
            dReal d = 0;
            for(int i = 0; i < (int)qnew.size(); ++i)
                d += (qnew[i]-qprev[i])*(qnew[i]-qprev[i]);
            return d < fConfigThresh*fConfigThresh;
        }

        // robot config + target body config
        virtual bool Sample(vector<dReal>& pNewSample)
        {
            for(int iter = 0; iter < 100; ++iter) {
                for(int i = _robot->GetActiveDOF(); i < GetDOF(); ++i) {
                    pNewSample[i] = _lower[i] + (_upper[i]-_lower[i])*RaveRandomFloat();
                    vtargvalues[_vtargetjoints[i]] = pNewSample[i];
                }

                ptarget->SetDOFValues(vtargvalues);

                // sample the grasp and execute in random permutation order (for now just select one grasp)
                boost::shared_ptr<vector<Transform> > vgrasps = Eval(pNewSample) < fGoalThresh ? pvGraspContactSet : pvGraspSet;
                if( SampleIkSolution(vgrasps->at(RaveRandomInt()%vgrasps->size()), vector<dReal>(), pNewSample))
                    return true;
            }
            return false;
        }

        static inline int GetTargetValue(int val)
        {
            return (val>1) ? (val-1) : (val-2);
        }

        virtual bool SampleNeigh(vector<dReal>& pNewSample, const vector<dReal>& pCurSample, dReal fRadius)
        {
            map<int, boost::shared_ptr<FINDGRASPDATA> >::iterator itgrasp;
            FORIT(itgrasp, mapgrasps)
            itgrasp->second->status = 0;

            // sample the neighborhood
            int failures = 0;
            while(1) {

                // sample the target object values
                dReal fbest = 10000;
                uint32_t bestid = 0;
                for(int iter = 0; iter < 3; ++iter) {
                    uint32_t id = 0;
                    for(uint32_t i = (uint32_t)_robot->GetActiveDOF(); i < (uint32_t)GetDOF(); ++i) {
                        int val = RaveRandomInt()&3;
                        id |= val<<(2*(i-_robot->GetActiveDOF()));
                        _vsample[i] = pCurSample[i] + fRadius*(float)GetTargetValue(val);
                        _vsample[i] = utils::ClampOnRange(_vsample[i], _lower[i], _upper[i]);
                    }

                    dReal fval = Eval(_vsample);
                    if( fval < fbest ) {
                        fbest = fval;
                        bestid = id;
                    }
                }

                for(uint32_t i = (uint32_t)_robot->GetActiveDOF(); i < (uint32_t)GetDOF(); ++i) {
                    _vsample[i] = pCurSample[i] + fRadius*(float)GetTargetValue((bestid>>(2*(i-_robot->GetActiveDOF())))&3);
                    vtargvalues[_vtargetjoints[i-_robot->GetActiveDOF()]] = _vsample[i];
                }

                ptarget->SetDOFValues(vtargvalues);

                // choose a grasp
                boost::shared_ptr<FINDGRASPDATA> pdata;
                itgrasp = mapgrasps.find(bestid);
                if( itgrasp == mapgrasps.end() ) {
                    // create a new permuter
                    pdata.reset(new FINDGRASPDATA());
                    pdata->status = 0;
                    pdata->pexecutor.reset(new RandomPermutationExecutor(boost::bind(&ConstrainedTaskData::FindGraspPermutation,shared_from_this(),_1,pdata)));
                    mapgrasps[bestid] = pdata;
                }
                else
                    pdata = itgrasp->second;

                if( pdata->status == 2 )
                    continue;

                if( pdata->status == 0 ) {
                    pdata->tcurgrasp = _robot->GetActiveManipulator()->GetTransform();
                    pdata->fThresh2 = fGraspThresh*fGraspThresh;
                    pdata->tlink = ptargetlink->GetTransform();

                    if( 0 &&( Eval(pNewSample) < fGoalThresh) )
                        pdata->pgrasps = pvGraspContactSet;
                    else
                        pdata->pgrasps = pvGraspSet;

                    pdata->status = 1;
                    pdata->pexecutor->PermuteStart(pdata->pgrasps->size());
                }

                BOOST_ASSERT( pdata->status == 1 );

                int index = pdata->pexecutor->PermuteContinue();
                if( index < 0 ) {
                    // couldn't find a grasp, check if we've run out of possible grasps
                    ++failures;
                    if( failures == pow(3.0f,(float)_vtargetjoints.size()) )
                        return false;

                    pdata->status = 2;
                    continue;         // sample again
                }

                _robot->SetActiveDOFValues(vector<dReal>(pCurSample.begin(),pCurSample.begin()+_robot->GetActiveDOF()));
                if( !SampleIkSolution(pdata->tlink * pdata->pgrasps->at(index), pCurSample, _vsample) )
                    continue;

                pNewSample = _vsample;
                break;
            }

            return true;
        }

        bool SampleIkSolution(const Transform& tgrasp, const vector<dReal>& pCurSolution, vector<dReal>& psolution)
        {
            vector< dReal> solution;
            RobotBase::ManipulatorConstPtr pmanip = _robot->GetActiveManipulator();
            if(( nMaxIkIterations == 0) ||( pmanip->GetIkSolver()->GetNumFreeParameters() == 0) ) {
                if( pmanip->FindIKSolution(tgrasp, solution, true) ) {
                    if( pCurSolution.size() == solution.size() ) {
                        // make sure solution is close to current solution
                        if( !AcceptConfig(pCurSolution, solution) ) {
                            return false;
                        }
                    }

                    psolution = solution;
                    return true;
                }

                return false;
            }

            // have a classifier that knows when ik solution is impossible
            if( pmanip->GetIkSolver()->GetNumFreeParameters() > 0 ) {
                _vfreeparams.resize(pmanip->GetIkSolver()->GetNumFreeParameters());
                pmanip->GetIkSolver()->GetFreeParameters(_vcurfreeparams);
            }

            dReal startphi = _vcurfreeparams[0];
            dReal upperphi = 1, lowerphi = 0, deltaphi = 0;
            int iter = 0;
            dReal incphi = 0.01f;

            while(iter < nMaxIkIterations) {

                dReal curphi = startphi;
                if( iter & 1 ) {         // increment
                    curphi += deltaphi;
                    if( curphi > upperphi ) {

                        if( startphi-deltaphi < lowerphi) {
                            break;         // reached limit
                        }
                        ++iter;
                        continue;
                    }
                }
                else {         // decrement
                    curphi -= deltaphi;
                    if( curphi < lowerphi ) {

                        if( startphi+deltaphi > upperphi ) {
                            break;         // reached limit
                        }
                        ++iter;
                        deltaphi += incphi;         // increment
                        continue;
                    }

                    deltaphi += incphi;         // increment
                }

                iter++;
                _vfreeparams[0] = curphi;

                if( pmanip->FindIKSolution(tgrasp, _vfreeparams, solution, true) ) {
                    if( solution.size() == pCurSolution.size() ) {
                        // make sure solution is close to current solution
                        if( !AcceptConfig(pCurSolution, solution) ) {
                            continue;
                        }
                    }

                    psolution = solution;
                    return true;
                }
            }

            //int iter = nMaxIkIterations;
            //    while(iter-- > 0) {
            //        // randomly sample params
            //        for(int j = 0; j < pmanip->GetIkSolver()->GetNumFreeParameters(); ++j)
            //            _vfreeparams[j] = utils::ClampOnRange(_vcurfreeparams[j] + 0.5f*fConfigThresh*(RaveRandomFloat()-0.5f),(dReal)0,(dReal)1);
            //
            //        if( pmanip->FindIKSolution(tgrasp, _vfreeparams, solution, true) ) {
            //
            //            if( pCurSolution.size() == solution.size() ) {
            //                // make sure solution is close to current solution
            //                if( !AcceptConfig(pCurSolution, solution) ) {
            //                    //RAVELOG_WARN("no accept\n");
            //                    continue;
            //                }
            //            }
            //
            //            memcpy(psolution, &solution[0], sizeof(dReal)*solution.size());
            //            return true;
            //        }
            //    }

            return false;
        }

        void FillIkSolutions(GRASP& g, const vector<vector<dReal> >& solutions)
        {
            g.iksolutions.clear();

            if( bSortSolutions ) {
                // fill the list
                FOREACHC(itsol, solutions) {
                    g.iksolutions.push_back(ConstrainedTaskData::IKSOL(*itsol, EvalWithFeatures(*itsol)));
                }
                g.iksolutions.sort(IkSolutionCompare());
                //RAVELOG_WARN("%f %f\n", g.iksolutions.at(0).second, g.iksolutions.back().second);
            }
            else {
                // fill the list
                FOREACHC(itsol, solutions) {
                    g.iksolutions.push_back(ConstrainedTaskData::IKSOL(*itsol, FEATURES()));
                }
            }
        }

        void SetGenerateFeatures(const string& filename)
        {
            bGenerateFeatures = true;
            flog.open(filename.c_str());
        }

        void Log(IKSOL& iksol)
        {
            for(size_t i = 0; i < iksol.second.features.size(); ++i) {
                flog << iksol.second.features[i] << " ";
            }
            flog << iksol.second.bSuccess << " ";
            FOREACH(it, iksol.first) {
                flog << *it << " ";
            }
            flog << endl;
        }

        inline bool IsGenerateFeatures() {
            return bGenerateFeatures;
        }

        // grasping
        boost::shared_ptr< vector< Transform > > pvGraspSet, pvGraspContactSet, pvGraspStartSet;
        vector< list<GRASP> > vlistGraspSet;
        dReal fGraspThresh, fConfigThresh;

        // target object
        RobotBasePtr _robot;
        KinBodyPtr ptarget;
        KinBody::LinkPtr ptargetlink;
        vector<vector<dReal> > vtargettraj;
        vector<int> _vtargetjoints;         ///< active joints of the target
        int nMaxIkIterations;         ///< for sampling free parameters
        int nMaxSamples;         ///< for trying out heuristics
        float fGoalThresh;

        // heuristics
        boost::array<dReal,3> fWeights;         // joint limit, manipulability, goal config
        bool bSortSolutions;
        bool bCheckFullCollision;

        const vector<dReal>& GetLower() const {
            return _lower;
        }
        const vector<dReal>& GetUpper() const {
            return _upper;
        }
        const vector<dReal>& GetResolution() const {
            return _resolution;
        }

protected:
        struct FINDGRASPDATA
        {
            boost::shared_ptr<RandomPermutationExecutor> pexecutor;
            boost::shared_ptr< vector< Transform > > pgrasps;
            Transform tcurgrasp;
            Transform tlink;
            dReal fThresh2;
            int status;         // 0 not init, 1 init, 2 dead
        };

        bool FindGraspPermutation(unsigned int index, boost::shared_ptr<FINDGRASPDATA> pdata)
        {
            return GraspDist(pdata->tcurgrasp, vector<dReal>(), pdata->tlink * pdata->pgrasps->at(index)) < pdata->fThresh2;
        }

        bool bGenerateFeatures;
        ofstream flog;         ///< logs features

        vector<dReal> _lower, _upper, _vsample, _vbestsample, _resolution;
        vector<dReal> _J, _JJt;        // _J is 3xdof
        vector<dReal> vtargvalues, _vfreeparams, _vcurfreeparams;
        vector<dReal> _vfeatures;
        vector<dReal> _vRobotWeights;
        map<int, boost::shared_ptr<FINDGRASPDATA> > mapgrasps;
    };

    // used to prune grasps that are not caging a target object
    class GraspConstraint
    {
public:
        GraspConstraint() {
            Nrandconfigs = 10;
            fRandPerturb.resize(7);
            // 0.01 for translation and 0.07 for quaterions (~8-10 degrees)
            fRandPerturb[0] = fRandPerturb[1] = fRandPerturb[2] = 0.01f;
            fRandPerturb[3] = fRandPerturb[4] = fRandPerturb[5] = fRandPerturb[6] = 0.07f;
        }

        virtual bool Constraint(const vector<dReal>& pSrcConf, const vector<dReal>& pDestConf, IntervalType interval, PlannerBase::ConfigurationListPtr configurations)
        {
            if( !plink ||( vtargetjoints.size() == 0) ) {
                return true;
            }
            _robot->SetActiveDOFValues(pDestConf);
            if( _robot->GetEnv()->CheckCollision(KinBodyConstPtr(_robot)) || _robot->CheckSelfCollision() ) {
                return false;
            }

            bool bCaged = false;
            vector<dReal> vrobotconfig;
            _robot->GetActiveDOFValues(vrobotconfig);

            // find N noncolliding grasps and check if they still cage the object
            for(int iter = 0; iter < Nrandconfigs; ++iter) {
                if( iter > 0 ) {
                    plink->GetParent()->SetLinkTransformations(_vTargetTransforms);

                    int niter=0;
                    for(niter = 0; niter < 100; niter++) {
                        for(int i = 0; i < _robot->GetActiveDOF(); ++i) {
                            vrobotconfig[i] = pDestConf[i] + fRandPerturb[i]*(RaveRandomFloat()-0.5f);
                        }

                        _robot->SetActiveDOFValues(vrobotconfig);
                        if( !_robot->GetEnv()->CheckCollision(KinBodyConstPtr(_robot)) && !_robot->CheckSelfCollision() ) {
                            break;
                        }
                    }

                    if( niter >= 100 ) {
                        continue;
                    }
                }

                FOREACH(itvv, _vvvCachedTransforms) {
                    bCaged = false;
                    FOREACH(itv, *itvv) {
                        plink->GetParent()->SetLinkTransformations(*itv);
                        if( _robot->GetEnv()->CheckCollision(KinBodyConstPtr(_robot), KinBodyConstPtr(plink->GetParent())) ) {
                            bCaged = true;
                            break;
                        }
                    }

                    if( !bCaged ) {
                        break;
                    }
                }

                if( !bCaged ) {
                    break;
                }
            }

            _robot->SetActiveDOFValues(pDestConf);
            plink->GetParent()->SetLinkTransformations(_vTargetTransforms);
            return bCaged;
        }

        dReal Dist6D(const vector<dReal>& v1, const vector<dReal>& v2)
        {
            BOOST_ASSERT(v1.size()==7&&v2.size()==7);
            dReal frot1=0,frot2=0,ftrans=0;
            for(int i = 0; i < 3; ++i) {
                ftrans += (v1[i]-v2[i])*(v1[i]-v2[i]);
            }
            for(int i = 3; i < 7; ++i) {
                frot1 += (v1[i]-v2[i])*(v1[i]-v2[i]);
                frot2 += (v1[i]+v2[i])*(v1[i]+v2[i]);
            }
            return RaveSqrt(min(frot1,frot2)+0.2f*ftrans);
        }

        // \param vTargetSides - -1 for only negative sides, 0 for both, 1 for only positive sides)
        void CacheTransforms(const vector<int>& vTargetSides)
        {
            static vector<dReal> values;
            values = vtargetvalues;

            _vvvCachedTransforms.resize(vtargetjoints.size()*2);
            vector< vector< vector<Transform> > >::iterator itvv = _vvvCachedTransforms.begin();

            plink->GetParent()->SetDOFValues(vtargetvalues);
            plink->GetParent()->GetLinkTransformations(_vTargetTransforms);
            vector<int>::const_iterator itside = vTargetSides.begin();

            FOREACH(itjoint, vtargetjoints) {
                if( *itside <= 0 ) {
                    itvv->resize(0);
                    for(dReal finc = fIncrement; finc < fCagedConfig; finc += fIncrement) {
                        values[*itjoint] = vtargetvalues[*itjoint] - finc;
                        plink->GetParent()->SetDOFValues(values);

                        itvv->push_back(vector<Transform>());
                        plink->GetParent()->GetLinkTransformations(itvv->back());
                    }
                    ++itvv;
                }

                if( *itside >= 0 ) {
                    itvv->resize(0);
                    for(dReal finc = fIncrement; finc < fCagedConfig; finc += fIncrement) {
                        values[*itjoint] = vtargetvalues[*itjoint] + finc;
                        plink->GetParent()->SetDOFValues(values);

                        itvv->push_back(vector<Transform>());
                        plink->GetParent()->GetLinkTransformations(itvv->back());

                    }
                    ++itvv;
                }

                values[*itjoint] = vtargetvalues[*itjoint];         // revert
                ++itside;
            }

            _vvvCachedTransforms.erase(itvv,_vvvCachedTransforms.end());
            plink->GetParent()->SetDOFValues(vtargetvalues);
        }

        dReal fIncrement, fCagedConfig;
        RobotBasePtr _robot;
        KinBody::LinkPtr plink;
        vector<int> vtargetjoints;
        vector<dReal> vtargetvalues;
        int Nrandconfigs;
        vector<dReal> fRandPerturb;

private:
        vector< vector< vector<Transform> > > _vvvCachedTransforms;
        vector<Transform> _vTargetTransforms;
    };

    inline boost::shared_ptr<TaskCaging> shared_problem() {
        return boost::dynamic_pointer_cast<TaskCaging>(shared_from_this());
    }
    inline boost::shared_ptr<TaskCaging const> shared_problem_const() const {
        return boost::dynamic_pointer_cast<TaskCaging const>(shared_from_this());
    }

public:
    TaskCaging(EnvironmentBasePtr penv) : ModuleBase(penv) {
        __description = ":Interface Author: Rosen Diankov\n\n\
.. image:: ../../../images/interface_taskcaging.jpg\n\
  :width: 500\n\n\
Implements various algorithms to open and close \
doors by having the hand cage the handles instead of tightly grip. \
This greatly relaxes the constraints on the robot (see the door manipluation example). The relevant paper is:\n\n\
\
- Rosen Diankov, Siddhartha Srinivasa, Dave Ferguson, James Kuffner. Manipulation Planning with Caging Grasps. IEEE-RAS Intl. Conf. on Humanoid Robots, December 2008.";
        RegisterCommand("graspset",boost::bind(&TaskCaging::GraspSet, this, _1, _2),
                        "Creates a grasp set given a robot end-effector floating in space.\n"
                        "Options: step exploreprob size target targetjoint contactconfigdelta cagedconfig");
        RegisterCommand("taskconstraintplan", boost::bind(&TaskCaging::TaskConstrainedPlanner, this, _1, _2),
                        "Invokes the relaxed task constrained planner");
        RegisterCommand("simpleconstraintplan", boost::bind(&TaskCaging::SimpleConstrainedPlanner, this, _1, _2),
                        "Invokes a simple one grasp planner");
        RegisterCommand("bodytraj",boost::bind(&TaskCaging::BodyTrajectory, this, _1, _2),
                        "Starts a body to follow a trajectory. The trajrectory must contain timestamps\n"
                        "Options: target targettraj");
    }
    virtual ~TaskCaging() {
    }
    virtual void Destroy()
    {
        _robot.reset();
        ModuleBase::Destroy();
        _listBodyTrajs.clear();
    }

    virtual void Reset()
    {
        ModuleBase::Reset();
        _listBodyTrajs.clear();
    }

    virtual int main(const string& args)
    {
        stringstream ss(args);
        ss >> _strRobotName;
        return 0;
    }

    virtual bool SimulationStep(dReal fElapsedTime)
    {
        FOREACH_NOINC(itbody, _listBodyTrajs) {
            Trajectory::TPOINT tp;
            itbody->ptraj->SampleTrajectory(itbody->time, tp);

            BOOST_ASSERT( (int)tp.q.size() == itbody->ptarget->GetDOF());

            if( tp.q.size() > 0 ) {
                itbody->ptarget->SetDOFValues(tp.q);
            }
            itbody->ptarget->SetTransform(tp.trans);

            if( itbody->time > itbody->ptraj->GetTotalDuration() ) {
                itbody = _listBodyTrajs.erase(itbody);
            }
            else {
                itbody->time += fElapsedTime;
                ++itbody;
            }
        }

        return false;
    }

    virtual bool SendCommand(std::ostream& sout, std::istream& sinput)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _robot = GetEnv()->GetRobot(_strRobotName);
        return ModuleBase::SendCommand(sout,sinput);
    }

private:

    bool GraspSet(ostream& sout, istream& sinput)
    {
        dReal fStep = 0.01f;
        dReal fExploreProb = 0.02f;

        dReal fContactConfigDelta = 0.01f;     // how much to move in order to find the contact grasp set
        boost::shared_ptr<GraspConstraint> graspfn(new GraspConstraint());

        KinBodyPtr ptarget;
        graspfn->fCagedConfig = 0.05f;
        graspfn->fIncrement = 0.005f;
        graspfn->_robot = _robot;

        boost::shared_ptr<ExplorationParameters> params(new ExplorationParameters());
        params->_nExpectedDataSize = 1000;

        vector<int> vTargetSides;

        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "step" )
                sinput >> fStep;
            else if( cmd == "exploreprob")
                sinput >> fExploreProb;
            else if( cmd == "size")
                sinput >> params->_nExpectedDataSize;
            else if( cmd == "target") {
                string name;
                int linkindex;
                sinput >> name >> linkindex;
                ptarget = GetEnv()->GetKinBody(name);
                graspfn->plink = ptarget->GetLinks().at(linkindex);
            }
            else if( cmd == "targetjoint") {
                int joint; sinput >> joint;
                graspfn->vtargetjoints.push_back(joint);
            }
            else if( cmd == "contactconfigdelta" ) {
                sinput >> fContactConfigDelta;
            }
            else if( cmd == "cagedconfig" ) {
                sinput >> graspfn->fCagedConfig;
            }
            else if( cmd == "cagesides" ) {
                vTargetSides.resize(graspfn->vtargetjoints.size());
                FOREACH(it, vTargetSides)
                sinput >> *it;
            }
            else {
                RAVELOG_WARN(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERROR(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        if( !ptarget || !graspfn->plink ) {
            RAVELOG_WARN("invalid target\n");
            return false;
        }

        if( vTargetSides.size() == 0 ) {
            vTargetSides.resize(graspfn->vtargetjoints.size(),0);
        }

        string plannername = "ExplorationRRT";
        PlannerBasePtr planner = RaveCreatePlanner(GetEnv(),plannername);
        if( !planner ) {
            RAVELOG_WARN(str(boost::format("failed to find planner %s\n")%plannername));
            return false;
        }

        RobotBase::RobotStateSaver state(_robot);

        Transform tlinkorig, tlinknew;
        vector<dReal> vtargetvalues, vorigvalues;
        ptarget->GetDOFValues(vorigvalues);
        tlinkorig = graspfn->plink->GetTransform();

        vector<dReal> upper, lower;
        ptarget->GetDOFLimits(lower, upper);
        graspfn->vtargetvalues = lower;
        for(size_t i = 0; i < lower.size(); ++i) {
            graspfn->vtargetvalues[i] = 0.5f*(lower[i]+upper[i]);
        }
        ptarget->SetDOFValues(graspfn->vtargetvalues);
        tlinknew = graspfn->plink->GetTransform();

        graspfn->CacheTransforms(vTargetSides);

        // move the robot according to the way the target object moved
        _robot->SetTransform(tlinknew*tlinkorig.inverse()*_robot->GetTransform());

        if( !planningutils::JitterTransform(_robot, 0.004f) ) {
            RAVELOG_WARN("failed to jitter\n");
            return false;
        }

        Transform trobot = _robot->GetTransform();
        _robot->SetAffineTranslationLimits(trobot.trans-Vector(0.5f,0.5f,0.5f),trobot.trans+Vector(0.5f,0.5f,0.5f));
        _robot->SetActiveDOFs(vector<int>(), RobotBase::DOF_X|RobotBase::DOF_Y|RobotBase::DOF_Z|RobotBase::DOF_RotationQuat);
        _robot->GetActiveDOFValues(params->vinitialconfig);
        params->SetRobotActiveJoints(_robot);
        params->_checkpathconstraintsfn = boost::bind(&GraspConstraint::Constraint,graspfn,_1,_2,_3,_4);
        params->_distmetricfn = boost::bind(&GraspConstraint::Dist6D,graspfn,_1,_2);

        params->_fStepLength = fStep;
        params->_fExploreProb = fExploreProb;
        params->_nMaxIterations = params->_nExpectedDataSize*5;

        if( !planner->InitPlan(_robot, params) ) {
            RAVELOG_WARN("failed to initplan\n");
            return false;
        }

        uint32_t basetime = utils::GetMilliTime();
        TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),_robot->GetActiveDOF());

        if( !planner->PlanPath(ptraj) ) {
            RAVELOG_WARN("failed to plan\n");
            return false;
        }


        RAVELOG_INFO(str(boost::format("finished computing grasp set: pts=%d in %dms\n")%ptraj->GetPoints().size()%(utils::GetMilliTime()-basetime)));

        vtargetvalues = graspfn->vtargetvalues;
        RobotBase::ManipulatorPtr pmanip = _robot->GetActiveManipulator();
        if( !pmanip ) {
            RAVELOG_WARN("robot doesn't have manipulator defined, storing robot transformation directly\n");
        }

        FOREACHC(itp, ptraj->GetPoints()) {

            int nContact = 0;

            if( graspfn->vtargetjoints.size() > 0 ) {
                _robot->SetActiveDOFValues(itp->q, false);

                // test 2^dof configurations of the target object
                int nTest = 1<<graspfn->vtargetjoints.size();
                for(int i = 0; i < nTest; ++i) {
                    for(int j = 0; j < (int)graspfn->vtargetjoints.size(); ++j) {
                        dReal fadd = (i & (1<<j)) ? fContactConfigDelta : -fContactConfigDelta;
                        vtargetvalues[graspfn->vtargetjoints[j]] = graspfn->vtargetvalues[graspfn->vtargetjoints[j]] + fadd;
                    }

                    ptarget->SetDOFValues(vtargetvalues, true);

                    if( GetEnv()->CheckCollision(KinBodyConstPtr(_robot), KinBodyConstPtr(ptarget)) ) {
                        // in collision, so part of contact set
                        nContact |= (1<<i);
                    }
                }
            }

            // convert the active dofs to a transform and save
            _robot->SetActiveDOFValues(itp->q, false);
            sout << tlinknew.inverse() * (!pmanip ? _robot->GetTransform() : pmanip->GetTransform()) << " "  << nContact << " ";
        }

        if( !!ptarget ) {
            ptarget->SetDOFValues(vorigvalues);
        }
        return true;
    }

    bool TaskConstrainedPlanner(ostream& sout, istream& sinput)
    {
        boost::shared_ptr<ConstrainedTaskData> taskdata(new ConstrainedTaskData());

        int nLinkIndex=-1;
        string strsavetraj, strbodytraj;

        // randomized algorithm parameters
        string plannername;
        boost::shared_ptr<RAStarParameters> params(new RAStarParameters());
        params->fDistThresh = 0.03f;
        params->fRadius = 0.1f;
        params->fGoalCoeff = 1;
        params->nMaxChildren = 5;
        params->nMaxSampleTries = 2;
        params->_nMaxIterations = 1001;

        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "target" ) {
                string name; sinput >> name;
                taskdata->ptarget = GetEnv()->GetKinBody(name);
                if( !taskdata->ptarget ) {
                    RAVELOG_WARN(str(boost::format("invalid target %s\n")%name));
                }
            }
            else if( cmd == "planner" ) {
                sinput >> plannername;
            }
            else if( cmd == "distthresh" ) {
                sinput >> params->fDistThresh;
            }
            else if( cmd == "sampleradius" ) {
                sinput >> params->fRadius;
            }
            else if( cmd == "goalcoeff" ) {
                sinput >> params->fGoalCoeff;
            }
            else if( cmd == "numchildren") {
                sinput >> params->nMaxChildren;
            }
            else if( cmd == "goalthresh") {
                sinput >> taskdata->fGoalThresh;
            }
            else if( cmd == "targetlink") {
                sinput >> nLinkIndex;
            }
            else if( cmd == "savetraj") {
                strsavetraj = utils::GetFilenameUntilSeparator(sinput,';');
            }
            else if( cmd == "savebodytraj") {
                strbodytraj = utils::GetFilenameUntilSeparator(sinput,';');
            }
            else if( cmd == "graspthresh") {
                sinput >> taskdata->fGraspThresh;
            }
            else if( cmd == "configthresh") {
                sinput >> taskdata->fConfigThresh;
            }
            else if( cmd == "maxsamples") {
                sinput >> taskdata->nMaxSamples;
            }
            else if( cmd == "maxiterations") {
                sinput >> params->_nMaxIterations;
            }
            else if( cmd == "maxikiterations") {
                sinput >> taskdata->nMaxIkIterations;
            }
            else if( cmd == "targetjoints") {
                int num=0;
                sinput >> num;
                taskdata->_vtargetjoints.resize(num);
                FOREACH(it, taskdata->_vtargetjoints)
                sinput >> *it;
            }
            else if(cmd == "graspset") {
                taskdata->pvGraspSet.reset(new vector<Transform>());
                taskdata->pvGraspSet->reserve(200);
                string filename = utils::GetFilenameUntilSeparator(sinput,';');
                ifstream fgrasp(filename.c_str());
                while(!fgrasp.eof()) {
                    Transform t;
                    fgrasp >> t;
                    if( !fgrasp ) {
                        break;
                    }
                    taskdata->pvGraspSet->push_back(t);
                }
                RAVELOG_DEBUG(str(boost::format("grasp set size = %d\n")%taskdata->pvGraspSet->size()));
            }
            else if( cmd == "graspcontactset") {
                taskdata->pvGraspContactSet.reset(new vector<Transform>());
                taskdata->pvGraspContactSet->reserve(200);
                string filename = utils::GetFilenameUntilSeparator(sinput,';');
                ifstream fgrasp(filename.c_str());
                while(!fgrasp.eof()) {
                    Transform t;
                    fgrasp >> t;
                    if( !fgrasp ) {
                        break;
                    }
                    taskdata->pvGraspContactSet->push_back(t);
                }
                RAVELOG_DEBUG(str(boost::format("grasp contact set size = %d\n")%taskdata->pvGraspContactSet->size()));
            }
            else if( cmd == "graspstartset" ) {
                taskdata->pvGraspStartSet.reset(new vector<Transform>());
                taskdata->pvGraspStartSet->reserve(200);
                string filename; sinput >> filename;
                ifstream fgrasp(filename.c_str());
                while(!fgrasp.eof()) {
                    Transform t;
                    fgrasp >> t;
                    if( !fgrasp ) {
                        break;
                    }
                    taskdata->pvGraspStartSet->push_back(t);
                }
                RAVELOG_DEBUG(str(boost::format("grasp start set size = %d\n")%taskdata->pvGraspStartSet->size()));
            }
            else if( cmd == "targettraj" ) {

                if( !taskdata->ptarget ) {
                    RAVELOG_WARN("target cannot be null when specifying trajectories!\n");
                    return false;
                }

                int nPoints; sinput >> nPoints;
                taskdata->vtargettraj.resize(nPoints);

                FOREACH(itp, taskdata->vtargettraj) {
                    itp->resize(taskdata->ptarget->GetDOF());
                    FOREACH(it, *itp)
                    sinput >> *it;
                }
            }
            else if( cmd == "usegoal" ) {
                sinput >> taskdata->fWeights[0] >> taskdata->fWeights[1] >> taskdata->fWeights[2];
                taskdata->bSortSolutions = true;
            }
            else if( cmd == "fullcol" )
                taskdata->bCheckFullCollision = true;
            else if( cmd == "features" ) {
                string filename;
                sinput >> filename;
                taskdata->SetGenerateFeatures(filename);
            }
            else {
                RAVELOG_WARN(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERROR(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        if( !taskdata->ptarget ) {
            RAVELOG_WARN("need to specify target!\n");
            return false;
        }

        if(( nLinkIndex < 0) ||( nLinkIndex >= (int)taskdata->ptarget->GetLinks().size()) ) {
            RAVELOG_WARN("need target link name\n");
            return false;
        }

        if( !taskdata->pvGraspSet ||( taskdata->pvGraspSet->size() == 0) ) {
            RAVELOG_WARN("error, graspset size 0\n");
            return false;
        }
        if( !taskdata->pvGraspContactSet ||( taskdata->pvGraspContactSet->size() == 0) ) {
            RAVELOG_WARN("error, graspset size 0\n");
            return false;
        }

        taskdata->ptargetlink = taskdata->ptarget->GetLinks()[nLinkIndex];

        RobotBase::ManipulatorPtr pmanip = _robot->GetActiveManipulator();
        if( !pmanip->GetIkSolver() ) {
            RAVELOG_WARN("need to select a robot manipulator with ik solver\n");
            return false;
        }

        RobotBase::RobotStateSaver saver(_robot);
        KinBody::KinBodyStateSaver targetsaver(taskdata->ptarget);

        _robot->SetActiveDOFs(pmanip->GetArmIndices());

        boost::shared_ptr<Trajectory> ptraj(RaveCreateTrajectory(GetEnv(),_robot->GetActiveDOF()));

        uint32_t basetime = utils::GetMilliTime(), finaltime;
        taskdata->SetRobot(_robot);

        boost::shared_ptr<Trajectory> ptrajtemp(RaveCreateTrajectory(GetEnv(),taskdata->GetDOF()));
        bool bReverseTrajectory = false;

        if( plannername.size() > 0 ) {

            if( taskdata->vtargettraj.size() < 2 ) {
                RAVELOG_WARN("not enough target trajectory points (need at least 2 for initial and goal\n");
                return false;
            }

            bReverseTrajectory = true;

            // create RA* planner
            params->_goalfn = boost::bind(&ConstrainedTaskData::Eval,taskdata,_1);
            params->_samplefn = boost::bind(&ConstrainedTaskData::Sample,taskdata,_1);
            params->_sampleneighfn = boost::bind(&ConstrainedTaskData::SampleNeigh,taskdata,_1,_2,_3);
            params->_distmetricfn = boost::bind(&ConstrainedTaskData::DistMetric,taskdata,_1,_2);
            params->_setstatefn = boost::bind(&ConstrainedTaskData::SetState,taskdata,_1);
            params->_getstatefn = boost::bind(&ConstrainedTaskData::GetState,taskdata,_1);

            params->_vConfigLowerLimit = taskdata->GetLower();
            params->_vConfigUpperLimit = taskdata->GetUpper();
            params->_vConfigResolution = taskdata->GetResolution();

            vector<dReal> vtargetinit;
            taskdata->ptarget->GetDOFValues(vtargetinit);

            taskdata->ptarget->SetDOFValues(taskdata->vtargettraj.back());
            Transform tlink  = taskdata->ptargetlink->GetTransform();

            bool bSucceed = false;
            FOREACH(it, *taskdata->pvGraspContactSet) {
                if( pmanip->FindIKSolution(tlink * *it, params->vinitialconfig, true) ) {
                    _robot->SetActiveDOFValues(params->vinitialconfig);
                    bSucceed = true;
                    break;
                }
            }

            if( !bSucceed ) {
                RAVELOG_WARN("failed to find goal configuration = %dms\n", utils::GetMilliTime()-basetime);
                return false;
            }

            FOREACH(it, taskdata->_vtargetjoints)
            params->vinitialconfig.push_back(taskdata->vtargettraj.back()[*it]);

            //// pick any solution at the initial config of the target object
            //        bool bSucceed = false;
            //        Transform tlink = taskdata->ptargetlink->GetTransform();
            //        vector<dReal> solution;
            //        FOREACH(itgrasp, taskdata->vGraspSet) {
            //            if( pmanip->FindIKSolution(tlink * *itgrasp, solution, true) ) {
            //                _robot->SetActiveDOFValues(solution);
            //                bSucceed = true;
            //                break;
            //            }
            //        }
            //
            //        if( !bSucceed ) {
            //            RAVELOG_WARN("failed to find initial configuration = %dms\n", utils::GetMilliTime()-basetime);
            //            return false;
            //        }


            // try to find an IK solution for every point on the trajectory (Optional!!)
            bool bHasIK = false;
            vector<dReal> solution;

            for(int ivgrasp = 0; ivgrasp < (int)taskdata->vtargettraj.size()-1; ++ivgrasp) {
                taskdata->ptarget->SetDOFValues(taskdata->vtargettraj[ivgrasp]);
                tlink = taskdata->ptargetlink->GetTransform();
                bHasIK = false;

                FOREACH(it, *taskdata->pvGraspSet) {
                    if( pmanip->FindIKSolution(tlink * *it, solution, true) ) {
                        bHasIK = true;
                        break;
                    }
                }

                if( !bHasIK ) {
                    // no ik solution found for this grasp, so quit
                    RAVELOG_ERROR(str(boost::format("failure, due to ik time=%dms, %d/%d\n")%(utils::GetMilliTime()-basetime)%ivgrasp%taskdata->vtargettraj.size()));
                    break;
                }
            }

            if( !bHasIK )
                return false;

            taskdata->SetState(params->vinitialconfig);

            boost::shared_ptr<PlannerBase> pra(RaveCreatePlanner(GetEnv(),plannername.c_str()));
            if( !pra ) {
                RAVELOG_WARN(str(boost::format("could not find %s planner\n")%plannername));
                return false;
            }

            if( !pra->InitPlan(_robot,params) )
                return false;

            RAVELOG_WARN("planning a caging grasp...\n");
            if( !pra->PlanPath(ptrajtemp) ) {
                RAVELOG_WARN("planner failure, time = %dms\n", utils::GetMilliTime()-basetime);
                return false;
            }
        }
        else {
            CollisionReportPtr report(new CollisionReport());

            if( taskdata->vtargettraj.size() < 2 ) {
                RAVELOG_WARN("not enough trajectory points\n");
                return false;
            }

            // Discretized Search
            vector<vector<dReal> > solutions;

            // initialize the grasp sets
            taskdata->vlistGraspSet.resize(taskdata->vtargettraj.size());

            bool bHasIK = false;

            for(int ivgrasp = 0; ivgrasp < (int)taskdata->vlistGraspSet.size(); ++ivgrasp) {
                int realindex = (ivgrasp+taskdata->vlistGraspSet.size()-1)%taskdata->vlistGraspSet.size();

                taskdata->ptarget->SetDOFValues(taskdata->vtargettraj[realindex]);
                Transform Ttarget = taskdata->ptargetlink->GetTransform();
                bHasIK = false;
                bool bIndependentCollision = pmanip->CheckIndependentCollision(report);
                // check if non-manipulator links are in collision
                if( !bIndependentCollision ) {
                    boost::shared_ptr< vector< Transform > > pvGraspSet = (realindex == 0 && !!taskdata->pvGraspStartSet && taskdata->pvGraspStartSet->size()>0) ? taskdata->pvGraspStartSet : taskdata->pvGraspSet;
                    FOREACH(it, *pvGraspSet) {
                        Transform tgrasp = Ttarget * *it;

                        // make sure that there exists at least 1 IK solution for every
                        // trajectory point of the target configruation space
                        if( !bHasIK ) {

                            if( pmanip->FindIKSolutions(tgrasp, solutions, true) ) {
                                bHasIK = true;
                                taskdata->vlistGraspSet[realindex].push_back(ConstrainedTaskData::GRASP(tgrasp));
                                taskdata->FillIkSolutions(taskdata->vlistGraspSet[realindex].back(), solutions);
                                if( taskdata->bCheckFullCollision ) {
                                    FOREACH(itsol, taskdata->vlistGraspSet[realindex].back().iksolutions) {
                                        FOREACH(itindex, taskdata->_vtargetjoints)
                                        itsol->first.push_back(taskdata->vtargettraj[realindex][*itindex]);
                                    }
                                }

                            }
                        }
                        else {
                            // add without testing ik (lazy computation)
                            taskdata->vlistGraspSet[realindex].push_back(ConstrainedTaskData::GRASP(tgrasp));
                        }
                    }
                }

                if( !bHasIK ) {
                    // no ik solution found for this grasp, so quit
                    RAVELOG_ERROR(str(boost::format("failure, due to ik time=%dms, %d/%d, col=%d\n")%(utils::GetMilliTime()-basetime)%realindex%taskdata->vtargettraj.size()%bIndependentCollision));
                    if( bIndependentCollision )
                        RAVELOG_ERROR(str(boost::format("colliding %s:%s with %s:%s\n")%report->plink1->GetParent()->GetName()%report->plink1->GetName()%report->plink2->GetParent()->GetName()%report->plink2->GetName()));
                    break;
                }
            }

            if( !bHasIK ) {
                return false;
            }

            BOOST_ASSERT(taskdata->vtargettraj.size()>0);
            taskdata->ptarget->SetDOFValues(taskdata->vtargettraj.back());
            Transform Ttarget = taskdata->ptargetlink->GetTransform();

            // start planning backwards
            bool bSuccess = false;
            FOREACHC(itgrasp, *taskdata->pvGraspContactSet) {
                if( pmanip->FindIKSolutions(Ttarget * *itgrasp, solutions, true) ) {
                    FOREACH(itsol, solutions) {
                        FOREACH(itindex, taskdata->_vtargetjoints)
                        itsol->push_back(taskdata->vtargettraj.back()[*itindex]);

                        if( FindAllRelaxedForward(*itsol, taskdata->vtargettraj.size()-2, ptrajtemp.get(), taskdata) ) {
                            ptrajtemp->AddPoint(Trajectory::TPOINT(*itsol, 0));
                            bSuccess = true;
                            break;
                        }
                    }

                    if( bSuccess )
                        break;
                }
            }

            if( !bSuccess ) {
                RAVELOG_WARN("failure, time=%dms\n", utils::GetMilliTime()-basetime);
                return false;
            }
        }

        finaltime = utils::GetMilliTime()-basetime;

        Trajectory::TPOINT tp;
        vector<Trajectory::TPOINT> vtemppoints = ptrajtemp->GetPoints();
        if( bReverseTrajectory ) {
            FOREACHRC(itpoint, vtemppoints) {
                tp.q.resize(0);
                tp.q.insert(tp.q.end(), itpoint->q.begin(), itpoint->q.begin()+_robot->GetActiveDOF());
                ptraj->AddPoint(tp);
            }
        }
        else {
            FOREACHC(itpoint, vtemppoints) {
                tp.q.resize(0);
                tp.q.insert(tp.q.end(), itpoint->q.begin(), itpoint->q.begin()+_robot->GetActiveDOF());
                ptraj->AddPoint(tp);
            }
        }

        ptraj->CalcTrajTiming(_robot, ptraj->GetInterpMethod(), true, true);

        if( strbodytraj.size() > 0 ) {

            boost::shared_ptr<Trajectory> pbodytraj(RaveCreateTrajectory(GetEnv(),taskdata->ptarget->GetDOF()));

            vector<Trajectory::TPOINT>::const_iterator itrobottraj = ptraj->GetPoints().begin();
            taskdata->ptarget->GetDOFValues(tp.q);
            Transform ttarget = taskdata->ptarget->GetTransform();

            if( bReverseTrajectory ) {
                FOREACHRC(itpoint, ptrajtemp->GetPoints()) {
                    for(size_t i = 0; i < taskdata->_vtargetjoints.size(); ++i)
                        tp.q[taskdata->_vtargetjoints[i]] = itpoint->q[_robot->GetActiveDOF()+i];
                    tp.time = itrobottraj++->time;
                    tp.trans = ttarget;
                    pbodytraj->AddPoint(tp);
                }
            }
            else {
                FOREACHC(itpoint, ptrajtemp->GetPoints()) {
                    for(size_t i = 0; i < taskdata->_vtargetjoints.size(); ++i)
                        tp.q[taskdata->_vtargetjoints[i]] = itpoint->q[_robot->GetActiveDOF()+i];
                    tp.time = itrobottraj++->time;
                    tp.trans = ttarget;
                    pbodytraj->AddPoint(tp);
                }
            }

            ofstream f(strbodytraj.c_str());
            pbodytraj->serialize(f);
        }

        RAVELOG_WARN("success, time=%dms\n", finaltime);
        sout << finaltime << " ";

        // write first and last points
        vector<dReal> values;
        _robot->SetActiveDOFValues(ptraj->GetPoints().at(0).q);
        _robot->GetDOFValues(values);
        FOREACH(it, values)
        sout << *it << " ";
        _robot->SetActiveDOFValues(ptraj->GetPoints().back().q);
        _robot->GetDOFValues(values);
        FOREACH(it, values)
        sout << *it << " ";

        if( strsavetraj.size() ) {
            boost::shared_ptr<Trajectory> pfulltraj(RaveCreateTrajectory(GetEnv(),_robot->GetDOF()));
            _robot->GetFullTrajectoryFromActive(pfulltraj, ptraj);
            ofstream f(strsavetraj.c_str());
            pfulltraj->serialize(f);
        }

        return true;
    }

    bool SimpleConstrainedPlanner(ostream& sout, istream& sinput)
    {
        RAVELOG_DEBUG("SimpleConstrainedPlanner\n");

        vector<vector<dReal> > vtargettraj;
        dReal fConfigThresh2 = 0.1f*0.1f;

        list< Transform > listGraspSet;

        boost::shared_ptr<ConstrainedTaskData> taskdata(new ConstrainedTaskData());

        int nLinkIndex=-1;
        string strsavetraj, strbodytraj;

        string cmd;
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "target" ) {
                string name; sinput >> name;
                taskdata->ptarget = GetEnv()->GetKinBody(name);
                if( !taskdata->ptarget )
                    RAVELOG_WARN("invalid target %s\n", name.c_str());
            }
            else if( cmd == "targetlink" )
                sinput >> nLinkIndex;
            else if( cmd == "savetraj")
                sinput >> strsavetraj;
            else if( cmd == "savebodytraj")
                sinput >> strbodytraj;
            else if( cmd == "configthresh") {
                sinput >> fConfigThresh2;
                fConfigThresh2 *= fConfigThresh2;
            }
            else if( cmd == "graspset") {
                listGraspSet.clear();
                string filename = utils::GetFilenameUntilSeparator(sinput,';');
                ifstream fgrasp(filename.c_str());
                while(!fgrasp.eof()) {
                    Transform t;
                    fgrasp >> t;
                    if( !fgrasp ) {
                        RAVELOG_ERROR("grasp set file corrupted\n");
                        break;
                    }
                    listGraspSet.push_back(t);
                }
            }
            else if( cmd == "targettraj" ) {

                if( !taskdata->ptarget ) {
                    RAVELOG_WARN("target cannot be null when specifying trajectories!\n");
                    return false;
                }

                int nPoints; sinput >> nPoints;

                vtargettraj.resize(nPoints);
                FOREACH(itp, vtargettraj) {
                    itp->resize(taskdata->ptarget->GetDOF());
                    FOREACH(it, *itp)
                    sinput >> *it;
                }
            }
            else if( cmd == "fullcol" )
                taskdata->bCheckFullCollision = true;
            else {
                RAVELOG_WARN(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERROR(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        if( !taskdata->ptarget ) {
            RAVELOG_WARN("need to specify target!\n");
            return false;
        }

        if(( nLinkIndex < 0) ||( nLinkIndex >= (int)taskdata->ptarget->GetLinks().size()) ) {
            RAVELOG_WARN("need target link name\n");
            return false;
        }
        taskdata->ptargetlink = taskdata->ptarget->GetLinks()[nLinkIndex];

        RobotBase::ManipulatorPtr pmanip = _robot->GetActiveManipulator();
        if( !pmanip->GetIkSolver() ) {
            RAVELOG_WARN("need to select a robot manipulator with ik solver\n");
            return false;
        }

        if( vtargettraj.size() == 0 )
            return false;

        for(int i = 0; i < (int)vtargettraj.at(0).size(); ++i)
            taskdata->_vtargetjoints.push_back(i);

        RobotBase::RobotStateSaver saver(_robot);
        KinBody::KinBodyStateSaver targetsaver(taskdata->ptarget);

        _robot->SetActiveDOFs(pmanip->GetArmIndices());
        taskdata->SetRobot(_robot);

        uint32_t basetime = utils::GetMilliTime();

        vector<vector<dReal> > solutions;
        list<vector<dReal> > vtrajectory;
        bool bSuccess = false;

        vector<list<vector<dReal> > > vsolutions(vtargettraj.size());

        FOREACHC(itgrasp, listGraspSet) {

            bool bNoIK = false;

            // get the IK solutions
            for(int i = 0; i < (int)vtargettraj.size(); ++i) {
                int realindex = (i+vtargettraj.size()-1)%vtargettraj.size();
                vsolutions[realindex].clear();
                taskdata->ptarget->SetDOFValues(vtargettraj[realindex]);
                Transform Ttarget = taskdata->ptargetlink->GetTransform();

                pmanip->FindIKSolutions(Ttarget * *itgrasp, solutions, true);

                if( solutions.size() == 0 ) {
                    bNoIK = true;
                    break;
                }

                FOREACH(itsol, solutions) {
                    if( taskdata->bCheckFullCollision )
                        itsol->insert(itsol->end(), vtargettraj[realindex].begin(), vtargettraj[realindex].end());
                    vsolutions[realindex].push_back(*itsol);
                }
            }

            if( bNoIK )
                continue;

            FOREACH(itsol, vsolutions.at(0)) {
                if( FindAllSimple(*itsol, 1, vtrajectory, fConfigThresh2, vsolutions, taskdata) ) {
                    vtrajectory.push_back(*itsol);
                    bSuccess = true;
                    break;
                }
            }

            if( bSuccess )
                break;
        }

        uint32_t finaltime = utils::GetMilliTime()-basetime;

        if( !bSuccess ) {
            RAVELOG_WARN("failure, time=%dms\n", finaltime);
            return false;
        }

        RAVELOG_WARN("success, time=%dms\n", finaltime);
        sout << finaltime << " ";

        // write the last point
        vector<dReal> values;

        BOOST_ASSERT(vtrajectory.size()>0);
        _robot->SetActiveDOFValues(vtrajectory.front());
        _robot->GetDOFValues(values);
        FOREACH(it, values)
        sout << *it << " ";
        _robot->SetActiveDOFValues(vtrajectory.back());
        _robot->GetDOFValues(values);
        FOREACH(it, values)
        sout << *it << " ";

        boost::shared_ptr<Trajectory> ptraj(RaveCreateTrajectory(GetEnv(),_robot->GetActiveDOF()));
        Trajectory::TPOINT tp;
        FOREACHR(itsol, vtrajectory) {
            tp.q.resize(0);
            tp.q.insert(tp.q.end(), itsol->begin(), itsol->begin()+_robot->GetActiveDOF());
            ptraj->AddPoint(tp);
        }

        ptraj->CalcTrajTiming(_robot, ptraj->GetInterpMethod(), true, true);

        BOOST_ASSERT( vtargettraj.size() == ptraj->GetPoints().size() );

        if( strbodytraj.size() > 0 ) {

            boost::shared_ptr<Trajectory> pbodytraj(RaveCreateTrajectory(GetEnv(),taskdata->ptarget->GetDOF()));
            vector<Trajectory::TPOINT>::const_iterator itrobottraj = ptraj->GetPoints().begin();

            taskdata->ptarget->GetDOFValues(tp.q);
            Transform ttarget = taskdata->ptarget->GetTransform();

            FOREACH(itv, vtargettraj) {
                for(size_t i = 0; i < taskdata->_vtargetjoints.size(); ++i)
                    tp.q[taskdata->_vtargetjoints[i]] = (*itv)[i];
                tp.time = itrobottraj++->time;
                tp.trans = ttarget;
                pbodytraj->AddPoint(tp);
            }

            ofstream f(strbodytraj.c_str());
            pbodytraj->serialize(f);
        }

        if( strsavetraj.size() ) {
            boost::shared_ptr<Trajectory> pfulltraj(RaveCreateTrajectory(GetEnv(),_robot->GetDOF()));
            _robot->GetFullTrajectoryFromActive(pfulltraj, ptraj);
            ofstream f(strsavetraj.c_str());
            pfulltraj->serialize(f);
        }

        return true;
    }

    bool BodyTrajectory(ostream& sout, istream& sinput)
    {
        BODYTRAJ body;
        string strtraj;
        string cmd;
        char sep = ' ';
        while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput )
                break;
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

            if( cmd == "targettraj") {
                strtraj = utils::GetFilenameUntilSeparator(sinput,sep);
            }
            else if( cmd == "sep") {
                sinput >> sep;
            }
            else if( cmd == "target") {
                string name; sinput >> name;
                body.ptarget = GetEnv()->GetKinBody(name);
            }
            else {
                RAVELOG_WARN(str(boost::format("unrecognized command: %s\n")%cmd));
                break;
            }

            if( !sinput ) {
                RAVELOG_ERROR(str(boost::format("failed processing command %s\n")%cmd));
                return false;
            }
        }

        if( !body.ptarget )
            return false;

        body.ptraj = RaveCreateTrajectory(GetEnv(),body.ptarget->GetDOF());
        ifstream f(strtraj.c_str());
        if( !body.ptraj->Read(f, RobotBasePtr()) ) {
            RAVELOG_ERROR(str(boost::format("failed to read %s\n")%strtraj));
            return false;
        }
        _listBodyTrajs.push_back(body);
        return true;
    }

    // relaxed task constraints
    bool FindAllRelaxedForward(const vector<dReal>& qprev, int j, Trajectory* ptraj, boost::shared_ptr<ConstrainedTaskData> taskdata)
    {
        //RAVELOG_WARN("%d\n", j);
        RobotBase::ManipulatorPtr pmanip = _robot->GetActiveManipulator();
        BOOST_ASSERT( !!pmanip->GetIkSolver() );

        taskdata->ptarget->SetDOFValues(taskdata->vtargettraj[j]);

        vector<dReal> qprevrobot(qprev.begin(),qprev.begin()+_robot->GetActiveDOF());
        _robot->SetActiveDOFValues(qprevrobot);
        Transform tprevgrasp = pmanip->GetTransform();

        vector<dReal> preshape;
        bool bFoundAtLeastOne = false;

        typedef pair<list<ConstrainedTaskData::GRASP>::iterator, dReal> GRASPPAIR;
        priority_queue<GRASPPAIR, deque<GRASPPAIR>, ConstrainedTaskData::GraspCompare> pqAcceptedGrasps;

        FOREACH(itgrasp, taskdata->vlistGraspSet[j]) {

            dReal fdist = taskdata->GraspDist(tprevgrasp, vector<dReal>(), itgrasp->tgrasp);

            if( fdist < taskdata->fGraspThresh*taskdata->fGraspThresh ) {
                // accept the grasp
                pqAcceptedGrasps.push( pair<list<ConstrainedTaskData::GRASP>::iterator,dReal>(itgrasp, fdist));
            }
        }

        while(!pqAcceptedGrasps.empty()) {
            ConstrainedTaskData::GRASP& grasp = *pqAcceptedGrasps.top().first;

            if( grasp.iksolutions.size() == 0 ) {
                vector< vector<dReal> > solutions;
                if( !pmanip->FindIKSolutions(grasp.tgrasp, solutions, true) ) {
                    // could not find ik solutions for this grasp, so remove it permanently
                    taskdata->vlistGraspSet[j].erase(pqAcceptedGrasps.top().first);
                    pqAcceptedGrasps.pop();
                    continue;
                }

                //RAVELOG_WARN("iksol\n", j);
                taskdata->FillIkSolutions(grasp, solutions);
                if( taskdata->bCheckFullCollision ) {
                    FOREACH(itsol, grasp.iksolutions) {
                        FOREACH(itindex, taskdata->_vtargetjoints)
                        itsol->first.push_back(taskdata->vtargettraj[j][*itindex]);
                    }
                }

                _robot->SetActiveDOFValues(qprevrobot);
            }

            FOREACH_NOINC(itsol, grasp.iksolutions) {
                if( taskdata->AcceptConfig(qprev, itsol->first) ) {

                    // check for intermediate collisions
                    if( taskdata->bCheckFullCollision ) {
                        if( taskdata->CheckCollisionInterval(qprev, itsol->first, IT_Open) ) {
                            ++itsol;
                            continue;
                        }
                    }

                    // go to next
                    if( itsol->second.bSuccess ) {
                        // already know it will succeed, no need to recurse
                        bFoundAtLeastOne = true;
                        ++itsol;
                        continue;
                    }

                    if( taskdata->bCheckFullCollision )
                        taskdata->ptarget->SetDOFValues(taskdata->vtargettraj[j]);
                    if(( j == 0) || FindAllRelaxedForward(itsol->first, j-1, ptraj, taskdata)) {

                        bFoundAtLeastOne = true;
                        itsol->second.bSuccess = true;

                        if( !taskdata->IsGenerateFeatures() ) {

                            if( !taskdata->bCheckFullCollision ) {
                                FOREACH(itindex, taskdata->_vtargetjoints)
                                itsol->first.push_back(taskdata->vtargettraj[j][*itindex]);
                            }
                            ptraj->AddPoint(Trajectory::TPOINT(itsol->first, 0));
                            return true;
                        }
                        else {
                            // go on
                            taskdata->Log(*itsol);
                        }
                    }
                    else {

                        if( taskdata->IsGenerateFeatures() )
                            taskdata->Log(*itsol);

                        if( taskdata->vlistGraspSet[j-1].size() == 0 ) {
                            // no more grasps at next level, so fail
                            taskdata->vlistGraspSet[j].clear();
                            return false;
                        }

                        // remove itsol from consideration in the future
                        itsol = grasp.iksolutions.erase(itsol);
                        continue;
                    }
                }
                ++itsol;
            }

            if( taskdata->bCheckFullCollision ) {
                taskdata->ptarget->SetDOFValues(taskdata->vtargettraj[j]);
            }
            if( grasp.iksolutions.size() == 0 ) {
                taskdata->vlistGraspSet[j].erase(pqAcceptedGrasps.top().first);     // remove
            }
            pqAcceptedGrasps.pop();
        }

        return bFoundAtLeastOne;
    }

    // simple task constraints
    bool FindAllSimple(const vector<dReal>& qprev, int j, list<vector<dReal> >& vtrajectory, dReal fConfigThresh2, vector<list<vector<dReal> > >& vsolutions, boost::shared_ptr<ConstrainedTaskData> taskdata)
    {
        FOREACH_NOINC(itsol, vsolutions[j]) {

            dReal d = 0;
            for(size_t i = 0; i < (*itsol).size(); ++i)
                d += ((*itsol)[i]-qprev[i])*((*itsol)[i]-qprev[i]);

            if( d > fConfigThresh2 ) {
                ++itsol;
                continue;
            }

            if( taskdata->bCheckFullCollision ) {
                if( taskdata->CheckCollisionInterval(qprev, *itsol, IT_Open) ) {
                    ++itsol;
                    continue;
                }
            }

            if(( j+1 >= (int)vsolutions.size()) ||
               FindAllSimple(*itsol, j+1, vtrajectory, fConfigThresh2, vsolutions, taskdata) ) {
                vtrajectory.push_back(*itsol);
                return true;
            }

            // no point in continuing
            if( vsolutions[j+1].size() == 0 ) {
                vsolutions[j].clear();
                return false;
            }

            itsol = vsolutions[j].erase(itsol);
        }

        return false;
    }

    list<BODYTRAJ> _listBodyTrajs;
    string _strRobotName;     // name to init robot with
    RobotBasePtr _robot;
};

#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()

BOOST_TYPEOF_REGISTER_TYPE(TaskCaging::BODYTRAJ)
BOOST_TYPEOF_REGISTER_TYPE(TaskCaging::ConstrainedTaskData)
BOOST_TYPEOF_REGISTER_TYPE(TaskCaging::ConstrainedTaskData::GRASP)
BOOST_TYPEOF_REGISTER_TYPE(TaskCaging::ConstrainedTaskData::FEATURES)
BOOST_TYPEOF_REGISTER_TYPE(TaskCaging::ConstrainedTaskData::FINDGRASPDATA)

#endif

ModuleBasePtr CreateTaskCaging(EnvironmentBasePtr penv) {
    return ModuleBasePtr(new TaskCaging(penv));
}
