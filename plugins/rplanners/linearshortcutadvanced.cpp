// -*- coding: utf-8 -*-
// Copyright (C) 2006-2018 Rosen Diankov <rosen.diankov@gmail.com>
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
// \author Puttichai Lertkultanon and Rosen Diankov
#include "openraveplugindefs.h"

// #define SHORTCUT_ONEDOF_DEBUG
// #define PROGRESS_DEBUG
// #define LINEAR_SMOOTHER_DEBUG

class ShortcutLinearPlanner : public PlannerBase
{
public:
    ShortcutLinearPlanner(EnvironmentBasePtr penv, std::istream& sinput) : PlannerBase(penv)
    {
        __description = ":Interface Author: Rosen Diankov\n\npath optimizer using linear shortcuts.";
        _linearretimer = RaveCreatePlanner(GetEnv(), "LinearTrajectoryRetimer");
    }
    virtual ~ShortcutLinearPlanner() {
    }

    virtual bool InitPlan(RobotBasePtr pbase, PlannerParametersConstPtr params)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new TrajectoryTimingParameters());
        _parameters->copy(params);
        _probot = pbase;
        return _InitPlan();
    }

    virtual bool InitPlan(RobotBasePtr pbase, std::istream& isParameters)
    {
        EnvironmentMutex::scoped_lock lock(GetEnv()->GetMutex());
        _parameters.reset(new TrajectoryTimingParameters());
        isParameters >> *_parameters;
        _probot = pbase;
        return _InitPlan();
    }

    bool _InitPlan()
    {
        if( _parameters->_nMaxIterations <= 0 ) {
            _parameters->_nMaxIterations = 100;
        }
        if( _parameters->_fStepLength <= 0 ) {
            _parameters->_fStepLength = 0.04;
        }
        _linearretimer->InitPlan(RobotBasePtr(), _parameters);
        _puniformsampler = RaveCreateSpaceSampler(GetEnv(),"mt19937");
        if( !!_puniformsampler ) {
            _puniformsampler->SetSeed(_parameters->_nRandomGeneratorSeed);
        }

        _logginguniformsampler = RaveCreateSpaceSampler(GetEnv(),"mt19937");
        if( !!_logginguniformsampler ) {
            _logginguniformsampler->SetSeed(utils::GetMicroTime());
        }
        _fileindex = _logginguniformsampler->SampleSequenceOneUInt32()%1000;
        return !!_puniformsampler;
    }

    virtual PlannerParametersConstPtr GetParameters() const {
        return _parameters;
    }

    virtual PlannerStatus PlanPath(TrajectoryBasePtr ptraj, int planningoptions) override
    {
        BOOST_ASSERT(!!_parameters && !!ptraj );
        if( ptraj->GetNumWaypoints() < 2 ) {
            return PlannerStatus(PS_Failed);
        }

        RobotBase::RobotStateSaverPtr statesaver;
        if( !!_probot ) {
            statesaver.reset(new RobotBase::RobotStateSaver(_probot));
        }

        uint32_t basetime = utils::GetMilliTime();
        PlannerParametersConstPtr parameters = GetParameters();

        // if( IS_DEBUGLEVEL(Level_Verbose) ) {
        //     // store the trajectory
        //     uint32_t randnum;
        //     if( !!_logginguniformsampler ) {
        //         randnum = _logginguniformsampler->SampleSequenceOneUInt32();
        //     }
        //     else {
        //         randnum = RaveRandomInt();
        //     }
        //     string filename = str(boost::format("%s/linearsmoother%d.parameters.xml")%RaveGetHomeDirectory()%(randnum%1000));
        //     ofstream f(filename.c_str());
        //     f << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
        //     f << *_parameters;
        //     RAVELOG_VERBOSE_FORMAT("saved linear parameters to %s", filename);
        //     _DumpTrajectory(ptraj, Level_Verbose, 0);
        // }

        // subsample trajectory and add to list
        list< std::pair< vector<dReal>, dReal> > listpath;

#ifdef LINEAR_SMOOTHER_DEBUG
        TrajectoryBasePtr ptraj0 = RaveCreateTrajectory(GetEnv(), "");
        TrajectoryBasePtr ptraj1 = RaveCreateTrajectory(GetEnv(), "");
        TrajectoryBasePtr ptraj2 = RaveCreateTrajectory(GetEnv(), "");
        ptraj0->Init(parameters->_configurationspecification);
        ptraj1->Init(parameters->_configurationspecification);
        ptraj2->Init(parameters->_configurationspecification);
        ptraj0->Clone(ptraj, 0);
        _DumpTrajectory(ptraj, Level_Verbose, 0);
#endif

        _SubsampleTrajectory(ptraj,listpath);

#ifdef LINEAR_SMOOTHER_DEBUG
        ptraj->Init(parameters->_configurationspecification);
        FOREACH(it, listpath) {
            ptraj->Insert(ptraj->GetNumWaypoints(),it->first);
        }
        ptraj1->Clone(ptraj, 0);
        _DumpTrajectory(ptraj, Level_Verbose, 1);
#endif

        int numshortcuts = _OptimizePath(listpath);
        if( numshortcuts < 0 ) {
            return OPENRAVE_PLANNER_STATUS(str(boost::format("env=%d, Planning was interrupted")%GetEnv()->GetId()), PS_Interrupted);
        }

#ifdef SHORTCUT_ONEDOF_DEBUG
        TrajectoryBasePtr ptrajbefore = RaveCreateTrajectory(GetEnv(), "");
        TrajectoryBasePtr ptrajafter = RaveCreateTrajectory(GetEnv(), "");
        ptrajbefore->Init(parameters->_configurationspecification);
        ptrajafter->Init(parameters->_configurationspecification);
        FOREACH(it, listpath) {
            ptrajbefore->Insert(ptrajbefore->GetNumWaypoints(), it->first);
        }
        _DumpTrajectory(ptrajbefore, Level_Debug, 1);
#endif

        int numshortcutsonedof = _OptimizePathOneDOF(listpath);
        if( numshortcutsonedof < 0 ) {
            return OPENRAVE_PLANNER_STATUS(str(boost::format("env=%d, Planning was interrupted")%GetEnv()->GetId()), PS_Interrupted);
        }

#ifdef SHORTCUT_ONEDOF_DEBUG
        FOREACH(it, listpath) {
            ptrajafter->Insert(ptrajafter->GetNumWaypoints(), it->first);
        }
        _DumpTrajectory(ptrajafter, Level_Debug, 2);
#endif

        ptraj->Init(parameters->_configurationspecification);
        FOREACH(it, listpath) {
            ptraj->Insert(ptraj->GetNumWaypoints(),it->first);
        }

#ifdef LINEAR_SMOOTHER_DEBUG
        ptraj2->Clone(ptraj, 0);
        _DumpTrajectory(ptraj, Level_Verbose, 2);
#endif

        RAVELOG_DEBUG_FORMAT("env=%d, path optimizing - computation time=%fs\n", GetEnv()->GetId()%(0.001f*(float)(utils::GetMilliTime()-basetime)));
        if( parameters->_sPostProcessingPlanner.size() == 0 ) {
            // no other planner so at least retime
            PlannerStatus status = _linearretimer->PlanPath(ptraj, planningoptions);
            if( status.GetStatusCode() != PS_HasSolution ) {
                return status;
            }
            return OPENRAVE_PLANNER_STATUS(PS_HasSolution);
        }
        return _ProcessPostPlanners(RobotBasePtr(),ptraj);
    }

protected:
    /// \brief Iteratively pick two nodes in the path then iterpolate a linear path between them. Note that the
    /// interpolated path might not be stirctly linear due to _neighstatefn.
    int _OptimizePath(list< std::pair< vector<dReal>, dReal> >& listpath)
    {
        PlannerParametersConstPtr parameters = GetParameters();
        list< std::pair< vector<dReal>, dReal> >::iterator itstartnode, itendnode;
        if( !_filterreturn ) {
            _filterreturn.reset(new ConstraintFilterReturn());
        }

        int dof = parameters->GetDOF();
        int nrejected = 0;
        int iiter = parameters->_nMaxIterations;
        int itercount = 0;
        int numiters = (int)parameters->_nMaxIterations;
        std::vector<dReal> vnewconfig0(dof), vnewconfig1(dof);

        int numshortcuts = 0; // keep track of the number of successful shortcuts
#ifdef PROGRESS_DEBUG
        std::stringstream ss;
        ss << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
#endif
        // while(iiter > 0  && nrejected < (int)listpath.size()+4 && listpath.size() > 2 ) {
        while(iiter > 0 && listpath.size() > 2 ) {
            --iiter;
            ++itercount;

            _progress._iteration = itercount;
            if( _CallCallbacks(_progress) == PA_Interrupt ) {
                return -1;
            }

            // pick a random node on the listpath, and a random jump ahead
            uint32_t endIndex = 2+(_puniformsampler->SampleSequenceOneUInt32()%((uint32_t)listpath.size()-2));
            uint32_t startIndex = _puniformsampler->SampleSequenceOneUInt32()%(endIndex-1);
#ifdef PROGRESS_DEBUG
            RAVELOG_DEBUG_FORMAT("env=%d, iter=%d/%d, start shortcutting with i0=%d; i1=%d", GetEnv()->GetId()%itercount%numiters%startIndex%endIndex);
#endif

            itstartnode = listpath.begin();
            advance(itstartnode, startIndex);
            itendnode = itstartnode;
            dReal totaldistance = 0;
            for(uint32_t j = 0; j < endIndex-startIndex; ++j) {
                ++itendnode;
                totaldistance += itendnode->second;
            }
            nrejected++;

            dReal expectedtotaldistance = parameters->_distmetricfn(itstartnode->first, itendnode->first);
            if( expectedtotaldistance > totaldistance - 0.1*parameters->_fStepLength ) {
                // The shortest possible distance between the start and the end of the shortcut (according to
                // _distmetricfn) is not really short so reject it.
#ifdef PROGRESS_DEBUG
                RAVELOG_DEBUG_FORMAT("env=%d, iter=%d/%d, rejecting since it may not make significant improvement. originalSegmentDistance=%.15e, expectedNewDistance=%.15e, diff=%.15e, fStepLength=%.15e", GetEnv()->GetId()%itercount%numiters%totaldistance%expectedtotaldistance%(totaldistance - expectedtotaldistance)%parameters->_fStepLength);
#endif
                continue;
            }

            // check if the nodes can be connected by a straight line
            _filterreturn->Clear();
            int ret = parameters->CheckPathAllConstraints(itstartnode->first, itendnode->first, std::vector<dReal>(), std::vector<dReal>(), 0, IT_Open, 0xffff|CFO_FillCheckedConfiguration, _filterreturn);
            if ( ret != 0 ) {
#ifdef PROGRESS_DEBUG
                ss.str(""); ss.clear();
                ss << "s=" << _filterreturn->_fTimeWhenInvalid << "; vInvalidConfig=[";
                FOREACH(itval, _filterreturn->_invalidvalues) {
                    ss << *itval << ", ";
                }
                ss << "]";
                RAVELOG_DEBUG_FORMAT("env=%d, iter=%d/%d, CheckPathAllConstraints failed, retcode=0x%x. %s", GetEnv()->GetId()%itercount%numiters%ret%ss.str());
#endif
                // if( nrejected++ > (int)listpath.size()+8 ) {
                if( false ) {
#ifdef PROGRESS_DEBUG
                    RAVELOG_DEBUG_FORMAT("env=%d, iter=%d/%d, breaking due to too many consecutive rejection. nrejected=%d, listpath.size()=%d", GetEnv()->GetId()%itercount%numiters%nrejected%listpath.size());
#endif
                    break;
                }
                continue;
            }
            if(_filterreturn->_configurations.size() == 0 ) {
#ifdef PROGRESS_DEBUG
                RAVELOG_DEBUG_FORMAT("env=%d, iter=%d/%d, CheckPathAllConstraints succeeded but did not fill in _filterreturn->_configurations so rejecting.", GetEnv()->GetId()%itercount%numiters);
#endif
                continue;
            }
            OPENRAVE_ASSERT_OP(_filterreturn->_configurations.size()%dof, ==, 0);
            // check how long the new path is
            _vtempdists.resize(_filterreturn->_configurations.size()/dof+1);
            std::vector<dReal>::iterator itdist = _vtempdists.begin();
            std::vector<dReal>::iterator itnewconfig = _filterreturn->_configurations.begin();
            std::copy(itnewconfig, itnewconfig+dof, vnewconfig0.begin());
            dReal newtotaldistance = parameters->_distmetricfn(itstartnode->first, vnewconfig0);
            *itdist++ = newtotaldistance;
            itnewconfig += dof;
            while( itnewconfig != _filterreturn->_configurations.end() ) {
                std::copy(itnewconfig, itnewconfig+dof, vnewconfig1.begin());
                *itdist = parameters->_distmetricfn(vnewconfig0, vnewconfig1);
                newtotaldistance += *itdist;
                ++itdist;
                vnewconfig0.swap(vnewconfig1);
                itnewconfig += dof;
            }
            *itdist = parameters->_distmetricfn(vnewconfig0, itendnode->first);
            newtotaldistance += *itdist;
            ++itdist;
            BOOST_ASSERT(itdist==_vtempdists.end());

            if( newtotaldistance > totaldistance-0.1*parameters->_fStepLength ) {
                // new path is not that good, so reject
                nrejected++;
#ifdef PROGRESS_DEBUG
                RAVELOG_DEBUG_FORMAT("env=%d, iter=%d/%d, rejecting since it does not make significant improvement. originalSegmentDistance=%.15e, newSegmentDistance=%.15e, diff=%.15e, fStepLength=%.15e", GetEnv()->GetId()%itercount%numiters%totaldistance%newtotaldistance%(totaldistance - newtotaldistance)%parameters->_fStepLength);
#endif
                continue;
            }

            // finally add
            itdist = _vtempdists.begin();
            ++itstartnode;
            itnewconfig = _filterreturn->_configurations.begin();
            while(itnewconfig != _filterreturn->_configurations.end()) {
                std::copy(itnewconfig, itnewconfig+dof, vnewconfig1.begin());
                // Add (vnewconfig1, dist) before *itstartnode
                listpath.insert(itstartnode, make_pair(vnewconfig1, *itdist++)); // now itstartnode still points to the same element
                itnewconfig += dof;
            }
            itendnode->second = *itdist++;
            BOOST_ASSERT(itdist==_vtempdists.end());

            // splice out in-between nodes in path
            listpath.erase(itstartnode, itendnode);
            nrejected = 0;

            ++numshortcuts;
#ifdef PROGRESS_DEBUG
            dReal newdistance = 0;
            FOREACH(ittempnode, listpath) {
                newdistance += ittempnode->second;
            }
            RAVELOG_DEBUG_FORMAT("env=%d, iter=%d/%d successful, listpath.size()=%d, totaldistance=%.15e", GetEnv()->GetId()%itercount%numiters%listpath.size()%newdistance);
#endif

            if( listpath.size() <= 2 ) {
#ifdef PROGRESS_DEBUG
                RAVELOG_DEBUG_FORMAT("env=%d, iter=%d/%d, breaking since listpath.size()=%d", GetEnv()->GetId()%itercount%numiters%listpath.size());
#endif
                break;
            }
        }
        RAVELOG_DEBUG_FORMAT("env=%d, linear shortcut finished at iter=%d, successful=%d", GetEnv()->GetId()%itercount%numshortcuts);
        return numshortcuts;
    }

    // Experimental function: shortcut only one DOF at a time. Although some non-default
    // distmetric/neighstatefn/diffstatefn might be used here, we still compute the distance of the
    // shortcut dof simply by taking the difference between two configs.
    int _OptimizePathOneDOF(list< std::pair< vector<dReal>, dReal> >& listpath)
    {
        int numshortcuts = 0;
        PlannerParametersConstPtr parameters = GetParameters();
        int ndof = parameters->GetDOF();
        int itercount = 0;
        int nrejected = 0;
        int numiters = parameters->_nMaxIterations;
        std::list< std::pair< std::vector<dReal>, dReal > >::iterator itstartnode, itendnode, itnode;

        std::list< std::pair< std::vector<dReal>, dReal > > listshortcutpath; // for keeping shortcut path
        std::vector<dReal> vcurconfig(ndof), vnextconfig(ndof), vnewconfig(ndof), vdeltaconfig(ndof);
        if( !_filterreturn ) {
            _filterreturn.reset(new ConstraintFilterReturn());
        }

#ifdef PROGRESS_DEBUG
        std::stringstream ss;
        ss << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
#endif

        for( int iiter = 0; iiter < numiters; ++iiter ) {
            ++itercount;
            if( listpath.size() <= 2 ) {
                return numshortcuts;
            }
            // Sample a DOF to shortcut. Give the last DOF twice as much chance.
            uint32_t idof = _puniformsampler->SampleSequenceOneUInt32()%(ndof + 1);
            if( idof > (uint32_t)(ndof - 1) ) {
                idof = ndof - 1;
            }

            // Pick a pair of nodes in listpath.
            uint32_t endIndex = 2 + (_puniformsampler->SampleSequenceOneUInt32()%((uint32_t)listpath.size() - 2));
            uint32_t startIndex = _puniformsampler->SampleSequenceOneUInt32()%(endIndex - 1);
            if( endIndex == startIndex + 1 ) {
                continue;
            }
#ifdef PROGRESS_DEBUG
            RAVELOG_DEBUG_FORMAT("env=%d, iter=%d/%d, start shortcutting idof=%d with i0=%d; i1=%d", GetEnv()->GetId()%iiter%numiters%idof%startIndex%endIndex);
#endif

            itstartnode = listpath.begin();
            std::advance(itstartnode, startIndex);
            itendnode = itstartnode;
            dReal fTotalDOFDistance = 0; // distance traveled by the DOF idof
            dReal fTotalDistance = 0; // distance traveled by all DOFs.
            dReal fCurDOFValue = itstartnode->first.at(idof);
            for( uint32_t j = 0; j < endIndex - startIndex; ++j ) {
                ++itendnode;
                fTotalDOFDistance += RaveFabs(itendnode->first.at(idof) - fCurDOFValue);
                fTotalDistance += itendnode->second;
                fCurDOFValue = itendnode->first.at(idof);
            }
            nrejected++;

            dReal fExpectedDOFDistance = itendnode->first.at(idof) - itstartnode->first.at(idof);
            // RAVELOG_DEBUG_FORMAT("env=%d, prevdofdist=%.15e; newdofdist=%.15e; diff=%.15e", GetEnv()->GetId()%totalDOFDistance%RaveFabs(expectedDOFDistance)%(totalDOFDistance - RaveFabs(expectedDOFDistance)));
            if( RaveFabs(fExpectedDOFDistance) > fTotalDOFDistance - 0.1*parameters->_vConfigResolution.at(idof) ) {
                // Even after a successful shortcut, the resulting path wouldn't be that much shorter. So skipping.
#ifdef PROGRESS_DEBUG
                RAVELOG_DEBUG_FORMAT("env=%d, iter=%d/%d, rejecting since it may not make significant improvement. originalDOFDistance=%.15e, expectedDOFDistance=%.15e, diff=%.15e, dofresolution=%.15e", GetEnv()->GetId()%iiter%numiters%fTotalDOFDistance%fExpectedDOFDistance%(fTotalDOFDistance - fExpectedDOFDistance)%parameters->_vConfigResolution.at(idof));
#endif
                continue;
            }

            _progress._iteration = iiter;
            if( _CallCallbacks(_progress) == PA_Interrupt ) {
                return -1;
            }

            // Main shortcut procedure
            listshortcutpath.clear();
            bool bSuccess = true;
            dReal fDelta = fExpectedDOFDistance / fTotalDistance;
            dReal fCurDist = 0;
            dReal fStartDOFValue = itstartnode->first.at(idof);
            itnode = itstartnode;
            vcurconfig = itnode->first;

            dReal fNewDOFDistance = 0;
            dReal fNewTotalDistance = 0;
            // Iterate through all expected intermediate configurations and check collision (and other constraints) along the segment preceeding it.
            do {
                std::advance(itnode, 1);
                fCurDist += itnode->second;
                vnextconfig = itnode->first;
                // Modify the value for the DOF idof according to linear interpolation:
                // if at the end, sample the last point directly (which should be inside the constraints)
                if( itnode != itendnode ) {
                    vnextconfig[idof] = fStartDOFValue + fCurDist*fDelta;
                }

                // The interpolated nodes might not be satisfying the tool constraints, so have to project in order to feed CheckPathAllConstraints endpoints that satisfy constraints.
                // filterreturn->_configurations only return configurations that satisfy the constraints
                if( !!parameters->_neighstatefn ) {
                    if( listshortcutpath.size() > 0 ) {
                        for(int ideltadof = 0; ideltadof < (int)vdeltaconfig.size(); ++ideltadof) {
                            dReal fprev = listshortcutpath.back().first[ideltadof];
                            dReal fnew = vnextconfig[ideltadof];
                            vdeltaconfig[ideltadof] = fnew - fprev;
                            vnextconfig[ideltadof] = fprev;
                        }
                    }
                    else {
                        for(int ideltadof = 0; ideltadof < (int)vdeltaconfig.size(); ++ideltadof) {
                            vdeltaconfig[ideltadof] = vnextconfig[ideltadof] - vcurconfig[ideltadof];
                            vnextconfig[ideltadof] = vcurconfig[ideltadof];
                        }
                    }

                    // Need to set state to vnextconfig since _neighstatefn expects the starting values to be set
                    if( parameters->SetStateValues(vnextconfig, 0) != 0 ) {
                        RAVELOG_DEBUG_FORMAT("env=%d, iter=%d/%d, failed to set state", GetEnv()->GetId()%itercount%numiters);
                        bSuccess = false;
                        break;
                    }

                    int neighstatus = parameters->_neighstatefn(vnextconfig, vdeltaconfig, NSO_OnlyHardConstraints);
                    if( neighstatus == NSS_Failed ) {
                        RAVELOG_DEBUG_FORMAT("env=%d, iter=%d/%d, failed neighstatus %d", GetEnv()->GetId()%itercount%numiters%neighstatus);
                        bSuccess = false;
                        break;
                    }

                    if( itnode == itendnode ) {
                        // last point, should be NSS_Reached (not NSS_SuccessfulWithDeviation)
                        if( neighstatus != NSS_Reached ) {
                            RAVELOG_WARN_FORMAT("env=%d, iter=%d/%d, expecting last point to be within constraints, but was not, cannot shortcut, unreached neighstatus=%d.", GetEnv()->GetId()%itercount%numiters%neighstatus);
                            bSuccess = false;
                            break;
                        }
                    }
                }

                _filterreturn->Clear();
                int ret = parameters->CheckPathAllConstraints(vcurconfig, vnextconfig, std::vector<dReal>(), std::vector<dReal>(), 0, IT_OpenStart, 0xffff|CFO_FillCheckedConfiguration, _filterreturn);
                if ( ret != 0 ) {
#ifdef PROGRESS_DEBUG
                    ss.str(""); ss.clear();
                    ss << "s=" << _filterreturn->_fTimeWhenInvalid << "; vInvalidConfig=[";
                    FOREACH(itval, _filterreturn->_invalidvalues) {
                        ss << *itval << ", ";
                    }
                    ss << "]";
                    RAVELOG_DEBUG_FORMAT("env=%d, iter=%d/%d, CheckPathAllConstraints failed, retcode=0x%x. %s", GetEnv()->GetId()%itercount%numiters%ret%ss.str());
#endif
                    bSuccess = false;
                    break;
                }
                if(_filterreturn->_configurations.size() == 0 ) {
#ifdef PROGRESS_DEBUG
                    RAVELOG_DEBUG_FORMAT("env=%d, iter=%d/%d, CheckPathAllConstraints succeeded but did not fill in _filterreturn->_configurations so rejecting.", GetEnv()->GetId()%itercount%numiters);
#endif
                    bSuccess = false;
                    break;
                }
                OPENRAVE_ASSERT_OP(_filterreturn->_configurations.size()%ndof, ==, 0);

                // The segment has passed the check. Add checked configurations to listshortcutpath
                std::vector<dReal>::iterator itnewconfig = _filterreturn->_configurations.begin();
                std::copy(itnewconfig, itnewconfig + ndof, vnewconfig.begin());
                listshortcutpath.push_back( make_pair(vnewconfig, parameters->_distmetricfn(itstartnode->first, vnewconfig)) );
                fNewDOFDistance += RaveFabs(vnewconfig.at(idof) - itstartnode->first.at(idof));
                fNewTotalDistance += listshortcutpath.back().second;
                itnewconfig += ndof;

                while( itnewconfig != _filterreturn->_configurations.end() ) {
                    std::copy(itnewconfig, itnewconfig + ndof, vnewconfig.begin());
                    fNewDOFDistance += RaveFabs(vnewconfig.at(idof) - listshortcutpath.back().first.at(idof));
                    if( fNewDOFDistance > fTotalDOFDistance - 0.1*parameters->_vConfigResolution.at(idof) ) {
                        // The distance the DOF idof travels is too much that this shortcut would not be useful
                        bSuccess = false;
                        break;
                    }
                    listshortcutpath.push_back( make_pair(vnewconfig, parameters->_distmetricfn(listshortcutpath.back().first, vnewconfig)) );
                    fNewTotalDistance += listshortcutpath.back().second;
                    if( fNewTotalDistance > 1.1 * fTotalDistance ) {
                        // The distance along the shortcut path is too much so rejecting it.
                        bSuccess = false;
                        break;
                    }
                    itnewconfig += ndof;
                }
                if( !bSuccess ) {
                    break;
                }

                vcurconfig.swap(vnextconfig);

            } while( itnode != itendnode );

            if( _CallCallbacks(_progress) == PA_Interrupt ) {
                return -1;
            }

            if( !bSuccess || listshortcutpath.size() == 0 ) {
                continue; // continue to the next shortcut iteration
            }

            // should check if the last point in listshortcutpath is close to itendnode
            //dReal fenddist = parameters->_distmetricfn(itendnode->first, listshortcutpath.back().first);

            // Shortcut is successful. Replace the original segments with the content in listshortcutpath.
            ++numshortcuts;
            std::advance(itstartnode, 1);
            std::advance(itendnode, 1);
            itnode = listshortcutpath.begin();
            while( itnode != listshortcutpath.end() ) {
                listpath.insert(itstartnode, *itnode);
                std::advance(itnode, 1);
            }
            listpath.erase(itstartnode, itendnode);
        }
        RAVELOG_DEBUG_FORMAT("env=%d, shortcut one dof; numshortcuts=%d", GetEnv()->GetId()%numshortcuts);
        return numshortcuts;
    }

    /// \brief Subsample the given linear trajectory according to robot config resolution. Subsampling on each linear
    /// segment is done via calling to _neighstatefn. If this _neighstatefn call returns a config that deviates from the
    /// original stright line, we give up subsampling that segment and continue to the next one. (We do not perform any
    /// collision checking here so we cannot allow any config outside of the collision-checked straight line to be in
    /// the subsampled trajectory.)
    /// \param[in] ptraj a linear trajectory
    /// \param[out] listpath list of (config, dist) pairs. Each dist is the distance from config to its predecessor.
    void _SubsampleTrajectory(TrajectoryBasePtr ptraj, list< std::pair< vector<dReal>, dReal> >& listpath) const
    {
        PlannerParametersConstPtr parameters = GetParameters();
        vector<dReal> q0(parameters->GetDOF()), q1(parameters->GetDOF()), dq(parameters->GetDOF()), qcur(parameters->GetDOF()), dq2;
        vector<dReal> vtrajdata;
        ptraj->GetWaypoints(0, ptraj->GetNumWaypoints(), vtrajdata, parameters->_configurationspecification);

        std::copy(vtrajdata.begin(), vtrajdata.begin() + parameters->GetDOF(), q0.begin());
        listpath.emplace_back(q0,  dReal(0));
        qcur = q0;

        for(size_t ipoint = 1; ipoint < ptraj->GetNumWaypoints(); ++ipoint) {
            std::copy(vtrajdata.begin() + (ipoint)*parameters->GetDOF(), vtrajdata.begin() + (ipoint + 1)*parameters->GetDOF(), q1.begin());
            dq = q1;
            parameters->_diffstatefn(dq, q0);
            int i, numSteps = 1;
            vector<dReal>::const_iterator itres = parameters->_vConfigResolution.begin();
            for (i = 0; i < parameters->GetDOF(); i++,itres++) {
                int steps;
                if( *itres != 0 ) {
                    steps = (int)(RaveFabs(dq[i]) / *itres);
                }
                else {
                    steps = (int)(RaveFabs(dq[i]) * 100);
                }
                if (steps > numSteps) {
                    numSteps = steps;
                }
            }
            dReal fisteps = dReal(1.0f)/numSteps;
            FOREACH(it, dq) {
                *it *= fisteps;
            }
            // Subsampling by recursively computing q + dq to get the next config. If computing q + dq fails, we
            // continue by computing the next config by q + 2*dq, q + 3*dq, and so on. If computing q + dq returns some
            // config outside of the original straight line, we give up subsampling.
            int mult = 1;
            for( int f = 1; f < numSteps; f++ ) {
                // Need to set state to vnextconfig since _neighstatefn expects the starting values to be set
                if( parameters->SetStateValues(qcur, 0) != 0 ) {
                    RAVELOG_WARN_FORMAT("env=%d, failed to set state values, stop subsampling segment (%d, %d) at step %d/%d, numwaypoints=%d", GetEnv()->GetId()%(ipoint - 1)%ipoint%f%numSteps%ptraj->GetNumWaypoints());
                    qcur = listpath.back().first; // restore qcur
                    break;
                }

                int neighstatus = NSS_Failed;
                if( mult > 1 ) {
                    dq2 = dq;
                    FOREACHC(it, dq2) {
                        *it *= mult;
                    }
                    neighstatus = parameters->_neighstatefn(qcur, dq2, NSO_OnlyHardConstraints);
                }
                else {
                    neighstatus = parameters->_neighstatefn(qcur, dq, NSO_OnlyHardConstraints);
                }
                if( neighstatus == NSS_SuccessfulWithDeviation ) {
                    RAVELOG_WARN_FORMAT("env=%d, neighstatefn returned different configuration than qcur, stop subsampling segment (%d, %d) at step %d/%d, numwaypoints=%d", GetEnv()->GetId()%(ipoint - 1)%ipoint%f%numSteps%ptraj->GetNumWaypoints());
                    qcur = listpath.back().first; // restore qcur
                    break;
                }
                else if( neighstatus == NSS_Failed ) {
                    RAVELOG_DEBUG_FORMAT("env=%d, neighstatefn failed mult=%d, perhaps non-linear constraints are used?", GetEnv()->GetId()%mult);
                    mult++;
                    continue;
                }
                dReal dist = parameters->_distmetricfn(listpath.back().first, qcur);
                listpath.emplace_back(qcur,  dist);
                mult = 1;
            }
            // always add the last point
            dReal dist = parameters->_distmetricfn(listpath.back().first, q1);
            listpath.emplace_back(q1,  dist);
            qcur = q1;
            q0.swap(q1);
        }

        std::copy(vtrajdata.end() - parameters->GetDOF(), vtrajdata.end(), q1.begin());
        dReal dist = parameters->_distmetricfn(listpath.back().first, q1);
        listpath.emplace_back(q1,  dist);
    }

    std::string _DumpTrajectory(TrajectoryBasePtr traj, DebugLevel level, int option)
    {
        if( IS_DEBUGLEVEL(level) ) {
            std::string filename = _DumpTrajectory(traj, option);
            RavePrintfA(str(boost::format("env=%d, wrote linearshortcutadvanced trajectory to %s")%GetEnv()->GetId()%filename), level);
            return filename;
        }
        return std::string();
    }

    std::string _DumpTrajectory(TrajectoryBasePtr traj, int option)
    {
        string filename;
        if (option == 0) {
            filename = str(boost::format("%s/linearshortcutadvanced%d.initial.xml")%RaveGetHomeDirectory()%(_fileindex));
        }
        else if (option == 1) {
            filename = str(boost::format("%s/linearshortcutadvanced%d.beforeshortcut.xml")%RaveGetHomeDirectory()%(_fileindex));
        }
        else if (option == 2) {
            filename = str(boost::format("%s/linearshortcutadvanced%d.aftershortcut.xml")%RaveGetHomeDirectory()%(_fileindex));
        }
        else {
            filename = str(boost::format("%s/linearshortcutadvanced%d.traj.xml")%RaveGetHomeDirectory()%(_fileindex));
        }
        ofstream f(filename.c_str());
        f << std::setprecision(std::numeric_limits<dReal>::digits10+1);     /// have to do this or otherwise precision gets lost
        traj->serialize(f);
        return filename;
    }

    TrajectoryTimingParametersPtr _parameters;
    SpaceSamplerBasePtr _puniformsampler, _logginguniformsampler;
    uint32_t _fileindex;

    RobotBasePtr _probot;
    PlannerBasePtr _linearretimer;
    ConstraintFilterReturnPtr _filterreturn;
    std::vector<dReal> _vtempdists;
    PlannerProgress _progress;
};

PlannerBasePtr CreateShortcutLinearPlanner(EnvironmentBasePtr penv, std::istream& sinput) {
    return PlannerBasePtr(new ShortcutLinearPlanner(penv, sinput));
}
