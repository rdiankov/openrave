// -*- coding: utf-8 -*-
// Copyright (C) 2006-2014 Rosen Diankov <rosen.diankov@gmail.com>
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
#ifndef OPENRAVE_MANIP_CONSTRAINTS_H
#define OPENRAVE_MANIP_CONSTRAINTS_H

#include "openraveplugindefs.h"
#include "ParabolicPathSmooth/DynamicPath.h"

namespace rplanners {

struct ManipConstraintInfo
{
    RobotBase::ManipulatorPtr pmanip;

    // the end-effector link of the manipulator
    KinBody::LinkPtr plink;
    // the points to check for in the end effector coordinate system
    // for now the checked points are the vertices of the bounding box
    // but this can be more general, e.g. the vertices of the convex hull
    std::list<Vector> checkpoints;

    std::vector<int> vuseddofindices; ///< a vector of unique DOF indices targetted for the body
    std::vector<int> vconfigindices; ///< for every index in vuseddofindices, returns the first configuration space index it came from
};

class ManipConstraintChecker
{
public:
    ManipConstraintChecker(EnvironmentBasePtr penv) : _penv(penv), _maxmanipspeed(0), _maxmanipaccel(0) {
    }

    /// \brief given a world AABB oriented, return its 8 vertices
    static void ConvertAABBtoCheckPoints(const AABB& ab, std::list<Vector>& checkpoints)
    {
        dReal signextents[24] = {1,1,1,   1,1,-1,  1,-1,1,   1,-1,-1,  -1,1,1,   -1,1,-1,  -1,-1,1,   -1,-1,-1};
        Vector incr;
        //Transform Tinv = T.inverse();
        checkpoints.resize(0);
        for(int i=0; i<8; i++) {
            incr[0] = ab.extents[0] * signextents[3*i+0];
            incr[1] = ab.extents[1] * signextents[3*i+1];
            incr[2] = ab.extents[2] * signextents[3*i+2];
            checkpoints.push_back(ab.pos + incr);
        }
    }

    /// \brief Compute the AABB that encloses all the links in linklist with respect to a coordinate system Tparent
    ///
    /// \param tparent is most likely the end effector of a manipulator
    static AABB ComputeEnclosingAABB(const std::list<KinBody::LinkPtr>& linklist, const Transform& tparent)
    {
        Vector vmin, vmax;
        bool binitialized=false;
        Transform tparentinv = tparent.inverse();
        FOREACHC(itlink,linklist) {
            AABB ablink = (*itlink)->ComputeLocalAABB(); // AABB of the link in its local coordinates
            Transform tdelta = tparentinv * (*itlink)->GetTransform();
            TransformMatrix tmdelta(tdelta);
            Vector vabsextents(RaveFabs(tmdelta.m[0])*ablink.extents[0] + RaveFabs(tmdelta.m[1])*ablink.extents[1] + RaveFabs(tmdelta.m[2])*ablink.extents[2],
                               RaveFabs(tmdelta.m[4])*ablink.extents[0] + RaveFabs(tmdelta.m[5])*ablink.extents[1] + RaveFabs(tmdelta.m[6])*ablink.extents[2],
                               RaveFabs(tmdelta.m[8])*ablink.extents[0] + RaveFabs(tmdelta.m[9])*ablink.extents[1] + RaveFabs(tmdelta.m[10])*ablink.extents[2]);
            Vector vcenter = tdelta*ablink.pos;
            Vector vnmin = vcenter - vabsextents;
            Vector vnmax = vcenter + vabsextents;
            if( !binitialized ) {
                vmin = vnmin;
                vmax = vnmax;
                binitialized = true;
            }
            else {
                if( vmin.x > vnmin.x ) {
                    vmin.x = vnmin.x;
                }
                if( vmin.y > vnmin.y ) {
                    vmin.y = vnmin.y;
                }
                if( vmin.z > vnmin.z ) {
                    vmin.z = vnmin.z;
                }
                if( vmax.x < vnmax.x ) {
                    vmax.x = vnmax.x;
                }
                if( vmax.y < vnmax.y ) {
                    vmax.y = vnmax.y;
                }
                if( vmax.z < vnmax.z ) {
                    vmax.z = vnmax.z;
                }
            }
        }
        AABB ab;
        ab.pos = (dReal)0.5 * (vmin + vmax);
        ab.extents = vmax - ab.pos;
        return ab;
    }

    void Init(const std::string& manipname, const ConfigurationSpecification& spec, dReal maxmanipspeed, dReal maxmanipaccel)
    {
        _manipname = manipname;
        _maxmanipspeed = maxmanipspeed;
        _maxmanipaccel = maxmanipaccel;
        listUsedBodies.clear();
        setCheckedManips.clear();
        _listCheckManips.clear();
        spec.ExtractUsedBodies(_penv, listUsedBodies);
        FOREACH(itbody, listUsedBodies) {
            KinBodyPtr pbody = *itbody;
            if( pbody->IsRobot() ) {
                RobotBasePtr probot = RaveInterfaceCast<RobotBase>(pbody);
                RobotBase::ManipulatorPtr pmanip = probot->GetManipulator(manipname);
                if( !!pmanip ) {
                    KinBody::LinkPtr endeffector = pmanip->GetEndEffector();
                    // Insert all child links of endeffector
                    std::list<KinBody::LinkPtr> globallinklist;
                    std::vector<KinBody::LinkPtr> vchildlinks;
                    pmanip->GetChildLinks(vchildlinks);
                    FOREACH(itlink,vchildlinks) {
                        globallinklist.push_back(*itlink);
                    }
                    // Insert all links of all bodies that the endeffector is grabbing
                    std::vector<KinBodyPtr> grabbedbodies;
                    probot->GetGrabbed(grabbedbodies);
                    FOREACH(itbody,grabbedbodies) {
                        if(pmanip->IsGrabbing(*itbody)) {
                            FOREACH(itlink,(*itbody)->GetLinks()) {
                                globallinklist.push_back(*itlink);
                            }
                        }
                    }
                    // Compute the enclosing AABB and add its vertices to the checkpoints
                    AABB enclosingaabb = ComputeEnclosingAABB(globallinklist, endeffector->GetTransform());
                    _listCheckManips.push_back(ManipConstraintInfo());
                    _listCheckManips.back().pmanip = pmanip;
                    spec.ExtractUsedIndices(pmanip->GetRobot(), _listCheckManips.back().vuseddofindices, _listCheckManips.back().vconfigindices);
                    _listCheckManips.back().plink = endeffector;
                    ConvertAABBtoCheckPoints(enclosingaabb, _listCheckManips.back().checkpoints);
                    setCheckedManips.insert(endeffector);
                }
            }
        }
    }

    /// checks at each ramp's edges
    ParabolicRampInternal::CheckReturn CheckManipConstraints2(const std::vector<ParabolicRampInternal::ParabolicRampND>& outramps)
    {
        dReal maxmanipspeed=0, maxmanipaccel=0;
        Vector endeffvellin,endeffvelang,endeffacclin,endeffaccang;
        FOREACHC(itramp, outramps) {
            // have to check both ends!?
            if(itramp->endTime<=g_fEpsilonLinear) {
                continue;
            }

            // Compute the velocity and accel of the end effector COM
            FOREACHC(itmanipinfo,_listCheckManips) {
                KinBodyPtr probot = itmanipinfo->plink->GetParent();
                itramp->Accel(0, ac);
                qfillactive.resize(itmanipinfo->vuseddofindices.size());
                vfillactive.resize(itmanipinfo->vuseddofindices.size());
                afill.resize(probot->GetDOF());
                for(size_t index = 0; index < itmanipinfo->vuseddofindices.size(); ++index) {
                    qfillactive[index] = itramp->x0.at(itmanipinfo->vconfigindices.at(index));
                    vfillactive[index] = itramp->dx0.at(itmanipinfo->vconfigindices.at(index));
                    afill[itmanipinfo->vuseddofindices.at(index)] = ac.at(itmanipinfo->vconfigindices.at(index));
                }
                int endeffindex = itmanipinfo->plink->GetIndex();
                KinBody::KinBodyStateSaver saver(probot, KinBody::Save_LinkTransformation|KinBody::Save_LinkVelocities);

                // Set robot to new state
                probot->SetDOFValues(qfillactive, KinBody::CLA_CheckLimits, itmanipinfo->vuseddofindices);
                probot->SetDOFVelocities(vfillactive, KinBody::CLA_CheckLimits, itmanipinfo->vuseddofindices);
                probot->GetLinkVelocities(endeffvels);
                probot->GetLinkAccelerations(afill,endeffaccs);
                endeffvellin = endeffvels.at(endeffindex).first;
                endeffvelang = endeffvels.at(endeffindex).second;
                endeffacclin = endeffaccs.at(endeffindex).first;
                endeffaccang = endeffaccs.at(endeffindex).second;
                Transform R = itmanipinfo->plink->GetTransform();
                // For each point in checkpoints, compute its vel and acc and check whether they satisfy the manipulator constraints
                FOREACH(itpoint,itmanipinfo->checkpoints) {
                    Vector point = R.rotate(*itpoint);
                    if(_maxmanipspeed>0) {
                        Vector vpoint = endeffvellin + endeffvelang.cross(point);
                        dReal manipspeed = RaveSqrt(vpoint.lengthsqr3());
                        if( manipspeed > 1e5 ) {
                            RAVELOG_WARN_FORMAT("manip speed is too great %.15e", manipspeed);
                        }
                        if( maxmanipspeed < manipspeed ) {
                            maxmanipspeed = manipspeed;
                        }
                        if( manipspeed > _maxmanipspeed || 0.9*_maxmanipspeed < 0.001*manipspeed ) {
                            if( manipspeed > 1e5 || 0.9*_maxmanipspeed < 0.05*manipspeed ) {
                                RAVELOG_WARN_FORMAT("manip speed is too great %.15e", manipspeed);
                            }
                            return ParabolicRampInternal::CheckReturn(CFO_CheckTimeBasedConstraints, 0.9*_maxmanipspeed/manipspeed);
                        }
                    }
                    if(_maxmanipaccel>0) {
                        Vector apoint = endeffacclin + endeffvelang.cross(endeffvelang.cross(point)) + endeffaccang.cross(point);
                        dReal manipaccel = RaveSqrt(apoint.lengthsqr3());
                        if( maxmanipaccel < manipaccel ) {
                            maxmanipaccel = manipaccel;
                        }
                        if(manipaccel > _maxmanipaccel) {
                            if( manipaccel > 1e5 || 0.9*_maxmanipaccel < 0.05*manipaccel ) {
                                RAVELOG_WARN_FORMAT("manip accel is too great %.15e", manipaccel);
                            }
                            return ParabolicRampInternal::CheckReturn(CFO_CheckTimeBasedConstraints, 0.9*_maxmanipaccel/manipaccel);
                        }
                    }
                }
            }
        }
        return ParabolicRampInternal::CheckReturn(0);
    }

private:
    EnvironmentBasePtr _penv;
    std::string _manipname;
    std::vector<KinBodyPtr> listUsedBodies;
    std::set<KinBody::LinkPtr> setCheckedManips;
    dReal _maxmanipspeed, _maxmanipaccel;

    //@{cache
    std::list< ManipConstraintInfo > _listCheckManips; ///< the manipulators and the points on their end efffectors to check for velocity and acceleration constraints
    std::vector<dReal> ac, qfillactive, vfillactive; // the active DOF
    std::vector<dReal> afill; // full robot DOF
    std::vector<std::pair<Vector,Vector> > endeffvels,endeffaccs;
    //@}
};
    
} // end namespace rplanners

#endif
