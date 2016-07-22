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
    ManipConstraintInfo() : fmaxdistfromcenter(0) {
    }
    
    RobotBase::ManipulatorPtr pmanip;

    // the end-effector link of the manipulator
    KinBody::LinkPtr plink;
    // the points to check for in the end effector link's coordinate system (plink)
    // for now the checked points are the vertices of the bounding box
    // but this can be more general, e.g. the vertices of the convex hull
    std::list<Vector> checkpoints;
    dReal fmaxdistfromcenter; ///< max length of checkpoints

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

    const std::list< ManipConstraintInfo >& GetCheckManips() const { return _listCheckManips; }

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
                    OPENRAVE_ASSERT_OP(pmanip->GetArmDOF(),<=,spec.GetDOF()); // make sure the planning dof includes pmanip

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
                    ManipConstraintInfo& info = _listCheckManips.back();
                    info.pmanip = pmanip;
                    spec.ExtractUsedIndices(pmanip->GetRobot(), info.vuseddofindices, info.vconfigindices);
                    info.plink = endeffector;
                    ConvertAABBtoCheckPoints(enclosingaabb, info.checkpoints);
                    info.fmaxdistfromcenter = 0;
                    FOREACH(itpoint, info.checkpoints) {
                        dReal f = itpoint->lengthsqr3();
                        if( info.fmaxdistfromcenter < f ) {
                            info.fmaxdistfromcenter = f;
                        }
                    }
                    info.fmaxdistfromcenter = RaveSqrt(info.fmaxdistfromcenter);
                    if( IS_DEBUGLEVEL(Level_Verbose) ) {
                        std::stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
                        ss << "[";
                        FOREACH(itpoint, info.checkpoints) {
                            ss << "[" << itpoint->x << ", " << itpoint->y << ", " << itpoint->z << "], ";
                        }
                        ss << "]";
                        RAVELOG_VERBOSE_FORMAT("env=%d, fmaxdistfromcenter=%f, checkpoints=%s", pbody->GetEnv()->GetId()%info.fmaxdistfromcenter%ss.str());
                    }
                    setCheckedManips.insert(endeffector);
                }
            }
        }
    }

    /// \brief given the current robot state
    ///
    /// \brief vellimits already initialized array with the current max velocities
    /// \brief accellimits already initialized array with the current max accelerations
    void GetMaxVelocitiesAccelerations(const std::vector<dReal>& curvels, std::vector<dReal>& vellimits, std::vector<dReal>& accellimits)
    {
        if( _maxmanipspeed<=0 && _maxmanipaccel <=0) {
            return; // don't do anything
        }

        dReal maxmanipspeed2=_maxmanipspeed*_maxmanipspeed*0.5, maxmanipaccel2=_maxmanipaccel*_maxmanipaccel*0.5; // have to slow down maxes by a factor since cannot accurate predict how many joints will combine to form the real extents. 0.5 has been experimentally determined
        vector<dReal> &vbestvels2 = _vbestvels2; vbestvels2.resize(vellimits.size());
        for(size_t j = 0; j < vbestvels2.size(); ++j) {
            vbestvels2[j] = vellimits[j]*vellimits[j];
        }
        vector<dReal> &vbestaccels2 = _vbestaccels2; vbestaccels2.resize(accellimits.size());
        for(size_t j = 0; j < vbestaccels2.size(); ++j) {
            vbestaccels2[j] = accellimits[j]*accellimits[j];
        }

        uint64_t changedvelsmask=0, changedaccelsmask=0;
        OPENRAVE_ASSERT_OP(vellimits.size(),<,64);

        FOREACHC(itmanipinfo,_listCheckManips) {
            RobotBasePtr probot = itmanipinfo->pmanip->GetRobot();
            Transform tlink = itmanipinfo->plink->GetTransform();

            // compute jacobians, make sure to transform by the world frame
            probot->CalculateAngularVelocityJacobian(itmanipinfo->plink->GetIndex(), _vangularjacobian);
            probot->CalculateJacobian(itmanipinfo->plink->GetIndex(), tlink.trans, _vtransjacobian);

            int armdof = itmanipinfo->pmanip->GetArmDOF();

            // checking for each point is too slow, so use fmaxdistfromcenter instead
            //FOREACH(itpoint,itmanipinfo->checkpoints)
            {
                //Vector vdeltapoint = tlink.rotate(*itpoint);
                dReal fmaxdistfromcenter = itmanipinfo->fmaxdistfromcenter;
                
                Vector vpointtotalvel; // total work velocity from jacobian accounting all axes moving
                for(int j = 0; j < armdof; ++j) {
                    Vector vtransaxis(_vtransjacobian[j], _vtransjacobian[armdof+j], _vtransjacobian[2*armdof+j]);
                    Vector vangularaxis(_vangularjacobian[j], _vangularjacobian[armdof+j], _vangularjacobian[2*armdof+j]);
                    //vpointtotalvel += (vtransaxis + vangularaxis.cross(vdeltapoint))*curvels.at(j);
                    vpointtotalvel += vtransaxis*curvels[j];
                }
                for(int j = 0; j < armdof; ++j) {
                    Vector vangularaxis(_vangularjacobian[j], _vangularjacobian[armdof+j], _vangularjacobian[2*armdof+j]);
                    Vector vd = Vector(RaveFabs(vangularaxis.y)+RaveFabs(vangularaxis.z), RaveFabs(vangularaxis.x)+RaveFabs(vangularaxis.z), RaveFabs(vangularaxis.x)+RaveFabs(vangularaxis.y))*fmaxdistfromcenter*RaveFabs(curvels.at(j));
                    if( vpointtotalvel.x < 0 ) {
                        vd.x = -vd.x;
                    }
                    if( vpointtotalvel.y < 0 ) {
                        vd.y = -vd.y;
                    }
                    if( vpointtotalvel.z < 0 ) {
                        vd.z = -vd.z;
                    }
                    vpointtotalvel += vd;
                }
                
                for(int j = 0; j < armdof; ++j) {
                    Vector vtransaxis(_vtransjacobian[j], _vtransjacobian[armdof+j], _vtransjacobian[2*armdof+j]);
                    Vector vangularaxis(_vangularjacobian[j], _vangularjacobian[armdof+j], _vangularjacobian[2*armdof+j]);
                    Vector vmoveaxis = vtransaxis;// + vangularaxis.cross(vdeltapoint);
                    Vector vpointvelbase = vpointtotalvel-vmoveaxis*curvels.at(j); // remove contribution of this point

                    // |vpointvelbase + vmoveaxis*jointvel| <= maxspeed
                    // vmoveaxis^2 * jointvel^2 + vpointvelbase.dot(vmoveaxis)*jointvel + vpointvelbase^2 <= maxspeed^2
                    // solve for jointvel and see if beyond vellimits
                    
                    if( maxmanipspeed2 > 0 ) {
                        dReal roots[2];
                        int numroots = mathextra::solvequad(vmoveaxis.lengthsqr3(), vpointvelbase.dot3(vmoveaxis), vpointvelbase.lengthsqr3() - maxmanipspeed2, roots[0], roots[1]);
                        if( numroots == 0 ) {
                            if( vpointvelbase.lengthsqr3() > maxmanipspeed2 ) {
                                // going way too fast right now!
                                if( curvels[j] > 0 ) {
                                    vellimits[j] = curvels[j];
                                }
                                else if( curvels[j] < 0 ) {
                                    vellimits[j] = -curvels[j];
                                }
                            }
                        }
                        else {
                            for(int iroot = 0; iroot < numroots; ++iroot ) {
                                dReal r = RaveFabs(roots[iroot]);
                                if( r > 0 && r < vellimits[j] ) {
                                    vellimits[j] = r;
                                }
                            }
                        }
                    }
                    
                    //Vector vabstotalmove(RaveFabs(vpointvelbase.x)
//                    dReal fworkspeed2 = vmoveaxis.lengthsqr3(); ///< total speed in work space of *itpoint
//
//                    if( maxmanipspeed2 > 0 ) {
//                        // sqrt(flen2) * vel <= maxspeed
//                        if( fworkspeed2 * vbestvels2[j] >= maxmanipspeed2 ) {
//                            vbestvels2[j] = maxmanipspeed2/fworkspeed2;
//                            changedvelsmask |= (1<<j);
//                        }
//                    }
//
                    if( maxmanipaccel2 > 0 ) {
                        // TODO should really be using the hessian here accel = Jacobian * dofaccelerations + dofvelocities^T * Hessian * dofvelocities, but it might be too slow, so just approximate with
                        // accel = Jacobian * dofaccelerations
                        Vector vaccelaxis = vmoveaxis;//(RaveFabs(vmoveaxis.x), RaveFabs(vmoveaxis.y), RaveFabs(vmoveaxis.z));
                        dReal fworkaccel2 = vaccelaxis.lengthsqr3(); ///< total speed in work space of *itpoint

                        if( fworkaccel2 * vbestaccels2[j] >= maxmanipaccel2 ) {
                            vbestaccels2[j] = maxmanipaccel2/fworkaccel2;
                            changedaccelsmask |= (1<<j);
                        }
                    }
                }
            }
        }

        // go through all the changes and update vellimits/accellimits
        for(size_t j = 0; j < vellimits.size(); ++j) {
            if( changedvelsmask & (1<<j) ) {
                vellimits[j] = RaveSqrt(vbestvels2[j]);
            }
            if( changedaccelsmask & (1<<j) ) {
                accellimits[j] = RaveSqrt(vbestaccels2[j]);
            }
        }
    }

    
    /// checks at each ramp's edges. This is called in the critical loop
    ParabolicRampInternal::CheckReturn CheckManipConstraints2(const std::vector<ParabolicRampInternal::ParabolicRampND>& outramps)
    {
        if( _maxmanipspeed<=0 && _maxmanipaccel <=0) {
            return ParabolicRampInternal::CheckReturn(0);
        }

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
                _vfillactive.resize(itmanipinfo->vuseddofindices.size());
                _afill.resize(probot->GetDOF());
                for(size_t index = 0; index < itmanipinfo->vuseddofindices.size(); ++index) {
                    qfillactive[index] = itramp->x0.at(itmanipinfo->vconfigindices.at(index));
                    _vfillactive[index] = itramp->dx0.at(itmanipinfo->vconfigindices.at(index));
                    _afill[itmanipinfo->vuseddofindices.at(index)] = ac.at(itmanipinfo->vconfigindices.at(index));
                }
                int endeffindex = itmanipinfo->plink->GetIndex();
                KinBody::KinBodyStateSaver saver(probot, KinBody::Save_LinkTransformation|KinBody::Save_LinkVelocities);

                // Set robot to new state
                probot->SetDOFValues(qfillactive, KinBody::CLA_CheckLimits, itmanipinfo->vuseddofindices);
                probot->SetDOFVelocities(_vfillactive, KinBody::CLA_CheckLimits, itmanipinfo->vuseddofindices);
                probot->GetLinkVelocities(endeffvels);
                probot->GetLinkAccelerations(_afill,endeffaccs);
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
                            RAVELOG_VERBOSE_FORMAT("ramp %d/%d: maxmanipspeed = %.15e; manipspeed = %.15e", (itramp - outramps.begin())%outramps.size()%_maxmanipspeed%manipspeed);
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
                            RAVELOG_VERBOSE_FORMAT("ramp %d/%d: maxmanipaccel = %.15e; manipaccel = %.15e", (itramp - outramps.begin())%outramps.size()%_maxmanipaccel%manipaccel);
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

    //@{ cache
    std::list< ManipConstraintInfo > _listCheckManips; ///< the manipulators and the points on their end efffectors to check for velocity and acceleration constraints
    std::vector<dReal> ac, qfillactive, _vfillactive; // the active DOF
    std::vector<dReal> _afill; // full robot DOF
    std::vector<std::pair<Vector,Vector> > endeffvels, endeffaccs;
    std::vector<dReal> _vtransjacobian, _vangularjacobian, _vbestvels2, _vbestaccels2;
    //@}
};

} // end namespace rplanners

#endif
