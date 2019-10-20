// -*- coding: utf-8 -*-
// Copyright (C) 2016 Rosen Diankov
//
// This program is free software: you can redistribute it and/or modify it under the terms of the
// GNU Lesser General Public License as published by the Free Software Foundation, either version 3
// of the License, or at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
// even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License along with this program.
// If not, see <http://www.gnu.org/licenses/>.
#ifndef OPENRAVE_MANIP_CONSTRAINT2_H
#define OPENRAVE_MANIP_CONSTRAINT2_H

#include "openraveplugindefs.h"
#include "rampoptimizer/ramp.h"
#include "rampoptimizer/feasibilitychecker.h"

//#define PROGRESS_DEBUG

namespace rplanners {

struct ManipConstraintInfo2
{
    ManipConstraintInfo2() : fmaxdistfromcenter(0) {
    }

    RobotBase::ManipulatorPtr pmanip; ///< the manipulator
    KinBody::LinkPtr plink; ///< the end-effector of the manipulator
    std::list<Vector> checkpoints; ///< points (in EE frame) at which to check
                                   ///manipconstraints. Currently they are vertices of the bounding
                                   ///box but they can be more general, e.g., the vertices of the
                                   ///convex hull.
    dReal fmaxdistfromcenter; ///< maximum distance from any check point to the EE center
    std::vector<int> vuseddofindices; ///< a vector of unique DOF indices targetted for the body
    std::vector<int> vconfigindices;  ///< for every index in vusedofindices, returns the first configuration space index it came from
};

class ManipConstraintChecker2
{
public:
    ManipConstraintChecker2(EnvironmentBasePtr penv) : _penv(penv), _maxmanipspeed(0), _maxmanipaccel(0) {
    }

    /// \brief Given a world AABB oriented, return its 8 vertices. All vertices are describted in the parent frame (see ComputeEnclosingAABB).
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

    const std::list< ManipConstraintInfo2 >& GetCheckManips() const {
        return _listCheckManips;
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

        _vdotproducts.resize(spec.GetDOF());
        _vindices.resize(spec.GetDOF());
        _vscalingfactors.resize(spec.GetDOF());

        FOREACH(itbody, listUsedBodies) {
            KinBodyPtr pbody = *itbody;
            if( pbody->IsRobot() ) {
                RobotBasePtr probot = RaveInterfaceCast<RobotBase>(pbody);
                RobotBase::ManipulatorPtr pmanip = probot->GetManipulator(manipname);
                if( !!pmanip ) {
                    OPENRAVE_ASSERT_OP(pmanip->GetArmDOF(),<=,spec.GetDOF()); // make sure the planning dof includes pmanip

                    KinBody::LinkPtr endeffector = pmanip->GetEndEffector();

                    // When the manipulator is grabbing items, globallinklist will contain all the grabbed
                    // links. Otherwise, globallinklist will contain the manip's children links. These links will be
                    // used for computing AABB to obtain checkpoints.
                    std::list<KinBody::LinkPtr> globallinklist;

                    std::vector<KinBodyPtr> grabbedbodies;
                    probot->GetGrabbed(grabbedbodies);

                    if( grabbedbodies.size() > 0 ) {
                        FOREACH(itbody, grabbedbodies) {
                            if( pmanip->IsGrabbing(**itbody) ) {
                                FOREACH(itlink, (*itbody)->GetLinks()) {
                                    globallinklist.push_back(*itlink);
                                }
                            }
                        }
                    }
                    else {
                        std::vector<KinBody::LinkPtr> vchildlinks;
                        pmanip->GetChildLinks(vchildlinks);
                        FOREACH(itlink, vchildlinks) {
                            globallinklist.push_back(*itlink);
                        }
                    }

                    // Compute the enclosing AABB and add its vertices to the checkpoints
                    AABB enclosingaabb = ComputeEnclosingAABB(globallinklist, endeffector->GetTransform());
                    _listCheckManips.push_back(ManipConstraintInfo2());
                    ManipConstraintInfo2& info = _listCheckManips.back();
                    info.pmanip = pmanip;
                    spec.ExtractUsedIndices(pmanip->GetRobot(), info.vuseddofindices, info.vconfigindices);
                    info.plink = endeffector;
                    ConvertAABBtoCheckPoints(enclosingaabb, info.checkpoints);
                    info.fmaxdistfromcenter = 0; // maximum distance between any checkpoints to the end-efffector
                    FOREACH(itpoint, info.checkpoints) {
                        dReal f = itpoint->lengthsqr3();
                        if( info.fmaxdistfromcenter < f ) {
                            info.fmaxdistfromcenter = f;
                        }
                    }
                    info.fmaxdistfromcenter = RaveSqrt(info.fmaxdistfromcenter);
#ifdef PROGRESS_DEBUG
                    std::stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
                    ss << "[";
                    FOREACH(itpoint, info.checkpoints) {
                        ss << "[" << itpoint->x << ", " << itpoint->y << ", " << itpoint->z << "], ";
                    }
                    ss << "]";
                    RAVELOG_DEBUG_FORMAT("env=%d, fmaxdistfromcenter=%f, checkpoints=%s", pbody->GetEnv()->GetId()%info.fmaxdistfromcenter%ss.str());
#endif
                    setCheckedManips.insert(endeffector);
                }
            }
        }
    }

    // Estimate the max vellimits and accellimits that should be used in the next shortcut iteration based on how fast
    // checkpoints move and how much vellimits and accellimits are violated.
    void GetMaxVelocitiesAccelerations(const std::vector<dReal>& curvels, std::vector<dReal>& vellimits, std::vector<dReal>& accellimits)
    {
        if( _maxmanipspeed<=0 && _maxmanipaccel <=0) {
            return; // don't do anything
        }

        // have to slow down maxes by a factor since cannot accurate predict how many joints will
        // combine to form the real extents. 0.5 has been experimentally determined
        dReal maxmanipspeed2=_maxmanipspeed*_maxmanipspeed*0.5, maxmanipaccel2=_maxmanipaccel*_maxmanipaccel*0.5;
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

                    // Vector vabstotalmove(RaveFabs(vpointvelbase.x));
                    // dReal fworkspeed2 = vmoveaxis.lengthsqr3(); ///< total speed in work space of *itpoint
                    // if( maxmanipspeed2 > 0 ) {
                    //     // sqrt(flen2) * vel <= maxspeed
                    //     if( fworkspeed2 * vbestvels2[j] >= maxmanipspeed2 ) {
                    //         vbestvels2[j] = maxmanipspeed2/fworkspeed2;
                    //         changedvelsmask |= (1<<j);
                    //     }
                    // }

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

    // Check manipulator's linear velocity and acceleration if they violate the given limits (_maxmanipspeed and
    // _maxmanipaccel). Linear velocity and accelerations are computed at checkpoints (itmanipinfo->checkpoints).
    //
    // Checking is done only at the start and the end of each parabolic segment. Within one parabolic segment,
    // manipaccel tends to increase or decrease monotonically and therefore, the maximum is occuring at either end. We
    // can reduce computational load by only checking at both end instead of checking at every subdivided segment and
    // still having the same result.
    //
    // After having found a point where manip constraints are violated, we compute Jacobian at that configuration and
    // use it to estimate contribution of each dof to the velocity/acceleration at the checkpoint so as to scale the
    // vellimits/accellimits of that dof down accordingly.
    //
    // Notes:
    // 1. For acceleration computation, we estimate the relation between joint accel and end-effector accel to be
    //    a = J * qdd
    //    to save computation time.
    // 2. We compute the contribution of each dof to "linear" velocity/acceleration of the "end-effector point" instead of
    //    the actual checkpoint.
    // 3. Consider velocity, for example. From the Jacobian equation v = J*qd, we have that how the velocity of each dof
    //    contributes to the linear eff velocity depends on the respective column of J. For example, if the i-th column
    //    of J is perpendicular to v, then dof i does not contribute to the velocity limit violation at that
    //    moment. Therefore, we can give a score to each dof to indicate how much it contributes to the eff velocity,
    //    and this score is computed from the dot product between the respective column of J and v. Now we rank dofs
    //    based on their contributions and the dof with the most contribution will have its velocity limit scaled down
    //    the most, and so on.
    // 4. If acceleration limits are violated, the scaling factors will be computed from the acceleration equation,
    //    regardless of whether or not velocity limits are violated. Otherwise, if velocity limits are violated, the
    //    scaling factors will be computed from the velocity equation.
    RampOptimizerInternal::CheckReturn CheckManipConstraints2(const std::vector<RampOptimizerInternal::RampND> &rampndVect, IntervalType interval=IT_OpenStart, bool bUseNewHeuristic=true)
    {
        if( _maxmanipspeed <= 0 && _maxmanipaccel <= 0 ) {
            return RampOptimizerInternal::CheckReturn(0);
        }

        BOOST_ASSERT(!(interval == IT_Open));
        dReal reductionFactor = 1; // default reduction factor
        dReal reductionFactorCutoff = 0.8; // If the originally computed reductionFactor is *not* less than this value,
                                           // we don't computed scaling factors separately for each DOF and use the
                                           // usual procedure.
        dReal fMaxReductionFactor = 1; // scaling factor for the DOF with least contribution to constriant violation

        Vector endeffvellin, endeffvelang, endeffacclin, endeffaccang;
        dReal maxactualmanipspeed = 0, maxactualmanipaccel = 0;

        dReal multiplier = 0.85;     // a multiplier to the scaling factor computed from the ratio between the violating value and the bound
        int retcode = 0;
        dReal maxallowedmult = 0.92; // the final reduction factor should not less than this value

        // Check manipspeed and manipaccel at the beginning of the segment
        std::vector<RampOptimizerInternal::RampND>::const_iterator itrampnd = rampndVect.begin();
        int velViolationIndex = -1; // index to manipinfo that violates manip vel constraint (-1 if constraints are respected)
        int accelViolationIndex = -1; // index to manipinfo that violates manip accel constraint (-1 if constraints are respected)
        Vector vVelViolation, vAccelViolation;
        int curmanipindex = 0;
        bool bBoundExceeded = false;

        std::vector<dReal>& vDOFValuesAtViolation = _vdofvalues, &vDOFVelAtViolation = _vdofvelocities, &vDOFAccelAtViolation = _vdofaccelerations;
        vDOFValuesAtViolation.resize(0);
        vDOFVelAtViolation.resize(0);
        vDOFAccelAtViolation.resize(0);

        if( !(interval == IT_OpenStart) ) {
            FOREACHC(itmanipinfo, _listCheckManips) {
                bBoundExceeded = false;
                KinBodyPtr probot = itmanipinfo->plink->GetParent();
                itrampnd->GetAVect(ac);
                qfillactive.resize(itmanipinfo->vuseddofindices.size());
                _vfillactive.resize(itmanipinfo->vuseddofindices.size());
                _afill.resize(probot->GetDOF());
                for(size_t index = 0; index < itmanipinfo->vuseddofindices.size(); ++index) {
                    qfillactive[index] = itrampnd->GetX0At(itmanipinfo->vconfigindices.at(index));
                    _vfillactive[index] = itrampnd->GetV0At(itmanipinfo->vconfigindices[index]);
                    _afill[itmanipinfo->vuseddofindices.at(index)] = ac.at(itmanipinfo->vconfigindices[index]);
                }

                int endeffindex = itmanipinfo->plink->GetIndex();
                KinBody::KinBodyStateSaver saver(probot, KinBody::Save_LinkTransformation|KinBody::Save_LinkVelocities);

                // Set the robot to the new state
                probot->SetDOFValues(qfillactive, KinBody::CLA_CheckLimits, itmanipinfo->vuseddofindices);
                probot->SetDOFVelocities(_vfillactive, KinBody::CLA_CheckLimits, itmanipinfo->vuseddofindices);
                probot->GetLinkVelocities(endeffvels);
                probot->GetLinkAccelerations(_afill,endeffaccs);
                endeffvellin = endeffvels.at(endeffindex).first;
                endeffvelang = endeffvels.at(endeffindex).second;
                endeffacclin = endeffaccs.at(endeffindex).first;
                endeffaccang = endeffaccs.at(endeffindex).second;
                Transform R = itmanipinfo->plink->GetTransform();

                FOREACH(itpoint, itmanipinfo->checkpoints) {
                    Vector point = R.rotate(*itpoint);

                    if( _maxmanipspeed > 0 ) {
                        // Compute the linear velocity: v_total = v + w x r
                        Vector vpoint = endeffvellin + endeffvelang.cross(point);
                        dReal actualmanipspeed = RaveSqrt(vpoint.lengthsqr3());
                        if( actualmanipspeed > maxactualmanipspeed ) {
                            bBoundExceeded = true;
                            maxactualmanipspeed = actualmanipspeed;
                            velViolationIndex = curmanipindex;
                            vVelViolation = vpoint;
                        }
                    }

                    if( _maxmanipaccel > 0 ) {
                        // Compute the linear acceleration: a_total = a + w x (w x r) + (alpha x r)
                        Vector apoint = endeffacclin + endeffvelang.cross(endeffvelang.cross(point)) + endeffaccang.cross(point);
                        dReal actualmanipaccel = RaveSqrt(apoint.lengthsqr3());
                        if( actualmanipaccel > maxactualmanipaccel ) {
                            bBoundExceeded = true;
                            maxactualmanipaccel = actualmanipaccel;
                            accelViolationIndex = curmanipindex;
                            vAccelViolation = apoint;
                        }
                    }
                }
                if( bBoundExceeded ) {
                    // Keep these values for later computation if constraints are violated
                    vDOFValuesAtViolation = qfillactive;
                    vDOFVelAtViolation = _vfillactive;
                    vDOFAccelAtViolation = _afill;
                }
                // Finished iterating through all checkpoints
                ++curmanipindex;
            }
            // Finished iterating through all manipulators

            if( bUseNewHeuristic ) {
                RampOptimizerInternal::CheckReturn retcheck;
                retcheck.retcode = 0;
                retcheck.fTimeBasedSurpassMult = reductionFactor;
                retcheck.fMaxManipSpeed = maxactualmanipspeed;
                retcheck.fMaxManipAccel = maxactualmanipaccel;

                if( _maxmanipaccel > 0 && maxactualmanipaccel > _maxmanipaccel ) {
                    // Accel limits are violated
                    reductionFactor = RaveSqrt(min(multiplier*_maxmanipaccel/maxactualmanipaccel, maxallowedmult));
                    retcheck.retcode = CFO_CheckTimeBasedConstraints;
                    retcheck.fTimeBasedSurpassMult = reductionFactor;
                    if( reductionFactor >= reductionFactorCutoff ) {
                        // Constraints are not severely violated. Don't bother to compute Jacobian and that stuff
                        return retcheck;
                    }

                    std::list< ManipConstraintInfo2 >::iterator itmanipinfo = _listCheckManips.begin();
                    std::advance(itmanipinfo, accelViolationIndex);
                    RobotBasePtr probot = itmanipinfo->pmanip->GetRobot();
                    KinBody::KinBodyStateSaver saver(probot, KinBody::Save_LinkTransformation|KinBody::Save_LinkVelocities);
                    // Set the robot to the new state
                    probot->SetDOFValues(vDOFValuesAtViolation, KinBody::CLA_CheckLimits, itmanipinfo->vuseddofindices);
                    Transform tlink = itmanipinfo->plink->GetTransform();
                    // compute jacobians, make sure to transform by the world frame
                    probot->CalculateJacobian(itmanipinfo->plink->GetIndex(), tlink.trans, _vtransjacobian);
                    int armdof = itmanipinfo->pmanip->GetArmDOF();
                    for( int idof = 0; idof < armdof; ++idof ) {
                        Vector vtransaxis(_vtransjacobian[idof], _vtransjacobian[armdof+idof], _vtransjacobian[2*armdof+idof]);
                        _vdotproducts[idof] = vDOFAccelAtViolation[idof] > 0 ? vtransaxis.dot3(vAccelViolation) : -vtransaxis.dot3(vAccelViolation);
                        if( RaveFabs(_vdotproducts[idof]) <= g_fEpsilonLinear ) {
                            _vdotproducts[idof] = 0;
                        }
                    }
                    // sort while keeping indices
                    std::size_t n(0);
                    std::generate(_vindices.begin(), _vindices.end(), [&] { return n++; });
                    std::sort(_vindices.begin(), _vindices.end(), [&](int i1, int i2) {
                            return _vdotproducts[i1] < _vdotproducts[i2];
                        });
                    dReal minPositiveDotProduct = _vdotproducts[_vindices.back()];
                    if( minPositiveDotProduct <= 0 ) {
                        // Is this possible?
                        return retcheck;
                    }
                    int minPositiveDotProductIndex = 0;
                    for( int i = 0; i < armdof; ++i ) {
                        minPositiveDotProduct = _vdotproducts[_vindices[i]];
                        _vscalingfactors[_vindices[i]] = 1.0;
                        if( minPositiveDotProduct > 0 ) {
                            minPositiveDotProductIndex = i;
                            break;
                        }
                    }
                    // Suppose the positive dot products are d1, d2, ..., dn (sorted in the ascending
                    // order). We want to build a linear function f (for convenience) such that f(d1) =
                    // rmax (least scaling for the DOF with least contribution) and f(dn) = reductionFactor
                    // (large reduction factor for DOF with most contribution). Since we build a linear
                    // function, we have
                    //         f(i) = m(d1/di) + (rmax - m)
                    // where m = (rmax - reductionFactor)/(1 - d1/dn)
                    if( minPositiveDotProductIndex == (int)(_vdotproducts.size() - 1) ) {
                        // Cannot use the above formula since the denominator is zero so just setting the scaling factor for this one dof.
                        _vscalingfactors[_vindices[minPositiveDotProductIndex]] = reductionFactor;
                    }
                    else {
                        dReal m = (fMaxReductionFactor - reductionFactor) / (1 - minPositiveDotProduct/_vdotproducts[_vindices.back()]);
                        for( int i = minPositiveDotProductIndex; i < armdof; ++i ) {
                            int idof = _vindices[i];
                            _vscalingfactors[idof] = m*(minPositiveDotProduct/_vdotproducts[idof]) + (fMaxReductionFactor - m);
                        }
                    }
                    retcheck.vReductionFactors = _vscalingfactors;
#ifdef PROGRESS_DEBUG
                    std::stringstream ss; ss << "env=" << probot->GetEnv()->GetId() << "; reductionFactor=" << reductionFactor << "; minPositiveDotProductIndex=" << minPositiveDotProductIndex << "; vdotproducts=[";
                    FOREACHC(itval, _vdotproducts) {
                        ss << *itval << ", ";
                    }
                    ss << "]; vindices=[";
                    FOREACHC(itval, _vindices) {
                        ss << *itval << ", ";
                    }
                    ss << "]; vscalingfactors=[";
                    FOREACHC(itval, _vscalingfactors) {
                        ss << *itval << ", ";
                    }
                    ss << "];";
                    RAVELOG_DEBUG(ss.str());
#endif
                }
                else if( _maxmanipspeed > 0 && maxactualmanipspeed > _maxmanipspeed ) {
                    // Vel limits are violated
                    reductionFactor = min(multiplier*_maxmanipspeed/maxactualmanipspeed, maxallowedmult);
                    retcheck.retcode = CFO_CheckTimeBasedConstraints;
                    retcheck.fTimeBasedSurpassMult = reductionFactor;
                    if( reductionFactor >= reductionFactorCutoff ) {
                        // Constraints are not severely violated. Don't bother to compute Jacobian and that stuff
                        return retcheck;
                    }

                    std::list< ManipConstraintInfo2 >::iterator itmanipinfo = _listCheckManips.begin();
                    std::advance(itmanipinfo, velViolationIndex);
                    RobotBasePtr probot = itmanipinfo->pmanip->GetRobot();
                    KinBody::KinBodyStateSaver saver(probot, KinBody::Save_LinkTransformation|KinBody::Save_LinkVelocities);
                    // Set the robot to the new state
                    probot->SetDOFValues(vDOFValuesAtViolation, KinBody::CLA_CheckLimits, itmanipinfo->vuseddofindices);
                    Transform tlink = itmanipinfo->plink->GetTransform();
                    // compute jacobians, make sure to transform by the world frame
                    probot->CalculateJacobian(itmanipinfo->plink->GetIndex(), tlink.trans, _vtransjacobian);
                    int armdof = itmanipinfo->pmanip->GetArmDOF();
                    for( int idof = 0; idof < armdof; ++idof ) {
                        Vector vtransaxis(_vtransjacobian[idof], _vtransjacobian[armdof+idof], _vtransjacobian[2*armdof+idof]);
                        _vdotproducts[idof] = vDOFVelAtViolation[idof] > 0 ? vtransaxis.dot3(vVelViolation) : -vtransaxis.dot3(vVelViolation);
                        if( RaveFabs(_vdotproducts[idof]) <= g_fEpsilonLinear ) {
                            _vdotproducts[idof] = 0;
                        }
                    }
                    // sort while keeping indices
                    std::size_t n(0);
                    std::generate(_vindices.begin(), _vindices.end(), [&] { return n++; });
                    std::sort(_vindices.begin(), _vindices.end(), [&](int i1, int i2) {
                            return _vdotproducts[i1] < _vdotproducts[i2];
                        });
                    dReal minPositiveDotProduct = _vdotproducts[_vindices.back()];
                    if( minPositiveDotProduct <= 0 ) {
                        // Is this possible?
                        return retcheck;
                    }
                    int minPositiveDotProductIndex = 0;
                    for( int i = 0; i < armdof; ++i ) {
                        minPositiveDotProduct = _vdotproducts[_vindices[i]];
                        _vscalingfactors[_vindices[i]] = 1.0;
                        if( minPositiveDotProduct > 0 ) {
                            minPositiveDotProductIndex = i;
                            break;
                        }
                    }
                    // Suppose the positive dot products are d1, d2, ..., dn (sorted in the ascending
                    // order). We want to build a linear function f (for convenience) such that f(d1) =
                    // 1 (no scaling for the DOF with least contribution) and f(dn) = reductionFactor
                    // (large reduction factor for DOF with most contribution). Since we build a linear
                    // function, we have
                    //         f(i) = m(d1/di) + (rmax - m)
                    // where m = (rmax - reductionFactor)/(1 - d1/dn)
                    if( minPositiveDotProductIndex == (int)(_vdotproducts.size() - 1) ) {
                        // Cannot use the above formula since the denominator is zero so just setting the scaling factor for this one dof.
                        _vscalingfactors[_vindices[minPositiveDotProductIndex]] = reductionFactor;
                    }
                    else {
                        dReal m = (fMaxReductionFactor - reductionFactor) / (1 - minPositiveDotProduct/_vdotproducts[_vindices.back()]);
                        for( int i = minPositiveDotProductIndex; i < armdof; ++i ) {
                            int idof = _vindices[i];
                            _vscalingfactors[idof] = m*(minPositiveDotProduct/_vdotproducts[idof]) + (fMaxReductionFactor - m);
                        }
                    }
                    retcheck.vReductionFactors = _vscalingfactors;
#ifdef PROGRESS_DEBUG
                    std::stringstream ss; ss << "env=" << probot->GetEnv()->GetId() << "; reductionFactor=" << reductionFactor << "; minPositiveDotProductIndex=" << minPositiveDotProductIndex << "; vdotproducts=[";
                    FOREACHC(itval, _vdotproducts) {
                        ss << *itval << ", ";
                    }
                    ss << "]; vindices=[";
                    FOREACHC(itval, _vindices) {
                        ss << *itval << ", ";
                    }
                    ss << "]; vscalingfactors=[";
                    FOREACHC(itval, _vscalingfactors) {
                        ss << *itval << ", ";
                    }
                    ss << "];";
                    RAVELOG_DEBUG(ss.str());
#endif
                }
                return retcheck;
            }
            else {
                bBoundExceeded = false;
                if( _maxmanipspeed > 0 && maxactualmanipspeed > _maxmanipspeed ) {
                    bBoundExceeded = true;
                    retcode = CFO_CheckTimeBasedConstraints;
                    // If the actual max value is very close to the bound (i.e., almost not violating
                    // the bound), the multiplier will be too large (too close to 1) to be useful.
                    reductionFactor = min(multiplier*_maxmanipspeed/maxactualmanipspeed, maxallowedmult);
                }
                if( _maxmanipaccel > 0 && maxactualmanipaccel > _maxmanipaccel ) {
                    bBoundExceeded = true;
                    retcode = CFO_CheckTimeBasedConstraints;
                    // If the actual max value is very close to the bound (i.e., almost not violating
                    // the bound), the multiplier will be too large (too close to 1) to be useful.
                    reductionFactor = RaveSqrt(min(multiplier*_maxmanipaccel/maxactualmanipaccel, maxallowedmult));
                }
            }

            // The ramp passes all the check and it's very short so return now.
            if( (itrampnd == rampndVect.end() - 1) && (itrampnd->GetDuration() <= g_fEpsilonLinear) ) {
                // No further checking is needed since the segment contains only one very small ramp.
                if( bBoundExceeded ) {
#ifdef PROGRESS_DEBUG
                    std::stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
                    ss << "q=[";
                    SerializeValues(ss, vDOFValuesAtViolation);
                    ss << "]; qd=[";
                    SerializeValues(ss, vDOFVelAtViolation);
                    ss << "]; qdd=[";
                    SerializeValues(ss, vDOFAccelAtViolation);
                    ss << "];";
                    RAVELOG_VERBOSE_FORMAT("env=%d, maxmanipspeed=%.15e; maxactualmanipspeed=%.15e; maxmanipaccel=%.15e; maxactualmanipaccel=%.15e; reductionFactor=%.15e; %s", _penv->GetId()%_maxmanipspeed%maxactualmanipspeed%_maxmanipaccel%maxactualmanipaccel%reductionFactor%ss.str());
#endif
                    return RampOptimizerInternal::CheckReturn(retcode, reductionFactor, maxactualmanipspeed, maxactualmanipaccel);
                }
                else {
                    return RampOptimizerInternal::CheckReturn(0);
                }
            }
        }

        // Check manipspeed and manipaccel at the end of the segment
        itrampnd = rampndVect.end() - 1;
        curmanipindex = 0;
        FOREACHC(itmanipinfo, _listCheckManips) {
            bBoundExceeded = false;
            KinBodyPtr probot = itmanipinfo->plink->GetParent();
            itrampnd->GetAVect(ac);
            qfillactive.resize(itmanipinfo->vuseddofindices.size());
            _vfillactive.resize(itmanipinfo->vuseddofindices.size());
            _afill.resize(probot->GetDOF());
            for(size_t index = 0; index < itmanipinfo->vuseddofindices.size(); ++index) {
                qfillactive[index] = itrampnd->GetX1At(itmanipinfo->vconfigindices.at(index));
                _vfillactive[index] = itrampnd->GetV1At(itmanipinfo->vconfigindices[index]);
                _afill[itmanipinfo->vuseddofindices.at(index)] = ac.at(itmanipinfo->vconfigindices[index]);
            }

            int endeffindex = itmanipinfo->plink->GetIndex();
            KinBody::KinBodyStateSaver saver(probot, KinBody::Save_LinkTransformation|KinBody::Save_LinkVelocities);

            // Set the robot to the new state
            probot->SetDOFValues(qfillactive, KinBody::CLA_CheckLimits, itmanipinfo->vuseddofindices);
            probot->SetDOFVelocities(_vfillactive, KinBody::CLA_CheckLimits, itmanipinfo->vuseddofindices);
            probot->GetLinkVelocities(endeffvels);
            probot->GetLinkAccelerations(_afill,endeffaccs);
            endeffvellin = endeffvels.at(endeffindex).first;
            endeffvelang = endeffvels.at(endeffindex).second;
            endeffacclin = endeffaccs.at(endeffindex).first;
            endeffaccang = endeffaccs.at(endeffindex).second;
            Transform R = itmanipinfo->plink->GetTransform();

            FOREACH(itpoint, itmanipinfo->checkpoints) {
                Vector point = R.rotate(*itpoint);

                if( _maxmanipspeed > 0 ) {
                    // Compute the linear velocity: v_total = v + w x r
                    Vector vpoint = endeffvellin + endeffvelang.cross(point);
                    dReal actualmanipspeed = RaveSqrt(vpoint.lengthsqr3());
                    if( actualmanipspeed > maxactualmanipspeed ) {
                        bBoundExceeded = true;
                        maxactualmanipspeed = actualmanipspeed;
                        velViolationIndex = curmanipindex;
                        vVelViolation = vpoint;
                    }
                }

                if( _maxmanipaccel > 0 ) {
                    // Compute the linear acceleration: a_total = a + w x (w x r) + (alpha x r)
                    Vector apoint = endeffacclin + endeffvelang.cross(endeffvelang.cross(point)) + endeffaccang.cross(point);
                    dReal actualmanipaccel = RaveSqrt(apoint.lengthsqr3());
                    if( actualmanipaccel > maxactualmanipaccel ) {
                        bBoundExceeded = true;
                        maxactualmanipaccel = actualmanipaccel;
                        accelViolationIndex = curmanipindex;
                        vAccelViolation = apoint;
                    }
                }
            }
            if( bBoundExceeded ) {
                vDOFValuesAtViolation = qfillactive;
                vDOFVelAtViolation = _vfillactive;
                vDOFAccelAtViolation = _afill;
            }
            // Finished iterating through all checkpoints
            ++curmanipindex;
        }
        // Finished iterating through all manipulators

        if( bUseNewHeuristic ) {
            RampOptimizerInternal::CheckReturn retcheck;
            retcheck.retcode = 0;
            retcheck.fTimeBasedSurpassMult = reductionFactor;
            retcheck.fMaxManipSpeed = maxactualmanipspeed;
            retcheck.fMaxManipAccel = maxactualmanipaccel;

            if( _maxmanipaccel > 0 && maxactualmanipaccel > _maxmanipaccel ) {
                reductionFactor = RaveSqrt(min(multiplier*_maxmanipaccel/maxactualmanipaccel, maxallowedmult));
                retcheck.retcode = CFO_CheckTimeBasedConstraints;
                retcheck.fTimeBasedSurpassMult = reductionFactor;
                if( reductionFactor >= reductionFactorCutoff ) {
                    // Constraints are not severely violated. Don't bother to compute Jacobian and that stuff
                    return retcheck;
                }

                std::list< ManipConstraintInfo2 >::iterator itmanipinfo = _listCheckManips.begin();
                std::advance(itmanipinfo, accelViolationIndex);
                RobotBasePtr probot = itmanipinfo->pmanip->GetRobot();
                KinBody::KinBodyStateSaver saver(probot, KinBody::Save_LinkTransformation|KinBody::Save_LinkVelocities);
                // Set the robot to the new state
                probot->SetDOFValues(vDOFValuesAtViolation, KinBody::CLA_CheckLimits, itmanipinfo->vuseddofindices);
                Transform tlink = itmanipinfo->plink->GetTransform();
                // compute jacobians, make sure to transform by the world frame
                probot->CalculateJacobian(itmanipinfo->plink->GetIndex(), tlink.trans, _vtransjacobian);
                int armdof = itmanipinfo->pmanip->GetArmDOF();
                for( int idof = 0; idof < armdof; ++idof ) {
                    Vector vtransaxis(_vtransjacobian[idof], _vtransjacobian[armdof+idof], _vtransjacobian[2*armdof+idof]);
                    _vdotproducts[idof] = vDOFAccelAtViolation[idof] > 0 ? vtransaxis.dot3(vAccelViolation) : -vtransaxis.dot3(vAccelViolation);
                    if( RaveFabs(_vdotproducts[idof]) <= g_fEpsilonLinear ) {
                        _vdotproducts[idof] = 0;
                    }
                }
                // sort while keeping indices
                std::size_t n(0);
                std::generate(_vindices.begin(), _vindices.end(), [&] { return n++; });
                std::sort(_vindices.begin(), _vindices.end(), [&](int i1, int i2) {
                        return _vdotproducts[i1] < _vdotproducts[i2];
                    });
                dReal minPositiveDotProduct = _vdotproducts[_vindices.back()];
                if( minPositiveDotProduct <= 0 ) {
                    // Is this possible?
                    return retcheck;
                }
                int minPositiveDotProductIndex = 0;
                for( int i = 0; i < armdof; ++i ) {
                    minPositiveDotProduct = _vdotproducts[_vindices[i]];
                    _vscalingfactors[_vindices[i]] = 1.0;
                    if( minPositiveDotProduct > 0 ) {
                        minPositiveDotProductIndex = i;
                        break;
                    }
                }
                // Suppose the positive dot products are d1, d2, ..., dn (sorted in the ascending
                // order). We want to build a linear function f (for convenience) such that f(d1) =
                // 1 (no scaling for the DOF with least contribution) and f(dn) = reductionFactor
                // (large reduction factor for DOF with most contribution). Since we build a linear
                // function, we have
                //         f(i) = m(d1/di) + (rmax - m)
                // where m = (rmax - reductionFactor)/(1 - d1/dn)
                if( minPositiveDotProductIndex == (int)(_vdotproducts.size() - 1) ) {
                    // Cannot use the above formula since the denominator is zero so just setting the scaling factor for this one dof.
                    _vscalingfactors[_vindices[minPositiveDotProductIndex]] = reductionFactor;
                }
                else {
                    dReal m = (fMaxReductionFactor - reductionFactor) / (1 - minPositiveDotProduct/_vdotproducts[_vindices.back()]);
                    for( int i = minPositiveDotProductIndex; i < armdof; ++i ) {
                        int idof = _vindices[i];
                        _vscalingfactors[idof] = m*(minPositiveDotProduct/_vdotproducts[idof]) + (fMaxReductionFactor - m);
                    }
                }
                retcheck.vReductionFactors = _vscalingfactors;
#ifdef PROGRESS_DEBUG
                std::stringstream ss; ss << "env=" << probot->GetEnv()->GetId() << "; reductionFactor=" << reductionFactor << "; minPositiveDotProductIndex=" << minPositiveDotProductIndex << "; vdotproducts=[";
                FOREACHC(itval, _vdotproducts) {
                    ss << *itval << ", ";
                }
                ss << "]; vindices=[";
                FOREACHC(itval, _vindices) {
                    ss << *itval << ", ";
                }
                ss << "]; vscalingfactors=[";
                FOREACHC(itval, _vscalingfactors) {
                    ss << *itval << ", ";
                }
                ss << "];";
                RAVELOG_DEBUG(ss.str());
#endif
            }
            else if( _maxmanipspeed > 0 && maxactualmanipspeed > _maxmanipspeed ) {
                reductionFactor = min(multiplier*_maxmanipspeed/maxactualmanipspeed, maxallowedmult);
                retcheck.retcode = CFO_CheckTimeBasedConstraints;
                retcheck.fTimeBasedSurpassMult = reductionFactor;
                if( reductionFactor >= reductionFactorCutoff ) {
                    // Constraints are not severely violated. Don't bother to compute Jacobian and that stuff
                    return retcheck;
                }

                std::list< ManipConstraintInfo2 >::iterator itmanipinfo = _listCheckManips.begin();
                std::advance(itmanipinfo, velViolationIndex);
                RobotBasePtr probot = itmanipinfo->pmanip->GetRobot();
                KinBody::KinBodyStateSaver saver(probot, KinBody::Save_LinkTransformation|KinBody::Save_LinkVelocities);
                // Set the robot to the new state
                probot->SetDOFValues(vDOFValuesAtViolation, KinBody::CLA_CheckLimits, itmanipinfo->vuseddofindices);
                Transform tlink = itmanipinfo->plink->GetTransform();
                // compute jacobians, make sure to transform by the world frame
                probot->CalculateJacobian(itmanipinfo->plink->GetIndex(), tlink.trans, _vtransjacobian);
                int armdof = itmanipinfo->pmanip->GetArmDOF();
                for( int idof = 0; idof < armdof; ++idof ) {
                    Vector vtransaxis(_vtransjacobian[idof], _vtransjacobian[armdof+idof], _vtransjacobian[2*armdof+idof]);
                    _vdotproducts[idof] = vDOFVelAtViolation[idof] > 0 ? vtransaxis.dot3(vVelViolation) : -vtransaxis.dot3(vVelViolation);
                    if( RaveFabs(_vdotproducts[idof]) <= g_fEpsilonLinear ) {
                        _vdotproducts[idof] = 0;
                    }
                }
                // sort while keeping indices
                std::size_t n(0);
                std::generate(_vindices.begin(), _vindices.end(), [&] { return n++; });
                std::sort(_vindices.begin(), _vindices.end(), [&](int i1, int i2) {
                        return _vdotproducts[i1] < _vdotproducts[i2];
                    });
                dReal minPositiveDotProduct = _vdotproducts[_vindices.back()];
                if( minPositiveDotProduct <= 0 ) {
                    // Is this possible?
                    return retcheck;
                }
                int minPositiveDotProductIndex = 0;
                for( int i = 0; i < armdof; ++i ) {
                    minPositiveDotProduct = _vdotproducts[_vindices[i]];
                    _vscalingfactors[_vindices[i]] = 1.0;
                    if( minPositiveDotProduct > 0 ) {
                        minPositiveDotProductIndex = i;
                        break;
                    }
                }
                // Suppose the positive dot products are d1, d2, ..., dn (sorted in the ascending
                // order). We want to build a linear function f (for convenience) such that f(d1) =
                // 1 (no scaling for the DOF with least contribution) and f(dn) = reductionFactor
                // (large reduction factor for DOF with most contribution). Since we build a linear
                // function, we have
                //         f(i) = m(d1/di) + (1 - m)
                // where m = (1 - reductionFactor)/(1 - d1/dn)
                if( minPositiveDotProductIndex == (int)(_vdotproducts.size() - 1) ) {
                    // Cannot use the above formula since the denominator is zero so just setting the scaling factor for this one dof.
                    _vscalingfactors[_vindices[minPositiveDotProductIndex]] = reductionFactor;
                }
                else {
                    dReal m = (fMaxReductionFactor - reductionFactor) / (1 - minPositiveDotProduct/_vdotproducts[_vindices.back()]);
                    for( int i = minPositiveDotProductIndex; i < armdof; ++i ) {
                        int idof = _vindices[i];
                        _vscalingfactors[idof] = m*(minPositiveDotProduct/_vdotproducts[idof]) + (fMaxReductionFactor - m);
                    }
                }
                retcheck.vReductionFactors = _vscalingfactors;
#ifdef PROGRESS_DEBUG
                std::stringstream ss; ss << "env=" << probot->GetEnv()->GetId() << "; reductionFactor=" << reductionFactor << "; minPositiveDotProductIndex=" << minPositiveDotProductIndex << "; vdotproducts=[";
                FOREACHC(itval, _vdotproducts) {
                    ss << *itval << ", ";
                }
                ss << "]; vindices=[";
                FOREACHC(itval, _vindices) {
                    ss << *itval << ", ";
                }
                ss << "]; vscalingfactors=[";
                FOREACHC(itval, _vscalingfactors) {
                    ss << *itval << ", ";
                }
                ss << "];";
                RAVELOG_DEBUG(ss.str());
#endif
            }
            return retcheck;
        }
        else {
            if( _maxmanipspeed > 0 && maxactualmanipspeed > _maxmanipspeed ) {
                retcode = CFO_CheckTimeBasedConstraints;
                // If the actual max value is very close to the bound (i.e., almost not violating
                // the bound), the multiplier will be too large (too close to 1) to be useful.
                reductionFactor = min(multiplier*_maxmanipspeed/maxactualmanipspeed, maxallowedmult);
            }
            if( _maxmanipaccel > 0 && maxactualmanipaccel > _maxmanipaccel ) {
                retcode = CFO_CheckTimeBasedConstraints;
                // If the actual max value is very close to the bound (i.e., almost not violating
                // the bound), the multiplier will be too large (too close to 1) to be useful.
                reductionFactor = RaveSqrt(min(multiplier*_maxmanipaccel/maxactualmanipaccel, maxallowedmult));
            }
#ifdef PROGRESS_DEBUG
            std::stringstream ss; ss << std::setprecision(std::numeric_limits<dReal>::digits10+1);
            ss << "q=[";
            SerializeValues(ss, vDOFValuesAtViolation);
            ss << "]; qd=[";
            SerializeValues(ss, vDOFVelAtViolation);
            ss << "]; qdd=[";
            SerializeValues(ss, vDOFAccelAtViolation);
            ss << "];";
            RAVELOG_VERBOSE_FORMAT("env=%d, maxmanipspeed=%.15e; maxactualmanipspeed=%.15e; maxmanipaccel=%.15e; maxactualmanipaccel=%.15e; reductionFactor=%.15e; %s", _penv->GetId()%_maxmanipspeed%maxactualmanipspeed%_maxmanipaccel%maxactualmanipaccel%reductionFactor%ss.str());
#endif
            return RampOptimizerInternal::CheckReturn(retcode, reductionFactor, maxactualmanipspeed, maxactualmanipaccel);
        }
    }

private:
    EnvironmentBasePtr _penv;
    std::string _manipname;
    std::vector<KinBodyPtr> listUsedBodies;
    std::set<KinBody::LinkPtr> setCheckedManips;
    dReal _maxmanipspeed, _maxmanipaccel;

//@{ cache
    std::list< ManipConstraintInfo2 > _listCheckManips; ///< the manipulators and the points on their end efffectors to check for velocity and acceleration constraints
    std::vector<dReal> ac, qfillactive, _vfillactive; // the active DOF
    std::vector<dReal> _afill; // full robot DOF
    std::vector<std::pair<Vector,Vector> > endeffvels, endeffaccs;
    std::vector<dReal> _vtransjacobian, _vangularjacobian, _vbestvels2, _vbestaccels2;
    std::vector<dReal> _vdotproducts, _vscalingfactors, _vdofvalues, _vdofvelocities, _vdofaccelerations;
    std::vector<int> _vindices;
//@}

};

} // end namespace rplanners

#endif
