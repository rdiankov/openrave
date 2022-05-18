// -*- coding: utf-8 -*-
// Copyright (C) 2016-2019 Rosen Diankov and Puttichai Lertkultanon
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
#ifndef OPENRAVE_MANIP_CONSTRAINT3_H
#define OPENRAVE_MANIP_CONSTRAINT3_H

#include "openraveplugindefs.h"
#include "piecewisepolynomials/polynomialtrajectory.h"
#include "piecewisepolynomials/feasibilitychecker.h"

// #define PROGRESS_DEBUG

namespace rplanners {

struct ManipConstraintInfo3
{
    ManipConstraintInfo3() : fmaxdistfromcenter(0) {
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

class ManipConstraintChecker3
{
public:
    ManipConstraintChecker3(EnvironmentBasePtr penv) : _penv(penv), _envId(penv->GetId()), _maxmanipspeed(0), _maxmanipaccel(0) {
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

    const std::list<ManipConstraintInfo3>& GetCheckManips() const {
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
                        FOREACH(itgrabbed, grabbedbodies) {
                            if( pmanip->IsGrabbing(**itgrabbed) ) {
                                FOREACH(itlink, (*itgrabbed)->GetLinks()) {
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
                    _listCheckManips.push_back(ManipConstraintInfo3());
                    ManipConstraintInfo3& info = _listCheckManips.back();
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

    PiecewisePolynomialsInternal::CheckReturn CheckChunkManipConstraints(const PiecewisePolynomialsInternal::Chunk& chunkIn, IntervalType interval=IT_OpenStart)
    {
        if( _maxmanipspeed <= 0 && _maxmanipaccel <= 0 ) {
            return PiecewisePolynomialsInternal::CheckReturn(0);
        }

        BOOST_ASSERT(!(interval == IT_Open));
        dReal maxActualManipSpeed = 0, maxActualManipAccel = 0;

        dReal multiplier = 0.7; // a multiplier to the scaling factor computed from the ratio between the violating value and the bound
        // If the actual max value is very close to the bound (i.e., almost not violating the
        // bound), the multiplier will be too large (too close to 1) to be useful. So in that
        // case, use fMaxAllowedMult instead.
        dReal fMaxAllowedMult = 0.92;

        int retcode = 0; // one of CFO_X

        // fReductionFactor is a suggested multiplier for scaling down the robot joint velocity limits. It will be
        // computed from how much the tool speed/acceleration bounds are violated. Since fReductionFactor is for scaling
        // down velocity limits, when scaling down the acceleration limits, the multiplier should become
        // fReductionFactor^2.
        dReal fReductionFactor = 1.0;

        // Important note: for all PiecewisePolynomialsInternal::Coordinate, we will be using only
        // their points (i.e. not values).
        std::vector<PiecewisePolynomialsInternal::Coordinate> &vAllCriticalCoords = _vcoords1, &vCoords = _vcoords2;
        vAllCriticalCoords.resize(0);
        FOREACHC(itPolynomial, chunkIn.vpolynomials) {
            // TODO: only add a new coord to vAllCriticalCoords if it's not cloes to any existing coords.
            itPolynomial->FindAllLocalExtrema(1, vCoords); // get velocity extrema
            if( vCoords.size() > 2 ) {
                for( std::vector<PiecewisePolynomialsInternal::Coordinate>::const_iterator itCoord = vCoords.begin() + 1; itCoord + 1 != vCoords.end(); ++itCoord ) {
                    if( itCoord->point > 0 && itCoord->point < chunkIn.duration ) {
                        vAllCriticalCoords.push_back(*itCoord);
                    }
                }
            }
            itPolynomial->FindAllLocalExtrema(2, vCoords); // get acceleration extrema
            if( vCoords.size() > 2 ) {
                for( std::vector<PiecewisePolynomialsInternal::Coordinate>::const_iterator itCoord = vCoords.begin() + 1; itCoord + 1 != vCoords.end(); ++itCoord ) {
                    if( itCoord->point > 0 && itCoord->point < chunkIn.duration ) {
                        vAllCriticalCoords.push_back(*itCoord);
                    }
                }
            }
        }
        std::sort(vAllCriticalCoords.begin(), vAllCriticalCoords.end());
        if( interval != IT_OpenStart ) {
            // Reuse vCoords as a temporary holder
            if( vCoords.size() == 0 ) {
                vCoords.resize(1);
            }
            vCoords[0].point = 0;
            vAllCriticalCoords.insert(vAllCriticalCoords.begin(), vCoords[0]);
        }
        if( interval != IT_OpenEnd ) {
            // Reuse vCoords as a temporary holder
            if( vCoords.size() == 0 ) {
                vCoords.resize(1);
            }
            vCoords[0].point = chunkIn.duration;
            vAllCriticalCoords.push_back(vCoords[0]);
        }

        // xAll, vAll, aAll store values evaluated from the chunk. xActive and vActive store only
        // values of active DOFs of the the current manipulator (itManipInfo). aFilled has the size
        // of robot DOFs. Its index specified by manip's vuseddofindices are filled.
        std::vector<dReal> &xAll = _cacheXVect0, &vAll = _cacheVVect0, &aAll = _cacheAVect0;
        std::vector<dReal> &xActive = _cacheXVect1, &vActive = _cacheVVect1, &aFilled = _cacheAVect1;

        std::vector<std::pair<Vector, Vector> > &vLinkVelocities = _cacheLinkVelocities, &vLinkAccelerations = _cacheLinkAccelerations;
        Vector &eeVelLin = _cacheEEVelLin, &eeVelAng = _cacheEEVelAng, &eeAccelLin = _cacheEEAccelLin, &eeAccelAng = _cacheEEAccelAng;
        Vector &point = _cacheCheckPoint, &vPoint = _cacheManipSpeed, &aPoint = _cacheManipAccel; // (linear) velocity and acceleration of a check point
        Transform &Tm = _cacheManipTransform;

        dReal fTimeWhenInvalid = -1;

        FOREACHC(itManipInfo, _listCheckManips) {
            KinBodyPtr probot = itManipInfo->plink->GetParent();
            int eeIndex = itManipInfo->plink->GetIndex();
            KinBody::KinBodyStateSaver saver(probot, KinBody::Save_LinkTransformation|KinBody::Save_LinkVelocities);

            xActive.resize(itManipInfo->vuseddofindices.size());
            vActive.resize(itManipInfo->vuseddofindices.size());
            aFilled.resize(probot->GetDOF(), 0);
            FOREACHC(itCoord, vAllCriticalCoords) {
                // Evaluate values at this time instant
                dReal t = itCoord->point;
                chunkIn.Eval(t, xAll);
                chunkIn.Evald1(t, vAll);
                chunkIn.Evald2(t, aAll);
                for( size_t index = 0; index < itManipInfo->vuseddofindices.size(); ++index ) {
                    xActive[index] = xAll[itManipInfo->vconfigindices.at(index)];
                    vActive[index] = vAll[itManipInfo->vconfigindices[index]];
                    aFilled[itManipInfo->vuseddofindices[index]] = aAll.at(itManipInfo->vconfigindices[index]);
                }

                // Compute EE velocity and acceleration
                probot->SetDOFValues(xActive, KinBody::CLA_CheckLimits, itManipInfo->vuseddofindices);
                probot->SetDOFVelocities(vActive, KinBody::CLA_CheckLimits, itManipInfo->vuseddofindices);
                probot->GetLinkVelocities(vLinkVelocities);
                probot->GetLinkAccelerations(aFilled, vLinkAccelerations);

                eeVelLin = vLinkVelocities.at(eeIndex).first;
                eeVelAng = vLinkVelocities[eeIndex].second;
                eeAccelLin = vLinkAccelerations.at(eeIndex).first;
                eeAccelAng = vLinkAccelerations[eeIndex].second;
                Tm = itManipInfo->plink->GetTransform();

                // Compute the velocity and acceleration of each check point
                FOREACHC(itPoint, itManipInfo->checkpoints) {
                    point = Tm.rotate(*itPoint); // the check point in the global frame

                    if( _maxmanipspeed > 0 ) {
                        // Linear velocity: vtotal = v + (w x r)
                        vPoint = eeVelLin + eeVelAng.cross(point);
                        dReal fActualManipSpeed = RaveSqrt(vPoint.lengthsqr3());
                        if( fActualManipSpeed > maxActualManipSpeed ) {
                            maxActualManipSpeed = fActualManipSpeed;
                        }
                    }

                    if( _maxmanipaccel > 0 ) {
                        // Linear acceleration: atotal = a + (w x (w x r)) + (alpha x r)
                        aPoint = eeAccelLin + eeVelAng.cross(eeVelAng.cross(point)) + eeAccelAng.cross(point);
                        dReal fActualManipAccel = RaveSqrt(aPoint.lengthsqr3());
                        if( fActualManipAccel > maxActualManipAccel ) {
                            maxActualManipAccel = fActualManipAccel;
                        }
                    }
                } // end iterating through all check points

                if( _maxmanipspeed > 0 && maxActualManipSpeed > _maxmanipspeed ) {
                    fTimeWhenInvalid = t;
                }
                if( _maxmanipaccel > 0 && maxActualManipAccel > _maxmanipaccel ) {
                    fTimeWhenInvalid = t;
                }
            } // end iterating through all chunkIn's critical points
        } // end iterating through all manips

        // Check for any violation.
        if( _maxmanipspeed > 0 && maxActualManipSpeed > _maxmanipspeed ) {
            retcode = CFO_CheckTimeBasedConstraints;
            fReductionFactor = min(multiplier*_maxmanipspeed/maxActualManipSpeed, fMaxAllowedMult);
        }
        if( _maxmanipaccel > 0 && maxActualManipAccel > _maxmanipaccel ) {
            retcode = CFO_CheckTimeBasedConstraints;
            fReductionFactor = RaveSqrt(min(multiplier*_maxmanipaccel/maxActualManipAccel, fMaxAllowedMult));
        }

        if( 0 ) {
            _sslog.str(""); _sslog.clear();
            _sslog << std::setprecision(std::numeric_limits<dReal>::digits10 + 1);
            _sslog << "criticalpoints=[";
            FOREACHC(itCoord, vAllCriticalCoords) {
                _sslog << itCoord->point << ",";
            }
            _sslog << "]";
            RAVELOG_DEBUG_FORMAT("env=%d, chunk.duration=%.15e; %s; fTimeWhenInvalid=%.15e; maxActualManipSpeed=%.15e; maxActualManipAccel=%.15e", _envId%chunkIn.duration%_sslog.str()%fTimeWhenInvalid%maxActualManipSpeed%maxActualManipAccel);
        }

        return PiecewisePolynomialsInternal::CheckReturn(retcode, fReductionFactor, maxActualManipSpeed, maxActualManipAccel);
    }

private:
    EnvironmentBasePtr _penv;
    int _envId;
    std::string _manipname;
    std::vector<KinBodyPtr> listUsedBodies;
    std::set<KinBody::LinkPtr> setCheckedManips;
    dReal _maxmanipspeed, _maxmanipaccel;

    //@{ cache
    std::list<ManipConstraintInfo3> _listCheckManips; ///< the manipulators and the points on their end efffectors to check for velocity and acceleration constraints

    // TODO: rename these parameters
    std::vector<dReal> _cacheXVect0, _cacheXVect1, _cacheVVect0, _cacheVVect1, _cacheAVect0, _cacheAVect1;

    std::vector<std::pair<Vector, Vector> > _cacheLinkVelocities, _cacheLinkAccelerations;
    Vector _cacheEEVelLin, _cacheEEVelAng, _cacheEEAccelLin, _cacheEEAccelAng, _cacheCheckPoint, _cacheManipSpeed, _cacheManipAccel;
    Transform _cacheManipTransform;
    std::vector<dReal> _vtransjacobian, _vangularjacobian, _vbestvels2, _vbestaccels2;
    std::vector<dReal> _vdofvalues, _vdofvelocities, _vdofaccelerations;
    std::vector<PiecewisePolynomialsInternal::Coordinate> _vcoords1, _vcoords2;
    //@}

    std::stringstream _sslog;
};

} // end namespace rplanners

#endif
