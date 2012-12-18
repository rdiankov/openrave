// -*- coding: utf-8 -*-
// Copyright (C) 2006-2010 Rosen Diankov (rdiankov@cs.cmu.edu)
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
#ifndef RAVE_PHYSICSENGINE_ODE
#define RAVE_PHYSICSENGINE_ODE

#include "odespace.h"

class ODEPhysicsEngine : public OpenRAVE::PhysicsEngineBase
{
    // ODE joint helper fns
    static dReal DummyGetParam(dJointID id, int param)
    {
        return 0;
    }

    static void DummyAddForce(dJointID id, const dReal* vals)
    {
        RAVELOG_ERROR(str(boost::format("failed to add force to dummy %d")%dJointGetType(id)));
    }

    static void dJointAddHingeTorque_(dJointID id, const dReal* vals) {
        dJointAddHingeTorque(id, vals[0]);
    }
    static void dJointAddSliderForce_(dJointID id, const dReal* vals) {
        dJointAddSliderForce(id, vals[0]);
    }
    static void dJointAddUniversalTorques_(dJointID id, const dReal* vals) {
        dJointAddUniversalTorques(id, vals[0], vals[1]);
    }
    static void dJointAddHinge2Torques_(dJointID id, const dReal* vals) {
        dJointAddHinge2Torques(id, vals[0], vals[1]);
    }

    static dReal dJointGetHinge2Angle2_(dJointID id) {
        return 0;
    }

    static dReal JointGetBallVelocityX(dJointID) {
        return 0;
    }
    static dReal JointGetBallVelocityY(dJointID) {
        return 0;
    }
    static dReal JointGetBallVelocityZ(dJointID) {
        return 0;
    }

    inline boost::shared_ptr<ODEPhysicsEngine> shared_physics() {
        return boost::dynamic_pointer_cast<ODEPhysicsEngine>(shared_from_this());
    }
    inline boost::shared_ptr<ODEPhysicsEngine const> shared_physics_const() const {
        return boost::dynamic_pointer_cast<ODEPhysicsEngine const>(shared_from_this());
    }

    class PhysicsPropertiesXMLReader : public BaseXMLReader
    {
public:
        PhysicsPropertiesXMLReader(boost::shared_ptr<ODEPhysicsEngine> physics, const AttributesList& atts) : _physics(physics) {
        }

        virtual ProcessElement startElement(const std::string& name, const AttributesList& atts) {
            if( !!_pcurreader ) {
                if( _pcurreader->startElement(name,atts) == PE_Support ) {
                    return PE_Support;
                }
                return PE_Ignore;
            }

            if( find(GetTags().begin(),GetTags().end(),name) == GetTags().end() ) {
                return PE_Pass;
            }
            _ss.str("");
            return PE_Support;
        }

        virtual bool endElement(const std::string& name)
        {
            if( name == "odeproperties" )
                return true;
            else if( name == "friction" ) {
                // read all the float values into a vector
                _ss >> _physics->_globalfriction;
            }
            else if( name == "selfcollision" ) {
                bool bSelfCollision = false;
                _ss >> bSelfCollision;
                if( !!_ss ) {
                    _physics->SetPhysicsOptions(_physics->GetPhysicsOptions()|OpenRAVE::PEO_SelfCollisions);
                }
            }
            else if( name == "gravity" ) {
                Vector v;
                _ss >> v.x >> v.y >> v.z;
                if( !!_ss ) {
                    _physics->SetGravity(v);
                }
            }
            else if( name == "elastic_reduction_parameter" || name == "erp" ) {
                _ss >> _physics->_globalerp;
            }
            else if( name == "constraint_force_mixing" || name == "cfm" ) {
                _ss >> _physics->_globalcfm;
            }
            else if( name == "contact" ) {         // check out http://www.ode.org/ode-latest-userguide.html#sec_7_3_7

            }
            else if( name == "dcontactapprox") {
                // check out http://www.ode.org/ode-latest-userguide.html#sec_7_3_7
                int temp=0;
                _ss >> temp;
                //Set or unset dContactApprox flag using internal enum
                if (temp != 0) {
                    _physics->_surface_mode |= dContactApprox1; //Set
                }
                else {
                    _physics->_surface_mode &= (!dContactApprox1);        //Unset
                }
                RAVELOG_DEBUG("surface mode flags: %x\n",_physics->_surface_mode);
            }
            else {
                RAVELOG_ERROR("unknown field %s\n", name.c_str());
            }

            if( !_ss ) {
                RAVELOG_WARN(str(boost::format("error parsing %s\n")%name));
            }

            return false;
        }

        virtual void characters(const std::string& ch)
        {
            if( !!_pcurreader ) {
                _pcurreader->characters(ch);
            }
            else {
                _ss.clear();
                _ss << ch;
            }
        }

        static const boost::array<string, 9>& GetTags() {
            static const boost::array<string, 9> tags = {{"friction","selfcollision", "gravity", "contact", "erp", "cfm", "elastic_reduction_parameter", "constraint_force_mixing", "dcontactapprox" }};
            return tags;
        }

protected:
        BaseXMLReaderPtr _pcurreader;
        boost::shared_ptr<ODEPhysicsEngine> _physics;
        stringstream _ss;
    };

public:
    static BaseXMLReaderPtr CreateXMLReader(InterfaceBasePtr ptr, const AttributesList& atts)
    {
        return BaseXMLReaderPtr(new PhysicsPropertiesXMLReader(boost::dynamic_pointer_cast<ODEPhysicsEngine>(ptr),atts));
    }

    ODEPhysicsEngine(OpenRAVE::EnvironmentBasePtr penv) : OpenRAVE::PhysicsEngineBase(penv), _odespace(new ODESpace(penv, "odephysics", true)) {
        stringstream ss;
        ss << ":Interface Author: Rosen Diankov\n\nODE physics engine\n\n\
It is possible to set ODE physics engine and its properties inside the <environment> XML tags by typing:\n\n\
.. code-block:: xml\n\n\
  <physicsengine type=\"ode\">\n\
    <odeproperties>\n\
      <friction>0.5</friction>\n\
      <gravity>0 0 -9.8</gravity>\n\
      <selfcollision>1</selfcollision>\n\
    </odeproperties>\n\
  </physicsengine>\n\n\
The possible properties that can be set are: ";
        FOREACHC(it, PhysicsPropertiesXMLReader::GetTags()) {
            ss << "**" << *it << "**, ";
        }
        ss << "\n\n";
        __description = ss.str();
        _globalfriction = 0.4;
        _globalerp = 0.01;
        _globalcfm = 1e-5;
        //Default to openrave 0.6.6 behavior, but this really should default to
        //enable the friction pyramid model.
        _surface_mode = 0;
        _options = OpenRAVE::PEO_SelfCollisions;

        memset(_jointadd, 0, sizeof(_jointadd));
        _jointadd[dJointTypeBall] = DummyAddForce;
        _jointadd[dJointTypeHinge] = dJointAddHingeTorque_;
        _jointadd[dJointTypeSlider] = dJointAddSliderForce_;
        _jointadd[dJointTypeUniversal] = dJointAddUniversalTorques_;
        _jointadd[dJointTypeHinge2] = dJointAddHinge2Torques_;

        _jointgetvel[dJointTypeBall].push_back(JointGetBallVelocityX);
        _jointgetvel[dJointTypeBall].push_back(JointGetBallVelocityY);
        _jointgetvel[dJointTypeBall].push_back(JointGetBallVelocityZ);
        _jointgetvel[dJointTypeHinge].push_back(dJointGetHingeAngleRate);
        _jointgetvel[dJointTypeSlider].push_back(dJointGetSliderPositionRate);
        _jointgetvel[dJointTypeUniversal].push_back(dJointGetUniversalAngle1Rate);
        _jointgetvel[dJointTypeUniversal].push_back(dJointGetUniversalAngle2Rate);
        _jointgetvel[dJointTypeHinge2].push_back(dJointGetHinge2Angle1Rate);
        _jointgetvel[dJointTypeHinge2].push_back(dJointGetHinge2Angle2Rate);
    }

    virtual bool InitEnvironment()
    {
        _report.reset(new CollisionReport());

        _odespace->SetSynchornizationCallback(boost::bind(&ODEPhysicsEngine::_SyncCallback, shared_physics(),_1));
        if( !_odespace->InitEnvironment() ) {
            return false;
        }
        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        FOREACHC(itbody, vbodies) {
            InitKinBody(*itbody);
        }
        SetGravity(_gravity);
        RAVELOG_DEBUG(str(boost::format("ode params: erp=%e (%e), cfm=%e (%e)")%_globalerp%dWorldGetERP(_odespace->GetWorld())%_globalcfm%dWorldGetCFM(_odespace->GetWorld())));
        dWorldSetERP(_odespace->GetWorld(),_globalerp);
        dWorldSetCFM(_odespace->GetWorld(),_globalcfm);
        return true;
    }

    virtual void DestroyEnvironment()
    {
        _listcallbacks.clear();
        _report.reset();
        _odespace->DestroyEnvironment();
        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        FOREACHC(itbody, vbodies) {
            (*itbody)->RemoveUserData("odephysics");
        }
    }

    virtual bool InitKinBody(KinBodyPtr pbody)
    {
        ODESpace::KinBodyInfoPtr pinfo = boost::dynamic_pointer_cast<ODESpace::KinBodyInfo>(pbody->GetUserData("odephysics"));
        // need the pbody check since kinbodies can be cloned and could have the wrong pointer
        if( !pinfo || pinfo->GetBody() != pbody ) {
            pinfo = _odespace->InitKinBody(pbody);
            pbody->SetUserData("odephysics", pinfo);
        }
        return !!pinfo;
    }

    virtual void RemoveKinBody(KinBodyPtr pbody)
    {
        if( !!pbody ) {
            pbody->RemoveUserData("odephysics");
        }
    }

    virtual bool SetPhysicsOptions(int physicsoptions)
    {
        _options = physicsoptions;
        return true;
    }

    virtual int GetPhysicsOptions() const
    {
        return _options;
    }

    virtual bool SetPhysicsOptions(std::ostream& sout, std::istream& sinput)
    {
        return false;
    }

    virtual void Clone(InterfaceBaseConstPtr preference, int cloningoptions)
    {
        PhysicsEngineBase::Clone(preference,cloningoptions);
        boost::shared_ptr<ODEPhysicsEngine const> r = boost::dynamic_pointer_cast<ODEPhysicsEngine const>(preference);

        SetGravity(r->_gravity);
        _options = r->_options;
        _globalfriction = r->_globalfriction;
        _globalcfm = r->_globalcfm;
        _globalerp = r->_globalerp;
        _surface_mode = r->_surface_mode;
        if( !!_odespace && _odespace->IsInitialized() ) {
            dWorldSetERP(_odespace->GetWorld(),_globalerp);
            dWorldSetCFM(_odespace->GetWorld(),_globalcfm);
        }
    }

    virtual bool SetLinkVelocity(KinBody::LinkPtr plink, const Vector& _linearvel, const Vector& angularvel)
    {
        _odespace->Synchronize(plink->GetParent());
        dBodyID body = _odespace->GetLinkBody(plink);
        if( !body ) {
            return false;
        }
        Vector linearvel = _linearvel + angularvel.cross(plink->GetTransform()*plink->GetCOMOffset() - plink->GetTransform().trans);
        dBodySetLinearVel(body, linearvel.x,linearvel.y,linearvel.z);
        dBodySetAngularVel(body, angularvel.x, angularvel.y, angularvel.z);
        return true;
    }

    virtual bool SetLinkVelocities(KinBodyPtr pbody, const std::vector<std::pair<Vector,Vector> >& velocities)
    {
        bool bsuccess = true;
        _odespace->Synchronize(pbody);
        FOREACHC(itlink, pbody->GetLinks()) {
            dBodyID body = _odespace->GetLinkBody(*itlink);
            if( body ) {
                Vector angularvel = velocities.at((*itlink)->GetIndex()).second;
                dBodySetAngularVel(body, angularvel.x,angularvel.y,angularvel.z);
                Vector linearvel = velocities.at((*itlink)->GetIndex()).first;
                linearvel += angularvel.cross((*itlink)->GetTransform()*(*itlink)->GetCOMOffset() - (*itlink)->GetTransform().trans);
                dBodySetLinearVel(body, linearvel.x,linearvel.y,linearvel.z);
            }
            else {
                bsuccess = false;
            }
        }
        return bsuccess;
    }

    virtual bool GetLinkVelocity(KinBody::LinkConstPtr plink, Vector& linearvel, Vector& angularvel)
    {
        _odespace->Synchronize(plink->GetParent());
        dBodyID body = _odespace->GetLinkBody(plink);
        if( body ) {
            const dReal* p = dBodyGetAngularVel(body);
            angularvel = Vector(p[0], p[1], p[2]);
            p = dBodyGetLinearVel(body);
            linearvel = Vector(p[0], p[1], p[2]);
            linearvel -= angularvel.cross(plink->GetTransform()*plink->GetCOMOffset() - plink->GetTransform().trans);
        }
        else {
            angularvel = linearvel = Vector(0,0,0);
        }
        return true;
    }

    virtual bool GetLinkVelocities(KinBodyConstPtr pbody, std::vector<std::pair<Vector,Vector> >& velocities)
    {
        _odespace->Synchronize(pbody);
        velocities.resize(0);
        velocities.resize(pbody->GetLinks().size());
        FOREACHC(itlink, pbody->GetLinks()) {
            dBodyID body = _odespace->GetLinkBody(*itlink);
            if( body ) {
                const dReal* pf = dBodyGetAngularVel(body);
                Vector angularvel(pf[0], pf[1], pf[2]);
                velocities.at((*itlink)->GetIndex()).second = angularvel;
                pf = dBodyGetLinearVel(body);
                velocities.at((*itlink)->GetIndex()).first = Vector(pf[0], pf[1], pf[2]) - angularvel.cross((*itlink)->GetTransform()*(*itlink)->GetCOMOffset() - (*itlink)->GetTransform().trans);
            }
        }
        return true;
    }

    virtual bool GetLinkForceTorque(KinBody::LinkConstPtr plink, Vector& force, Vector& torque) {
        _odespace->Synchronize(plink->GetParent());
        dBodyID body = _odespace->GetLinkBody(plink);
        force = Vector(0,0,0);
        torque = Vector(0,0,0);
        if( body ) {
            //Loop over all joints in the parent body
            FOREACHC(itjoint,plink->GetParent()->GetJoints()) {
                //if this joint's parent or child body is the current body
                if( (*itjoint)->GetHierarchyParentLink() == plink || (*itjoint)->GetHierarchyChildLink() == plink ) {
                    dJointID joint = _odespace->GetJoint(*itjoint);
                    dJointFeedback* feedback = dJointGetFeedback( joint );
                    BOOST_ASSERT(feedback != NULL);
                    if( dJointGetBody( joint,0 ) == body ) {
                        force[0] += feedback->f1[0]; force[1] += feedback->f1[1]; force[2] += feedback->f1[2];
                        torque[0] += feedback->t1[0]; torque[1] += feedback->t1[1]; torque[2] += feedback->t1[2];
                    }
                    else if( dJointGetBody( joint,1 ) == body ) {
                        force[0] += feedback->f2[0]; force[1] += feedback->f2[1]; force[2] += feedback->f2[2];
                        torque[0] += feedback->t2[0]; torque[1] += feedback->t2[1]; torque[2] += feedback->t2[2];
                    }
                }
            }
            FOREACHC(itjoint,plink->GetParent()->GetPassiveJoints()) {
                if( (*itjoint)->GetHierarchyParentLink() == plink || (*itjoint)->GetHierarchyChildLink() == plink ) {
                    dJointID joint = _odespace->GetJoint(*itjoint);
                    dJointFeedback* feedback = dJointGetFeedback( joint );
                    BOOST_ASSERT(feedback != NULL);
                    if( dJointGetBody( joint,0 ) == body ) {
                        force[0] += feedback->f1[0]; force[1] += feedback->f1[1]; force[2] += feedback->f1[2];
                        torque[0] += feedback->t1[0]; torque[1] += feedback->t1[1]; torque[2] += feedback->t1[2];
                    }
                    else if( dJointGetBody( joint,1 ) == body ) {
                        force[0] += feedback->f2[0]; force[1] += feedback->f2[1]; force[2] += feedback->f2[2];
                        torque[0] += feedback->t2[0]; torque[1] += feedback->t2[1]; torque[2] += feedback->t2[2];
                    }
                }
            }
        }
        return true;
    }

    virtual bool GetJointVelocity(KinBody::JointConstPtr pjoint, std::vector<OpenRAVE::dReal>& vJointVelocity)
    {
        dJointID joint = _odespace->GetJoint(pjoint);
        BOOST_ASSERT( joint != NULL );
        _odespace->Synchronize(pjoint->GetParent());
        vector<JointGetFn>::iterator itfn;
        vJointVelocity.resize(pjoint->GetDOF());
        vector<OpenRAVE::dReal>::iterator itvel = vJointVelocity.begin();
        FORIT(itfn, _jointgetvel[dJointGetType(joint)]) {
            *itvel++ = (*itfn)(joint);
        }
        return true;
    }

    virtual bool SetBodyForce(KinBody::LinkPtr plink, const Vector& force, const Vector& position, bool bAdd)
    {
        dBodyID body = _odespace->GetLinkBody(plink);
        if( body == NULL ) {
            return false;
        }
        _odespace->Synchronize(plink->GetParent());
        if( !bAdd ) {
            dBodySetForce(body, 0, 0, 0);
        }
        dBodyAddForceAtPos(body, force.x, force.y, force.z, position.x, position.y, position.z);
        return true;
    }

    virtual bool SetBodyTorque(KinBody::LinkPtr plink, const Vector& torque, bool bAdd)
    {
        dBodyID body = _odespace->GetLinkBody(plink);
        if( body == NULL ) {
            return false;
        }
        _odespace->Synchronize(plink->GetParent());

        if( !bAdd ) {
            dBodySetTorque(body, torque.x, torque.y, torque.z);
        }
        else {
            dBodyAddTorque(body, torque.x, torque.y, torque.z);
        }
        return true;
    }

    virtual bool AddJointTorque(KinBody::JointPtr pjoint, const std::vector<OpenRAVE::dReal>& pTorques)
    {
        dJointID joint = _odespace->GetJoint(pjoint);
        BOOST_ASSERT( joint != NULL );
        _odespace->Synchronize(pjoint->GetParent());
        std::vector<dReal> vtorques(pTorques.size());
        std::copy(pTorques.begin(),pTorques.end(),vtorques.begin());
        _jointadd[dJointGetType(joint)](joint, &vtorques[0]);
        return true;
    }

    virtual void SetGravity(const Vector& gravity)
    {
        _gravity = gravity;
        if( !!_odespace && _odespace->IsInitialized() ) {
            dWorldSetGravity(_odespace->GetWorld(),_gravity.x, _gravity.y, _gravity.z);
        }
    }

    virtual Vector GetGravity()
    {
        return _gravity;
    }

    virtual void SimulateStep(OpenRAVE::dReal fTimeElapsed)
    {
        _odespace->Synchronize();

        bool bHasCallbacks = GetEnv()->HasRegisteredCollisionCallbacks();
        if( bHasCallbacks ) {
            GetEnv()->GetRegisteredCollisionCallbacks(_listcallbacks);
        }
        else {
            _listcallbacks.clear();
        }

        dSpaceCollide (_odespace->GetSpace(),this,nearCallback);

        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);

        if( _options & OpenRAVE::PEO_SelfCollisions ) {
            FOREACHC(itbody, vbodies) {
                if( (*itbody)->GetLinks().size() > 1 ) {
                    // more than one link, check collision
                    dSpaceCollide(_odespace->GetBodySpace(*itbody), this, nearCallback);
                }
            }
        }

        dWorldQuickStep(_odespace->GetWorld(), fTimeElapsed);
        dJointGroupEmpty (_odespace->GetContactGroup());

        // synchronize all the objects from the ODE world to the OpenRAVE world
        Transform t;
        FOREACHC(itbody, vbodies) {
            ODESpace::KinBodyInfoPtr pinfo = _odespace->GetInfo(*itbody);
            BOOST_ASSERT( pinfo->vlinks.size() == (*itbody)->GetLinks().size());
            vector<Transform> vtrans(pinfo->vlinks.size());
            for(size_t i = 0; i < pinfo->vlinks.size(); ++i) {
                const dReal* prot = dBodyGetQuaternion(pinfo->vlinks[i]->body);
                Vector vrot(prot[0],prot[1],prot[2],prot[3]);
                if( vrot.lengthsqr4() == 0 ) {
                    RAVELOG_ERROR(str(boost::format("odephysics in body %s is returning invalid rotation!")%(*itbody)->GetName()));
                    continue;
                }
                const dReal* ptrans = dBodyGetPosition(pinfo->vlinks[i]->body);
                vtrans.at(i) = Transform(vrot,Vector(ptrans[0],ptrans[1],ptrans[2]));
            }
            (*itbody)->SetLinkTransformations(vtrans,pinfo->_vdofbranches);
            pinfo->nLastStamp = (*itbody)->GetUpdateStamp();
        }

        _listcallbacks.clear();
    }


private:
    static void nearCallback(void *data, dGeomID o1, dGeomID o2)
    {
        ((ODEPhysicsEngine*)data)->_nearCallback(o1,o2);
    }

    void _nearCallback(dGeomID o1, dGeomID o2)
    {
        if( !dGeomIsEnabled(o1) || !dGeomIsEnabled(o2) ) {
            return;
        }
        if (dGeomIsSpace(o1) || dGeomIsSpace(o2)) {
            // colliding a space with something
            dSpaceCollide2(o1,o2,this, nearCallback);
            return;
        }

        dBodyID b1,b2;
        b1 = dGeomGetBody(o1);
        b2 = dGeomGetBody(o2);
        if (!(_options & OpenRAVE::PEO_SelfCollisions) && b1 && b2 && dAreConnected (b1,b2)) {
            return;
        }

        // ignore static, static collisions
        if( (( b1 == NULL) || !dBodyIsEnabled(b1)) && (( b2 == NULL) || !dBodyIsEnabled(b2)) ) {
            return;
        }

        KinBody::LinkPtr pkb1,pkb2;
        if(!!b1 && dBodyGetData(b1)) {
            pkb1 = ((ODESpace::KinBodyInfo::LINK*)dBodyGetData(b1))->GetLink();
        }
        if(!!b2 && dBodyGetData(b1)) {
            pkb2 = ((ODESpace::KinBodyInfo::LINK*)dBodyGetData(b2))->GetLink();
        }

        if( !!pkb1 && !pkb1->IsEnabled() ) {
            return;
        }
        if( !!pkb2 && !pkb2->IsEnabled() ) {
            return;
        }

        if( pkb1->GetParent() == pkb2->GetParent() ) {
            // check if links are adjacent
            int minindex = min(pkb1->GetIndex(), pkb2->GetIndex());
            int maxindex = max(pkb1->GetIndex(), pkb2->GetIndex());

            if( pkb1->GetParent()->GetAdjacentLinks().find(minindex|(maxindex<<16)) != pkb1->GetParent()->GetAdjacentLinks().end() )
                return;
        }

        const int N = 16;
        dContact contact[N];
        int n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
        if( n <= 0 ) {
            return;
        }

        if( _listcallbacks.size() > 0 ) {
            // fill the collision report
            _report->Reset(OpenRAVE::CO_Contacts);
            _report->numCols = N;
            _report->plink1 = pkb1;
            _report->plink2 = pkb2;

            dGeomID checkgeom1 = dGeomGetClass(o1) == dGeomTransformClass ? dGeomTransformGetGeom(o1) : o1;
            for(int i = 0; i < n; ++i) {
                _report->contacts.push_back(CollisionReport::CONTACT(contact[i].geom.pos, checkgeom1 != contact[i].geom.g1 ? -Vector(contact[i].geom.normal) : Vector(contact[i].geom.normal), contact[i].geom.depth));
            }

            FOREACH(itfn, _listcallbacks) {
                OpenRAVE::CollisionAction action = (*itfn)(_report,true);
                if( action != OpenRAVE::CA_DefaultAction ) {
                    return;
                }
            }
        }

        // process collisions
        for (int i=0; i<n; i++) {
            contact[i].surface.mode = _surface_mode;
            contact[i].surface.mu = (dReal)_globalfriction;
            contact[i].surface.mu2 = (dReal)_globalfriction;
            //        contact[i].surface.slip1 = 0.7;
            //        contact[i].surface.slip2 = 0.7;
            //        contact[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactSlip1 | dContactSlip2;
            //        contact[i].surface.mu = 50.0; // was: dInfinity
            //        contact[i].surface.soft_erp = 0.96;
            //        contact[i].surface.soft_cfm = 0.04;
            dJointID c = dJointCreateContact (_odespace->GetWorld(),_odespace->GetContactGroup(),contact+i);

            // make sure that static objects are not enabled by adding a joint attaching them
            if( b1 ) {
                b1 = dBodyIsEnabled(b1) ? b1 : 0;
            }
            if( b2 ) {
                b2 = dBodyIsEnabled(b2) ? b2 : 0;
            }
            dJointAttach (c, b1, b2);

            //wprintf(L"intersection %s %s\n", ((KinBody::Link*)dBodyGetData(b1))->GetName(), ((KinBody::Link*)dBodyGetData(b2))->GetName());

            //        contact[i].surface.slip1 = 0.7;
            //        contact[i].surface.slip2 = 0.7;
            //        contact[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactSlip1 | dContactSlip2;
            //        contact[i].surface.mu = 50.0; // was: dInfinity
            //        contact[i].surface.soft_erp = 0.96;
            //        contact[i].surface.soft_cfm = 0.04;
            //        dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
            //        dJointAttach (c,
            //            dGeomGetBody(contact[i].geom.g1),
            //            dGeomGetBody(contact[i].geom.g2));
        }
        //
        //        dJointID c = dJointCreateContact (GetEnv()->world,GetEnv()->contactgroup,&contact);
        //        dJointAttach (c,b1,b2);
    }

    void _SyncCallback(ODESpace::KinBodyInfoConstPtr pinfo)
    {
        // things very difficult when dynamics are not reset
//        FOREACHC(itlink, pinfo->vlinks) {
//            if( (*itlink)->body != NULL ) {
//                dBodySetAngularVel((*itlink)->body, 0, 0, 0);
//                dBodySetLinearVel((*itlink)->body, 0, 0, 0);
//                // not sure if should reset forces since sync call can happen much later
//                //dBodySetForce((*itlink)->body, 0, 0, 0);
//                //dBodySetTorque((*itlink)->body, 0, 0, 0);
//            }
//        }
    }

    boost::shared_ptr<ODESpace> _odespace;
    Vector _gravity;
    int _options;
    dReal _globalfriction, _globalcfm, _globalerp;
    // _surface_mode stores global surface settings used in the
    // dSurfaceParameters structure in ODE.
    // See http://ode-wiki.org/wiki/index.php?title=Manual:_Joint_Types_and_Functions
    int _surface_mode;

    typedef void (*JointSetFn)(dJointID, int param, dReal val);
    typedef dReal (*JointGetFn)(dJointID);
    typedef void (*JointAddForceFn)(dJointID, const dReal* vals);

    JointSetFn _jointset[12];
    JointAddForceFn _jointadd[12];
    vector<JointGetFn> _jointgetvel[12];
    std::list<EnvironmentBase::CollisionCallbackFn> _listcallbacks;
    CollisionReportPtr _report;
};

#endif
