// -*- coding: utf-8 -*-
// Copyright (c) 2011 Max Argus, Nick Hillier, Katrina Monkley, Rosen Diankov
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

// 2013 Modifications: Theodoros Stouraitis and Praveen Ramanujam
#include "bulletspace.h"

class BulletPhysicsEngine : public PhysicsEngineBase
{
    class PhysicsFilterCallback : public OpenRAVEFilterCallback
    {
public:
        PhysicsFilterCallback() : OpenRAVEFilterCallback() {
        }
        virtual bool CheckLinks(KinBody::LinkPtr plink0, KinBody::LinkPtr plink1) const
        {
		bool flag_of_collision;
                KinBodyPtr pbody0 = plink0->GetParent();
                KinBodyPtr pbody1 = plink1->GetParent();

                if( pbody0 == pbody1 ) {
              
              // Read all the Adjacent link pairs from the robot and check if the 2 links 
              // you are about to compare, are in a pair inside the Adjacent link pair list
              FOREACHC(it,pbody0->GetAdjacentLinks()){
                 
                  //  The comparision is done bite retrieval process
                  //  list of pairs  -->  [(a,b),(a',b'),(a'',b''),.....]
                  /*   And we compare,  a with link0's Index and b with link1's Index 
                                        a' with link0's Index and b' with link1's Index
                                                        ...
                      so we try to find if the link0 and link1 form a pair inside the Adjacent 
                      link pair list, in order to avoid collision ---
                  */
                 if (((int)((*it)&0xffff) == plink0->GetIndex()) && ((int)((*it)>>16) == plink1->GetIndex())){
                   flag_of_collision = false;
                   break;
                  }
                 // the same condition as above but checking the inverse of the pair (a,b)'= (b,a)
                 else if( ((int)((*it)&0xffff) == plink1->GetIndex()) && ((int)((*it)>>16) == plink0->GetIndex())){
                   flag_of_collision = false;
                   break;
                 }
                 // otherwise we return true because collision check should be done
                 else {
                   flag_of_collision = true;
                 }
                 }
               }
            else {
               // If they are from different entities then collision check has to be performed
               flag_of_collision = true;
               
             }
             return flag_of_collision;

        }
    };

    inline boost::shared_ptr<BulletPhysicsEngine> shared_physics() {
        return boost::static_pointer_cast<BulletPhysicsEngine>(shared_from_this());
    }
    inline boost::shared_ptr<BulletPhysicsEngine const> shared_physics_const() const {
        return boost::static_pointer_cast<BulletPhysicsEngine const>(shared_from_this());
    }

class PhysicsPropertiesXMLReader : public BaseXMLReader
    {
public:
        PhysicsPropertiesXMLReader(boost::shared_ptr<BulletPhysicsEngine> physics, const AttributesList& atts) : _physics(physics) {
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
            if( name == "bulletproperties" )
                return true;
	    else if( name == "solver_iterations" ) {
                // read all the float values into a vector
                _ss >> _physics->_solver_iterations;
            }
            else if( name == "margin_depth" ) {
                // read all the float values into a vector
                _ss >> _physics->_margin_depth;
            }
             else if( name == "linear_damping" ) {
                // read all the float values into a vector
                _ss >> _physics->_linear_damping;
            }
             else if( name == "rotation_damping" ) {
                // read all the float values into a vector
                _ss >> _physics->_rotation_damping;
            }
             else if( name == "global_friction" ) {
                // read all the float values into a vector
                _ss >> _physics->_global_friction;
            }
             else if( name == "global_contact_force_mixing" ) {
                // read all the float values into a vector
                _ss >> _physics->_global_contact_force_mixing;
            }
            else if( name == "global_restitution" ) {
                // read all the float values into a vector
                _ss >> _physics->_global_restitution;
                
            }
            else if( name == "erp" ) {
                // read all the float values into a vector
                _ss >> _physics->_super_damp;
                
            }
             else if( name == "erp2" ) {
                // read all the float values into a vector
                _ss >> _physics->_super_damp2;
                
            }
            else if( name == "gravity" ) {
                Vector v;
                _ss >> v.x >> v.y >> v.z;
                if( !!_ss ) {
                    _physics->SetGravity(v);
                }
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

        static const boost::array<string, 8>& GetTags() {
        static const boost::array<string, 8> tags = {{"solver_iterations","margin_depth","linear_damping","rotation_damping",
        "global_contact_force_mixing","global_friction","global_restitution","gravity" }};
            return tags;
        }

protected:
        BaseXMLReaderPtr _pcurreader;
        boost::shared_ptr<BulletPhysicsEngine> _physics;
        stringstream _ss;
    };

public:
    
    static BaseXMLReaderPtr CreateXMLReader(InterfaceBasePtr ptr, const AttributesList& atts)
    {
    	return BaseXMLReaderPtr(new PhysicsPropertiesXMLReader(boost::dynamic_pointer_cast<BulletPhysicsEngine>(ptr),atts));
    }

    BulletPhysicsEngine(EnvironmentBasePtr penv, std::istream& sinput) : PhysicsEngineBase(penv), _space(new BulletSpace(penv, GetPhysicsInfo, true))
    {
	stringstream ss;        
	__description = ":Interface Authors: Max Argus, Nick Hillier, Katrina Monkley, Rosen Diankov\n\nInterface to `Bullet Physics Engine <http://bulletphysics.org/>`_\n";
        RegisterCommand("SetStaticBodyTransform",boost::bind(&BulletPhysicsEngine::SetStaticBodyTransform,this,_1,_2),"Sets the transformation of a static body manually, not allowed to use for dynamic bodies and it should be used with caution even for static bodies because it can cause instabilities in physics engine.");
        _solver_iterations = 5;
        _margin_depth = 0.001;
        _linear_damping = 0.1;
        _rotation_damping = 0.5;
       
        _global_contact_force_mixing = 0.3;
        _global_friction = 0.5;
        _global_restitution = 0.2 ;
        
        _super_damp = 0.3; 
        _super_damp2 = 0.9;
          
        FOREACHC(it, PhysicsPropertiesXMLReader::GetTags()) {
            ss << "**" << *it << "**, ";
        }
        ss << "\n\n";
         
	
        /* relative links -->   http://bulletphysics.org/Bullet/BulletFull/btContactSolverInfo_8h_source.html
                          -->   http://bulletphysics.org/Bullet/BulletFull/btDiscreteDynamicsWorld_8cpp_source.html#l01353   
          about parameters of the contact solver....!      
        */
    }
    /* This function is registered as a command to transform the static links of robots.
       The application can be seen when one wants to use a stand alone gripper with enabled physics and wants to move the base of the gripper */
    bool SetStaticBodyTransform(ostream& sout, istream& sinput)
    // bool SetStaticBodyTransform(KinBody::LinkPtr plink, const Vector& translation, const Vector& axisofrot,const dReal& angleofrot)
    {
	KinBody::LinkPtr plink;
        Vector _rotation;
        Vector _translation; 
        dReal _angle;
        string linkname;
        string cmd;
       	while(!sinput.eof()) {
            sinput >> cmd;
            if( !sinput ) {
                break;
            }
            std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
            if( cmd == "link" ) {
                sinput >> linkname;
              
            }
            else if( cmd == "translation" ) {
                for(size_t i = 0; i < 3; i++) {
                sinput >> _translation[i];
                  if( !sinput ) {
                    RAVELOG_WARN("Translation needs to have three values\n");
                    return false;
                  }
            	}
              }
            else if( cmd == "axisofrot" ) {
                for(size_t i = 0; i < 3; i++) {
                sinput >> _rotation[i];
                if( !sinput ) {
                    RAVELOG_WARN("Axis of Rotation needs to have three values\n");
                    return false;
                }
              }
              
            }
	    else if( cmd == "angleofrot" ) {
                sinput >> _angle;
            }
          
            if( sinput.fail() || !sinput ) {
                break;
            }
        }
        vector<KinBodyPtr> vbodies;
        bool _foundlink = false;
        GetEnv()->GetBodies(vbodies);
        FOREACHC(itbody, vbodies) {
	BulletSpace::KinBodyInfoPtr pinfo = GetPhysicsInfo(*itbody);
            FOREACH(itlink, pinfo->vlinks) {
                if((*itlink)->plink->GetName()==linkname){
                       if ((*itlink)->plink->IsStatic() ){
			plink = (*itlink)->plink;
                        _foundlink = true;
                     }
		}
	        if (_foundlink){
			break;
		}
	    }
	
	}
	boost::shared_ptr<btRigidBody> rigidbody = boost::dynamic_pointer_cast<btRigidBody>(_space->GetLinkBody(plink));
        btVector3 _axis(_rotation[0], _rotation[1], _rotation[2]);
        btVector3 _Position(_translation[0], _translation[1], _translation[2]);
        _space->Synchronize(KinBodyConstPtr(plink->GetParent()));
        
        rigidbody->clearForces();
        // -- In case of pure translation of the body ---------------
    	btTransform _transform = rigidbody->getCenterOfMassTransform(); 
     	btVector3 position_what = _transform.getOrigin();
     	btVector3 pp(position_what[0] + _Position[0],position_what[1] + _Position[1],position_what[2] + _Position[2]); 
     	_transform.setOrigin(pp);
     	
        // -- In case of pure rotation of the body ---------------
         btQuaternion qua_temp; 
         btQuaternion qua = rigidbody->getOrientation();
         qua_temp.setRotation(_axis,_angle); // create the relative quaternion around which we need to rotate
         qua = qua_temp*qua;
            
          _transform.setRotation(qua);
          rigidbody->proceedToTransform(_transform); 
        
        sout << true;
	return true;
	
    }
    virtual bool InitEnvironment()
    {
         RAVELOG_VERBOSE("init bullet physics environment\n");
        _space->SetSynchronizationCallback(boost::bind(&BulletPhysicsEngine::_SyncCallback, shared_physics(),_1));

        _broadphase.reset(new btDbvtBroadphase());

        // allowes configuration of collision detection
        _collisionConfiguration.reset(new btDefaultCollisionConfiguration());

        // handels conves and concave collisions
        //_dispatcher = new btOpenraveDispatcher::btOpenraveDispatcher(_collisionConfiguration);
        _dispatcher.reset(new btCollisionDispatcher(_collisionConfiguration.get()));
        _solver.reset(new btSequentialImpulseConstraintSolver());
        
        // btContinuousDynamicsWorld gives a segfault for some reason
        _dynamicsWorld.reset(new btDiscreteDynamicsWorld(_dispatcher.get(),_broadphase.get(),_solver.get(),_collisionConfiguration.get()));
        
        // Critical point when you introduce the hand in the simulation
        // the PhysicsFilterCallback() is derived from the OpenRAVEFilterCallback which is in the file bulletspace.h
        _filterCallback.reset(new PhysicsFilterCallback());
        _dynamicsWorld->getPairCache()->setOverlapFilterCallback(_filterCallback.get());
        

        btContactSolverInfo& solverInfo = _dynamicsWorld->getSolverInfo();
        RAVELOG_DEBUG(str(boost::format("bullet dynamics: m_numIterations=%d, m_globalCfm=%f")%_solver_iterations%_global_contact_force_mixing));
        
        solverInfo.m_numIterations = _solver_iterations;
        solverInfo.m_globalCfm = _global_contact_force_mixing;
        
        solverInfo.m_erp = _super_damp; 
        solverInfo.m_erp2 = _super_damp2;
        //solverInfo. m_splitImpulsePenetrationThreshold = 0.5;
        
        solverInfo.m_solverMode |= SOLVER_SIMD | SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION |SOLVER_USE_2_FRICTION_DIRECTIONS; //SOLVER_ENABLE_FRICTION_DIRECTION_CACHING ;
        
	//solverInfo.m_solverMode |=    SOLVER_FRICTION_SEPARATE  |SOLVER_USE_2_FRICTION_DIRECTIONS;
	

	//solverInfo.m_solverMode |=  SOLVER_SIMD | SOLVER_FRICTION_SEPARATE  |SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION | SOLVER_RANDMIZE_ORDER SOLVER_USE_2_FRICTION_DIRECTIONS;
	
	// The 2 friction direction are not good at all for collision points with flat surfaces ....
	// Because the the is no rotational friction at all... 

	//solverInfo.m_restingContactRestitutionThreshold = 1e10;
	//solverInfo.m_splitImpulse = 1; 
	//solverInfo.m_splitImpulsePenetrationThreshold = 0.2;
        if(!_space->InitEnvironment(_dynamicsWorld)) {
            return false;
        }
        
       
        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies); 
        FOREACHC(itbody, vbodies) { 
            InitKinBody(*itbody);
        }
        SetGravity(_gravity);
        return true;
    }

    virtual void DestroyEnvironment()
    {
        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        FOREACHC(itbody, vbodies) {
            (*itbody)->RemoveUserData("bulletphysics");
        }
        RAVELOG_VERBOSE("destroy bullet physics environment\n");
        _space->DestroyEnvironment();
        _dynamicsWorld.reset();
        _collisionConfiguration.reset();
        _broadphase.reset();
        _solver.reset();
        _dispatcher.reset();
        _report.reset();
        _filterCallback.reset();
        _listcallbacks.clear();
    }

     virtual bool InitKinBody(KinBodyPtr pbody)
    {
        if( !_dynamicsWorld ) {
            return false;
        }
        BulletSpace::KinBodyInfoPtr pinfo = _space->InitKinBody(pbody);
        pbody->SetUserData("bulletphysics", pinfo);
       {
            
                if (pbody->IsRobot()){
                
                FOREACHC(po, pbody->GetLinks()) {
                     FOREACH(itlink,pinfo->vlinks) { 
                        /*-- The following lines are used in order to keep the robot(hand)
                          -- awake, so we can command the joints with torques anytime we 
                          -- need. 
                          <<This is a very usefull mechanism (available in all physics engines) 
                          to keep the CPU load low. It is called "deactivation" or "sleeping".
                          It make kinbodies sleep(do not move), when the are not in dynamic 
                          states(situation,instance)>>
                        */
                        (*itlink)->_rigidbody->setActivationState(DISABLE_DEACTIVATION);
                        (*itlink)->_rigidbody->forceActivationState(DISABLE_DEACTIVATION);
                        (*itlink)->_rigidbody->setFriction(0.9);
                        (*itlink)->_rigidbody->setRestitution(0.35); 
                        
                        
                   }
                 
                }
              }
              else{
                //do nothing
              }
                
              FOREACHC(po, pbody->GetLinks()) {
                  if ((*po)->IsStatic() ){
                    FOREACH(itlink,pinfo->vlinks) { 
                      (*itlink)->_rigidbody->setFriction(0.5);
                      (*itlink)->_rigidbody->setRestitution(0.8);  //0.65
                    }
                  }
                  else{
                      FOREACH(itlink,pinfo->vlinks) { 
                        (*itlink)->_rigidbody->setFriction(_global_friction);
                        (*itlink)->_rigidbody->setRestitution(_global_restitution);
                        (*itlink)->_rigidbody->setDamping(_linear_damping,_rotation_damping);
                  
                  }
             }
            }
        }
        return !!pinfo;
    }


    virtual void RemoveKinBody(KinBodyPtr pbody)
    {
        if( !!pbody ) {
            pbody->RemoveUserData("bulletphysics");
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

    virtual bool SetPhysicsOptions(std::ostream& sout, std::istream& sinput) {
        return false;
    }

    virtual bool SetLinkVelocity(KinBody::LinkPtr plink, const Vector& linearvel, const Vector& angularvel)
    {
        BulletSpace::KinBodyInfoPtr pinfo = GetPhysicsInfo(plink->GetParent());
        boost::shared_ptr<btRigidBody> rigidbody(pinfo->vlinks.at(plink->GetIndex())->_rigidbody);
        if( !rigidbody ) {
            RAVELOG_DEBUG(str(boost::format("link %s does not have rigid body")%plink->GetName()));
        }
        rigidbody->setLinearVelocity(BulletSpace::GetBtVector(linearvel));
        rigidbody->setAngularVelocity(BulletSpace::GetBtVector(angularvel));
        return false;
    }
    virtual bool SetLinkVelocities(KinBodyPtr pbody, const std::vector<std::pair<Vector,Vector> >& velocities)
    {
        BulletSpace::KinBodyInfoPtr pinfo = GetPhysicsInfo(pbody);
        FOREACH(itlink, pinfo->vlinks) {
            if( !!(*itlink)->_rigidbody ) {
                int index = (*itlink)->plink->GetIndex();
                (*itlink)->_rigidbody->setLinearVelocity(BulletSpace::GetBtVector(velocities.at(index).first));
                (*itlink)->_rigidbody->setAngularVelocity(BulletSpace::GetBtVector(velocities.at(index).second));
            }
            else {
                RAVELOG_DEBUG(str(boost::format("link %s does not have rigid body")%(*itlink)->plink->GetName()));
            }
        }
        return false;
    }

    virtual bool GetLinkVelocity(KinBody::LinkConstPtr plink, Vector& linearvel, Vector& angularvel)
    {
        _space->Synchronize(KinBodyConstPtr(plink->GetParent()));
        boost::shared_ptr<btRigidBody> rigidbody = boost::dynamic_pointer_cast<btRigidBody>(_space->GetLinkBody(plink));
        if (!!rigidbody) {
            btVector3 pf = rigidbody->getLinearVelocity();
            linearvel = Vector(pf[0],pf[1],pf[2]);
            pf = rigidbody->getAngularVelocity();
            angularvel = Vector(pf[0],pf[1],pf[2]);
        }
        else {
            linearvel = angularvel = Vector(0,0,0);
        }
        return true;
    }

    virtual bool GetLinkVelocities(KinBodyConstPtr pbody, std::vector<std::pair<Vector,Vector> >& velocities)
    {
        _space->Synchronize(pbody);
        velocities.resize(0);
        velocities.resize(pbody->GetLinks().size());

        FOREACHC(itlink, pbody->GetLinks()) {
            boost::shared_ptr<btRigidBody> rigidbody = boost::dynamic_pointer_cast<btRigidBody>(_space->GetLinkBody(*itlink));
            if(!!rigidbody) {
                btVector3 pf = rigidbody->getLinearVelocity();
                Vector angularvel(pf[0], pf[1], pf[2]);
                velocities.at((*itlink)->GetIndex()).second = angularvel;
                pf = rigidbody->getAngularVelocity();
                velocities.at((*itlink)->GetIndex()).first = Vector(pf[0], pf[1], pf[2]) - angularvel.cross((*itlink)->GetTransform()*(*itlink)->GetCOMOffset() - (*itlink)->GetTransform().trans);
            }
        }

        return true;
    }

    virtual bool SetJointVelocity(KinBody::JointPtr pjoint, const std::vector<dReal>& pJointVelocity)
    {
        boost::shared_ptr<btTypedConstraint> joint = _space->GetJoint(pjoint);
        _space->Synchronize(KinBodyConstPtr(pjoint->GetParent()));
        std::vector<btScalar> vVelocity(pJointVelocity.size());
        std::copy(pJointVelocity.begin(),pJointVelocity.end(),vVelocity.begin());
        RAVELOG_ERROR("SetJointVelocity not implemented\n");
        switch(joint->getConstraintType()) {
        case HINGE_CONSTRAINT_TYPE:
            break;
        case SLIDER_CONSTRAINT_TYPE:
            break;
        default:
            RAVELOG_ERROR(str(boost::format("SetJointVelocity joint type 0x%x not supported\n")%joint->getConstraintType()));
        }
        return true;
    }

    virtual bool GetJointVelocity(KinBody::JointConstPtr pjoint, std::vector<dReal>& pJointVelocity)
    {
        boost::shared_ptr<btTypedConstraint> joint = _space->GetJoint(pjoint);
        _space->Synchronize(KinBodyConstPtr(pjoint->GetParent()));
        pJointVelocity.resize(pjoint->GetDOF());
        switch(joint->getConstraintType()) {
        case HINGE_CONSTRAINT_TYPE:
            break;
        case SLIDER_CONSTRAINT_TYPE:
            break;
        default:
            RAVELOG_ERROR(str(boost::format("GetJointVelocity joint type 0x%x not supported\n")%joint->getConstraintType()));
        }
        return true;
    }

      virtual bool AddJointTorque(KinBody::JointPtr pjoint, const std::vector<dReal>& pTorques)
    {
       
        boost::shared_ptr<btTypedConstraint> joint = _space->GetJoint(pjoint);
        _space->Synchronize(KinBodyConstPtr(pjoint->GetParent()));
        btVector3 t;
        t.setX(pTorques.at(0));
        t.setY(pTorques.at(1));
        t.setZ(pTorques.at(2));

        
        std::vector<btScalar> vtorques(pTorques.size());
        std::copy(pTorques.begin(),pTorques.end(),vtorques.begin());
        btRigidBody& bodyA = joint->getRigidBodyA();
        btRigidBody& bodyB = joint->getRigidBodyB();
        
       switch(joint->getConstraintType()) {
        
        case D6_CONSTRAINT_TYPE:{
          
            boost::shared_ptr<btGeneric6DofConstraint> d6joint = boost::dynamic_pointer_cast<btGeneric6DofConstraint>(joint);
            btTransform transB = bodyB.getWorldTransform();
            btTransform jointTransB = d6joint->getFrameOffsetB();
            btMatrix3x3 torqueB = transB.getBasis();
            btVector3 trorque_vect = torqueB * t;
            bodyB.applyTorque(trorque_vect);
            break;
        }
          
        case HINGE_CONSTRAINT_TYPE: {
            boost::shared_ptr<btHingeConstraint> hingejoint = boost::dynamic_pointer_cast<btHingeConstraint>(joint);
            btTransform transB = bodyB.getWorldTransform();
            btTransform jointTransB = hingejoint->getBFrame();
            transB = transB * jointTransB;
            btMatrix3x3 torqueB = transB.getBasis();
            btVector3 trorque_vect = torqueB * t;
            bodyB.applyTorque(trorque_vect);
            break;
        }
        case SLIDER_CONSTRAINT_TYPE: {
            boost::shared_ptr<btSliderConstraint> slidejoint = boost::dynamic_pointer_cast<btSliderConstraint>(joint);
            btTransform frameInA = slidejoint->getFrameOffsetA();
            btTransform frameInB = slidejoint->getFrameOffsetB();
            btVector3 locA = bodyA.getCenterOfMassTransform().getOrigin();
            btVector3 locB = bodyB.getCenterOfMassTransform().getOrigin();
            btTransform transA = bodyA.getCenterOfMassTransform()*frameInA;
            btTransform transB = bodyB.getCenterOfMassTransform()*frameInB;
            btVector3 forceA = vtorques.at(0) * transA.getBasis().getColumn(0);
            btVector3 forceB = -vtorques.at(0) * transB.getBasis().getColumn(0);
            bodyA.applyForce(forceA, locA);
            bodyB.applyForce(forceB, locB);
            break;
        }
        default:
            RAVELOG_ERROR(str(boost::format("AddJointTorque joint type 0x%x not supported\n")%joint->getConstraintType()));
        }
        return true;
    }

   virtual bool SetBodyForce(KinBody::LinkPtr plink, const Vector& force, const Vector& position, bool bAdd)
    {
        boost::shared_ptr<btRigidBody> rigidbody = boost::dynamic_pointer_cast<btRigidBody>(_space->GetLinkBody(plink));
        btVector3 _Force(force[0], force[1], force[2]);
        btVector3 _Position(position[0], position[1], position[2]);
        _space->Synchronize(KinBodyConstPtr(plink->GetParent()));
        if( !bAdd ) {
            rigidbody->clearForces();
        }
        rigidbody->applyForce(_Force,_Position);
        return true;
    
    }

    virtual bool SetBodyTorque(KinBody::LinkPtr plink, const Vector& torque, bool bAdd)
    {
        btVector3 _Torque(torque[0], torque[1], torque[2]);
        boost::shared_ptr<btRigidBody> rigidbody = boost::dynamic_pointer_cast<btRigidBody>(_space->GetLinkBody(plink));
        _space->Synchronize(KinBodyConstPtr(plink->GetParent()));
        if( !bAdd ) {
            rigidbody->clearForces();
        }
        rigidbody->applyTorque(_Torque);
        return true;
    }

    virtual void SetGravity(const Vector& gravity)
    {
        _gravity = gravity;
        //behaviour similar to ODE
        if( !!_space && _space->IsInitialized() ) {
             _dynamicsWorld->setGravity(btVector3(_gravity.x,_gravity.y,_gravity.z));
        }
    }

    virtual const Vector& GetGravity()
    {
        return _gravity;
    }

    virtual void SimulateStep(dReal fTimeElapsed)
    {
        _space->Synchronize();
        int maxSubSteps = 0;  // --> reduced sub steps
        //_dynamicsWorld->applyGravity();
        _dynamicsWorld->stepSimulation(0.005,maxSubSteps); //-> reduced elapse time

        vector<KinBodyPtr> vbodies;
        GetEnv()->GetBodies(vbodies);
        FOREACHC(itbody, vbodies) {
            BulletSpace::KinBodyInfoPtr pinfo = GetPhysicsInfo(*itbody);
            FOREACH(itlink, pinfo->vlinks) {
                Transform t = BulletSpace::GetTransform((*itlink)->_rigidbody->getCenterOfMassTransform());
                (*itlink)->plink->SetTransform(t*(*itlink)->tlocal.inverse());
            }
            pinfo->nLastStamp = (*itbody)->GetUpdateStamp();
        }
        //_dynamicsWorld->clearForces();
    }

//xml parameters has to be public

    Vector _gravity;
    btScalar _global_friction;
    int _solver_iterations;
    btScalar _margin_depth;
    btScalar _linear_damping, _rotation_damping;
    btScalar _global_contact_force_mixing;
    btScalar _global_restitution;
    btScalar _super_damp;
    btScalar _super_damp2;

private:
    static BulletSpace::KinBodyInfoPtr GetPhysicsInfo(KinBodyConstPtr pbody)
    {
        return boost::dynamic_pointer_cast<BulletSpace::KinBodyInfo>(pbody->GetUserData("bulletphysics"));
    }

    void _SyncCallback(BulletSpace::KinBodyInfoConstPtr pinfo)
    {
        // reset dynamics
        FOREACH(itlink, pinfo->vlinks) {
            if( !!(*itlink)->_rigidbody ) {
                (*itlink)->_rigidbody->setLinearVelocity(btVector3(0,0,0));
                (*itlink)->_rigidbody->setAngularVelocity(btVector3(0,0,0));
            }
        }
    }

    int _options;
    boost::shared_ptr<BulletSpace> _space;
    boost::shared_ptr<btDiscreteDynamicsWorld> _dynamicsWorld;
    boost::shared_ptr<btDefaultCollisionConfiguration> _collisionConfiguration;
    boost::shared_ptr<btBroadphaseInterface> _broadphase;
    boost::shared_ptr<btCollisionDispatcher> _dispatcher;
    boost::shared_ptr<btConstraintSolver> _solver;
    boost::shared_ptr<btOverlapFilterCallback> _filterCallback;

    std::list<EnvironmentBase::CollisionCallbackFn> _listcallbacks;
    CollisionReportPtr _report;
};


PhysicsEngineBasePtr CreateBulletPhysicsEngine(EnvironmentBasePtr penv, std::istream& sinput)
{
    return PhysicsEngineBasePtr(new BulletPhysicsEngine(penv,sinput));
}
