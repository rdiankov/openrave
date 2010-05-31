// Copyright (C) 2006-2009 Rosen Diankov (rdiankov@cs.cmu.edu)
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
/// functions that allow plugins to program for the RAVE simulator
#ifndef OPENRAVE_COLLADA_WRITER_H
#define OPENRAVE_COLLADA_WRITER_H

#include "ravep.h"

using namespace OpenRAVE;
using namespace std;

#include <dae.h>
#include <dae/daeErrorHandler.h>
#include <dom/domCOLLADA.h>
#include <dae/domAny.h>
#include <dom/domConstants.h>
#include <dom/domTriangles.h>
#include <dae/daeDocument.h>
#include <dom/domTypes.h>
#include <dom/domElements.h>

class ColladaWriter : public daeErrorHandler
{
public:
  struct SCENE
  {
    domVisual_sceneRef vscene;
    domKinematics_sceneRef kscene;
    domPhysics_sceneRef pscene;
    domInstance_with_extraRef viscene;
    domInstance_kinematics_sceneRef kiscene;
    domInstance_with_extraRef piscene;
  };

  struct LINKOUTPUT
  {
    list<int> listusedlinks;
    daeElementRef plink;
    domNodeRef pnode;
  };

  /// Constructor
  /// penv    OpenRAVE environment to write
    ColladaWriter(EnvironmentBaseConstPtr penv) : _dom(NULL), _penv(penv)
  {
    RAVELOG_INFOA("ColladaWriter(const EnvironmentBasePtr penv)\n");

    daeErrorHandler::setErrorHandler(this);

    RAVELOG_VERBOSEA("init COLLADA writer version: %s, namespace: %s\n", COLLADA_VERSION, COLLADA_NAMESPACE);
    _collada.reset(new DAE);
    _collada->setIOPlugin( NULL );
    _collada->setDatabase( NULL );

    const char* documentName = "openrave_snapshot";

    daeDocument *doc = NULL;
    daeInt error = _collada->getDatabase()->insertDocument(documentName, &doc ); // also creates a collada root

    if ( error != DAE_OK || doc == NULL )
    {
      RAVELOG_ERRORA("Failed to create new document\n");
      throw;
    }

    _dom = daeSafeCast<domCOLLADA>(doc->getDomRoot());
    _dom->setAttribute("xmlns:math","http://www.w3.org/1998/Math/MathML");

    //create the required asset tag
    domAssetRef asset = daeSafeCast<domAsset>( _dom->createAndPlace( COLLADA_ELEMENT_ASSET ) );
    {
      domAsset::domCreatedRef created = daeSafeCast<domAsset::domCreated>( asset->createAndPlace( COLLADA_ELEMENT_CREATED ) );
      created->setValue("2009-04-06T17:01:00.891550");
      domAsset::domModifiedRef modified = daeSafeCast<domAsset::domModified>( asset->createAndPlace( COLLADA_ELEMENT_MODIFIED ) );
      modified->setValue("2009-04-06T17:01:00.891550");

      domAsset::domContributorRef contrib = daeSafeCast<domAsset::domContributor>( asset->createAndPlace( COLLADA_TYPE_CONTRIBUTOR ) );
      domAsset::domContributor::domAuthoring_toolRef authoringtool = daeSafeCast<domAsset::domContributor::domAuthoring_tool>( contrib->createAndPlace( COLLADA_ELEMENT_AUTHORING_TOOL ) );
      authoringtool->setValue("OpenRAVE Collada Writer");

      domAsset::domUnitRef units = daeSafeCast<domAsset::domUnit>( asset->createAndPlace( COLLADA_ELEMENT_UNIT ) );
      units->setMeter(1);
      units->setName("meter");

      domAsset::domUp_axisRef zup = daeSafeCast<domAsset::domUp_axis>( asset->createAndPlace( COLLADA_ELEMENT_UP_AXIS ) );
      zup->setValue(UP_AXIS_Z_UP);
    }

    _scene = _dom->getScene();
    if( !_scene )
    {
      _scene = daeSafeCast<domCOLLADA::domScene>( _dom->createAndPlace( COLLADA_ELEMENT_SCENE ) );
    }

    _visualScenesLib = daeSafeCast<domLibrary_visual_scenes>(_dom->createAndPlace (COLLADA_ELEMENT_LIBRARY_VISUAL_SCENES));
    _visualScenesLib->setId("vscenes");
    _geometriesLib = daeSafeCast<domLibrary_geometries>(_dom->createAndPlace(COLLADA_ELEMENT_LIBRARY_GEOMETRIES));
    _geometriesLib->setId("geometries");
    _effectsLib = daeSafeCast<domLibrary_effects>(_dom->createAndPlace(COLLADA_ELEMENT_LIBRARY_EFFECTS));
    _effectsLib->setId("effects");
    _materialsLib = daeSafeCast<domLibrary_materials>(_dom->createAndPlace(COLLADA_ELEMENT_LIBRARY_MATERIALS));
    _materialsLib->setId("materials");
    _kinematicsModelsLib = daeSafeCast<domLibrary_kinematics_models>(_dom->createAndPlace (COLLADA_ELEMENT_LIBRARY_KINEMATICS_MODELS));
    _kinematicsModelsLib->setId("kmodels");
    _articulatedSystemsLib = daeSafeCast<domLibrary_articulated_systems>(_dom->createAndPlace (COLLADA_ELEMENT_LIBRARY_ARTICULATED_SYSTEMS));
    _articulatedSystemsLib->setId("asystems");
    _kinematicsScenesLib = daeSafeCast<domLibrary_kinematics_scenes>(_dom->createAndPlace (COLLADA_ELEMENT_LIBRARY_KINEMATICS_SCENES));
    _kinematicsScenesLib->setId("kscenes");
    _physicsScenesLib = daeSafeCast<domLibrary_physics_scenes>(_dom->createAndPlace (COLLADA_ELEMENT_LIBRARY_PHYSICS_SCENES));
    _physicsScenesLib->setId("pscenes");

    //  Sensors and other specific OpenRAVE features
    domExtraRef _extraLib   =   daeSafeCast<domExtra>(_dom->createAndPlace(COLLADA_ELEMENT_EXTRA));
    _extraLib->setId("libraries");
    domTechniqueRef tech    =   daeSafeCast<domTechnique>(_extraLib->createAndPlace(COLLADA_ELEMENT_TECHNIQUE));
    tech->setProfile("OpenRAVE");
//        _sensorsLib =   daeSafeCast<domLibrary_sensors>(tech->createAndPlace(COLLADA_ELEMENT_LIBRARY_SENSORS));
//        _sensorsLib->setId("libsensors");
  }

  /// Destructor
  virtual ~ColladaWriter()
  {
    _collada.reset();
    DAE::cleanup();
  }

  /// Write scene
  virtual SCENE CreateScene()
  {
    RAVELOG_INFOA("virtual SCENE CreateScene()\n");

    SCENE s;

    //  Create visual scene
    s.vscene = daeSafeCast<domVisual_scene>(_visualScenesLib->createAndPlace (COLLADA_ELEMENT_VISUAL_SCENE));
    s.vscene->setId("vscene");
    s.vscene->setName("OpenRAVE Visual Scene");

    //  Create kinematics scene
    s.kscene = daeSafeCast<domKinematics_scene>(_kinematicsScenesLib->createAndPlace (COLLADA_ELEMENT_KINEMATICS_SCENE));
    s.kscene->setId("kscene");
    s.kscene->setName("OpenRAVE Kinematics Scene");

    //  Create physic scene
    s.pscene = daeSafeCast<domPhysics_scene>(_physicsScenesLib->createAndPlace (COLLADA_ELEMENT_PHYSICS_SCENE));
    s.pscene->setId("pscene");
    s.pscene->setName("OpenRAVE Physics Scene");

    //  Create instance visual scene
    s.viscene = daeSafeCast<domInstance_with_extra>( _scene->createAndPlace( COLLADA_ELEMENT_INSTANCE_VISUAL_SCENE ) );

    s.viscene->setUrl( (string("#") + string(s.vscene->getID())).c_str() );

    //  Create instance kinematics scene
    s.kiscene = daeSafeCast<domInstance_kinematics_scene>( _scene->createAndPlace( COLLADA_ELEMENT_INSTANCE_KINEMATICS_SCENE ) );
    s.kiscene->setUrl( (string("#") + string(s.kscene->getID())).c_str() );

    //  Create instance physics scene
    s.piscene = daeSafeCast<domInstance_with_extra>( _scene->createAndPlace( COLLADA_ELEMENT_INSTANCE_PHYSICS_SCENE ) );
    s.piscene->setUrl( (string("#") + string(s.pscene->getID())).c_str() );

    return s;
  }

  /// Write down environment
  virtual bool Write(EnvironmentBasePtr penv)
  {
    EnvironmentMutex::scoped_lock lockenv(penv->GetMutex());

    RAVELOG_INFOA("virtual bool Write(EnvironmentBasePtr penv)\n");

    //  Create scene
    SCENE scene = CreateScene();

    //  Create physics technique common
    domPhysics_scene::domTechnique_commonRef common = daeSafeCast<domPhysics_scene::domTechnique_common>(scene.pscene->createAndPlace (COLLADA_ELEMENT_TECHNIQUE_COMMON));

    //  Create gravity
    domTargetable_float3Ref g = daeSafeCast<domTargetable_float3>(common->createAndPlace (COLLADA_ELEMENT_GRAVITY));
    Vector vgravity = penv->GetPhysicsEngine()->GetGravity();
    g->getValue().set3 (vgravity.x, vgravity.y, vgravity.z);

    //      //  For each robot adds it to the scene
    //      FOREACHC(itrobot, penv->GetRobots()) {
    //          if( !Write(*itrobot, scene) ) {
    //              RAVELOG_ERRORA("failed to write body %s\n", (*itrobot)->GetName().c_str());
    //              continue;
    //          }
    //      }

    //  For each body adds it to the scene
    vector<KinBodyPtr> vbodies;
    penv->GetBodies(vbodies);
    FOREACHC(itbody, vbodies) {
      if( !Write(*itbody, scene) ) {
        RAVELOG_ERRORA("failed to write body %s\n", (*itbody)->GetName().c_str());
        continue;
      }
    }

    return true;
  }

  /// Gets robot pointer from kinbody name
  /// name    Name of the kinbody that is a Robot
  RobotBasePtr getRobot(KinBodyPtr pbody)
  {
    //  For each robot adds it to the scene
    vector<RobotBasePtr> vrobots;
    pbody->GetEnv()->GetRobots(vrobots);
    FOREACHC(itrobot, vrobots) {
      if( pbody->GetName() == (*itrobot)->GetName() )
      {
        RAVELOG_INFOA("Find Robot %s\n", (*itrobot)->GetName().c_str());
        return *itrobot;
      }
    }

    return RobotBasePtr();
  }

  /// Write down kinematic body
  virtual bool Write(KinBodyPtr pbody)
  {
    EnvironmentMutex::scoped_lock lockenv(_penv->GetMutex());
    RAVELOG_INFOA("virtual bool Write(KinBodyPtr pbody)\n");

    //  Create scene
    SCENE scene = CreateScene();

    return  Write(pbody, scene);
  }

  /// Write down kinematic body in a given scene
  virtual bool Write(KinBodyPtr pbody, SCENE& scene)
  {
    EnvironmentMutex::scoped_lock lockenv(_penv->GetMutex());
    RAVELOG_INFOA("virtual bool Write(KinBodyPtr pbody, SCENE& scene)\n");

    //  Debug
    RAVELOG_WARNA("Kinbody Name: %s\n",pbody->GetName().c_str());

    KinBody::KinBodyStateSaver saver(pbody);
    vector<dReal> vjointvalues, vzero(pbody->GetDOF());
    pbody->GetJointValues(vjointvalues);

    if( vzero.size() > 0 )
    {
      pbody->SetJointValues(vzero);
    }

    //  Create root node for the visual scene
    domNodeRef pnoderoot = daeSafeCast<domNode>(scene.vscene->createAndPlace(COLLADA_ELEMENT_NODE));
    string bodyid = string("v")+toString(pbody->GetNetworkId());

    //  Debug
    RAVELOG_VERBOSEA("bodyid: %s\n",bodyid.c_str());

    pnoderoot->setId(bodyid.c_str());
    pnoderoot->setName(pbody->GetName().c_str());

    //  Create kinematics model
    domKinematics_modelRef kmodel = daeSafeCast<domKinematics_model>(_kinematicsModelsLib->createAndPlace(COLLADA_ELEMENT_KINEMATICS_MODEL));
    kmodel->setId((string("k")+toString(pbody->GetNetworkId())).c_str());

    //  Debug
    RAVELOG_VERBOSEA("Kinematic model Id: %s\n",kmodel->getId());

    kmodel->setName(pbody->GetName().c_str());

    string strModelId = kmodel->getID();
    string strInstModelSid = string("inst_") + string(kmodel->getID());

    vector<KinBody::JointPtr > vjoints;
    vjoints.reserve(pbody->GetJoints().size()+pbody->_vecPassiveJoints.size());

    FOREACHC(itj, pbody->GetJoints() )
    {
      KinBody::JointPtr pj(new KinBody::Joint(pbody));
      *pj = **itj;
      vjoints.push_back(pj);
    }

    int dof = pbody->GetDOF();

    FOREACHC(itj, pbody->_vecPassiveJoints)
    {
      KinBody::JointPtr pj(new KinBody::Joint(pbody));
      *pj = **itj;
      vjoints.push_back(pj);
      vjoints.back()->jointindex += pbody->GetJoints().size();
      vjoints.back()->dofindex = dof;
      dof += vjoints.back()->GetDOF();
    }

    //  Robot definition
    if (pbody->IsRobot())
    {
      RAVELOG_INFOA("%s is a Robot\n",pbody->GetName().c_str());

      //  Get robot pointer.
      RobotBasePtr probot   =   getRobot(pbody);

      //  Debug
      RAVELOG_INFOA("Got the robot %s\n",probot->GetName().c_str());

      domArticulated_systemRef    askinematics    =   daeSafeCast<domArticulated_system>(_articulatedSystemsLib->createAndPlace(COLLADA_ELEMENT_ARTICULATED_SYSTEM));
      askinematics->setId((string("askinematics")+toString(pbody->GetNetworkId())).c_str());
      domKinematicsRef    kinematics = daeSafeCast<domKinematics>(askinematics->createAndPlace(COLLADA_ELEMENT_KINEMATICS));

      string  strASKinematicsId   =   askinematics->getID();

      domArticulated_systemRef    asmotion    =   daeSafeCast<domArticulated_system>(_articulatedSystemsLib->createAndPlace(COLLADA_ELEMENT_ARTICULATED_SYSTEM));
      asmotion->setId((string("asmotion")+toString(pbody->GetNetworkId())).c_str());
      domMotionRef motion =   daeSafeCast<domMotion>(asmotion->createAndPlace(COLLADA_ELEMENT_MOTION));

      //  Articulated system motion identifier
      string  strASMotionId   =   asmotion->getID();

      //  Create Instance Kinematics Model
      domInstance_articulated_systemRef ias = daeSafeCast<domInstance_articulated_system>(scene.kscene->createAndPlace(COLLADA_ELEMENT_INSTANCE_ARTICULATED_SYSTEM));

      //  Set bindings of instance articulated system in Kinematics Scene
      SetBinding(ias,string(scene.kscene->getID()),strInstModelSid,strASMotionId,vjoints);

      //  Create 'instance articulated system' in 'motion' section
      domInstance_articulated_systemRef ias_motion    =   daeSafeCast<domInstance_articulated_system>(
          motion->createAndPlace(COLLADA_ELEMENT_INSTANCE_ARTICULATED_SYSTEM));

      //  Set bindings of 'instnance articulated system' in 'motion' section of articulated systems
      SetBinding(ias_motion,strASMotionId,strInstModelSid,strASKinematicsId,vjoints);

      //  Create 'instance kinematics model'
      domInstance_kinematics_modelRef ikm = daeSafeCast<domInstance_kinematics_model>(kinematics->createAndPlace(COLLADA_ELEMENT_INSTANCE_KINEMATICS_MODEL));
      SetSidRef(ikm,strASKinematicsId,strInstModelSid,strModelId,vjoints,vjointvalues);

      //  Create 'technique_common' in 'kinematics' section
      domKinematics_techniqueRef  kt  =   daeSafeCast<domKinematics_technique>(kinematics->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));

      //  Create 'technique_common' in 'motion' section
      domMotion_techniqueRef  mt  =   daeSafeCast<domMotion_technique>(motion->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));

      if (probot->GetManipulators().size() >0)
      {
        RobotBase::ManipulatorPtr manipulator = probot->GetManipulators().back();
        string  sframe_origin   =   strModelId + string("/") + manipulator->GetBase()->GetName();
        string  sframe_tip      =   strModelId + string("/") + manipulator->GetEndEffector()->GetName();
        daeSafeCast<domKinematics_frame>(kt->createAndPlace(COLLADA_ELEMENT_FRAME_ORIGIN))->setLink(sframe_origin.c_str());
        daeSafeCast<domKinematics_frame>(kt->createAndPlace(COLLADA_ELEMENT_FRAME_TIP))->setLink(sframe_tip.c_str());
      }

      //  Number of the current axis info
      size_t  sidcount    =   0;

      //  Axis info parameters of 'kinematics' and 'motion' sections
      FOREACH(itjoint, vjoints)
      {
        string jointname = string("joint") + toString((*itjoint)->GetJointIndex());
        KinBody::LinkPtr pchildlink = GetChildLink(*itjoint, vjoints);

        if( pchildlink == NULL )
        {
          continue;
        }

        for(int idof = 0; idof < (*itjoint)->GetDOF(); ++idof)
        {
          string  axisname                        =   string("axis") + toString(idof);
          string  axis_kinematics_tag =   strModelId + string("/") + jointname + string("/") + axisname;
          string  sidtag                          =   string("a") + toString(sidcount++);
          string  axis_motion_tag         =   strASKinematicsId + string("/") + sidtag;

          //  Motion axis info
          domKinematics_axis_infoRef kai;
          kai =   daeSafeCast<domKinematics_axis_info>(kt->createAndPlace(COLLADA_ELEMENT_AXIS_INFO));
          kai->setAxis(axis_kinematics_tag.c_str());
          kai->setSid(sidtag.c_str());

          domCommon_bool_or_paramRef  active  =   daeSafeCast<domCommon_bool_or_param>(kai->createAndPlace(COLLADA_ELEMENT_ACTIVE));
          daeSafeCast<domCommon_bool_or_param::domBool>(active->createAndPlace(COLLADA_ELEMENT_BOOL))->setValue((*itjoint)->GetParent()->IsEnabled());

          RAVELOG_INFOA("Motion axis info definition\n");

          //  Kinematics axis info
          domMotion_axis_infoRef mai;
          mai =   daeSafeCast<domMotion_axis_info>(mt->createAndPlace(COLLADA_ELEMENT_AXIS_INFO));
          mai->setAxis(axis_motion_tag.c_str());

          RAVELOG_INFOA("Motion axis info parameters\n");

          domCommon_float_or_paramRef speed   =   daeSafeCast<domCommon_float_or_param>(mai->createAndPlace(COLLADA_ELEMENT_SPEED));
          daeSafeCast<domCommon_float_or_param::domFloat>(speed->createAndPlace(COLLADA_ELEMENT_FLOAT))->setValue((*itjoint)->fMaxVel);

          domCommon_float_or_paramRef accel   =   daeSafeCast<domCommon_float_or_param>(mai->createAndPlace(COLLADA_ELEMENT_ACCELERATION));
          daeSafeCast<domCommon_float_or_param::domFloat>(accel->createAndPlace(COLLADA_ELEMENT_FLOAT))->setValue((*itjoint)->fMaxAccel);
        }
      }

      //  TODO : Instances of Sensors. Sensors attached to the robot
//            if (probot->GetSensors().size() > 0)
//            {
//                domExtraRef extra   =   daeSafeCast<domExtra>(askinematics->createAndPlace(COLLADA_ELEMENT_EXTRA));
//                extra->setType("sensors");
//                domTechniqueRef tech    =   daeSafeCast<domTechnique>(extra->createAndPlace(COLLADA_ELEMENT_TECHNIQUE));
//                tech->setProfile("OpenRAVE");
//
//                    for (size_t i = 0; i < probot->GetSensors().size();i++)
//                    {
//                        string  strsensor   =   string("sensor")+toString(i)+string("_")+probot->GetName();
//                        string  strurl      =   string("#") + strsensor;
//                        RobotBase::AttachedSensorPtr  asensor = probot->GetSensors().at(i);
//
//                        //  Instance of sensor into 'articulated_system'
//                        domInstance_sensorRef   isensor =   daeSafeCast<domInstance_sensor>(tech->createAndPlace(COLLADA_ELEMENT_INSTANCE_SENSOR));
//                        isensor->setId(asensor->GetName().c_str());
//                        isensor->setLink(asensor->GetAttachingLink()->GetName().c_str());
//                        isensor->setUrl(strurl.c_str());
//
//                        //  Sensor definition into 'library_sensors'
//                        domSensorRef    sensor  =   daeSafeCast<domSensor>(_sensorsLib->createAndPlace(COLLADA_ELEMENT_SENSOR));
//                        sensor->setType(asensor->GetSensor()->GetXMLId().c_str());
//                        sensor->setId(strsensor.c_str());
//                        sensor->setName(strsensor.c_str());
//
//                        //  Debug
//                        RAVELOG_VERBOSEA("Plugin Name: %s\n",asensor->GetSensor()->GetXMLId().c_str());
//                    }
//            }
        }
    else
    {
      //  Create instance kinematics model
      domInstance_kinematics_modelRef ikm =
          daeSafeCast<domInstance_kinematics_model>(scene.kscene->createAndPlace(COLLADA_ELEMENT_INSTANCE_KINEMATICS_MODEL));
      SetSidRef(ikm,string(scene.kscene->getID()).c_str(),strInstModelSid,strModelId,vjoints,vjointvalues);
    }// End Create Instance Kinematics Model

    //  Binding in Scene 'instance kinematics scene'
    domBind_kinematics_modelRef pmodelbind = daeSafeCast<domBind_kinematics_model>(scene.kiscene->createAndPlace(COLLADA_ELEMENT_BIND_KINEMATICS_MODEL));
    pmodelbind->setNode((bodyid+string(".node0")).c_str());
    daeSafeCast<domCommon_param>(pmodelbind->createAndPlace(COLLADA_ELEMENT_PARAM))->
        setValue(string(string(scene.kscene->getID()) + string(".") + strInstModelSid).c_str());

    //  Joint parameters
    FOREACH(itjoint, vjoints)
    {
      string jointname = string("joint") + toString((*itjoint)->GetJointIndex());
      KinBody::LinkPtr pchildlink = GetChildLink(*itjoint, vjoints);

      if( pchildlink == NULL )
      {
        continue;
      }

      for(int idof = 0; idof < (*itjoint)->GetDOF(); ++idof)
      {
        string axisname = string("axis") + toString(idof);
        string bindname = string(scene.kscene->getID()) + string(".") + strInstModelSid + string(".") + jointname + string("_") + axisname;
        string uriname = string(scene.kscene->getID()) + string(".") + strInstModelSid + string(".") + jointname + string("_") + axisname;

        // binding
        domBind_joint_axisRef pjointbind = daeSafeCast<domBind_joint_axis>(scene.kiscene->createAndPlace(COLLADA_ELEMENT_BIND_JOINT_AXIS));
        pjointbind->setTarget((bodyid+string(".node")+toString(pchildlink->GetIndex())+string("/node_joint")+toString((*itjoint)->GetJointIndex())+string("_axis")+toString(idof)).c_str());
        domCommon_sidref_or_paramRef paxisbind = daeSafeCast<domCommon_sidref_or_param>(pjointbind->createAndPlace(COLLADA_ELEMENT_AXIS));
        daeSafeCast<domCommon_param>(paxisbind->createAndPlace(COLLADA_TYPE_PARAM))->setValue(bindname.c_str());
        domCommon_float_or_paramRef pvaluebind = daeSafeCast<domCommon_float_or_param>(pjointbind->createAndPlace(COLLADA_ELEMENT_VALUE));
        daeSafeCast<domCommon_param>(pvaluebind->createAndPlace(COLLADA_TYPE_PARAM))->setValue((bindname+string("_value")).c_str());
      }
    }

    //      //  Create instance kinematics model
    //      domInstance_kinematics_modelRef kimodel = daeSafeCast<domInstance_kinematics_model>(scene.kscene->createAndPlace(COLLADA_ELEMENT_INSTANCE_KINEMATICS_MODEL));
    //      {
    //          // set the instance kinematics model and default initial joint values, also create the bindings with scene
    //          kimodel->setSid(strInstModelSid.c_str());
    //          kimodel->setUrl((string("#")+strModelId).c_str());
    //          string sidinstmodelbase = string(scene.kscene->getID()) + string(".") + strInstModelSid;
    //          string uriinstmodelbase = string(scene.kscene->getID()) + string("/") + strInstModelSid;
    //          domKinematics_newparamRef pmodelparam = daeSafeCast<domKinematics_newparam>(kimodel->createAndPlace(COLLADA_ELEMENT_NEWPARAM));
    //          pmodelparam->setSid(sidinstmodelbase.c_str());
    //          domKinematics_newparam::domSIDREFRef sidref = daeSafeCast<domKinematics_newparam::domSIDREF>(pmodelparam->createAndPlace(COLLADA_ELEMENT_SIDREF));
    //          sidref->setValue(uriinstmodelbase.c_str());
    //
    //          domBind_kinematics_modelRef pmodelbind = daeSafeCast<domBind_kinematics_model>(scene.kiscene->createAndPlace(COLLADA_ELEMENT_BIND_KINEMATICS_MODEL));
    //          pmodelbind->setNode((bodyid+string(".node0")).c_str());
    //          daeSafeCast<domCommon_param>(pmodelbind->createAndPlace(COLLADA_ELEMENT_PARAM))->setValue(sidinstmodelbase.c_str());
    //
    //          //  Joint parameters
    //          FOREACH(itjoint, vjoints)
    //          {
    //              string jointname = string("joint") + toString((*itjoint)->GetJointIndex());
    //              KinBody::LinkPtr pchildlink = GetChildLink(*itjoint, vjoints);
    //
    //              if( pchildlink == NULL )
    //              {
    //                  continue;
    //              }
    //
    //              // Axis kinematics model
    //              for(int idof = 0; idof < (*itjoint)->GetDOF(); ++idof)
    //              {
    //                  string axisname = string("axis") + toString(idof);
    //                  string sidname = sidinstmodelbase + string(".") + jointname + string(".") + axisname;
    //                  string uriname = uriinstmodelbase + string("/") + jointname + string("/") + axisname;
    //
    //                  // binding
    //                  domKinematics_newparamRef paxisparam = daeSafeCast<domKinematics_newparam>(kimodel->createAndPlace(COLLADA_ELEMENT_NEWPARAM));
    //                  paxisparam->setSid(sidname.c_str());
    //                  domKinematics_newparam::domSIDREFRef sidref = daeSafeCast<domKinematics_newparam::domSIDREF>(paxisparam->createAndPlace(COLLADA_ELEMENT_SIDREF));
    //                  sidref->setValue(uriname.c_str());
    //                  // value
    //                  domKinematics_newparamRef pvalueparam = daeSafeCast<domKinematics_newparam>(kimodel->createAndPlace(COLLADA_ELEMENT_NEWPARAM));
    //                  pvalueparam->setSid((sidname+string("_value")).c_str());
    //                  domKinematics_newparam::domFloatRef valueref = daeSafeCast<domKinematics_newparam::domFloat>(pvalueparam->createAndPlace(COLLADA_ELEMENT_FLOAT));
    //                  valueref->getValue() = vjointvalues[(*itjoint)->GetDOFIndex()+idof];
    //
    //                  // binding
    //                  domBind_joint_axisRef pjointbind = daeSafeCast<domBind_joint_axis>(scene.kiscene->createAndPlace(COLLADA_ELEMENT_BIND_JOINT_AXIS));
    //                  pjointbind->setTarget((bodyid+string(".node")+toString(pchildlink->GetIndex())+string("/node_joint")+toString((*itjoint)->GetJointIndex())+string("_axis")+toString(idof)).c_str());
    //                  domCommon_sidref_or_paramRef paxisbind = daeSafeCast<domCommon_sidref_or_param>(pjointbind->createAndPlace(COLLADA_ELEMENT_AXIS));
    //                  daeSafeCast<domCommon_param>(paxisbind->createAndPlace(COLLADA_TYPE_PARAM))->setValue(sidname.c_str());
    //                  domCommon_float_or_paramRef pvaluebind = daeSafeCast<domCommon_float_or_param>(pjointbind->createAndPlace(COLLADA_ELEMENT_VALUE));
    //                  daeSafeCast<domCommon_param>(pvaluebind->createAndPlace(COLLADA_TYPE_PARAM))->setValue((sidname+string("_value")).c_str());
    //              }
    //          }
    //      }

    //  Create kinematics model technique common
    domKinematics_model_techniqueRef ktec = daeSafeCast<domKinematics_model_technique>(kmodel->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));

    //  Declare all the joints
    vector<domJointRef> vdomjoints(vjoints.size());

    FOREACHC(itjoint, vjoints)
    {
      domJointRef pjoint = daeSafeCast<domJoint>(ktec->createAndPlace(COLLADA_ELEMENT_JOINT));
      pjoint->setSid( (string("joint")+toString((*itjoint)->GetJointIndex())).c_str() );
      pjoint->setName((*itjoint)->GetName().c_str());

      vector<dReal> lmin, lmax;
      (*itjoint)->GetLimits(lmin, lmax);

      vector<domAxis_constraintRef> vaxes((*itjoint)->GetDOF());

      for(int ia = 0; ia < (*itjoint)->GetDOF(); ++ia)
      {
        switch((*itjoint)->GetType())
        {
        case KinBody::Joint::JointRevolute:
          vaxes[ia] = daeSafeCast<domAxis_constraint>(pjoint->createAndPlace(COLLADA_ELEMENT_REVOLUTE));
          lmin[ia]*=180.0f/PI;
          lmax[ia]*=180.0f/PI;
          break;
        case KinBody::Joint::JointPrismatic:
          vaxes[ia] = daeSafeCast<domAxis_constraint>(pjoint->createAndPlace(COLLADA_ELEMENT_PRISMATIC));
          break;
        case KinBody::Joint::JointUniversal:
        case KinBody::Joint::JointHinge2:
        default:
          RAVELOG_WARNA("unsupported joint type specified %d\n", (*itjoint)->GetType());
          break;
        }

        if( !vaxes[ia] )
        {
          continue;
        }

        vaxes[ia]->setSid((string("axis")+toString(ia)).c_str());
        domAxisRef paxis = daeSafeCast<domAxis>(vaxes[ia]->createAndPlace(COLLADA_ELEMENT_AXIS));
        paxis->getValue().setCount(3);
                paxis->getValue()[0] = -(*itjoint)->vAxes[ia].x;
                paxis->getValue()[1] = -(*itjoint)->vAxes[ia].y;
                paxis->getValue()[2] = -(*itjoint)->vAxes[ia].z;
        domJoint_limitsRef plimits = daeSafeCast<domJoint_limits>(vaxes[ia]->createAndPlace(COLLADA_TYPE_LIMITS));
        daeSafeCast<domMinmax>(plimits->createAndPlace(COLLADA_ELEMENT_MIN))->getValue() = lmin[ia];
        daeSafeCast<domMinmax>(plimits->createAndPlace(COLLADA_ELEMENT_MAX))->getValue() = lmax[ia];
      }

      vdomjoints[(*itjoint)->GetJointIndex()] = pjoint;
    }

    list<int> listunusedlinks;

    for(int ilink = 0; ilink < (int)pbody->GetLinks().size(); ++ilink)
    {
      listunusedlinks.push_back(ilink);
    }

    while(listunusedlinks.size()>0)
    {
      LINKOUTPUT childinfo = WriteLink(pbody->GetLinks()[listunusedlinks.front()], ktec, pnoderoot, strModelId, vjoints);
      Transform t = pbody->GetLinks()[listunusedlinks.front()]->GetTransform();
      AddTransformation(childinfo.plink, t, false);
      AddTransformation(childinfo.pnode, t, false);

      FOREACHC(itused, childinfo.listusedlinks)
      listunusedlinks.erase(find(listunusedlinks.begin(),listunusedlinks.end(),*itused));
    }

    // process all mimic joints
    FOREACH(itjoint, vjoints)
    {
      if( (*itjoint)->GetMimicJointIndex() < 0 )
      {
        continue;
      }

      domFormulaRef pf = daeSafeCast<domFormula>(ktec->createAndPlace(COLLADA_ELEMENT_FORMULA));
      pf->setSid((string("joint")+toString((*itjoint)->GetJointIndex())+string(".formula")).c_str());
      domCommon_float_or_paramRef ptarget = daeSafeCast<domCommon_float_or_param>(pf->createAndPlace(COLLADA_ELEMENT_TARGET));
      daeSafeCast<domCommon_param>(ptarget->createAndPlace(COLLADA_TYPE_PARAM))->setValue((strModelId+string("/joint")+toString((*itjoint)->GetJointIndex())).c_str());

      domFormula_techniqueRef pftec = daeSafeCast<domFormula_technique>(pf->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));
      // create a const0*joint+const1 formula
      // <apply> <plus/> <apply> <times/> <cn>a</cn> x </apply> <cn>b</cn> </apply>
      daeElementRef pmath_math = pftec->createAndPlace("math");
      daeElementRef pmath_apply = pmath_math->createAndPlace("apply");
      {
        daeElementRef pmath_plus = pmath_apply->createAndPlace("plus");
        daeElementRef pmath_apply1 = pmath_apply->createAndPlace("apply");
        {
          daeElementRef pmath_times = pmath_apply1->createAndPlace("times");
          daeElementRef pmath_const0 = pmath_apply1->createAndPlace("cn");
          pmath_const0->setCharData(toString((*itjoint)->GetMimicCoeffs()[0]));
          daeElementRef pmath_symb = pmath_apply1->createAndPlace("csymbol");
          pmath_symb->setAttribute("encoding","COLLADA");
          pmath_symb->setCharData(strModelId+string("/joint")+toString((*itjoint)->GetMimicJointIndex()));
        }
        daeElementRef pmath_const1 = pmath_apply->createAndPlace("cn");
        pmath_const1->setCharData(toString((*itjoint)->GetMimicCoeffs()[1]));
      }
    }

    return true;
  }// End Write kinbody in a given scene  //

  /// Write down Robot
  /// probot  Robot to write
  virtual bool Write(RobotBasePtr probot)
  {
    EnvironmentMutex::scoped_lock lockenv(_penv->GetMutex());
    RAVELOG_INFOA("virtual bool Write(RobotBasePtr probot)\n");

    //  Create scene
    SCENE scene = CreateScene();

    return  Write(probot, scene);
  }

  /// TODO : Write down Robot in a given scene
  /// probot  Robot to write
  /// scene       Where to write Robot
  virtual bool Write(RobotBasePtr probot, SCENE& scene)
  {
    EnvironmentMutex::scoped_lock lockenv(_penv->GetMutex());
        RAVELOG_INFOA("virtual bool Write(RobotBasePtr probot, SCENE& scene)\n");

    KinBody::KinBodyStateSaver saver(probot);
    vector<dReal> vjointvalues, vzero(probot->GetDOF());
    probot->GetJointValues(vjointvalues);

    if( vzero.size() > 0 )
    {
      probot->SetJointValues(vzero);
    }

    //  Create root node for the visual scene
    domNodeRef pnoderoot = daeSafeCast<domNode>(scene.vscene->createAndPlace(COLLADA_ELEMENT_NODE));
    string bodyid = string("v")+toString(probot->GetNetworkId());

    //  Debug
        RAVELOG_VERBOSEA("bodyid: %s\n",bodyid.c_str());

    pnoderoot->setId(bodyid.c_str());
    pnoderoot->setName(probot->GetName().c_str());

    //  Create kinematics model
    domKinematics_modelRef kmodel = daeSafeCast<domKinematics_model>(_kinematicsModelsLib->createAndPlace(COLLADA_ELEMENT_KINEMATICS_MODEL));
    kmodel->setId((string("k")+toString(probot->GetNetworkId())).c_str());

    string  strModelId = kmodel->getID();
    string  strInstModelSid = string("inst_") + string(kmodel->getID());

    domArticulated_systemRef    askinematics    =   daeSafeCast<domArticulated_system>(_articulatedSystemsLib->createAndPlace(COLLADA_ELEMENT_ARTICULATED_SYSTEM));
    askinematics->setId((string("askinematics")+toString(probot->GetNetworkId())).c_str());
    domKinematicsRef    kinematics = daeSafeCast<domKinematics>(askinematics->createAndPlace(COLLADA_ELEMENT_KINEMATICS));

    string  strASKinematicsId   =   askinematics->getID();

    domArticulated_systemRef    asmotion    =   daeSafeCast<domArticulated_system>(_articulatedSystemsLib->createAndPlace(COLLADA_ELEMENT_ARTICULATED_SYSTEM));
    asmotion->setId((string("asmotion")+toString(probot->GetNetworkId())).c_str());
    domMotionRef motion =   daeSafeCast<domMotion>(asmotion->createAndPlace(COLLADA_ELEMENT_MOTION));

    //  Articulated system motion identifier
    string  strASMotionId   =   asmotion->getID();

    //  Debug
    RAVELOG_VERBOSEA("Kinematic model Id: %s\n",kmodel->getId());

    kmodel->setName(probot->GetName().c_str());




    vector<KinBody::JointPtr > vjoints;
    vjoints.reserve(probot->GetJoints().size()+probot->_vecPassiveJoints.size());

    FOREACHC(itj, probot->GetJoints() )
    {
      KinBody::JointPtr pj(new KinBody::Joint(probot));
      *pj = **itj;
      vjoints.push_back(pj);
    }

    int dof = probot->GetDOF();

    FOREACHC(itj, probot->_vecPassiveJoints)
    {
      KinBody::JointPtr pj(new KinBody::Joint(probot));
      *pj = **itj;
      vjoints.push_back(pj);
      vjoints.back()->jointindex += probot->GetJoints().size();
      vjoints.back()->dofindex = dof;
      dof += vjoints.back()->GetDOF();
    }

    {
      //  Create Instance Kinematics Model
      domInstance_articulated_systemRef ias = daeSafeCast<domInstance_articulated_system>(scene.kscene->createAndPlace(COLLADA_ELEMENT_INSTANCE_ARTICULATED_SYSTEM));

      //  Set bindings of instance articulated system in Kinematics Scene
      SetBinding(ias,string(scene.kscene->getID()),strInstModelSid,strASMotionId,vjoints);

      //  Binding in Scene 'instance kinematics scene'
      domBind_kinematics_modelRef pmodelbind = daeSafeCast<domBind_kinematics_model>(scene.kiscene->createAndPlace(COLLADA_ELEMENT_BIND_KINEMATICS_MODEL));
      pmodelbind->setNode((bodyid+string(".node0")).c_str());
      daeSafeCast<domCommon_param>(pmodelbind->createAndPlace(COLLADA_ELEMENT_PARAM))->setValue(string(string(scene.kscene->getID()) + string(".") + strInstModelSid).c_str());

      //  Create 'instance articulated system' in 'motion' section
      domInstance_articulated_systemRef ias_motion    =   daeSafeCast<domInstance_articulated_system>(
          motion->createAndPlace(COLLADA_ELEMENT_INSTANCE_ARTICULATED_SYSTEM));

      //  Set bindings of 'instnance articulated system' in 'motion' section of articulated systems
      SetBinding(ias_motion,strASMotionId,strInstModelSid,strASKinematicsId,vjoints);

      //  Create 'instance kinematics model'
      domInstance_kinematics_modelRef ikm = daeSafeCast<domInstance_kinematics_model>(kinematics->createAndPlace(COLLADA_ELEMENT_INSTANCE_KINEMATICS_MODEL));
      SetSidRef(ikm,strASKinematicsId,strInstModelSid,strModelId,vjoints,vjointvalues);

      //  Joint parameters
      FOREACH(itjoint, vjoints)
      {
        string jointname = string("joint") + toString((*itjoint)->GetJointIndex());
        KinBody::LinkPtr pchildlink = GetChildLink(*itjoint, vjoints);

        if( pchildlink == NULL )
        {
          continue;
        }

        for(int idof = 0; idof < (*itjoint)->GetDOF(); ++idof)
        {
          string axisname = string("axis") + toString(idof);
          string bindname = string(scene.kscene->getID()) + string(".") + strInstModelSid + string(".") + jointname + string("_") + axisname;
          string uriname = string(scene.kscene->getID()) + string(".") + strInstModelSid + string(".") + jointname + string("_") + axisname;

          // binding
          domBind_joint_axisRef pjointbind = daeSafeCast<domBind_joint_axis>(scene.kiscene->createAndPlace(COLLADA_ELEMENT_BIND_JOINT_AXIS));
          pjointbind->setTarget((bodyid+string(".node")+toString(pchildlink->GetIndex())+string("/node_joint")+toString((*itjoint)->GetJointIndex())+string("_axis")+toString(idof)).c_str());
          domCommon_sidref_or_paramRef paxisbind = daeSafeCast<domCommon_sidref_or_param>(pjointbind->createAndPlace(COLLADA_ELEMENT_AXIS));
          daeSafeCast<domCommon_param>(paxisbind->createAndPlace(COLLADA_TYPE_PARAM))->setValue(bindname.c_str());
          domCommon_float_or_paramRef pvaluebind = daeSafeCast<domCommon_float_or_param>(pjointbind->createAndPlace(COLLADA_ELEMENT_VALUE));
          daeSafeCast<domCommon_param>(pvaluebind->createAndPlace(COLLADA_TYPE_PARAM))->setValue((bindname+string("_value")).c_str());
        }
      }
    }// End Create Instance Kinematics Model

    return true;
  }

  /// TODO : Simplifie Binding of instance articulated systems
  /// ias         Intance of articulated system
  /// ref_base    Reference of the element that calls the instance articulated system
  /// model_base  Central part of the bind string
  /// uri_base    Reference to the element that the instance calls.
  bool SetBinding(domInstance_articulated_systemRef ias, string ref_base, string model_base, string uri_base,vector<KinBody::JointPtr > vjoints)
  {
    ias->setUrl((string("#")+uri_base).c_str());

    string bind_base = ref_base + string(".") + model_base;
    string param_base = uri_base + string(".") + model_base;

    domKinematics_bindRef articulated_bind = daeSafeCast<domKinematics_bind>(ias->createAndPlace(COLLADA_ELEMENT_BIND));
    articulated_bind->setSymbol(bind_base.c_str());
    daeSafeCast<domKinematics_param>(articulated_bind->createAndPlace(COLLADA_ELEMENT_PARAM))->setRef(param_base.c_str());

    //  Joint parameters
    FOREACH(itjoint, vjoints)
    {
      string jointname = string("joint") + toString((*itjoint)->GetJointIndex());
      KinBody::LinkPtr pchildlink = GetChildLink(*itjoint, vjoints);

      if( pchildlink == NULL )
      {
        continue;
      }

      for(int idof = 0; idof < (*itjoint)->GetDOF(); ++idof)
      {
        string axisname = string("axis") + toString(idof);
        string bindname = bind_base + string(".") + jointname + string("_") + axisname;
        string uriname = param_base + string(".") + jointname + string("_") + axisname;

        // binding
        domKinematics_bindRef axis_motion_bind = daeSafeCast<domKinematics_bind>(ias->createAndPlace(COLLADA_ELEMENT_BIND));
        axis_motion_bind->setSymbol(bindname.c_str());
        daeSafeCast<domKinematics_param>(axis_motion_bind->createAndPlace(COLLADA_ELEMENT_PARAM))->setRef(uriname.c_str());

        // value
        domKinematics_bindRef axis_motion_value =   daeSafeCast<domKinematics_bind>(ias->createAndPlace(COLLADA_ELEMENT_BIND));
        axis_motion_value->setSymbol((bindname+string("_value")).c_str());
        daeSafeCast<domKinematics_param>(axis_motion_value->createAndPlace(COLLADA_ELEMENT_PARAM))->setRef((uriname+string("_value")).c_str());
      }

    }

    return true;
  }

  /// TODO : Simplifie sidref binding
  /// ias                 Intance of articulated system
  /// ref_base        Reference of the element that calls the instance articulated system
  /// model_base  Central part of the bind string
  /// uri_base        Reference to the element that the instance calls.
  bool SetSidRef( domInstance_kinematics_modelRef instance,
      string ref_base,
      string model_base,
      string uri_base,
      const vector<KinBody::JointPtr >& vjoints,
      vector<dReal> vjointvalues)
  {
    //  Heading instance values
    instance->setUrl((string("#")+uri_base).c_str());
    instance->setSid(model_base.c_str());

    string bind_base = ref_base + string(".") + model_base;
    string param_base = ref_base + string("/") + model_base;

    domKinematics_newparamRef newparam = daeSafeCast<domKinematics_newparam>(instance->createAndPlace(COLLADA_ELEMENT_NEWPARAM));
    newparam->setSid(bind_base.c_str());
    daeSafeCast<domKinematics_newparam::domSIDREF>(newparam->createAndPlace(COLLADA_ELEMENT_SIDREF))->setValue(param_base.c_str());

    //  Joint parameters
    FOREACHC(itjoint, vjoints)
    {
      string jointname = string("joint") + toString((*itjoint)->GetJointIndex());
      KinBody::LinkPtr pchildlink = GetChildLink(*itjoint, vjoints);

      if( pchildlink == NULL )
      {
        continue;
      }

      for(int idof = 0; idof < (*itjoint)->GetDOF(); ++idof)
      {
        string axisname = string("axis") + toString(idof);
        string bindname = bind_base + string(".") + jointname + string("_") + axisname;
        string uriname = param_base + string("/") + jointname + string("/") + axisname;

        // binding
        domKinematics_newparamRef axis_bind = daeSafeCast<domKinematics_newparam>(instance->createAndPlace(COLLADA_ELEMENT_NEWPARAM));
        axis_bind->setSid(bindname.c_str());
        daeSafeCast<domKinematics_newparam::domSIDREF>(axis_bind->createAndPlace(COLLADA_ELEMENT_SIDREF))->setValue(uriname.c_str());

        // value
        domKinematics_newparamRef axis_value    =   daeSafeCast<domKinematics_newparam>(instance->createAndPlace(COLLADA_ELEMENT_NEWPARAM));
        axis_value->setSid((bindname+string("_value")).c_str());
        daeSafeCast<domKinematics_newparam::domFloat>(axis_value->createAndPlace(COLLADA_ELEMENT_FLOAT))->getValue()    =
            vjointvalues[(*itjoint)->GetDOFIndex()+idof];
      }

    }

    return true;
  }

  /// Write link of a kinematic body
  /// link                Link to write
  /// pkinparent  Kinbody parent
  /// pnodeparent Node parent
  /// strModelUri
  /// vjoints         Vector of joints
  virtual LINKOUTPUT WriteLink(KinBody::LinkConstPtr plink, daeElementRef pkinparent,
      domNodeRef pnodeparent, const string& strModelUri,
      const vector<KinBody::JointPtr >& vjoints)
  {
    RAVELOG_INFOA("virtual LINKOUTPUT WriteLink(const KinBody::LinkPtr plink, daeElementRef pkinparent, domNodeRef pnodeparent, const string& strModelUri, const vector<KinBody::JointPtr >& vjoints)\n");

    LINKOUTPUT out;
    //string linkid = string("link")+toString(plink->GetIndex());
    string linkid = string(plink->GetName().c_str());
    domLinkRef pdomlink = daeSafeCast<domLink>(pkinparent->createAndPlace(COLLADA_ELEMENT_LINK));
    pdomlink->setName(plink->GetName().c_str());
    pdomlink->setSid(linkid.c_str());

    domNodeRef pnode = daeSafeCast<domNode>(pnodeparent->createAndPlace(COLLADA_ELEMENT_NODE));
    pnode->setId( (string("v")+toString(plink->GetParent()->GetNetworkId())+string(".node")+toString(plink->GetIndex())).c_str() );
    pnode->setSid( (string("node")+toString(plink->GetIndex())).c_str());
    pnode->setName(plink->GetName().c_str());

    int igeom = 0;
    FOREACHC(itgeom, plink->GetGeometries()) {
      string geomid = string("g") + toString(plink->GetParent()->GetNetworkId()) + string(".") + linkid + string(".geom") + toString(igeom++);
      domGeometryRef pdomgeom = WriteGeometry(*itgeom, geomid);
      domInstance_geometryRef pinstgeom = daeSafeCast<domInstance_geometry>(pnode->createAndPlace(COLLADA_ELEMENT_INSTANCE_GEOMETRY));
      pinstgeom->setUrl((string("#")+geomid).c_str());

      domBind_materialRef pmat = daeSafeCast<domBind_material>(pinstgeom->createAndPlace(COLLADA_ELEMENT_BIND_MATERIAL));
      domBind_material::domTechnique_commonRef pmattec = daeSafeCast<domBind_material::domTechnique_common>(pmat->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));
      domInstance_materialRef pinstmat = daeSafeCast<domInstance_material>(pmattec->createAndPlace(COLLADA_ELEMENT_INSTANCE_MATERIAL));
      pinstmat->setTarget(xsAnyURI(*pdomgeom, string("#")+geomid+string(".mat")));
      pinstmat->setSymbol("mat0");
    }

    // look for all the child links
    FOREACHC(itjoint, vjoints) {
      if( (*itjoint)->GetFirstAttached() != plink && (*itjoint)->GetSecondAttached() != plink )
        continue;
      KinBody::LinkPtr pchild = GetChildLink(*itjoint, vjoints);
      if( pchild == NULL || plink == pchild )
        continue;

      domLink::domAttachment_fullRef pattfull = daeSafeCast<domLink::domAttachment_full>(pdomlink->createAndPlace(COLLADA_TYPE_ATTACHMENT_FULL));
      pattfull->setJoint((strModelUri+string("/joint")+toString((*itjoint)->GetJointIndex())).c_str());

      LINKOUTPUT childinfo = WriteLink(pchild, pattfull, pnode, strModelUri, vjoints);
      out.listusedlinks.insert(out.listusedlinks.end(), childinfo.listusedlinks.begin(), childinfo.listusedlinks.end());

      Transform tLeft = (*itjoint)->tLeft;
      if( (*itjoint)->GetType() == KinBody::Joint::JointRevolute ) {
        // remove the offset
        tLeft = tLeft * Transform().rotfromaxisangle((*itjoint)->vAxes[0], (*itjoint)->offset);
      }

      AddTransformation(pattfull, tLeft, false);
      AddTransformation(childinfo.plink, (*itjoint)->tRight,false);

      AddTransformation(childinfo.pnode, (*itjoint)->tRight, false);
      // rotate/translate elements
      switch((*itjoint)->GetType())
      {
      case KinBody::Joint::JointRevolute:
      {
        domRotateRef protate = daeSafeCast<domRotate>(childinfo.pnode->createAndPlaceAt(0,COLLADA_ELEMENT_ROTATE));
        protate->setSid((string("node_joint")+toString((*itjoint)->GetJointIndex())+string("_axis0")).c_str());
        protate->getValue().setCount(4);
        protate->getValue()[0] = (*itjoint)->vAxes[0].x;
        protate->getValue()[1] = (*itjoint)->vAxes[0].y;
        protate->getValue()[2] = (*itjoint)->vAxes[0].z;
        protate->getValue()[3] = 0;
        break;
      }
      case KinBody::Joint::JointPrismatic: {
        domTranslateRef ptrans = daeSafeCast<domTranslate>(childinfo.pnode->createAndPlaceAt(0,COLLADA_ELEMENT_TRANSLATE));
        ptrans->setSid((string("node_joint")+toString((*itjoint)->GetJointIndex())+string("_axis0")).c_str());
        ptrans->getValue().setCount(3);
        ptrans->getValue()[0] = (*itjoint)->vAxes[0].x;
        ptrans->getValue()[1] = (*itjoint)->vAxes[0].y;
        ptrans->getValue()[2] = (*itjoint)->vAxes[0].z;
        break;
      }
      case KinBody::Joint::JointUniversal:
      case KinBody::Joint::JointHinge2:
      default:
        RAVELOG_WARNA("unsupported joint type specified %d\n", (*itjoint)->GetType());
        break;
      }

      AddTransformation(childinfo.pnode, tLeft, false);
    }

    out.listusedlinks.push_back(plink->GetIndex());

    out.plink = pdomlink;
    out.pnode = pnode;

    return  out;
  }

  /// Write geometry properties
  /// geom            Link geometry
  /// parentid    Parent Identifier
  virtual domGeometryRef WriteGeometry(const KinBody::Link::GEOMPROPERTIES& geom, const string& parentid)
  {
    RAVELOG_INFOA("virtual domGeometryRef WriteGeometry(const KinBody::Link::GEOMPROPERTIES& geom, const string& parentid)\n");

    const KinBody::Link::TRIMESH& mesh = geom.GetCollisionMesh();

    string effid = parentid+string(".eff");
    string matid = parentid+string(".mat");

    domEffectRef pdomeff = WriteEffect(geom.GetAmbientColor(), geom.GetDiffuseColor());
    pdomeff->setId(effid.c_str());

    domMaterialRef pdommat = daeSafeCast<domMaterial>(_materialsLib->createAndPlace(COLLADA_ELEMENT_MATERIAL));
    pdommat->setId(matid.c_str());
    domInstance_effectRef pdominsteff = daeSafeCast<domInstance_effect>(pdommat->createAndPlace(COLLADA_ELEMENT_INSTANCE_EFFECT));
    pdominsteff->setUrl((string("#")+effid).c_str());

    domGeometryRef pdomgeom = daeSafeCast<domGeometry>(_geometriesLib->createAndPlace(COLLADA_ELEMENT_GEOMETRY));
    {
      pdomgeom->setId(parentid.c_str());
      domMeshRef pdommesh = daeSafeCast<domMesh>(pdomgeom->createAndPlace(COLLADA_ELEMENT_MESH));
      {
        domSourceRef pvertsource = daeSafeCast<domSource>(pdommesh->createAndPlace(COLLADA_ELEMENT_SOURCE));
        {
          pvertsource->setId((parentid+string(".positions")).c_str());

          domFloat_arrayRef parray = daeSafeCast<domFloat_array>(pvertsource->createAndPlace(COLLADA_ELEMENT_FLOAT_ARRAY));
          parray->setId((parentid+string(".positions-array")).c_str());
          parray->setCount(mesh.vertices.size());
          parray->setDigits(6); // 6 decimal places
          parray->getValue().setCount(3*mesh.vertices.size());

          for(size_t ind = 0; ind < mesh.vertices.size(); ++ind)
          {
            Vector v = geom.GetTransform() * mesh.vertices[ind];
            parray->getValue()[3*ind+0] = v.x;
            parray->getValue()[3*ind+1] = v.y;
            parray->getValue()[3*ind+2] = v.z;
          }

          domSource::domTechnique_commonRef psourcetec = daeSafeCast<domSource::domTechnique_common>(pvertsource->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));
          domAccessorRef pacc = daeSafeCast<domAccessor>(psourcetec->createAndPlace(COLLADA_ELEMENT_ACCESSOR));
          pacc->setCount(mesh.vertices.size());
          pacc->setSource(xsAnyURI(*parray, string("#")+parentid+string(".positions-array")));
          pacc->setStride(3);

          domParamRef px = daeSafeCast<domParam>(pacc->createAndPlace(COLLADA_ELEMENT_PARAM));
          px->setName("X"); px->setType("float");
          domParamRef py = daeSafeCast<domParam>(pacc->createAndPlace(COLLADA_ELEMENT_PARAM));
          py->setName("Y"); py->setType("float");
          domParamRef pz = daeSafeCast<domParam>(pacc->createAndPlace(COLLADA_ELEMENT_PARAM));
          pz->setName("Z"); pz->setType("float");
        }

        domVerticesRef pverts = daeSafeCast<domVertices>(pdommesh->createAndPlace(COLLADA_ELEMENT_VERTICES));
        {
          pverts->setId("vertices");
          domInput_localRef pvertinput = daeSafeCast<domInput_local>(pverts->createAndPlace(COLLADA_ELEMENT_INPUT));
          pvertinput->setSemantic("POSITION");
          pvertinput->setSource(domUrifragment(*pvertsource, string("#")+parentid+string(".positions")));
        }

        domTrianglesRef ptris = daeSafeCast<domTriangles>(pdommesh->createAndPlace(COLLADA_ELEMENT_TRIANGLES));
        {
          ptris->setCount(mesh.indices.size()/3);
          ptris->setMaterial("mat0");

          domInput_local_offsetRef pvertoffset = daeSafeCast<domInput_local_offset>(ptris->createAndPlace(COLLADA_ELEMENT_INPUT));
          pvertoffset->setSemantic("VERTEX");
          pvertoffset->setOffset(0);
          pvertoffset->setSource(domUrifragment(*pverts, string("#")+parentid+string("/vertices")));
          domPRef pindices = daeSafeCast<domP>(ptris->createAndPlace(COLLADA_ELEMENT_P));
          pindices->getValue().setCount(mesh.indices.size());
          for(size_t ind = 0; ind < mesh.indices.size(); ++ind)
            pindices->getValue()[ind] = mesh.indices[ind];
        }
      }
    }

    return pdomgeom;
  }

  /// Write light effect
  /// vambient    Ambient light color
  /// vdiffuse    Diffuse light color
  virtual domEffectRef WriteEffect(const Vector& vambient, const Vector& vdiffuse)
  {
    RAVELOG_INFOA("virtual domEffectRef WriteEffect(const Vector& vambient, const Vector& vdiffuse)\n");

    domEffectRef pdomeff = daeSafeCast<domEffect>(_effectsLib->createAndPlace(COLLADA_ELEMENT_EFFECT));

    domProfile_commonRef pprofile = daeSafeCast<domProfile_common>(pdomeff->createAndPlace(COLLADA_ELEMENT_PROFILE_COMMON));
    domProfile_common::domTechniqueRef ptec = daeSafeCast<domProfile_common::domTechnique>(pprofile->createAndPlace(COLLADA_ELEMENT_TECHNIQUE));

    domProfile_common::domTechnique::domPhongRef pphong = daeSafeCast<domProfile_common::domTechnique::domPhong>(ptec->createAndPlace(COLLADA_ELEMENT_PHONG));

    domFx_common_color_or_textureRef pambient = daeSafeCast<domFx_common_color_or_texture>(pphong->createAndPlace(COLLADA_ELEMENT_AMBIENT));
    domFx_common_color_or_texture::domColorRef pambientcolor = daeSafeCast<domFx_common_color_or_texture::domColor>(pambient->createAndPlace(COLLADA_ELEMENT_COLOR));
    setVector4(pambientcolor->getValue(), vambient);

    domFx_common_color_or_textureRef pdiffuse = daeSafeCast<domFx_common_color_or_texture>(pphong->createAndPlace(COLLADA_ELEMENT_DIFFUSE));
    domFx_common_color_or_texture::domColorRef pdiffusecolor = daeSafeCast<domFx_common_color_or_texture::domColor>(pdiffuse->createAndPlace(COLLADA_ELEMENT_COLOR));
    setVector4(pdiffusecolor->getValue(), vdiffuse);

    return pdomeff;
  }

  /// Write transformation
  /// pelt                    Element to transform
  /// t                           Transform to write
  /// bAtEnd = true   Where to place the tranformation. If true, translate before rotation. If false the contrary.
  void AddTransformation(daeElementRef pelt, Transform t, bool bAtEnd = true)
  {
    RAVELOG_INFOA("void AddTransformation(daeElementRef pelt, Transform t, bool bAtEnd = true)\n");

    domTranslateRef ptrans;
    domRotateRef prot;

    if( bAtEnd )
    {
      ptrans = daeSafeCast<domTranslate>(pelt->createAndPlace(COLLADA_ELEMENT_TRANSLATE));
      prot = daeSafeCast<domRotate>(pelt->createAndPlace(COLLADA_ELEMENT_ROTATE));
    }
    else
    {
      prot = daeSafeCast<domRotate>(pelt->createAndPlaceAt(0,COLLADA_ELEMENT_ROTATE));
      ptrans = daeSafeCast<domTranslate>(pelt->createAndPlaceAt(0,COLLADA_ELEMENT_TRANSLATE));
    }

    ptrans->getValue().setCount(3);
    ptrans->getValue()[0] = t.trans.x;
    ptrans->getValue()[1] = t.trans.y;
    ptrans->getValue()[2] = t.trans.z;

    prot->getValue().setCount(4);
    // extract axis from quaternion
    dReal fnorm = RaveSqrt(t.rot.y*t.rot.y+t.rot.z*t.rot.z+t.rot.w*t.rot.w);

    if( fnorm > 0 )
    {
      prot->getValue()[0] = t.rot.y/fnorm;
      prot->getValue()[1] = t.rot.z/fnorm;
      prot->getValue()[2] = t.rot.w/fnorm;
      prot->getValue()[3] = atan2(fnorm, t.rot.x)*360.0f/PI;
    }
    else
    {
      prot->getValue()[0] = 1;
      prot->getValue()[1] = 0;
      prot->getValue()[2] = 0;
      prot->getValue()[3] = 0;
    }
  }

  /// Set vector of four elements
  template <typename T>
  void setVector4(T& t, const Vector& v)
  {
    t.setCount(4);
    t[0] = v.x;
    t[1] = v.y;
    t[2] = v.z;
    t[3] = v.w;
  }

  /// Set vector of three elements
  template <typename T>
  void setVector3(T& t, const Vector& v)
  {
    t.setCount(3);
    t[0] = v.x;
    t[1] = v.y;
    t[2] = v.z;
  }

  /// Write down a COLLADA file
  virtual bool Save(const string& filename)
  {
    RAVELOG_WARNA("virtual bool Save(const string& filename)\n");

    return _collada->saveAs(filename.c_str())>0;
  }

  virtual void handleError( daeString msg )
  {
    RAVELOG_INFOA("virtual void handleError( daeString msg )\n");

    RAVELOG_ERRORA("COLLADA error: %s\n", msg);
  }

  virtual void handleWarning( daeString msg )
  {
    RAVELOG_INFOA("virtual void handleWarning( daeString msg )\n");

    RAVELOG_WARNA("COLLADA warning: %s\n", msg);
  }

  virtual KinBody::LinkPtr GetChildLink(KinBody::JointPtr pjoint, const vector<KinBody::JointPtr >& vjoints)
  {
    RAVELOG_INFOA("virtual KinBody::LinkPtr GetChildLink(KinBody::JointPtr pjoint, const vector<KinBody::JointPtr >& vjoints)\n");

    KinBodyConstPtr pbody = pjoint->GetParent();
    KinBody::LinkPtr pchildlink;
    int jointindex = pjoint->GetMimicJointIndex() < 0 ? pjoint->GetJointIndex() : pjoint->GetMimicJointIndex();

    if( pjoint->GetFirstAttached() != NULL && pbody->DoesAffect(jointindex, pjoint->GetFirstAttached()->GetIndex()) )
    {
      pchildlink = pjoint->GetFirstAttached();
    }

    if( pjoint->GetSecondAttached() != NULL && pbody->DoesAffect(jointindex, pjoint->GetSecondAttached()->GetIndex()) )
    {
      BOOST_ASSERT( pjoint->GetMimicJointIndex() >= 0 || pchildlink == NULL );
      bool bSetSecond = true;
      if( pchildlink != NULL )
      {
        // in case both are affected, choose link closest to base
        // by checking which parent joints contain the link
        FOREACHC(itjoint, vjoints)
                    {
          if( (*itjoint)->GetMimicJointIndex() < 0 )
          {
            if( (*itjoint)->GetFirstAttached() == pjoint->GetSecondAttached() ||
                (*itjoint)->GetSecondAttached() == pjoint->GetSecondAttached() )
            {
              bSetSecond  =   false;
              break;
            }
          }
                    }
      }

      if( bSetSecond )
      {
        pchildlink = pjoint->GetSecondAttached();
      }
    }

    if( pchildlink == NULL )
      RAVELOG_ERRORA("joint %s attached to invalid links\n", pjoint->GetName().c_str());

    return pchildlink;
  }

private:
  template <class T>
  static string toString(const T& t) {
    stringstream ss;
    ss << t;
    return ss.str();
  }

  boost::shared_ptr<DAE> _collada;
  domCOLLADA* _dom;
  domCOLLADA::domSceneRef                     _scene;
  domLibrary_visual_scenesRef             _visualScenesLib;
  domLibrary_kinematics_scenesRef     _kinematicsScenesLib;
  domLibrary_kinematics_modelsRef     _kinematicsModelsLib;
  domLibrary_articulated_systemsRef   _articulatedSystemsLib;
  domLibrary_physics_scenesRef            _physicsScenesLib;
  domLibrary_materialsRef                     _materialsLib;
  domLibrary_effectsRef                       _effectsLib;
  domLibrary_geometriesRef                    _geometriesLib;
    //domLibrary_sensorsRef                           _sensorsLib;//  Specifics to OpenRAVE Library sensors.
    EnvironmentBaseConstPtr _penv;
};

#endif
