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
#ifndef OPENRAVE_COLLADA_READER_H
#define OPENRAVE_COLLADA_READER_H

#include "ravep.h"

using namespace OpenRAVE;
using namespace std;

#include <dae.h>
#include <dae/daeErrorHandler.h>
#include <dom/domCOLLADA.h>
#include <dae/domAny.h>
#include <dom/domConstants.h>
#include <dom/domTriangles.h>
#include <dae/daeStandardURIResolver.h>

//  Stefan change
struct NodeMatcher: public daeElement::matchElement {
  public:
    NodeMatcher(){};
    virtual bool operator()(daeElement* elt) const
    {return (domNode::ID()==elt->typeID());};
};
//  ---

class ColladaReader: public daeErrorHandler {
  struct KINEMATICSBINDING {

    KINEMATICSBINDING(daeElementRef pvisualtrans,
        domAxis_constraintRef pkinematicaxis, dReal fjointvalue, domKinematics_axis_infoRef dkai, domMotion_axis_infoRef dmai) :
          pvisualtrans(pvisualtrans), pkinematicaxis(pkinematicaxis), fjointvalue(
              fjointvalue), pkinematicaxisinfo(dkai), pmotionaxisinfo(dmai) {

      BOOST_ASSERT( pkinematicaxis != NULL );

      pvisualnode = NULL;

      daeElement* pae = pvisualtrans->getParentElement();

      while (pae != NULL)
      {
        pvisualnode = daeSafeCast<domNode> (pae);

        if (pvisualnode != NULL)
        {
          break;
        }

        pae = pae->getParentElement();
      }

      if (pvisualnode == NULL)
      {
        RAVELOG_WARNA("couldn't find parent node of element id %s, sid %s\n",
            pkinematicaxis->getID(), pkinematicaxis->getSid());
      }
    }

    daeElementRef                   pvisualtrans;
    domAxis_constraintRef   pkinematicaxis;
    dReal                                   fjointvalue;
    domNodeRef                      pvisualnode;
    domKinematics_axis_infoRef  pkinematicaxisinfo;
    domMotion_axis_infoRef          pmotionaxisinfo;
  };

  struct USERDATA {
    USERDATA() {
    }
    USERDATA(dReal scale) :
      scale(scale) {
    }
    dReal scale;
  };

public:
  ColladaReader(EnvironmentBasePtr penv) :
    _dom(NULL), _penv(penv) {
    daeErrorHandler::setErrorHandler(this);
  }
  virtual ~ColladaReader() {
    _collada.reset();
    DAE::cleanup();
  }

  bool InitFromFile(const string& filename) {
    RAVELOG_VERBOSEA(
        "init COLLADA reader version: %s, namespace: %s, filename: %s\n",
        COLLADA_VERSION, COLLADA_NAMESPACE, filename.c_str());
    _collada.reset(new DAE);

    //  Debug
    RAVELOG_VERBOSEA("Open file %s\n",filename.c_str());

    _dom = _collada->open(filename);
    if (!_dom)
      return false;

    //  Debug
    RAVELOG_VERBOSEA("File Opened!!!\n");

    size_t maxchildren = countChildren(_dom);
    _vuserdata.resize(0);
    _vuserdata.reserve(maxchildren);

    //  Debug
    dReal dScale = 1.0;
    processUserData(_dom, dScale);
    RAVELOG_VERBOSEA("processed children: %d/%d\n", _vuserdata.size(),
        maxchildren);
    return true;
  }
  bool InitFromData(const string& pdata) {
    BOOST_ASSERT(0);
    return false;
  }

  size_t countChildren(daeElement* pelt) {
    size_t c = 1;
    for (size_t i = 0; i < pelt->getChildren().getCount(); ++i)
      c += countChildren(pelt->getChildren()[i]);
    return c;
  }

  void processUserData(daeElement* pelt, dReal scale) {

    // getChild could be optimized since asset tag is supposed to appear as the first element
    domAssetRef passet = daeSafeCast<domAsset> (pelt->getChild("asset"));
    if (passet != NULL && passet->getUnit() != NULL)
    {
      scale = passet->getUnit()->getMeter();
    }

    _vuserdata.push_back(USERDATA(scale));
    pelt->setUserData(&_vuserdata.back());

    for (size_t i = 0; i < pelt->getChildren().getCount(); ++i) {
      if (pelt->getChildren()[i] != passet)
        processUserData(pelt->getChildren()[i], scale);
    }
  }

  //  TODO :  Search axis info corresponding to the axis given
  template <typename T,typename U>
  const daeSmartRef<T> getAxisInfo(daeString axis, const U& axis_array)
  {
    //  Debug
    RAVELOG_VERBOSEA("Axis to find: %s\n",axis);

    for (uint32_t i = 0; i < axis_array.getCount(); i++)
    {
      if (strcmp(axis_array[i]->getAxis(),axis) == 0)
      {
        //  Debug
        RAVELOG_DEBUGA("Axis found: %s\n",axis);
        return axis_array[i];
      }
    }

    return NULL;
  }

  /// Extract environment from a COLLADA Scene
  /// This is the main proccess in the parser
  bool Extract(EnvironmentBasePtr penv) {
    bool                    isRobot;        //  Distinguish between a Kinematic Model that is a Robot or not
    daeString               robotName = NULL;   //  Articulated System ID. Describes the Robot
    domKinematics_frameRef  pframe_origin = NULL;   //  Manipulator Base
    domKinematics_frameRef  pframe_tip      = NULL; //  Manipulator Effector

    domCOLLADA::domSceneRef allscene = _dom->getScene();
    BOOST_ASSERT(allscene != NULL);
    vector<domKinematics_newparam*> vnewparams;

    //  All bindings of the kinematics models presents in the scene
    vector<KINEMATICSBINDING> v_all_bindings;

    //  Nodes that are part of kinematics models
    vector<string>  processed_nodes;

    //  For each Kinematics Scene
    for (size_t iscene = 0; iscene < allscene->getInstance_kinematics_scene_array().getCount(); iscene++)
    {
      //  Gets the Scene
      domInstance_kinematics_sceneRef kiscene =
        allscene->getInstance_kinematics_scene_array()[iscene];

      domKinematics_sceneRef kscene = daeSafeCast<domKinematics_scene> (
          kiscene->getUrl().getElement().cast());

      // If there is not Kinematic Scene
      if (!kscene)
      {
        continue;
      }

      //  For each Bind of Kinematics Model
      for (size_t imodel = 0; imodel
      < kiscene->getBind_kinematics_model_array().getCount(); imodel++) {

        //  Kinimatics model may NOT be a Robot
        domArticulated_systemRef articulated_system =   NULL;

        //  Initially kinematics model is NOT a Robot
        isRobot = false;

        // Gets Bind of Kinematics Model
        domBind_kinematics_modelRef kbindmodel =
          kiscene->getBind_kinematics_model_array()[imodel];

        //  If there is no model
        if (kbindmodel->getNode() == NULL) {
          RAVELOG_WARNA(
              "do not support kinematics models without references to nodes\n");
          continue;
        }

        //  Debug
        RAVELOG_WARNA("Node: %s\n",kbindmodel->getNode());

        // Gets the Node of the Kinematics Model
        domNodeRef pnode = daeSafeCast<domNode> (daeSidRef(
            kbindmodel->getNode(), kbindmodel).resolve().elt);
        if (pnode == NULL || pnode->typeID() != domNode::ID()) {
          RAVELOG_WARNA(
              "bind_kinematics_model does not reference valid node %s\n",
              kbindmodel->getNode());
          continue;
        }
        else
        {
          RAVELOG_WARNA("Kinematics model node name: %s\n",pnode->getName());
        }

        //  Get ID of node parent
        string  parentnode  = daeSafeCast<domNode>(pnode->getParent())->getID();

        //  Store ID's of nodes that are part of kinematics model
        processed_nodes.push_back(parentnode);

        RAVELOG_VERBOSEA("Parent node ID %s\n",parentnode.c_str());

        //  Instance Kinematics Model
        domInstance_kinematics_modelRef kimodel;
        kimodel = getSidRef<domInstance_kinematics_model> (kbindmodel,kscene);
        if (kimodel == NULL)
        {
          RAVELOG_WARNA(
              "bind_kinematics_model does not reference valid kinematics\n");
        }
        else
        {
          //  TODO : Debug
          RAVELOG_DEBUGA("Instance Kinematics model %s\n",kimodel->getSid());
        }


        if (getElement()->typeID() == domArticulated_system::ID())
        {
          articulated_system = daeSafeCast<domArticulated_system>(getElement().cast());

          //  Debug
          RAVELOG_DEBUGA("Got articulated_system!!!\n");

          isRobot     = true;
          robotName   = articulated_system->getId();

          //  Check if there is a technique_common section
          if (!articulated_system->getKinematics()->getTechnique_common())
          {
            RAVELOG_VERBOSEA("Skip technique_common in articulated_system/kinematics\n");
          }
          else
          {
            RAVELOG_VERBOSEA("Kinematics Model is an Articulated System (Robot)\n");

            pframe_origin   = articulated_system->getKinematics()->getTechnique_common()->getFrame_origin();
            pframe_tip      = articulated_system->getKinematics()->getTechnique_common()->getFrame_tip();
          }
        }

        // Kinematics Model
        domKinematics_modelRef kmodel = daeSafeCast<domKinematics_model> (
            kimodel->getUrl().getElement().cast());
        if (!kmodel) {
          RAVELOG_WARNA(
              "bind_kinematics_model does not reference valid kinematics\n");
          return false;
        }

        // TODO : Get the joint binds
        vector<KINEMATICSBINDING> vbindings;
        for (size_t ijoint = 0; ijoint < kiscene->getBind_joint_axis_array().getCount(); ++ijoint) {

          int pos;

          domBind_joint_axisRef bindjoint =   kiscene->getBind_joint_axis_array()[ijoint];

          string  strNode     =   string(kbindmodel->getParam()->getValue());
          string  strTarget   =   string(bindjoint->getAxis()->getParam()->getValue());

          pos =   strTarget.find(strNode);

          //  Debug
          RAVELOG_DEBUGA("Kinematics Node %s Target %s Pos %d\n",strNode.c_str(),strTarget.c_str(),pos);

          if (pos == -1)
          {
            RAVELOG_VERBOSEA("This Axis NOT belongs to the Kinematics Model\n");
            continue;
          }

          domKinematics_axis_infoRef  dkai    = NULL;
          domMotion_axis_infoRef      dmai    = NULL;

          daeElementRef pjtarget =    daeSidRef(bindjoint->getTarget(), bindjoint).resolve().elt;

          if (pjtarget == NULL) {
            RAVELOG_ERRORA("Target Node %s NOT found!!!\n", bindjoint->getTarget());
            continue;
          }

          RAVELOG_WARNA("Target Node %s FOUND!!!\n", bindjoint->getTarget());

          //  Retrieve the joint
          domAxis_constraintRef pjointaxis = getSidRef<domAxis_constraint> (
              bindjoint->getAxis(),
              kscene);

          if (pjointaxis == NULL) {
            RAVELOG_ERRORA("Joint Axis %s NOT found\n",getRef());
            RAVELOG_ERRORA("Bind Joint Axis %s\n",bindjoint->getAxis()->getID());

            continue;
          }

          RAVELOG_WARNA("Joint Axis %s FOUND!!!\n",getRef());

          domInstance_kinematics_model_Array instance_kinematics_array;

          //  If the system is NOT a Robot
          if (getElement()->typeID() == domKinematics_scene::ID())
          {
            RAVELOG_WARNA("Kinematics Scene Element \n");
            domKinematics_sceneRef dks = daeSafeCast<domKinematics_scene>(getElement().cast());
            instance_kinematics_array = dks->getInstance_kinematics_model_array();
          }
          //  If the system is a Robot
          else if (getElement()->typeID() == domArticulated_system::ID())
          {
            RAVELOG_WARNA("Articulated System Element \n");
            //  Gets articulated system KINEMATICS
            //domArticulated_systemRef das = daeSafeCast<domArticulated_system>(getElement().cast());
            instance_kinematics_array = articulated_system->getKinematics()->getInstance_kinematics_model_array();

            // Search and fill Joint properties
            domKinematics_axis_info_Array dkai_array =  articulated_system->getKinematics()->getTechnique_common()->getAxis_info_array();

            string strAxis  =   string(getSidRef_value());

            //  Debug
            RAVELOG_VERBOSEA("SidRef value %s\n",strAxis.c_str());

            pos =   strAxis.find_last_of("/");
            pos =   strAxis.find_last_of("/",pos-1);

            strAxis =   kimodel->getUrl().fragment() + strAxis.substr(pos).c_str();

            RAVELOG_DEBUGA("Search axis string: %s\n",strAxis.c_str());

            //  Gets axis info of the articulated system KINEMATICS
            dkai = getAxisInfo<domKinematics_axis_info>(strAxis.c_str(),dkai_array);

            //  Check if there is a 'technique_common' section
            if (!!_motion_element->getTechnique_common())
            {
              domMotion_axis_info_Array dmai_array = _motion_element->getTechnique_common()->getAxis_info_array();

              RAVELOG_VERBOSEA("Check out articulated system ID\n");

              if (articulated_system->getID() == NULL)
              {
                RAVELOG_WARNA("ARTICULATED SYSTEM KINEMATICS Id is NULL ...\n");
              }

              RAVELOG_VERBOSEA("SidRef: %s\n",getSidRef_value());


              //                          //  Build the TAG for search axis info in the articulated system MOTION section. Dot format!!!
              //                          char axis_tag[256];
              //                          strcpy(axis_tag,articulated_system->getID());
              //                          strcat(axis_tag,"/");
              //                          strcat(axis_tag,dkai->getSid());

              strAxis =   string(articulated_system->getID()) + string("/") + string(dkai->getSid());

              //  Debug
              RAVELOG_WARNA("str: %s\n",strAxis.c_str());

              //  Gets axis info of the articulated system MOTION
              dmai    =   getAxisInfo<domMotion_axis_info>(strAxis.c_str(),dmai_array);

              //  Debug
              RAVELOG_WARNA("End of the search for AXIS INFO ...\n");
            }
          }

          domFloat fdefjointvalue = resolveFloat(bindjoint->getValue(),
              instance_kinematics_array);

          //  Stores joint info
          vbindings.push_back(KINEMATICSBINDING(pjtarget, pjointaxis,
              fdefjointvalue, dkai, dmai));

          //  Store joint info used for the visual scene
          v_all_bindings.push_back(KINEMATICSBINDING(pjtarget, pjointaxis,
              fdefjointvalue, dkai, dmai));
        }

        //  Obtain Kinmodel from COLLADA
        domPhysics_modelRef pmodel = NULL;
        KinBodyPtr pbody;
        RobotBasePtr probot;
        if (!Extract(pbody, kmodel, pmodel, pnode, vbindings)) {
          RAVELOG_WARNA("failed to load kinbody from kin instance %s\n",
              kimodel->getID());
          continue;
        }

        //  The kinbody is NOT a Robot
        if (!isRobot)
        {
          //  Adds a Kinbody to the Environment
          RAVELOG_VERBOSEA("Kinbody %s added to the environment\n",pbody->GetName().c_str());
          _penv->AddKinBody(pbody);
        }
        else
        {
          //  Create Robot
          probot  = _penv->CreateRobot("");

          //  Copy the kinbody information into the Robot structure
          probot->KinBody::Clone(pbody,0);

          //  Extract instances of sensors
          ExtractSensors<domArticulated_system>(articulated_system,probot);

//          //  Debug
//          RAVELOG_INFO("Number of sensors of the Robot: %d\n",(int)probot->GetSensors().size());
//
//          //  Setup Manipulator of the Robot
//          RobotBase::ManipulatorPtr manipulator(new RobotBase::Manipulator(probot));
//          probot->GetManipulators().push_back(manipulator);
//
//          //  Debug
//          RAVELOG_WARNA("Number of Manipulators %d ¡¡¡\n",probot->GetManipulators().size());
//
//          int     pos;
//          string  linkName  = string(pframe_origin->getLink());
//
//          pos       = linkName.find_first_of("/");
//          linkName  = linkName.substr(pos + 1);
//
//          RAVELOG_VERBOSEA("Manipulator link name %s\n",linkName.c_str());
//
//          //  Sets frame_origin and frame_tip
//          manipulator->_pBase              = probot->GetLink(linkName);
//
//          if (!!manipulator->_pBase)
//          {
//            RAVELOG_WARNA("Manipulator::pBase ... %s\n",manipulator->_pBase->GetName().c_str());
//          }
//          else
//          {
//            RAVELOG_WARNA("Error initializing Manipulator::pBase\n");
//          }
//
//          linkName = string(pframe_tip->getLink());
//          pos       = linkName.find_first_of("/");
//          linkName  = linkName.substr(pos + 1);
//
//          manipulator->_pEndEffector   = probot->GetLink(linkName);
//
//          if (!!manipulator->_pEndEffector)
//          {
//            RAVELOG_WARNA("Manipulator::pEndEffector ... %s\n",manipulator->_pEndEffector->GetName().c_str());
//          }
//          else
//          {
//            RAVELOG_WARNA("Error initializing Manipulator::pEndEffector\n");
//          }
//
//          //  Initialize indices that then manipulator controls
//          for (size_t i   =   0;  i < probot->GetJointIndices().size();   i++)
//          {
//            manipulator->_vgripperjoints.push_back(probot->GetJointIndices()[i]);
//          }
//
//          RAVELOG_VERBOSEA("Indices initialized...\n");

          //  Add the robot to the environment
          _penv->AddRobot(probot);

          RAVELOG_WARNA("Robot %s created ...\n",robotName);
        }

      }// End Kinematics model Process

    }// End Instance Kinematics scene Process

    //  TODO : Add Rigid objects without joints
    //  Visual Scene process
    // Iis this really necessary? aren't instance visual scene + instance kinematics scene pointing to the same thing?
    if (allscene->getInstance_visual_scene() != NULL)
    {
      domVisual_sceneRef visual_scene = daeSafeCast<domVisual_scene>(allscene->getInstance_visual_scene()->getUrl().getElement().cast());

      for (size_t node = 0; node < visual_scene->getNode_array().getCount(); node++)
      {
        KinBodyPtr rigid_body;
        domNodeRef pnode        =   visual_scene->getNode_array()[node];

        bool    found   = false;
        string  nodeId  = string(pnode->getID());

        //  Search if the node is into processed nodes
        for(size_t i=0;i < processed_nodes.size();i++)
        {
          //  If node belongs to processed nodes
          if (nodeId == processed_nodes[i])
          {
            RAVELOG_VERBOSEA("Processed node name: %s\n",processed_nodes[i].c_str());
            found = true;
            break;
          }
        }

        //  If the node is not part of a kinbody
        if (!found)
        {
          //  Debug
          RAVELOG_WARNA("Extract node: %s\n",pnode->getName());

          if (!Extract(rigid_body, NULL, NULL, pnode, v_all_bindings))
          {
            RAVELOG_WARNA("failed to load kinbody WIHTOUT Joints\n");
            continue;
          }

          _penv->AddKinBody(rigid_body);

          RAVELOG_VERBOSEA("Found node %s\n",visual_scene->getNode_array()[node]->getName());
        }
      }
    }// End Visual Scene Process

    return true;
  } // End Extract(EnvironmentBasePtr penv)

  /// Extract Sensors attached to a Robot
  template <typename T>
  bool ExtractSensors(const T* parent, RobotBasePtr probot)
  {
    if (parent->getExtra_array().getCount() == 0)
    {
      RAVELOG_WARNA("There is not an Extra Label\n");
      return false;
    }

    for (size_t i = 0; i < parent->getExtra_array().getCount(); i++)
    {
      domExtraRef extra = parent->getExtra_array()[i];

      for (size_t j = 0; j < extra->getTechnique_array().getCount(); j++)
      {
        domTechniqueRef technique = extra->getTechnique_array()[j];

        if (strcmp(technique->getProfile(),"OpenRAVE") == 0)
        {
          RAVELOG_WARNA("Technique Profile: %s\n",technique->getProfile());

          ExtractInstance_sensor(technique->getContents(),probot);
        }
      }
    }

    return  true;
  }

  /// Extract instance sensor info
  /// Extract instances of sensors located in articulated_systems extra node
  bool ExtractInstance_sensor(daeElementRefArray instances, RobotBasePtr probot)
  {
    std::string instance_id;
    std::string instance_url;
    std::string instance_link;
    std::string definition_id;
    std::string definition_type;
    daeElementRef dom_SensorActuatorManipulator;

    for (size_t i_instance = 0; i_instance < instances.getCount(); i_instance++)
    {
      daeElementRef instance_SensorActuatorManipulator = instances[i_instance];
      RAVELOG_DEBUG("Instance name: %s\n",instance_SensorActuatorManipulator->getElementName());
      if ((strcmp(instance_SensorActuatorManipulator->getElementName(),"instance_actuator") == 0)
          ||
          (strcmp(instance_SensorActuatorManipulator->getElementName(),"instance_sensor") == 0)
          ||
          (strcmp(instance_SensorActuatorManipulator->getElementName(),"instance_manipulator") == 0))
      {
        //  Get instance attributes
        daeTArray<daeElement::attr> instance_attributes = instance_SensorActuatorManipulator->getAttributes();
        for (size_t i_ins_attr = 0; i_ins_attr < instance_attributes.getCount(); i_ins_attr++)
        {
          RAVELOG_DEBUG("Instance attribute %d %s: %s\n",i_ins_attr,instance_attributes[i_ins_attr].name.c_str(),instance_attributes[i_ins_attr].value.c_str());

          if (instance_attributes[i_ins_attr].name == "id")
          {
            instance_id = instance_attributes[i_ins_attr].value;
          }

          if (instance_attributes[i_ins_attr].name == "url")
          {
            instance_url =  instance_attributes[i_ins_attr].value;
          }

          if (instance_attributes[i_ins_attr].name == "link")
          {
            instance_link = instance_attributes[i_ins_attr].value;
          }
        }

        RAVELOG_DEBUG("Get SensorActuatorManipulator info from url\n");

        daeURI  url = daeURI(*(instance_SensorActuatorManipulator.cast()),instance_url);
        dom_SensorActuatorManipulator = getElementFromUrl(url);

        //  Get definition attributes
        daeTArray<daeElement::attr> definition_attributes = dom_SensorActuatorManipulator->getAttributes();
        for (size_t i_def_attr = 0; i_def_attr < definition_attributes.getCount(); i_def_attr++)
        {
          if (definition_attributes[i_def_attr].name == "type")
          {
            definition_type  = definition_attributes[i_def_attr].value;
          }

          if (definition_attributes[i_def_attr].name == "id")
          {
            definition_id = definition_attributes[i_def_attr].value;
          }

          RAVELOG_DEBUG("SensorActuator attribute %d %s: %s\n", i_def_attr,
                                                                definition_attributes[i_def_attr].name.c_str(),
                                                                definition_attributes[i_def_attr].value.c_str());
        }

//        //  Create Actuator
//        if (strcmp(instance_SensorActuatorManipulator->getElementName(),"instance_actuator") == 0)
//        {
//          addActuator(probot,definition_id,definition_type,instance_id,dom_SensorActuatorManipulator);
//        }
        //  Create Sensor
        if (strcmp(instance_SensorActuatorManipulator->getElementName(),"instance_sensor") == 0)
        {
          addSensor(probot,definition_id,
                    definition_type,
                    instance_id,
                    instance_link,
                    dom_SensorActuatorManipulator,
                    instance_SensorActuatorManipulator);
        }
//        //  Create Manipulator
//        else
//        {
//          addManipulator(probot,instance_id,dom_SensorActuatorManipulator);
//        }

      }
    }

    return true;
  }

  //  Create Sensor and initilize it
  bool addSensor( RobotBasePtr  probot,
                  std::string   definition_id,
                  std::string   definition_type,
                  std::string   instance_id,
                  std::string   instance_link,
                  daeElementRef dom_SensorActuatorManipulator,
                  daeElementRef instance_SensorActuatorManipulator)
  {
    RobotBase::AttachedSensorPtr att_Sensor;
    //RobotBase::AttachedSensorPtr att_SensorActuator(new RobotBase::AttachedSensor(probot));
    att_Sensor = boost::shared_ptr<RobotBase::AttachedSensor>(new RobotBase::AttachedSensor(probot));

    probot->GetSensors().push_back(att_Sensor);

    //  Create Sensor of the TYPE required
    att_Sensor->psensor = probot->GetEnv()->CreateSensor(definition_type.c_str());

    att_Sensor->psensor->SetName(definition_id.c_str());

    //  Sets attached actuator name from instance actuator Id
    att_Sensor->_name   =   instance_id.c_str();

    //  Sets sensor name from dom sensor Id
    RAVELOG_WARN("Sensor name: %s\n",att_Sensor->GetName().c_str());

    //  Create XML reader for this Sensor TYPE
    OpenRAVEXMLParser::READERSMAP::iterator it = OpenRAVEXMLParser::GetRegisteredReaders()[PT_Sensor].find(att_Sensor->psensor->GetXMLId());
    if( it != OpenRAVEXMLParser::GetRegisteredReaders()[PT_Sensor].end() )
    {
      _pcurreader = it->second(att_Sensor->psensor, std::list<std::pair<std::string,std::string> >());
    }
    else
    {
      _pcurreader.reset();
    }

    RAVELOG_VERBOSE("XML (Sensor) Reader Initialized\n");

    //  Fill params from the COLLADA's file
    setSensorActuatorParams(dom_SensorActuatorManipulator);

    //  Close the actuator reader
    _pcurreader.reset();

    if( !att_Sensor->psensor->Init("") )
    {
      RAVELOG_INFOA("failed to initialize sensor %s\n", att_Sensor->GetName().c_str());
      att_Sensor->psensor.reset();
    }
    else
    {
      att_Sensor->pdata = att_Sensor->psensor->CreateSensorData();

      if( att_Sensor->pattachedlink.expired() ) {
        RAVELOG_DEBUGA("attached link is NULL, setting to base of robot\n");
        if( probot->GetLinks().size() == 0 )
        {
          RAVELOG_WARNA("robot has no links!\n");
        }
        else
        {
          size_t  pos;
          string  link_name;

          link_name  =   instance_link;
          pos        =   link_name.find_last_of("/");
          link_name  =   link_name.substr(pos + 1);

          //  TODO : Get link in which the sensor will be attached
          for (size_t i_link = 0; i_link < probot->GetLinks().size(); i_link++)
          {
            string robot_link   = probot->GetLinks()[i_link]->GetName();
            RAVELOG_DEBUGA("link_name: %s robot_link: %s\n",link_name.c_str(),robot_link.c_str());
            if (link_name == robot_link)
            {
              att_Sensor->pattachedlink = probot->GetLinks()[i_link];
              break;
            }
          }

          //att_sensor->pattachedlink = probot->GetLinks().front();
        }
      }

      //Relative Transform to sensors
      att_Sensor->trelative = getFullTransform(instance_SensorActuatorManipulator);
    }

    return true;
  }

  /// Fills Sensor and Actuator params from COLLADAS's file
  /// dom_SensorActuator  COLLADA sensor/actuator info
  bool setSensorActuatorParams(daeElementRef dom_SensorActuator)
  {
    daeTArray<daeElementRef> childrens = dom_SensorActuator->getChildren();

    //  For each feature of the Actuator send this INFO to Actuator's plugin
    for (size_t i = 0; i < childrens.getCount(); i++)
    {
      std::list<std::pair<std::string,std::string> >  atts;
      string xmltag = childrens[i]->getElementName();
      RAVELOG_DEBUG("(SensorActuator) %s: %s Type ID %d\n",childrens[i]->getElementName(),
          childrens[i]->getCharData().c_str(),
          childrens[i]->typeID());
      std::transform(xmltag.begin(), xmltag.end(), xmltag.begin(), ::tolower);
      _pcurreader->startElement(xmltag,atts);
      _pcurreader->characters(childrens[i]->getCharData());
      _pcurreader->endElement(xmltag);
    }

    return true;
  }

  /// Search the Link Name that is in the Robot structure
  char* getLinkName(const string& link)
  {
    char*       token       = NULL;
    char*       link_name   = NULL;
    char* line = new char[link.size()+1];

    strcpy(line,link.c_str());

    token   = strtok(line,"/");
    while (token != NULL)
    {
      link_name = token;
      token = strtok(NULL,"/");
    }

    delete line;
    return link_name;
  }

  bool Extract(RobotBasePtr& probot, domArticulated_systemRef partic) {
    if (!probot) {
      probot = _penv->CreateRobot(partic->getID());
    }
    return true;
  }

  bool Extract(RobotBasePtr& probot) {

    if (!probot) {
      probot = _penv->CreateRobot();
    }
    BOOST_ASSERT(probot->IsRobot());

    //  Debug
    RAVELOG_VERBOSEA("Executing Extract(RobotBasePtr&) !!!!!!!!!!!!!!!!!!\n");

    bool                    isRobot;        //  Distinguish between a Kinematic Model that is a Robot or not
    daeString               robotName = NULL;   //  Articulated System ID. Describes the Robot
    domKinematics_frameRef  pframe_origin = NULL;   //  Manipulator Base
    domKinematics_frameRef  pframe_tip      = NULL; //  Manipulator Effector

    domCOLLADA::domSceneRef allscene = _dom->getScene();
    BOOST_ASSERT(allscene != NULL);
    vector<domKinematics_newparam*> vnewparams;

    //  All bindings of the kinematics models presents in the scene
    vector<KINEMATICSBINDING> v_all_bindings;

    //  Nodes that are part of kinematics models
    vector<string>  processed_nodes;

    //  For each Kinematics Scene
    for (size_t iscene = 0; iscene < allscene->getInstance_kinematics_scene_array().getCount(); iscene++)
    {
      //  Gets the Scene
      domInstance_kinematics_sceneRef kiscene =
        allscene->getInstance_kinematics_scene_array()[iscene];

      domKinematics_sceneRef kscene = daeSafeCast<domKinematics_scene> (
          kiscene->getUrl().getElement().cast());

      // If there is not Kinematic Scene
      if (!kscene)
      {
        continue;
      }

      //  For each Bind of Kinematics Model
      for (size_t imodel = 0; imodel
      < kiscene->getBind_kinematics_model_array().getCount(); imodel++) {

        //  Kinimatics model may NOT be a Robot
        domArticulated_systemRef articulated_system =   NULL;

        //  Initially kinematics model is NOT a Robot
        isRobot = false;

        // Gets Bind of Kinematics Model
        domBind_kinematics_modelRef kbindmodel =
          kiscene->getBind_kinematics_model_array()[imodel];

        //  If there is no model
        if (kbindmodel->getNode() == NULL) {
          RAVELOG_WARNA(
              "do not support kinematics models without references to nodes\n");
          continue;
        }

        //  Debug
        RAVELOG_WARNA("Node: %s\n",kbindmodel->getNode());

        // Gets the Node of the Kinematics Model
        domNodeRef pnode = daeSafeCast<domNode> (daeSidRef(
            kbindmodel->getNode(), kbindmodel).resolve().elt);
        if (pnode == NULL || pnode->typeID() != domNode::ID()) {
          RAVELOG_WARNA(
              "bind_kinematics_model does not reference valid node %s\n",
              kbindmodel->getNode());
          continue;
        }
        else
        {
          RAVELOG_WARNA("Kinematics model node name: %s\n",pnode->getName());
        }

        //  Get ID of node parent
        string  parentnode  = daeSafeCast<domNode>(pnode->getParent())->getID();

        //  Store ID's of nodes that are part of kinematics model
        processed_nodes.push_back(parentnode);

        RAVELOG_VERBOSEA("Parent node ID %s\n",parentnode.c_str());

        //  Instance Kinematics Model
        domInstance_kinematics_modelRef kimodel;
        kimodel = getSidRef<domInstance_kinematics_model> (kbindmodel,kscene);
        if (kimodel == NULL)
        {
          RAVELOG_WARNA(
              "bind_kinematics_model does not reference valid kinematics\n");
        }
        else
        {
          //  TODO : Debug
          RAVELOG_DEBUGA("Instance Kinematics model %s\n",kimodel->getSid());
        }


        if (getElement()->typeID() == domArticulated_system::ID())
        {
          articulated_system = daeSafeCast<domArticulated_system>(getElement().cast());

          //  Debug
          RAVELOG_DEBUGA("Got articulated_system!!!\n");

          isRobot     = true;
          robotName   = articulated_system->getId();

          //  Check if there is a technique_common section
          if (!articulated_system->getKinematics()->getTechnique_common())
          {
            RAVELOG_VERBOSEA("Skip technique_common in articulated_system/kinematics\n");
          }
          else
          {
            RAVELOG_VERBOSEA("Kinematics Model is an Articulated System (Robot)\n");

            pframe_origin   = articulated_system->getKinematics()->getTechnique_common()->getFrame_origin();
            pframe_tip      = articulated_system->getKinematics()->getTechnique_common()->getFrame_tip();
          }
        }

        // Kinematics Model
        domKinematics_modelRef kmodel = daeSafeCast<domKinematics_model> (
            kimodel->getUrl().getElement().cast());
        if (!kmodel) {
          RAVELOG_WARNA(
              "bind_kinematics_model does not reference valid kinematics\n");
          return false;
        }

        // TODO : Get the joint binds
        vector<KINEMATICSBINDING> vbindings;
        for (size_t ijoint = 0; ijoint < kiscene->getBind_joint_axis_array().getCount(); ++ijoint) {

          int pos;

          domBind_joint_axisRef bindjoint =   kiscene->getBind_joint_axis_array()[ijoint];

          string  strNode     =   string(kbindmodel->getParam()->getValue());
          string  strTarget   =   string(bindjoint->getAxis()->getParam()->getValue());

          pos =   strTarget.find(strNode);

          //  Debug
          RAVELOG_DEBUGA("Kinematics Node %s Target %s Pos %d\n",strNode.c_str(),strTarget.c_str(),pos);

          if (pos == -1)
          {
            RAVELOG_VERBOSEA("This Axis NOT belongs to the Kinematics Model\n");
            continue;
          }

          domKinematics_axis_infoRef  dkai    = NULL;
          domMotion_axis_infoRef      dmai    = NULL;

          daeElementRef pjtarget =    daeSidRef(bindjoint->getTarget(), bindjoint).resolve().elt;

          if (pjtarget == NULL) {
            RAVELOG_ERRORA("Target Node %s NOT found!!!\n", bindjoint->getTarget());
            continue;
          }

          RAVELOG_WARNA("Target Node %s FOUND!!!\n", bindjoint->getTarget());

          //  Retrieve the joint
          domAxis_constraintRef pjointaxis = getSidRef<domAxis_constraint> (
              bindjoint->getAxis(),
              kscene);

          if (pjointaxis == NULL) {
            RAVELOG_ERRORA("Joint Axis %s NOT found\n",getRef());
            continue;
          }

          RAVELOG_WARNA("Joint Axis %s FOUND!!!\n",getRef());

          domInstance_kinematics_model_Array instance_kinematics_array;

          //  If the system is NOT a Robot
          if (getElement()->typeID() == domKinematics_scene::ID())
          {
            RAVELOG_WARNA("Kinematics Scene Element \n");
            domKinematics_sceneRef dks = daeSafeCast<domKinematics_scene>(getElement().cast());
            instance_kinematics_array = dks->getInstance_kinematics_model_array();
          }
          //  If the system is a Robot
          else if (getElement()->typeID() == domArticulated_system::ID())
          {
            RAVELOG_WARNA("Articulated System Element \n");
            //  Gets articulated system KINEMATICS
            //domArticulated_systemRef das = daeSafeCast<domArticulated_system>(getElement().cast());
            instance_kinematics_array = articulated_system->getKinematics()->getInstance_kinematics_model_array();

            // Search and fill Joint properties
            domKinematics_axis_info_Array dkai_array =  articulated_system->getKinematics()->getTechnique_common()->getAxis_info_array();

            string strAxis  =   string(getSidRef_value());

            //  Debug
            RAVELOG_VERBOSEA("SidRef value %s\n",strAxis.c_str());

            pos =   strAxis.find_last_of("/");
            pos =   strAxis.find_last_of("/",pos-1);

            strAxis =   kimodel->getUrl().fragment() + strAxis.substr(pos).c_str();

            RAVELOG_DEBUGA("Search axis string: %s\n",strAxis.c_str());

            //  Gets axis info of the articulated system KINEMATICS
            dkai = getAxisInfo<domKinematics_axis_info>(strAxis.c_str(),dkai_array);

            //  Check if there is a 'technique_common' section
            if (!!_motion_element->getTechnique_common())
            {
              domMotion_axis_info_Array dmai_array = _motion_element->getTechnique_common()->getAxis_info_array();

              RAVELOG_VERBOSEA("Check out articulated system ID\n");

              if (articulated_system->getID() == NULL)
              {
                RAVELOG_WARNA("ARTICULATED SYSTEM KINEMATICS Id is NULL ...\n");
              }

              RAVELOG_VERBOSEA("SidRef: %s\n",getSidRef_value());


              //                          //  Build the TAG for search axis info in the articulated system MOTION section. Dot format!!!
              //                          char axis_tag[256];
              //                          strcpy(axis_tag,articulated_system->getID());
              //                          strcat(axis_tag,"/");
              //                          strcat(axis_tag,dkai->getSid());

              strAxis =   string(articulated_system->getID()) + string("/") + string(dkai->getSid());

              //  Debug
              RAVELOG_WARNA("str: %s\n",strAxis.c_str());

              //  Gets axis info of the articulated system MOTION
              dmai    =   getAxisInfo<domMotion_axis_info>(strAxis.c_str(),dmai_array);

              //  Debug
              RAVELOG_WARNA("End of the search for AXIS INFO ...\n");
            }
          }

          domFloat fdefjointvalue = resolveFloat(bindjoint->getValue(),
              instance_kinematics_array);

          //  Stores joint info
          vbindings.push_back(KINEMATICSBINDING(pjtarget, pjointaxis,
              fdefjointvalue, dkai, dmai));

          //  Store joint info used for the visual scene
          v_all_bindings.push_back(KINEMATICSBINDING(pjtarget, pjointaxis,
              fdefjointvalue, dkai, dmai));
        }

        //  Obtain Kinmodel from COLLADA
        domPhysics_modelRef pmodel = NULL;
        KinBodyPtr pbody(probot);
        if (!Extract(pbody, kmodel, pmodel, pnode, vbindings)) {
          RAVELOG_WARNA("failed to load kinbody from kin instance %s\n",
              kimodel->getID());
          continue;
        }
        if( pbody != probot ) {
            BOOST_ASSERT(pbody->IsRobot());
            probot = boost::static_pointer_cast<RobotBase>(pbody);
        }

        if (isRobot)
        {
          //  Extract instances of sensors
          ExtractSensors<domArticulated_system>(articulated_system,probot);

          //  Debug
          RAVELOG_INFO(str(boost::format("Number of sensors of the Robot: %d\n")%probot->GetSensors().size()));

          //  Setup Manipulator of the Robot
          RobotBase::ManipulatorPtr manipulator(new RobotBase::Manipulator(probot));
          probot->GetManipulators().push_back(manipulator);

          //  Debug
          RAVELOG_WARNA(str(boost::format("Number of Manipulators %d\n")%probot->GetManipulators().size()));

          int     pos;
          string  linkName  = string(pframe_origin->getLink());

          pos       = linkName.find_first_of("/");
          linkName  = linkName.substr(pos + 1);

          RAVELOG_VERBOSEA(str(boost::format("Manipulator link name %s\n")%linkName));

          //  Sets frame_origin and frame_tip
          manipulator->_pBase              = probot->GetLink(linkName);

          if (!!manipulator->_pBase)
          {
              RAVELOG_WARNA(str(boost::format("Manipulator::pBase ... %s\n")%manipulator->_pBase->GetName()));
          }
          else
          {
            RAVELOG_WARNA("Error initializing Manipulator::pBase\n");
          }

          linkName = string(pframe_tip->getLink());
          pos       = linkName.find_first_of("/");
          linkName  = linkName.substr(pos + 1);

          manipulator->_pEndEffector   = probot->GetLink(linkName);

          if (!!manipulator->_pEndEffector)
          {
              RAVELOG_WARNA(str(boost::format("Manipulator::pEndEffector ... %s\n")%manipulator->_pEndEffector->GetName()));
          }
          else
          {
            RAVELOG_WARNA("Error initializing Manipulator::pEndEffector\n");
          }

          //  Initialize indices that then manipulator controls
          for (size_t i   =   0;  i < probot->GetJointIndices().size();   i++)
          {
            manipulator->_vgripperjoints.push_back(probot->GetJointIndices()[i]);
          }

          RAVELOG_VERBOSEA("Indices initialized...\n");
        }
        
        RAVELOG_WARNA("Robot %s created ...\n",robotName);
      }// End Kinematics model Process

    }// End Instance Kinematics scene Process

    return true;
  }

  bool Extract(KinBodyPtr& ppbody) {
    RAVELOG_ERROR("extract(kinbodyptr) dummy function\n");
    return true;
  }

  /// Create an openrave body
  bool Extract(KinBodyPtr& pkinbody, domKinematics_modelRef kmodel,
      domPhysics_modelRef pmodel, domNodeRef pnode, const vector<KINEMATICSBINDING>& vbindings)
  {
    vector<domJointRef> vdomjoints;

    // If there is NO kinbody create one
    if (!pkinbody)
    {
      //  Debug
      RAVELOG_WARNA("Create a KINBODY......................\n");
      pkinbody = _penv->CreateKinBody();
    }

    //  If there is a Kinematic Model
    if (kmodel != NULL)
    {
      pkinbody->SetName(kmodel->getName());

      if (pkinbody->GetName() == "")
      {
        pkinbody->SetName(string(pnode->getId()));
      }

      //  Debug.
      RAVELOG_WARNA("KinBody Name: %s\n", pkinbody->GetName().c_str());
      RAVELOG_VERBOSEA("Kinbody node: %s\n",pnode->getId());

      //  Process joint of the kinbody
      domKinematics_model_techniqueRef ktec = kmodel->getTechnique_common();

      //  Store joints
      for (size_t ijoint = 0; ijoint < ktec->getJoint_array().getCount(); ++ijoint)
      {
        vdomjoints.push_back(ktec->getJoint_array()[ijoint]);
      }

      //  Store instances of joints
      for (size_t ijoint = 0; ijoint < ktec->getInstance_joint_array().getCount(); ++ijoint)
      {
        domJointRef pelt = daeSafeCast<domJoint> (
            ktec->getInstance_joint_array()[ijoint]->getUrl().getElement());

        if (!pelt)
        {
          RAVELOG_WARNA("failed to get joint from instance\n");
        }
        else
        {
          vdomjoints.push_back(pelt);
        }
      }

      //  Debug
      RAVELOG_VERBOSEA("Number of links in the kmodel %d\n",ktec->getLink_array().getCount());

//      //  Gets parent node.
      domNodeRef  parent  = daeSafeCast<domNode>(pnode->getParent());

      // Stefan: This is not a safe cast - predecessor might be a transformation
//      domNodeRef  parent  = daeSafeCast<domNode>(pnode->getAncestor(NodeMatcher()));
      // ---

      //  Extract all links into the kinematics_model
      for (size_t ilink = 0; ilink < ktec->getLink_array().getCount(); ++ilink)
      {
        domLinkRef plink = ktec->getLink_array()[ilink];

        domNodeRef  child = daeSafeCast<domNode>(parent->getChildren()[ilink]);

        //  Stefan change
//        domNodeRef  child = daeSafeCast<domNode>(parent->getChildrenByType<domNode>()[ilink]);
        // ---


        //  Debug
        RAVELOG_VERBOSEA("Node Name out of ExtractLink: %s\n",child->getName());

        ExtractLink(pkinbody, plink, child, Transform(), vdomjoints, vbindings);

        //  Stefan change
//        ExtractLink(pkinbody, plink, child, getNodeParentTransform(child), vdomjoints, vbindings);
        // ---


        //        //  Debug
        //        RAVELOG_VERBOSEA("Node Name out of ExtractLink: %s\n",pnode->getName());
        //
        //        ExtractLink(pkinbody, plink, pnode, Transform(), vdomjoints, vbindings);
      }

      //  TODO : Extract FORMULAS of the kinematics_model
      for (size_t iform = 0; iform < ktec->getFormula_array().getCount(); ++iform)
      {
        domFormulaRef pf = ktec->getFormula_array()[iform];
        if (!pf->getTarget()) {
            RAVELOG_WARNA("formula target not valid\n");
            continue;
        }

        // find the target joint
        xsToken target = pf->getTarget()->getParam()->getValue();
        KinBody::JointPtr pjoint = _getJointFromRef(target,pf,pkinbody);
        if (!pjoint) {
            continue;
        }
        
        if (!!pf->getTechnique_common()) {
          daeElementRef peltmath;
          for (size_t ichild = 0; ichild < pf->getTechnique_common()->getChildren().getCount(); ++ichild)
          {
            daeElementRef pelt = pf->getTechnique_common()->getChildren()[ichild];
            if (pelt->getElementName() == string("math") || pelt->getElementName() == string("math:math"))
                peltmath = pelt;
            else
                RAVELOG_WARNA(str(boost::format("unsupported formula element: %s\n")%pelt->getElementName()));
          }
          if (!!peltmath) {
            // full math xml spec not supported, only looking for ax+b pattern:
            // <apply> <plus/> <apply> <times/> <ci>a</ci> x </apply> <ci>b</ci> </apply>
            dReal a = 1, b = 0;
            daeElementRef psymboljoint;
            try {
                BOOST_ASSERT(peltmath->getChildren().getCount()>0);
                daeElementRef papplyelt = peltmath->getChildren()[0];
                BOOST_ASSERT(_checkMathML(papplyelt,"apply"));
                BOOST_ASSERT(papplyelt->getChildren().getCount()>0);
                if( _checkMathML(papplyelt->getChildren()[0],"plus") ) {
                    BOOST_ASSERT(papplyelt->getChildren().getCount()==3);
                    daeElementRef pa = papplyelt->getChildren()[1];
                    daeElementRef pb = papplyelt->getChildren()[2];
                    if( !_checkMathML(papplyelt->getChildren()[1],"apply") ) {
                        swap(pa,pb);
                    }
                    if( !_checkMathML(pa,"csymbol") ) {
                        BOOST_ASSERT(_checkMathML(pa,"apply"));
                        BOOST_ASSERT(_checkMathML(pa->getChildren()[0],"times"));
                        if( _checkMathML(pa->getChildren()[1],"csymbol") ) {
                            psymboljoint = pa->getChildren()[1];
                            BOOST_ASSERT(_checkMathML(pa->getChildren()[2],"cn"));
                            stringstream ss(pa->getChildren()[2]->getCharData());
                            ss >> a;
                        }
                        else {
                            psymboljoint = pa->getChildren()[2];
                            BOOST_ASSERT(_checkMathML(pa->getChildren()[1],"cn"));
                            stringstream ss(pa->getChildren()[1]->getCharData());
                            ss >> a;
                        }
                    }
                    else {
                        psymboljoint = pa;
                    }
                    BOOST_ASSERT(_checkMathML(pb,"cn"));
                    {
                        stringstream ss(pb->getCharData());
                        ss >> b;
                    }
                }
                else if( _checkMathML(papplyelt->getChildren()[0],"minus") ) {
                    BOOST_ASSERT(_checkMathML(papplyelt->getChildren()[1],"csymbol"));
                    a = -1;
                    psymboljoint = papplyelt->getChildren()[1];
                }

                BOOST_ASSERT(psymboljoint->hasAttribute("encoding"));
                BOOST_ASSERT(psymboljoint->getAttribute("encoding")==string("COLLADA"));
                KinBody::JointPtr pbasejoint = _getJointFromRef(psymboljoint->getCharData().c_str(),pf,pkinbody);
                if( !!pbasejoint ) {
                    // set the mimic properties
                    pjoint->nMimicJointIndex = pbasejoint->GetJointIndex();
                    pjoint->vMimicCoeffs.resize(2);
                    pjoint->vMimicCoeffs[0] = a;
                    pjoint->vMimicCoeffs[1] = b;
                    RAVELOG_DEBUG(str(boost::format("assigning joint %s to mimic %s(%d) %f %f\n")%pjoint->GetName()%pbasejoint->GetName()%pjoint->nMimicJointIndex%a%b));
                }
            }
            catch(const openrave_exception& ex) {
                RAVELOG_WARN(str(boost::format("exception occured when parsing formula for target joint %s: %s\n")%pjoint->GetName()%ex.what()));
            }
          }
        }
      }
    }
    //  This Maybe is a NO Kinematics object
    else
    {
      ExtractLink(pkinbody, NULL, pnode, Transform(), vdomjoints, vbindings);
    }
    //pkinbody->_vForcedAdjacentLinks.push_back(entry);
    return true;
  }

  //  Extract Link info and adds it to OpenRAVE
  KinBody::LinkPtr  ExtractLink(KinBodyPtr pkinbody, const domLinkRef pdomlink,
      const domNodeRef pdomnode, Transform tParentLink, const vector<
      domJointRef>& vdomjoints, const vector<KINEMATICSBINDING>& vbindings) {

    //  Initially the link has the name of the node
    string linkname;
    if (pdomnode->getName() == NULL)
    {
      linkname = pdomnode->getId();
    }
    else
    {
      linkname = pdomnode->getName();
    }

    //  Set link name with the name of the COLLADA's Link
    if (!!pdomlink && pdomlink->getName())
    {
      linkname = pdomlink->getName();
    }

    KinBody::LinkPtr plink = pkinbody->GetLink(linkname);
    if( !plink ) {
        plink.reset(new KinBody::Link(pkinbody));
        plink->name = linkname;

    //  Initialize Link Mass
    plink->_mass    =   1.0;

    plink->bStatic  = false;
    plink->index    = (int) pkinbody->_veclinks.size();
        pkinbody->_veclinks.push_back(plink);
    }

    if (pkinbody->GetName() == "")
    {
      //  Sets name of kinbody
      pkinbody->SetName(string(pdomnode->getName()));
    }

    RAVELOG_VERBOSEA("Node Id %s and Name %s\n", pdomnode->getId(), pdomnode->getName());

    if (!pdomlink)
    {
      //  Debug
      RAVELOG_INFO("Extract object NOT kinematics !!!\n");

      // Get the geometry
      ExtractGeometry(pdomnode,plink,vbindings);
    }
    else
    {
      //  Debug
      RAVELOG_VERBOSEA("ExtractLink !!!\n");

      //  Debug
      RAVELOG_DEBUGA("Attachment link elements: %d\n",pdomlink->getAttachment_full_array().getCount());

      Transform tlink = getFullTransform(pdomlink);
      plink->_t = tParentLink * tlink; // use the kinematics coordinate system for each link

      // Get the geometry
      ExtractGeometry(pdomnode,plink,vbindings);

      //  Debug
      RAVELOG_DEBUGA("After ExtractGeometry Attachment link elements: %d\n",pdomlink->getAttachment_full_array().getCount());

      //  Process all atached links
      for (size_t iatt = 0; iatt < pdomlink->getAttachment_full_array().getCount(); ++iatt) {
        domLink::domAttachment_fullRef pattfull =
          pdomlink->getAttachment_full_array()[iatt];

        // get link kinematics transformation
        TransformMatrix tatt = getFullTransform(pattfull);

        // process attached links
        daeElement* peltjoint =
          daeSidRef(pattfull->getJoint(), pattfull).resolve().elt;
        domJointRef pdomjoint = daeSafeCast<domJoint> (peltjoint);

        if (!pdomjoint) {
          domInstance_jointRef pdomijoint = daeSafeCast<domInstance_joint> (
              peltjoint);
          if (!!pdomijoint)
            pdomjoint = daeSafeCast<domJoint> (
                pdomijoint->getUrl().getElement().cast());
        }

        if (!pdomjoint || pdomjoint->typeID() != domJoint::ID()) {
          RAVELOG_WARNA("could not find attached joint %s!\n",
              pattfull->getJoint());
          return KinBody::LinkPtr();
        }

        // get direct child link
        if (!pattfull->getLink()) {
          RAVELOG_WARNA("joint %s needs to be attached to a valid link\n",
              pdomjoint->getID());
          continue;
        }

        // find the correct node in vbindings
        daeTArray<domAxis_constraintRef>  vdomaxes            = pdomjoint->getChildrenByType<domAxis_constraint> ();
        domNodeRef                        pchildnode          = NULL;
        daeElementRef                     paxisnode           = NULL;
        domKinematics_axis_infoRef        pkinematicaxisinfo  = NULL;
        domMotion_axis_infoRef            pmotionaxisinfo     = NULL;

        for (size_t ibind = 0; ibind < vbindings.size() && !pchildnode
        && !paxisnode; ++ibind)
        {
          domAxis_constraintRef   paxisfound  = NULL;
          for (size_t ic = 0; ic < vdomaxes.getCount(); ++ic)
          {
            //  If the binding for the joint axis is found, retrieve the info
            if (vdomaxes[ic] == vbindings[ibind].pkinematicaxis)
            {
              pchildnode          = vbindings[ibind].pvisualnode;
              paxisnode           = vbindings[ibind].pvisualtrans;
              pkinematicaxisinfo  = vbindings[ibind].pkinematicaxisinfo;
              pmotionaxisinfo     = vbindings[ibind].pmotionaxisinfo;
              break;
            }
          }
        }

        if (!pchildnode) {
          RAVELOG_ERROR("failed to find associating node for joint %s\n",
              pdomjoint->getID());
          continue;
        }

        // create the joints before creating the child links
        vector<KinBody::JointPtr> vjoints(vdomaxes.getCount());
        RAVELOG_WARN("vdomaxes.getCount: %d\n",vdomaxes.getCount());

        for (size_t ic = 0; ic < vdomaxes.getCount(); ++ic) {
          KinBody::JointPtr pjoint(new KinBody::Joint(pkinbody));
          pjoint->bodies[0] = plink;
          pjoint->bodies[1].reset();
          pjoint->name = pdomjoint->getName();
          pjoint->jointindex = (int) pkinbody->_vecjoints.size();

          //  Set type of the joint
          domAxis_constraintRef pdomaxis = vdomaxes[ic];

          if( strcmp(pdomaxis->getElementName(), "revolute") == 0 )
          {
            RAVELOG_WARNA("Revolute joint\n");
            pjoint->type = KinBody::Joint::JointRevolute;
          }
          else if( strcmp(pdomaxis->getElementName(), "prismatic") == 0 )
          {
            RAVELOG_WARNA("Prismatic joint\n");
            pjoint->type = KinBody::Joint::JointPrismatic;
          }

          RAVELOG_INFO("dofindex %d\n",pkinbody->GetDOF());
          pjoint->dofindex = pkinbody->GetDOF();
          RAVELOG_INFO("pjoint->dofindex %d\n",pjoint->dofindex);

          pjoint->_vweights.resize(pjoint->GetDOF());
          FOREACH(it,pjoint->_vweights) {
              *it = 1;
          }
          pkinbody->_vecJointIndices.push_back(pjoint->dofindex);
          pkinbody->_vecjoints.push_back(pjoint);
          RAVELOG_WARN("Number of pkinbody->_vecjoints: %d\n",pkinbody->_vecjoints.size());
          vjoints[ic] = pjoint;
        }

        KinBody::LinkPtr pchildlink = ExtractLink(pkinbody, pattfull->getLink(),
            pchildnode, plink->_t * tatt, vdomjoints, vbindings);

        if (pchildlink == NULL) {
          //  Debug
          RAVELOG_WARNA("Link NULL: %s \n", plink->GetName().c_str());

          continue;
        }

        int numjoints = 0;
        for (size_t ic = 0; ic < vdomaxes.getCount(); ++ic) {
          domAxis_constraintRef pdomaxis = vdomaxes[ic];

          if (!pchildlink) {
            // create dummy child link
            // multiple axes can be easily done with "empty links"
            RAVELOG_WARNA(
                "openrave does not support collada joints with > 1 degrees: \n");

            //      Debug.
            RAVELOG_WARNA("Link: %s Num joints %d\n", plink->GetName().c_str(), numjoints);

            stringstream ss;
            ss << plink->name;
            ss <<"_dummy" << numjoints;
            pchildlink.reset(new KinBody::Link(pkinbody));
            pchildlink->name = ss.str().c_str();
            pchildlink->bStatic = false;
            pchildlink->index = (int)pkinbody->_veclinks.size();
            pkinbody->_veclinks.push_back(pchildlink);
          }

          RAVELOG_WARNA("Joint assigned %d \n",ic);

          KinBody::JointPtr pjoint = vjoints[ic];
          pjoint->bodies[1] = pchildlink;

          //          //  Set type of the joint
          //          if( strcmp(pdomaxis->getElementName(), "revolute") == 0 )
          //          {
          //            RAVELOG_WARNA("Revolute joint\n");
          //            pjoint->type = KinBody::Joint::JointRevolute;
          //          }
          //          else if( strcmp(pdomaxis->getElementName(), "prismatic") == 0 )
          //          {
          //            RAVELOG_WARNA("Prismatic joint\n");
          //            pjoint->type = KinBody::Joint::JointPrismatic;
          //          }

          //  Axes and Anchor assignment.
          pjoint->vAxes[0] = Vector(-pdomaxis->getAxis()->getValue()[0], -pdomaxis->getAxis()->getValue()[1], -pdomaxis->getAxis()->getValue()[2]).normalize3();
          pjoint->vanchor = Vector(0,0,0);

          int numbad = 0;
          pjoint->offset = 0; // to overcome -pi to pi boundary

          if (!pmotionaxisinfo)
          {
            RAVELOG_ERRORA(str(boost::format("Not Exists Motion axis info, joint %s\n")%pjoint->GetName()));
          }

          //  Sets the Speed and the Acceleration of the joint
          if (pmotionaxisinfo != NULL)
          {
            if (pmotionaxisinfo->getSpeed() != NULL)
            {
              pjoint->fMaxVel = pmotionaxisinfo->getSpeed()->getFloat()->getValue();

              //  Debug
              RAVELOG_VERBOSEA("... Joint Speed: %f...\n",pjoint->GetMaxVel());
            }
            if (pmotionaxisinfo->getAcceleration())
            {
              pjoint->fMaxAccel = pmotionaxisinfo->getAcceleration()->getFloat()->getValue();

              //  Debug
              RAVELOG_VERBOSEA("... Joint Acceleration: %f...\n",pjoint->GetMaxAccel());
            }
          }

          //  If the joint is locked or NOT
          bool    joint_locked            = false;
          bool    kinematics_limits   =    false;

          //  If there is NO kinematicaxisinfo
          if (!!pkinematicaxisinfo)
          {
            //  Debug
            RAVELOG_VERBOSEA("Axis_info Start...\n");

            if (!pkinematicaxisinfo->getActive())
            {
              //  Debug
              RAVELOG_WARNA("Joint Disable...\n");
              pjoint->GetParent()->Enable(false);
            }

            if (!!pkinematicaxisinfo->getLocked())
            {
              if (pkinematicaxisinfo->getLocked()->getBool()->getValue())
              {
                //  Debug
                RAVELOG_WARNA("Joint Locked...\n");
                joint_locked = true;
              }
            }

            // If joint is locked set limits to 0. There is NO move
            if (joint_locked)
            {
              if( pjoint->type == KinBody::Joint::JointRevolute
                  ||
                  pjoint->type ==KinBody::Joint::JointPrismatic)
              {
                pjoint->_vlowerlimit.push_back(0.0f);
                pjoint->_vupperlimit.push_back(0.0f);
              }
            }
            // If there are articulated system kinematics limits
            else if (pkinematicaxisinfo->getLimits())
            {
              //  There is a kinematics limits
              kinematics_limits   = true;

              RAVELOG_WARNA("Articulated System Joint Limit Min: %f, Max: %f\n",
                  pkinematicaxisinfo->getLimits()->getMin()->getFloat()->getValue(),
                  pkinematicaxisinfo->getLimits()->getMax()->getFloat()->getValue());

              if( pjoint->type == KinBody::Joint::JointRevolute
                  ||
                  pjoint->type ==KinBody::Joint::JointPrismatic)
              {
                pjoint->_vlowerlimit.push_back((dReal)(pkinematicaxisinfo->getLimits()->getMin()->getFloat()->getValue()));
                pjoint->_vupperlimit.push_back((dReal)(pkinematicaxisinfo->getLimits()->getMax()->getFloat()->getValue()));
              }
            }
          }

          //  Search limits in the joints section
          if ((!joint_locked && !kinematics_limits) || !pkinematicaxisinfo)
          {
            RAVELOG_WARNA("Degrees of Freedom: %d\n",pjoint->GetDOF());

            for(int i = 0; i < pjoint->GetDOF(); ++i)
            {
              //  Debug
              RAVELOG_WARNA("DOF Number %d...\n",i);

              //  If there are NOT LIMITS
              if( !pdomaxis->getLimits() )
              {
                //  Debug
                RAVELOG_WARNA("There are NO LIMITS in the joint ...\n");
                if( pjoint->type == KinBody::Joint::JointRevolute )
                {
                  pjoint->_bIsCircular = true;
                  pjoint->_vlowerlimit.push_back(-PI);
                  pjoint->_vupperlimit.push_back(PI);
                }
                else
                {
                  pjoint->_vlowerlimit.push_back(-100000);
                  pjoint->_vupperlimit.push_back(100000);
                }
              }
              //  If there are LIMITS
              else
              {
                //  Debug
                RAVELOG_WARNA("There are LIMITS in the joint ...\n");

                dReal fscale = (pjoint->type == KinBody::Joint::JointRevolute)?(PI/180.0f):1.0f;

                //  Debug
                RAVELOG_WARNA("Joint Limits ...\n");

                pjoint->_vlowerlimit.push_back(pdomaxis->getLimits()->getMin()->getValue()*fscale);
                pjoint->_vupperlimit.push_back(pdomaxis->getLimits()->getMax()->getValue()*fscale);

                //  Debug
                RAVELOG_WARNA("Joint offset ...\n");

                if( pjoint->type == KinBody::Joint::JointRevolute )
                {
                  if( pjoint->_vlowerlimit.back() < -PI || pjoint->_vupperlimit.back()> PI )
                  {
                    pjoint->offset += 0.5f * (pjoint->_vlowerlimit[i] + pjoint->_vupperlimit[i]);
                    ++numbad;
                  }
                }
              }
            }
          }

          if( numbad> 0 )
          {
            pjoint->offset *= 1.0f / (dReal)numbad;
            RAVELOG_VERBOSEA("joint %s offset is %f\n", pjoint->GetName().c_str(), (float)pjoint->offset);
          }

          // Transform applied to the joint
          Transform tbody0, tbody1;
          tbody0 = pjoint->bodies[0]->GetTransform();
          tbody1 = pjoint->bodies[1]->GetTransform();
          Transform trel;
          trel = tbody0.inverse() * tbody1;

          Transform toffsetfrom;

          if (!!pchildnode)
          {
            RAVELOG_INFO("Applies Transform Offset From Parent Link!!!\n");
            toffsetfrom = plink->_t * tatt;
            toffsetfrom = tbody0.inverse() * toffsetfrom;
          }

          pjoint->vanchor = toffsetfrom * pjoint->vanchor;

          //  Debug.
          RAVELOG_DEBUG("Node %s offset is %f\n", pdomnode->getName(), (float)pjoint->offset);
          RAVELOG_DEBUG("OffsetFrom Translation trans.x:%f, trans.y:%f, trans.z:%f\n",toffsetfrom.trans.x,toffsetfrom.trans.y,toffsetfrom.trans.z);
          RAVELOG_DEBUG("OffsetFrom Rotation rot.x:%f, rot.y:%f, rot.z:%f, rot.w:%f\n",toffsetfrom.rot.x,toffsetfrom.rot.y,toffsetfrom.rot.z,toffsetfrom.rot.w);
          RAVELOG_DEBUG("vAnchor (%s) x:%f, y:%f, z:%f\n",pjoint->GetName().c_str(),pjoint->vanchor.x,pjoint->vanchor.y,pjoint->vanchor.z);
          RAVELOG_DEBUG("vAxes x:%f, y:%f, z:%f\n",pjoint->vAxes[0].x,pjoint->vAxes[0].y,pjoint->vAxes[0].z);

          //  Rotate axis from the parent offset
          for(int i = 0; i < pjoint->GetDOF(); ++i)
          {
              pjoint->vAxes[i] = toffsetfrom.rotate(pjoint->vAxes[i]);
          }

          if( pjoint->type == KinBody::Joint::JointRevolute )
          {
            pjoint->tLeft.rotfromaxisangle(pjoint->vAxes[0], -pjoint->offset);
          }

          pjoint->fMaxVel = pjoint->GetType() == KinBody::Joint::JointPrismatic ? 0.013 : 0.5f;
          
          pjoint->tLeft.trans   = pjoint->vanchor;
          pjoint->tRight.trans  = -pjoint->vanchor;
          pjoint->tRight = pjoint->tRight * trel;

          pjoint->tinvLeft = pjoint->tLeft.inverse();
          pjoint->tinvRight = pjoint->tRight.inverse();

          // mimic joints
          //pjoint->nMimicJointIndex = -1;
          //pjoint->fMimicCoeffs[0] = 1; pjoint->fMimicCoeffs[1] = 0;
          //            _pchain->_vecPassiveJoints.push_back(pnewjoint);
          // physics, control?
          //                pjoint->fMaxVel;
          //                pjoint->fMaxAccel;
          //                pjoint->fMaxTorque;
          //                pjoint->fResolution;

          pchildlink.reset();
          ++numjoints;
        }
      }

    }
    //pdomlink->getAttachment_start_array();
    //pdomlink->getAttachment_end_array();

    return plink;
  }

  /// Extract Geometry and apply the transformations of the node
  /// \param pdomnode Node to extract the goemetry
  /// \param plink    Link of the kinematics model
  void ExtractGeometry(const domNodeRef pdomnode,KinBody::LinkPtr plink, const vector<KINEMATICSBINDING> &vbindings)
  {
    //  Debug
    RAVELOG_WARNA("ExtractGeometry(node,link) of %s\n",pdomnode->getName());


    for (size_t i = 0; i < pdomnode->getNode_array().getCount(); i++)
    {
      RAVELOG_VERBOSEA("[stef] (%d/%d) Children %s (%s)\n",i,pdomnode->getNode_array().getCount(),pdomnode->getNode_array()[i]->getID(), pdomnode->getID());
    }

    // For all child nodes of pdomnode
    for (size_t i = 0; i < pdomnode->getNode_array().getCount(); i++)
    {
      RAVELOG_VERBOSEA("[stef]  %s: Process Children Children %s (%d/%d) \n",pdomnode->getID(),pdomnode->getNode_array()[i]->getID(),i,pdomnode->getNode_array().getCount());
      // check if contains a joint
      bool contains=false;
      for (vector<KINEMATICSBINDING>::const_iterator it=vbindings.begin(); it!= vbindings.end();it++){
        RAVELOG_VERBOSEA("[stef] child node '%s' ?=  link node '%s'",pdomnode->getNode_array()[i]->getID(), it->pvisualnode->getID());

        // don't check ID's check if the reference is the same!
        //if (string(pdomnode->getNode_array()[i]->getID()).compare(string(it->pvisualnode->getID()))==0){
        if ( (pdomnode->getNode_array()[i])  == (it->pvisualnode)){
          //domNode *pv = *(it->pvisualnode);
          //domNode *child = *(pdomnode->getNode_array()[i]);
          //if ( (child) == pv){
          contains=true;
          RAVELOG_VERBOSEA(" yes\n");
          break;
        }
        RAVELOG_VERBOSEA(" no\n");
      }
      if (contains) continue;

      RAVELOG_VERBOSEA("[stef] Process child node: %s\n", pdomnode->getNode_array()[i]->getID());

      ExtractGeometry(pdomnode->getNode_array()[i],plink, vbindings);
      // Plink stayes the same for all children
      // replace pdomnode by child = pdomnode->getNode_array()[i]
      // hope for the best!
      // put everything in a subroutine in order to process pdomnode too!
    }


    unsigned int nGeomBefore =  plink->_listGeomProperties.size(); // #of Meshes already associated to this link

    // get the geometry
    for (size_t igeom = 0; igeom
    < pdomnode->getInstance_geometry_array().getCount(); ++igeom) {

      domInstance_geometryRef domigeom =
        pdomnode->getInstance_geometry_array()[igeom];

      domGeometryRef domgeom = daeSafeCast<domGeometry> (
          domigeom->getUrl().getElement());

      if (!domgeom) {
        RAVELOG_WARNA("link %s does not have valid geometry\n",
            plink->GetName().c_str());
        continue;
      }

      //  Gets materials
      map<string, domMaterialRef> mapmaterials;
      if (!!domigeom->getBind_material()
          && !!domigeom->getBind_material()->getTechnique_common()) {
        const domInstance_material_Array
        & matarray =
          domigeom->getBind_material()->getTechnique_common()->getInstance_material_array();
        for (size_t imat = 0; imat < matarray.getCount(); ++imat) {
          domMaterialRef pmat = daeSafeCast<domMaterial> (
              matarray[imat]->getTarget().getElement());
          if (!!pmat)
            mapmaterials[matarray[imat]->getSymbol()] = pmat;
        }
      }

      //  Gets the geometry
      ExtractGeometry(domgeom, mapmaterials, plink);
    }
    TransformMatrix tmnodegeom = (TransformMatrix) plink->_t.inverse()
    * getNodeParentTransform(pdomnode) * getFullTransform(pdomnode);
    Transform tnodegeom;
    Vector vscale;
    decompose(tmnodegeom, tnodegeom, vscale);


    list<KinBody::Link::GEOMPROPERTIES>::iterator itgeom= plink->_listGeomProperties.begin();
    for (unsigned int i=0; i< nGeomBefore; i++) itgeom++; // change only the transformations of the newly found geometries.

    //  Switch between different type of geometry PRIMITIVES
    for (; itgeom != plink->_listGeomProperties.end(); itgeom++)
    {
      itgeom->_t = tnodegeom;
      switch (itgeom->GetType()) {
      case KinBody::Link::GEOMPROPERTIES::GeomBox:
        itgeom->vGeomData *= vscale;
        break;
      case KinBody::Link::GEOMPROPERTIES::GeomSphere:
        itgeom->vGeomData *= max(vscale.z, max(vscale.x, vscale.y));
        break;
      case KinBody::Link::GEOMPROPERTIES::GeomCylinder:
        itgeom->vGeomData.x *= max(vscale.x, vscale.y);
        itgeom->vGeomData.y *= vscale.z;
        break;
      case KinBody::Link::GEOMPROPERTIES::GeomTrimesh:
        itgeom->collisionmesh.ApplyTransform(tmnodegeom);
        itgeom->_t = Transform(); // reset back to identity
        break;
      default:
        RAVELOG_WARNA("unknown geometry type: %d\n", itgeom->GetType());
      }

      //  Gets collision mesh
      KinBody::Link::TRIMESH trimesh = itgeom->GetCollisionMesh();
      trimesh.ApplyTransform(itgeom->_t);
      plink->collision.Append(trimesh);
    }
    //      RAVELOG_VERBOSEA("End Extract Geometry (%s)\n",pdomnode->getID());
  }

  /// Paint the Geometry with the color material
  /// \param  pmat    Material info of the COLLADA's model
  /// \param  geom    Geometry properties in OpenRAVE
  void FillGeometryColor(const domMaterialRef pmat, KinBody::Link::GEOMPROPERTIES& geom)
  {
    if( !!pmat && !!pmat->getInstance_effect() )
    {
      domEffectRef peffect = daeSafeCast<domEffect>(pmat->getInstance_effect()->getUrl().getElement().cast());
      if( !!peffect )
      {
        domProfile_common::domTechnique::domPhongRef pphong = daeSafeCast<domProfile_common::domTechnique::domPhong>(peffect->getDescendant(daeElement::matchType(domProfile_common::domTechnique::domPhong::ID())));
        if( !!pphong )
        {

          if( !!pphong->getAmbient() && !!pphong->getAmbient()->getColor() )
          {
            //                      RAVELOG_VERBOSEA("Set ambient color ...\n");
            geom.ambientColor = getVector4(pphong->getAmbient()->getColor()->getValue());
          }
          if( !!pphong->getDiffuse() && !!pphong->getDiffuse()->getColor() )
          {
            //                      RAVELOG_VERBOSEA("Set diffuse color ...\n");
            geom.diffuseColor = getVector4(pphong->getDiffuse()->getColor()->getValue());
          }
        }
      }
    }
  }

  /// Extract the Geometry in TRIANGLES and adds it to OpenRave
  /// \param  triRef  Array of triangles of the COLLADA's model
  /// \param  vertsRef    Array of vertices of the COLLADA's model
  /// \param  mapmaterials    Materials applied to the geometry
  /// \param  plink   Link of the kinematics model
  bool ExtractGeometry(const domTrianglesRef triRef, const domVerticesRef vertsRef, const map<string,domMaterialRef>& mapmaterials, KinBody::LinkPtr plink)
  {
    //  Debug
    RAVELOG_VERBOSEA("ExtractGeometry in TRIANGLES and adds to OpenRAVE............\n");
    if( triRef == NULL )
      return false;

    plink->_listGeomProperties.push_back(KinBody::Link::GEOMPROPERTIES(plink));
    KinBody::Link::GEOMPROPERTIES& geom = plink->_listGeomProperties.back();
    KinBody::Link::TRIMESH& trimesh = geom.collisionmesh;
    geom.type = KinBody::Link::GEOMPROPERTIES::GeomTrimesh;

    // resolve the material and assign correct colors to the geometry
    map<string,domMaterialRef>::const_iterator itmat = mapmaterials.find(triRef->getMaterial());
    if( itmat != mapmaterials.end() )
      FillGeometryColor(itmat->second,geom);

    int triangleIndexStride = 0;
    int vertexoffset = -1;
    domInput_local_offsetRef indexOffsetRef;

    for (unsigned int w=0;w<triRef->getInput_array().getCount();w++)
    {
      int offset = triRef->getInput_array()[w]->getOffset();
      daeString str = triRef->getInput_array()[w]->getSemantic();
      if (!strcmp(str,"VERTEX"))
      {
        indexOffsetRef = triRef->getInput_array()[w];
        vertexoffset = offset;
      }
      if (offset> triangleIndexStride)
      {
        triangleIndexStride = offset;
      }
    }

    triangleIndexStride++;
    const domList_of_uints& indexArray =triRef->getP()->getValue();

    for (size_t i=0;i<vertsRef->getInput_array().getCount();++i)
    {
      domInput_localRef localRef = vertsRef->getInput_array()[i];
      daeString str = localRef->getSemantic();
      if ( strcmp(str,"POSITION") == 0 )
      {
        const domSourceRef node = daeSafeCast<domSource>(localRef->getSource().getElement());
        if( !node )
          continue;
        dReal fUnitScale = GetUnitScale(node);
        const domFloat_arrayRef flArray = node->getFloat_array();
        if (!!flArray)
        {
          const domList_of_floats& listFloats = flArray->getValue();
          int k=vertexoffset;
          int vertexStride = 3;//instead of hardcoded stride, should use the 'accessor'

          if( trimesh.indices.capacity() < trimesh.indices.size()+triRef->getCount() )
            trimesh.indices.reserve(trimesh.indices.size()+triRef->getCount());
          if( trimesh.vertices.capacity() < trimesh.vertices.size()+triRef->getCount() )
            trimesh.vertices.reserve(trimesh.vertices.size()+triRef->getCount());
          while(k < (int)indexArray.getCount() )
          {
            for (int i=0;i<3;i++)
            {
              int index0 = indexArray.get(k)*vertexStride;
              domFloat fl0 = listFloats.get(index0);
              domFloat fl1 = listFloats.get(index0+1);
              domFloat fl2 = listFloats.get(index0+2);


              //RAVELOG_VERBOSEA("fUnitScale %f \n",fUnitScale);

              k+=triangleIndexStride;
              trimesh.indices.push_back(trimesh.vertices.size());
              trimesh.vertices.push_back(Vector(fl0*fUnitScale,fl1*fUnitScale,fl2*fUnitScale));
            }
          }
        }
      }
    }


    //  Debug trimesh with 0.0 positions
    //      for (size_t i = 0;i < trimesh.vertices.size();i++)
    //      {
    //          float fl0 = trimesh.vertices[i].x;
    //          float fl1 = trimesh.vertices[i].y;
    //          float fl2 = trimesh.vertices[i].z;
    //
    //          RAVELOG_VERBOSEA("Trimesh Position %f, %f, %f\n",fl0,fl1,fl2);
    //      }

    geom.InitCollisionMesh();
    return true;
  }

  /// Extract the Geometry in TRIGLE FANS and adds it to OpenRave
  /// \param  triRef  Array of triangle fans of the COLLADA's model
  /// \param  vertsRef    Array of vertices of the COLLADA's model
  /// \param  mapmaterials    Materials applied to the geometry
  /// \param  plink   Link of the kinematics model
  bool ExtractGeometry(const domTrifansRef triRef, const domVerticesRef vertsRef, const map<string,domMaterialRef>& mapmaterials, KinBody::LinkPtr plink)
  {
    //  Debug
    RAVELOG_WARNA("ExtractGeometry in TRIANGLE FANS and adds to OpenRAVE............\n");

    plink->_listGeomProperties.push_back(KinBody::Link::GEOMPROPERTIES(plink));
    KinBody::Link::GEOMPROPERTIES& geom = plink->_listGeomProperties.back();
    KinBody::Link::TRIMESH& trimesh = geom.collisionmesh;
    geom.type = KinBody::Link::GEOMPROPERTIES::GeomTrimesh;

    // resolve the material and assign correct colors to the geometry
    map<string,domMaterialRef>::const_iterator itmat = mapmaterials.find(triRef->getMaterial());
    if( itmat != mapmaterials.end() )
      FillGeometryColor(itmat->second,geom);

    int triangleIndexStride = 0;
    int vertexoffset = -1;
    domInput_local_offsetRef indexOffsetRef;

    for (unsigned int w=0;w<triRef->getInput_array().getCount();w++)
    {
      int offset = triRef->getInput_array()[w]->getOffset();
      daeString str = triRef->getInput_array()[w]->getSemantic();
      if (!strcmp(str,"VERTEX"))
      {
        indexOffsetRef = triRef->getInput_array()[w];
        vertexoffset = offset;
      }
      if (offset> triangleIndexStride)
      {
        triangleIndexStride = offset;
      }
    }
    triangleIndexStride++;

    for(size_t ip = 0; ip < triRef->getP_array().getCount(); ++ip)
    {
      domList_of_uints indexArray =triRef->getP_array()[ip]->getValue();

      for (size_t i=0;i<vertsRef->getInput_array().getCount();++i)
      {
        domInput_localRef localRef = vertsRef->getInput_array()[i];
        daeString str = localRef->getSemantic();
        if ( strcmp(str,"POSITION") == 0 )
        {
          const domSourceRef node = daeSafeCast<domSource>(localRef->getSource().getElement());
          if( !node )
            continue;
          dReal fUnitScale = GetUnitScale(node);
          const domFloat_arrayRef flArray = node->getFloat_array();
          if (!!flArray)
          {
            const domList_of_floats& listFloats = flArray->getValue();
            int k=vertexoffset;
            int vertexStride = 3;//instead of hardcoded stride, should use the 'accessor'

            if( trimesh.indices.capacity() < trimesh.indices.size()+triRef->getCount() )
              trimesh.indices.reserve(trimesh.indices.size()+triRef->getCount());
            if( trimesh.vertices.capacity() < trimesh.vertices.size()+triRef->getCount() )
              trimesh.vertices.reserve(trimesh.vertices.size()+triRef->getCount());

            size_t startoffset = (int)trimesh.vertices.size();

            while(k < (int)indexArray.getCount() )
            {
              int index0 = indexArray.get(k)*vertexStride;
              domFloat fl0 = listFloats.get(index0);
              domFloat fl1 = listFloats.get(index0+1);
              domFloat fl2 = listFloats.get(index0+2);
              k+=triangleIndexStride;
              trimesh.vertices.push_back(Vector(fl0*fUnitScale,fl1*fUnitScale,fl2*fUnitScale));
            }

            for(size_t ivert = startoffset+2; ivert < trimesh.vertices.size(); ++ivert)
            {
              trimesh.indices.push_back(startoffset);
              trimesh.indices.push_back(ivert-1);
              trimesh.indices.push_back(ivert);
            }
          }
        }
      }
    }

    geom.InitCollisionMesh();
    return false;
  }

  /// Extract the Geometry in TRIANGLE STRIPS and adds it to OpenRave
  /// \param  triRef  Array of Triangle Strips of the COLLADA's model
  /// \param  vertsRef    Array of vertices of the COLLADA's model
  /// \param  mapmaterials    Materials applied to the geometry
  /// \param  plink   Link of the kinematics model
  bool ExtractGeometry(const domTristripsRef triRef, const domVerticesRef vertsRef, const map<string,domMaterialRef>& mapmaterials, KinBody::LinkPtr plink)
  {
    //  Debug
    RAVELOG_WARNA("ExtractGeometry in TRIANGLE STRIPS and adds to OpenRAVE............\n");

    plink->_listGeomProperties.push_back(KinBody::Link::GEOMPROPERTIES(plink));
    KinBody::Link::GEOMPROPERTIES& geom = plink->_listGeomProperties.back();
    KinBody::Link::TRIMESH& trimesh = geom.collisionmesh;
    geom.type = KinBody::Link::GEOMPROPERTIES::GeomTrimesh;

    // resolve the material and assign correct colors to the geometry
    map<string,domMaterialRef>::const_iterator itmat = mapmaterials.find(triRef->getMaterial());
    if( itmat != mapmaterials.end() )
      FillGeometryColor(itmat->second,geom);

    int triangleIndexStride = 0;
    int vertexoffset = -1;
    domInput_local_offsetRef indexOffsetRef;

    for (unsigned int w=0;w<triRef->getInput_array().getCount();w++)
    {
      int offset = triRef->getInput_array()[w]->getOffset();
      daeString str = triRef->getInput_array()[w]->getSemantic();
      if (!strcmp(str,"VERTEX"))
      {
        indexOffsetRef = triRef->getInput_array()[w];
        vertexoffset = offset;
      }
      if (offset> triangleIndexStride)
      {
        triangleIndexStride = offset;
      }
    }
    triangleIndexStride++;

    for(size_t ip = 0; ip < triRef->getP_array().getCount(); ++ip)
    {
      domList_of_uints indexArray =triRef->getP_array()[ip]->getValue();

      for (size_t i=0;i<vertsRef->getInput_array().getCount();++i)
      {
        domInput_localRef localRef = vertsRef->getInput_array()[i];
        daeString str = localRef->getSemantic();
        if ( strcmp(str,"POSITION") == 0 )
        {
          const domSourceRef node = daeSafeCast<domSource>(localRef->getSource().getElement());
          if( !node )
            continue;
          dReal fUnitScale = GetUnitScale(node);
          const domFloat_arrayRef flArray = node->getFloat_array();
          if (!!flArray)
          {
            const domList_of_floats& listFloats = flArray->getValue();
            int k=vertexoffset;
            int vertexStride = 3;//instead of hardcoded stride, should use the 'accessor'

            if( trimesh.indices.capacity() < trimesh.indices.size()+triRef->getCount() )
              trimesh.indices.reserve(trimesh.indices.size()+triRef->getCount());
            if( trimesh.vertices.capacity() < trimesh.vertices.size()+triRef->getCount() )
              trimesh.vertices.reserve(trimesh.vertices.size()+triRef->getCount());

            size_t startoffset = (int)trimesh.vertices.size();

            while(k < (int)indexArray.getCount() )
            {
              int index0 = indexArray.get(k)*vertexStride;
              domFloat fl0 = listFloats.get(index0);
              domFloat fl1 = listFloats.get(index0+1);
              domFloat fl2 = listFloats.get(index0+2);
              k+=triangleIndexStride;
              trimesh.vertices.push_back(Vector(fl0*fUnitScale,fl1*fUnitScale,fl2*fUnitScale));
            }

            bool bFlip = false;
            for(size_t ivert = startoffset+2; ivert < trimesh.vertices.size(); ++ivert)
            {
              trimesh.indices.push_back(ivert-2);
              trimesh.indices.push_back(bFlip ? ivert : ivert-1);
              trimesh.indices.push_back(bFlip ? ivert-1 : ivert);
              bFlip = !bFlip;
            }
          }
        }
      }
    }

    geom.InitCollisionMesh();
    return false;
  }

  /// Extract the Geometry and adds it to OpenRave
  /// \param  geom    Geometry to extract of the COLLADA's model
  /// \param  mapmaterials    Materials applied to the geometry
  /// \param  plink   Link of the kinematics model
  bool ExtractGeometry(const domGeometryRef geom, const map<string,domMaterialRef>& mapmaterials, KinBody::LinkPtr plink)
  {
    vector<Vector> vconvexhull;
    if (!!geom && geom->getMesh())
    {
      const domMeshRef meshRef = geom->getMesh();

      //  Extract Geometry of all array of TRIANGLES
      for (size_t tg = 0;tg<meshRef->getTriangles_array().getCount();tg++)
      {
        ExtractGeometry(meshRef->getTriangles_array()[tg], meshRef->getVertices(), mapmaterials, plink);
      }

      //  Extract Geometry of all array of TRIANGLE FANS
      for (size_t tg = 0;tg<meshRef->getTrifans_array().getCount();tg++)
      {
        ExtractGeometry(meshRef->getTrifans_array()[tg], meshRef->getVertices(), mapmaterials, plink);
      }

      // Extract Geometry of all array of TRIANGLE STRIPS
      for (size_t tg = 0;tg<meshRef->getTristrips_array().getCount();tg++)
      {
        ExtractGeometry(meshRef->getTristrips_array()[tg], meshRef->getVertices(), mapmaterials, plink);
      }

      // Types of PRIMITIVES NOT SUPPORTED by OpenRAVE
      if( meshRef->getPolylist_array().getCount()> 0 )
        RAVELOG_WARNA("openrave does not support collada polylists\n");
      if( meshRef->getPolygons_array().getCount()> 0 )
        RAVELOG_WARNA("openrave does not support collada polygons\n");

      //            if( alltrimesh.vertices.size() == 0 ) {
      //                const domVerticesRef vertsRef = meshRef->getVertices();
      //                for (size_t i=0;i<vertsRef->getInput_array().getCount();i++) {
      //                    domInput_localRef localRef = vertsRef->getInput_array()[i];
      //                    daeString str = localRef->getSemantic();
      //                    if ( strcmp(str,"POSITION") == 0 ) {
      //                        const domSourceRef node = daeSafeCast<domSource>(localRef->getSource().getElement());
      //                        if( !node )
      //                            continue;
      //                        dReal fUnitScale = GetUnitScale(node);
      //                        const domFloat_arrayRef flArray = node->getFloat_array();
      //                        if (!!flArray) {
      //                            const domList_of_floats& listFloats = flArray->getValue();
      //                            int vertexStride = 3;//instead of hardcoded stride, should use the 'accessor'
      //                            vconvexhull.reserve(vconvexhull.size()+listFloats.getCount());
      //                            for (size_t vertIndex = 0;vertIndex < listFloats.getCount();vertIndex+=vertexStride) {
      //                                //btVector3 verts[3];
      //                                domFloat fl0 = listFloats.get(vertIndex);
      //                                domFloat fl1 = listFloats.get(vertIndex+1);
      //                                domFloat fl2 = listFloats.get(vertIndex+2);
      //                                vconvexhull.push_back(Vector(fl0*fUnitScale,fl1*fUnitScale,fl2*fUnitScale));
      //                            }
      //                        }
      //                    }
      //                }
      //
      //                computeConvexHull(vconvexhull,alltrimesh);
      //            }

      return true;
    }

    if (!!geom && geom->getConvex_mesh())
    {
      {
        const domConvex_meshRef convexRef = geom->getConvex_mesh();
        daeElementRef otherElemRef = convexRef->getConvex_hull_of().getElement();
        if ( otherElemRef != NULL )
        {
          domGeometryRef linkedGeom = *(domGeometryRef*)&otherElemRef;
          printf( "otherLinked\n");
        }
        else
        {
          printf("convexMesh polyCount = %d\n",(int)convexRef->getPolygons_array().getCount());
          printf("convexMesh triCount = %d\n",(int)convexRef->getTriangles_array().getCount());
        }
      }

      const domConvex_meshRef convexRef = geom->getConvex_mesh();
      //daeString urlref = convexRef->getConvex_hull_of().getURI();
      daeString urlref2 = convexRef->getConvex_hull_of().getOriginalURI();
      if (urlref2)
      {
        daeElementRef otherElemRef = convexRef->getConvex_hull_of().getElement();

        // Load all the geometry libraries
        for ( size_t i = 0; i < _dom->getLibrary_geometries_array().getCount(); i++)
        {
          domLibrary_geometriesRef libgeom = _dom->getLibrary_geometries_array()[i];

          for (size_t i = 0; i < libgeom->getGeometry_array().getCount(); i++)
          {
            domGeometryRef lib = libgeom->getGeometry_array()[i];
            if (!strcmp(lib->getId(),urlref2+1))
            { // skip the # at the front of urlref2
              //found convex_hull geometry
              domMesh *meshElement = lib->getMesh();//linkedGeom->getMesh();
              if (meshElement)
              {
                const domVerticesRef vertsRef = meshElement->getVertices();
                for (size_t i=0;i<vertsRef->getInput_array().getCount();i++)
                {
                  domInput_localRef localRef = vertsRef->getInput_array()[i];
                  daeString str = localRef->getSemantic();
                  if ( strcmp(str,"POSITION") == 0)
                  {
                    const domSourceRef node = daeSafeCast<domSource>(localRef->getSource().getElement());
                    if( !node )
                      continue;
                    dReal fUnitScale = GetUnitScale(node);
                    const domFloat_arrayRef flArray = node->getFloat_array();
                    if (!!flArray)
                    {
                      vconvexhull.reserve(vconvexhull.size()+flArray->getCount());
                      const domList_of_floats& listFloats = flArray->getValue();
                      for (size_t k=0;k+2<flArray->getCount();k+=3)
                      {
                        domFloat fl0 = listFloats.get(k);
                        domFloat fl1 = listFloats.get(k+1);
                        domFloat fl2 = listFloats.get(k+2);
                        vconvexhull.push_back(Vector(fl0*fUnitScale,fl1*fUnitScale,fl2*fUnitScale));
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
      else
      {
        //no getConvex_hull_of but direct vertices
        const domVerticesRef vertsRef = convexRef->getVertices();
        for (size_t i=0;i<vertsRef->getInput_array().getCount();i++)
        {
          domInput_localRef localRef = vertsRef->getInput_array()[i];
          daeString str = localRef->getSemantic();
          if ( strcmp(str,"POSITION") == 0 )
          {
            const domSourceRef node = daeSafeCast<domSource>(localRef->getSource().getElement());
            if( !node )
              continue;
            dReal fUnitScale = GetUnitScale(node);
            const domFloat_arrayRef flArray = node->getFloat_array();
            if (!!flArray)
            {
              const domList_of_floats& listFloats = flArray->getValue();
              vconvexhull.reserve(vconvexhull.size()+flArray->getCount());
              for (size_t k=0;k+2<flArray->getCount();k+=3)
              {
                domFloat fl0 = listFloats.get(k);
                domFloat fl1 = listFloats.get(k+1);
                domFloat fl2 = listFloats.get(k+2);
                vconvexhull.push_back(Vector(fl0*fUnitScale,fl1*fUnitScale,fl2*fUnitScale));
              }
            }
          }
        }
      }

      if( vconvexhull.size()> 0 )
      {
        plink->_listGeomProperties.push_back(KinBody::Link::GEOMPROPERTIES(plink));
        KinBody::Link::GEOMPROPERTIES& geom = plink->_listGeomProperties.back();
        KinBody::Link::TRIMESH& trimesh = geom.collisionmesh;
        geom.type = KinBody::Link::GEOMPROPERTIES::GeomTrimesh;

        computeConvexHull(vconvexhull,trimesh);
        geom.InitCollisionMesh();
      }
      return true;
    }

    return false;
  }

  /// Get daeElement of a given URI
  template <typename T>
  daeSmartRef<T> getElementFromUrl(daeURI &uri)
  {
    daeStandardURIResolver* daeURIRes = new daeStandardURIResolver(*(_collada.get()));
    daeElementRef   element =   daeURIRes->resolveElement(uri);
    return daeSafeCast<T>(element.cast());
  }

  daeElementRef getElementFromUrl(daeURI &uri)
  {
    daeStandardURIResolver* daeURIRes = new daeStandardURIResolver(*(_collada.get()));
    daeElementRef   element =   daeURIRes->resolveElement(uri);
    return element;
  }

  template <typename T, typename U>
  daeSmartRef<T> getSidRef(domCommon_sidref_or_paramRef paddr, const U& element)
  {
    domSidref sidref = NULL;

    //  If SIDREF founded
    if( paddr->getSIDREF() != NULL )
    {
      sidref = paddr->getSIDREF()->getValue();
    }
    //  If Parameter founded
    else if (paddr->getParam() != NULL)
    {
      if (searchIKM(paddr->getParam()->getValue(),element))
      {
        domInstance_kinematics_model_Array instance_kinematics_array;
        if (getElement()->typeID() == domKinematics_scene::ID())
        {
          RAVELOG_WARNA("Kinematics Scene Element \n");
          domKinematics_sceneRef dks = daeSafeCast<domKinematics_scene>(getElement().cast());
          instance_kinematics_array = dks->getInstance_kinematics_model_array();
        }
        else if (getElement()->typeID() == domArticulated_system::ID())
        {
          RAVELOG_WARNA("Articulated System Element \n");
          domArticulated_systemRef das = daeSafeCast<domArticulated_system>(getElement().cast());
          instance_kinematics_array = das->getKinematics()->getInstance_kinematics_model_array();
        }
        return getObject<T>(getRef(),instance_kinematics_array);
      }
    }

    //  Debug
    return NULL;
  }

  /// Search Intance Kinematics Model
  /// This recursive procedure stops when find IKM
  bool searchIKM(daeString ref, daeElementRef element)
  {
    if (element->typeID() == domKinematics_scene::ID()){

      //  Debug
      RAVELOG_WARNA("Kinematics Scene: %s\n",ref);

      domKinematics_sceneRef ks = daeSafeCast<domKinematics_scene>(element.cast());

      //  If the parameter is in articulated system
      if (ks->getInstance_articulated_system_array().getCount() != 0)
      {
        if (searchParam(ref, ks->getInstance_articulated_system_array()))
        {
          //  Debug
          RAVELOG_VERBOSEA("Articulated system found !!!\n");
          return searchIKM(getRef(),getElement());
        }
      }

      //  If the parameter there is not in articulated system find in kinematics model
      if (ks->getInstance_kinematics_model_array().getCount() != 0)
      {
        //  Debug
        RAVELOG_VERBOSEA("Kinematics model found !!!\n");

        setRef(ref);
        setElement(element);
        return true;
      }
    }
    else if (element->typeID() == domArticulated_system::ID()){
      RAVELOG_WARNA("Articulated System \n");
      domArticulated_systemRef as = daeSafeCast<domArticulated_system>(element.cast());
      if (as->getKinematics() != NULL){
        RAVELOG_WARNA("Kinematics \n");
        return true;
      }
      else if (as->getMotion() != NULL){
        //  Gets the motion element;
        _motion_element = as->getMotion();

        daeTArray <domInstance_articulated_systemRef> instance_articulated_array;
        instance_articulated_array.append(as->getMotion()->getInstance_articulated_system());
        if (searchParam(ref, instance_articulated_array))
        {
          RAVELOG_WARNA("Motion \n");
          return searchIKM(getRef(),getElement());
        }
      }
    }

    return false;
  }

  /// Store parameter reference
  void setRef(daeString ref)
  {
    _ref =  ref;
  }

  /// Store element reference
  void setElement(daeElementRef element)
  {
    _element = element;
  }

  /// Retrieve element stored
  daeElementRef getElement()
  {
    return _element;
  }

  /// Retrieve parameter reference stored
  daeString getRef()
  {
    return _ref;
  }

  daeString getSidRef_value()
  {
    return  _sidref;
  }

  /// Search a given parameter reference and stores the new reference to search
  /// \param ref The reference to search
  /// \param paramArray The array of parameter where the method searchs
  template <typename T>
  bool searchParam(daeString ref, const T& paramArray)
  {
    bool param_founded = false;

    //  Debug
    RAVELOG_WARNA("search Param: %s\n",ref);

    // For each Instance find binds
    for(size_t iikm = 0; iikm < paramArray.getCount(); ++iikm)
    {
      //  Debug
      RAVELOG_WARNA("Number of Binds: %d\n",paramArray[iikm]->getBind_array().getCount());

      //  For each Bind
      for (size_t iparam = 0; iparam < paramArray[iikm]->getBind_array().getCount(); ++iparam)
      {
        // Compare bind parameter and given param
        if( strcmp(paramArray[iikm]->getBind_array()[iparam]->getSymbol(), ref) == 0 )
        {
          if( paramArray[iikm]->getBind_array()[iparam]->getParam() != NULL )
          {
            //  Debug
            RAVELOG_WARNA("Ref: %s\n",paramArray[iikm]->getBind_array()[iparam]->getParam()->getRef());

            setRef(paramArray[iikm]->getBind_array()[iparam]->getParam()->getRef());
            setElement(paramArray[iikm]->getUrl().getElement());

            param_founded = true;
            break;
          }
        }
      }

      //  If found parameter
      if( param_founded )
        break;
    }

    return param_founded;
  }

  /// Gets the object that points SIDREF
  template <typename T, typename U>
  daeSmartRef<T> getObject(daeString ref, const U& paramArray)
  {
    domSidref sidref = NULL;
    daeElement* pref = NULL;

    //  Debug
    RAVELOG_WARNA("Number of instances: %d\n",paramArray.getCount());

    // parameter of kinematics
    // search children parameters of kscene whose sid's match
    for(size_t iikm = 0; iikm < paramArray.getCount(); ++iikm)
    {
      //  Debug
      RAVELOG_WARNA("Number of newparam: %d\n",paramArray[iikm]->getNewparam_array().getCount());

      for(size_t iparam = 0; iparam < paramArray[iikm]->getNewparam_array().getCount(); ++iparam)
      {
        // take the SIDREF of those parameters to get the real kinematics model
        if( strcmp(paramArray[iikm]->getNewparam_array()[iparam]->getSid(), ref) == 0 )
        {
          if( paramArray[iikm]->getNewparam_array()[iparam]->getSIDREF() != NULL )
          {
            sidref = paramArray[iikm]->getNewparam_array()[iparam]->getSIDREF()->getValue();
            pref = paramArray[iikm]->getNewparam_array()[iparam];
            break;
          }
        }
      }

      if( sidref != NULL )
        break;
    }

    if( sidref == NULL )
    {
      RAVELOG_WARNA("failed to find instance kinematics sidref\n");

      _sidref =   NULL;

      return NULL;
    }

    //  Store sidref value
    _sidref = sidref;

    //  Debug
    if (!pref)
    {
      RAVELOG_ERROR("pref not FOUND\n");
    }
    else
    {
      RAVELOG_VERBOSE("pref WAS FOUND!!\n");
    }

    // SID path to kinematics
    daeSIDResolver resolver(pref,sidref);
    daeSmartRef<T> ptarget = daeSafeCast<T>(resolver.getElement());
    //daeSmartRef<T> ptarget = daeSafeCast<T>(daeSidRef(sidref, pref).resolve().elt);

    if( !ptarget )
    {
      RAVELOG_ERROR("failed to resolve %s from %s\n", sidref, ref);
      return NULL;
    }
    else
    {
      RAVELOG_VERBOSE("Resolved %s from %s\n", sidref, ref);
    }

    if( ptarget->typeID() != T::ID() )
    {
      RAVELOG_ERROR("unexpected resolved type (%s) \n", ptarget->getTypeName());
      return NULL;
    }

    return ptarget;
  }

  template <typename U>
  domFloat resolveFloat(domCommon_float_or_paramRef paddr, const U& paramArray)
  {
    if( paddr->getFloat() != NULL )
      return paddr->getFloat()->getValue();

    if( paddr->getParam() == NULL )
    {
      RAVELOG_WARNA("joint value not specified, setting to 0\n");
      return 0;
    }

    for(size_t iikm = 0; iikm < paramArray.getCount(); ++iikm)
    {
      for(size_t iparam = 0; iparam < paramArray[iikm]->getNewparam_array().getCount(); ++iparam)
      {
        domKinematics_newparamRef pnewparam = paramArray[iikm]->getNewparam_array()[iparam];
        if( strcmp(pnewparam->getSid(), paddr->getParam()->getValue()) == 0 )
        {
          if( pnewparam->getFloat() != NULL )
            return pnewparam->getFloat()->getValue();
          else if( pnewparam->getSIDREF() != NULL )
          {
            domKinematics_newparam::domFloatRef ptarget = daeSafeCast<domKinematics_newparam::domFloat>(daeSidRef(pnewparam->getSIDREF()->getValue(), pnewparam).resolve().elt);
            if( ptarget == NULL )
            {
              RAVELOG_WARNA("failed to resolve %s from %s\n", pnewparam->getSIDREF()->getValue(), paddr->getID());
              continue;
            }

            if( ptarget->getElementType() != COLLADA_TYPE::FLOAT )
            {
              RAVELOG_WARNA("unexpected resolved element type (%d) \n", ptarget->getElementType());
              continue;
            }

            return ptarget->getValue();
          }
        }
      }
    }

    return NULL;
  }

  /// Gets all transformations applied to the node
  TransformMatrix getTransform(daeElementRef pelt)
  {
    TransformMatrix t;
    domRotateRef protate = daeSafeCast<domRotate>(pelt);
    if( !!protate )
    {
//        if( !protate->getSid() ) { // if sid is valid, then controlled by joint?
          t.rotfromaxisangle(Vector(protate->getValue()[0],protate->getValue()[1],protate->getValue()[2]), (dReal)(protate->getValue()[3]*(PI/180.0)));
//      }
      return t;
    }

    domTranslateRef ptrans = daeSafeCast<domTranslate>(pelt);
    if( !!ptrans )
    {
//      if( !ptrans->getSid() ) { // if sid is valid, then controlled by joint?
          t.trans = Vector(ptrans->getValue()[0], ptrans->getValue()[1], ptrans->getValue()[2]);
          t.trans *= GetUnitScale(pelt);
//      }
      return t;
    }

    domMatrixRef pmat = daeSafeCast<domMatrix>(pelt);
    if( !!pmat )
    {
      for(int i = 0; i < 3; ++i)
      {
        t.m[4*i+0] = pmat->getValue()[4*i+0];
        t.m[4*i+1] = pmat->getValue()[4*i+1];
        t.m[4*i+2] = pmat->getValue()[4*i+2];
        t.trans[i] = pmat->getValue()[4*i+3];
      }
      t.trans *= GetUnitScale(pelt);
      return t;
    }

    domScaleRef pscale = daeSafeCast<domScale>(pelt);
    if( !!pscale )
    {
      t.m[0] = pscale->getValue()[0];
      t.m[4*1+1] = pscale->getValue()[1];
      t.m[4*2+2] = pscale->getValue()[2];
      return t;
    }

    domLookatRef pcamera = daeSafeCast<domLookat>(pelt);
    if( pelt->typeID() == domLookat::ID() )
    {
      Vector campos(pcamera->getValue()[0], pcamera->getValue()[1], pcamera->getValue()[2]);
      Vector lookat(pcamera->getValue()[3], pcamera->getValue()[4], pcamera->getValue()[5]);
      Vector camup(pcamera->getValue()[6], pcamera->getValue()[7], pcamera->getValue()[8]);

      Vector dir = -(lookat - campos);
      dReal len = RaveSqrt(dir.lengthsqr3());

      if( len> 1e-6 )
        dir *= 1/len;
      else
        dir = Vector(0,0,1);

      Vector up = camup - dir * dot3(dir,camup);
      len = up.lengthsqr3();
      if( len < 1e-8 )
      {
        up = Vector(0,1,0);
        up -= dir * dot3(dir,up);
        len = up.lengthsqr3();
        if( len < 1e-8 )
        {
          up = Vector(1,0,0);
          up -= dir * dot3(dir,up);
          len = up.lengthsqr3();
        }
      }

      up *= 1/RaveSqrt(len);

      Vector right; right.Cross(up,dir);
      t.m[0] = right.x; t.m[1] = up.x; t.m[2] = dir.x;
      t.m[4] = right.y; t.m[5] = up.y; t.m[6] = dir.y;
      t.m[8] = right.z; t.m[9] = up.z; t.m[10] = dir.z;
      t.trans = campos * GetUnitScale(pelt);
      return t;
    }

    domSkewRef pskew = daeSafeCast<domSkew>(pelt);
    if( !!pskew )
    {
      RAVELOG_ERRORA("skew transform not implemented\n");
    }

    return t;
  }

  /// Travels recursively the node parents of the given one
  /// to extract the Transform arrays that affects the node given
  template <typename T>
  TransformMatrix getNodeParentTransform(const T pelt)
  {
    domNodeRef pnode = daeSafeCast<domNode>(pelt->getParent());
    if( !pnode )
      return TransformMatrix();
    return getNodeParentTransform(pnode) * getFullTransform(pnode);
  }

  /// Travel by the transformation array and calls the getTransform method
  template <typename T>
  TransformMatrix getFullTransform(const T pelt)
  {
    TransformMatrix t;

    for(size_t i = 0; i < pelt->getContents().getCount(); ++i)
    {
      t = t * getTransform(pelt->getContents()[i]);
    }
    return t;
  }

  TransformMatrix getFullTransform(daeElementRef pelt)
  {
    TransformMatrix t;

    for(size_t i = 0; i < pelt->getChildren().getCount(); ++i)
    {
      t = t * getTransform(pelt->getChildren()[i]);
    }
    return t;
  }

  template <typename T>
  Vector getVector3(const T& t)
  {
    return Vector(t[0],t[1],t[2],0);
  }

  template <typename T>
  Vector getVector4(const T& t)
  {
    return Vector(t[0],t[1],t[2],t[3]);
  }

  // decompose a matrix into a scale and rigid transform (necessary for model scales)
  void decompose(const TransformMatrix& tm, Transform& tout, Vector& vscale)
  {
    tout = tm;
  }

  virtual void handleError( daeString msg )
  {
    RAVELOG_ERRORA("COLLADA error: %s\n", msg);
  }

  virtual void handleWarning( daeString msg )
  {
    RAVELOG_WARNA("COLLADA warning: %s\n", msg);
  }

  inline dReal GetUnitScale(daeElement* pelt)
  {
    return ((USERDATA*)pelt->getUserData())->scale;
  }

private:

  bool _checkMathML(daeElementRef pelt,const string& type)
  {
      return pelt->getElementName()==type || pelt->getElementName()==(string("math:")+type);
  }

  KinBody::JointPtr _getJointFromRef(xsToken targetref, daeElementRef peltref, KinBodyPtr pkinbody)
  {
      daeElement* peltjoint = daeSidRef(targetref, peltref).resolve().elt;
      domJointRef pdomjoint = daeSafeCast<domJoint> (peltjoint);

      if (!pdomjoint) {
          domInstance_jointRef pdomijoint = daeSafeCast<domInstance_joint> (peltjoint);
          if (!!pdomijoint)
              pdomjoint = daeSafeCast<domJoint> (pdomijoint->getUrl().getElement().cast());
      }

      if (!pdomjoint || pdomjoint->typeID() != domJoint::ID()) {
          RAVELOG_WARNA(str(boost::format("could not find collada joint %s!\n")%targetref));
          return KinBody::JointPtr();
      }

      KinBody::JointPtr pjoint = pkinbody->GetJoint(pdomjoint->getName());
      if (!pjoint) {
          RAVELOG_WARNA(str(boost::format("could not find openrave joint %s!\n")%pdomjoint->getName()));
      }
      return pjoint;
  }

  bool computeConvexHull(const vector<Vector>& verts, KinBody::Link::TRIMESH& trimesh)
  {
    RAVELOG_ERRORA("convex hulls not supported\n");
    return false;
    //        if( verts.size() <= 3 )
    //            return false;
    //
    //        int dim = 3;                    // dimension of points
    //        vector<coordT> qpoints(3*verts.size());
    //        for(size_t i = 0; i < verts.size(); ++i) {
    //            qpoints[3*i+0] = verts[i].x;
    //            qpoints[3*i+1] = verts[i].y;
    //            qpoints[3*i+2] = verts[i].z;
    //        }
    //
    //        bool bSuccess = false;
    //        boolT ismalloc = 0;           // True if qhull should free points in qh_freeqhull() or reallocation
    //        char flags[]= "qhull Tv"; // option flags for qhull, see qh_opt.htm
    //        FILE *outfile = NULL;    // stdout, output from qh_produce_output(), use NULL to skip qh_produce_output()
    //        FILE *errfile = tmpfile();    // stderr, error messages from qhull code
    //
    //        int exitcode= qh_new_qhull (dim, qpoints.size()/3, &qpoints[0], ismalloc, flags, outfile, errfile);
    //        if (!exitcode) { // no error
    //            vconvexplanes.reserve(100);
    //
    //            facetT *facet;              // set by FORALLfacets
    //            FORALLfacets { // 'qh facet_list' contains the convex hull
    //                vconvexplanes.push_back(Vector(facet->normal[0], facet->normal[1], facet->normal[2], facet->offset));
    //            }
    //
    //            bSuccess = true;
    //        }
    //
    //        qh_freeqhull(!qh_ALL);
    //        int curlong, totlong;   // memory remaining after qh_memfreeshort
    //        qh_memfreeshort (&curlong, &totlong);
    //        if (curlong || totlong)
    //            ROS_ERROR("qhull internal warning (main): did not free %d bytes of long memory (%d pieces)", totlong, curlong);
    //
    //        fclose(errfile);
    //        return bSuccess;
  }

  boost::shared_ptr<DAE>  _collada;
  domCOLLADA*                         _dom;
  EnvironmentBasePtr              _penv;
  vector<USERDATA>                _vuserdata; // all userdata

  //  Parameters of the searchParam
  daeString           _ref;
  daeString           _sidref;
  daeElementRef   _element;
  domMotionRef    _motion_element;

  //  Reader needed to Sensors
  boost::shared_ptr<BaseXMLReader> _pcurreader;
};

#endif
