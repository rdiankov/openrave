// Copyright (C) 2006-2010 Rosen Diankov (rdiankov@cs.cmu.edu), Stefan Ulbrich, Gustavo Rodriguez
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

struct NodeMatcher: public daeElement::matchElement {
 public:
    NodeMatcher(){};
    virtual bool operator()(daeElement* elt) const
    {return (domNode::ID()==elt->typeID());};
};

class ColladaReader: public daeErrorHandler
{
    class JointAxisBinding
    {
    public:
        JointAxisBinding(daeElementRef pvisualtrans, domAxis_constraintRef pkinematicaxis, domCommon_float_or_paramRef jointvalue, domKinematics_axis_infoRef kinematics_axis_info, domMotion_axis_infoRef motion_axis_info) : pvisualtrans(pvisualtrans), pkinematicaxis(pkinematicaxis), jointvalue(jointvalue), kinematics_axis_info(kinematics_axis_info), motion_axis_info(motion_axis_info) {
            BOOST_ASSERT( !!pkinematicaxis );   
            visualnode = NULL;
            daeElement* pae = pvisualtrans->getParentElement();
            while (!!pae) {
                visualnode = daeSafeCast<domNode> (pae);            
                if (!!visualnode) {
                    break;
                }
                pae = pae->getParentElement();
            }
        
            if (!visualnode) {
                RAVELOG_WARNA(str(boost::format("couldn't find parent node of element id %s, sid %s\n")%pkinematicaxis->getID()%pkinematicaxis->getSid()));
            }
        }
        
        daeElementRef pvisualtrans;
        domAxis_constraintRef   pkinematicaxis;
        domCommon_float_or_paramRef jointvalue;
        domNodeRef visualnode;
        domKinematics_axis_infoRef kinematics_axis_info;
        domMotion_axis_infoRef motion_axis_info;
    };

    /// \brief inter-collada bindings for a kinematics scene
    class KinematicsSceneBindings
    {
    public:
        std::list< std::pair<domNodeRef,domInstance_kinematics_modelRef> > listKinematicsVisualBindings;
        std::list<JointAxisBinding> listAxisBindings;

        bool AddAxisInfo(const domInstance_kinematics_model_Array& arr, domKinematics_axis_infoRef kinematics_axis_info, domMotion_axis_infoRef motion_axis_info)
        {
            if( !kinematics_axis_info ) {
                return false;
            }
            for(size_t ik = 0; ik < arr.getCount(); ++ik) {
                daeElement* pelt = daeSidRef(kinematics_axis_info->getAxis(), arr[ik]).resolve().elt;
                if( !!pelt ) {
                    // look for the correct placement
                    bool bfound = false;
                    FOREACH(itbinding,listAxisBindings) {
                        if( itbinding->pkinematicaxis.cast() == pelt ) {
                            itbinding->kinematics_axis_info = kinematics_axis_info;
                            if( !!motion_axis_info ) {
                                itbinding->motion_axis_info = motion_axis_info;
                            }
                            bfound = true;
                            break;
                        }
                    }
                    if( !bfound ) {
                        RAVELOG_WARN(str(boost::format("could not find binding for axis: %s, %s\n")%kinematics_axis_info->getAxis()%pelt->getAttribute("sid")));
                        return false;
                    }
                    return true;
                }
            }
            RAVELOG_WARN(str(boost::format("could not find kinematics axis target: %s\n")%kinematics_axis_info->getAxis()));
            return false;
        }
    };

    struct USERDATA
    {
        USERDATA() {}
        USERDATA(dReal scale) : scale(scale) {}
        dReal scale;
        boost::shared_ptr<void> p; ///< custom managed data
    };

 public:
 ColladaReader(EnvironmentBasePtr penv) : _dom(NULL), _penv(penv), _nGlobalSensorId(0), _nGlobalManipulatorId(0) {
        daeErrorHandler::setErrorHandler(this);
    }
    virtual ~ColladaReader() {
        _vuserdata.clear();
        _collada.reset();
        DAE::cleanup();
    }

    bool InitFromFile(const string& filename) {
        RAVELOG_VERBOSE(str(boost::format("init COLLADA reader version: %s, namespace: %s, filename: %s\n")%COLLADA_VERSION%COLLADA_NAMESPACE%filename));
        _collada.reset(new DAE);
        _dom = _collada->open(filename);
        if (!_dom) {
            return false;
        }

        size_t maxchildren = _countChildren(_dom);
        _vuserdata.resize(0);
        _vuserdata.reserve(maxchildren);

        dReal dScale = 1.0;
        _processUserData(_dom, dScale);
        RAVELOG_VERBOSE(str(boost::format("processed children: %d/%d\n")%_vuserdata.size()%maxchildren));
        return true;
    }
    bool InitFromData(const string& pdata) {
        BOOST_ASSERT(0);
        return false;
    }

    /// \brief Extract all possible collada scene objects into the environment
    bool Extract()
    {
        domCOLLADA::domSceneRef allscene = _dom->getScene();
        if( !allscene ) {
            return false;
        }

        //  parse each instance kinematics scene
        for (size_t iscene = 0; iscene < allscene->getInstance_kinematics_scene_array().getCount(); iscene++) {
            domInstance_kinematics_sceneRef kiscene = allscene->getInstance_kinematics_scene_array()[iscene];
            domKinematics_sceneRef kscene = daeSafeCast<domKinematics_scene> (kiscene->getUrl().getElement().cast());
            if (!kscene) {
                continue;
            }

            KinematicsSceneBindings bindings;
            _ExtractKinematicsVisualBindings(kiscene,bindings);

            for(size_t ias = 0; ias < kscene->getInstance_articulated_system_array().getCount(); ++ias) {
                RobotBasePtr probot;
                if( ExtractArticulatedSystem(probot, kscene->getInstance_articulated_system_array()[ias], bindings) && !!probot ) {
                    RAVELOG_DEBUG(str(boost::format("Robot %s added to the environment ...\n")%probot->GetName()));
                    _penv->AddRobot(probot,true);
                }
            }
            for(size_t ikmodel = 0; ikmodel < kscene->getInstance_kinematics_model_array().getCount(); ++ikmodel) {
                KinBodyPtr pbody;
                if( ExtractKinematicsModel(pbody, kscene->getInstance_kinematics_model_array()[ikmodel], bindings) && !!pbody ) {
                    RAVELOG_VERBOSE(str(boost::format("Kinbody %s added to the environment\n")%pbody->GetName()));
                    _penv->AddKinBody(pbody,true);
                }
            }
        }

        //vector<std::string>  processed_nodes; //  Nodes that are part of kinematics models
//                RAVELOG_DEBUG(str(boost::format("kinematics model node name: %s\n")%node->getName()));
//                domNodeRef parentnode = daeSafeCast<domNode>(node->getParent());
//                if( !pparentnode ) {
//                    RAVELOG_WARN(str(boost::format("parent of node of id=%s is not a node?\n")%node->getID()));
//                    parentnode = node;
//                }
//                processed_nodes.push_back(parentnode->getID()); //  Store ID's of nodes that are part of kinematics model


        // add left-over visual objects
//        if (!!allscene->getInstance_visual_scene()) {
//            domVisual_sceneRef visual_scene = daeSafeCast<domVisual_scene>(allscene->getInstance_visual_scene()->getUrl().getElement().cast());
//            for (size_t node = 0; node < visual_scene->getNode_array().getCount(); node++) {
//                KinBodyPtr rigid_body;
//                domNodeRef pnode = visual_scene->getNode_array()[node];
//                bool found   = false;
//                string nodeId  = string(pnode->getID());
//                //  Search if the node is into processed nodes
//                for(size_t i=0;i < processed_nodes.size();i++) {
//                    //  If node belongs to processed nodes
//                    if (nodeId == processed_nodes[i]) {
//                        RAVELOG_VERBOSEA("Processed node name: %s\n",processed_nodes[i].c_str());
//                        found = true;
//                        break;
//                    }
//                }
//
//                //  If the node is not part of a kinbody
//                if (!found) {
//                    if (!Extract(rigid_body, NULL, NULL, pnode, v_all_bindings)) {
//                        RAVELOG_WARNA("failed to load kinbody WIHTOUT Joints\n");
//                        continue;
//                    }
//                    _penv->AddKinBody(rigid_body, true);
//                    RAVELOG_VERBOSEA("Found node %s\n",visual_scene->getNode_array()[node]->getName());
//                }
//            }
//        }

        //ExtractLink(pkinbody, NULL, pnode, Transform(), vdomjoints, vbindings);
        return true;
    }

    /// \extract the first possible robot in the scene
    bool Extract(RobotBasePtr& probot)
    {
        std::list< pair<domInstance_kinematics_modelRef, boost::shared_ptr<KinematicsSceneBindings> > > listPossibleBodies;
        domCOLLADA::domSceneRef allscene = _dom->getScene();
        if( !allscene ) {
            return false;
        }

        //  parse each instance kinematics scene, prioritize robots
        for (size_t iscene = 0; iscene < allscene->getInstance_kinematics_scene_array().getCount(); iscene++) {
            domInstance_kinematics_sceneRef kiscene = allscene->getInstance_kinematics_scene_array()[iscene];
            domKinematics_sceneRef kscene = daeSafeCast<domKinematics_scene> (kiscene->getUrl().getElement().cast());
            if (!kscene) {
                continue;
            }
            boost::shared_ptr<KinematicsSceneBindings> bindings(new KinematicsSceneBindings());
            _ExtractKinematicsVisualBindings(kiscene,*bindings);
            for(size_t ias = 0; ias < kscene->getInstance_articulated_system_array().getCount(); ++ias) {
                if( ExtractArticulatedSystem(probot, kscene->getInstance_articulated_system_array()[ias], *bindings) && !!probot ) {
                    return true;
                }
            }
            for(size_t ikmodel = 0; ikmodel < kscene->getInstance_kinematics_model_array().getCount(); ++ikmodel) {
                listPossibleBodies.push_back(make_pair(kscene->getInstance_kinematics_model_array()[ikmodel], bindings));
            }
        }

        KinBodyPtr pbody = probot;
        FOREACH(it, listPossibleBodies) {
            if( ExtractKinematicsModel(pbody, it->first, *it->second) && !!pbody ) {
                return true;
            }
        }

        return false;
    }

    bool Extract(KinBodyPtr& pbody)
    {
        domCOLLADA::domSceneRef allscene = _dom->getScene();
        if( !allscene ) {
            return false;
        }
        //  parse each instance kinematics scene for the first available model
        for (size_t iscene = 0; iscene < allscene->getInstance_kinematics_scene_array().getCount(); iscene++) {
            domInstance_kinematics_sceneRef kiscene = allscene->getInstance_kinematics_scene_array()[iscene];
            domKinematics_sceneRef kscene = daeSafeCast<domKinematics_scene> (kiscene->getUrl().getElement().cast());
            if (!kscene) {
                continue;
            }
            KinematicsSceneBindings bindings;
            _ExtractKinematicsVisualBindings(kiscene,bindings);
            for(size_t ikmodel = 0; ikmodel < kscene->getInstance_kinematics_model_array().getCount(); ++ikmodel) {
                if( ExtractKinematicsModel(pbody, kscene->getInstance_kinematics_model_array()[ikmodel], bindings) && !!pbody ) {
                    return true;
                }
            }
        }
        return true;
    }

    domTechniqueRef _ExtractOpenRAVEProfile(const domTechnique_Array& arr)
    {
        for(size_t i = 0; i < arr.getCount(); ++i) {
            if( strcmp(arr[i]->getProfile(), "OpenRAVE") == 0 ) {
                return arr[i];
            }
        }
        return domTechniqueRef();
    }

    /// \brief returns an openrave interface type from the extra array
    boost::shared_ptr<std::string> _ExtractInterfaceType(const domExtra_Array& arr) {
        for(size_t i = 0; i < arr.getCount(); ++i) {
            if( strcmp(arr[i]->getType(),"interface_type") == 0 ) {
                domTechniqueRef tec = _ExtractOpenRAVEProfile(arr[i]->getTechnique_array());
                if( !!tec ) {
                    daeElement* ptype = tec->getChild("interface");
                    if( !!ptype ) {
                        return boost::shared_ptr<std::string>(new std::string(ptype->getCharData()));
                    }
                }
            }
        }
        return boost::shared_ptr<std::string>();
    }

    /// \brief extracts an articulated system. Note that an articulated system can include other articulated systems
    /// \param probot the robot to be created from the system
    bool ExtractArticulatedSystem(RobotBasePtr& probot, domInstance_articulated_systemRef ias, KinematicsSceneBindings& bindings)
    {
        if( !ias ) {
            return false;
        }
        RAVELOG_DEBUG(str(boost::format("instance articulated system sid %s\n")%ias->getSid()));
        domArticulated_systemRef articulated_system = daeSafeCast<domArticulated_system> (ias->getUrl().getElement().cast());
        if( !articulated_system ) {
            return false;
        }
        if( !probot ) {
            boost::shared_ptr<std::string> pinterface_type = _ExtractInterfaceType(ias->getExtra_array());
            if( !pinterface_type ) {
                pinterface_type = _ExtractInterfaceType(articulated_system->getExtra_array());
            }
            if( !!pinterface_type ) {
                probot = RaveCreateRobot(_penv,*pinterface_type);
            }
        }
        if( !!articulated_system->getMotion() ) {
            domInstance_articulated_systemRef ias_new = articulated_system->getMotion()->getInstance_articulated_system();
            if( !!articulated_system->getMotion()->getTechnique_common() ) {
                for(size_t i = 0; i < articulated_system->getMotion()->getTechnique_common()->getAxis_info_array().getCount(); ++i) {
                    domMotion_axis_infoRef motion_axis_info = articulated_system->getMotion()->getTechnique_common()->getAxis_info_array()[i];
                    // this should point to a kinematics axis_info
                    domKinematics_axis_infoRef kinematics_axis_info = daeSafeCast<domKinematics_axis_info>(daeSidRef(motion_axis_info->getAxis(), ias_new).resolve().elt);
                    if( !!kinematics_axis_info ) {
                        // find the parent kinematics and go through all its instance kinematics models
                        daeElement* pparent = kinematics_axis_info->getParent();
                        while(!!pparent && pparent->typeID() != domKinematics::ID()) {
                            pparent = pparent->getParent();
                        }
                        BOOST_ASSERT(pparent!=NULL);
                        bindings.AddAxisInfo(daeSafeCast<domKinematics>(pparent)->getInstance_kinematics_model_array(), kinematics_axis_info, motion_axis_info);
                    }
                    else {
                        RAVELOG_WARN(str(boost::format("failed to find kinematics axis %s\n")%motion_axis_info->getAxis()));
                    }
                }
            }
            if( !ExtractArticulatedSystem(probot,ias_new,bindings) ) {
                return false;
            }
        }
        else {
            if( !articulated_system->getKinematics() ) {
                RAVELOG_WARN(str(boost::format("collada <kinematics> tag empty? instance_articulated_system=%s\n")%ias->getID()));
                return true;
            }

            if( !!articulated_system->getKinematics()->getTechnique_common() ) {
                for(size_t i = 0; i < articulated_system->getKinematics()->getTechnique_common()->getAxis_info_array().getCount(); ++i) {
                    bindings.AddAxisInfo(articulated_system->getKinematics()->getInstance_kinematics_model_array(), articulated_system->getKinematics()->getTechnique_common()->getAxis_info_array()[i], NULL);
                }
            }

            // parse the kinematics information
            if (!probot) {
                probot = RaveCreateRobot(_penv, "GenericRobot");
            }
            if( !probot ) {
                probot = RaveCreateRobot(_penv, "");
            }
            BOOST_ASSERT(probot->IsRobot());

            KinBodyPtr pbody = probot;
            for(size_t ik = 0; ik < articulated_system->getKinematics()->getInstance_kinematics_model_array().getCount(); ++ik) {
                ExtractKinematicsModel(pbody,articulated_system->getKinematics()->getInstance_kinematics_model_array()[ik],bindings);
            }
        }

        //  Add the robot to the environment
        if( probot->GetName().size() == 0 ) {
            probot->SetName(articulated_system->getName());
        }
        if( probot->GetName().size() == 0 ) {
            probot->SetName(articulated_system->getId());
        }

        ExtractRobotManipulators(probot, articulated_system);
        ExtractRobotAttachedSensors(probot, articulated_system);
        return true;
    }

    bool ExtractKinematicsModel(KinBodyPtr& pkinbody, domInstance_kinematics_modelRef ikm, KinematicsSceneBindings& bindings)
    {
        if( !ikm ) {
            return false;
        }
        RAVELOG_DEBUG(str(boost::format("instance kinematics model sid %s\n")%ikm->getSid()));
        domKinematics_modelRef kmodel = daeSafeCast<domKinematics_model> (ikm->getUrl().getElement().cast());
        if (!kmodel) {
            RAVELOG_WARN(str(boost::format("%s does not reference valid kinematics\n")%ikm->getSid()));
            return false;
        }
        domPhysics_modelRef pmodel;
        if( !pkinbody ) {
            boost::shared_ptr<std::string> pinterface_type = _ExtractInterfaceType(ikm->getExtra_array());
            if( !pinterface_type ) {
                pinterface_type = _ExtractInterfaceType(kmodel->getExtra_array());
            }
            if( !!pinterface_type ) {
                pkinbody = RaveCreateKinBody(_penv,*pinterface_type);
            }
        }

        // find matching visual node
        domNodeRef pvisualnode;
        FOREACH(it, bindings.listKinematicsVisualBindings) {
            if( it->second == ikm ) {
                pvisualnode = it->first;
                break;
            }
        }
        if( !pvisualnode ) {
            RAVELOG_WARN(str(boost::format("failed to find visual node for instance kinematics model %s\n")%ikm->getSid()));
            return false;
        }

        if (!ExtractKinematicsModel(pkinbody, kmodel, pvisualnode, pmodel, bindings.listAxisBindings)) {
            RAVELOG_WARN(str(boost::format("failed to load kinbody from kinematics model %s\n")%kmodel->getID()));
            return false;
        }
        return true;
    }

    /// \brief append the kinematics model to the openrave kinbody
    bool ExtractKinematicsModel(KinBodyPtr& pkinbody, domKinematics_modelRef kmodel, domNodeRef pnode, domPhysics_modelRef pmodel, const std::list<JointAxisBinding>& listAxisBindings)
    {
        vector<domJointRef> vdomjoints;
        if (!pkinbody) {
            pkinbody = RaveCreateKinBody(_penv);
        }
        if( !!kmodel->getName() ) {
            pkinbody->SetName(kmodel->getName());
        }
        if( pkinbody->GetName().size() == 0 && !!kmodel->getID() ) {
            pkinbody->SetName(kmodel->getID());
        }
        RAVELOG_DEBUG(str(boost::format("kinematics model: %s\n")%pkinbody->GetName()));
        if( !!pnode ) {
            RAVELOG_DEBUG(str(boost::format("node name: %s\n")%pnode->getId()));
        }

        //  Process joint of the kinbody
        domKinematics_model_techniqueRef ktec = kmodel->getTechnique_common();

        //  Store joints
        for (size_t ijoint = 0; ijoint < ktec->getJoint_array().getCount(); ++ijoint) {
            vdomjoints.push_back(ktec->getJoint_array()[ijoint]);
        }

        //  Store instances of joints
        for (size_t ijoint = 0; ijoint < ktec->getInstance_joint_array().getCount(); ++ijoint) {
            domJointRef pelt = daeSafeCast<domJoint> (ktec->getInstance_joint_array()[ijoint]->getUrl().getElement());
            if (!pelt) {
                RAVELOG_WARNA("failed to get joint from instance\n");
            }
            else {
                vdomjoints.push_back(pelt);
            }
        }

        RAVELOG_VERBOSE(str(boost::format("Number of links in the kmodel %d\n")%ktec->getLink_array().getCount()));

        if( !!pnode ) {
            // This is not a safe cast - predecessor might be a transformation
            domNodeRef  parent  = daeSafeCast<domNode>(pnode->getAncestor(NodeMatcher()));

            //  Extract all links into the kinematics_model
            for (size_t ilink = 0; ilink < ktec->getLink_array().getCount(); ++ilink) {
                domNodeRef  child = daeSafeCast<domNode>(parent->getChildrenByType<domNode>()[ilink]);
                RAVELOG_VERBOSE(str(boost::format("Node Name out of ExtractLink: %s\n")%child->getName()));
                ExtractLink(pkinbody, ktec->getLink_array()[ilink], child, getNodeParentTransform(child), vdomjoints, listAxisBindings);
            }
        }
        else {
            for (size_t ilink = 0; ilink < ktec->getLink_array().getCount(); ++ilink) {
                ExtractLink(pkinbody, ktec->getLink_array()[ilink], domNodeRef(), Transform(), vdomjoints, listAxisBindings);
            }
        }

        //  TODO: implement mathml
        for (size_t iform = 0; iform < ktec->getFormula_array().getCount(); ++iform) {
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
                for (size_t ichild = 0; ichild < pf->getTechnique_common()->getChildren().getCount(); ++ichild) {
                    daeElementRef pelt = pf->getTechnique_common()->getChildren()[ichild];
                    if (pelt->getElementName() == string("math") || pelt->getElementName() == string("math:math")) {
                        peltmath = pelt;
                    }
                    else {
                        RAVELOG_WARNA(str(boost::format("unsupported formula element: %s\n")%pelt->getElementName()));
                    }
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
        return true;
    }

    ///  \brief Extract Link info and add it to an existing body
    KinBody::LinkPtr ExtractLink(KinBodyPtr pkinbody, const domLinkRef pdomlink,const domNodeRef pdomnode, Transform tParentLink, const vector<domJointRef>& vdomjoints, const std::list<JointAxisBinding>& listAxisBindings) {
        //  Set link name with the name of the COLLADA's Link
        std::string linkname = _ExtractLinkName(pdomlink);
        if( linkname.size() == 0 ) {
            RAVELOG_WARN("<link> has no name or id!\n");
            if( !!pdomnode ) {
                if (!!pdomnode->getName()) {
                    linkname = pdomnode->getName();
                }
                if( linkname.size() == 0 && !!pdomnode->getID()) {
                    linkname = pdomnode->getID();
                }
            }
        }

        KinBody::LinkPtr plink = pkinbody->GetLink(linkname);
        if( !plink ) {
            plink.reset(new KinBody::Link(pkinbody));
            plink->name = linkname;
            plink->_mass = 1.0;
            plink->bStatic = false;
            plink->index = (int) pkinbody->_veclinks.size();
            pkinbody->_veclinks.push_back(plink);
        }

        _getUserData(pdomlink)->p = plink;

        if( !!pdomnode ) {
            RAVELOG_VERBOSE(str(boost::format("Node Id %s and Name %s\n")%pdomnode->getId()%pdomnode->getName()));
        }

        if (!pdomlink) {
            RAVELOG_WARN("Extract object NOT kinematics !!!\n");
            ExtractGeometry(pdomnode,plink,listAxisBindings);
        }
        else {
            RAVELOG_DEBUGA(str(boost::format("Attachment link elements: %d\n")%pdomlink->getAttachment_full_array().getCount()));
            Transform tlink = _ExtractFullTransform(pdomlink);
            plink->_t = tParentLink * tlink; // use the kinematics coordinate system for each link
          
            {
                stringstream ss; ss << plink->GetName() << ": " << plink->_t << endl;
                RAVELOG_DEBUG(ss.str());
            }
          
            // Get the geometry
            ExtractGeometry(pdomnode,plink,listAxisBindings);
            
            RAVELOG_DEBUG(str(boost::format("After ExtractGeometry Attachment link elements: %d\n")%pdomlink->getAttachment_full_array().getCount()));
          
            //  Process all atached links
            for (size_t iatt = 0; iatt < pdomlink->getAttachment_full_array().getCount(); ++iatt) {
                domLink::domAttachment_fullRef pattfull = pdomlink->getAttachment_full_array()[iatt];

                // get link kinematics transformation
                TransformMatrix tatt = _ExtractFullTransform(pattfull);

                // process attached links
                daeElement* peltjoint = daeSidRef(pattfull->getJoint(), pattfull).resolve().elt;
                domJointRef pdomjoint = daeSafeCast<domJoint> (peltjoint);

                if (!pdomjoint) {
                    domInstance_jointRef pdomijoint = daeSafeCast<domInstance_joint> (peltjoint);
                    if (!!pdomijoint) {
                        pdomjoint = daeSafeCast<domJoint> (pdomijoint->getUrl().getElement().cast());
                    }
                }

                if (!pdomjoint || pdomjoint->typeID() != domJoint::ID()) {
                    RAVELOG_WARN(str(boost::format("could not find attached joint %s!\n")%pattfull->getJoint()));
                    return KinBody::LinkPtr();
                }

                // get direct child link
                if (!pattfull->getLink()) {
                    RAVELOG_WARN(str(boost::format("joint %s needs to be attached to a valid link\n")%pdomjoint->getID()));
                    continue;
                }

                // find the correct joint in the bindings
                daeTArray<domAxis_constraintRef> vdomaxes = pdomjoint->getChildrenByType<domAxis_constraint>();
                domNodeRef pchildnode;
                daeElementRef paxisnode;
                domKinematics_axis_infoRef kinematics_axis_info;
                domMotion_axis_infoRef motion_axis_info;

                // see if joint has a binding to a visual node
                FOREACHC(itaxisbinding,listAxisBindings) {
                    for (size_t ic = 0; ic < vdomaxes.getCount(); ++ic) {
                        //  If the binding for the joint axis is found, retrieve the info
                        if (vdomaxes[ic] == itaxisbinding->pkinematicaxis) {
                            pchildnode          = itaxisbinding->visualnode;
                            paxisnode           = itaxisbinding->pvisualtrans;
                            kinematics_axis_info  = itaxisbinding->kinematics_axis_info;
                            motion_axis_info     = itaxisbinding->motion_axis_info;
                            break;
                        }
                    }
                    if( !!pchildnode || !!paxisnode ) {
                        break;
                    }
                }
              
                if (!pchildnode) {
                    RAVELOG_DEBUG(str(boost::format("joint %s has no visual binding\n")%pdomjoint->getID()));
                }

                // create the joints before creating the child links
                vector<KinBody::JointPtr> vjoints(vdomaxes.getCount());
                for (size_t ic = 0; ic < vdomaxes.getCount(); ++ic) {
                    KinBody::JointPtr pjoint(new KinBody::Joint(pkinbody));
                    pjoint->bodies[0] = plink;
                    pjoint->bodies[1].reset();
                    pjoint->name = pdomjoint->getName();
                    pjoint->jointindex = (int) pkinbody->_vecjoints.size();

                    domAxis_constraintRef pdomaxis = vdomaxes[ic];
                    if( strcmp(pdomaxis->getElementName(), "revolute") == 0 ) {
                        pjoint->type = KinBody::Joint::JointRevolute;
                    }
                    else if( strcmp(pdomaxis->getElementName(), "prismatic") == 0 ) {
                        pjoint->type = KinBody::Joint::JointPrismatic;
                    }
                    else {
                        RAVELOG_WARN(str(boost::format("unsupported joint type: %s\n")%pdomaxis->getElementName()));
                    }

                    pjoint->dofindex = pkinbody->GetDOF();
                    pjoint->_vweights.resize(pjoint->GetDOF());
                    FOREACH(it,pjoint->_vweights) {
                        *it = 1;
                    }
                    pkinbody->_vecJointIndices.push_back(pjoint->dofindex);
                    pkinbody->_vecjoints.push_back(pjoint);
                    _getUserData(pdomjoint)->p = pjoint;
                    _getUserData(pdomaxis)->p = boost::shared_ptr<int>(new int(pjoint->dofindex));
                    vjoints[ic] = pjoint;
                }

                KinBody::LinkPtr pchildlink = ExtractLink(pkinbody, pattfull->getLink(), pchildnode, plink->_t * tatt, vdomjoints, listAxisBindings);

                if (!pchildlink) {
                    RAVELOG_WARN(str(boost::format("Link NULL: %s\n")%plink->GetName()));
                    continue;
                }

                int numjoints = 0;
                for (size_t ic = 0; ic < vdomaxes.getCount(); ++ic) {
                    domAxis_constraintRef pdomaxis = vdomaxes[ic];
                    if (!pchildlink) {
                        // create dummy child link
                        // multiple axes can be easily done with "empty links"
                        RAVELOG_WARN(str(boost::format("creating dummy link %s, num joints %d\n")%plink->GetName()%numjoints));

                        stringstream ss;
                        ss << plink->name;
                        ss <<"_dummy" << numjoints;
                        pchildlink.reset(new KinBody::Link(pkinbody));
                        pchildlink->name = ss.str().c_str();
                        pchildlink->bStatic = false;
                        pchildlink->index = (int)pkinbody->_veclinks.size();
                        pkinbody->_veclinks.push_back(pchildlink);
                    }

                    RAVELOG_VERBOSE(str(boost::format("Joint %s assigned %d \n")%vjoints[ic]->GetName()%ic));
                    KinBody::JointPtr pjoint = vjoints[ic];
                    pjoint->bodies[1] = pchildlink;

                    //  Axes and Anchor assignment.
                    pjoint->vAxes[0] = Vector(pdomaxis->getAxis()->getValue()[0], pdomaxis->getAxis()->getValue()[1], pdomaxis->getAxis()->getValue()[2]).normalize3();
                    pjoint->vanchor = Vector(0,0,0);

                    int numbad = 0;
                    pjoint->offset = 0; // to overcome -pi to pi boundary
                    if (pkinbody->IsRobot() && !motion_axis_info) {
                        RAVELOG_WARN(str(boost::format("No motion axis info for joint %s\n")%pjoint->GetName()));
                    }

                    //  Sets the Speed and the Acceleration of the joint
                    if (!!motion_axis_info) {
                        if (!!motion_axis_info->getSpeed()) {
                            pjoint->fMaxVel = resolveFloat(motion_axis_info->getSpeed(),motion_axis_info);
                            RAVELOG_VERBOSE("... Joint Speed: %f...\n",pjoint->GetMaxVel());
                        }
                        if (!!motion_axis_info->getAcceleration()) {
                            pjoint->fMaxAccel = resolveFloat(motion_axis_info->getAcceleration(),motion_axis_info);
                            RAVELOG_VERBOSE("... Joint Acceleration: %f...\n",pjoint->GetMaxAccel());
                        }
                    }

                    bool joint_active = true; // if not active, put into the passive list
                    bool joint_locked = false; // if locked, joint angle is static
                    bool kinematics_limits = false; 

                    //  If there is NO kinematicaxisinfo
                    if (!!kinematics_axis_info) {
                        if( !!kinematics_axis_info->getActive() ) {
                            joint_active = resolveBool(kinematics_axis_info->getActive(),kinematics_axis_info);
                        }
                        if (!!kinematics_axis_info->getLocked()) {
                            joint_locked = resolveBool(kinematics_axis_info->getLocked(),kinematics_axis_info);
                        }
                        
                        if (joint_locked) { // If joint is locked set limits to the static value.
                            if( pjoint->type == KinBody::Joint::JointRevolute || pjoint->type ==KinBody::Joint::JointPrismatic) {
                                RAVELOG_WARN("lock joint!!\n");
                                pjoint->_vlowerlimit.push_back(0.0f);
                                pjoint->_vupperlimit.push_back(0.0f);
                            }
                        }
                        else if (kinematics_axis_info->getLimits()) { // If there are articulated system kinematics limits
                            kinematics_limits   = true;
                            dReal fscale = (pjoint->type == KinBody::Joint::JointRevolute)?(PI/180.0f):GetUnitScale(kinematics_axis_info);
                            if( pjoint->type == KinBody::Joint::JointRevolute || pjoint->type ==KinBody::Joint::JointPrismatic) {
                                pjoint->_vlowerlimit.push_back(fscale*(dReal)(resolveFloat(kinematics_axis_info->getLimits()->getMin(),kinematics_axis_info)));
                                pjoint->_vupperlimit.push_back(fscale*(dReal)(resolveFloat(kinematics_axis_info->getLimits()->getMax(),kinematics_axis_info)));
                                if( pjoint->type == KinBody::Joint::JointRevolute ) {
                                    if( pjoint->_vlowerlimit.back() < -PI || pjoint->_vupperlimit.back()> PI ) {
                                        pjoint->offset += 0.5f * (pjoint->_vlowerlimit.back() + pjoint->_vupperlimit.back());
                                        ++numbad;
                                    }
                                }
                            }
                        }
                    }

                    if( !joint_active ) {
                        RAVELOG_WARN(str(boost::format("joint %s is passive\n")%pjoint->GetName()));
                    }
                  
                    //  Search limits in the joints section
                    if (!kinematics_axis_info || (!joint_locked && !kinematics_limits)) {
                        for(int i = 0; i < pjoint->GetDOF(); ++i) {
                            //  If there are NO LIMITS
                            if( !pdomaxis->getLimits() ) {
                                RAVELOG_VERBOSE(str(boost::format("There are NO LIMITS in joint %s:%d ...\n")%pjoint->GetName()%kinematics_limits));
                                if( pjoint->type == KinBody::Joint::JointRevolute ) {
                                    pjoint->_bIsCircular = true;
                                    pjoint->_vlowerlimit.push_back(-PI);
                                    pjoint->_vupperlimit.push_back(PI);
                                }
                                else {
                                    pjoint->_vlowerlimit.push_back(-100000);
                                    pjoint->_vupperlimit.push_back(100000);
                                }
                            }
                            else {
                                RAVELOG_VERBOSE(str(boost::format("There are LIMITS in joint %s ...\n")%pjoint->GetName()));
                                dReal fscale = (pjoint->type == KinBody::Joint::JointRevolute)?(PI/180.0f):GetUnitScale(pdomaxis);
                                pjoint->_vlowerlimit.push_back((dReal)pdomaxis->getLimits()->getMin()->getValue()*fscale);
                                pjoint->_vupperlimit.push_back((dReal)pdomaxis->getLimits()->getMax()->getValue()*fscale);
                                if( pjoint->type == KinBody::Joint::JointRevolute ) {
                                    if( pjoint->_vlowerlimit.back() < -PI || pjoint->_vupperlimit.back()> PI ) {
                                        pjoint->offset += 0.5f * (pjoint->_vlowerlimit[i] + pjoint->_vupperlimit[i]);
                                        ++numbad;
                                    }
                                }
                            }
                        }
                    }

                    if( numbad> 0 ) {
                        pjoint->offset *= 1.0f / (dReal)numbad;
                        RAVELOG_VERBOSE("joint %s offset is %f\n", pjoint->GetName().c_str(), (float)pjoint->offset);
                    }
                  
                    // Transform applied to the joint
                    Transform tbody0, tbody1;
                    tbody0 = pjoint->bodies[0]->GetTransform();
                    tbody1 = pjoint->bodies[1]->GetTransform();
                    Transform trel;
                    trel = tbody0.inverse() * tbody1;

                    Transform toffsetfrom;

                    if (!!pchildnode) {
                        RAVELOG_DEBUG(str(boost::format("Applies Transform Offset From Parent Link (%s:%s)!!!\n")%pjoint->bodies[0]->GetName()%pjoint->bodies[1]->GetName()));
                        toffsetfrom = plink->_t * tatt;
                        toffsetfrom = tbody0.inverse() * toffsetfrom;
                    }
                  
                    pjoint->vanchor = toffsetfrom * pjoint->vanchor;

                    RAVELOG_DEBUG(str(boost::format("joint dof: %d, link %s offset is %f\n")%pjoint->dofindex%plink->GetName()%pjoint->offset));
                    RAVELOG_DEBUG(str(boost::format("OffsetFrom Translation trans.x:%f, trans.y:%f, trans.z:%f\n")%toffsetfrom.trans.x%toffsetfrom.trans.y%toffsetfrom.trans.z));
                    RAVELOG_DEBUG(str(boost::format("OffsetFrom Rotation rot.x:%f, rot.y:%f, rot.z:%f, rot.w:%f\n")%toffsetfrom.rot.x%toffsetfrom.rot.y%toffsetfrom.rot.z%toffsetfrom.rot.w));
                    RAVELOG_DEBUG(str(boost::format("vAnchor (%s) x:%f, y:%f, z:%f\n")%pjoint->GetName()%pjoint->vanchor.x%pjoint->vanchor.y%pjoint->vanchor.z));
                    RAVELOG_DEBUG(str(boost::format("vAxes x:%f, y:%f, z:%f\n")%pjoint->vAxes.at(0).x%pjoint->vAxes.at(0).y%pjoint->vAxes.at(0).z));

                    //  Rotate axis from the parent offset
                    for(int i = 0; i < pjoint->GetDOF(); ++i) {
                        pjoint->vAxes[i] = toffsetfrom.rotate(pjoint->vAxes[i]);
                    }

                    if( pjoint->type == KinBody::Joint::JointRevolute ) {
                        pjoint->tLeft = matrixFromAxisAngle(pjoint->vAxes[0], -pjoint->offset);
                    }

                    pjoint->fMaxVel = pjoint->GetType() == KinBody::Joint::JointPrismatic ? 0.01 : 0.5f;
          
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
    void ExtractGeometry(const domNodeRef pdomnode,KinBody::LinkPtr plink, const std::list<JointAxisBinding>& listAxisBindings)
    {
        if( !pdomnode ) {
            return;
        }

        RAVELOG_VERBOSE(str(boost::format("ExtractGeometry(node,link) of %s\n")%pdomnode->getName()));
//        for (size_t i = 0; i < pdomnode->getNode_array().getCount(); i++) {
//            RAVELOG_VERBOSE("[stef] (%d/%d) Children %s (%s)\n",i,pdomnode->getNode_array().getCount(),pdomnode->getNode_array()[i]->getID(), pdomnode->getID());
//        }

        // For all child nodes of pdomnode
        for (size_t i = 0; i < pdomnode->getNode_array().getCount(); i++) {
            //RAVELOG_VERBOSE("[stef]  %s: Process Children Children %s (%d/%d) \n",pdomnode->getID(),pdomnode->getNode_array()[i]->getID(),i,pdomnode->getNode_array().getCount());
            // check if contains a joint
            bool contains=false;
            FOREACHC(it,listAxisBindings) {
                //RAVELOG_VERBOSE("[stef] child node '%s' ?=  link node '%s'",pdomnode->getNode_array()[i]->getID(), it->visualnode->getID());

                // don't check ID's check if the reference is the same!
                //if (string(pdomnode->getNode_array()[i]->getID()).compare(string(it->visualnode->getID()))==0){
                if ( (pdomnode->getNode_array()[i])  == (it->visualnode)){
                    //domNode *pv = *(it->visualnode);
                    //domNode *child = *(pdomnode->getNode_array()[i]);
                    //if ( (child) == pv){
                    contains=true;
                    break;
                }
            }
            if (contains) {
                continue;
            }
            //RAVELOG_VERBOSEA("[stef] Process child node: %s\n", pdomnode->getNode_array()[i]->getID());

            ExtractGeometry(pdomnode->getNode_array()[i],plink, listAxisBindings);
            // Plink stayes the same for all children
            // replace pdomnode by child = pdomnode->getNode_array()[i]
            // hope for the best!
            // put everything in a subroutine in order to process pdomnode too!
        }

        unsigned int nGeomBefore =  plink->_listGeomProperties.size(); // #of Meshes already associated to this link

        // get the geometry
        for (size_t igeom = 0; igeom < pdomnode->getInstance_geometry_array().getCount(); ++igeom) {
            domInstance_geometryRef domigeom = pdomnode->getInstance_geometry_array()[igeom];
            domGeometryRef domgeom = daeSafeCast<domGeometry> (domigeom->getUrl().getElement());
            if (!domgeom) {
                RAVELOG_WARNA("link %s does not have valid geometry\n", plink->GetName().c_str());
                continue;
            }

            //  Gets materials
            map<string, domMaterialRef> mapmaterials;
            if (!!domigeom->getBind_material() && !!domigeom->getBind_material()->getTechnique_common()) {
                const domInstance_material_Array& matarray = domigeom->getBind_material()->getTechnique_common()->getInstance_material_array();
                for (size_t imat = 0; imat < matarray.getCount(); ++imat) {
                    domMaterialRef pmat = daeSafeCast<domMaterial>(matarray[imat]->getTarget().getElement());
                    if (!!pmat) {
                        mapmaterials[matarray[imat]->getSymbol()] = pmat;
                    }
                }
            }

            //  Gets the geometry
            ExtractGeometry(domgeom, mapmaterials, plink);
        }

        TransformMatrix tmnodegeom = (TransformMatrix) plink->_t.inverse() * getNodeParentTransform(pdomnode) * _ExtractFullTransform(pdomnode);
        Transform tnodegeom;
        Vector vscale;
        decompose(tmnodegeom, tnodegeom, vscale);

        list<KinBody::Link::GEOMPROPERTIES>::iterator itgeom= plink->_listGeomProperties.begin();
        for (unsigned int i=0; i< nGeomBefore; i++) {
            itgeom++; // change only the transformations of the newly found geometries.
        }

        //  Switch between different type of geometry PRIMITIVES
        for (; itgeom != plink->_listGeomProperties.end(); itgeom++) {
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
                RAVELOG_WARN("unknown geometry type: %d\n", itgeom->GetType());
            }

            //  Gets collision mesh
            KinBody::Link::TRIMESH trimesh = itgeom->GetCollisionMesh();
            trimesh.ApplyTransform(itgeom->_t);
            plink->collision.Append(trimesh);
        }
        //RAVELOG_VERBOSE("End Extract Geometry (%s)\n",pdomnode->getID());
    }

    /// Paint the Geometry with the color material
    /// \param  pmat    Material info of the COLLADA's model
    /// \param  geom    Geometry properties in OpenRAVE
    void FillGeometryColor(const domMaterialRef pmat, KinBody::Link::GEOMPROPERTIES& geom)
    {
        if( !!pmat && !!pmat->getInstance_effect() ) {
            domEffectRef peffect = daeSafeCast<domEffect>(pmat->getInstance_effect()->getUrl().getElement().cast());
            if( !!peffect ) {
                domProfile_common::domTechnique::domPhongRef pphong = daeSafeCast<domProfile_common::domTechnique::domPhong>(peffect->getDescendant(daeElement::matchType(domProfile_common::domTechnique::domPhong::ID())));
                if( !!pphong ) {
                    if( !!pphong->getAmbient() && !!pphong->getAmbient()->getColor() ) {
                        geom.ambientColor = getVector4(pphong->getAmbient()->getColor()->getValue());
                    }
                    if( !!pphong->getDiffuse() && !!pphong->getDiffuse()->getColor() ) {
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
        RAVELOG_VERBOSE("ExtractGeometry in TRIANGLES and adds to OpenRAVE............\n");
        if( triRef == NULL ) {
            return false;
        }
        plink->_listGeomProperties.push_back(KinBody::Link::GEOMPROPERTIES(plink));
        KinBody::Link::GEOMPROPERTIES& geom = plink->_listGeomProperties.back();
        KinBody::Link::TRIMESH& trimesh = geom.collisionmesh;
        geom.type = KinBody::Link::GEOMPROPERTIES::GeomTrimesh;

        // resolve the material and assign correct colors to the geometry
        if( !!triRef->getMaterial() ) {
            map<string,domMaterialRef>::const_iterator itmat = mapmaterials.find(triRef->getMaterial());
            if( itmat != mapmaterials.end() ) {
                FillGeometryColor(itmat->second,geom);
            }
        }

        int triangleIndexStride = 0;
        int vertexoffset = -1;
        domInput_local_offsetRef indexOffsetRef;

        for (unsigned int w=0;w<triRef->getInput_array().getCount();w++) {
            int offset = triRef->getInput_array()[w]->getOffset();
            daeString str = triRef->getInput_array()[w]->getSemantic();
            if (!strcmp(str,"VERTEX")) {
                indexOffsetRef = triRef->getInput_array()[w];
                vertexoffset = offset;
            }
            if (offset> triangleIndexStride) {
                triangleIndexStride = offset;
            }
        }

        triangleIndexStride++;
        const domList_of_uints& indexArray =triRef->getP()->getValue();

        for (size_t i=0;i<vertsRef->getInput_array().getCount();++i) {
            domInput_localRef localRef = vertsRef->getInput_array()[i];
            daeString str = localRef->getSemantic();
            if ( strcmp(str,"POSITION") == 0 ) {
                const domSourceRef node = daeSafeCast<domSource>(localRef->getSource().getElement());
                if( !node ) {
                    continue;
                }
                dReal fUnitScale = GetUnitScale(node);
                const domFloat_arrayRef flArray = node->getFloat_array();
                if (!!flArray) {
                    const domList_of_floats& listFloats = flArray->getValue();
                    int k=vertexoffset;
                    int vertexStride = 3;//instead of hardcoded stride, should use the 'accessor'

                    if( trimesh.indices.capacity() < trimesh.indices.size()+triRef->getCount() ) {
                        trimesh.indices.reserve(trimesh.indices.size()+triRef->getCount());
                    }
                    if( trimesh.vertices.capacity() < trimesh.vertices.size()+triRef->getCount() ) {
                        trimesh.vertices.reserve(trimesh.vertices.size()+triRef->getCount());
                    }
                    while(k < (int)indexArray.getCount() ) {
                        for (int i=0;i<3;i++) {
                            int index0 = indexArray.get(k)*vertexStride;
                            domFloat fl0 = listFloats.get(index0);
                            domFloat fl1 = listFloats.get(index0+1);
                            domFloat fl2 = listFloats.get(index0+2);
                            k+=triangleIndexStride;
                            trimesh.indices.push_back(trimesh.vertices.size());
                            trimesh.vertices.push_back(Vector(fl0*fUnitScale,fl1*fUnitScale,fl2*fUnitScale));
                        }
                    }
                }
            }
        }

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
        RAVELOG_VERBOSE("ExtractGeometry in TRIANGLE FANS and adds to OpenRAVE............\n");

        plink->_listGeomProperties.push_back(KinBody::Link::GEOMPROPERTIES(plink));
        KinBody::Link::GEOMPROPERTIES& geom = plink->_listGeomProperties.back();
        KinBody::Link::TRIMESH& trimesh = geom.collisionmesh;
        geom.type = KinBody::Link::GEOMPROPERTIES::GeomTrimesh;

        // resolve the material and assign correct colors to the geometry
        if( !!triRef->getMaterial() ) {
            map<string,domMaterialRef>::const_iterator itmat = mapmaterials.find(triRef->getMaterial());
            if( itmat != mapmaterials.end() ) {
                FillGeometryColor(itmat->second,geom);
            }
        }

        int triangleIndexStride = 0;
        int vertexoffset = -1;
        domInput_local_offsetRef indexOffsetRef;

        for (unsigned int w=0;w<triRef->getInput_array().getCount();w++) {
            int offset = triRef->getInput_array()[w]->getOffset();
            daeString str = triRef->getInput_array()[w]->getSemantic();
            if (!strcmp(str,"VERTEX")) {
                indexOffsetRef = triRef->getInput_array()[w];
                vertexoffset = offset;
            }
            if (offset> triangleIndexStride) {
                triangleIndexStride = offset;
            }
        }
        triangleIndexStride++;

        for(size_t ip = 0; ip < triRef->getP_array().getCount(); ++ip) {
            domList_of_uints indexArray =triRef->getP_array()[ip]->getValue();
            for (size_t i=0;i<vertsRef->getInput_array().getCount();++i) {
                domInput_localRef localRef = vertsRef->getInput_array()[i];
                daeString str = localRef->getSemantic();
                if ( strcmp(str,"POSITION") == 0 ) {
                    const domSourceRef node = daeSafeCast<domSource>(localRef->getSource().getElement());
                    if( !node ) {
                        continue;
                    }
                    dReal fUnitScale = GetUnitScale(node);
                    const domFloat_arrayRef flArray = node->getFloat_array();
                    if (!!flArray) {
                        const domList_of_floats& listFloats = flArray->getValue();
                        int k=vertexoffset;
                        int vertexStride = 3;//instead of hardcoded stride, should use the 'accessor'

                        if( trimesh.indices.capacity() < trimesh.indices.size()+triRef->getCount() ) {
                            trimesh.indices.reserve(trimesh.indices.size()+triRef->getCount());
                        }
                        if( trimesh.vertices.capacity() < trimesh.vertices.size()+triRef->getCount() ) {
                            trimesh.vertices.reserve(trimesh.vertices.size()+triRef->getCount());
                        }

                        size_t startoffset = (int)trimesh.vertices.size();
                        while(k < (int)indexArray.getCount() ) {
                            int index0 = indexArray.get(k)*vertexStride;
                            domFloat fl0 = listFloats.get(index0);
                            domFloat fl1 = listFloats.get(index0+1);
                            domFloat fl2 = listFloats.get(index0+2);
                            k+=triangleIndexStride;
                            trimesh.vertices.push_back(Vector(fl0*fUnitScale,fl1*fUnitScale,fl2*fUnitScale));
                        }

                        for(size_t ivert = startoffset+2; ivert < trimesh.vertices.size(); ++ivert) {
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
        RAVELOG_VERBOSE("ExtractGeometry in TRIANGLE STRIPS and adds to OpenRAVE............\n");

        plink->_listGeomProperties.push_back(KinBody::Link::GEOMPROPERTIES(plink));
        KinBody::Link::GEOMPROPERTIES& geom = plink->_listGeomProperties.back();
        KinBody::Link::TRIMESH& trimesh = geom.collisionmesh;
        geom.type = KinBody::Link::GEOMPROPERTIES::GeomTrimesh;

        // resolve the material and assign correct colors to the geometry
        if( !!triRef->getMaterial() ) {
            map<string,domMaterialRef>::const_iterator itmat = mapmaterials.find(triRef->getMaterial());
            if( itmat != mapmaterials.end() ) {
                FillGeometryColor(itmat->second,geom);
            }
        }
        int triangleIndexStride = 0;
        int vertexoffset = -1;
        domInput_local_offsetRef indexOffsetRef;

        for (unsigned int w=0;w<triRef->getInput_array().getCount();w++) {
            int offset = triRef->getInput_array()[w]->getOffset();
            daeString str = triRef->getInput_array()[w]->getSemantic();
            if (!strcmp(str,"VERTEX")) {
                indexOffsetRef = triRef->getInput_array()[w];
                vertexoffset = offset;
            }
            if (offset> triangleIndexStride) {
                triangleIndexStride = offset;
            }
        }
        triangleIndexStride++;

        for(size_t ip = 0; ip < triRef->getP_array().getCount(); ++ip) {
            domList_of_uints indexArray =triRef->getP_array()[ip]->getValue();

            for (size_t i=0;i<vertsRef->getInput_array().getCount();++i) {
                domInput_localRef localRef = vertsRef->getInput_array()[i];
                daeString str = localRef->getSemantic();
                if ( strcmp(str,"POSITION") == 0 ) {
                    const domSourceRef node = daeSafeCast<domSource>(localRef->getSource().getElement());
                    if( !node ) {
                        continue;
                    }
                    dReal fUnitScale = GetUnitScale(node);
                    const domFloat_arrayRef flArray = node->getFloat_array();
                    if (!!flArray) {
                        const domList_of_floats& listFloats = flArray->getValue();
                        int k=vertexoffset;
                        int vertexStride = 3;//instead of hardcoded stride, should use the 'accessor'

                        if( trimesh.indices.capacity() < trimesh.indices.size()+triRef->getCount() ) {
                            trimesh.indices.reserve(trimesh.indices.size()+triRef->getCount());
                        }
                        if( trimesh.vertices.capacity() < trimesh.vertices.size()+triRef->getCount() ) {
                            trimesh.vertices.reserve(trimesh.vertices.size()+triRef->getCount());
                        }

                        size_t startoffset = (int)trimesh.vertices.size();

                        while(k < (int)indexArray.getCount() ) {
                            int index0 = indexArray.get(k)*vertexStride;
                            domFloat fl0 = listFloats.get(index0);
                            domFloat fl1 = listFloats.get(index0+1);
                            domFloat fl2 = listFloats.get(index0+2);
                            k+=triangleIndexStride;
                            trimesh.vertices.push_back(Vector(fl0*fUnitScale,fl1*fUnitScale,fl2*fUnitScale));
                        }

                        bool bFlip = false;
                        for(size_t ivert = startoffset+2; ivert < trimesh.vertices.size(); ++ivert) {
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
        if (!!geom && geom->getMesh()) {
            const domMeshRef meshRef = geom->getMesh();

            //  Extract Geometry of all array of TRIANGLES
            for (size_t tg = 0;tg<meshRef->getTriangles_array().getCount();tg++) {
                ExtractGeometry(meshRef->getTriangles_array()[tg], meshRef->getVertices(), mapmaterials, plink);
            }

            //  Extract Geometry of all array of TRIANGLE FANS
            for (size_t tg = 0;tg<meshRef->getTrifans_array().getCount();tg++) {
                ExtractGeometry(meshRef->getTrifans_array()[tg], meshRef->getVertices(), mapmaterials, plink);
            }
                
            // Extract Geometry of all array of TRIANGLE STRIPS
            for (size_t tg = 0;tg<meshRef->getTristrips_array().getCount();tg++) {
                ExtractGeometry(meshRef->getTristrips_array()[tg], meshRef->getVertices(), mapmaterials, plink);
            }

            // Types of PRIMITIVES NOT SUPPORTED by OpenRAVE
            if( meshRef->getPolylist_array().getCount()> 0 ) {
                RAVELOG_WARN("openrave does not support collada polylists\n");
            }
            if( meshRef->getPolygons_array().getCount()> 0 ) {
                RAVELOG_WARN("openrave does not support collada polygons\n");
            }

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
            //                _computeConvexHull(vconvexhull,alltrimesh);
            //            }

            return true;
        }

        if (!!geom && geom->getConvex_mesh()) {
            {
                const domConvex_meshRef convexRef = geom->getConvex_mesh();
                daeElementRef otherElemRef = convexRef->getConvex_hull_of().getElement();
                if ( !!otherElemRef ) {
                    domGeometryRef linkedGeom = *(domGeometryRef*)&otherElemRef;
                    RAVELOG_WARN( "otherLinked\n");
                }
                else {
                    RAVELOG_WARN("convexMesh polyCount = %d\n",(int)convexRef->getPolygons_array().getCount());
                    RAVELOG_WARN("convexMesh triCount = %d\n",(int)convexRef->getTriangles_array().getCount());
                }
            }

            const domConvex_meshRef convexRef = geom->getConvex_mesh();
            //daeString urlref = convexRef->getConvex_hull_of().getURI();
            daeString urlref2 = convexRef->getConvex_hull_of().getOriginalURI();
            if (urlref2) {
                daeElementRef otherElemRef = convexRef->getConvex_hull_of().getElement();

                // Load all the geometry libraries
                for ( size_t i = 0; i < _dom->getLibrary_geometries_array().getCount(); i++) {
                    domLibrary_geometriesRef libgeom = _dom->getLibrary_geometries_array()[i];
                    for (size_t i = 0; i < libgeom->getGeometry_array().getCount(); i++) {
                        domGeometryRef lib = libgeom->getGeometry_array()[i];
                        if (!strcmp(lib->getId(),urlref2+1)) { // skip the # at the front of urlref2
                            //found convex_hull geometry
                            domMesh *meshElement = lib->getMesh();//linkedGeom->getMesh();
                            if (meshElement) {
                                const domVerticesRef vertsRef = meshElement->getVertices();
                                for (size_t i=0;i<vertsRef->getInput_array().getCount();i++) {
                                    domInput_localRef localRef = vertsRef->getInput_array()[i];
                                    daeString str = localRef->getSemantic();
                                    if ( strcmp(str,"POSITION") == 0) {
                                        const domSourceRef node = daeSafeCast<domSource>(localRef->getSource().getElement());
                                        if( !node ) {
                                            continue;
                                        }
                                        dReal fUnitScale = GetUnitScale(node);
                                        const domFloat_arrayRef flArray = node->getFloat_array();
                                        if (!!flArray) {
                                            vconvexhull.reserve(vconvexhull.size()+flArray->getCount());
                                            const domList_of_floats& listFloats = flArray->getValue();
                                            for (size_t k=0;k+2<flArray->getCount();k+=3) {
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
            else {
                //no getConvex_hull_of but direct vertices
                const domVerticesRef vertsRef = convexRef->getVertices();
                for (size_t i=0;i<vertsRef->getInput_array().getCount();i++) {
                    domInput_localRef localRef = vertsRef->getInput_array()[i];
                    daeString str = localRef->getSemantic();
                    if ( strcmp(str,"POSITION") == 0 ) {
                        const domSourceRef node = daeSafeCast<domSource>(localRef->getSource().getElement());
                        if( !node ) {
                            continue;
                        }
                        dReal fUnitScale = GetUnitScale(node);
                        const domFloat_arrayRef flArray = node->getFloat_array();
                        if (!!flArray) {
                            const domList_of_floats& listFloats = flArray->getValue();
                            vconvexhull.reserve(vconvexhull.size()+flArray->getCount());
                            for (size_t k=0;k+2<flArray->getCount();k+=3) {
                                domFloat fl0 = listFloats.get(k);
                                domFloat fl1 = listFloats.get(k+1);
                                domFloat fl2 = listFloats.get(k+2);
                                vconvexhull.push_back(Vector(fl0*fUnitScale,fl1*fUnitScale,fl2*fUnitScale));
                            }
                        }
                    }
                }
            }

            if( vconvexhull.size()> 0 ) {
                plink->_listGeomProperties.push_back(KinBody::Link::GEOMPROPERTIES(plink));
                KinBody::Link::GEOMPROPERTIES& geom = plink->_listGeomProperties.back();
                KinBody::Link::TRIMESH& trimesh = geom.collisionmesh;
                geom.type = KinBody::Link::GEOMPROPERTIES::GeomTrimesh;

                _computeConvexHull(vconvexhull,trimesh);
                geom.InitCollisionMesh();
            }
            return true;
        }

        return false;
    }

    /// \brief extract the robot manipulators
    void ExtractRobotManipulators(RobotBasePtr probot, const domArticulated_systemRef as)
    {
        for(size_t ie = 0; ie < as->getExtra_array().getCount(); ++ie) {
            domExtraRef pextra = as->getExtra_array()[ie];
            if( strcmp(pextra->getType(), "manipulator") == 0 ) {
                string name = pextra->getAttribute("name");
                if( name.size() == 0 ) {
                    name = str(boost::format("manipulator%d")%_nGlobalManipulatorId++);
                }
                domTechniqueRef tec = _ExtractOpenRAVEProfile(pextra->getTechnique_array());
                if( !!tec ) {
                    RobotBase::ManipulatorPtr pmanip(new RobotBase::Manipulator(probot));
                    pmanip->_name = name;
                    daeElement* pframe_origin = tec->getChild("frame_origin");
                    daeElement* pframe_tip = tec->getChild("frame_tip");
                    if( !!pframe_origin ) {
                        domLinkRef plink = daeSafeCast<domLink>(daeSidRef(pframe_origin->getAttribute("link"), as).resolve().elt);
                        if( !!plink ) {
                            pmanip->_pBase = boost::static_pointer_cast<KinBody::Link>(_getUserData(plink)->p);
                        }
                        if( !pmanip->_pBase ) {
                            RAVELOG_WARN(str(boost::format("failed to find manipulator %s frame origin %s\n")%name%pframe_origin->getAttribute("link")));
                            continue;
                        }
                    }
                    if( !!pframe_tip ) {
                        daeElementRef plink = daeSafeCast<domLink>(daeSidRef(pframe_tip->getAttribute("link"), as).resolve().elt);
                        if( !!plink ) {
                            pmanip->_pEndEffector = boost::static_pointer_cast<KinBody::Link>(_getUserData(plink)->p);
                        }
                        if( !pmanip->_pEndEffector ) {
                            RAVELOG_WARN(str(boost::format("failed to find manipulator %s frame tip %s\n")%name%pframe_tip->getAttribute("link")));
                            continue;
                        }
                        pmanip->_tGrasp = _ExtractFullTransformFromChildren(pframe_tip);
                    }

                    for(size_t ic = 0; ic < tec->getContents().getCount(); ++ic) {
                        daeElementRef pgripper_axis = tec->getContents()[ic];
                        if( pgripper_axis->getElementName() == string("gripper_axis") ) {
                            domAxis_constraintRef paxis = daeSafeCast<domAxis_constraint>(daeSidRef(pgripper_axis->getAttribute("axis"), as).resolve().elt);
                            if( !!paxis ) {
                                boost::shared_ptr<int> pdofindex = boost::static_pointer_cast<int>(_getUserData(paxis)->p);
                                if( !!pdofindex ) {
                                    float closingdirection = 0;
                                    daeElementRef pclosingdirection = daeElementRef(pgripper_axis->getChild("closingdirection"));
                                    if( !pclosingdirection || !resolveCommon_float_or_param(pclosingdirection,as,closingdirection) ) {
                                        RAVELOG_WARN(str(boost::format("manipulator %s gripper axis %s failed to process closing direction\n")%name%pgripper_axis->getAttribute("axis")));
                                    }
                                    pmanip->_vgripperdofindices.push_back(*pdofindex);
                                    pmanip->_vClosingDirection.push_back((dReal)closingdirection);
                                    continue;
                                }
                            }
                            RAVELOG_WARN(str(boost::format("could not find manipulator gripper axis %s\n")%pgripper_axis->getAttribute("axis")));
                        }
                    }

                    probot->GetManipulators().push_back(pmanip);
                }
                else {
                    RAVELOG_WARN(str(boost::format("cannot create robot %s manipulator %s\n")%probot->GetName()%name));
                }
            }
        }
    }

    /// \brief Extract Sensors attached to a Robot
    void ExtractRobotAttachedSensors(RobotBasePtr probot, const domArticulated_systemRef as)
    {
        for (size_t ie = 0; ie < as->getExtra_array().getCount(); ie++) {
            domExtraRef pextra = as->getExtra_array()[ie];
            if( strcmp(pextra->getType(), "sensor") == 0 ) {
                string name = pextra->getAttribute("name");
                if( name.size() == 0 ) {
                    name = str(boost::format("sensor%d")%_nGlobalSensorId++);
                }
                domTechniqueRef tec = _ExtractOpenRAVEProfile(pextra->getTechnique_array());
                if( !!tec ) {
                    
                }
                else {
                    RAVELOG_WARN(str(boost::format("cannot create robot %s attached sensor %s\n")%probot->GetName()%name));
                }
            }
        }
    }

    /// Extract instance sensor info
    /// Extract instances of sensors located in articulated_systems extra node
    bool ExtractInstance_sensor(RobotBasePtr probot, daeElementRefArray instances)
    {
        std::string instance_id;
        std::string instance_url;
        std::string instance_link;
        std::string definition_id;
        std::string definition_type;
        daeElementRef dom_SensorActuatorManipulator;

        for (size_t i_instance = 0; i_instance < instances.getCount(); i_instance++) {
            daeElementRef instance_SensorActuatorManipulator = instances[i_instance];
            RAVELOG_DEBUG("Instance name: %s\n",instance_SensorActuatorManipulator->getElementName());
            if (strcmp(instance_SensorActuatorManipulator->getElementName(),"instance_sensor") == 0) {
                //  Get instance attributes
                daeTArray<daeElement::attr> instance_attributes = instance_SensorActuatorManipulator->getAttributes();
                for (size_t i_ins_attr = 0; i_ins_attr < instance_attributes.getCount(); i_ins_attr++) {
                    RAVELOG_DEBUG("Instance attribute %d %s: %s\n",i_ins_attr,instance_attributes[i_ins_attr].name.c_str(),instance_attributes[i_ins_attr].value.c_str());

                    if (instance_attributes[i_ins_attr].name == "id") {
                        instance_id = instance_attributes[i_ins_attr].value;
                    }

                    if (instance_attributes[i_ins_attr].name == "url") {
                        instance_url =  instance_attributes[i_ins_attr].value;
                    }

                    if (instance_attributes[i_ins_attr].name == "link") {
                        instance_link = instance_attributes[i_ins_attr].value;
                    }
                }

                RAVELOG_DEBUG("Get SensorActuatorManipulator info from url\n");

                daeURI  url = daeURI(*(instance_SensorActuatorManipulator.cast()),instance_url);
                dom_SensorActuatorManipulator = getElementFromUrl(url);

                //  Get definition attributes
                daeTArray<daeElement::attr> definition_attributes = dom_SensorActuatorManipulator->getAttributes();
                for (size_t i_def_attr = 0; i_def_attr < definition_attributes.getCount(); i_def_attr++) {
                    if (definition_attributes[i_def_attr].name == "type") {
                        definition_type  = definition_attributes[i_def_attr].value;
                    }

                    if (definition_attributes[i_def_attr].name == "id") {
                        definition_id = definition_attributes[i_def_attr].value;
                    }

                    RAVELOG_DEBUG("SensorActuator attribute %d %s: %s\n", i_def_attr, definition_attributes[i_def_attr].name.c_str(), definition_attributes[i_def_attr].value.c_str());
                }

                //  Create Sensor
                if (strcmp(instance_SensorActuatorManipulator->getElementName(),"instance_sensor") == 0) {
                    addSensor(probot,definition_id, definition_type, instance_id, instance_link, dom_SensorActuatorManipulator, instance_SensorActuatorManipulator);
                }
            }
        }

        return true;
    }

    //  Create Sensor and initilize it
    bool addSensor( RobotBasePtr  probot, std::string   definition_id, std::string   definition_type, std::string   instance_id, std::string   instance_link, daeElementRef dom_SensorActuatorManipulator, daeElementRef instance_SensorActuatorManipulator)
    {
        RobotBase::AttachedSensorPtr att_Sensor;
        //RobotBase::AttachedSensorPtr att_SensorActuator(new RobotBase::AttachedSensor(probot));
        att_Sensor = boost::shared_ptr<RobotBase::AttachedSensor>(new RobotBase::AttachedSensor(probot));

        probot->GetAttachedSensors().push_back(att_Sensor);

        //  Create Sensor of the TYPE required
        att_Sensor->psensor = RaveCreateSensor(probot->GetEnv(), definition_type.c_str());

        att_Sensor->psensor->SetName(definition_id.c_str());

        //  Sets attached actuator name from instance actuator Id
        att_Sensor->_name   =   instance_id.c_str();

        //  Sets sensor name from dom sensor Id
        RAVELOG_DEBUG("Sensor name: %s\n",att_Sensor->GetName().c_str());

        //  Create XML reader for this Sensor TYPE
        boost::shared_ptr<BaseXMLReader> pcurreader = RaveCallXMLReader(PT_Sensor,att_Sensor->psensor->GetXMLId(),att_Sensor->psensor, std::list<std::pair<std::string,std::string> >());    
        setSensorActuatorParams(dom_SensorActuatorManipulator, pcurreader);
        pcurreader.reset();

        if( !att_Sensor->psensor->Init("") ) {
            RAVELOG_INFOA("failed to initialize sensor %s\n", att_Sensor->GetName().c_str());
            att_Sensor->psensor.reset();
        }
        else {
            att_Sensor->pdata = att_Sensor->psensor->CreateSensorData();

            if( att_Sensor->pattachedlink.expired() ) {
                RAVELOG_DEBUG("attached link is NULL, setting to base of robot\n");
                if( probot->GetLinks().size() == 0 ) {
                    RAVELOG_WARN("robot has no links!\n");
                }
                else {
                    size_t  pos;
                    string  link_name;

                    link_name  =   instance_link;
                    pos        =   link_name.find_last_of("/");
                    link_name  =   link_name.substr(pos + 1);

                    //  TODO : Get link in which the sensor will be attached
                    for (size_t i_link = 0; i_link < probot->GetLinks().size(); i_link++) {
                        string robot_link   = probot->GetLinks()[i_link]->GetName();
                        RAVELOG_DEBUG(str(boost::format("link_name: %s robot_link: %s\n")%link_name%robot_link));
                        if (link_name == robot_link) {
                            att_Sensor->pattachedlink = probot->GetLinks()[i_link];
                            break;
                        }
                    }

                    //att_sensor->pattachedlink = probot->GetLinks().front();
                }
            }

            //Relative Transform to sensors
            att_Sensor->trelative = _ExtractFullTransformFromChildren(instance_SensorActuatorManipulator);
        }

        return true;
    }

    /// Fills Sensor and Actuator params from COLLADAS's file
    /// dom_SensorActuator  COLLADA sensor/actuator info
    bool setSensorActuatorParams(daeElementRef dom_SensorActuator, boost::shared_ptr<BaseXMLReader> pcurreader)
    {
        daeTArray<daeElementRef> childrens = dom_SensorActuator->getChildren();

        //  For each feature of the Actuator send this INFO to Actuator's plugin
        for (size_t i = 0; i < childrens.getCount(); i++) {
            std::list<std::pair<std::string,std::string> >  atts;
            string xmltag = childrens[i]->getElementName();
            RAVELOG_DEBUG("(SensorActuator) %s: %s Type ID %d\n",childrens[i]->getElementName(), childrens[i]->getCharData().c_str(), childrens[i]->typeID());
            std::transform(xmltag.begin(), xmltag.end(), xmltag.begin(), ::tolower);
            pcurreader->startElement(xmltag,atts);
            pcurreader->characters(childrens[i]->getCharData());
            pcurreader->endElement(xmltag);
        }

        return true;
    }

    daeElementRef getElementFromUrl(daeURI &uri)
    {
        daeStandardURIResolver* daeURIRes = new daeStandardURIResolver(*(_collada.get()));
        daeElementRef   element =   daeURIRes->resolveElement(uri);
        return element;
    }

    static daeElement* searchBinding(domCommon_sidref_or_paramRef paddr, daeElementRef parent)
    {
        if( !!paddr->getSIDREF() ) {
            return daeSidRef(paddr->getSIDREF()->getValue(),parent).resolve().elt;
        }
        if (!!paddr->getParam()) {
            return searchBinding(paddr->getParam()->getValue(),parent);
        }
        return NULL;
    }

    /// Search a given parameter reference and stores the new reference to search.
    /// \param ref the reference name to search
    /// \param parent The array of parameter where the method searchs.
    static daeElement* searchBinding(daeString ref, daeElementRef parent)
    {
        if( !parent ) {
            return NULL;
        }
        daeElement* pelt = NULL;
        domKinematics_sceneRef kscene = daeSafeCast<domKinematics_scene>(parent.cast());
        if( !!kscene ) {
            pelt = searchBindingArray(ref,kscene->getInstance_articulated_system_array());
            if( !!pelt ) {
                return pelt;
            }
            return searchBindingArray(ref,kscene->getInstance_kinematics_model_array());
        }
        domArticulated_systemRef articulated_system = daeSafeCast<domArticulated_system>(parent.cast());
        if( !!articulated_system ) {
            if( !!articulated_system->getKinematics() ) {
                pelt = searchBindingArray(ref,articulated_system->getKinematics()->getInstance_kinematics_model_array());
                if( !!pelt ) {
                    return pelt;
                }
            }
            if( !!articulated_system->getMotion() ) {
                return searchBinding(ref,articulated_system->getMotion()->getInstance_articulated_system());
            }
            return NULL;
        }
        // try to get a bind array
        daeElementRef pbindelt;
        const domKinematics_bind_Array* pbindarray = NULL;
        const domKinematics_newparam_Array* pnewparamarray = NULL;
        domInstance_articulated_systemRef ias = daeSafeCast<domInstance_articulated_system>(parent.cast());
        if( !!ias ) {
            pbindarray = &ias->getBind_array();
            pbindelt = ias->getUrl().getElement();
            pnewparamarray = &ias->getNewparam_array();
        }
        if( !pbindarray || !pbindelt ) {
            domInstance_kinematics_modelRef ikm = daeSafeCast<domInstance_kinematics_model>(parent.cast());
            if( !!ikm ) {
                pbindarray = &ikm->getBind_array();
                pbindelt = ikm->getUrl().getElement();
                pnewparamarray = &ikm->getNewparam_array();
            }
        }
        if( !!pbindarray && !!pbindelt ) {
            for (size_t ibind = 0; ibind < pbindarray->getCount(); ++ibind) {
                domKinematics_bindRef pbind = (*pbindarray)[ibind];
                if( !!pbind->getSymbol() && strcmp(pbind->getSymbol(), ref) == 0 ) { 
                    // found a match
                    if( !!pbind->getParam() ) {
                        return searchBinding(pbind->getParam()->getRef(), pbindelt);
                    }
                    else if( !!pbind->getSIDREF() ) {
                        return daeSidRef(pbind->getSIDREF()->getValue(), pbindelt).resolve().elt;
                    }
                }
            }
            for(size_t inewparam = 0; inewparam < pnewparamarray->getCount(); ++inewparam) {
                domKinematics_newparamRef newparam = (*pnewparamarray)[inewparam];
                if( !!newparam->getSid() && strcmp(newparam->getSid(), ref) == 0 ) {
                    if( !!newparam->getSIDREF() ) { // can only bind with SIDREF
                        return daeSidRef(newparam->getSIDREF()->getValue(),pbindelt).resolve().elt;
                    }
                    RAVELOG_WARN(str(boost::format("newparam sid=%s does not have SIDREF\n")%newparam->getSid()));
                }
            }
        }
        RAVELOG_WARN(str(boost::format("failed to get binding for element: %s\n")%parent->getElementName()));
        return NULL;
    }

    static daeElement* searchBindingArray(daeString ref, const domInstance_articulated_system_Array& paramArray)
    {
        for(size_t iikm = 0; iikm < paramArray.getCount(); ++iikm) {
            daeElement* pelt = searchBinding(ref,paramArray[iikm].cast());
            if( !!pelt ) {
                return pelt;
            }
        }
        return NULL;
    }

    static daeElement* searchBindingArray(daeString ref, const domInstance_kinematics_model_Array& paramArray)
    {
        for(size_t iikm = 0; iikm < paramArray.getCount(); ++iikm) {
            daeElement* pelt = searchBinding(ref,paramArray[iikm].cast());
            if( !!pelt ) {
                return pelt;
            }
        }
        return NULL;
    }

    template <typename U> static xsBoolean resolveBool(domCommon_bool_or_paramRef paddr, const U& parent) {
        if( !!paddr->getBool() ) {
            return paddr->getBool()->getValue();
        }
        if( !paddr->getParam() ) {
            RAVELOG_WARN("param not specified, setting to 0\n");
            return false;
        }
        for(size_t iparam = 0; iparam < parent->getNewparam_array().getCount(); ++iparam) {
            domKinematics_newparamRef pnewparam = parent->getNewparam_array()[iparam];
            if( !!pnewparam->getSid() && strcmp(pnewparam->getSid(), paddr->getParam()->getValue()) == 0 ) {
                if( !!pnewparam->getBool() ) {
                    return pnewparam->getBool()->getValue();
                }
                else if( !!pnewparam->getSIDREF() ) {
                    domKinematics_newparam::domBoolRef ptarget = daeSafeCast<domKinematics_newparam::domBool>(daeSidRef(pnewparam->getSIDREF()->getValue(), pnewparam).resolve().elt);
                    if( !ptarget ) {
                        RAVELOG_WARNA("failed to resolve %s from %s\n", pnewparam->getSIDREF()->getValue(), paddr->getID());
                        continue;
                    }
                    return ptarget->getValue();
                }
            }
        }
        RAVELOG_WARN(str(boost::format("failed to resolve %s\n")%paddr->getParam()->getValue()));
        return false;
    }
    template <typename U> static domFloat resolveFloat(domCommon_float_or_paramRef paddr, const U& parent) {
        if( !!paddr->getFloat() ) {
            return paddr->getFloat()->getValue();
        }
        if( !paddr->getParam() ) {
            RAVELOG_WARN("param not specified, setting to 0\n");
            return 0;
        }
        for(size_t iparam = 0; iparam < parent->getNewparam_array().getCount(); ++iparam) {
            domKinematics_newparamRef pnewparam = parent->getNewparam_array()[iparam];
            if( !!pnewparam->getSid() && strcmp(pnewparam->getSid(), paddr->getParam()->getValue()) == 0 ) {
                if( !!pnewparam->getFloat() ) {
                    return pnewparam->getFloat()->getValue();
                }
                else if( !!pnewparam->getSIDREF() ) {
                    domKinematics_newparam::domFloatRef ptarget = daeSafeCast<domKinematics_newparam::domFloat>(daeSidRef(pnewparam->getSIDREF()->getValue(), pnewparam).resolve().elt);
                    if( !ptarget ) {
                        RAVELOG_WARNA("failed to resolve %s from %s\n", pnewparam->getSIDREF()->getValue(), paddr->getID());
                        continue;
                    }
                    return ptarget->getValue();
                }
            }
        }
        RAVELOG_WARN(str(boost::format("failed to resolve %s\n")%paddr->getParam()->getValue()));
        return 0;
    }

    static bool resolveCommon_float_or_param(daeElementRef pcommon, daeElementRef parent, float& f)
    {
        daeElement* pfloat = pcommon->getChild("float");
        if( !!pfloat ) {
            stringstream sfloat(pfloat->getCharData());
            sfloat >> f;
            return !!sfloat;
        }
        daeElement* pparam = pcommon->getChild("param");
        if( !!pparam ) {
            if( pparam->hasAttribute("ref") ) {
                RAVELOG_WARN("cannot process param ref\n");
            }
            else {
                daeElement* pelt = daeSidRef(pparam->getCharData(),parent).resolve().elt;
                if( !!pelt ) {
                    RAVELOG_WARN(str(boost::format("found param ref: %s from %s\n")%pelt->getCharData()%pparam->getCharData()));
                }
            }
        }
        return false;
    }

    /// Gets all transformations applied to the node
    static TransformMatrix getTransform(daeElementRef pelt)
    {
        TransformMatrix t;
        domRotateRef protate = daeSafeCast<domRotate>(pelt);
        if( !!protate ) {
            //        if( !protate->getSid() ) { // if sid is valid, then controlled by joint?
            t = matrixFromAxisAngle(Vector(protate->getValue()[0],protate->getValue()[1],protate->getValue()[2]), (dReal)(protate->getValue()[3]*(PI/180.0)));
            //      }
            return t;
        }

        domTranslateRef ptrans = daeSafeCast<domTranslate>(pelt);
        if( !!ptrans ) {
            //      if( !ptrans->getSid() ) { // if sid is valid, then controlled by joint?
            t.trans = Vector(ptrans->getValue()[0], ptrans->getValue()[1], ptrans->getValue()[2]);
            t.trans *= GetUnitScale(pelt);
            //      }
            return t;
        }

        domMatrixRef pmat = daeSafeCast<domMatrix>(pelt);
        if( !!pmat ) {
            for(int i = 0; i < 3; ++i) {
                t.m[4*i+0] = pmat->getValue()[4*i+0];
                t.m[4*i+1] = pmat->getValue()[4*i+1];
                t.m[4*i+2] = pmat->getValue()[4*i+2];
                t.trans[i] = pmat->getValue()[4*i+3];
            }
            t.trans *= GetUnitScale(pelt);
            return t;
        }

        domScaleRef pscale = daeSafeCast<domScale>(pelt);
        if( !!pscale ) {
            t.m[0] = pscale->getValue()[0];
            t.m[4*1+1] = pscale->getValue()[1];
            t.m[4*2+2] = pscale->getValue()[2];
            return t;
        }

        domLookatRef pcamera = daeSafeCast<domLookat>(pelt);
        if( pelt->typeID() == domLookat::ID() ) {
            Vector campos(pcamera->getValue()[0], pcamera->getValue()[1], pcamera->getValue()[2]);
            Vector lookat(pcamera->getValue()[3], pcamera->getValue()[4], pcamera->getValue()[5]);
            Vector camup(pcamera->getValue()[6], pcamera->getValue()[7], pcamera->getValue()[8]);
            t = transformLookat(lookat,campos*GetUnitScale(pelt),camup);
            return t;
        }

        domSkewRef pskew = daeSafeCast<domSkew>(pelt);
        if( !!pskew ) {
            RAVELOG_ERRORA("skew transform not implemented\n");
        }

        return t;
    }

    /// Travels recursively the node parents of the given one
    /// to extract the Transform arrays that affects the node given
    template <typename T> static TransformMatrix getNodeParentTransform(const T pelt) {
        domNodeRef pnode = daeSafeCast<domNode>(pelt->getParent());
        if( !pnode ) {
            return TransformMatrix();
        }
        return getNodeParentTransform(pnode) * _ExtractFullTransform(pnode);
    }

    /// \brief Travel by the transformation array and calls the getTransform method
    template <typename T> static TransformMatrix _ExtractFullTransform(const T pelt) {
        TransformMatrix t;
        for(size_t i = 0; i < pelt->getContents().getCount(); ++i) {
            t = t * getTransform(pelt->getContents()[i]);
        }
        return t;
    }

    /// \brief Travel by the transformation array and calls the getTransform method
    template <typename T> static TransformMatrix _ExtractFullTransformFromChildren(const T pelt) {
        TransformMatrix t;
        for(size_t i = 0; i < pelt->getChildren().getCount(); ++i) {
            t = t * getTransform(pelt->getChildren()[i]);
        }
        return t;
    }

    template <typename T> Vector getVector3(const T& t) {
        return Vector(t[0],t[1],t[2],0);
    }
    
    template <typename T> Vector getVector4(const T& t) {
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

    inline static dReal GetUnitScale(daeElement* pelt)
    {
        return ((USERDATA*)pelt->getUserData())->scale;
    }

 private:

    std::string _ExtractLinkName(domLinkRef pdomlink) {
        std::string linkname;
        if( !!pdomlink ) {
            if( !!pdomlink->getName() ) {
                linkname = pdomlink->getName();
            }
            if( linkname.size() == 0 ) {
                linkname = pdomlink->getID();
            }
        }
        return linkname;
    }

    bool _checkMathML(daeElementRef pelt,const string& type)
    {
        return pelt->getElementName()==type || pelt->getElementName()==(string("math:")+type);
    }

    KinBody::JointPtr _getJointFromRef(xsToken targetref, daeElementRef peltref, KinBodyPtr pkinbody) {
        daeElement* peltjoint = daeSidRef(targetref, peltref).resolve().elt;
        domJointRef pdomjoint = daeSafeCast<domJoint> (peltjoint);

        if (!pdomjoint) {
            domInstance_jointRef pdomijoint = daeSafeCast<domInstance_joint> (peltjoint);
            if (!!pdomijoint) {
                pdomjoint = daeSafeCast<domJoint> (pdomijoint->getUrl().getElement().cast());
            }
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

    /// \brief go through all kinematics binds to get a kinematics/visual pair
    /// \param kiscene instance of one kinematics scene, binds the kinematic and visual models
    /// \param bindings the extracted bindings
    static void _ExtractKinematicsVisualBindings(domInstance_kinematics_sceneRef kiscene, KinematicsSceneBindings& bindings)
    {
        domKinematics_sceneRef kscene = daeSafeCast<domKinematics_scene> (kiscene->getUrl().getElement().cast());
        if (!kscene) {
            return;
        }
        for (size_t imodel = 0; imodel < kiscene->getBind_kinematics_model_array().getCount(); imodel++) {
            domArticulated_systemRef articulated_system = NULL; // if filled, contains robot-specific information, so create a robot
            domBind_kinematics_modelRef kbindmodel = kiscene->getBind_kinematics_model_array()[imodel];
            if (!kbindmodel->getNode()) {
                RAVELOG_WARNA("do not support kinematics models without references to nodes\n");
                continue;
            }
       
            // visual information
            domNodeRef node = daeSafeCast<domNode> (daeSidRef(kbindmodel->getNode(), kbindmodel).resolve().elt);
            if (!node) {
                RAVELOG_WARNA(str(boost::format("bind_kinematics_model does not reference valid node %s\n")%kbindmodel->getNode()));
                continue;
            }

            //  kinematics information
            daeElement* pelt = searchBinding(kbindmodel,kscene);
            domInstance_kinematics_modelRef kimodel = daeSafeCast<domInstance_kinematics_model>(pelt);
            if (!kimodel) {
                if( !pelt ) {
                    RAVELOG_WARN("bind_kinematics_model does not reference element\n");
                }
                else {
                    RAVELOG_WARN(str(boost::format("bind_kinematics_model does references %s\n")%pelt->getElementName()));
                }
                continue;
            }
            bindings.listKinematicsVisualBindings.push_back(make_pair(node,kimodel));
        }
        // axis info
        for (size_t ijoint = 0; ijoint < kiscene->getBind_joint_axis_array().getCount(); ++ijoint) {
            domBind_joint_axisRef bindjoint = kiscene->getBind_joint_axis_array()[ijoint];
            daeElementRef pjtarget = daeSidRef(bindjoint->getTarget(), bindjoint).resolve().elt;
            if (!pjtarget) {
                RAVELOG_ERRORA(str(boost::format("Target Node %s NOT found!!!\n")%bindjoint->getTarget()));
                continue;
            }
            daeElement* pelt = searchBinding(bindjoint->getAxis(),kscene);
            domAxis_constraintRef pjointaxis = daeSafeCast<domAxis_constraint>(pelt);
            if (!pjointaxis) {
                continue;
            }
            bindings.listAxisBindings.push_back(JointAxisBinding(pjtarget, pjointaxis, bindjoint->getValue(), NULL, NULL));
        }
    }

    bool _computeConvexHull(const vector<Vector>& verts, KinBody::Link::TRIMESH& trimesh)
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

    size_t _countChildren(daeElement* pelt) {
        size_t c = 1;
        for (size_t i = 0; i < pelt->getChildren().getCount(); ++i) {
            c += _countChildren(pelt->getChildren()[i]);
        }
        return c;
    }

    void _processUserData(daeElement* pelt, dReal scale)
    {
        // getChild could be optimized since asset tag is supposed to appear as the first element
        domAssetRef passet = daeSafeCast<domAsset> (pelt->getChild("asset"));
        if (passet != NULL && passet->getUnit() != NULL) {
            scale = passet->getUnit()->getMeter();
        }
        
        _vuserdata.push_back(USERDATA(scale));
        pelt->setUserData(&_vuserdata.back());

        for (size_t i = 0; i < pelt->getChildren().getCount(); ++i) {
            if (pelt->getChildren()[i] != passet) {
                _processUserData(pelt->getChildren()[i], scale);
            }
        }
    }

    USERDATA* _getUserData(daeElement* pelt)
    {
        BOOST_ASSERT(pelt != NULL);
        void* p = pelt->getUserData();
        BOOST_ASSERT(p != NULL );
        return (USERDATA*)p;
    }
        
    boost::shared_ptr<DAE> _collada;
    domCOLLADA* _dom;
    EnvironmentBasePtr _penv;
    vector<USERDATA> _vuserdata; // all userdata
    int _nGlobalSensorId, _nGlobalManipulatorId;
};

#endif
