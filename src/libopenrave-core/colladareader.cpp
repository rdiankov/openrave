// -*- coding: utf-8 -*-
// Copyright (C) 2006-2011 Rosen Diankov (rosen.diankov@gmail.com), Stefan Ulbrich, Gustavo Rodriguez
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
// functions that allow plugins to program for the RAVE simulator
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
#include <boost/lexical_cast.hpp>

class ColladaReader : public daeErrorHandler
{
    /// \brief bindings for instance models
    class ModelBinding
    {
public:
        ModelBinding(domNodeRef node, domInstance_kinematics_modelRef ikmodel) : _node(node), _ikmodel(ikmodel) {
        }
        domNodeRef _node;
        domInstance_kinematics_modelRef _ikmodel;
    };

    /// \brief bindings for joints between different specs
    class JointAxisBinding
    {
public:
        JointAxisBinding(daeElementRef pvisualtrans, domAxis_constraintRef pkinematicaxis, domCommon_float_or_paramRef jointvalue, domKinematics_axis_infoRef kinematics_axis_info, domMotion_axis_infoRef motion_axis_info) : pvisualtrans(pvisualtrans), pkinematicaxis(pkinematicaxis), jointvalue(jointvalue), kinematics_axis_info(kinematics_axis_info), motion_axis_info(motion_axis_info) {
            BOOST_ASSERT( !!pkinematicaxis );
            daeElement* pae = pvisualtrans->getParentElement();
            while (!!pae) {
                visualnode = daeSafeCast<domNode> (pae);
                if (!!visualnode) {
                    break;
                }
                pae = pae->getParentElement();
            }

            if (!visualnode) {
                RAVELOG_WARN(str(boost::format("couldn't find parent node of element id %s, sid %s\n")%pkinematicaxis->getID()%pkinematicaxis->getSid()));
            }
        }

        daeElementRef pvisualtrans;
        domAxis_constraintRef pkinematicaxis;
        domCommon_float_or_paramRef jointvalue;
        domNodeRef visualnode;
        domKinematics_axis_infoRef kinematics_axis_info;
        domMotion_axis_infoRef motion_axis_info;
    };

    /// \brief bindings for links between different specs
    class LinkBinding
    {
public:
        KinBody::LinkPtr _link;
        domNodeRef _node;
        domLinkRef _domlink;
        domInstance_rigid_bodyRef _irigidbody;
        domRigid_bodyRef _rigidbody;
        domNodeRef _nodephysicsoffset; // the physics rigid body is in this coordinate system
    };

    /// \brief inter-collada bindings for a kinematics scene
    class KinematicsSceneBindings
    {
public:
        std::list<ModelBinding> listModelBindings;
        std::list<JointAxisBinding> listAxisBindings;
        std::list<LinkBinding> listLinkBindings;

        bool AddAxisInfo(const domInstance_kinematics_model_Array& arr, domKinematics_axis_infoRef kinematics_axis_info, domMotion_axis_infoRef motion_axis_info)
        {
            if( !kinematics_axis_info ) {
                return false;
            }
            for(size_t ik = 0; ik < arr.getCount(); ++ik) {
                daeElement* pelt = daeSidRef(kinematics_axis_info->getAxis(), arr[ik]->getUrl().getElement()).resolve().elt;
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

public:
    ColladaReader(EnvironmentBasePtr penv) : _dom(NULL), _penv(penv), _nGlobalSensorId(0), _nGlobalManipulatorId(0), _nGlobalIndex(0)
    {
        daeErrorHandler::setErrorHandler(this);
        _bOpeningZAE = false;
        _bSkipGeometry = false;
        _fGlobalScale = 1;
    }
    virtual ~ColladaReader() {
        _dae.reset();
        DAE::cleanup();
    }

    bool InitFromFile(const string& filename,const AttributesList& atts)
    {
        RAVELOG_VERBOSE(str(boost::format("init COLLADA reader version: %s, namespace: %s, filename: %s\n")%COLLADA_VERSION%COLLADA_NAMESPACE%filename));
        _dae.reset(new DAE);
        _bOpeningZAE = filename.find(".zae") == filename.size()-4;
        _dom = _dae->open(filename);
        _bOpeningZAE = false;
        if (!_dom) {
            return false;
        }
        _filename=filename;
        return _Init(atts);
    }

    bool InitFromData(const string& pdata,const AttributesList& atts)
    {
        RAVELOG_DEBUG(str(boost::format("init COLLADA reader version: %s, namespace: %s\n")%COLLADA_VERSION%COLLADA_NAMESPACE));
        _dae.reset(new DAE);
        _dom = _dae->openFromMemory(".",pdata.c_str());
        if (!_dom) {
            return false;
        }
        return _Init(atts);
    }

    bool _Init(const AttributesList& atts)
    {
        _fGlobalScale = 1;
        if( !!_dom->getAsset() ) {
            if( !!_dom->getAsset()->getUnit() ) {
                _fGlobalScale = _dom->getAsset()->getUnit()->getMeter();
            }
        }
        _bSkipGeometry = false;
        FOREACHC(itatt,atts) {
            if( itatt->first == "skipgeometry" ) {
                _bSkipGeometry = stricmp(itatt->second.c_str(), "true") == 0 || itatt->second=="1";
            }
            else if( itatt->first == "scalegeometry" ) {
                stringstream ss(itatt->second);
                Vector v(1,1,1);
                ss >> v.x;
                _fGlobalScale *= v.x;
            }
            else if( itatt->first == "prefix" ) {
                _prefix = itatt->second;
            }
            else if( itatt->first == "name" ) {
                RAVELOG_VERBOSE(str(boost::format("collada reader robot name=%s is processed from xmlreaders side")%itatt->second));
            }
            else {
                RAVELOG_WARN(str(boost::format("collada reader unprocessed attribute pair: %s:%s")%itatt->first%itatt->second));
            }
        }
        return true;
    }

    /// \brief Extract all possible collada scene objects into the environment
    bool Extract()
    {
        domCOLLADA::domSceneRef allscene = _dom->getScene();
        if( !allscene ) {
            return false;
        }

        //  parse each instance kinematics scene
        vector<std::string>  vprocessednodes;
        for (size_t iscene = 0; iscene < allscene->getInstance_kinematics_scene_array().getCount(); iscene++) {
            domInstance_kinematics_sceneRef kiscene = allscene->getInstance_kinematics_scene_array()[iscene];
            domKinematics_sceneRef kscene = daeSafeCast<domKinematics_scene> (kiscene->getUrl().getElement().cast());
            if (!kscene) {
                continue;
            }

            daeElementRef kscenelib = _dom->getLibrary_kinematics_scenes_array()[0]->getKinematics_scene_array()[0];
            KinematicsSceneBindings bindings;
            _ExtractKinematicsVisualBindings(allscene->getInstance_visual_scene(),kiscene,bindings);
            _ExtractPhysicsBindings(allscene,bindings);
            FOREACH(itbinding,bindings.listModelBindings) {
                if( !!itbinding->_node->getId() ) {
                    vprocessednodes.push_back(itbinding->_node->getId());
                }
            }
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

            FOREACH(itlinkbinding,bindings.listLinkBindings) {
                if( !!itlinkbinding->_link && !!itlinkbinding->_node && !!itlinkbinding->_node->getID()) {
                    vprocessednodes.push_back(itlinkbinding->_node->getID());
                }
            }
        }

        // add left-over visual objects
        if (!!allscene->getInstance_visual_scene()) {
            domVisual_sceneRef visual_scene = daeSafeCast<domVisual_scene>(allscene->getInstance_visual_scene()->getUrl().getElement().cast());
            for (size_t node = 0; node < visual_scene->getNode_array().getCount(); node++) {
                KinBodyPtr pbody = _ExtractKinematicsModel(visual_scene->getNode_array()[node], KinematicsSceneBindings(),vprocessednodes);
                if( !!pbody ) {
                    _penv->AddKinBody(pbody, true);
                }
            }
        }

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

        _setInitialLinks.clear();
        _setInitialJoints.clear();
        _setInitialManipulators.clear();
        _setInitialSensors.clear();
        if( !!probot ) {
            FOREACH(itlink,probot->_veclinks) {
                _setInitialLinks.insert(*itlink);
            }
            FOREACH(itjoint,probot->_vecjoints) {
                _setInitialJoints.insert(*itjoint);
            }
            FOREACH(itjoint,probot->_vPassiveJoints) {
                _setInitialJoints.insert(*itjoint);
            }
            FOREACH(itmanip,probot->_vecManipulators) {
                _setInitialManipulators.insert(*itmanip);
            }
            FOREACH(itsensor,probot->_vecSensors) {
                _setInitialSensors.insert(*itsensor);
            }
        }

        // parse each instance kinematics scene, prioritize robots
        bool bSuccess = false;
        for (size_t iscene = 0; iscene < allscene->getInstance_kinematics_scene_array().getCount(); iscene++) {
            domInstance_kinematics_sceneRef kiscene = allscene->getInstance_kinematics_scene_array()[iscene];
            domKinematics_sceneRef kscene = daeSafeCast<domKinematics_scene> (kiscene->getUrl().getElement().cast());
            if (!kscene) {
                continue;
            }
            boost::shared_ptr<KinematicsSceneBindings> bindings(new KinematicsSceneBindings());
            _ExtractKinematicsVisualBindings(allscene->getInstance_visual_scene(),kiscene,*bindings);
            _ExtractPhysicsBindings(allscene,*bindings);
            for(size_t ias = 0; ias < kscene->getInstance_articulated_system_array().getCount(); ++ias) {
                if( ExtractArticulatedSystem(probot, kscene->getInstance_articulated_system_array()[ias], *bindings) && !!probot ) {
                    bSuccess = true;
                    break;
                }
            }
            if( bSuccess ) {
                break;
            }
            for(size_t ikmodel = 0; ikmodel < kscene->getInstance_kinematics_model_array().getCount(); ++ikmodel) {
                listPossibleBodies.push_back(make_pair(kscene->getInstance_kinematics_model_array()[ikmodel], bindings));
            }
        }

        KinBodyPtr pbody = probot;
        FOREACH(it, listPossibleBodies) {
            if( ExtractKinematicsModel(pbody, it->first, *it->second) && !!pbody ) {
                bSuccess = true;
                break;
            }
        }

        if( bSuccess ) {
            if( _prefix.size() > 0 ) {
                _AddPrefixForKinBody(probot,_prefix);
                FOREACH(itmanip,probot->_vecManipulators) {
                    if( _setInitialManipulators.find(*itmanip) == _setInitialManipulators.end()) {
                        (*itmanip)->_name = _prefix + (*itmanip)->_name;
                        FOREACH(itgrippername,(*itmanip)->_vgripperjointnames) {
                            *itgrippername = _prefix + *itgrippername;
                        }
                    }
                }
                FOREACH(itsensor, probot->_vecSensors) {
                    if( _setInitialSensors.find(*itsensor) == _setInitialSensors.end() ) {
                        (*itsensor)->_name = _prefix + (*itsensor)->_name;
                    }
                }
            }
        }
        return bSuccess;
    }

    bool Extract(KinBodyPtr& pbody)
    {
        domCOLLADA::domSceneRef allscene = _dom->getScene();
        if( !allscene ) {
            return false;
        }

        _setInitialLinks.clear();
        _setInitialJoints.clear();
        if( !!pbody ) {
            FOREACH(itlink,pbody->_veclinks) {
                _setInitialLinks.insert(*itlink);
            }
            FOREACH(itjoint,pbody->_vecjoints) {
                _setInitialJoints.insert(*itjoint);
            }
        }

        bool bSuccess = false;
        //  parse each instance kinematics scene for the first available model
        for (size_t iscene = 0; iscene < allscene->getInstance_kinematics_scene_array().getCount(); iscene++) {
            domInstance_kinematics_sceneRef kiscene = allscene->getInstance_kinematics_scene_array()[iscene];
            domKinematics_sceneRef kscene = daeSafeCast<domKinematics_scene> (kiscene->getUrl().getElement().cast());
            if (!kscene) {
                continue;
            }
            KinematicsSceneBindings bindings;
            _ExtractKinematicsVisualBindings(allscene->getInstance_visual_scene(),kiscene,bindings);
            _ExtractPhysicsBindings(allscene,bindings);
            for(size_t ikmodel = 0; ikmodel < kscene->getInstance_kinematics_model_array().getCount(); ++ikmodel) {
                if( ExtractKinematicsModel(pbody, kscene->getInstance_kinematics_model_array()[ikmodel], bindings) && !!pbody ) {
                    bSuccess = true;
                    break;
                }
            }
            if( bSuccess ) {
                break;
            }
        }
        if( bSuccess ) {
            return true;
        }

        // search for anything left over
        std::vector<std::string> vprocessednodes;
        if (!!allscene->getInstance_visual_scene()) {
            domVisual_sceneRef visual_scene = daeSafeCast<domVisual_scene>(allscene->getInstance_visual_scene()->getUrl().getElement().cast());
            for (size_t node = 0; node < visual_scene->getNode_array().getCount(); node++) {
                pbody = _ExtractKinematicsModel(visual_scene->getNode_array()[node], KinematicsSceneBindings(),vprocessednodes);
                if( !!pbody ) {
                    bSuccess = true;
                    break;
                }
            }
        }

        if( bSuccess && _prefix.size() > 0 ) {
            _AddPrefixForKinBody(pbody,_prefix);
        }
        return bSuccess;
    }

    void _AddPrefixForKinBody(KinBodyPtr pbody, const std::string& prefix)
    {
        FOREACH(itlink,pbody->_veclinks) {
            if( _setInitialLinks.find(*itlink) == _setInitialLinks.end()) {
                (*itlink)->_name = prefix + (*itlink)->_name;
            }
        }
        std::list<KinBody::JointPtr> listprocessjoints;
        std::vector< std::pair<std::string, std::string> > jointnamepairs; jointnamepairs.reserve(listprocessjoints.size());
        FOREACH(itjoint,pbody->_vecjoints) {
            if( _setInitialJoints.find(*itjoint) == _setInitialJoints.end()) {
                jointnamepairs.push_back(make_pair((*itjoint)->_name, prefix +(*itjoint)->_name));
                (*itjoint)->_name = prefix + (*itjoint)->_name;
                listprocessjoints.push_back(*itjoint);
            }
        }
        FOREACH(itjoint,pbody->_vPassiveJoints) {
            if( _setInitialJoints.find(*itjoint) == _setInitialJoints.end()) {
                jointnamepairs.push_back(make_pair((*itjoint)->_name, prefix +(*itjoint)->_name));
                (*itjoint)->_name = prefix + (*itjoint)->_name;
                listprocessjoints.push_back(*itjoint);
            }
        }
        // repeat again for the mimic equations, if any exist
        FOREACH(itjoint, listprocessjoints) {
            for(int idof = 0; idof < (*itjoint)->GetDOF(); ++idof) {
                if( (*itjoint)->IsMimic(idof) ) {
                    for(int ieq = 0; ieq < 3; ++ieq) {
                        string neweq;
                        SearchAndReplace(neweq,(*itjoint)->_vmimic[idof]->_equations[ieq],jointnamepairs);
                        (*itjoint)->_vmimic[idof]->_equations[ieq] = neweq;
                    }
                }
            }
        }
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
            if( !probot ) {
                probot = RaveCreateRobot(_penv,"genericrobot");
                if( !probot ) {
                    probot = RaveCreateRobot(_penv,"");
                    RAVELOG_WARN("creating default robot with no controller support\n");
                }
            }
            _mapJointUnits.clear();
            _mapJointIds.clear();
        }
        if( probot->__struri.size() == 0 ) {
            probot->__struri = _filename;
        }

        // set the name
        if(( probot->GetName().size() == 0) && !!ias->getName() ) {
            probot->SetName(ias->getName());
        }
        if(( probot->GetName().size() == 0) && !!ias->getSid()) {
            probot->SetName(ias->getSid());
        }
        if(( probot->GetName().size() == 0) && !!articulated_system->getName() ) {
            probot->SetName(articulated_system->getName());
        }
        if(( probot->GetName().size() == 0) && !!articulated_system->getId()) {
            probot->SetName(articulated_system->getId());
        }

        if( !!articulated_system->getMotion() ) {
            domInstance_articulated_systemRef ias_new = articulated_system->getMotion()->getInstance_articulated_system();
            if( !!articulated_system->getMotion()->getTechnique_common() ) {
                for(size_t i = 0; i < articulated_system->getMotion()->getTechnique_common()->getAxis_info_array().getCount(); ++i) {
                    domMotion_axis_infoRef motion_axis_info = articulated_system->getMotion()->getTechnique_common()->getAxis_info_array()[i];
                    // this should point to a kinematics axis_info
                    domKinematics_axis_infoRef kinematics_axis_info = daeSafeCast<domKinematics_axis_info>(daeSidRef(motion_axis_info->getAxis(), ias_new->getUrl().getElement()).resolve().elt);
                    if( !!kinematics_axis_info ) {
                        // find the parent kinematics and go through all its instance kinematics models
                        daeElement* pparent = kinematics_axis_info->getParent();
                        while(!!pparent && pparent->typeID() != domKinematics::ID()) {
                            pparent = pparent->getParent();
                        }
                        BOOST_ASSERT(!!pparent);
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
                if( !probot ) {
                    probot = RaveCreateRobot(_penv, "");
                }
                _mapJointUnits.clear();
                _mapJointIds.clear();
            }
            BOOST_ASSERT(probot->IsRobot());

            KinBodyPtr pbody = probot;
            for(size_t ik = 0; ik < articulated_system->getKinematics()->getInstance_kinematics_model_array().getCount(); ++ik) {
                ExtractKinematicsModel(pbody,articulated_system->getKinematics()->getInstance_kinematics_model_array()[ik],bindings);
            }
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

        if( !pkinbody ) {
            boost::shared_ptr<std::string> pinterface_type = _ExtractInterfaceType(ikm->getExtra_array());
            if( !pinterface_type ) {
                pinterface_type = _ExtractInterfaceType(kmodel->getExtra_array());
            }
            if( !!pinterface_type ) {
                pkinbody = RaveCreateKinBody(_penv,*pinterface_type);
            }
            if( !pkinbody ) {
                pkinbody = RaveCreateKinBody(_penv,"");
            }
            _mapJointUnits.clear();
            _mapJointIds.clear();
        }
        if( pkinbody->__struri.size() == 0 ) {
            pkinbody->__struri = _filename;
        }

        // find matching visual node
        domNodeRef pvisualnode;
        FOREACH(it, bindings.listModelBindings) {
            if( it->_ikmodel == ikm ) {
                pvisualnode = it->_node;
                break;
            }
        }
        if( !pvisualnode ) {
            RAVELOG_WARN(str(boost::format("failed to find visual node for instance kinematics model %s\n")%ikm->getSid()));
            return false;
        }

        if(( pkinbody->GetName().size() == 0) && !!ikm->getName() ) {
            pkinbody->SetName(ikm->getName());
        }
        if(( pkinbody->GetName().size() == 0) && !!ikm->getID() ) {
            pkinbody->SetName(ikm->getID());
        }

        if (!_ExtractKinematicsModel(pkinbody, kmodel, pvisualnode, bindings)) {
            RAVELOG_WARN(str(boost::format("failed to load kinbody from kinematics model %s\n")%kmodel->getID()));
            return false;
        }
        return true;
    }

    /// \brief extract one rigid link composed of the node hierarchy
    KinBodyPtr _ExtractKinematicsModel(domNodeRef pdomnode, const KinematicsSceneBindings& bindings, const std::vector<std::string>& vprocessednodes)
    {
        if( !!pdomnode->getID() &&( find(vprocessednodes.begin(),vprocessednodes.end(),pdomnode->getID()) != vprocessednodes.end()) ) {
            return KinBodyPtr();
        }
        _mapJointUnits.clear();
        _mapJointIds.clear();
        KinBodyPtr pkinbody = RaveCreateKinBody(_penv);
        if( pkinbody->__struri.size() == 0 ) {
            pkinbody->__struri = _filename;
        }
        string name = !pdomnode->getName() ? "" : _ConvertToOpenRAVEName(pdomnode->getName());
        if( name.size() == 0 ) {
            name = _ConvertToOpenRAVEName(pdomnode->getID());
        }
        KinBody::LinkPtr plink(new KinBody::Link(pkinbody));
        plink->_name = name;
        plink->_mass = 1.0;
        plink->_bStatic = false;
        bool bhasgeometry = ExtractGeometry(pdomnode,plink,bindings,vprocessednodes);
        if( !bhasgeometry ) {
            return KinBodyPtr();
        }

        RAVELOG_INFO(str(boost::format("Loading non-kinematics node '%s'")%name));
        pkinbody->SetName(name);
        plink->_index = (int) pkinbody->_veclinks.size();
        pkinbody->_veclinks.push_back(plink);
        return pkinbody;
    }

    /// \brief append the kinematics model to the openrave kinbody
    bool _ExtractKinematicsModel(KinBodyPtr& pkinbody, domKinematics_modelRef kmodel, domNodeRef pnode, KinematicsSceneBindings& bindings)
    {
        vector<domJointRef> vdomjoints;
        if (!pkinbody) {
            pkinbody = RaveCreateKinBody(_penv);
            _mapJointUnits.clear();
            _mapJointIds.clear();
        }
        if(( pkinbody->GetName().size() == 0) && !!kmodel->getName() ) {
            pkinbody->SetName(kmodel->getName());
        }
        if(( pkinbody->GetName().size() == 0) && !!kmodel->getID() ) {
            pkinbody->SetName(kmodel->getID());
        }
        RAVELOG_DEBUG(str(boost::format("kinematics model: %s\n")%pkinbody->GetName()));
        if( !!pnode ) {
            RAVELOG_DEBUG(str(boost::format("node name: %s\n")%pnode->getId()));
        }
        if( !kmodel->getID() ) {
            RAVELOG_DEBUG(str(boost::format("kinematics model: %s has no id attribute!\n")%pkinbody->GetName()));
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
                RAVELOG_WARN("failed to get joint from instance\n");
            }
            else {
                vdomjoints.push_back(pelt);
            }
        }

        RAVELOG_VERBOSE(str(boost::format("Number of root links in the kmodel %d\n")%ktec->getLink_array().getCount()));
        for (size_t ilink = 0; ilink < ktec->getLink_array().getCount(); ++ilink) {
            ExtractLink(pkinbody, ktec->getLink_array()[ilink], ilink == 0 ? pnode : domNodeRef(), Transform(), vdomjoints, bindings);
        }

        for (size_t iform = 0; iform < ktec->getFormula_array().getCount(); ++iform) {
            domFormulaRef pf = ktec->getFormula_array()[iform];
            if (!pf->getTarget()) {
                RAVELOG_WARN("formula target not valid\n");
                continue;
            }

            // find the target joint
            KinBody::JointPtr pjoint = _getJointFromRef(pf->getTarget()->getParam()->getValue(),pf,pkinbody).first;
            if (!pjoint) {
                continue;
            }

            if( pjoint->GetDOF() > 1 ) {
                RAVELOG_WARN("collada cannot parse joint formulas when joint dof is greater than 1\n");
            }

            int iaxis = 0;
            pjoint->_vmimic[iaxis].reset(new KinBody::Joint::MIMIC());
            dReal ftargetunit = 1;
            if(_mapJointUnits.find(pjoint) != _mapJointUnits.end() ) {
                ftargetunit = _mapJointUnits[pjoint].at(iaxis);
            }

            daeTArray<daeElementRef> children;
            domTechniqueRef popenravetec = _ExtractOpenRAVEProfile(pf->getTechnique_array());
            if( !!popenravetec ) {
                for(size_t ic = 0; ic < popenravetec->getContents().getCount(); ++ic) {
                    daeElementRef pequation = popenravetec->getContents()[ic];
                    if( pequation->getElementName() == string("equation") ) {
                        children.clear();
                        pequation->getChildren(children);
                        if( !pequation->hasAttribute("type") ) {
                            RAVELOG_WARN("equaiton needs 'type' attribute, ignoring\n");
                            continue;
                        }
                        if( children.getCount() != 1 ) {
                            RAVELOG_WARN("equaiton needs exactly one child\n");
                            continue;
                        }
                        std::string equationtype = pequation->getAttribute("type");
                        KinBody::JointPtr pjointtarget;
                        if( pequation->hasAttribute("target") ) {
                            pjointtarget = _getJointFromRef(pequation->getAttribute("target").c_str(),pf,pkinbody).first;
                        }
                        try {
                            std::string eq = _ExtractMathML(pf,pkinbody,children[0]);
                            if( ftargetunit != 1 ) {
                                eq = str(boost::format("%f*(%s)")%ftargetunit%eq);
                            }
                            if( equationtype == "position" ) {
                                pjoint->_vmimic[iaxis]->_equations[0] = eq;
                            }
                            else if( equationtype == "first_partial" ) {
                                if( !pjointtarget ) {
                                    RAVELOG_WARN(str(boost::format("first_partial equation '%s' needs a target attribute! ignoring...\n")%eq));
                                    continue;
                                }
                                pjoint->_vmimic[iaxis]->_equations[1] += str(boost::format("|%s %s ")%pjointtarget->GetName()%eq);
                            }
                            else if( equationtype == "second_partial" ) {
                                if( !pjointtarget ) {
                                    RAVELOG_WARN(str(boost::format("second_partial equation '%s' needs a target attribute! ignoring...\n")%eq));
                                    continue;
                                }
                                pjoint->_vmimic[iaxis]->_equations[2] += str(boost::format("|%s %s ")%pjointtarget->GetName()%eq);
                            }
                            else {
                                RAVELOG_WARN(str(boost::format("unknown equation type %s")%equationtype));
                            }
                        }
                        catch(const openrave_exception &ex) {
                            RAVELOG_WARN(str(boost::format("failed to parse formula %s for target %s")%equationtype%pjoint->GetName()));
                        }
                    }
                }
            }
            else if (!!pf->getTechnique_common()) {
                try {
                    pf->getTechnique_common()->getChildren(children);
                    for(size_t ic = 0; ic < children.getCount(); ++ic) {
                        string eq = _ExtractMathML(pf,pkinbody,children[ic]);
                        if( ftargetunit != 1 ) {
                            eq = str(boost::format("%f*(%s)")%ftargetunit%eq);
                        }
                        if( eq.size() > 0 ) {
                            pjoint->_vmimic[iaxis]->_equations[0] = eq;
                            break;
                        }
                    }
                }
                catch(const openrave_exception &ex) {
                    RAVELOG_WARN(str(boost::format("failed to parse formula for target %s: %s")%pjoint->GetName()%ex.what()));
                }
            }
        }

        // read the collision data
        for(size_t ie = 0; ie < kmodel->getExtra_array().getCount(); ++ie) {
            domExtraRef pextra = kmodel->getExtra_array()[ie];
            if( strcmp(pextra->getType(), "collision") == 0 ) {
                domTechniqueRef tec = _ExtractOpenRAVEProfile(pextra->getTechnique_array());
                if( !!tec ) {
                    for(size_t ic = 0; ic < tec->getContents().getCount(); ++ic) {
                        daeElementRef pelt = tec->getContents()[ic];
                        if( pelt->getElementName() == string("ignore_link_pair") ) {
                            domLinkRef pdomlink0 = daeSafeCast<domLink>(daeSidRef(pelt->getAttribute("link0"), kmodel).resolve().elt);
                            domLinkRef pdomlink1 = daeSafeCast<domLink>(daeSidRef(pelt->getAttribute("link1"), kmodel).resolve().elt);
                            if( !pdomlink0 || !pdomlink1 ) {
                                RAVELOG_WARN(str(boost::format("failed to reference <ignore_link_pair> links: %s %s\n")%pelt->getAttribute("link0")%pelt->getAttribute("link1")));
                                continue;
                            }
                            KinBody::LinkPtr plink0 = pkinbody->GetLink(_ExtractLinkName(pdomlink0));
                            KinBody::LinkPtr plink1 = pkinbody->GetLink(_ExtractLinkName(pdomlink1));
                            if( !plink0 && !plink1 ) {
                                RAVELOG_WARN("failed to find openrave links from <ignore_link_pair>\n");
                                continue;
                            }
                            pkinbody->_vForcedAdjacentLinks.push_back(make_pair(plink0->GetName(),plink1->GetName()));
                        }
                        else if( pelt->getElementName() == string("bind_instance_geometry") ) {
                            RAVELOG_WARN("currently do not support bind_instance_geometry\n");
                        }
                    }
                }
            }
        }
        return true;
    }

    ///  \brief Extract Link info and add it to an existing body
    KinBody::LinkPtr ExtractLink(KinBodyPtr pkinbody, const domLinkRef pdomlink,const domNodeRef pdomnode, const Transform& tParentLink, const std::vector<domJointRef>& vdomjoints, KinematicsSceneBindings& bindings)
    {
        //  Set link name with the name of the COLLADA's Link
        std::string linkname;
        if( !!pdomlink ) {
            linkname = _ExtractLinkName(pdomlink);
            if( linkname.size() == 0 ) {
                RAVELOG_WARN("<link> has no name or id, falling back to <node>!\n");
            }
        }
        if( linkname.size() == 0 ) {
            if( !!pdomnode ) {
                if (!!pdomnode->getName()) {
                    linkname = _ConvertToOpenRAVEName(pdomnode->getName());
                }
                if(( linkname.size() == 0) && !!pdomnode->getID()) {
                    linkname = _ConvertToOpenRAVEName(pdomnode->getID());
                }
            }
        }

        KinBody::LinkPtr plink = pkinbody->GetLink(linkname);
        if( !plink ) {
            plink.reset(new KinBody::Link(pkinbody));
            plink->_name = linkname;
            plink->_mass = 1e-10;
            plink->_bStatic = false;
            plink->_index = (int) pkinbody->_veclinks.size();
            pkinbody->_veclinks.push_back(plink);
        }
        else {
            RAVELOG_DEBUG(str(boost::format("found previously defined link '%s")%linkname));
        }

        plink->_t = tParentLink;
        // use the kinematics coordinate system for each link
        if( !!pdomlink ) {
            plink->_t = plink->_t * _ExtractFullTransform(pdomlink);
        }

        domInstance_rigid_bodyRef irigidbody;
        domRigid_bodyRef rigidbody;
        Transform trigidoffset = pkinbody->GetLinks().at(0)->GetTransform();
        if( !!pdomnode ) {
            RAVELOG_VERBOSE(str(boost::format("Node Id %s and Name %s\n")%pdomnode->getId()%pdomnode->getName()));

            bool bFoundBinding = false;
            FOREACH(itlinkbinding, bindings.listLinkBindings) {
                if( !!pdomnode->getID() && !!itlinkbinding->_node->getID() && strcmp(pdomnode->getID(),itlinkbinding->_node->getID()) == 0 ) {
                    bFoundBinding = true;
                    irigidbody = itlinkbinding->_irigidbody;
                    rigidbody = itlinkbinding->_rigidbody;
                    itlinkbinding->_domlink = pdomlink;
                    itlinkbinding->_link = plink;
                    if( !!itlinkbinding->_nodephysicsoffset ) {
                        // set the rigid offset to the transform of the link that the node points to
                        FOREACH(itlinkbinding2, bindings.listLinkBindings) {
                            if( !!itlinkbinding2->_node->getID() && strcmp(itlinkbinding2->_node->getID(),itlinkbinding->_nodephysicsoffset->getID()) ) {
                                if( !!itlinkbinding2->_link ) {
                                    trigidoffset = itlinkbinding2->_link->_t;
                                }
                                break;
                            }
                        }
                    }
                    break;
                }
            }
            if( !bFoundBinding ) {
                LinkBinding lb;
                lb._node = pdomnode;
                lb._link = plink;
                bindings.listLinkBindings.push_back(lb);
            }
        }

        if( !!rigidbody && !!rigidbody->getTechnique_common() ) {
            domRigid_body::domTechnique_commonRef rigiddata = rigidbody->getTechnique_common();
            if( !!rigiddata->getMass() ) {
                plink->_mass = rigiddata->getMass()->getValue();
            }
            Transform tmassframe = trigidoffset;
            if( !!rigiddata->getMass_frame() ) {
                tmassframe *= _ExtractFullTransform(rigiddata->getMass_frame());
            }
            TransformMatrix minertia;
            if( !!rigiddata->getInertia() ) {
                minertia.m[0] = rigiddata->getInertia()->getValue()[0];
                minertia.m[5] = rigiddata->getInertia()->getValue()[1];
                minertia.m[10] = rigiddata->getInertia()->getValue()[2];
            }
            TransformMatrix transMass = plink->_t.inverse() * tmassframe;
            plink->_transMass = transMass*minertia*transMass.inverse();
            plink->_transMass.trans = transMass.trans;
            if( !!rigiddata->getDynamic() ) {
                plink->_bStatic = !rigiddata->getDynamic()->getValue();
            }
        }

        if (!pdomlink) {
            ExtractGeometry(pdomnode,plink,bindings,std::vector<std::string>());
        }
        else {
            RAVELOG_DEBUG(str(boost::format("Attachment link elements: %d\n")%pdomlink->getAttachment_full_array().getCount()));

            {
                stringstream ss; ss << plink->GetName() << ": " << plink->_t << endl;
                RAVELOG_DEBUG(ss.str());
            }

            // Get the geometry
            if( !ExtractGeometry(pdomnode,plink,bindings,std::vector<std::string>()) ) {
                RAVELOG_DEBUG(str(boost::format("link %s has no geometry\n")%plink->GetName()));
            }

            RAVELOG_DEBUG(str(boost::format("After ExtractGeometry Attachment link elements: %d\n")%pdomlink->getAttachment_full_array().getCount()));

            //  Process all atached links
            for (size_t iatt = 0; iatt < pdomlink->getAttachment_full_array().getCount(); ++iatt) {
                domLink::domAttachment_fullRef pattfull = pdomlink->getAttachment_full_array()[iatt];

                // get link kinematics transformation
                TransformMatrix tatt = _ExtractFullTransform(pattfull);

                // process attached links
                daeElement* peltjoint = daeSidRef(pattfull->getJoint(), pattfull).resolve().elt;
                if( !peltjoint ) {
                    RAVELOG_WARN(str(boost::format("could not find attached joint %s!\n")%pattfull->getJoint()));
                    continue;
                }
                string jointid;
                if( string(pattfull->getJoint()).find("./") == 0 ) {
                    jointid = str(boost::format("%s/%s")%_ExtractParentId(pattfull)%&pattfull->getJoint()[1]);
                }
                else {
                    jointid = pattfull->getJoint();
                }

                domJointRef pdomjoint = daeSafeCast<domJoint> (peltjoint);
                if (!pdomjoint) {
                    domInstance_jointRef pdomijoint = daeSafeCast<domInstance_joint> (peltjoint);
                    if (!!pdomijoint) {
                        pdomjoint = daeSafeCast<domJoint> (pdomijoint->getUrl().getElement().cast());
                    }
                    else {
                        RAVELOG_WARN(str(boost::format("could not cast element <%s> to <joint>!\n")%peltjoint->getElementName()));
                        continue;
                    }
                }

                // get direct child link
                if (!pattfull->getLink()) {
                    RAVELOG_WARN(str(boost::format("joint %s needs to be attached to a valid link\n")%jointid));
                    continue;
                }

                // find the correct joint in the bindings
                daeTArray<domAxis_constraintRef> vdomaxes = pdomjoint->getChildrenByType<domAxis_constraint>();
                domNodeRef pchildnode;

                // see if joint has a binding to a visual node
                FOREACHC(itaxisbinding,bindings.listAxisBindings) {
                    for (size_t ic = 0; ic < vdomaxes.getCount(); ++ic) {
                        //  If the binding for the joint axis is found, retrieve the info
                        if (vdomaxes[ic] == itaxisbinding->pkinematicaxis) {
                            pchildnode = itaxisbinding->visualnode;
                            break;
                        }
                    }
                    if( !!pchildnode ) {
                        break;
                    }
                }
                if (!pchildnode) {
                    RAVELOG_DEBUG(str(boost::format("joint %s has no visual binding\n")%jointid));
                }

                // create the joints before creating the child links
                KinBody::JointPtr pjoint(new KinBody::Joint(pkinbody));
                int jointtype = vdomaxes.getCount();
                pjoint->_bActive = true;     // if not active, put into the passive list
                FOREACH(it,pjoint->_vweights) {
                    *it = 1;
                }
                std::vector<dReal> vaxisunits(vdomaxes.getCount(),dReal(1));
                for (size_t ic = 0; ic < vdomaxes.getCount(); ++ic) {
                    FOREACHC(itaxisbinding,bindings.listAxisBindings) {
                        if (vdomaxes[ic] == itaxisbinding->pkinematicaxis) {
                            if( !!itaxisbinding->kinematics_axis_info ) {
                                if( !!itaxisbinding->kinematics_axis_info->getActive() ) {
                                    // what if different axes have different active profiles?
                                    pjoint->_bActive = resolveBool(itaxisbinding->kinematics_axis_info->getActive(),itaxisbinding->kinematics_axis_info);
                                }
                            }
                            break;
                        }
                    }
                    domAxis_constraintRef pdomaxis = vdomaxes[ic];
                    if( strcmp(pdomaxis->getElementName(), "revolute") == 0 ) {
                        pjoint->_type = KinBody::Joint::JointRevolute;
                        pjoint->_vmaxvel[ic] = 0.5;
                    }
                    else if( strcmp(pdomaxis->getElementName(), "prismatic") == 0 ) {
                        pjoint->_type = KinBody::Joint::JointPrismatic;
                        vaxisunits[ic] = _GetUnitScale(pdomaxis,_fGlobalScale);
                        jointtype |= 1<<(4+ic);
                        pjoint->_vmaxvel[ic] = 0.01;
                    }
                    else {
                        RAVELOG_WARN(str(boost::format("unsupported joint type: %s\n")%pdomaxis->getElementName()));
                    }
                }

                pjoint->_type = (KinBody::Joint::JointType)jointtype;
                _mapJointUnits[pjoint] = vaxisunits;
                if( pjoint->_bActive ) {
                    pjoint->jointindex = (int) pkinbody->_vecjoints.size();
                    pjoint->dofindex = pkinbody->GetDOF();
                }
                if( !!pdomjoint->getName() ) {
                    pjoint->_name = _ConvertToOpenRAVEName(pdomjoint->getName());
                }
                else {
                    pjoint->_name = str(boost::format("dummy%d")%pjoint->jointindex);
                }

                if( pjoint->_bActive ) {
                    pkinbody->_vecjoints.push_back(pjoint);
                }
                else {
                    RAVELOG_VERBOSE(str(boost::format("joint %s is passive\n")%pjoint->_name));
                    pkinbody->_vPassiveJoints.push_back(pjoint);
                }

                if( _mapJointIds.find(jointid) != _mapJointIds.end() ) {
                    RAVELOG_WARN(str(boost::format("jointid '%s' is duplicated!")%jointid));
                }
                _mapJointIds[jointid] = pjoint;
                RAVELOG_DEBUG(str(boost::format("joint %s (%d:%d)")%pjoint->_name%pjoint->jointindex%pjoint->dofindex));

                KinBody::LinkPtr pchildlink = ExtractLink(pkinbody, pattfull->getLink(), pchildnode, plink->_t * tatt, vdomjoints, bindings);

                if (!pchildlink) {
                    RAVELOG_WARN(str(boost::format("Link '%s' has no child link, creating dummy link\n")%plink->GetName()));
                    // create dummy child link
                    stringstream ss;
                    ss << plink->_name;
                    ss <<"_dummy" << pkinbody->_veclinks.size();
                    pchildlink.reset(new KinBody::Link(pkinbody));
                    pchildlink->_name = ss.str();
                    pchildlink->_bStatic = false;
                    pchildlink->_index = (int)pkinbody->_veclinks.size();
                    pkinbody->_veclinks.push_back(pchildlink);
                }

                std::vector<Vector> vAxes(vdomaxes.getCount());
                for (size_t ic = 0; ic < vdomaxes.getCount(); ++ic) {
                    domKinematics_axis_infoRef kinematics_axis_info;
                    domMotion_axis_infoRef motion_axis_info;
                    FOREACHC(itaxisbinding,bindings.listAxisBindings) {
                        if (vdomaxes[ic] == itaxisbinding->pkinematicaxis) {
                            kinematics_axis_info = itaxisbinding->kinematics_axis_info;
                            motion_axis_info = itaxisbinding->motion_axis_info;
                            break;
                        }
                    }
                    domAxis_constraintRef pdomaxis = vdomaxes[ic];

                    //  Axes and Anchor assignment.
                    vAxes[ic] = Vector(pdomaxis->getAxis()->getValue()[0], pdomaxis->getAxis()->getValue()[1], pdomaxis->getAxis()->getValue()[2]);
                    if( vAxes[ic].lengthsqr3() > 0 ) {
                        vAxes[ic].normalize3();
                    }
                    else {
                        vAxes[ic] = Vector(0,0,1);
                    }

                    pjoint->_voffsets[ic] = 0;     // to overcome -pi to pi boundary
                    if (pkinbody->IsRobot() && !motion_axis_info) {
                        RAVELOG_WARN(str(boost::format("No motion axis info for joint %s\n")%pjoint->GetName()));
                    }

                    //  Sets the Speed and the Acceleration of the joint
                    if (!!motion_axis_info) {
                        if (!!motion_axis_info->getSpeed()) {
                            pjoint->_vmaxvel[ic] = resolveFloat(motion_axis_info->getSpeed(),motion_axis_info);
                            RAVELOG_VERBOSE("... Joint Speed: %f...\n",pjoint->GetMaxVel());
                        }
                        if (!!motion_axis_info->getAcceleration()) {
                            pjoint->_vmaxaccel[ic] = resolveFloat(motion_axis_info->getAcceleration(),motion_axis_info);
                            RAVELOG_VERBOSE("... Joint Acceleration: %f...\n",pjoint->GetMaxAccel());
                        }
                    }

                    bool joint_locked = false;     // if locked, joint angle is static
                    bool has_soft_limits = false;
                    bool has_hard_limits = false;

                    if (!!kinematics_axis_info) {
                        // contains the soft controller limits
                        if (!!kinematics_axis_info->getLocked()) {
                            joint_locked = resolveBool(kinematics_axis_info->getLocked(),kinematics_axis_info);
                        }

                        if (joint_locked) {     // If joint is locked set limits to the static value.
                            RAVELOG_WARN("lock joint!!\n");
                            pjoint->_vlowerlimit.at(ic) = 0;
                            pjoint->_vupperlimit.at(ic) = 0;
                        }
                        else if (!!kinematics_axis_info->getLimits()) {     // If there are articulated system kinematics limits
                            has_soft_limits = true;
                            dReal fscale = pjoint->IsRevolute(ic) ? (PI/180.0f) : _GetUnitScale(kinematics_axis_info,_fGlobalScale);
                            pjoint->_vlowerlimit.at(ic) = fscale*(dReal)(resolveFloat(kinematics_axis_info->getLimits()->getMin(),kinematics_axis_info));
                            pjoint->_vupperlimit.at(ic) = fscale*(dReal)(resolveFloat(kinematics_axis_info->getLimits()->getMax(),kinematics_axis_info));
                            if( pjoint->IsRevolute(ic) ) {
                                if(( pjoint->_vlowerlimit.at(ic) < -PI) ||( pjoint->_vupperlimit[ic] > PI) ) {
                                    pjoint->_voffsets[ic] = 0.5f * (pjoint->_vlowerlimit.at(ic) + pjoint->_vupperlimit[ic]);
                                    if( pjoint->_vupperlimit[ic] - pjoint->_voffsets[ic] > PI ) {
                                        RAVELOG_WARN(str(boost::format("joint %s, cannot allow joint range [%f,%f] of more than 2*pi radians\n")%pjoint->GetName()%pjoint->_vlowerlimit.at(ic)%pjoint->_vupperlimit[ic]));
                                        pjoint->_vupperlimit.at(ic) = pjoint->_voffsets[ic] + PI - 1e-5;
                                        pjoint->_vlowerlimit.at(ic) = pjoint->_voffsets[ic] - PI + 1e-5;
                                    }
                                }
                            }
                        }
                    }

                    if (!joint_locked && !!pdomaxis->getLimits() ) {
                        has_hard_limits = true;
                        // contains the hard limits (prioritize over soft limits)
                        RAVELOG_VERBOSE(str(boost::format("There are LIMITS in joint %s ...\n")%pjoint->GetName()));
                        dReal fscale = pjoint->IsRevolute(ic) ? (PI/180.0f) : _GetUnitScale(pdomaxis,_fGlobalScale);
                        pjoint->_vlowerlimit.at(ic) = (dReal)pdomaxis->getLimits()->getMin()->getValue()*fscale;
                        pjoint->_vupperlimit.at(ic) = (dReal)pdomaxis->getLimits()->getMax()->getValue()*fscale;
                        if( pjoint->IsRevolute(ic) ) {
                            if(( pjoint->_vlowerlimit[ic] < -PI) ||( pjoint->_vupperlimit[ic] > PI) ) {
                                pjoint->_voffsets[ic] = 0.5f * (pjoint->_vlowerlimit[ic] + pjoint->_vupperlimit[ic]);
                                if( pjoint->_vupperlimit[ic] - pjoint->_voffsets[ic] > PI ) {
                                    RAVELOG_WARN(str(boost::format("joint %s, cannot allow joint range [%f,%f] of more than 2*pi radians\n")%pjoint->GetName()%pjoint->_vlowerlimit[ic]%pjoint->_vupperlimit[ic]));
                                    pjoint->_vupperlimit[ic] = pjoint->_voffsets[ic] + PI - 1e-5;
                                    pjoint->_vlowerlimit[ic] = pjoint->_voffsets[ic] - PI + 1e-5;
                                }
                            }
                        }
                    }

                    if( !has_soft_limits && !has_hard_limits && !joint_locked ) {
                        RAVELOG_VERBOSE(str(boost::format("There are NO LIMITS in joint %s ...\n")%pjoint->GetName()));
                        if( pjoint->IsRevolute(ic) ) {
                            pjoint->_bIsCircular.at(ic) = true;
                            pjoint->_vlowerlimit.at(ic) = -PI;
                            pjoint->_vupperlimit.at(ic) = PI;
                        }
                        else {
                            pjoint->_vlowerlimit.at(ic) =-1000000;
                            pjoint->_vupperlimit.at(ic) = 1000000;
                        }
                    }

                    //  Rotate axis from the parent offset
                    vAxes[ic] = tatt.rotate(vAxes[ic]);
                }
                RAVELOG_DEBUG(str(boost::format("joint dof: %d, link %s\n")%pjoint->dofindex%plink->GetName()));
                pjoint->_ComputeInternalInformation(plink,pchildlink,tatt.trans,vAxes,std::vector<dReal>());
            }
            if( pdomlink->getAttachment_start_array().getCount() > 0 ) {
                RAVELOG_WARN("openrave collada reader does not support attachment_start\n");
            }
            if( pdomlink->getAttachment_end_array().getCount() > 0 ) {
                RAVELOG_WARN("openrave collada reader does not support attachment_end\n");
            }
        }
        return plink;
    }

    /// Extract Geometry and apply the transformations of the node
    /// \param pdomnode Node to extract the goemetry
    /// \param plink    Link of the kinematics model
    bool ExtractGeometry(const domNodeRef pdomnode,KinBody::LinkPtr plink, const KinematicsSceneBindings& bindings, const std::vector<std::string>& vprocessednodes)
    {
        if( !pdomnode ) {
            return false;
        }
        if( !!pdomnode->getID() &&( find(vprocessednodes.begin(),vprocessednodes.end(),pdomnode->getID()) != vprocessednodes.end()) ) {
            return false;
        }
        if( _bSkipGeometry ) {
            return false;
        }

        RAVELOG_VERBOSE(str(boost::format("ExtractGeometry(node,link) of %s\n")%pdomnode->getName()));

        bool bhasgeometry = false;
        // For all child nodes of pdomnode
        for (size_t i = 0; i < pdomnode->getNode_array().getCount(); i++) {
            // check if contains a joint
            bool contains=false;
            FOREACHC(it,bindings.listAxisBindings) {
                // don't check ID's check if the reference is the same!
                if ( (pdomnode->getNode_array()[i])  == (it->visualnode)) {
                    contains=true;
                    break;
                }
            }
            if (contains) {
                continue;
            }

            bhasgeometry |= ExtractGeometry(pdomnode->getNode_array()[i],plink, bindings, vprocessednodes);
            // Plink stayes the same for all children
            // replace pdomnode by child = pdomnode->getNode_array()[i]
            // hope for the best!
            // put everything in a subroutine in order to process pdomnode too!
        }

        size_t nGeomBefore =  plink->_listGeomProperties.size();     // #of Meshes already associated to this link

        // get the geometry
        for (size_t igeom = 0; igeom < pdomnode->getInstance_geometry_array().getCount(); ++igeom) {
            domInstance_geometryRef domigeom = pdomnode->getInstance_geometry_array()[igeom];
            domGeometryRef domgeom = daeSafeCast<domGeometry> (domigeom->getUrl().getElement());
            if (!domgeom) {
                RAVELOG_WARN("link %s does not have valid geometry\n", plink->GetName().c_str());
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
            bhasgeometry |= ExtractGeometry(domgeom, mapmaterials, plink);
        }

        if( !bhasgeometry ) {
            RAVELOG_DEBUG(str(boost::format("node %s has no geometry\n")%pdomnode->getName()));
            return false;
        }

        TransformMatrix tmnodegeom = (TransformMatrix) plink->_t.inverse() * getNodeParentTransform(pdomnode) * _ExtractFullTransform(pdomnode);
        Transform tnodegeom;
        Vector vscale;
        decompose(tmnodegeom, tnodegeom, vscale);

        list<KinBody::Link::GEOMPROPERTIES>::iterator itgeom= plink->_listGeomProperties.begin();
        for (unsigned int i=0; i< nGeomBefore; i++) {
            itgeom++;     // change only the transformations of the newly found geometries.
        }

        //  Switch between different type of geometry PRIMITIVES
        for (; itgeom != plink->_listGeomProperties.end(); ++itgeom) {
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
                itgeom->_t = Transform();     // reset back to identity
                break;
            default:
                RAVELOG_WARN("unknown geometry type: %d\n", itgeom->GetType());
            }

            //  Gets collision mesh
            KinBody::Link::TRIMESH trimesh = itgeom->GetCollisionMesh();
            trimesh.ApplyTransform(itgeom->_t);
            plink->collision.Append(trimesh);
        }

        return bhasgeometry || plink->_listGeomProperties.size() > nGeomBefore;
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
    bool _ExtractGeometry(const domTrianglesRef triRef, const domVerticesRef vertsRef, const map<string,domMaterialRef>& mapmaterials, KinBody::LinkPtr plink)
    {
        if( !triRef ) {
            return false;
        }
        plink->_listGeomProperties.push_back(KinBody::Link::GEOMPROPERTIES(plink));
        KinBody::Link::GEOMPROPERTIES& geom = plink->_listGeomProperties.back();
        KinBody::Link::TRIMESH& trimesh = geom.collisionmesh;
        geom._type = KinBody::Link::GEOMPROPERTIES::GeomTrimesh;

        // resolve the material and assign correct colors to the geometry
        if( !!triRef->getMaterial() ) {
            map<string,domMaterialRef>::const_iterator itmat = mapmaterials.find(triRef->getMaterial());
            if( itmat != mapmaterials.end() ) {
                FillGeometryColor(itmat->second,geom);
            }
        }

        size_t triangleIndexStride = 0, vertexoffset = -1;
        domInput_local_offsetRef indexOffsetRef;

        for (unsigned int w=0; w<triRef->getInput_array().getCount(); w++) {
            size_t offset = triRef->getInput_array()[w]->getOffset();
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
        trimesh.indices.reserve(triRef->getCount()*3);
        trimesh.vertices.reserve(triRef->getCount()*3);
        for (size_t i=0; i<vertsRef->getInput_array().getCount(); ++i) {
            domInput_localRef localRef = vertsRef->getInput_array()[i];
            daeString str = localRef->getSemantic();
            if ( strcmp(str,"POSITION") == 0 ) {
                const domSourceRef node = daeSafeCast<domSource>(localRef->getSource().getElement());
                if( !node ) {
                    continue;
                }
                dReal fUnitScale = _GetUnitScale(node,_fGlobalScale);
                const domFloat_arrayRef flArray = node->getFloat_array();
                if (!!flArray) {
                    const domList_of_floats& listFloats = flArray->getValue();
                    int k=vertexoffset;
                    int vertexStride = 3;     //instead of hardcoded stride, should use the 'accessor'
                    for(size_t itri = 0; itri < triRef->getCount(); ++itri) {
                        if(k+2*triangleIndexStride < indexArray.getCount() ) {
                            for (int j=0; j<3; j++) {
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
                else {
                    RAVELOG_WARN("float array not defined!\n");
                }
                break;
            }
        }
        if( trimesh.indices.size() != 3*triRef->getCount() ) {
            RAVELOG_WARN("triangles declares wrong count!\n");
        }
        geom.InitCollisionMesh();
        return true;
    }

    /// Extract the Geometry in TRIGLE FANS and adds it to OpenRave
    /// \param  triRef  Array of triangle fans of the COLLADA's model
    /// \param  vertsRef    Array of vertices of the COLLADA's model
    /// \param  mapmaterials    Materials applied to the geometry
    /// \param  plink   Link of the kinematics model
    bool _ExtractGeometry(const domTrifansRef triRef, const domVerticesRef vertsRef, const map<string,domMaterialRef>& mapmaterials, KinBody::LinkPtr plink)
    {
        if( !triRef ) {
            return false;
        }
        plink->_listGeomProperties.push_back(KinBody::Link::GEOMPROPERTIES(plink));
        KinBody::Link::GEOMPROPERTIES& geom = plink->_listGeomProperties.back();
        KinBody::Link::TRIMESH& trimesh = geom.collisionmesh;
        geom._type = KinBody::Link::GEOMPROPERTIES::GeomTrimesh;

        // resolve the material and assign correct colors to the geometry
        if( !!triRef->getMaterial() ) {
            map<string,domMaterialRef>::const_iterator itmat = mapmaterials.find(triRef->getMaterial());
            if( itmat != mapmaterials.end() ) {
                FillGeometryColor(itmat->second,geom);
            }
        }

        size_t triangleIndexStride = 0, vertexoffset = -1;
        domInput_local_offsetRef indexOffsetRef;

        for (unsigned int w=0; w<triRef->getInput_array().getCount(); w++) {
            size_t offset = triRef->getInput_array()[w]->getOffset();
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
        size_t primitivecount = triRef->getCount();
        if( primitivecount > triRef->getP_array().getCount() ) {
            RAVELOG_WARN("trifans has incorrect count\n");
            primitivecount = triRef->getP_array().getCount();
        }
        for(size_t ip = 0; ip < primitivecount; ++ip) {
            domList_of_uints indexArray =triRef->getP_array()[ip]->getValue();
            for (size_t i=0; i<vertsRef->getInput_array().getCount(); ++i) {
                domInput_localRef localRef = vertsRef->getInput_array()[i];
                daeString str = localRef->getSemantic();
                if ( strcmp(str,"POSITION") == 0 ) {
                    const domSourceRef node = daeSafeCast<domSource>(localRef->getSource().getElement());
                    if( !node ) {
                        continue;
                    }
                    dReal fUnitScale = _GetUnitScale(node,_fGlobalScale);
                    const domFloat_arrayRef flArray = node->getFloat_array();
                    if (!!flArray) {
                        const domList_of_floats& listFloats = flArray->getValue();
                        int k=vertexoffset;
                        int vertexStride = 3;     //instead of hardcoded stride, should use the 'accessor'
                        size_t usedindices = 3*(indexArray.getCount()-2);
                        if( trimesh.indices.capacity() < trimesh.indices.size()+usedindices ) {
                            trimesh.indices.reserve(trimesh.indices.size()+usedindices);
                        }
                        if( trimesh.vertices.capacity() < trimesh.vertices.size()+indexArray.getCount() ) {
                            trimesh.vertices.reserve(trimesh.vertices.size()+indexArray.getCount());
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
                    else {
                        RAVELOG_WARN("float array not defined!\n");
                    }
                    break;
                }
            }
        }

        geom.InitCollisionMesh();
        return true;
    }

    /// Extract the Geometry in TRIANGLE STRIPS and adds it to OpenRave
    /// \param  triRef  Array of Triangle Strips of the COLLADA's model
    /// \param  vertsRef    Array of vertices of the COLLADA's model
    /// \param  mapmaterials    Materials applied to the geometry
    /// \param  plink   Link of the kinematics model
    bool _ExtractGeometry(const domTristripsRef triRef, const domVerticesRef vertsRef, const map<string,domMaterialRef>& mapmaterials, KinBody::LinkPtr plink)
    {
        if( !triRef ) {
            return false;
        }
        plink->_listGeomProperties.push_back(KinBody::Link::GEOMPROPERTIES(plink));
        KinBody::Link::GEOMPROPERTIES& geom = plink->_listGeomProperties.back();
        KinBody::Link::TRIMESH& trimesh = geom.collisionmesh;
        geom._type = KinBody::Link::GEOMPROPERTIES::GeomTrimesh;

        // resolve the material and assign correct colors to the geometry
        if( !!triRef->getMaterial() ) {
            map<string,domMaterialRef>::const_iterator itmat = mapmaterials.find(triRef->getMaterial());
            if( itmat != mapmaterials.end() ) {
                FillGeometryColor(itmat->second,geom);
            }
        }
        size_t triangleIndexStride = 0, vertexoffset = -1;
        domInput_local_offsetRef indexOffsetRef;

        for (unsigned int w=0; w<triRef->getInput_array().getCount(); w++) {
            size_t offset = triRef->getInput_array()[w]->getOffset();
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
        size_t primitivecount = triRef->getCount();
        if( primitivecount > triRef->getP_array().getCount() ) {
            RAVELOG_WARN("tristrips has incorrect count\n");
            primitivecount = triRef->getP_array().getCount();
        }
        for(size_t ip = 0; ip < primitivecount; ++ip) {
            domList_of_uints indexArray = triRef->getP_array()[ip]->getValue();
            for (size_t i=0; i<vertsRef->getInput_array().getCount(); ++i) {
                domInput_localRef localRef = vertsRef->getInput_array()[i];
                daeString str = localRef->getSemantic();
                if ( strcmp(str,"POSITION") == 0 ) {
                    const domSourceRef node = daeSafeCast<domSource>(localRef->getSource().getElement());
                    if( !node ) {
                        continue;
                    }
                    dReal fUnitScale = _GetUnitScale(node,_fGlobalScale);
                    const domFloat_arrayRef flArray = node->getFloat_array();
                    if (!!flArray) {
                        const domList_of_floats& listFloats = flArray->getValue();
                        int k=vertexoffset;
                        int vertexStride = 3;     //instead of hardcoded stride, should use the 'accessor'
                        size_t usedindices = 3*(indexArray.getCount()-2);
                        if( trimesh.indices.capacity() < trimesh.indices.size()+usedindices ) {
                            trimesh.indices.reserve(trimesh.indices.size()+usedindices);
                        }
                        if( trimesh.vertices.capacity() < trimesh.vertices.size()+indexArray.getCount() ) {
                            trimesh.vertices.reserve(trimesh.vertices.size()+indexArray.getCount());
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
                    else {
                        RAVELOG_WARN("float array not defined!\n");
                    }
                    break;
                }
            }
        }
        geom.InitCollisionMesh();
        return true;
    }

    /// Extract the Geometry in TRIANGLE STRIPS and adds it to OpenRave
    /// \param  triRef  Array of Triangle Strips of the COLLADA's model
    /// \param  vertsRef    Array of vertices of the COLLADA's model
    /// \param  mapmaterials    Materials applied to the geometry
    /// \param  plink   Link of the kinematics model
    bool _ExtractGeometry(const domPolylistRef triRef, const domVerticesRef vertsRef, const map<string,domMaterialRef>& mapmaterials, KinBody::LinkPtr plink)
    {
        if( !triRef ) {
            return false;
        }
        plink->_listGeomProperties.push_back(KinBody::Link::GEOMPROPERTIES(plink));
        KinBody::Link::GEOMPROPERTIES& geom = plink->_listGeomProperties.back();
        KinBody::Link::TRIMESH& trimesh = geom.collisionmesh;
        geom._type = KinBody::Link::GEOMPROPERTIES::GeomTrimesh;

        // resolve the material and assign correct colors to the geometry
        if( !!triRef->getMaterial() ) {
            map<string,domMaterialRef>::const_iterator itmat = mapmaterials.find(triRef->getMaterial());
            if( itmat != mapmaterials.end() ) {
                FillGeometryColor(itmat->second,geom);
            }
        }

        size_t triangleIndexStride = 0,vertexoffset = -1;
        domInput_local_offsetRef indexOffsetRef;
        for (unsigned int w=0; w<triRef->getInput_array().getCount(); w++) {
            size_t offset = triRef->getInput_array()[w]->getOffset();
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
        for (size_t i=0; i<vertsRef->getInput_array().getCount(); ++i) {
            domInput_localRef localRef = vertsRef->getInput_array()[i];
            daeString str = localRef->getSemantic();
            if ( strcmp(str,"POSITION") == 0 ) {
                const domSourceRef node = daeSafeCast<domSource>(localRef->getSource().getElement());
                if( !node ) {
                    continue;
                }
                dReal fUnitScale = _GetUnitScale(node,_fGlobalScale);
                const domFloat_arrayRef flArray = node->getFloat_array();
                if (!!flArray) {
                    const domList_of_floats& listFloats = flArray->getValue();
                    size_t k=vertexoffset;
                    int vertexStride = 3;     //instead of hardcoded stride, should use the 'accessor'
                    for(size_t ipoly = 0; ipoly < triRef->getVcount()->getValue().getCount(); ++ipoly) {
                        size_t numverts = triRef->getVcount()->getValue()[ipoly];
                        if(( numverts > 0) &&( k+(numverts-1)*triangleIndexStride < indexArray.getCount()) ) {
                            size_t startoffset = trimesh.vertices.size();
                            for (size_t j=0; j<numverts; j++) {
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
                else {
                    RAVELOG_WARN("float array not defined!\n");
                }
                break;
            }
        }
        geom.InitCollisionMesh();
        return true;
    }

    /// Extract the Geometry and adds it to OpenRave
    /// \param  geom    Geometry to extract of the COLLADA's model
    /// \param  mapmaterials    Materials applied to the geometry
    /// \param  plink   Link of the kinematics model
    bool ExtractGeometry(const domGeometryRef geom, const map<string,domMaterialRef>& mapmaterials, KinBody::LinkPtr plink)
    {
        if( !geom ) {
            return false;
        }
        vector<Vector> vconvexhull;
        if (geom->getMesh()) {
            const domMeshRef meshRef = geom->getMesh();
            for (size_t tg = 0; tg<meshRef->getTriangles_array().getCount(); tg++) {
                _ExtractGeometry(meshRef->getTriangles_array()[tg], meshRef->getVertices(), mapmaterials, plink);
            }
            for (size_t tg = 0; tg<meshRef->getTrifans_array().getCount(); tg++) {
                _ExtractGeometry(meshRef->getTrifans_array()[tg], meshRef->getVertices(), mapmaterials, plink);
            }
            for (size_t tg = 0; tg<meshRef->getTristrips_array().getCount(); tg++) {
                _ExtractGeometry(meshRef->getTristrips_array()[tg], meshRef->getVertices(), mapmaterials, plink);
            }
            for (size_t tg = 0; tg<meshRef->getPolylist_array().getCount(); tg++) {
                _ExtractGeometry(meshRef->getPolylist_array()[tg], meshRef->getVertices(), mapmaterials, plink);
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
            //                        dReal fUnitScale = _GetUnitScale(node,_fGlobalScale);
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
        else if (geom->getConvex_mesh()) {
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
                        if (!strcmp(lib->getId(),urlref2+1)) {     // skip the # at the front of urlref2
                            //found convex_hull geometry
                            domMesh *meshElement = lib->getMesh();     //linkedGeom->getMesh();
                            if (meshElement) {
                                const domVerticesRef vertsRef = meshElement->getVertices();
                                for (size_t i=0; i<vertsRef->getInput_array().getCount(); i++) {
                                    domInput_localRef localRef = vertsRef->getInput_array()[i];
                                    daeString str = localRef->getSemantic();
                                    if ( strcmp(str,"POSITION") == 0) {
                                        const domSourceRef node = daeSafeCast<domSource>(localRef->getSource().getElement());
                                        if( !node ) {
                                            continue;
                                        }
                                        dReal fUnitScale = _GetUnitScale(node,_fGlobalScale);
                                        const domFloat_arrayRef flArray = node->getFloat_array();
                                        if (!!flArray) {
                                            vconvexhull.reserve(vconvexhull.size()+flArray->getCount());
                                            const domList_of_floats& listFloats = flArray->getValue();
                                            for (size_t k=0; k+2<flArray->getCount(); k+=3) {
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
                for (size_t i=0; i<vertsRef->getInput_array().getCount(); i++) {
                    domInput_localRef localRef = vertsRef->getInput_array()[i];
                    daeString str = localRef->getSemantic();
                    if ( strcmp(str,"POSITION") == 0 ) {
                        const domSourceRef node = daeSafeCast<domSource>(localRef->getSource().getElement());
                        if( !node ) {
                            continue;
                        }
                        dReal fUnitScale = _GetUnitScale(node,_fGlobalScale);
                        const domFloat_arrayRef flArray = node->getFloat_array();
                        if (!!flArray) {
                            const domList_of_floats& listFloats = flArray->getValue();
                            vconvexhull.reserve(vconvexhull.size()+flArray->getCount());
                            for (size_t k=0; k+2<flArray->getCount(); k+=3) {
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
                geom._type = KinBody::Link::GEOMPROPERTIES::GeomTrimesh;

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
                    pmanip->_name = _ConvertToOpenRAVEName(name);
                    daeElement* pframe_origin = tec->getChild("frame_origin");
                    daeElement* pframe_tip = tec->getChild("frame_tip");
                    if( !!pframe_origin ) {
                        domLinkRef pdomlink = daeSafeCast<domLink>(daeSidRef(pframe_origin->getAttribute("link"), as).resolve().elt);
                        if( !!pdomlink ) {
                            pmanip->_pBase = probot->GetLink(_ExtractLinkName(pdomlink));
                        }
                        if( !pmanip->_pBase ) {
                            RAVELOG_WARN(str(boost::format("failed to find manipulator %s frame origin %s\n")%name%pframe_origin->getAttribute("link")));
                            continue;
                        }
                    }
                    if( !!pframe_tip ) {
                        domLinkRef pdomlink = daeSafeCast<domLink>(daeSidRef(pframe_tip->getAttribute("link"), as).resolve().elt);
                        if( !!pdomlink ) {
                            pmanip->_pEndEffector = probot->GetLink(_ExtractLinkName(pdomlink));
                        }
                        if( !pmanip->_pEndEffector ) {
                            RAVELOG_WARN(str(boost::format("failed to find manipulator %s frame tip %s\n")%name%pframe_tip->getAttribute("link")));
                            continue;
                        }
                        pmanip->_tLocalTool = _ExtractFullTransformFromChildren(pframe_tip);
                    }

                    for(size_t ic = 0; ic < tec->getContents().getCount(); ++ic) {
                        daeElementRef pmanipchild = tec->getContents()[ic];
                        if( pmanipchild->getElementName() == string("gripper_joint") ) {
                            std::pair<KinBody::JointPtr, domJointRef> result = _getJointFromRef(pmanipchild->getAttribute("joint").c_str(),as,probot);
                            KinBody::JointPtr pjoint = result.first;
                            domJointRef pdomjoint = result.second;
                            if( !!pjoint && !!pdomjoint ) {
                                pmanip->_vgripperjointnames.push_back(pjoint->GetName());
                                daeTArray<daeElementRef> children;
                                pmanipchild->getChildren(children);
                                for (size_t i = 0; i < children.getCount(); i++) {
                                    if( children[i]->getElementName() == string("closing_direction") ) {
                                        domAxis_constraintRef paxis = daeSafeCast<domAxis_constraint>(daeSidRef(children[i]->getAttribute("axis"), pdomjoint).resolve().elt);
                                        float closing_direction = 0;
                                        if( !paxis ) {
                                            RAVELOG_WARN(str(boost::format("cannot resolve joint %s axis %s")%pmanipchild->getAttribute("joint")%children[i]->getAttribute("axis")));
                                        }
                                        else {
                                            if( !resolveCommon_float_or_param(children[i],as,closing_direction) ) {
                                                RAVELOG_WARN(str(boost::format("gripper joint %s axis %s cannot extract closing_direction\n")%children[i]->getAttribute("axis")%pmanipchild->getAttribute("joint")));
                                            }
                                        }
                                        pmanip->_vClosingDirection.push_back((dReal)closing_direction);
                                    }
                                }
                                continue;
                            }
                            RAVELOG_WARN(str(boost::format("could not find manipulator '%s' gripper joint '%s'\n")%pmanip->GetName()%pmanipchild->getAttribute("joint")));
                        }
                        else if( pmanipchild->getElementName() == string("iksolver") ) {
                            boost::shared_ptr<std::string> interfacename = _ExtractInterfaceType(tec->getContents()[ic]);
                            if( !!interfacename ) {
                                pmanip->_strIkSolver = *interfacename;
                                pmanip->_pIkSolver = RaveCreateIkSolver(_penv,pmanip->_strIkSolver);
                            }
                        }
                        else if((pmanipchild->getElementName() != string("frame_origin"))&&(pmanipchild->getElementName() != string("frame_tip"))) {
                            RAVELOG_WARN(str(boost::format("unrecognized tag <%s> in manipulator '%s'")%pmanipchild->getElementName()%pmanip->GetName()));
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
            if( strcmp(pextra->getType(), "attach_sensor") == 0 ) {
                string name = pextra->getAttribute("name");
                if( name.size() == 0 ) {
                    name = str(boost::format("sensor%d")%_nGlobalSensorId++);
                }
                domTechniqueRef tec = _ExtractOpenRAVEProfile(pextra->getTechnique_array());
                if( !!tec ) {
                    RobotBase::AttachedSensorPtr pattachedsensor(new RobotBase::AttachedSensor(probot));
                    pattachedsensor->_name = _ConvertToOpenRAVEName(name);
                    daeElement* pframe_origin = tec->getChild("frame_origin");
                    if( !!pframe_origin ) {
                        domLinkRef pdomlink = daeSafeCast<domLink>(daeSidRef(pframe_origin->getAttribute("link"), as).resolve().elt);
                        if( !!pdomlink ) {
                            pattachedsensor->pattachedlink = probot->GetLink(_ExtractLinkName(pdomlink));
                        }
                        if( !pattachedsensor->pattachedlink.lock() ) {
                            RAVELOG_WARN(str(boost::format("failed to find manipulator %s frame origin %s\n")%name%pframe_origin->getAttribute("link")));
                            continue;
                        }
                        pattachedsensor->trelative = _ExtractFullTransformFromChildren(pframe_origin);
                    }
                    if( !_ExtractSensor(pattachedsensor->psensor,tec->getChild("instance_sensor")) ) {
                        RAVELOG_WARN(str(boost::format("cannot find instance_sensor for attached sensor %s:%s\n")%probot->GetName()%name));
                    }
                    else {
                        pattachedsensor->pdata = pattachedsensor->GetSensor()->CreateSensorData();
                    }
                    probot->GetAttachedSensors().push_back(pattachedsensor);
                }
                else {
                    RAVELOG_WARN(str(boost::format("cannot create robot %s attached sensor %s\n")%probot->GetName()%name));
                }
            }
        }
    }

    /// \brief Extract an instance of a sensor
    bool _ExtractSensor(SensorBasePtr& psensor, daeElementRef instance_sensor)
    {
        if( !instance_sensor ) {
            return false;
        }
        if( !instance_sensor->hasAttribute("url") ) {
            RAVELOG_WARN("instance_sensor has no url\n");
            return false;
        }

        std::string instance_id = instance_sensor->getAttribute("id");
        std::string instance_url = instance_sensor->getAttribute("url");
        daeElementRef domsensor = _getElementFromUrl(daeURI(*instance_sensor,instance_url));
        if( !domsensor ) {
            RAVELOG_WARN(str(boost::format("failed to find senor id %s url=%s\n")%instance_id%instance_url));
            return false;
        }
        if( !domsensor->hasAttribute("type") ) {
            RAVELOG_WARN("collada <sensor> needs type attribute\n");
            return false;
        }
        psensor = RaveCreateSensor(_penv, domsensor->getAttribute("type"));
        if( !psensor ) {
            return false;
        }

        // Create the custom XML reader to read in the data (determined by users)
        BaseXMLReaderPtr pcurreader = RaveCallXMLReader(PT_Sensor,psensor->GetXMLId(),psensor, AttributesList());
        if( !pcurreader ) {
            pcurreader.reset();
            return false;
        }
        _ProcessXMLReader(pcurreader,domsensor);
        psensor->__mapReadableInterfaces[psensor->GetXMLId()] = pcurreader->GetReadable();
        return true;
    }

    /// \brief feed the collada data into the base readers xml class
    static void _ProcessXMLReader(BaseXMLReaderPtr preader, daeElementRef elt)
    {
        daeTArray<daeElementRef> children;
        elt->getChildren(children);
        AttributesList atts;
        for (size_t i = 0; i < children.getCount(); i++) {
            string xmltag = tolowerstring(children[i]->getElementName());
            daeTArray<daeElement::attr> domatts;
            children[i]->getAttributes(domatts);
            atts.clear();
            for(size_t j = 0; j < domatts.getCount(); ++j) {
                atts.push_back(make_pair(domatts[j].name,domatts[j].value));
            }
            if( preader->startElement(xmltag,atts) == BaseXMLReader::PE_Support ) {
                _ProcessXMLReader(preader,children[i]);
                preader->characters(children[i]->getCharData());
                preader->endElement(xmltag);
            }
        }
    }

    inline daeElementRef _getElementFromUrl(const daeURI &uri)
    {
        return daeStandardURIResolver(*_dae).resolveElement(uri);
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
                if( !!pbind->getSymbol() &&( strcmp(pbind->getSymbol(), ref) == 0) ) {
                    // found a match
                    if( !!pbind->getParam() ) {
                        //return searchBinding(pbind->getParam()->getRef(), pbindelt);
                        return daeSidRef(pbind->getParam()->getRef(), pbindelt).resolve().elt;
                    }
                    else if( !!pbind->getSIDREF() ) {
                        return daeSidRef(pbind->getSIDREF()->getValue(), pbindelt).resolve().elt;
                    }
                }
            }
            for(size_t inewparam = 0; inewparam < pnewparamarray->getCount(); ++inewparam) {
                domKinematics_newparamRef newparam = (*pnewparamarray)[inewparam];
                if( !!newparam->getSid() &&( strcmp(newparam->getSid(), ref) == 0) ) {
                    if( !!newparam->getSIDREF() ) {     // can only bind with SIDREF
                        return daeSidRef(newparam->getSIDREF()->getValue(),pbindelt).resolve().elt;
                    }
                    RAVELOG_WARN(str(boost::format("newparam sid=%s does not have SIDREF\n")%newparam->getSid()));
                }
            }
        }
        RAVELOG_WARN(str(boost::format("failed to get binding '%s' for element: %s\n")%ref%parent->getElementName()));
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
            if( !!pnewparam->getSid() &&( strcmp(pnewparam->getSid(), paddr->getParam()->getValue()) == 0) ) {
                if( !!pnewparam->getBool() ) {
                    return pnewparam->getBool()->getValue();
                }
                else if( !!pnewparam->getSIDREF() ) {
                    domKinematics_newparam::domBoolRef ptarget = daeSafeCast<domKinematics_newparam::domBool>(daeSidRef(pnewparam->getSIDREF()->getValue(), pnewparam).resolve().elt);
                    if( !ptarget ) {
                        RAVELOG_WARN("failed to resolve %s from %s\n", pnewparam->getSIDREF()->getValue(), paddr->getID());
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
            if( !!pnewparam->getSid() &&( strcmp(pnewparam->getSid(), paddr->getParam()->getValue()) == 0) ) {
                if( !!pnewparam->getFloat() ) {
                    return pnewparam->getFloat()->getValue();
                }
                else if( !!pnewparam->getSIDREF() ) {
                    domKinematics_newparam::domFloatRef ptarget = daeSafeCast<domKinematics_newparam::domFloat>(daeSidRef(pnewparam->getSIDREF()->getValue(), pnewparam).resolve().elt);
                    if( !ptarget ) {
                        RAVELOG_WARN("failed to resolve %s from %s\n", pnewparam->getSIDREF()->getValue(), paddr->getID());
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
    TransformMatrix getTransform(daeElementRef pelt)
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
            t.trans *= _GetUnitScale(pelt,_fGlobalScale);
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
            t.trans *= _GetUnitScale(pelt,_fGlobalScale);
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
            t = transformLookat(lookat*_GetUnitScale(pelt,_fGlobalScale),campos*_GetUnitScale(pelt,_fGlobalScale),camup);
            return t;
        }

        domSkewRef pskew = daeSafeCast<domSkew>(pelt);
        if( !!pskew ) {
            RAVELOG_ERROR("skew transform not implemented\n");
        }

        return t;
    }

    /// Travels recursively the node parents of the given one
    /// to extract the Transform arrays that affects the node given
    template <typename T> TransformMatrix getNodeParentTransform(const T pelt) {
        domNodeRef pnode = daeSafeCast<domNode>(pelt->getParent());
        if( !pnode ) {
            return TransformMatrix();
        }
        return getNodeParentTransform(pnode) * _ExtractFullTransform(pnode);
    }

    /// \brief Travel by the transformation array and calls the getTransform method
    template <typename T> TransformMatrix _ExtractFullTransform(const T pelt) {
        TransformMatrix t;
        for(size_t i = 0; i < pelt->getContents().getCount(); ++i) {
            t = t * getTransform(pelt->getContents()[i]);
        }
        return t;
    }

    /// \brief Travel by the transformation array and calls the getTransform method
    template <typename T> TransformMatrix _ExtractFullTransformFromChildren(const T pelt) {
        TransformMatrix t;
        daeTArray<daeElementRef> children;
        pelt->getChildren(children);
        for(size_t i = 0; i < children.getCount(); ++i) {
            t = t * getTransform(children[i]);
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
        tout = tm;     // quaternion removes the scale?
        TransformMatrix tnormalized = tout;
        for(int i = 0; i < 3; ++i) {
            vscale[i] = (RaveFabs(tm.m[0+i])+RaveFabs(tm.m[4+i])+RaveFabs(tm.m[8+i]))/(RaveFabs(tnormalized.m[0+i])+RaveFabs(tnormalized.m[4+i])+RaveFabs(tnormalized.m[8+i]));
        }
    }

    virtual void handleError( daeString msg )
    {
        if( _bOpeningZAE && (( msg == string("Document is empty\n")) ||( msg == string("Error parsing XML in daeLIBXMLPlugin::read\n")) ) ) {
            return;     // collada-dom prints these messages even if no error
        }
        RAVELOG_ERROR(str(boost::format("COLLADA error: %s")%msg));
    }

    virtual void handleWarning( daeString msg )
    {
        RAVELOG_WARN(str(boost::format("COLLADA warning: %s")%msg));
    }

private:

    /// \brief go through all kinematics binds to get a kinematics/visual pair
    ///
    /// \param kiscene instance of one kinematics scene, binds the kinematic and visual models
    /// \param bindings the extracted bindings
    static void _ExtractKinematicsVisualBindings(domInstance_with_extraRef viscene, domInstance_kinematics_sceneRef kiscene, KinematicsSceneBindings& bindings)
    {
        domKinematics_sceneRef kscene = daeSafeCast<domKinematics_scene> (kiscene->getUrl().getElement().cast());
        if (!kscene) {
            return;
        }
        for (size_t imodel = 0; imodel < kiscene->getBind_kinematics_model_array().getCount(); imodel++) {
            domArticulated_systemRef articulated_system;     // if filled, contains robot-specific information, so create a robot
            domBind_kinematics_modelRef kbindmodel = kiscene->getBind_kinematics_model_array()[imodel];
            if (!kbindmodel->getNode()) {
                RAVELOG_WARN("do not support kinematics models without references to nodes\n");
                continue;
            }

            // visual information
            domNodeRef node = daeSafeCast<domNode>(daeSidRef(kbindmodel->getNode(), viscene->getUrl().getElement()).resolve().elt);
            if (!node) {
                RAVELOG_WARN(str(boost::format("bind_kinematics_model does not reference valid node %s\n")%kbindmodel->getNode()));
                continue;
            }

            //  kinematics information
            daeElement* pelt = searchBinding(kbindmodel,kscene);
            domInstance_kinematics_modelRef ikmodel = daeSafeCast<domInstance_kinematics_model>(pelt);
            if (!ikmodel) {
                if( !pelt ) {
                    RAVELOG_WARN("bind_kinematics_model does not reference element\n");
                }
                else {
                    RAVELOG_WARN(str(boost::format("bind_kinematics_model cannot find reference to %s:%s:\n")%kbindmodel->getNode()%pelt->getElementName()));
                }
                continue;
            }
            bindings.listModelBindings.push_back(ModelBinding(node,ikmodel));
        }
        // axis info
        for (size_t ijoint = 0; ijoint < kiscene->getBind_joint_axis_array().getCount(); ++ijoint) {
            domBind_joint_axisRef bindjoint = kiscene->getBind_joint_axis_array()[ijoint];
            daeElementRef pjtarget = daeSidRef(bindjoint->getTarget(), viscene->getUrl().getElement()).resolve().elt;
            if (!pjtarget) {
                RAVELOG_WARN(str(boost::format("Target Node '%s' not found\n")%bindjoint->getTarget()));
                continue;
            }
            daeElement* pelt = searchBinding(bindjoint->getAxis(),kscene);
            domAxis_constraintRef pjointaxis = daeSafeCast<domAxis_constraint>(pelt);
            if (!pjointaxis) {
                RAVELOG_WARN(str(boost::format("joint axis for target %s\n")%bindjoint->getTarget()));
                continue;
            }
            bindings.listAxisBindings.push_back(JointAxisBinding(pjtarget, pjointaxis, bindjoint->getValue(), NULL, NULL));
        }
    }

    static void _ExtractPhysicsBindings(domCOLLADA::domSceneRef allscene, KinematicsSceneBindings& bindings)
    {
        for(size_t iphysics = 0; iphysics < allscene->getInstance_physics_scene_array().getCount(); ++iphysics) {
            domPhysics_sceneRef pscene = daeSafeCast<domPhysics_scene>(allscene->getInstance_physics_scene_array()[iphysics]->getUrl().getElement().cast());
            for(size_t imodel = 0; imodel < pscene->getInstance_physics_model_array().getCount(); ++imodel) {
                domInstance_physics_modelRef ipmodel = pscene->getInstance_physics_model_array()[imodel];
                domPhysics_modelRef pmodel = daeSafeCast<domPhysics_model> (ipmodel->getUrl().getElement().cast());
                domNodeRef nodephysicsoffset = daeSafeCast<domNode>(ipmodel->getParent().getElement().cast());
                for(size_t ibody = 0; ibody < ipmodel->getInstance_rigid_body_array().getCount(); ++ibody) {
                    LinkBinding lb;
                    lb._irigidbody = ipmodel->getInstance_rigid_body_array()[ibody];
                    lb._node = daeSafeCast<domNode>(lb._irigidbody->getTarget().getElement().cast());
                    lb._rigidbody = daeSafeCast<domRigid_body>(daeSidRef(lb._irigidbody->getBody(),pmodel).resolve().elt);
                    lb._nodephysicsoffset = nodephysicsoffset;
                    if( !!lb._rigidbody && !!lb._node ) {
                        bindings.listLinkBindings.push_back(lb);
                    }
                }
            }
        }
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

    daeElementRef _ExtractOpenRAVEProfile(const daeElementRef pelt)
    {
        daeTArray<daeElementRef> children;
        pelt->getChildren(children);
        for(size_t i = 0; i < children.getCount(); ++i) {
            if(( children[i]->getElementName() == string("technique")) && children[i]->hasAttribute("profile") &&( children[i]->getAttribute("profile") == string("OpenRAVE")) ) {
                return children[i];
            }
        }
        return daeElementRef();
    }

    domInstance_physics_modelRef _GetPhysicsModelNodeOffset(domCOLLADA::domSceneRef allscene, domNodeRef parentnode)
    {
        BOOST_ASSERT( !!parentnode && !!parentnode->getID() );
        for(size_t iphysics = 0; iphysics < allscene->getInstance_physics_scene_array().getCount(); ++iphysics) {
            domPhysics_sceneRef pscene = daeSafeCast<domPhysics_scene>(allscene->getInstance_physics_scene_array()[iphysics]->getUrl().getElement().cast());
            for(size_t imodel = 0; imodel < pscene->getInstance_physics_model_array().getCount(); ++imodel) {
                domInstance_physics_modelRef ipmodel = pscene->getInstance_physics_model_array()[imodel];
                domNodeRef prefnode = daeSafeCast<domNode>(ipmodel->getParent().getElement().cast());
                if( !!prefnode && !!prefnode->getID() ) {
                    if( strcmp(prefnode->getID(),parentnode->getID())==0 ) {
                        return ipmodel;
                    }
                }
            }
        }
        return domInstance_physics_modelRef();
    }

    /// \brief returns an openrave interface type from the extra array
    boost::shared_ptr<std::string> _ExtractInterfaceType(const daeElementRef pelt) {
        daeTArray<daeElementRef> children;
        pelt->getChildren(children);
        for(size_t i = 0; i < children.getCount(); ++i) {
            if( children[i]->getElementName() == string("interface_type") ) {
                daeElementRef ptec = _ExtractOpenRAVEProfile(children[i]);
                if( !!ptec ) {
                    daeElement* ptype = ptec->getChild("interface");
                    if( !!ptype ) {
                        return boost::shared_ptr<std::string>(new std::string(ptype->getCharData()));
                    }
                }
            }
        }
        return boost::shared_ptr<std::string>();
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

    std::string _ExtractLinkName(domLinkRef pdomlink) {
        std::string linkname;
        if( !!pdomlink ) {
            if( !!pdomlink->getName() ) {
                linkname = pdomlink->getName();
            }
            if(( linkname.size() == 0) && !!pdomlink->getID() ) {
                linkname = pdomlink->getID();
            }
        }
        return _ConvertToOpenRAVEName(linkname);
    }

    bool _checkMathML(daeElementRef pelt,const string& type)
    {
        if( pelt->getElementName()==type ) {
            return true;
        }
        // check the substring after ':', the substring before is the namespace set in some parent attribute
        string name = pelt->getElementName();
        size_t pos = name.find_last_of(':');
        if( pos == string::npos ) {
            return false;
        }
        return name.substr(pos+1)==type;
    }

    std::pair<KinBody::JointPtr,domJointRef> _getJointFromRef(xsToken targetref, daeElementRef peltref, KinBodyPtr pkinbody) {
        daeElement* peltjoint = daeSidRef(targetref, peltref).resolve().elt;
        domJointRef pdomjoint = daeSafeCast<domJoint> (peltjoint);
        if (!pdomjoint) {
            domInstance_jointRef pdomijoint = daeSafeCast<domInstance_joint> (peltjoint);
            if (!!pdomijoint) {
                pdomjoint = daeSafeCast<domJoint> (pdomijoint->getUrl().getElement().cast());
            }
        }

        if (!pdomjoint) {
            RAVELOG_WARN(str(boost::format("could not find collada joint '%s'!\n")%targetref));
            return std::make_pair(KinBody::JointPtr(),domJointRef());
        }

        if( string(targetref).find("./") != 0 ) {
            std::map<std::string,KinBody::JointPtr>::iterator itjoint = _mapJointIds.find(targetref);
            if( itjoint != _mapJointIds.end() ) {
                return std::make_pair(itjoint->second,pdomjoint);
            }
            RAVELOG_WARN(str(boost::format("failed to find joint target '%s' in _mapJointIds")%targetref));
        }

        KinBody::JointPtr pjoint = pkinbody->GetJoint(pdomjoint->getName());
        if(!pjoint) {
            RAVELOG_WARN(str(boost::format("could not find openrave joint '%s'!\n")%pdomjoint->getName()));
        }
        return std::make_pair(pjoint,pdomjoint);
    }

    /// \brief get the element name without the namespace
    std::string _getElementName(daeElementRef pelt) {
        std::string name = pelt->getElementName();
        std::size_t pos = name.find_last_of(':');
        if( pos != string::npos ) {
            return name.substr(pos+1);
        }
        return name;
    }

    std::string _ExtractParentId(daeElementRef p) {
        while(!!p) {
            if( p->hasAttribute("id") ) {
                return p->getAttribute("id");
            }
            p = p->getParent();
        }
        return "";
    }

    /// \brief Extracts MathML into fparser equation format
    std::string _ExtractMathML(daeElementRef proot, KinBodyPtr pkinbody, daeElementRef pelt)
    {
        std::string name = _getElementName(pelt);
        std::string eq;
        daeTArray<daeElementRef> children;
        pelt->getChildren(children);
        if( name == "math" ) {
            for(std::size_t ic = 0; ic < children.getCount(); ++ic) {
                std::string childname = _getElementName(children[ic]);
                if(( childname == "apply") ||( childname == "csymbol") ||( childname == "cn") ||( childname == "ci") ) {
                    eq = _ExtractMathML(proot, pkinbody, children[ic]);
                }
                else {
                    throw openrave_exception(str(boost::format("_ExtractMathML: do not support element %s in mathml")%childname),ORE_CommandNotSupported);
                }
            }
        }
        else if( name == "apply" ) {
            if( children.getCount() == 0 ) {
                return eq;
            }
            string childname = _getElementName(children[0]);
            if( childname == "plus" ) {
                eq += '(';
                for(size_t ic = 1; ic < children.getCount(); ++ic) {
                    eq += _ExtractMathML(proot, pkinbody, children[ic]);
                    if( ic+1 < children.getCount() ) {
                        eq += '+';
                    }
                }
                eq += ')';
            }
            else if( childname == "quotient" ) {
                BOOST_ASSERT(children.getCount()==3);
                eq += str(boost::format("floor(%s/%s)")%_ExtractMathML(proot,pkinbody,children[1])%_ExtractMathML(proot,pkinbody,children[2]));
            }
            else if( childname == "divide" ) {
                BOOST_ASSERT(children.getCount()==3);
                eq += str(boost::format("(%s/%s)")%_ExtractMathML(proot,pkinbody,children[1])%_ExtractMathML(proot,pkinbody,children[2]));
            }
            else if( childname == "minus" ) {
                BOOST_ASSERT(children.getCount()>1 && children.getCount()<=3);
                if( children.getCount() == 2 ) {
                    eq += str(boost::format("(-%s)")%_ExtractMathML(proot,pkinbody,children[1]));
                }
                else {
                    eq += str(boost::format("(%s-%s)")%_ExtractMathML(proot,pkinbody,children[1])%_ExtractMathML(proot,pkinbody,children[2]));
                }
            }
            else if( childname == "power" ) {
                BOOST_ASSERT(children.getCount()==3);
                std::string sbase = _ExtractMathML(proot,pkinbody,children[1]);
                std::string sexp = _ExtractMathML(proot,pkinbody,children[2]);
                //                try {
                //                    int degree = boost::lexical_cast<int>(sexp);
                //                    if( degree == 1 ) {
                //                        eq += str(boost::format("(%s)")%sbase);
                //                    }
                //                    else if( degree == 2 ) {
                //                        eq += str(boost::format("sqr(%s)")%sbase);
                //                    }
                //                    else {
                //                        eq += str(boost::format("pow(%s,%s)")%sbase%sexp);
                //                    }
                //                }
                //                catch(const boost::bad_lexical_cast&) {
                eq += str(boost::format("pow(%s,%s)")%sbase%sexp);
                //}
            }
            else if( childname == "rem" ) {
                BOOST_ASSERT(children.getCount()==3);
                eq += str(boost::format("(%s%%%s)")%_ExtractMathML(proot,pkinbody,children[1])%_ExtractMathML(proot,pkinbody,children[2]));
            }
            else if( childname == "times" ) {
                eq += '(';
                for(size_t ic = 1; ic < children.getCount(); ++ic) {
                    eq += _ExtractMathML(proot, pkinbody, children[ic]);
                    if( ic+1 < children.getCount() ) {
                        eq += '*';
                    }
                }
                eq += ')';
            }
            else if( childname == "root" ) {
                BOOST_ASSERT(children.getCount()==3);
                string sdegree, snum;
                for(size_t ic = 1; ic < children.getCount(); ++ic) {
                    if( _getElementName(children[ic]) == string("degree") ) {
                        sdegree = _ExtractMathML(proot,pkinbody,children[ic]->getChildren()[0]);
                    }
                    else {
                        snum = _ExtractMathML(proot,pkinbody,children[ic]);
                    }
                }
                try {
                    int degree = boost::lexical_cast<int>(sdegree);
                    if( degree == 1 ) {
                        eq += str(boost::format("(%s)")%snum);
                    }
                    else if( degree == 2 ) {
                        eq += str(boost::format("sqrt(%s)")%snum);
                    }
                    else if( degree == 3 ) {
                        eq += str(boost::format("cbrt(%s)")%snum);
                    }
                    else {
                        eq += str(boost::format("pow(%s,1.0/%s)")%snum%sdegree);
                    }
                }
                catch(const boost::bad_lexical_cast&) {
                    eq += str(boost::format("pow(%s,1.0/%s)")%snum%sdegree);
                }
            }
            else if( childname == "and" ) {
                eq += '(';
                for(size_t ic = 1; ic < children.getCount(); ++ic) {
                    eq += _ExtractMathML(proot, pkinbody, children[ic]);
                    if( ic+1 < children.getCount() ) {
                        eq += '&';
                    }
                }
                eq += ')';
            }
            else if( childname == "or" ) {
                eq += '(';
                for(size_t ic = 1; ic < children.getCount(); ++ic) {
                    eq += _ExtractMathML(proot, pkinbody, children[ic]);
                    if( ic+1 < children.getCount() ) {
                        eq += '|';
                    }
                }
                eq += ')';
            }
            else if( childname == "not" ) {
                BOOST_ASSERT(children.getCount()==2);
                eq += str(boost::format("(!%s)")%_ExtractMathML(proot,pkinbody,children[1]));
            }
            else if( childname == "floor" ) {
                BOOST_ASSERT(children.getCount()==2);
                eq += str(boost::format("floor(%s)")%_ExtractMathML(proot,pkinbody,children[1]));
            }
            else if( childname == "ceiling" ) {
                BOOST_ASSERT(children.getCount()==2);
                eq += str(boost::format("ceil(%s)")%_ExtractMathML(proot,pkinbody,children[1]));
            }
            else if( childname == "eq" ) {
                BOOST_ASSERT(children.getCount()==3);
                eq += str(boost::format("(%s=%s)")%_ExtractMathML(proot,pkinbody,children[1])%_ExtractMathML(proot,pkinbody,children[2]));
            }
            else if( childname == "neq" ) {
                BOOST_ASSERT(children.getCount()==3);
                eq += str(boost::format("(%s!=%s)")%_ExtractMathML(proot,pkinbody,children[1])%_ExtractMathML(proot,pkinbody,children[2]));
            }
            else if( childname == "gt" ) {
                BOOST_ASSERT(children.getCount()==3);
                eq += str(boost::format("(%s>%s)")%_ExtractMathML(proot,pkinbody,children[1])%_ExtractMathML(proot,pkinbody,children[2]));
            }
            else if( childname == "lt" ) {
                BOOST_ASSERT(children.getCount()==3);
                eq += str(boost::format("(%s<%s)")%_ExtractMathML(proot,pkinbody,children[1])%_ExtractMathML(proot,pkinbody,children[2]));
            }
            else if( childname == "geq" ) {
                BOOST_ASSERT(children.getCount()==3);
                eq += str(boost::format("(%s>=%s)")%_ExtractMathML(proot,pkinbody,children[1])%_ExtractMathML(proot,pkinbody,children[2]));
            }
            else if( childname == "leq" ) {
                BOOST_ASSERT(children.getCount()==3);
                eq += str(boost::format("(%s<=%s)")%_ExtractMathML(proot,pkinbody,children[1])%_ExtractMathML(proot,pkinbody,children[2]));
            }
            else if( childname == "ln" ) {
                BOOST_ASSERT(children.getCount()==2);
                eq += str(boost::format("log(%s)")%_ExtractMathML(proot,pkinbody,children[1]));
            }
            else if( childname == "log" ) {
                BOOST_ASSERT(children.getCount()==2 || children.getCount()==3);
                string sbase="10", snum;
                for(size_t ic = 1; ic < children.getCount(); ++ic) {
                    if( _getElementName(children[ic]) == string("logbase") ) {
                        sbase = _ExtractMathML(proot,pkinbody,children[ic]->getChildren()[0]);
                    }
                    else {
                        snum = _ExtractMathML(proot,pkinbody,children[ic]);
                    }
                }
                try {
                    int base = boost::lexical_cast<int>(sbase);
                    if( base == 10 ) {
                        eq += str(boost::format("log10(%s)")%snum);
                    }
                    else if( base == 2 ) {
                        eq += str(boost::format("log2(%s)")%snum);
                    }
                    else {
                        eq += str(boost::format("(log(%s)/log(%s))")%snum%sbase);
                    }
                }
                catch(const boost::bad_lexical_cast&) {
                    eq += str(boost::format("(log(%s)/log(%s))")%snum%sbase);
                }
            }
            else if( childname == "arcsin" ) {
                BOOST_ASSERT(children.getCount()==2);
                eq += str(boost::format("asin(%s)")%_ExtractMathML(proot,pkinbody,children[1]));
            }
            else if( childname == "arccos" ) {
                BOOST_ASSERT(children.getCount()==2);
                eq += str(boost::format("acos(%s)")%_ExtractMathML(proot,pkinbody,children[1]));
            }
            else if( childname == "arctan" ) {
                BOOST_ASSERT(children.getCount()==2);
                eq += str(boost::format("atan(%s)")%_ExtractMathML(proot,pkinbody,children[1]));
            }
            else if( childname == "arccosh" ) {
                BOOST_ASSERT(children.getCount()==2);
                eq += str(boost::format("acosh(%s)")%_ExtractMathML(proot,pkinbody,children[1]));
            }
            else if( childname == "arccot" ) {
                BOOST_ASSERT(children.getCount()==2);
                eq += str(boost::format("acot(%s)")%_ExtractMathML(proot,pkinbody,children[1]));
            }
            else if( childname == "arccoth" ) {
                BOOST_ASSERT(children.getCount()==2);
                eq += str(boost::format("acoth(%s)")%_ExtractMathML(proot,pkinbody,children[1]));
            }
            else if( childname == "arccsc" ) {
                BOOST_ASSERT(children.getCount()==2);
                eq += str(boost::format("acsc(%s)")%_ExtractMathML(proot,pkinbody,children[1]));
            }
            else if( childname == "arccsch" ) {
                BOOST_ASSERT(children.getCount()==2);
                eq += str(boost::format("acsch(%s)")%_ExtractMathML(proot,pkinbody,children[1]));
            }
            else if( childname == "arcsec" ) {
                BOOST_ASSERT(children.getCount()==2);
                eq += str(boost::format("asec(%s)")%_ExtractMathML(proot,pkinbody,children[1]));
            }
            else if( childname == "arcsech" ) {
                BOOST_ASSERT(children.getCount()==2);
                eq += str(boost::format("asech(%s)")%_ExtractMathML(proot,pkinbody,children[1]));
            }
            else if( childname == "arcsinh" ) {
                BOOST_ASSERT(children.getCount()==2);
                eq += str(boost::format("asinh(%s)")%_ExtractMathML(proot,pkinbody,children[1]));
            }
            else if( childname == "arctanh" ) {
                BOOST_ASSERT(children.getCount()==2);
                eq += str(boost::format("atanh(%s)")%_ExtractMathML(proot,pkinbody,children[1]));
            }
            else if((childname == "implies")||(childname == "forall")||(childname == "exists")||(childname == "conjugate")||(childname == "arg")||(childname == "real")||(childname == "imaginary")||(childname == "lcm")||(childname == "factorial")||(childname == "xor")) {
                throw openrave_exception(str(boost::format("_ExtractMathML: %s function in <apply> tag not supported")%childname),ORE_CommandNotSupported);
            }
            else if( childname == "csymbol" ) {
                if( children[0]->getAttribute("encoding")==string("text/xml") ) {
                    domFormulaRef pformula;
                    string functionname;
                    if( children[0]->hasAttribute("definitionURL") ) {
                        // search for the formula in library_formulas
                        string formulaurl = children[0]->getAttribute("definitionURL");
                        if( formulaurl.size() > 0 ) {
                            daeElementRef pelt = _getElementFromUrl(daeURI(*children[0],formulaurl));
                            pformula = daeSafeCast<domFormula>(pelt);
                            if( !pformula ) {
                                RAVELOG_WARN(str(boost::format("could not find csymbol %s formula\n")%children[0]->getAttribute("definitionURL")));
                            }
                            else {
                                RAVELOG_DEBUG(str(boost::format("csymbol formula %s found\n")%pformula->getId()));
                            }
                        }
                    }
                    if( !pformula ) {
                        if( children[0]->hasAttribute("type") ) {
                            if( children[0]->getAttribute("type") == "function" ) {
                                functionname = children[0]->getCharData();
                            }
                        }
                    }
                    else {
                        if( !!pformula->getName() ) {
                            functionname = pformula->getName();
                        }
                        else {
                            functionname = children[0]->getCharData();
                        }
                    }

                    if( functionname == "INRANGE" ) {
                        BOOST_ASSERT(children.getCount()==4);
                        string a = _ExtractMathML(proot,pkinbody,children[1]), b = _ExtractMathML(proot,pkinbody,children[2]), c = _ExtractMathML(proot,pkinbody,children[3]);
                        eq += str(boost::format("((%s>=%s)&(%s<=%s))")%a%b%a%c);
                    }
                    else if((functionname == "SSSA")||(functionname == "SASA")||(functionname == "SASS")) {
                        BOOST_ASSERT(children.getCount()==4);
                        eq += str(boost::format("%s(%s,%s,%s)")%functionname%_ExtractMathML(proot,pkinbody,children[1])%_ExtractMathML(proot,pkinbody,children[2])%_ExtractMathML(proot,pkinbody,children[3]));
                    }
                    else if( functionname == "atan2") {
                        BOOST_ASSERT(children.getCount()==3);
                        eq += str(boost::format("atan2(%s,%s)")%_ExtractMathML(proot,pkinbody,children[1])%_ExtractMathML(proot,pkinbody,children[2]));
                    }
                    else {
                        RAVELOG_WARN(str(boost::format("csymbol %s not implemented\n")%functionname));
                        eq += "1";
                    }
                }
                else if( children[0]->getAttribute("encoding")!=string("COLLADA") ) {
                    RAVELOG_WARN(str(boost::format("_ExtractMathML: csymbol '%s' has unknown encoding '%s'")%children[0]->getCharData()%children[0]->getAttribute("encoding")));
                }
                else {
                    eq += _ExtractMathML(proot,pkinbody,children[0]);
                }
            }
            else {
                // make a function call
                eq += childname;
                eq += '(';
                for(size_t ic = 1; ic < children.getCount(); ++ic) {
                    eq += _ExtractMathML(proot, pkinbody, children[ic]);
                    if( ic+1 < children.getCount() ) {
                        eq += ',';
                    }
                }
                eq += ')';
            }
        }
        else if( name == "csymbol" ) {
            if( !pelt->hasAttribute("encoding") ) {
                RAVELOG_WARN(str(boost::format("_ExtractMathML: csymbol '%s' does not have any encoding")%pelt->getCharData()));
            }
            else if( pelt->getAttribute("encoding")!=string("COLLADA") ) {
                RAVELOG_WARN(str(boost::format("_ExtractMathML: csymbol '%s' has unknown encoding '%s'")%pelt->getCharData()%pelt->getAttribute("encoding")));
            }
            KinBody::JointPtr pjoint = _getJointFromRef(pelt->getCharData().c_str(),proot,pkinbody).first;
            if( !pjoint ) {
                RAVELOG_WARN(str(boost::format("_ExtractMathML: failed to find csymbol: %s")%pelt->getCharData()));
                eq = pelt->getCharData();
            }
            if( pjoint->GetDOF() > 1 ) {
                RAVELOG_WARN(str(boost::format("formulas do not support using joints with > 1 DOF yet (%s)")%pjoint->GetName()));
            }
            else {
                if(( _mapJointUnits.find(pjoint) != _mapJointUnits.end()) &&( _mapJointUnits[pjoint].at(0) != 1) ) {
                    eq = str(boost::format("(%f*%s)")%(1/_mapJointUnits[pjoint].at(0))%pjoint->GetName());
                }
                else {
                    eq = pjoint->GetName();
                }
            }
        }
        else if( name == "cn" ) {
            eq = pelt->getCharData();
        }
        else if( name == "ci" ) {
            eq = pelt->getCharData();
        }
        else if( name == "pi" ) {
            eq = "3.14159265358979323846";
        }
        else {
            RAVELOG_WARN(str(boost::format("mathml unprocessed tag: %s")));
        }
        return eq;
    }

    bool _computeConvexHull(const vector<Vector>& verts, KinBody::Link::TRIMESH& trimesh)
    {
        RAVELOG_ERROR("convex hulls not supported\n");
        // since there is no easy way of getting geometry boxes, check if convex hull is a box
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

    inline std::string _ConvertToOpenRAVEName(const std::string& name) {
        if( name.size() == 0 ) {
            return str(boost::format("__dummy%d")%_nGlobalIndex++);
        }
        return ConvertToOpenRAVEName(name);
    }

    inline static dReal _GetUnitScale(daeElementRef pelt, dReal startscale)
    {
        // getChild could be optimized since asset tag is supposed to appear as the first element
        domExtraRef pextra = daeSafeCast<domExtra> (pelt->getChild("extra"));
        if( !!pextra && !!pextra->getAsset() && !!pextra->getAsset()->getUnit() ) {
            return pextra->getAsset()->getUnit()->getMeter();
        }
        if( !!pelt->getParent() ) {
            return _GetUnitScale(pelt->getParent(),startscale);
        }
        return startscale;
    }

    boost::shared_ptr<DAE> _dae;
    domCOLLADA* _dom;
    EnvironmentBasePtr _penv;
    dReal _fGlobalScale;
    std::map<KinBody::JointPtr, std::vector<dReal> > _mapJointUnits;
    std::map<std::string,KinBody::JointPtr> _mapJointIds;
    string _prefix;
    int _nGlobalSensorId, _nGlobalManipulatorId, _nGlobalIndex;
    std::string _filename;
    bool _bOpeningZAE;
    bool _bSkipGeometry;
    std::set<KinBody::LinkPtr> _setInitialLinks;
    std::set<KinBody::JointPtr> _setInitialJoints;
    std::set<RobotBase::ManipulatorPtr> _setInitialManipulators;
    std::set<RobotBase::AttachedSensorPtr> _setInitialSensors;
};

bool RaveParseColladaFile(EnvironmentBasePtr penv, const string& filename,const AttributesList& atts)
{
    ColladaReader reader(penv);
    boost::shared_ptr<pair<string,string> > filedata = OpenRAVEXMLParser::FindFile(filename);
    if (!filedata || !reader.InitFromFile(filedata->second,atts)) {
        return false;
    }
    return reader.Extract();
}

bool RaveParseColladaFile(EnvironmentBasePtr penv, KinBodyPtr& pbody, const string& filename,const AttributesList& atts)
{
    ColladaReader reader(penv);
    boost::shared_ptr<pair<string,string> > filedata = OpenRAVEXMLParser::FindFile(filename);
    if (!filedata || !reader.InitFromFile(filedata->second,atts)) {
        return false;
    }
    return reader.Extract(pbody);
}

bool RaveParseColladaFile(EnvironmentBasePtr penv, RobotBasePtr& probot, const string& filename,const AttributesList& atts)
{
    ColladaReader reader(penv);
    boost::shared_ptr<pair<string,string> > filedata = OpenRAVEXMLParser::FindFile(filename);
    if (!filedata || !reader.InitFromFile(filedata->second,atts)) {
        return false;
    }
    return reader.Extract(probot);
}

bool RaveParseColladaData(EnvironmentBasePtr penv, const string& pdata,const AttributesList& atts) {
    ColladaReader reader(penv);
    if (!reader.InitFromData(pdata,atts)) {
        return false;
    }
    return reader.Extract();
}

bool RaveParseColladaData(EnvironmentBasePtr penv, KinBodyPtr& pbody, const string& pdata,const AttributesList& atts)
{
    ColladaReader reader(penv);
    if (!reader.InitFromData(pdata,atts)) {
        return false;
    }
    return reader.Extract(pbody);
}

bool RaveParseColladaData(EnvironmentBasePtr penv, RobotBasePtr& probot, const string& pdata,const AttributesList& atts)
{
    ColladaReader reader(penv);
    if (!reader.InitFromData(pdata,atts)) {
        return false;
    }
    return reader.Extract(probot);
}
