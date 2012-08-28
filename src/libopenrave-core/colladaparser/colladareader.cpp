// -*- coding: utf-8 -*-
// Copyright (C) 2006-2012 Rosen Diankov <rosen.diankov@gmail.com>, Stefan Ulbrich, Gustavo Rodriguez
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
#include "colladacommon.h"
#include <boost/algorithm/string.hpp>
using namespace ColladaDOM150;

class ColladaReader : public daeErrorHandler
{
    template <typename T>
    inline static std::string getSid(T t)
    {
        if( !t->getSid() ) {
            return "(None)";
        }
        return t->getSid();
    }

    class daeOpenRAVEURIResolver : public daeURIResolver
    {
public:
        daeOpenRAVEURIResolver(DAE& dae, const std::string& scheme, ColladaReader* preader) : daeURIResolver(dae), _scheme(scheme), _preader(preader) {
        }

        ~daeOpenRAVEURIResolver() {
        }

public:
        virtual daeElement* resolveElement(const daeURI& uri) {
            string docuri = cdom::assembleUri(uri.scheme(), uri.authority(), uri.path(), "", "");
            daeDocument* doc = dae->getDatabase()->getDocument(docuri.c_str(), true);
            if( !doc ) {
                if( uri.scheme() == _scheme ) {
                    if( uri.path().size() == 0 ) {
                        return NULL;
                    }
                    // remove first slash because we need relative file
                    std::string docurifull="file:";
                    if( uri.path().at(0) == '/' ) {
                        docurifull += RaveFindLocalFile(uri.path().substr(1), "/");
                    }
                    else {
                        docurifull += RaveFindLocalFile(uri.path(), "/");
                    }
                    if( docurifull.size() == 5 ) {
                        RAVELOG_WARN(str(boost::format("daeOpenRAVEURIResolver::resolveElement() - Failed to resolve %s ")%uri.str()));
                        return NULL;
                    }
                    domCOLLADA* proxy = (domCOLLADA*)dae->open(docurifull);
                    if( !!proxy ) {
                        if( !!_preader ) {
                            _preader->_mapInverseResolvedURIList.insert(make_pair(docurifull,daeURI(*uri.getDAE(),docuri)));
                        }
                        doc = uri.getDAE()->getDatabase()->getDocument(docurifull.c_str(),true);
                        if( !!doc ) {
                            // insert it again with the original URI
                            uri.getDAE()->getDatabase()->insertDocument(docuri.c_str(),doc->getDomRoot(),&doc,doc->isZAERootDocument(),doc->getExtractedFileURI().getURI());
                        }
                    }
                }
                else {
                    dae->open(docuri);
                    doc = uri.getReferencedDocument();
                    if( !!doc && !!doc->getDocumentURI() ) {
                        // this document could be extracted from a zae, so have to link with the original zae
                        daeUInt num = dae->getDatabase()->getDocumentCount();
                        for(daeUInt i = 0; i < num; ++i) {
                            daeDocument* doctest = dae->getDatabase()->getDocument(i);
                            if( doctest->getExtractedFileURI().str() == docuri ) {
                                // found
                                if( !!doctest->getDocumentURI() ) {
                                    _preader->_mapInverseResolvedURIList.insert(make_pair(doc->getDocumentURI()->str(),*doctest->getDocumentURI()));
                                    break;
                                }
                            }
                        }
                    }
                }
                if (!doc) {
                    RAVELOG_WARN(str(boost::format("daeOpenRAVEURIResolver::resolveElement() - Failed to resolve %s ")%uri.str()));
                    return NULL;
                }
            }
            daeElement* elt = dae->getDatabase()->idLookup(uri.id(), doc);
            if (!elt) {
                RAVELOG_WARN(str(boost::format("daeOpenRAVEURIResolver::resolveElement() - Failed to resolve %s ")%uri.str()));
            }
            return elt;
        }
        virtual daeString getName() {
            return "OpenRAVEResolver";
        }

        std::string _scheme;
        ColladaReader *_preader;
    };

    class InterfaceType
    {
public:
        InterfaceType(const std::string& type, const std::string& name) : type(type), name(name) {
        }
        std::string type, name;
    };
    typedef boost::shared_ptr<InterfaceType> InterfaceTypePtr;

    /// \brief bindings for instance models
    class ModelBinding
    {
public:
        ModelBinding(domNodeRef node, domInstance_kinematics_modelRef ikmodel) : _node(node), _ikmodel(ikmodel) {
        }
        domNodeRef _node;
        domInstance_kinematics_modelRef _ikmodel;
        domInstance_physics_modelRef _ipmodel;
    };

    /// \brief bindings for joints between different specs
    class JointAxisBinding
    {
public:
        JointAxisBinding(daeElementRef pvisualtrans, domAxis_constraintRef pkinematicaxis, dReal jointvalue, domKinematics_axis_infoRef kinematics_axis_info, domMotion_axis_infoRef motion_axis_info) : pvisualtrans(pvisualtrans), pkinematicaxis(pkinematicaxis), jointvalue(jointvalue), kinematics_axis_info(kinematics_axis_info), motion_axis_info(motion_axis_info),_iaxis(0) {
            BOOST_ASSERT( !!pkinematicaxis );
            daeElement* pae = pvisualtrans->getParentElement();
            while (!!pae) {
                visualnode = daeSafeCast<domNode> (pae);
                if (!!visualnode) {
                    break;
                }
                visualnode = _InstantiateNode(pae);
                if( !!visualnode ) {
                    break;
                }
                pae = pae->getParentElement();
            }

            if (!visualnode) {
                RAVELOG_WARN(str(boost::format("couldn't find parent node of element id %s, sid %s\n")%pkinematicaxis->getID()%getSid(pkinematicaxis)));
            }
        }

        daeElementRef pvisualtrans;
        domAxis_constraintRef pkinematicaxis;
        dReal jointvalue;
        domNodeRef visualnode;
        domKinematics_axis_infoRef kinematics_axis_info;
        domMotion_axis_infoRef motion_axis_info;
        KinBody::JointPtr _pjoint;
        int _iaxis;
    };

    /// \brief bindings for links between different specs
    class LinkBinding
    {
public:
        KinBody::LinkPtr _link;
        domNodeRef _node;
        boost::shared_ptr<daeURI> _nodeurifromphysics; ///< node URL from instance_rigid_body
        domLinkRef _domlink;
        domInstance_rigid_bodyRef _irigidbody;
        domRigid_bodyRef _rigidbody;
        domNodeRef _nodephysicsoffset; ///< the physics rigid body is in this coordinate system
        domInstance_physics_modelRef _ipmodel; ///< parent to _irigidbody
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
        if( sizeof(daeFloat) == 4 ) {
            RAVELOG_WARN("collada-dom compiled with 32-bit floating-point, so there might be precision errors\n");
        }
    }
    virtual ~ColladaReader() {
    }

    bool InitFromURI(const string& uristr, const AttributesList& atts)
    {
        _InitPreOpen(atts);
        _bOpeningZAE = uristr.find(".zae") == uristr.size()-4;
        daeURI urioriginal(*_dae, uristr);
        std::string uriresolved;

        if( find(_vOpenRAVESchemeAliases.begin(),_vOpenRAVESchemeAliases.end(),urioriginal.scheme()) != _vOpenRAVESchemeAliases.end() ) {
            if( urioriginal.path().size() == 0 ) {
                return NULL;
            }
            // remove first slash because we need relative file
            uriresolved="file:";
            if( urioriginal.path().at(0) == '/' ) {
                uriresolved += RaveFindLocalFile(urioriginal.path().substr(1), "/");
            }
            else {
                uriresolved += RaveFindLocalFile(urioriginal.path(), "/");
            }
            if( uriresolved.size() == 5 ) {
                return false;
            }
        }
        _dom = daeSafeCast<domCOLLADA>(_dae->open(uriresolved.size() > 0 ? uriresolved : uristr));
        if( !!_dom && uriresolved.size() > 0 ) {
            _mapInverseResolvedURIList.insert(make_pair(uriresolved, daeURI(*_dae,urioriginal.str())));
        }
        return _InitPostOpen(atts);
    }

    bool InitFromFile(const string& filename, const AttributesList& atts)
    {
        _InitPreOpen(atts);
        _bOpeningZAE = filename.find(".zae") == filename.size()-4;
        _dom = daeSafeCast<domCOLLADA>(_dae->open(filename));
        _bOpeningZAE = false;
        if (!_dom) {
            return false;
        }
        _filename=filename;
        return _InitPostOpen(atts);
    }

    bool InitFromData(const string& pdata,const AttributesList& atts)
    {
        _InitPreOpen(atts);
        _dom = daeSafeCast<domCOLLADA>(_dae->openFromMemory(".",pdata.c_str()));
        if (!_dom) {
            return false;
        }
        return _InitPostOpen(atts);
    }

    bool _InitPreOpen(const AttributesList& atts)
    {
        RAVELOG_VERBOSE(str(boost::format("init COLLADA reader version: %s, namespace: %s\n")%COLLADA_VERSION%COLLADA_NAMESPACE));
        _dae.reset(new DAE);
        _bSkipGeometry = false;
        _vOpenRAVESchemeAliases.resize(0);
        FOREACHC(itatt,atts) {
            if( itatt->first == "skipgeometry" ) {
                _bSkipGeometry = _stricmp(itatt->second.c_str(), "true") == 0 || itatt->second=="1";
            }
            else if( itatt->first == "prefix" ) {
                _prefix = itatt->second;
            }
            else if( itatt->first == "name" ) {
                RAVELOG_VERBOSE(str(boost::format("collada reader robot name=%s is processed from xmlreaders side")%itatt->second));
            }
            else if( itatt->first == "openravescheme" ) {
                _dae->getURIResolvers().list().clear();
                stringstream ss(itatt->second);
                _vOpenRAVESchemeAliases = std::vector<std::string>((istream_iterator<std::string>(ss)), istream_iterator<std::string>());
            }
            else if( itatt->first == "uripassword" ) {
                size_t passwordindex = itatt->second.find_last_of(' ');
                if( passwordindex != std::string::npos ) {
                    string name = itatt->second.substr(0,passwordindex);
                    boost::trim(name);
                    string password = itatt->second.substr(passwordindex+1);
                    boost::trim(password);
                }
            }
            else if( itatt->first == "scalegeometry" ) {
            }
            else {
                //RAVELOG_WARN(str(boost::format("collada reader unprocessed attribute pair: %s:%s")%itatt->first%itatt->second));
                if( !!_dae->getIOPlugin() ) {
                    _dae->getIOPlugin()->setOption(itatt->first.c_str(),itatt->second.c_str());
                }
            }
        }

        if( _vOpenRAVESchemeAliases.size() == 0 ) {
            _vOpenRAVESchemeAliases.push_back("openrave");
        }
        FOREACHC(itname,_vOpenRAVESchemeAliases) {
            _dae->getURIResolvers().list().prepend(new daeOpenRAVEURIResolver(*_dae,*itname,this));
        }
        return true;
    }

    bool _InitPostOpen(const AttributesList& atts)
    {
        _fGlobalScale = 1;
        if( !!_dom->getAsset() ) {
            if( !!_dom->getAsset()->getUnit() ) {
                _fGlobalScale = _dom->getAsset()->getUnit()->getMeter();
            }
        }
        FOREACHC(itatt,atts) {
            if( itatt->first == "scalegeometry" ) {
                stringstream ss(itatt->second);
                Vector v(1,1,1);
                ss >> v.x;
                _fGlobalScale *= v.x;
            }
        }
        return true;
    }

    /// \brief Extract all possible collada scene objects into the environment
    bool Extract()
    {
        uint64_t starttime = utils::GetNanoPerformanceTime();
        domCOLLADA::domSceneRef allscene = _dom->getScene();
        if( !allscene ) {
            return false;
        }

        //  parse each instance kinematics scene
        vector<std::string>  vprocessednodes;
        std::vector<KinematicsSceneBindings> allbindings(allscene->getInstance_kinematics_scene_array().getCount());
        for (size_t iscene = 0; iscene < allscene->getInstance_kinematics_scene_array().getCount(); iscene++) {
            domInstance_kinematics_sceneRef kiscene = allscene->getInstance_kinematics_scene_array()[iscene];
            domKinematics_sceneRef kscene = daeSafeCast<domKinematics_scene> (kiscene->getUrl().getElement().cast());
            if (!kscene) {
                continue;
            }

            KinematicsSceneBindings& bindings = allbindings[iscene];
            _ExtractKinematicsVisualBindings(allscene->getInstance_visual_scene(),kiscene,bindings);
            _ExtractPhysicsBindings(allscene,bindings);
            FOREACH(itbinding,bindings.listModelBindings) {
                if( !!itbinding->_node->getId() ) {
                    vprocessednodes.push_back(itbinding->_node->getId());
                }
            }
            for(size_t ias = 0; ias < kscene->getInstance_articulated_system_array().getCount(); ++ias) {
                KinBodyPtr pbody;
                if( ExtractArticulatedSystem(pbody, kscene->getInstance_articulated_system_array()[ias], bindings) && !!pbody ) {
                    RAVELOG_DEBUG(str(boost::format("Robot %s added to the environment ...\n")%pbody->GetName()));
                    _penv->Add(pbody,true);
                    _SetDOFValues(pbody,bindings);
                }
            }
            for(size_t ikmodel = 0; ikmodel < kscene->getInstance_kinematics_model_array().getCount(); ++ikmodel) {
                KinBodyPtr pbody;
                if( ExtractKinematicsModel(pbody, kscene->getInstance_kinematics_model_array()[ikmodel], bindings) && !!pbody ) {
                    RAVELOG_VERBOSE(str(boost::format("Kinbody %s added to the environment\n")%pbody->GetName()));
                    _penv->Add(pbody,true);
                    _SetDOFValues(pbody,bindings);
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
                    _penv->Add(pbody, true);
                }
            }
        }

        // process grabbed objects
        for(size_t iscene = 0; iscene < allscene->getInstance_physics_scene_array().getCount(); ++iscene) {
            domInstance_with_extraRef piscene = allscene->getInstance_physics_scene_array()[iscene];
            for(size_t ie = 0; ie < piscene->getExtra_array().getCount(); ++ie) {
                domExtraRef pextra = piscene->getExtra_array()[ie];
                std::string extra_type = pextra->getType();
                if( extra_type == "dynamic_rigid_constraints" ) {
                    domTechniqueRef tec = _ExtractOpenRAVEProfile(pextra->getTechnique_array());
                    if( !!tec ) {
                        for(size_t ic = 0; ic < tec->getContents().getCount(); ++ic) {
                            daeElementRef pchild = tec->getContents()[ic];
                            if( std::string(pchild->getElementName()) == "rigid_constraint" ) {
                                // parse the rigid attachment
                                daeElementRef pref_attachment = pchild->getChild("ref_attachment");
                                daeElementRef pattachment = pchild->getChild("attachment");
                                if( !!pref_attachment && !!pattachment ) {
                                    string ref_attachment_sid = pref_attachment->getAttribute("rigid_body");
                                    string attachment_sid = pattachment->getAttribute("rigid_body");
                                    daeElementRef ref_attachment_rigid_body = daeSidRef(ref_attachment_sid,piscene).resolve().elt;
                                    daeElementRef attachment_rigid_body = daeSidRef(attachment_sid,piscene).resolve().elt;
                                    if( !!ref_attachment_rigid_body && !!attachment_rigid_body ) {
                                        domInstance_physics_modelRef ref_ipmodel = daeSafeCast<domInstance_physics_model>(daeSidRef(ref_attachment_sid.substr(0,ref_attachment_sid.rfind('/')), piscene).resolve().elt);
                                        domInstance_physics_modelRef ipmodel = daeSafeCast<domInstance_physics_model>(daeSidRef(attachment_sid.substr(0,attachment_sid.rfind('/')), piscene).resolve().elt);
                                        // get the instance_rigid_body
                                        domInstance_rigid_bodyRef ref_irigidbody, irigidbody;
                                        string sidfind = ref_attachment_rigid_body->getAttribute("sid");
                                        KinBody::LinkPtr ref_link, link;
                                        for(size_t irigid = 0; irigid < ref_ipmodel->getInstance_rigid_body_array().getCount(); ++irigid) {
                                            domInstance_rigid_bodyRef temprigid = ref_ipmodel->getInstance_rigid_body_array()[irigid];
                                            if( temprigid->getBody() == sidfind ) {
                                                ref_irigidbody = temprigid;
                                                string nodeid = temprigid->getTarget().getElement()->getID();
                                                FOREACH(itbinding,allbindings) {
                                                    FOREACH(itlinkbinding, itbinding->listLinkBindings) {
                                                        if( nodeid == itlinkbinding->_node->getID() ) {
                                                            ref_link = itlinkbinding->_link;
                                                            break;
                                                        }
                                                    }
                                                    if( !!ref_link ) {
                                                        break;
                                                    }
                                                }
                                                break;
                                            }
                                        }
                                        sidfind = attachment_rigid_body->getAttribute("sid");
                                        for(size_t irigid = 0; irigid < ipmodel->getInstance_rigid_body_array().getCount(); ++irigid) {
                                            domInstance_rigid_bodyRef temprigid = ipmodel->getInstance_rigid_body_array()[irigid];
                                            if( temprigid->getBody() == sidfind ) {
                                                irigidbody = temprigid;
                                                string nodeid = temprigid->getTarget().getElement()->getID();
                                                FOREACH(itbinding,allbindings) {
                                                    FOREACH(itlinkbinding, itbinding->listLinkBindings) {
                                                        if( nodeid == itlinkbinding->_node->getID() ) {
                                                            link = itlinkbinding->_link;
                                                            break;
                                                        }
                                                    }
                                                    if( !!link ) {
                                                        break;
                                                    }
                                                }
                                                break;
                                            }
                                        }

                                        if( !!ref_link && !!link ) {
                                            RobotBasePtr probot = RaveInterfaceCast<RobotBase>(ref_link->GetParent());
                                            if( !!probot ) {
                                                probot->Grab(link->GetParent(),ref_link);
                                            }
                                            else {
                                                RAVELOG_WARN(str(boost::format("%s needs to be a robot in order to grab")%ref_link->GetParent()->GetName()));
                                            }
                                        }
                                        else {
                                            RAVELOG_WARN(str(boost::format("failed to find KinBody::LinkPtr from instance_rigid_body sidrefs: %s and %s")%ref_attachment_sid%attachment_sid));
                                        }
                                    }
                                    else {
                                        RAVELOG_WARN(str(boost::format("invalid rigid_body references for rigid_constraint %s")%pchild->getAttribute("sid")));
                                    }
                                }
                                else {
                                    RAVELOG_WARN(str(boost::format("could not find ref_attachment and attachment elements for rigid_constraint %s")%pchild->getAttribute("sid")));
                                }
                            }
                        }
                    }
                }
            }
        }

        RAVELOG_VERBOSE("collada read time %fs\n",(utils::GetNanoPerformanceTime()-starttime)*1e-9);
        return true;
    }

    /// \brief sets the dof values of the body given the scene bindings. in the future might also set the body transformation?
    void _SetDOFValues(KinBodyPtr pbody, KinematicsSceneBindings& bindings)
    {
        vector<dReal> values;
        pbody->GetDOFValues(values);
        FOREACH(itaxisbinding,bindings.listAxisBindings) {
            if( !!itaxisbinding->_pjoint && itaxisbinding->_pjoint->GetParent() == pbody ) {
                if( itaxisbinding->_pjoint->GetDOFIndex() >= 0 ) {
                    values.at(itaxisbinding->_pjoint->GetDOFIndex()+itaxisbinding->_iaxis) = itaxisbinding->jointvalue;
                }
            }
        }
        pbody->SetDOFValues(values);
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
                KinBodyPtr pbody=probot;
                if( ExtractArticulatedSystem(pbody, kscene->getInstance_articulated_system_array()[ias], *bindings) && !!pbody ) {
                    probot = RaveInterfaceCast<RobotBase>(pbody);
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
                        (*itmanip)->_info._name = _prefix + (*itmanip)->_info._name;
                        FOREACH(itgrippername,(*itmanip)->_info._vGripperJointNames) {
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
                        utils::SearchAndReplace(neweq,(*itjoint)->_vmimic[idof]->_equations[ieq],jointnamepairs);
                        (*itjoint)->_vmimic[idof]->_equations[ieq] = neweq;
                    }
                }
            }
        }
    }

    /// \brief extracts an articulated system. Note that an articulated system can include other articulated systems
    /// \param probot the robot to be created from the system
    bool ExtractArticulatedSystem(KinBodyPtr& pbody, domInstance_articulated_systemRef ias, KinematicsSceneBindings& bindings)
    {
        if( !ias ) {
            return false;
        }
        RAVELOG_DEBUG(str(boost::format("instance articulated system sid %s\n")%getSid(ias)));

        domArticulated_systemRef articulated_system = daeSafeCast<domArticulated_system> (ias->getUrl().getElement().cast());
        if( !articulated_system ) {
            return false;
        }
        if( !pbody ) {
            InterfaceTypePtr pinterface_type = _ExtractInterfaceType(ias->getExtra_array());
            if( !pinterface_type ) {
                pinterface_type = _ExtractInterfaceType(articulated_system->getExtra_array());
            }
            if( !!pinterface_type ) {
                if( pinterface_type->type == "kinbody" ) {
                    pbody = RaveCreateKinBody(_penv,pinterface_type->name);
                }
                else if( pinterface_type->type.size() == 0 || pinterface_type->type == "robot" ) {
                    pbody = RaveCreateRobot(_penv,pinterface_type->name);
                }
                else {
                    RAVELOG_WARN("invalid interface_type\n");
                    return false;
                }
            }
            if( !pbody ) {
                pbody = RaveCreateRobot(_penv,"genericrobot");
                if( !pbody ) {
                    pbody = RaveCreateRobot(_penv,"");
                    RAVELOG_WARN("creating default robot with no controller support\n");
                }
            }
            _mapJointUnits.clear();
            _mapJointIds.clear();
        }
        if( pbody->__struri.size() == 0 ) {
            pbody->__struri = _filename;
        }

        // set the name
        if(( pbody->GetName().size() == 0) && !!ias->getName() ) {
            pbody->SetName(ias->getName());
        }
        if(( pbody->GetName().size() == 0) && !!ias->getSid()) {
            pbody->SetName(ias->getSid());
        }
        if(( pbody->GetName().size() == 0) && !!articulated_system->getName() ) {
            pbody->SetName(articulated_system->getName());
        }
        if(( pbody->GetName().size() == 0) && !!articulated_system->getId()) {
            pbody->SetName(articulated_system->getId());
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
            if( !ExtractArticulatedSystem(pbody,ias_new,bindings) ) {
                return false;
            }

            // write the axis parameters
            ColladaXMLReadablePtr pcolladainfo = boost::dynamic_pointer_cast<ColladaXMLReadable>(pbody->GetReadableInterface(ColladaXMLReadable::GetXMLIdStatic()));
            if( !!pcolladainfo ) {
                pcolladainfo->_articulated_systemURL = _MakeFullURI(ias->getUrl(),ias); // updated with a articulated_system up the hierarchy
                pcolladainfo->_bindingAxesSIDs.resize(pbody->GetDOF());
                // go through each parameter and see what axis it resolves to
                for(size_t iparam = 0; iparam < ias->getNewparam_array().getCount(); ++iparam) {
                    domKinematics_newparamRef param = ias->getNewparam_array()[iparam];
                    // find the axis index
                    if( !!param->getSIDREF() ) {
                        daeElement* pelt = daeSidRef(param->getSIDREF()->getValue(),ias).resolve().elt;
                        if( !!pelt ) {
                            domAxis_constraintRef pjointaxis = daeSafeCast<domAxis_constraint>(pelt);
                            if( !!pjointaxis ) {
                                FOREACH(itaxis, bindings.listAxisBindings) {
                                    if( !!itaxis->_pjoint && itaxis->_pjoint->GetParent() == pbody && itaxis->_pjoint->GetDOFIndex() >= 0 ) {
                                        if( itaxis->pkinematicaxis == pjointaxis ) {
                                            std::list<ModelBinding>::iterator itmodel = _FindParentModel(itaxis->visualnode,bindings.listModelBindings);
                                            BOOST_ASSERT(itmodel != bindings.listModelBindings.end());
                                            pcolladainfo->_bindingAxesSIDs.at(itaxis->_pjoint->GetDOFIndex()+itaxis->_iaxis) = ColladaXMLReadable::AxisBinding(param->getSIDREF()->getValue(), itaxis->pvisualtrans->getAttribute("sid"));
                                        }
                                    }
                                }
                            }
                            else {
                                domInstance_kinematics_modelRef ikmodel = daeSafeCast<domInstance_kinematics_model>(pelt);
                                if( !!ikmodel ) {
                                    // found a newparam to ikmodel, so have to store
                                    string kmodeluri = _MakeFullURI(ikmodel->getUrl(), ikmodel);
                                    FOREACH(itmodel,pcolladainfo->_bindingModelURLs) {
                                        if( itmodel->kmodel == kmodeluri ) {
                                            itmodel->ikmodelsidref = param->getSIDREF()->getValue();
                                        }
                                    }
                                }
                                else {
                                    // expected since newparam can point to custom elements
                                }
                            }
                        }
                    }
                }
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
            if (!pbody) {
                pbody = RaveCreateRobot(_penv, "GenericRobot");
                if( !pbody ) {
                    pbody = RaveCreateRobot(_penv, "");
                }
                _mapJointUnits.clear();
                _mapJointIds.clear();
            }

            ColladaXMLReadablePtr pcolladainfo(new ColladaXMLReadable());
            pcolladainfo->_articulated_systemURL = _MakeFullURI(ias->getUrl(),ias);
            std::map<domInstance_physics_modelRef,int> mapModelIndices;
            for(size_t ik = 0; ik < articulated_system->getKinematics()->getInstance_kinematics_model_array().getCount(); ++ik) {
                domInstance_kinematics_modelRef ikmodel = articulated_system->getKinematics()->getInstance_kinematics_model_array()[ik];
                ExtractKinematicsModel(pbody,ikmodel,bindings);
                FOREACH(it, bindings.listModelBindings) {
                    if( it->_ikmodel == ikmodel ) {
                        if( !!it->_ikmodel && !!it->_ipmodel && !!it->_node ) {
                            mapModelIndices[it->_ipmodel] = (int)pcolladainfo->_bindingModelURLs.size();
                            ColladaXMLReadable::ModelBinding mbinding(_MakeFullURI(it->_ikmodel->getUrl(), it->_ikmodel), _MakeFullURI(it->_ipmodel->getUrl(), it->_ipmodel), _MakeFullURIFromId(it->_node->getId(),it->_node));
                            pcolladainfo->_bindingModelURLs.push_back(mbinding);
                        }
                        break;
                    }
                }
            }

            pcolladainfo->_bindingLinkSIDs.resize(pbody->GetLinks().size());
            FOREACH(itlink, bindings.listLinkBindings) {
                const LinkBinding& lb = *itlink;
                if( !!lb._link && lb._link->GetParent() == pbody ) {
                    if( !!lb._domlink && !!lb._rigidbody && !!lb._node ) {
                        if( !!lb._domlink->getSid() && !!lb._rigidbody->getSid() && !!lb._node->getSid() ) {
                            // find out the model that this link binding belongs to
                            ColladaXMLReadable::Binding binding(lb._domlink->getSid(), lb._rigidbody->getSid(), _MakeFullURIFromId(lb._node->getId(),lb._node)); //lb._node->getSid());
                            if( mapModelIndices.find(lb._ipmodel) != mapModelIndices.end() ) {
                                binding.index = mapModelIndices[lb._ipmodel];
                            }
                            pcolladainfo->_bindingLinkSIDs.at(lb._link->GetIndex()) = binding;
                        }
                    }
                    else {
                        RAVELOG_WARN(str(boost::format("link %s doesn't have all bindings")%lb._link->GetName()));
                    }
                }
            }
            pbody->SetReadableInterface(pcolladainfo->GetXMLId(),pcolladainfo);
        }

        RobotBasePtr probot = RaveInterfaceCast<RobotBase>(pbody);
        if( !!probot ) {
            ExtractRobotManipulators(probot, articulated_system);
            ExtractRobotAttachedSensors(probot, articulated_system);
            ExtractRobotAttachedActuators(probot, articulated_system);
        }
        _ExtractExtraData(pbody,ias->getExtra_array());
        return true;
    }

    bool ExtractKinematicsModel(KinBodyPtr& pkinbody, domInstance_kinematics_modelRef ikm, KinematicsSceneBindings& bindings)
    {
        if( !ikm ) {
            return false;
        }
        RAVELOG_DEBUG(str(boost::format("instance kinematics model sid %s\n")%getSid(ikm)));
        domKinematics_modelRef kmodel = daeSafeCast<domKinematics_model> (ikm->getUrl().getElement().cast());
        if (!kmodel) {
            RAVELOG_WARN(str(boost::format("%s does not reference valid kinematics\n")%getSid(ikm)));
            return false;
        }

        if( !pkinbody ) {
            InterfaceTypePtr pinterface_type = _ExtractInterfaceType(ikm->getExtra_array());
            if( !pinterface_type ) {
                pinterface_type = _ExtractInterfaceType(kmodel->getExtra_array());
            }
            if( !!pinterface_type ) {
                if( pinterface_type->type.size() == 0 || pinterface_type->type == "kinbody" ) {
                    pkinbody = RaveCreateKinBody(_penv,pinterface_type->name);
                }
                else if( pinterface_type->type == "robot" ) {
                    pkinbody = RaveCreateRobot(_penv,pinterface_type->name);
                }
                else {
                    RAVELOG_WARN("invalid interface_type\n");
                    return false;
                }
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

        // check if kmodel has asset/subject, if yes, then set it to the description
        if( !!kmodel->getAsset() ) {
            if( !!kmodel->getAsset()->getSubject() ) {
                if( !!kmodel->getAsset()->getSubject()->getValue() ) {
                    pkinbody->SetDescription(kmodel->getAsset()->getSubject()->getValue());
                }
            }
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
            RAVELOG_WARN(str(boost::format("failed to find visual node for instance kinematics model %s\n")%getSid(ikm)));
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
        plink->_t = getNodeParentTransform(pdomnode) * _ExtractFullTransform(pdomnode);
        bool bhasgeometry = ExtractGeometries(pdomnode,plink,bindings,vprocessednodes);
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
        if(( pkinbody->GetName().size() == 0) && !!kmodel->getId() ) {
            pkinbody->SetName(kmodel->getId());
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

        RAVELOG_VERBOSE(str(boost::format("Number of root links in kmodel %s: %d\n")%kmodel->getId()%ktec->getLink_array().getCount()));
        for (size_t ilink = 0; ilink < ktec->getLink_array().getCount(); ++ilink) {
            Transform tnode;
            if( ilink == 0 ) {
                tnode = getNodeParentTransform(pnode);
            }
            ExtractLink(pkinbody, ktec->getLink_array()[ilink], ilink == 0 ? pnode : domNodeRef(), tnode, vdomjoints, bindings);
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
                        catch(const std::exception &ex) {
                            RAVELOG_WARN(str(boost::format("failed to parse formula %s for target %s: %s")%equationtype%pjoint->GetName()%ex.what()));
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
                catch(const std::exception &ex) {
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
            // convert any instance_node child elements to real nodes
            daeTArray<domInstance_nodeRef> vinstance_nodes = pdomnode->getChildrenByType<domInstance_node>();
            for(size_t inode = 0; inode < vinstance_nodes.getCount(); ++inode) {
                _InstantiateNode(vinstance_nodes[inode]);
            }

            RAVELOG_VERBOSE(str(boost::format("Node Id %s and Name %s\n")%pdomnode->getId()%pdomnode->getName()));

            bool bFoundBinding = false;
            FOREACH(itlinkbinding, bindings.listLinkBindings) {
                // visual scenes can have weird hierarchy which makes physics assignment difficult
                LinkBinding& lb = *itlinkbinding;
                if( !lb._node ) {
                    // have to resolve _node
                    lb._node = daeSafeCast<domNode>(lb._nodeurifromphysics->getElement().cast());
                    if( !lb._node ) {
                        continue;
                    }
                }
                if( _CompareElementURI(pdomnode,lb._node) > 0 ) {
                    bFoundBinding = true;
                    irigidbody = lb._irigidbody;
                    rigidbody = lb._rigidbody;
                    lb._domlink = pdomlink;
                    lb._link = plink;
                    lb._node = pdomnode;
                    if( !!lb._nodephysicsoffset ) {
                        // set the rigid offset to the transform of the instance physics model parent
                        trigidoffset = getNodeParentTransform(lb._nodephysicsoffset) * _ExtractFullTransform(lb._nodephysicsoffset);
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
            if( !!rigiddata->getInertia() ) {
                plink->_vinertiamoments[0] = rigiddata->getInertia()->getValue()[0];
                plink->_vinertiamoments[1] = rigiddata->getInertia()->getValue()[1];
                plink->_vinertiamoments[2] = rigiddata->getInertia()->getValue()[2];
            }
            plink->_tMassFrame = plink->_t.inverse() * tmassframe;
            if( !!rigiddata->getDynamic() ) {
                plink->_bStatic = !rigiddata->getDynamic()->getValue();
            }
        }
        else {
            RAVELOG_WARN(str(boost::format("failed to find rigid_body info for link %s")%plink->_name));
        }

        if (!pdomlink) {
            ExtractGeometries(pdomnode,plink,bindings,std::vector<std::string>());
        }
        else {
            RAVELOG_DEBUG(str(boost::format("Attachment link elements: %d")%pdomlink->getAttachment_full_array().getCount()));

            {
                stringstream ss; ss << plink->GetName() << ": " << plink->_t << endl;
                RAVELOG_DEBUG(ss.str());
            }

            // Get the geometry
            if( !ExtractGeometries(pdomnode,plink,bindings,std::vector<std::string>()) ) {
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

                for (size_t ic = 0; ic < vdomaxes.getCount(); ++ic) {
                    FOREACH(itaxisbinding,bindings.listAxisBindings) {
                        if (vdomaxes[ic] == itaxisbinding->pkinematicaxis) {
                            itaxisbinding->_pjoint = pjoint;
                            itaxisbinding->_iaxis = ic;
                        }
                    }
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
                    bool has_soft_limits = false, has_hard_limits = false;
                    boost::shared_ptr<uint8_t> is_circular;

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
                                    // TODO, necessary?
                                    pjoint->_voffsets[ic] = 0.5f * (pjoint->_vlowerlimit.at(ic) + pjoint->_vupperlimit[ic]);
                                }
                            }
                        }

                        for (size_t iparam = 0; iparam < kinematics_axis_info->getNewparam_array().getCount(); iparam++) {
                            domKinematics_newparamRef axisparam = kinematics_axis_info->getNewparam_array()[iparam];
                            if( !!axisparam->getSid() ) {
                                string paramsid = axisparam->getSid();
                                if( paramsid == "circular" ) {
                                    if( !!axisparam->getBool() ) {
                                        is_circular.reset(new uint8_t(axisparam->getBool()->getValue()));
                                    }
                                    else if( !!axisparam->getInt() ) {
                                        is_circular.reset(new uint8_t(axisparam->getInt()->getValue()!=0));
                                    }
                                    else {
                                        RAVELOG_WARN("failed to resolve axis_info/circular newparam\n");
                                    }
                                }
                                else if( paramsid == "planning_weight" ) {
                                    if( !!axisparam->getFloat() ) {
                                        if( axisparam->getFloat()->getValue() > 0 ) {
                                            pjoint->_vweights[ic] = axisparam->getFloat()->getValue();
                                        }
                                        else {
                                            RAVELOG_WARN(str(boost::format("bad joint weight %f")%axisparam->getFloat()->getValue()));
                                        }
                                    }
                                    else {
                                        RAVELOG_WARN("failed to resolve axis_info/planning_weight newparam\n");
                                    }
                                }
                                else if( paramsid == "discretization_resolution" ) {
                                    if( !!axisparam->getFloat() ) {
                                        if( axisparam->getFloat()->getValue() > 0 ) {
                                            pjoint->fResolution = axisparam->getFloat()->getValue();
                                        }
                                        else {
                                            RAVELOG_WARN(str(boost::format("bad joint resolution %f")%axisparam->getFloat()->getValue()));
                                        }
                                    }
                                    else {
                                        RAVELOG_WARN("failed to resolve axis_info/discretization_resolution newparam\n");
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
                                // TODO, necessary?
                                pjoint->_voffsets[ic] = 0.5f * (pjoint->_vlowerlimit[ic] + pjoint->_vupperlimit[ic]);
                            }
                        }
                    }

                    if( !!is_circular ) {
                        pjoint->_bIsCircular.at(ic) = *is_circular;
                    }

                    if( !has_soft_limits && !has_hard_limits && !joint_locked ) {
                        RAVELOG_VERBOSE(str(boost::format("There are NO LIMITS in joint %s ...\n")%pjoint->GetName()));
                        if( pjoint->IsRevolute(ic) ) {
                            if( !is_circular ) {
                                pjoint->_bIsCircular.at(ic) = true;
                            }
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
                RAVELOG_DEBUG(str(boost::format("joint dof: %d, links %s->%s\n")%pjoint->dofindex%plink->GetName()%pchildlink->GetName()));
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
    bool ExtractGeometries(const domNodeRef pdomnode,KinBody::LinkPtr plink, const KinematicsSceneBindings& bindings, const std::vector<std::string>& vprocessednodes)
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
                if ( pdomnode->getNode_array()[i] == it->visualnode) {
                    contains=true;
                    break;
                }
            }
            if (contains) {
                continue;
            }

            bhasgeometry |= ExtractGeometries(pdomnode->getNode_array()[i],plink, bindings, vprocessednodes);
            // Plink stayes the same for all children
            // replace pdomnode by child = pdomnode->getNode_array()[i]
            // hope for the best!
            // put everything in a subroutine in order to process pdomnode too!
        }

        std::list<KinBody::Link::GeometryInfo> listGeometryInfos;

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
            bhasgeometry |= ExtractGeometry(domgeom, mapmaterials, listGeometryInfos);
        }

        if( !bhasgeometry ) {
            RAVELOG_DEBUG(str(boost::format("node %s has no geometry\n")%pdomnode->getName()));
            return false;
        }

        TransformMatrix tnodeparent = getNodeParentTransform(pdomnode);
        TransformMatrix tmnodegeom = (TransformMatrix) plink->_t.inverse() * tnodeparent * _ExtractFullTransform(pdomnode);
        Transform tnodegeom;
        Vector vscale;
        decompose(tmnodegeom, tnodegeom, vscale);

        FOREACH(itgeominfo, listGeometryInfos) {
            //  Switch between different type of geometry PRIMITIVES
            Transform toriginal = itgeominfo->_t;
            itgeominfo->_t = tnodegeom * itgeominfo->_t;
            switch (itgeominfo->_type) {
            case KinBody::Link::GeomBox:
                itgeominfo->_vGeomData *= vscale;
                break;
            case KinBody::Link::GeomSphere:
                itgeominfo->_vGeomData *= max(vscale.z, max(vscale.x, vscale.y));
                break;
            case KinBody::Link::GeomCylinder:
                itgeominfo->_vGeomData.x *= max(vscale.x, vscale.y);
                itgeominfo->_vGeomData.y *= vscale.z;
                break;
            case KinBody::Link::GeomTrimesh:
                itgeominfo->_meshcollision.ApplyTransform(TransformMatrix(itgeominfo->_t).inverse() * tmnodegeom * TransformMatrix(toriginal));
                break;
            default:
                RAVELOG_WARN(str(boost::format("unknown geometry type: 0x%x")%itgeominfo->_type));
            }

            KinBody::Link::GeometryPtr pgeom(new KinBody::Link::Geometry(plink,*itgeominfo));
            pgeom->_info.InitCollisionMesh();
            plink->_vGeometries.push_back(pgeom);
            //  Append the collision mesh
            KinBody::Link::TRIMESH trimesh = pgeom->GetCollisionMesh();
            trimesh.ApplyTransform(pgeom->_info._t);
            plink->collision.Append(trimesh);
        }

        return bhasgeometry || listGeometryInfos.size() > 0;
    }

    /// Paint the Geometry with the color material
    /// \param  pmat    Material info of the COLLADA's model
    /// \param  geom    Geometry properties in OpenRAVE
    void FillGeometryColor(const domMaterialRef pmat, KinBody::Link::GeometryInfo& geom)
    {
        if( !!pmat && !!pmat->getInstance_effect() ) {
            domEffectRef peffect = daeSafeCast<domEffect>(pmat->getInstance_effect()->getUrl().getElement().cast());
            if( !!peffect ) {
                domProfile_common::domTechnique::domPhongRef pphong = daeSafeCast<domProfile_common::domTechnique::domPhong>(peffect->getDescendant(daeElement::matchType(domProfile_common::domTechnique::domPhong::ID())));
                if( !!pphong ) {
                    if( !!pphong->getAmbient() && !!pphong->getAmbient()->getColor() ) {
                        geom._vAmbientColor = getVector4(pphong->getAmbient()->getColor()->getValue());
                    }
                    if( !!pphong->getDiffuse() && !!pphong->getDiffuse()->getColor() ) {
                        geom._vDiffuseColor = getVector4(pphong->getDiffuse()->getColor()->getValue());
                    }
                    if( !!pphong->getTransparency() && !!pphong->getTransparency()->getFloat() ) {
                        geom._fTransparency = pphong->getTransparency()->getFloat()->getValue();
                    }
                }
            }
        }
    }

    /// Extract the Geometry in TRIANGLES and adds it to OpenRave
    /// \param triRef  Array of triangles of the COLLADA's model
    /// \param vertsRef    Array of vertices of the COLLADA's model
    /// \param mapmaterials    Materials applied to the geometry
    /// \param geom The geometry info to store
    /// \param transgeom transform all vertices before storing
    bool _ExtractGeometry(const domTrianglesRef triRef, const domVerticesRef vertsRef, const map<string,domMaterialRef>& mapmaterials, KinBody::Link::GeometryInfo& geom, const Transform& transgeom)
    {
        if( !triRef ) {
            return false;
        }

        KinBody::Link::TRIMESH& trimesh = geom._meshcollision;
        geom._type = KinBody::Link::GeomTrimesh;

        // resolve the material and assign correct colors to the geometry
        if( !!triRef->getMaterial() ) {
            map<string,domMaterialRef>::const_iterator itmat = mapmaterials.find(triRef->getMaterial());
            if( itmat != mapmaterials.end() ) {
                FillGeometryColor(itmat->second,geom);
            }
        }

        domUint triangleIndexStride = 0, vertexoffset = -1;
        domInput_local_offsetRef indexOffsetRef;

        for (unsigned int w=0; w<triRef->getInput_array().getCount(); w++) {
            domUint offset = triRef->getInput_array()[w]->getOffset();
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
        trimesh.indices.reserve(size_t(triRef->getCount())*3);
        trimesh.vertices.reserve(size_t(triRef->getCount())*3);
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
                    domUint k = vertexoffset;
                    domUint vertexStride = 3;     //instead of hardcoded stride, should use the 'accessor'
                    for(size_t itri = 0; itri < triRef->getCount(); ++itri) {
                        if(k+2*triangleIndexStride < indexArray.getCount() ) {
                            for (int j=0; j<3; j++) {
                                domUint index0 = indexArray.get(size_t(k))*vertexStride;
                                domFloat fl0 = listFloats.get(size_t(index0));
                                domFloat fl1 = listFloats.get(size_t(index0+1));
                                domFloat fl2 = listFloats.get(size_t(index0+2));
                                k+=triangleIndexStride;
                                trimesh.indices.push_back(trimesh.vertices.size());
                                trimesh.vertices.push_back(transgeom*Vector(fl0*fUnitScale,fl1*fUnitScale,fl2*fUnitScale));
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
        return true;
    }

    /// Extract the Geometry in TRIGLE FANS and adds it to OpenRave
    /// \param  triRef  Array of triangle fans of the COLLADA's model
    /// \param  vertsRef    Array of vertices of the COLLADA's model
    /// \param  mapmaterials    Materials applied to the geometry
    /// \param  geom The geometry info to store
    /// \param transgeom transform all vertices before storing
    bool _ExtractGeometry(const domTrifansRef triRef, const domVerticesRef vertsRef, const map<string,domMaterialRef>& mapmaterials, KinBody::Link::GeometryInfo& geom, const Transform& transgeom)
    {
        if( !triRef ) {
            return false;
        }
        KinBody::Link::TRIMESH& trimesh = geom._meshcollision;
        geom._type = KinBody::Link::GeomTrimesh;

        // resolve the material and assign correct colors to the geometry
        if( !!triRef->getMaterial() ) {
            map<string,domMaterialRef>::const_iterator itmat = mapmaterials.find(triRef->getMaterial());
            if( itmat != mapmaterials.end() ) {
                FillGeometryColor(itmat->second,geom);
            }
        }

        domUint triangleIndexStride = 0, vertexoffset = -1;
        domInput_local_offsetRef indexOffsetRef;

        for (unsigned int w=0; w<triRef->getInput_array().getCount(); w++) {
            domUint offset = triRef->getInput_array()[w]->getOffset();
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
        domUint primitivecount = triRef->getCount();
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
                        domUint k=vertexoffset;
                        domUint vertexStride = 3;     //instead of hardcoded stride, should use the 'accessor'
                        size_t usedindices = 3*(indexArray.getCount()-2);
                        if( trimesh.indices.capacity() < trimesh.indices.size()+usedindices ) {
                            trimesh.indices.reserve(trimesh.indices.size()+usedindices);
                        }
                        if( trimesh.vertices.capacity() < trimesh.vertices.size()+indexArray.getCount() ) {
                            trimesh.vertices.reserve(trimesh.vertices.size()+indexArray.getCount());
                        }
                        size_t startoffset = trimesh.vertices.size();
                        while(k < indexArray.getCount() ) {
                            domUint index0 = indexArray.get(size_t(k))*vertexStride;
                            domFloat fl0 = listFloats.get(size_t(index0));
                            domFloat fl1 = listFloats.get(size_t(index0+1));
                            domFloat fl2 = listFloats.get(size_t(index0+2));
                            k+=triangleIndexStride;
                            trimesh.vertices.push_back(transgeom*Vector(fl0*fUnitScale,fl1*fUnitScale,fl2*fUnitScale));
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
        return true;
    }

    /// Extract the Geometry in TRIANGLE STRIPS and adds it to OpenRave
    /// \param  triRef  Array of Triangle Strips of the COLLADA's model
    /// \param  vertsRef    Array of vertices of the COLLADA's model
    /// \param  mapmaterials    Materials applied to the geometry
    /// \param  geom The geometry info to store
    /// \param transgeom transform all vertices before storing
    bool _ExtractGeometry(const domTristripsRef triRef, const domVerticesRef vertsRef, const map<string,domMaterialRef>& mapmaterials, KinBody::Link::GeometryInfo& geom, const Transform& transgeom)
    {
        if( !triRef ) {
            return false;
        }
        KinBody::Link::TRIMESH& trimesh = geom._meshcollision;
        geom._type = KinBody::Link::GeomTrimesh;

        // resolve the material and assign correct colors to the geometry
        if( !!triRef->getMaterial() ) {
            map<string,domMaterialRef>::const_iterator itmat = mapmaterials.find(triRef->getMaterial());
            if( itmat != mapmaterials.end() ) {
                FillGeometryColor(itmat->second,geom);
            }
        }
        domUint triangleIndexStride = 0, vertexoffset = -1;
        domInput_local_offsetRef indexOffsetRef;

        for (size_t w=0; w<triRef->getInput_array().getCount(); w++) {
            domUint offset = triRef->getInput_array()[w]->getOffset();
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
        domUint primitivecount = triRef->getCount();
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
                        domUint k=vertexoffset;
                        domUint vertexStride = 3;     //instead of hardcoded stride, should use the 'accessor'
                        size_t usedindices = 3*(size_t(indexArray.getCount())-2);
                        if( trimesh.indices.capacity() < trimesh.indices.size()+usedindices ) {
                            trimesh.indices.reserve(trimesh.indices.size()+usedindices);
                        }
                        if( trimesh.vertices.capacity() < trimesh.vertices.size()+indexArray.getCount() ) {
                            trimesh.vertices.reserve(trimesh.vertices.size()+indexArray.getCount());
                        }

                        size_t startoffset = trimesh.vertices.size();
                        while(k < indexArray.getCount() ) {
                            domUint index0 = indexArray.get(size_t(k))*vertexStride;
                            domFloat fl0 = listFloats.get(size_t(index0));
                            domFloat fl1 = listFloats.get(size_t(index0+1));
                            domFloat fl2 = listFloats.get(size_t(index0+2));
                            k+=triangleIndexStride;
                            trimesh.vertices.push_back(transgeom*Vector(fl0*fUnitScale,fl1*fUnitScale,fl2*fUnitScale));
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
        return true;
    }

    /// Extract the Geometry in TRIANGLE STRIPS and adds it to OpenRave
    /// \param  triRef  Array of Triangle Strips of the COLLADA's model
    /// \param  vertsRef    Array of vertices of the COLLADA's model
    /// \param  mapmaterials    Materials applied to the geometry
    /// \param  geom The geometry info to store
    /// \param transgeom transform all vertices before storing
    bool _ExtractGeometry(const domPolylistRef triRef, const domVerticesRef vertsRef, const map<string,domMaterialRef>& mapmaterials, KinBody::Link::GeometryInfo& geom, const Transform& transgeom)
    {
        if( !triRef ) {
            return false;
        }
        KinBody::Link::TRIMESH& trimesh = geom._meshcollision;
        geom._type = KinBody::Link::GeomTrimesh;

        // resolve the material and assign correct colors to the geometry
        if( !!triRef->getMaterial() ) {
            map<string,domMaterialRef>::const_iterator itmat = mapmaterials.find(triRef->getMaterial());
            if( itmat != mapmaterials.end() ) {
                FillGeometryColor(itmat->second,geom);
            }
        }

        domUint triangleIndexStride = 0,vertexoffset = -1;
        domInput_local_offsetRef indexOffsetRef;
        for (unsigned int w=0; w<triRef->getInput_array().getCount(); w++) {
            domUint offset = triRef->getInput_array()[w]->getOffset();
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
                    domUint k=vertexoffset;
                    domUint vertexStride = 3;     //instead of hardcoded stride, should use the 'accessor'
                    for(size_t ipoly = 0; ipoly < triRef->getVcount()->getValue().getCount(); ++ipoly) {
                        domUint numverts = triRef->getVcount()->getValue()[ipoly];
                        if(( numverts > 0) &&( k+(numverts-1)*triangleIndexStride < indexArray.getCount()) ) {
                            size_t startoffset = trimesh.vertices.size();
                            for (size_t j=0; j<numverts; j++) {
                                domUint index0 = indexArray.get(size_t(k))*vertexStride;
                                domFloat fl0 = listFloats.get(size_t(index0));
                                domFloat fl1 = listFloats.get(size_t(index0+1));
                                domFloat fl2 = listFloats.get(size_t(index0+2));
                                k+=triangleIndexStride;
                                trimesh.vertices.push_back(transgeom*Vector(fl0*fUnitScale,fl1*fUnitScale,fl2*fUnitScale));
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
        return true;
    }

    domMaterialRef _ExtractFirstMaterial(const domGeometryRef domgeom, const map<string,domMaterialRef>& mapmaterials)
    {
        map<string,domMaterialRef>::const_iterator itmat;
        if (!!domgeom->getMesh()) {
            const domMeshRef meshRef = domgeom->getMesh();
            for (size_t tg = 0; tg<meshRef->getTriangles_array().getCount(); tg++) {
                itmat = mapmaterials.find(meshRef->getTriangles_array()[tg]->getMaterial());
                if( itmat != mapmaterials.end() ) {
                    return itmat->second;
                }
            }
            for (size_t tg = 0; tg<meshRef->getTrifans_array().getCount(); tg++) {
                itmat = mapmaterials.find(meshRef->getTrifans_array()[tg]->getMaterial());
                if( itmat != mapmaterials.end() ) {
                    return itmat->second;
                }
            }
            for (size_t tg = 0; tg<meshRef->getTristrips_array().getCount(); tg++) {
                itmat = mapmaterials.find(meshRef->getTristrips_array()[tg]->getMaterial());
                if( itmat != mapmaterials.end() ) {
                    return itmat->second;
                }
            }
            for (size_t tg = 0; tg<meshRef->getPolylist_array().getCount(); tg++) {
                itmat = mapmaterials.find(meshRef->getPolylist_array()[tg]->getMaterial());
                if( itmat != mapmaterials.end() ) {
                    return itmat->second;
                }
            }
            for (size_t tg = 0; tg<meshRef->getPolygons_array().getCount(); tg++) {
                itmat = mapmaterials.find(meshRef->getPolygons_array()[tg]->getMaterial());
                if( itmat != mapmaterials.end() ) {
                    return itmat->second;
                }
            }
        }
        else if( !!domgeom->getConvex_mesh() ) {
            const domConvex_meshRef convexRef = domgeom->getConvex_mesh();
            daeElementRef otherElemRef = convexRef->getConvex_hull_of().getElement();
            if ( !!otherElemRef ) {
                domGeometryRef linkedGeom = daeSafeCast<domGeometry>(otherElemRef);
                if( !linkedGeom ) {
                    return domMaterialRef();
                }
                return _ExtractFirstMaterial(linkedGeom,mapmaterials);
            }
        }

        return domMaterialRef();
    }

    /// Extract the Geometry and adds it to OpenRave
    /// \param  domgeom    Geometry to extract of the COLLADA's model
    /// \param  mapmaterials    Materials applied to the geometry
    /// \param  listGeometryInfos the geometry infos to output
    bool ExtractGeometry(const domGeometryRef domgeom, const map<string,domMaterialRef>& mapmaterials, std::list<KinBody::Link::GeometryInfo>& listGeometryInfos)
    {
        if( !domgeom ) {
            return false;
        }

        Transform tlocalgeom;
        // check for OpenRAVE profile simple geometric primitives
        for(size_t ie = 0; ie < domgeom->getExtra_array().getCount(); ++ie) {
            domExtraRef pextra = domgeom->getExtra_array()[ie];
            string extra_type = pextra->getType();
            if( extra_type == "geometry_info" ) {
                daeElementRef ptec = _ExtractOpenRAVEProfile(pextra);
                if( !!ptec ) {
                    bool bfoundgeom = false;
                    tlocalgeom = _ExtractFullTransformFromChildren(ptec);
                    KinBody::Link::GeometryInfo geominfo;
                    daeTArray<daeElementRef> children;
                    ptec->getChildren(children);
                    for(size_t i = 0; i < children.getCount(); ++i) {
                        std::string name = children[i]->getElementName();
                        if( name == "box" ) {
                            daeElementRef phalf_extents = children[i]->getChild("half_extents");
                            if( !!phalf_extents ) {
                                stringstream ss(phalf_extents->getCharData());
                                Vector vextents;
                                ss >> vextents.x >> vextents.y >> vextents.z;
                                if( ss.eof() || !!ss ) {
                                    geominfo._type = KinBody::Link::GeomBox;
                                    geominfo._vGeomData = vextents;
                                    geominfo._t = tlocalgeom;
                                    bfoundgeom = true;
                                }
                            }
                        }
                        else if( name == "sphere" ) {
                            daeElementRef pradius = children[i]->getChild("radius");
                            if( !!pradius ) {
                                dReal fradius = 0;
                                stringstream ss(pradius->getCharData());
                                ss >> fradius;
                                if( ss.eof() || !!ss ) {
                                    geominfo._type = KinBody::Link::GeomSphere;
                                    geominfo._vGeomData.x = fradius;
                                    geominfo._t = tlocalgeom;
                                    bfoundgeom = true;
                                }
                            }
                        }
                        else if( name == "cylinder" ) {
                            daeElementRef pradius = children[i]->getChild("radius");
                            daeElementRef pheight = children[i]->getChild("height");
                            if( !!pradius && !!pheight ) {
                                Vector vGeomData;
                                stringstream ss(pradius->getCharData());
                                ss >> vGeomData.x;
                                stringstream ss2(pheight->getCharData());
                                ss2 >> vGeomData.y;
                                if( (ss.eof() || !!ss) && (ss2.eof() || !!ss2) ) {
                                    Transform trot(quatRotateDirection(Vector(0,0,1),Vector(0,1,0)),Vector());
                                    tlocalgeom = tlocalgeom * trot;
                                    geominfo._type = KinBody::Link::GeomCylinder;
                                    geominfo._vGeomData = vGeomData;
                                    geominfo._t = tlocalgeom;
                                    bfoundgeom = true;
                                }
                            }
                        }
                        else if( name == "capsule" ) {
                            RAVELOG_WARN("capsule geometries are not supported");
                        }
                        else if( name == "plane" ) {
                            RAVELOG_WARN("plane geometries are not supported");
                        }
                        else if( name == "visible" ) {
                            resolveCommon_bool_or_param(children[i],domgeom,geominfo._bVisible);
                        }
                    }
                    if( bfoundgeom ) {
                        FillGeometryColor(_ExtractFirstMaterial(domgeom,mapmaterials),geominfo);
                        listGeometryInfos.push_back(geominfo);
                        return true;
                    }
                }
            }
        }

        Transform tlocalgeominv = tlocalgeom.inverse();
        if (!!domgeom->getMesh()) {
            const domMeshRef meshRef = domgeom->getMesh();
            for (size_t tg = 0; tg<meshRef->getTriangles_array().getCount(); tg++) {
                listGeometryInfos.push_back(KinBody::Link::GeometryInfo());
                _ExtractGeometry(meshRef->getTriangles_array()[tg], meshRef->getVertices(), mapmaterials, listGeometryInfos.back(),tlocalgeominv);
                listGeometryInfos.back()._t = tlocalgeom;
            }
            for (size_t tg = 0; tg<meshRef->getTrifans_array().getCount(); tg++) {
                listGeometryInfos.push_back(KinBody::Link::GeometryInfo());
                _ExtractGeometry(meshRef->getTrifans_array()[tg], meshRef->getVertices(), mapmaterials, listGeometryInfos.back(),tlocalgeominv);
                listGeometryInfos.back()._t = tlocalgeom;
            }
            for (size_t tg = 0; tg<meshRef->getTristrips_array().getCount(); tg++) {
                listGeometryInfos.push_back(KinBody::Link::GeometryInfo());
                _ExtractGeometry(meshRef->getTristrips_array()[tg], meshRef->getVertices(), mapmaterials, listGeometryInfos.back(),tlocalgeominv);
                listGeometryInfos.back()._t = tlocalgeom;
            }
            for (size_t tg = 0; tg<meshRef->getPolylist_array().getCount(); tg++) {
                listGeometryInfos.push_back(KinBody::Link::GeometryInfo());
                _ExtractGeometry(meshRef->getPolylist_array()[tg], meshRef->getVertices(), mapmaterials, listGeometryInfos.back(),tlocalgeominv);
                listGeometryInfos.back()._t = tlocalgeom;
            }
            if( meshRef->getPolygons_array().getCount()> 0 ) {
                RAVELOG_WARN("openrave does not support collada polygons\n");
            }
            return true;
        }
        else if (!!domgeom->getConvex_mesh()) {
            vector<Vector> vconvexhull;
            const domConvex_meshRef convexRef = domgeom->getConvex_mesh();
            daeElementRef otherElemRef = convexRef->getConvex_hull_of().getElement();
            if ( !!otherElemRef ) {
                domGeometryRef linkedGeom = daeSafeCast<domGeometry>(otherElemRef);
                if( !linkedGeom ) {
                    RAVELOG_WARN("convex_hull_of invalid geometry id\n");
                    return false;
                }

                std::list<KinBody::Link::GeometryInfo> listNewGeometryInfos;
                ExtractGeometry(linkedGeom, mapmaterials, listNewGeometryInfos);
                // need to get the convex hull of listNewGeometryInfos, quickest way is to use Geometry to compute the geometry vertices
                FOREACH(itgeominfo,listNewGeometryInfos) {
                    itgeominfo->InitCollisionMesh();
                    Transform tnew = tlocalgeominv * itgeominfo->_t;
                    FOREACH(itvertex, itgeominfo->_meshcollision.vertices) {
                        vconvexhull.push_back(tnew * *itvertex);
                    }
                }
            }
            else {
                // no getConvex_hull_of but direct vertices
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
                            vconvexhull.reserve(vconvexhull.size()+size_t(flArray->getCount()));
                            for (size_t k=0; k+2<flArray->getCount(); k+=3) {
                                domFloat fl0 = listFloats.get(k);
                                domFloat fl1 = listFloats.get(k+1);
                                domFloat fl2 = listFloats.get(k+2);
                                vconvexhull.push_back(tlocalgeominv * Vector(fl0*fUnitScale,fl1*fUnitScale,fl2*fUnitScale));
                            }
                        }
                    }
                }
            }

            if( vconvexhull.size()> 0 ) {
                listGeometryInfos.push_back(KinBody::Link::GeometryInfo());
                listGeometryInfos.back()._type = KinBody::Link::GeomTrimesh;
                listGeometryInfos.back()._t = tlocalgeom;
                _computeConvexHull(vconvexhull,listGeometryInfos.back()._meshcollision);
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
                    RobotBase::ManipulatorInfo manipinfo;
                    manipinfo._name = _ConvertToOpenRAVEName(name);
                    daeElementRef pframe_origin = tec->getChild("frame_origin");
                    daeElementRef pframe_tip = tec->getChild("frame_tip");
                    if( !!pframe_origin ) {
                        domLinkRef pdomlink = daeSafeCast<domLink>(daeSidRef(pframe_origin->getAttribute("link"), as).resolve().elt);
                        if( !!pdomlink ) {
                            manipinfo._sBaseLinkName = _ExtractLinkName(pdomlink);
                        }
                        if( !probot->GetLink(manipinfo._sBaseLinkName) ) {
                            RAVELOG_WARN(str(boost::format("failed to find manipulator %s frame origin %s\n")%name%pframe_origin->getAttribute("link")));
                            continue;
                        }
                    }
                    if( !!pframe_tip ) {
                        domLinkRef pdomlink = daeSafeCast<domLink>(daeSidRef(pframe_tip->getAttribute("link"), as).resolve().elt);
                        if( !!pdomlink ) {
                            manipinfo._sEffectorLinkName = _ExtractLinkName(pdomlink);
                        }
                        if( !probot->GetLink(manipinfo._sEffectorLinkName) ) {
                            RAVELOG_WARN(str(boost::format("failed to find manipulator %s frame tip %s\n")%name%pframe_tip->getAttribute("link")));
                            continue;
                        }
                        manipinfo._tLocalTool = _ExtractFullTransformFromChildren(pframe_tip);
                        daeElementRef pdirection = pframe_tip->getChild("direction");
                        if( !!pdirection ) {
                            stringstream ss(pdirection->getCharData());
                            ss >> manipinfo._vdirection.x >> manipinfo._vdirection.y >> manipinfo._vdirection.z;
                            if( !ss ) {
                                RAVELOG_WARN(str(boost::format("could not read frame_tip/direction of manipulator %s frame tip %s")%name%pframe_tip->getAttribute("link")));
                            }
                        }
                    }

                    for(size_t ic = 0; ic < tec->getContents().getCount(); ++ic) {
                        daeElementRef pmanipchild = tec->getContents()[ic];
                        if( pmanipchild->getElementName() == string("gripper_joint") ) {
                            std::pair<KinBody::JointPtr, domJointRef> result = _getJointFromRef(pmanipchild->getAttribute("joint").c_str(),as,probot);
                            KinBody::JointPtr pjoint = result.first;
                            domJointRef pdomjoint = result.second;
                            if( !!pjoint && !!pdomjoint ) {
                                manipinfo._vGripperJointNames.push_back(pjoint->GetName());
                                daeTArray<daeElementRef> children;
                                pmanipchild->getChildren(children);
                                for (size_t i = 0; i < children.getCount(); i++) {
                                    if( children[i]->getElementName() == string("closing_direction") ) {
                                        domAxis_constraintRef paxis = daeSafeCast<domAxis_constraint>(daeSidRef(children[i]->getAttribute("axis"), pdomjoint).resolve().elt);
                                        domFloat closing_direction = 0;
                                        if( !paxis ) {
                                            RAVELOG_WARN(str(boost::format("cannot resolve joint %s axis %s")%pmanipchild->getAttribute("joint")%children[i]->getAttribute("axis")));
                                        }
                                        else {
                                            if( !resolveCommon_float_or_param(children[i],as,closing_direction) ) {
                                                RAVELOG_WARN(str(boost::format("gripper joint %s axis %s cannot extract closing_direction\n")%children[i]->getAttribute("axis")%pmanipchild->getAttribute("joint")));
                                            }
                                        }
                                        manipinfo._vClosingDirection.push_back((dReal)closing_direction);
                                    }
                                }
                                continue;
                            }
                            RAVELOG_WARN(str(boost::format("could not find manipulator '%s' gripper joint '%s'\n")%manipinfo._name%pmanipchild->getAttribute("joint")));
                        }
                        else if( pmanipchild->getElementName() == string("iksolver") ) {
                            InterfaceTypePtr pinterfacetype = _ExtractInterfaceType(tec->getContents()[ic]);
                            if( !!pinterfacetype ) {
                                if( pinterfacetype->type.size() == 0 || pinterfacetype->type == "iksolver" ) {
                                    manipinfo._sIkSolverXMLId = pinterfacetype->name;
                                }
                                else {
                                    RAVELOG_WARN("invalid interface_type\n");
                                }
                            }
                        }
                        else if((pmanipchild->getElementName() != string("frame_origin"))&&(pmanipchild->getElementName() != string("frame_tip"))) {
                            RAVELOG_WARN(str(boost::format("unrecognized tag <%s> in manipulator '%s'")%pmanipchild->getElementName()%manipinfo._name));
                        }
                    }

                    probot->GetManipulators().push_back(RobotBase::ManipulatorPtr(new RobotBase::Manipulator(probot,manipinfo)));
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
                    daeElementRef pframe_origin = tec->getChild("frame_origin");
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

    /// \brief extract the robot manipulators
    void ExtractRobotAttachedActuators(RobotBasePtr probot, const domArticulated_systemRef as)
    {
        list<KinBody::JointPtr> listOrderedJoints;
        for(size_t ie = 0; ie < as->getExtra_array().getCount(); ++ie) {
            domExtraRef pextra = as->getExtra_array()[ie];
            if( strcmp(pextra->getType(), "attach_actuator") == 0 ) {
                string name = pextra->getAttribute("name");
                domTechniqueRef tec = _ExtractOpenRAVEProfile(pextra->getTechnique_array());
                if( !!tec ) {
                    for(size_t ic = 0; ic < tec->getContents().getCount(); ++ic) {
                        daeElementRef pchild = tec->getContents()[ic];
                        if( pchild->getElementName() == string("bind_actuator") ) {
                            std::pair<KinBody::JointPtr, domJointRef> result = _getJointFromRef(pchild->getAttribute("joint").c_str(),as,probot);
                            KinBody::JointPtr pjoint = result.first;
                            domJointRef pdomjoint = result.second;
                            if( !!pjoint && !!pdomjoint ) {
                                listOrderedJoints.push_back(pjoint);
                            }
                            else {
                                RAVELOG_WARN(str(boost::format("failed to find joint %s in actuator %s\n")%pchild->getAttribute("joint")%name));
                            }
                        }
                    }
                }
                else {
                    RAVELOG_WARN(str(boost::format("cannot create robot %s attached actuator %s\n")%probot->GetName()%name));
                }
            }
        }

        // for all robot joints after _setInitialJoints, first put listJoints in that order
        vector<KinBody::JointPtr> vjoints, vlastjoints;
        vjoints.reserve(probot->_vecjoints.size());
        if( probot->_vecjoints.size() > listOrderedJoints.size() ) {
            vlastjoints.reserve(probot->_vecjoints.size()-listOrderedJoints.size());
        }
        FOREACH(itjoint,probot->_vecjoints) {
            if( _setInitialJoints.find(*itjoint) == _setInitialJoints.end()) {
                if( find(listOrderedJoints.begin(),listOrderedJoints.end(),*itjoint) == listOrderedJoints.end() ) {
                    vlastjoints.push_back(*itjoint);
                }
            }
            else {
                vjoints.push_back(*itjoint);
            }
        }
        probot->_vecjoints = vjoints;
        probot->_vecjoints.insert(probot->_vecjoints.end(),listOrderedJoints.begin(),listOrderedJoints.end());
        probot->_vecjoints.insert(probot->_vecjoints.end(),vlastjoints.begin(),vlastjoints.end());
        // have to reset the joint indices and dof indices
        int jointindex=0;
        int dofindex=0;
        FOREACH(itjoint,probot->_vecjoints) {
            (*itjoint)->jointindex = jointindex++;
            (*itjoint)->dofindex = dofindex;
            dofindex += (*itjoint)->GetDOF();
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
        daeElementRef domsensor = daeURI(*instance_sensor,instance_url).getElement();
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
        if( _ProcessXMLReader(pcurreader,domsensor) ) {
            if( !!pcurreader->GetReadable() ) {
                psensor->SetReadableInterface(psensor->GetXMLId(),pcurreader->GetReadable());
            }
        }
        return true;
    }

    /// \brief feed the collada data into the base readers xml class
    ///
    /// \param preader the reader returned from RaveCallXMLReader
    /// \param elt the parent element (usually <extra>)
    static bool _ProcessXMLReader(BaseXMLReaderPtr preader, daeElementRef elt)
    {
        daeTArray<daeElementRef> children;
        elt->getChildren(children);
        AttributesList atts;
        for (size_t i = 0; i < children.getCount(); i++) {
            string xmltag = utils::ConvertToLowerCase(children[i]->getElementName());
            daeTArray<daeElement::attr> domatts;
            children[i]->getAttributes(domatts);
            atts.clear();
            for(size_t j = 0; j < domatts.getCount(); ++j) {
                atts.push_back(make_pair(domatts[j].name,domatts[j].value));
            }
            BaseXMLReader::ProcessElement action = preader->startElement(xmltag,atts);
            if( action  == BaseXMLReader::PE_Support ) {
                _ProcessXMLReader(preader,children[i]);
                preader->characters(children[i]->getCharData());
                if( preader->endElement(xmltag) ) {
                    return true;
                }
            }
            else {
                RAVELOG_WARN(str(boost::format("unprocessed tag: %s/%s")%elt->getElementName()%xmltag));
            }
        }
        return false;
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

    static daeElement* searchBindingFromSIDREF(domSidref value, daeElementRef parent)
    {
        return daeSidRef(value,parent).resolve().elt;
    }

    /// Search a given parameter reference and stores the new reference to search.
    /// \param ref the reference name to search
    /// \param parent The array of parameter where the method searchs.
    static daeElement* searchBinding(daeString ref, daeElementRef parent, bool bLogWarning=true)
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
                    RAVELOG_WARN(str(boost::format("newparam sid=%s does not have SIDREF\n")%getSid(newparam)));
                }
            }
        }
        if( bLogWarning ) {
            RAVELOG_WARN(str(boost::format("failed to get binding '%s' for element: %s\n")%ref%parent->getElementName()));
        }
        return NULL;
    }

    static daeElement* searchBindingArray(daeString ref, const domInstance_articulated_system_Array& paramArray)
    {
        for(size_t iikm = 0; iikm < paramArray.getCount(); ++iikm) {
            daeElement* pelt = searchBinding(ref,paramArray[iikm].cast(),false);
            if( !!pelt ) {
                return pelt;
            }
        }
        RAVELOG_WARN(str(boost::format("failed to get binding '%s'")%ref));
        return NULL;
    }

    static daeElement* searchBindingArray(daeString ref, const domInstance_kinematics_model_Array& paramArray)
    {
        for(size_t iikm = 0; iikm < paramArray.getCount(); ++iikm) {
            daeElement* pelt = searchBinding(ref,paramArray[iikm].cast(),false);
            if( !!pelt ) {
                return pelt;
            }
        }
        RAVELOG_WARN(str(boost::format("failed to get binding '%s'")%ref));
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

    static bool resolveCommon_float_or_param(daeElementRef pcommon, daeElementRef parent, domFloat& f)
    {
        if( !pcommon ) {
            return false;
        }
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

    static bool resolveCommon_bool_or_param(daeElementRef pcommon, daeElementRef parent, bool& bvalue)
    {
        if( !pcommon ) {
            return false;
        }
        daeElement* pbool = pcommon->getChild("bool");
        if( !!pbool ) {
            if( pbool->getCharData() == "true" ) {
                bvalue = true;
                return true;
            }
            else if( pbool->getCharData() == "false" ) {
                bvalue = false;
                return true;
            }
            RAVELOG_WARN(str(boost::format("invalid bool data in element %s: %s")%pcommon->getElementName()%pbool->getCharData()));
            return false;
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

    /// \brief if inode points to a valid node, inserts it in the scene and removes the instance_node
    static domNodeRef _InstantiateNode(daeElementRef pelt)
    {
        domInstance_nodeRef inode = daeSafeCast<domInstance_node>(pelt);
        if( !inode ) {
            return domNodeRef();
        }
        domNodeRef node = daeSafeCast<domNode> (inode->getUrl().getElement().cast());
        if( !node ) {
            RAVELOG_WARN(str(boost::format("failed to resolve node %s\n")%inode->getUrl().str()));
            return domNodeRef();
        }
        // extra elements can contain the suffix of all the ids
        std::string idsuffix;
        for(size_t ie = 0; ie < inode->getExtra_array().getCount(); ++ie) {
            domExtraRef pextra = inode->getExtra_array()[ie];
            std::string extra_type = pextra->getType();
            if( extra_type == "idsuffix" ) {
                if( !!pextra->getName() ) {
                    idsuffix = pextra->getName();
                }
            }
        }

        // have to clone since the parents are different
        daeElementRef parentelt = pelt->getParent();
        daeElementRef newnode = daeSafeCast<domNode>(node->clone(idsuffix.size() > 0 ? idsuffix.c_str() : NULL));
        if( !!node->getDocumentURI() ) {
            if( !parentelt->getDocumentURI() || !(*node->getDocumentURI() == *parentelt->getDocumentURI()) ) {
                _ResolveURLs(newnode,*node->getDocumentURI());
            }
        }
        parentelt->add(newnode);
        BOOST_ASSERT(parentelt == newnode->getParentElement());
        parentelt->removeChildElement(pelt); // have to remove the instance_node
        return daeSafeCast<domNode>(newnode);
    }

    static void _ResolveURLs(daeElementRef elt, const daeURI& srcuri)
    {
        // resolve all xsAnyURIs
        for(size_t iattr = 0; iattr < elt->getAttributeCount(); ++iattr) {
            daeMetaAttribute* pattr = elt->getAttributeObject(iattr);
            if( !!pattr ) {
                daeAtomicType* ptype = pattr->getType();
                if( !!ptype ) {
                    //xsAnyURI
                    if( ptype->getTypeEnum() == daeAtomicType::ResolverType ) {
                        std::ostringstream buffer; buffer << std::setprecision(std::numeric_limits<dReal>::digits10+1);
                        pattr->memoryToString(elt,buffer);
                        daeURI newuri(srcuri,buffer.str());
                        pattr->stringToMemory(elt, newuri.str().c_str());
                    }
                }
            }
        }

        daeTArray<daeElementRef> children;
        elt->getChildren(children);
        for(size_t i = 0; i < children.getCount(); ++i) {
            _ResolveURLs(children[i],srcuri);
        }
    }

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
            daeElement* pnodeelt = daeSidRef(kbindmodel->getNode(), viscene->getUrl().getElement()).resolve().elt;
            domNodeRef node = daeSafeCast<domNode>(pnodeelt);
            if (!node) {
                node = _InstantiateNode(pnodeelt);
                if( !node ) {
                    RAVELOG_WARN(str(boost::format("bind_kinematics_model does not reference valid node %s\n")%kbindmodel->getNode()));
                    continue;
                }
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

            domFloat jointvalue=0;
            if( !!bindjoint->getValue() ) {
                if (!!bindjoint->getValue()->getParam()) {
                    pelt = searchBinding(bindjoint->getValue()->getParam()->getValue(),kscene);
                }
                else {
                    pelt = bindjoint->getValue();
                }
            }

            resolveCommon_float_or_param(pelt,kscene,jointvalue);
            bindings.listAxisBindings.push_back(JointAxisBinding(pjtarget, pjointaxis, jointvalue, NULL, NULL));
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
                std::list<ModelBinding>::iterator itmodelbindings = _FindParentModel(nodephysicsoffset,bindings.listModelBindings);
                if( itmodelbindings == bindings.listModelBindings.end() ) {
                    itmodelbindings = _FindChildModel(nodephysicsoffset,bindings.listModelBindings);
                }
                if( itmodelbindings == bindings.listModelBindings.end() ) {
                    RAVELOG_WARN(str(boost::format("instance_physics_model %s did not find visual binding to %s")%ipmodel->getSid()%ipmodel->getParent().getOriginalURI()));
                }
                else {
                    itmodelbindings->_ipmodel = ipmodel;
                }
                for(size_t ibody = 0; ibody < ipmodel->getInstance_rigid_body_array().getCount(); ++ibody) {
                    LinkBinding lb;
                    lb._ipmodel = ipmodel;
                    lb._irigidbody = ipmodel->getInstance_rigid_body_array()[ibody];
                    // don't resolve the node here since it could be pointing to a node inside <instance_node>
                    lb._nodeurifromphysics.reset(new daeURI(lb._irigidbody->getTarget()));
                    //lb._node = daeSafeCast<domNode>(lb._irigidbody->getTarget().getElement().cast());
                    lb._rigidbody = daeSafeCast<domRigid_body>(daeSidRef(lb._irigidbody->getBody(),pmodel).resolve().elt);
                    lb._nodephysicsoffset = nodephysicsoffset;
                    if( !!lb._rigidbody ) { // && !!lb._node ) {
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

    /// \brief extract kinematics/geometry independent parameters from instance_articulated_system extra fields
    void _ExtractExtraData(KinBodyPtr pbody, const domExtra_Array& arr) {
        AttributesList atts;
        for(size_t i = 0; i < arr.getCount(); ++i) {
            _ExtractAttributesList(arr[i],atts);
            string extratype = arr[i]->getType();
            BaseXMLReaderPtr preader = RaveCallXMLReader(pbody->IsRobot() ? PT_Robot : PT_KinBody, extratype, pbody,atts);
            if( !!preader ) {
                if( _ProcessXMLReader(preader,arr[i]) ) {
                    if( !!preader->GetReadable() ) {
                        pbody->SetReadableInterface(extratype,preader->GetReadable());
                    }
                }
            }
        }
    }

    void _ExtractAttributesList(daeElementRef elt, AttributesList& atts)
    {
        atts.clear();
        size_t num = elt->getAttributeCount();
        std::ostringstream buffer; buffer << std::setprecision(std::numeric_limits<dReal>::digits10+1);
        for(size_t i = 0; i < num; ++i) {
            daeMetaAttribute* pmeta = elt->getAttributeObject(i);
            buffer.str("");  buffer.clear();
            pmeta->memoryToString(elt, buffer);
            atts.push_back(make_pair(string(pmeta->getName()), buffer.str()));
        }
    }

    /// \brief returns an openrave interface type from the extra array
    InterfaceTypePtr _ExtractInterfaceType(const daeElementRef pelt) {
        daeTArray<daeElementRef> children;
        pelt->getChildren(children);
        for(size_t i = 0; i < children.getCount(); ++i) {
            if( children[i]->getElementName() == string("interface_type") ) {
                daeElementRef ptec = _ExtractOpenRAVEProfile(children[i]);
                if( !!ptec ) {
                    daeElementRef ptype = ptec->getChild("interface");
                    if( !!ptype ) {
                        return InterfaceTypePtr(new InterfaceType(ptype->getAttribute("type"), ptype->getCharData()));
                    }
                }
            }
        }
        return InterfaceTypePtr();
    }

    /// \brief returns an openrave interface type from the extra array
    InterfaceTypePtr _ExtractInterfaceType(const domExtra_Array& arr) {
        for(size_t i = 0; i < arr.getCount(); ++i) {
            if( strcmp(arr[i]->getType(),"interface_type") == 0 ) {
                domTechniqueRef tec = _ExtractOpenRAVEProfile(arr[i]->getTechnique_array());
                if( !!tec ) {
                    daeElementRef ptype = tec->getChild("interface");
                    if( !!ptype ) {
                        return InterfaceTypePtr(new InterfaceType(ptype->getAttribute("type"), ptype->getCharData()));
                    }
                }
            }
        }
        return InterfaceTypePtr();
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

    /// \brief searches through the node's parents until one matches the node stored in listModelBindings
    static std::list<ModelBinding>::iterator _FindParentModel(domNodeRef pnode, std::list<ModelBinding>& listModelBindings)
    {
        if( !pnode ) {
            return listModelBindings.end();
        }
        while(!!pnode) {
            FOREACH(itmodel,listModelBindings) {
                if( _CompareElementURI(pnode,itmodel->_node) > 0 ) {
                    return itmodel;
                }
            }
            pnode = daeSafeCast<domNode>(pnode->getParentElement());
        }
        return listModelBindings.end();
    }

    /// \brief searches through the node's children until one matches the node stored in listModelBindings
    static std::list<ModelBinding>::iterator _FindChildModel(domNodeRef pnode, std::list<ModelBinding>& listModelBindings)
    {
        if( !pnode ) {
            return listModelBindings.end();
        }
        FOREACH(itmodel,listModelBindings) {
            domNodeRef pelt = itmodel->_node;
            while(!!pelt) {
                if( _CompareElementURI(pnode,pelt) > 0 ) {
                    return itmodel;
                }
                pelt = daeSafeCast<domNode>(pelt->getParentElement());
            }
        }
        return listModelBindings.end();
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
                            daeElementRef pelt = daeURI(*children[0],formulaurl).getElement();
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

    // -1 don't know
    // 0 no
    // 1 same uri
    template <typename T>
    static int _CompareElementURI(T elt1, T elt2) {
        if( !elt1 || !elt2 ) {
            return -1;
        }
        if( elt1->typeID() != elt2->typeID() ) {
            return 0;
        }
        if( !elt1->getDocumentURI() || !elt2->getDocumentURI() ) {
            if( !elt1->getDocumentURI() && !elt2->getDocumentURI() && elt1 == elt2 ) {
                return 1;
            }
            return -1;
        }
        if( !elt1->getId() || !elt2->getId() ) {
            if( !elt1->getId() && !elt2->getId() && elt1 == elt2 ) {
                return 1;
            }
            return -1;
        }
        if( string(elt1->getDocumentURI()->getURI()) != elt2->getDocumentURI()->getURI() ) {
            return 0;
        }
        return string(elt1->getId()) == elt2->getId();
    }

    template <typename T>
    static int _CompareElementSid(T elt1, T elt2) {
        if( !elt1 || !elt2 ) {
            return -1;
        }
        if( elt1->typeID() != elt2->typeID() ) {
            return 0;
        }
        if( !elt1->getDocumentURI() || !elt2->getDocumentURI() ) {
            if( !elt1->getDocumentURI() && !elt2->getDocumentURI() && elt1 == elt2 ) {
                return 1;
            }
            return -1;
        }
        if( !elt1->getSid() || !elt2->getSid() ) {
            if( !elt1->getSid() && !elt2->getSid() && elt1 == elt2 ) {
                return 1;
            }
            return -1;
        }
        if( string(elt1->getDocumentURI()->getURI()) != elt2->getDocumentURI()->getURI() ) {
            return 0;
        }
        return string(elt1->getSid()) == elt2->getSid();
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
        std::string newname = utils::ConvertToOpenRAVEName(name);
        if( name != newname ) {
            RAVELOG_WARN(boost::str(boost::format("name '%s' is not a valid OpenRAVE name, converting to '%s'")%name%newname));
        }
        return newname;
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

    /// \brief do the inverse resolve file:/... -> openrave:/...
    ///
    /// if none found, returns the original uri
    daeURI _ResolveInverse(const daeURI& uri)
    {
        std::map<std::string,daeURI>::iterator itindex = _mapInverseResolvedURIList.find(uri.str());
        if( itindex != _mapInverseResolvedURIList.end() ) {
            // try to resolve again
            return _ResolveInverse(itindex->second);
        }
        return uri;
    }

    /// \brief returns a string with the full URI
    std::string _MakeFullURI(const xsAnyURI& uri, daeElementRef pelt) {
        daeURI* docuri = pelt->getDocumentURI();
        if( !docuri ) {
            RAVELOG_WARN(str(boost::format("failed to get the URI of the document, so cannot resolve %s")%uri.str()));
            return uri.str();
        }
        else {
            // don't use uri.str() since it resolves to temporary dae files if coming from a zae
            daeURI newdocuri = _ResolveInverse(*docuri);
            daeURI newuri(newdocuri,uri.getOriginalURI());
            return newuri.str();
        }
    }

    std::string _MakeFullURIFromId(const std::string& id, daeElementRef pelt) {
        daeURI* docuri = pelt->getDocumentURI();
        if( !docuri ) {
            RAVELOG_WARN(str(boost::format("failed to get the URI of the document, so cannot resolve id %s")%id));
            return string("#")+id;
        }
        daeURI newdocuri = _ResolveInverse(*docuri);
        daeURI newuri(newdocuri,string("#")+id);
        return newuri.str();
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
    bool _bOpeningZAE; ///< true if currently opening a zae
    bool _bSkipGeometry;
    std::set<KinBody::LinkPtr> _setInitialLinks;
    std::set<KinBody::JointPtr> _setInitialJoints;
    std::set<RobotBase::ManipulatorPtr> _setInitialManipulators;
    std::set<RobotBase::AttachedSensorPtr> _setInitialSensors;
    std::vector<std::string> _vOpenRAVESchemeAliases;
    std::map<std::string,daeURI> _mapInverseResolvedURIList; ///< holds a list of inverse resolved relationships file:// -> openrave://
};

bool RaveParseColladaURI(EnvironmentBasePtr penv, const std::string& uri,const AttributesList& atts)
{
    ColladaReader reader(penv);
    if( !reader.InitFromURI(uri,atts) ) {
        return false;
    }
    return reader.Extract();
}

bool RaveParseColladaFile(EnvironmentBasePtr penv, const string& filename,const AttributesList& atts)
{
    ColladaReader reader(penv);
    string filedata = RaveFindLocalFile(filename);
    if (filedata.size() == 0 || !reader.InitFromFile(filedata,atts)) {
        return false;
    }
    return reader.Extract();
}

bool RaveParseColladaFile(EnvironmentBasePtr penv, KinBodyPtr& pbody, const string& filename,const AttributesList& atts)
{
    ColladaReader reader(penv);
    string filedata = RaveFindLocalFile(filename);
    if (filedata.size() == 0 || !reader.InitFromFile(filedata,atts)) {
        return false;
    }
    return reader.Extract(pbody);
}

bool RaveParseColladaFile(EnvironmentBasePtr penv, RobotBasePtr& probot, const string& filename,const AttributesList& atts)
{
    ColladaReader reader(penv);
    string filedata = RaveFindLocalFile(filename);
    if (filedata.size() == 0 || !reader.InitFromFile(filedata,atts)) {
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
