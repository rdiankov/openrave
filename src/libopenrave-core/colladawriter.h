// Copyright (C) 2006-2010 Rosen Diankov (rdiankov@cs.cmu.edu)
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

#include <locale>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/time_facet.hpp>

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
        list<pair<int,string> > listusedlinks;
        list<pair<int,string> > listprocesseddofs;
        daeElementRef plink;
        domNodeRef pnode;
    };

    struct kinematics_model_output
    {
        struct axis_output
        {
            //axis_output(const string& sid, KinBody::JointConstPtr pjoint, int iaxis) : sid(sid), pjoint(pjoint), iaxis(iaxis) {}
        axis_output() : iaxis(0) {}
            string sid, nodesid;
            KinBody::JointConstPtr pjoint;
            int iaxis;
            string jointnodesid;
        };
        domKinematics_modelRef kmodel;
        std::vector<axis_output> vaxissids;
        std::vector<std::string > vlinksids;
    };

    struct axis_sids
    {
    axis_sids(const string& axissid, const string& valuesid, const string& jointnodesid) : axissid(axissid), valuesid(valuesid), jointnodesid(jointnodesid) {}
        string axissid, valuesid, jointnodesid;
    };

    struct instance_kinematics_model_output
    {
        domInstance_kinematics_modelRef ikm;
        std::vector<axis_sids> vaxissids;
        boost::shared_ptr<kinematics_model_output> kmout;
        std::vector<std::pair<std::string,std::string> > vkinematicsbindings;
    };

    struct instance_articulated_system_output
    {
        domInstance_articulated_systemRef ias;
        std::vector<axis_sids> vaxissids;
        std::vector<std::string > vlinksids;
        std::vector<std::pair<std::string,std::string> > vkinematicsbindings;
    };

 ColladaWriter(EnvironmentBaseConstPtr penv) : _dom(NULL), _penv(penv)
    {
        daeErrorHandler::setErrorHandler(this);

        RAVELOG_VERBOSE("init COLLADA writer version: %s, namespace: %s\n", COLLADA_VERSION, COLLADA_NAMESPACE);
        _collada.reset(new DAE);
        _collada->setIOPlugin( NULL );
        _collada->setDatabase( NULL );

        const char* documentName = "openrave_snapshot";
        daeDocument *doc = NULL;
        daeInt error = _collada->getDatabase()->insertDocument(documentName, &doc ); // also creates a collada root
        BOOST_ASSERT( error == DAE_OK && !!doc );
        _dom = daeSafeCast<domCOLLADA>(doc->getDomRoot());
        _dom->setAttribute("xmlns:math","http://www.w3.org/1998/Math/MathML");

        //create the required asset tag
        domAssetRef asset = daeSafeCast<domAsset>( _dom->add( COLLADA_ELEMENT_ASSET ) );
        {
            // facet becomes owned by locale, so no need to explicitly delete
            boost::posix_time::time_facet* facet = new boost::posix_time::time_facet("%Y-%m-%dT%H:%M:%s");
            std::stringstream ss;
            ss.imbue(std::locale(ss.getloc(), facet));
            ss << boost::posix_time::second_clock::local_time();

            domAsset::domCreatedRef created = daeSafeCast<domAsset::domCreated>( asset->add( COLLADA_ELEMENT_CREATED ) );
            created->setValue(ss.str().c_str());
            domAsset::domModifiedRef modified = daeSafeCast<domAsset::domModified>( asset->add( COLLADA_ELEMENT_MODIFIED ) );
            modified->setValue(ss.str().c_str());

            domAsset::domContributorRef contrib = daeSafeCast<domAsset::domContributor>( asset->add( COLLADA_TYPE_CONTRIBUTOR ) );
            domAsset::domContributor::domAuthoring_toolRef authoringtool = daeSafeCast<domAsset::domContributor::domAuthoring_tool>( contrib->add( COLLADA_ELEMENT_AUTHORING_TOOL ) );
            authoringtool->setValue("OpenRAVE Collada Writer");

            domAsset::domUnitRef units = daeSafeCast<domAsset::domUnit>( asset->add( COLLADA_ELEMENT_UNIT ) );
            units->setMeter(1);
            units->setName("meter");

            domAsset::domUp_axisRef zup = daeSafeCast<domAsset::domUp_axis>( asset->add( COLLADA_ELEMENT_UP_AXIS ) );
            zup->setValue(UP_AXIS_Z_UP);
        }

        _globalscene = _dom->getScene();
        if( !_globalscene ) {
            _globalscene = daeSafeCast<domCOLLADA::domScene>( _dom->add( COLLADA_ELEMENT_SCENE ) );
        }

        _visualScenesLib = daeSafeCast<domLibrary_visual_scenes>(_dom->add(COLLADA_ELEMENT_LIBRARY_VISUAL_SCENES));
        _visualScenesLib->setId("vscenes");
        _geometriesLib = daeSafeCast<domLibrary_geometries>(_dom->add(COLLADA_ELEMENT_LIBRARY_GEOMETRIES));
        _geometriesLib->setId("geometries");
        _effectsLib = daeSafeCast<domLibrary_effects>(_dom->add(COLLADA_ELEMENT_LIBRARY_EFFECTS));
        _effectsLib->setId("effects");
        _materialsLib = daeSafeCast<domLibrary_materials>(_dom->add(COLLADA_ELEMENT_LIBRARY_MATERIALS));
        _materialsLib->setId("materials");
        _kinematicsModelsLib = daeSafeCast<domLibrary_kinematics_models>(_dom->add(COLLADA_ELEMENT_LIBRARY_KINEMATICS_MODELS));
        _kinematicsModelsLib->setId("kmodels");
        _articulatedSystemsLib = daeSafeCast<domLibrary_articulated_systems>(_dom->add(COLLADA_ELEMENT_LIBRARY_ARTICULATED_SYSTEMS));
        _articulatedSystemsLib->setId("asystems");
        _kinematicsScenesLib = daeSafeCast<domLibrary_kinematics_scenes>(_dom->add(COLLADA_ELEMENT_LIBRARY_KINEMATICS_SCENES));
        _kinematicsScenesLib->setId("kscenes");
        _physicsScenesLib = daeSafeCast<domLibrary_physics_scenes>(_dom->add(COLLADA_ELEMENT_LIBRARY_PHYSICS_SCENES));
        _physicsScenesLib->setId("pscenes");
        domExtraRef pextra_library_sensors = daeSafeCast<domExtra>(_dom->add(COLLADA_ELEMENT_EXTRA));
        pextra_library_sensors->setId("sensors");
        pextra_library_sensors->setType("library_sensors");
        _sensorsLib = daeSafeCast<domTechnique>(pextra_library_sensors->add(COLLADA_ELEMENT_TECHNIQUE));
        _sensorsLib->setProfile("OpenRAVE");
    }
    virtual ~ColladaWriter() {
        _collada.reset();
        DAE::cleanup();
    }
    
    /// \brief Write down environment
    virtual bool Write(EnvironmentBasePtr penv)
    {
        EnvironmentMutex::scoped_lock lockenv(penv->GetMutex());
        _CreateScene();
        domPhysics_scene::domTechnique_commonRef common = daeSafeCast<domPhysics_scene::domTechnique_common>(_scene.pscene->add(COLLADA_ELEMENT_TECHNIQUE_COMMON));

        //  Create gravity
        domTargetable_float3Ref g = daeSafeCast<domTargetable_float3>(common->add(COLLADA_ELEMENT_GRAVITY));
        Vector vgravity = penv->GetPhysicsEngine()->GetGravity();
        g->getValue().set3 (vgravity.x, vgravity.y, vgravity.z);

        vector<RobotBasePtr> vrobots;
        penv->GetRobots(vrobots);
        FOREACHC(it, vrobots) {
            boost::shared_ptr<instance_articulated_system_output> iasout = _WriteRobot(*it);
            if( !!iasout ) {
                _WriteBindingsInstance_kinematics_scene(_scene.kiscene,KinBodyConstPtr(*it),iasout->vaxissids,iasout->vkinematicsbindings);
            }
            else {
                RAVELOG_WARN(str(boost::format("failed to write robot %s\n")%(*it)->GetName()));
            }
        }

        vector<KinBodyPtr> vbodies;
        penv->GetBodies(vbodies);
        FOREACHC(itbody, vbodies) {
            if( !(*itbody)->IsRobot() ) {
                boost::shared_ptr<instance_kinematics_model_output> ikmout = _WriteInstance_kinematics_model(*itbody,_scene.kscene,_scene.kscene->getID());
                if( !!ikmout ) {
                    _WriteBindingsInstance_kinematics_scene(_scene.kiscene,*itbody,ikmout->vaxissids,ikmout->vkinematicsbindings);
                }
                else {
                    RAVELOG_WARN(str(boost::format("failed to write body %s\n")%(*itbody)->GetName()));
                }
            }
        }

        return true;
    }

    /// \brief Write one robot as a file
    virtual bool Write(RobotBasePtr probot)
    {
        EnvironmentMutex::scoped_lock lockenv(_penv->GetMutex());
        _CreateScene();
        boost::shared_ptr<instance_articulated_system_output> iasout = _WriteRobot(probot);
        if( !iasout ) {
            return false;
        }
        _WriteBindingsInstance_kinematics_scene(_scene.kiscene,KinBodyConstPtr(probot),iasout->vaxissids,iasout->vkinematicsbindings);
        return true;
    }

    /// \brief Write one kinematic body as a file
    virtual bool Write(KinBodyPtr pbody)
    {
        EnvironmentMutex::scoped_lock lockenv(_penv->GetMutex());
        _CreateScene();
        boost::shared_ptr<instance_kinematics_model_output> ikmout = _WriteInstance_kinematics_model(pbody,_scene.kscene,_scene.kscene->getID());
        if( !ikmout ) {
            return false;
        }
        _WriteBindingsInstance_kinematics_scene(_scene.kiscene,KinBodyConstPtr(pbody),ikmout->vaxissids,ikmout->vkinematicsbindings);
        return true;
    }

    /// \brief Write kinematic body in a given scene
    virtual boost::shared_ptr<instance_articulated_system_output> _WriteRobot(RobotBasePtr probot)
    {
        RAVELOG_VERBOSE(str(boost::format("writing robot as instance_articulated_system (%d) %s\n")%probot->GetEnvironmentId()%probot->GetName()));
        string asid = str(boost::format("robot%d")%probot->GetEnvironmentId());
        string askid = str(boost::format("%s_kinematics")%asid);
        string asmid = str(boost::format("%s_motion")%asid);
        string iassid = str(boost::format("%s_inst")%asmid);

        domInstance_articulated_systemRef ias = daeSafeCast<domInstance_articulated_system>(_scene.kscene->add(COLLADA_ELEMENT_INSTANCE_ARTICULATED_SYSTEM));
        ias->setSid(iassid.c_str());
        ias->setUrl((string("#")+asmid).c_str());
        ias->setName(probot->GetName().c_str());

        boost::shared_ptr<instance_articulated_system_output> iasout(new instance_articulated_system_output());
        iasout->ias = ias;

        // motion info
        domArticulated_systemRef articulated_system_motion = daeSafeCast<domArticulated_system>(_articulatedSystemsLib->add(COLLADA_ELEMENT_ARTICULATED_SYSTEM));
        articulated_system_motion->setId(asmid.c_str());
        domMotionRef motion = daeSafeCast<domMotion>(articulated_system_motion->add(COLLADA_ELEMENT_MOTION));
        domMotion_techniqueRef mt = daeSafeCast<domMotion_technique>(motion->add(COLLADA_ELEMENT_TECHNIQUE_COMMON));
        domInstance_articulated_systemRef ias_motion = daeSafeCast<domInstance_articulated_system>(motion->add(COLLADA_ELEMENT_INSTANCE_ARTICULATED_SYSTEM));
        ias_motion->setUrl(str(boost::format("#%s")%askid).c_str());

        // kinematics info
        domArticulated_systemRef articulated_system_kinematics = daeSafeCast<domArticulated_system>(_articulatedSystemsLib->add(COLLADA_ELEMENT_ARTICULATED_SYSTEM));
        articulated_system_kinematics->setId(askid.c_str());
        domKinematicsRef kinematics = daeSafeCast<domKinematics>(articulated_system_kinematics->add(COLLADA_ELEMENT_KINEMATICS));
        domKinematics_techniqueRef kt = daeSafeCast<domKinematics_technique>(kinematics->add(COLLADA_ELEMENT_TECHNIQUE_COMMON));

        vector<dReal> vlower, vupper;
        boost::shared_ptr<instance_kinematics_model_output> ikmout = _WriteInstance_kinematics_model(KinBodyPtr(probot),kinematics,askid);

        for(size_t idof = 0; idof < ikmout->vaxissids.size(); ++idof) {
            string axis_infosid = str(boost::format("axis_info_inst%d")%idof);
            KinBody::JointConstPtr pjoint = ikmout->kmout->vaxissids.at(idof).pjoint;
            int iaxis = ikmout->kmout->vaxissids.at(idof).iaxis;

            //  Kinematics axis info
            domKinematics_axis_infoRef kai = daeSafeCast<domKinematics_axis_info>(kt->add(COLLADA_ELEMENT_AXIS_INFO));
            kai->setAxis(str(boost::format("%s/%s")%ikmout->kmout->kmodel->getID()%ikmout->kmout->vaxissids.at(idof).sid).c_str());
            kai->setSid(axis_infosid.c_str());
            domCommon_bool_or_paramRef active = daeSafeCast<domCommon_bool_or_param>(kai->add(COLLADA_ELEMENT_ACTIVE));
            daeSafeCast<domCommon_bool_or_param::domBool>(active->add(COLLADA_ELEMENT_BOOL))->setValue(pjoint->GetDOFIndex()>=0);
            domCommon_bool_or_paramRef locked = daeSafeCast<domCommon_bool_or_param>(kai->add(COLLADA_ELEMENT_LOCKED));
            daeSafeCast<domCommon_bool_or_param::domBool>(locked->add(COLLADA_ELEMENT_BOOL))->setValue(false);
            if( !pjoint->IsCircular() ) {
                pjoint->GetLimits(vlower,vupper);
                dReal fmult = 1.0;
                if( pjoint->GetType() != KinBody::Joint::JointPrismatic ) {
                    fmult = 180.0/PI;
                }
                domKinematics_limitsRef plimits = daeSafeCast<domKinematics_limits>(kai->add(COLLADA_ELEMENT_LIMITS));
                daeSafeCast<domCommon_float_or_param::domFloat>(plimits->add(COLLADA_ELEMENT_MIN)->add(COLLADA_ELEMENT_FLOAT))->setValue(vlower.at(iaxis)*fmult);
                daeSafeCast<domCommon_float_or_param::domFloat>(plimits->add(COLLADA_ELEMENT_MAX)->add(COLLADA_ELEMENT_FLOAT))->setValue(vupper.at(iaxis)*fmult);
            }
        
            //  Motion axis info
            domMotion_axis_infoRef mai = daeSafeCast<domMotion_axis_info>(mt->add(COLLADA_ELEMENT_AXIS_INFO));
            mai->setAxis(str(boost::format("%s/%s")%askid%axis_infosid).c_str());
            pjoint->GetVelocityLimits(vlower,vupper);
            domCommon_float_or_paramRef speed = daeSafeCast<domCommon_float_or_param>(mai->add(COLLADA_ELEMENT_SPEED));
            daeSafeCast<domCommon_float_or_param::domFloat>(speed->add(COLLADA_ELEMENT_FLOAT))->setValue(pjoint->fMaxVel);

            domCommon_float_or_paramRef accel = daeSafeCast<domCommon_float_or_param>(mai->add(COLLADA_ELEMENT_ACCELERATION));
            daeSafeCast<domCommon_float_or_param::domFloat>(accel->add(COLLADA_ELEMENT_FLOAT))->setValue(pjoint->fMaxAccel);

        }

        // write the bindings
        string asmsym = str(boost::format("%s_%s")%asmid%ikmout->ikm->getSid());
        string assym = str(boost::format("%s_%s")%_scene.kscene->getID()%ikmout->ikm->getSid());
        FOREACH(it, ikmout->vkinematicsbindings) {
            domKinematics_newparamRef abm = daeSafeCast<domKinematics_newparam>(ias_motion->add(COLLADA_ELEMENT_NEWPARAM));
            abm->setSid(asmsym.c_str());
            daeSafeCast<domKinematics_newparam::domSIDREF>(abm->add(COLLADA_ELEMENT_SIDREF))->setValue(str(boost::format("%s/%s")%askid%it->first).c_str());
            domKinematics_bindRef ab = daeSafeCast<domKinematics_bind>(ias->add(COLLADA_ELEMENT_BIND));
            ab->setSymbol(assym.c_str());
            daeSafeCast<domKinematics_param>(ab->add(COLLADA_ELEMENT_PARAM))->setRef(str(boost::format("%s/%s")%asmid%asmsym).c_str());
            iasout->vkinematicsbindings.push_back(make_pair(string(ab->getSymbol()), it->second));
        }
        for(size_t idof = 0; idof < ikmout->vaxissids.size(); ++idof) {
            const axis_sids& kas = ikmout->vaxissids.at(idof);
            domKinematics_newparamRef abm = daeSafeCast<domKinematics_newparam>(ias_motion->add(COLLADA_ELEMENT_NEWPARAM));
            abm->setSid(str(boost::format("%s_%s")%asmid%kas.axissid).c_str());
            daeSafeCast<domKinematics_newparam::domSIDREF>(abm->add(COLLADA_ELEMENT_SIDREF))->setValue(str(boost::format("%s/%s")%askid%kas.axissid).c_str());
            domKinematics_bindRef ab = daeSafeCast<domKinematics_bind>(ias->add(COLLADA_ELEMENT_BIND));
            ab->setSymbol(str(boost::format("%s_%s")%assym%kas.axissid).c_str());
            daeSafeCast<domKinematics_param>(ab->add(COLLADA_ELEMENT_PARAM))->setRef(str(boost::format("%s/%s_%s")%asmid%asmid%kas.axissid).c_str());
            string valuesid;
            if( kas.valuesid.size() > 0 ) {
                domKinematics_newparamRef abmvalue = daeSafeCast<domKinematics_newparam>(ias_motion->add(COLLADA_ELEMENT_NEWPARAM));
                abmvalue->setSid(str(boost::format("%s_%s")%asmid%kas.valuesid).c_str());
                daeSafeCast<domKinematics_newparam::domSIDREF>(abmvalue->add(COLLADA_ELEMENT_SIDREF))->setValue(str(boost::format("%s/%s")%askid%kas.valuesid).c_str());
                domKinematics_bindRef abvalue = daeSafeCast<domKinematics_bind>(ias->add(COLLADA_ELEMENT_BIND));
                valuesid = str(boost::format("%s_%s")%assym%kas.valuesid);
                abvalue->setSymbol(valuesid.c_str());
                daeSafeCast<domKinematics_param>(abvalue->add(COLLADA_ELEMENT_PARAM))->setRef(str(boost::format("%s/%s_%s")%asmid%asmid%kas.valuesid).c_str());
            }
            iasout->vaxissids.push_back(axis_sids(ab->getSymbol(),valuesid,kas.jointnodesid));
        }

        // interface type
        {
            domExtraRef pextra = daeSafeCast<domExtra>(articulated_system_motion->add(COLLADA_ELEMENT_EXTRA));
            pextra->setType("interface_type");
            domTechniqueRef ptec = daeSafeCast<domTechnique>(pextra->add(COLLADA_ELEMENT_TECHNIQUE));
            ptec->setProfile("OpenRAVE");
            ptec->add("interface")->setCharData(probot->GetXMLId());
        }

        FOREACHC(itmanip, probot->GetManipulators()) {
            domExtraRef pextra = daeSafeCast<domExtra>(articulated_system_motion->add(COLLADA_ELEMENT_EXTRA));
            pextra->setName((*itmanip)->GetName().c_str());
            pextra->setType("manipulator");
            domTechniqueRef ptec = daeSafeCast<domTechnique>(pextra->add(COLLADA_ELEMENT_TECHNIQUE));
            ptec->setProfile("OpenRAVE");
            boost::shared_ptr<kinematics_model_output> kmout = _GetKinematics_model(KinBodyPtr(probot));
            string kmodelid = kmout->kmodel->getID(); kmodelid += "/";
            daeElementRef frame_origin = ptec->add("frame_origin");
            frame_origin->setAttribute("link",(kmodelid+kmout->vlinksids.at((*itmanip)->GetBase()->GetIndex())).c_str());
            daeElementRef frame_tip = ptec->add("frame_tip");
            frame_tip->setAttribute("link",(kmodelid+kmout->vlinksids.at((*itmanip)->GetEndEffector()->GetIndex())).c_str());
            _WriteTransformation(frame_tip,(*itmanip)->GetGraspTransform());
            int i = 0;
            FOREACHC(itindex,(*itmanip)->GetGripperIndices()) {
                daeElementRef gripper_axis = ptec->add("gripper_axis");
                gripper_axis->setAttribute("axis",(kmodelid+kmout->vaxissids.at(*itindex).sid).c_str());
                daeElementRef closingdirection = gripper_axis->add("closingdirection");
                closingdirection->add("float")->setCharData(str(boost::format("%f")%(*itmanip)->GetClosingDirection().at(i)));
                ++i;
           }
//            <iksolver interface="WAM7ikfast" type="Transform6D">
//              <free_axis axis="jointname3"/>
//            </iksolver>
//            <iksolver type="Translation3D">
//              <free_axis axis="jointname4"/>
//            </iksolver>
        }

//            if (probot->GetAttachedSensors().size() > 0)
//            {
//                domExtraRef extra   =   daeSafeCast<domExtra>(askinematics->add(COLLADA_ELEMENT_EXTRA));
//                extra->setType("sensors");
//                domTechniqueRef tech    =   daeSafeCast<domTechnique>(extra->add(COLLADA_ELEMENT_TECHNIQUE));
//                tech->setProfile("OpenRAVE");
//
//                    for (size_t i = 0; i < probot->GetAttachedSensors().size();i++)
//                    {
//                        string  strsensor   =   string("sensor")+toString(i)+string("_")+probot->GetName();
//                        string  strurl      =   string("#") + strsensor;
//                        RobotBase::AttachedSensorPtr  asensor = probot->GetAttachedSensors().at(i);
//
//                        //  Instance of sensor into 'articulated_system'
//                        domInstance_sensorRef   isensor =   daeSafeCast<domInstance_sensor>(tech->add(COLLADA_ELEMENT_INSTANCE_SENSOR));
//                        isensor->setId(asensor->GetName().c_str());
//                        isensor->setLink(asensor->GetAttachingLink()->GetName().c_str());
//                        isensor->setUrl(strurl.c_str());
//
//                        //  Sensor definition into 'library_sensors'
//                        domSensorRef    sensor  =   daeSafeCast<domSensor>(_sensorsLib->add(COLLADA_ELEMENT_SENSOR));
//                        sensor->setType(asensor->GetSensor()->GetXMLId().c_str());
//                        sensor->setId(strsensor.c_str());
//                        sensor->setName(strsensor.c_str());
//
//                        RAVELOG_VERBOSE("Plugin Name: %s\n",asensor->GetSensor()->GetXMLId().c_str());
//                    }
//            }
        
        return iasout;
    }

    /// \brief Write kinematic body in a given scene
    virtual boost::shared_ptr<instance_kinematics_model_output> _WriteInstance_kinematics_model(KinBodyPtr pbody, daeElementRef parent, const string& sidscope)
    {
        EnvironmentMutex::scoped_lock lockenv(_penv->GetMutex());
        RAVELOG_VERBOSE(str(boost::format("writing instance_kinematics_model (%d) %s\n")%pbody->GetEnvironmentId()%pbody->GetName()));
        boost::shared_ptr<kinematics_model_output> kmout = WriteKinematics_model(pbody);

        boost::shared_ptr<instance_kinematics_model_output> ikmout(new instance_kinematics_model_output());
        ikmout->kmout = kmout;
        ikmout->ikm = daeSafeCast<domInstance_kinematics_model>(parent->add(COLLADA_ELEMENT_INSTANCE_KINEMATICS_MODEL));

        string symscope, refscope;
        if( sidscope.size() > 0 ) {
            symscope = sidscope+string("_");
            refscope = sidscope+string("/");
        }
        string ikmsid = str(boost::format("%s_inst")%kmout->kmodel->getID());
        ikmout->ikm->setUrl(str(boost::format("#%s")%kmout->kmodel->getID()).c_str());
        ikmout->ikm->setSid(ikmsid.c_str());

        domKinematics_newparamRef kbind = daeSafeCast<domKinematics_newparam>(ikmout->ikm->add(COLLADA_ELEMENT_NEWPARAM));
        kbind->setSid((symscope+ikmsid).c_str());
        daeSafeCast<domKinematics_newparam::domSIDREF>(kbind->add(COLLADA_ELEMENT_SIDREF))->setValue((refscope+ikmsid).c_str());
        ikmout->vkinematicsbindings.push_back(make_pair(string(kbind->getSid()), str(boost::format("visual%d/node0")%pbody->GetEnvironmentId())));

        ikmout->vaxissids.reserve(kmout->vaxissids.size());
        vector<dReal> vdofvalues;
        pbody->GetDOFValues(vdofvalues);
        int i = 0;
        FOREACH(it,kmout->vaxissids) {
            domKinematics_newparamRef kbind = daeSafeCast<domKinematics_newparam>(ikmout->ikm->add(COLLADA_ELEMENT_NEWPARAM));
            string ref = it->sid;
            size_t index = ref.find("/");
            while(index != string::npos) {
                ref[index] = '.';
                index = ref.find("/",index+1);
            }
            string sid = symscope+ikmsid+"_"+ref;
            kbind->setSid(sid.c_str());
            daeSafeCast<domKinematics_newparam::domSIDREF>(kbind->add(COLLADA_ELEMENT_SIDREF))->setValue((refscope+ikmsid+"/"+it->sid).c_str());
            dReal value=0;
            if( it->pjoint->GetDOFIndex() >= 0 ) {
                value = vdofvalues.at(it->pjoint->GetDOFIndex()+it->iaxis);
            }
            else if( it->pjoint->GetMimicJointIndex() >= 0 ) {
                value = vdofvalues.at(pbody->GetJoints().at(it->pjoint->GetMimicJointIndex())->GetDOFIndex()+it->iaxis);
                value = it->pjoint->GetMimicCoeffs().at(0)*value + it->pjoint->GetMimicCoeffs().at(1);
            }
            else {
                vector<dReal> vlower, vupper;
                it->pjoint->GetLimits(vlower,vupper);
                if( vlower.at(it->iaxis) == vupper.at(it->iaxis) ) {
                    value = vlower.at(it->iaxis);
                }
                else {
                    RAVELOG_WARN(str(boost::format("could not assign value for joint %s\n")%it->pjoint->GetName()));
                }
            }

            domKinematics_newparamRef pvalueparam = daeSafeCast<domKinematics_newparam>(ikmout->ikm->add(COLLADA_ELEMENT_NEWPARAM));
            pvalueparam->setSid((sid+string("_value")).c_str());
            daeSafeCast<domKinematics_newparam::domFloat>(pvalueparam->add(COLLADA_ELEMENT_FLOAT))->setValue(value);
            ikmout->vaxissids.push_back(axis_sids(sid,pvalueparam->getSid(),kmout->vaxissids.at(i).jointnodesid));
            ++i;
        }

        return ikmout;
    }

    virtual boost::shared_ptr<kinematics_model_output> WriteKinematics_model(KinBodyPtr pbody)
    {
        EnvironmentMutex::scoped_lock lockenv(_penv->GetMutex());
        boost::shared_ptr<kinematics_model_output> kmout = _GetKinematics_model(pbody);
        if( !!kmout ) {
            return kmout;
        }

        domKinematics_modelRef kmodel = daeSafeCast<domKinematics_model>(_kinematicsModelsLib->add(COLLADA_ELEMENT_KINEMATICS_MODEL));
        string kmodelid = str(boost::format("kmodel%d")%pbody->GetEnvironmentId());
        kmodel->setId(kmodelid.c_str());
        kmodel->setName(pbody->GetName().c_str());

        domKinematics_model_techniqueRef ktec = daeSafeCast<domKinematics_model_technique>(kmodel->add(COLLADA_ELEMENT_TECHNIQUE_COMMON));

        KinBody::KinBodyStateSaver saver(pbody);
        vector<dReal> vjointvalues, vzero(pbody->GetDOF());
        pbody->GetDOFValues(vjointvalues);
        pbody->SetDOFValues(vzero);

        //  Create root node for the visual scene
        domNodeRef pnoderoot = daeSafeCast<domNode>(_scene.vscene->add(COLLADA_ELEMENT_NODE));
        string bodyid = str(boost::format("visual%d")%pbody->GetEnvironmentId());
        pnoderoot->setId(bodyid.c_str());
        pnoderoot->setSid(bodyid.c_str());
        pnoderoot->setName(pbody->GetName().c_str());

        //  Declare all the joints
        vector< pair<int,KinBody::JointConstPtr> > vjoints;
        vjoints.reserve(pbody->GetJoints().size()+pbody->_vecPassiveJoints.size());
        FOREACHC(itj, pbody->GetJoints() ) {
            vjoints.push_back(make_pair((*itj)->GetJointIndex(),*itj));
        }
        int index=pbody->GetJoints().size();
        FOREACHC(itj, pbody->_vecPassiveJoints) {
            vjoints.push_back(make_pair(index++,*itj));
        }
        vector<dReal> lmin, lmax;
        vector<domJointRef> vdomjoints(vjoints.size());
        kmout.reset(new kinematics_model_output());
        kmout->kmodel = kmodel;
        kmout->vaxissids.resize(vjoints.size());
        kmout->vlinksids.resize(pbody->GetLinks().size());

        FOREACHC(itjoint, vjoints) {
            KinBody::JointConstPtr pjoint = itjoint->second;
            domJointRef pdomjoint = daeSafeCast<domJoint>(ktec->add(COLLADA_ELEMENT_JOINT));
            string jointid = str(boost::format("joint%d")%itjoint->first);
            pdomjoint->setSid( jointid.c_str() );
            pdomjoint->setName(pjoint->GetName().c_str());
            pjoint->GetLimits(lmin, lmax);
            vector<domAxis_constraintRef> vaxes(pjoint->GetDOF());
            for(int ia = 0; ia < pjoint->GetDOF(); ++ia) {
                dReal fmult = 1.0;
                switch(pjoint->GetType()) {
                case KinBody::Joint::JointRevolute:
                    vaxes[ia] = daeSafeCast<domAxis_constraint>(pdomjoint->add(COLLADA_ELEMENT_REVOLUTE));
                    fmult = 180.0f/PI;
                    lmin[ia]*=fmult;
                    lmax[ia]*=fmult;
                    break;
                case KinBody::Joint::JointPrismatic:
                    vaxes[ia] = daeSafeCast<domAxis_constraint>(pdomjoint->add(COLLADA_ELEMENT_PRISMATIC));
                    break;
                case KinBody::Joint::JointUniversal:
                case KinBody::Joint::JointHinge2:
                default:
                    fmult = 180.0f/PI;
                    RAVELOG_WARN(str(boost::format("unsupported joint type specified %d\n")%pjoint->GetType()));
                    break;
                }

                if( !vaxes.at(ia) ) {
                    continue;
                }

                string axisid = str(boost::format("axis%d")%ia);
                vaxes[ia]->setSid(axisid.c_str());
                kmout->vaxissids.at(itjoint->first).pjoint = pjoint;
                kmout->vaxissids.at(itjoint->first).sid = jointid+string("/")+axisid;
                kmout->vaxissids.at(itjoint->first).iaxis = ia;
                domAxisRef paxis = daeSafeCast<domAxis>(vaxes.at(ia)->add(COLLADA_ELEMENT_AXIS));
                paxis->getValue().setCount(3);
                paxis->getValue()[0] = pjoint->vAxes.at(ia).x;
                paxis->getValue()[1] = pjoint->vAxes.at(ia).y;
                paxis->getValue()[2] = pjoint->vAxes.at(ia).z;
                if( !pjoint->IsCircular() ) {
                    domJoint_limitsRef plimits = daeSafeCast<domJoint_limits>(vaxes[ia]->add(COLLADA_TYPE_LIMITS));
                    daeSafeCast<domMinmax>(plimits->add(COLLADA_ELEMENT_MIN))->getValue() = lmin.at(ia);
                    daeSafeCast<domMinmax>(plimits->add(COLLADA_ELEMENT_MAX))->getValue() = lmax.at(ia);
                }
            }
            vdomjoints.at(itjoint->first) = pdomjoint;
        }

        list<int> listunusedlinks;
        FOREACHC(itlink,pbody->GetLinks()) {
            listunusedlinks.push_back((*itlink)->GetIndex());
        }

        while(listunusedlinks.size()>0) {
            LINKOUTPUT childinfo = _WriteLink(pbody->GetLinks().at(listunusedlinks.front()), ktec, pnoderoot, kmodel->getID(), vjoints);
            Transform t = pbody->GetLinks()[listunusedlinks.front()]->GetTransform();
            _WriteTransformation(childinfo.plink, t);
            _WriteTransformation(childinfo.pnode, t);
            FOREACHC(itused, childinfo.listusedlinks) {
                kmout->vlinksids.at(itused->first) = itused->second;
                listunusedlinks.remove(itused->first);
            }
            FOREACH(itprocessed,childinfo.listprocesseddofs) {
                kmout->vaxissids.at(itprocessed->first).jointnodesid = itprocessed->second;
            }
        }

        // interface type
        {
            domExtraRef pextra = daeSafeCast<domExtra>(kmout->kmodel->add(COLLADA_ELEMENT_EXTRA));
            pextra->setType("interface_type");
            domTechniqueRef ptec = daeSafeCast<domTechnique>(pextra->add(COLLADA_ELEMENT_TECHNIQUE));
            ptec->setProfile("OpenRAVE");
            ptec->add("interface")->setCharData(pbody->GetXMLId());
        }

        // create the formulas for all mimic joints
        FOREACHC(itjoint, vjoints) {
            KinBody::JointConstPtr pjoint = itjoint->second;
            if( pjoint->GetMimicJointIndex() < 0 ) {
                continue;
            }

            domFormulaRef pf = daeSafeCast<domFormula>(ktec->add(COLLADA_ELEMENT_FORMULA));
            string formulaid = str(boost::format("joint%d.formula")%itjoint->first);
            pf->setSid(formulaid.c_str());
            domCommon_float_or_paramRef ptarget = daeSafeCast<domCommon_float_or_param>(pf->add(COLLADA_ELEMENT_TARGET));
            string targetjointid = str(boost::format("%s/joint%d")%kmodel->getID()%itjoint->first);
            daeSafeCast<domCommon_param>(ptarget->add(COLLADA_TYPE_PARAM))->setValue(targetjointid.c_str());

            domFormula_techniqueRef pftec = daeSafeCast<domFormula_technique>(pf->add(COLLADA_ELEMENT_TECHNIQUE_COMMON));
            // create a const0*joint+const1 formula
            // <apply> <plus/> <apply> <times/> <cn>a</cn> x </apply> <cn>b</cn> </apply>
            daeElementRef pmath_math = pftec->add("math");
            daeElementRef pmath_apply = pmath_math->add("apply");
            {
                daeElementRef pmath_plus = pmath_apply->add("plus");
                daeElementRef pmath_apply1 = pmath_apply->add("apply");
                {
                    daeElementRef pmath_times = pmath_apply1->add("times");
                    daeElementRef pmath_const0 = pmath_apply1->add("cn");
                    pmath_const0->setCharData(str(boost::format("%f")%pjoint->GetMimicCoeffs().at(0)));
                    daeElementRef pmath_symb = pmath_apply1->add("csymbol");
                    pmath_symb->setAttribute("encoding","COLLADA");
                    pmath_symb->setCharData(str(boost::format("%s/joint%d")%kmodel->getID()%pjoint->GetMimicJointIndex()));
                }
                daeElementRef pmath_const1 = pmath_apply->add("cn");
                pmath_const1->setCharData(str(boost::format("%f")%pjoint->GetMimicCoeffs().at(1)));
            }
        }

        _listkinbodies.push_back(make_pair(pbody,kmout));
        return kmout;
    }

    /// \brief Write geometry properties
    /// \param geom Link geometry
    /// \param parentid Parent Identifier
    virtual domGeometryRef WriteGeometry(const KinBody::Link::GEOMPROPERTIES& geom, const string& parentid)
    {
        const KinBody::Link::TRIMESH& mesh = geom.GetCollisionMesh();

        string effid = parentid+string("_eff");
        string matid = parentid+string("_mat");

        domEffectRef pdomeff = WriteEffect(geom.GetAmbientColor(), geom.GetDiffuseColor());
        pdomeff->setId(effid.c_str());

        domMaterialRef pdommat = daeSafeCast<domMaterial>(_materialsLib->add(COLLADA_ELEMENT_MATERIAL));
        pdommat->setId(matid.c_str());
        domInstance_effectRef pdominsteff = daeSafeCast<domInstance_effect>(pdommat->add(COLLADA_ELEMENT_INSTANCE_EFFECT));
        pdominsteff->setUrl((string("#")+effid).c_str());

        domGeometryRef pdomgeom = daeSafeCast<domGeometry>(_geometriesLib->add(COLLADA_ELEMENT_GEOMETRY));
        {
            pdomgeom->setId(parentid.c_str());
            domMeshRef pdommesh = daeSafeCast<domMesh>(pdomgeom->add(COLLADA_ELEMENT_MESH));
            {
                domSourceRef pvertsource = daeSafeCast<domSource>(pdommesh->add(COLLADA_ELEMENT_SOURCE));
                {
                    pvertsource->setId((parentid+string("_positions")).c_str());

                    domFloat_arrayRef parray = daeSafeCast<domFloat_array>(pvertsource->add(COLLADA_ELEMENT_FLOAT_ARRAY));
                    parray->setId((parentid+string("_positions-array")).c_str());
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

                    domSource::domTechnique_commonRef psourcetec = daeSafeCast<domSource::domTechnique_common>(pvertsource->add(COLLADA_ELEMENT_TECHNIQUE_COMMON));
                    domAccessorRef pacc = daeSafeCast<domAccessor>(psourcetec->add(COLLADA_ELEMENT_ACCESSOR));
                    pacc->setCount(mesh.vertices.size());
                    pacc->setSource(xsAnyURI(*parray, string("#")+parentid+string("_positions-array")));
                    pacc->setStride(3);

                    domParamRef px = daeSafeCast<domParam>(pacc->add(COLLADA_ELEMENT_PARAM));
                    px->setName("X"); px->setType("float");
                    domParamRef py = daeSafeCast<domParam>(pacc->add(COLLADA_ELEMENT_PARAM));
                    py->setName("Y"); py->setType("float");
                    domParamRef pz = daeSafeCast<domParam>(pacc->add(COLLADA_ELEMENT_PARAM));
                    pz->setName("Z"); pz->setType("float");
                }

                domVerticesRef pverts = daeSafeCast<domVertices>(pdommesh->add(COLLADA_ELEMENT_VERTICES));
                {
                    pverts->setId("vertices");
                    domInput_localRef pvertinput = daeSafeCast<domInput_local>(pverts->add(COLLADA_ELEMENT_INPUT));
                    pvertinput->setSemantic("POSITION");
                    pvertinput->setSource(domUrifragment(*pvertsource, string("#")+parentid+string("_positions")));
                }

                domTrianglesRef ptris = daeSafeCast<domTriangles>(pdommesh->add(COLLADA_ELEMENT_TRIANGLES));
                {
                    ptris->setCount(mesh.indices.size()/3);
                    ptris->setMaterial("mat0");

                    domInput_local_offsetRef pvertoffset = daeSafeCast<domInput_local_offset>(ptris->add(COLLADA_ELEMENT_INPUT));
                    pvertoffset->setSemantic("VERTEX");
                    pvertoffset->setOffset(0);
                    pvertoffset->setSource(domUrifragment(*pverts, string("#")+parentid+string("/vertices")));
                    domPRef pindices = daeSafeCast<domP>(ptris->add(COLLADA_ELEMENT_P));
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
        domEffectRef pdomeff = daeSafeCast<domEffect>(_effectsLib->add(COLLADA_ELEMENT_EFFECT));

        domProfile_commonRef pprofile = daeSafeCast<domProfile_common>(pdomeff->add(COLLADA_ELEMENT_PROFILE_COMMON));
        domProfile_common::domTechniqueRef ptec = daeSafeCast<domProfile_common::domTechnique>(pprofile->add(COLLADA_ELEMENT_TECHNIQUE));

        domProfile_common::domTechnique::domPhongRef pphong = daeSafeCast<domProfile_common::domTechnique::domPhong>(ptec->add(COLLADA_ELEMENT_PHONG));

        domFx_common_color_or_textureRef pambient = daeSafeCast<domFx_common_color_or_texture>(pphong->add(COLLADA_ELEMENT_AMBIENT));
        domFx_common_color_or_texture::domColorRef pambientcolor = daeSafeCast<domFx_common_color_or_texture::domColor>(pambient->add(COLLADA_ELEMENT_COLOR));
        setVector4(pambientcolor->getValue(), vambient);

        domFx_common_color_or_textureRef pdiffuse = daeSafeCast<domFx_common_color_or_texture>(pphong->add(COLLADA_ELEMENT_DIFFUSE));
        domFx_common_color_or_texture::domColorRef pdiffusecolor = daeSafeCast<domFx_common_color_or_texture::domColor>(pdiffuse->add(COLLADA_ELEMENT_COLOR));
        setVector4(pdiffusecolor->getValue(), vdiffuse);

        return pdomeff;
    }

    /// Write down a COLLADA file
    virtual bool Save(const string& filename)
    {
        return _collada->saveAs(filename.c_str())>0;
    }

 private:

    /// \brief save all the loaded scene models and their current state.
    virtual void _CreateScene()
    {
        // Create visual scene
        _scene.vscene = daeSafeCast<domVisual_scene>(_visualScenesLib->add(COLLADA_ELEMENT_VISUAL_SCENE));
        _scene.vscene->setId("vscene");
        _scene.vscene->setName("OpenRAVE Visual Scene");

        // Create kinematics scene
        _scene.kscene = daeSafeCast<domKinematics_scene>(_kinematicsScenesLib->add(COLLADA_ELEMENT_KINEMATICS_SCENE));
        _scene.kscene->setId("kscene");
        _scene.kscene->setName("OpenRAVE Kinematics Scene");

        // Create physic scene
        _scene.pscene = daeSafeCast<domPhysics_scene>(_physicsScenesLib->add(COLLADA_ELEMENT_PHYSICS_SCENE));
        _scene.pscene->setId("pscene");
        _scene.pscene->setName("OpenRAVE Physics Scene");

        // Create instance visual scene
        _scene.viscene = daeSafeCast<domInstance_with_extra>(_globalscene->add( COLLADA_ELEMENT_INSTANCE_VISUAL_SCENE ));
        _scene.viscene->setUrl( (string("#") + string(_scene.vscene->getID())).c_str() );

        // Create instance kinematics scene
        _scene.kiscene = daeSafeCast<domInstance_kinematics_scene>(_globalscene->add( COLLADA_ELEMENT_INSTANCE_KINEMATICS_SCENE ));
        _scene.kiscene->setUrl( (string("#") + string(_scene.kscene->getID())).c_str() );

        // Create instance physics scene
        _scene.piscene = daeSafeCast<domInstance_with_extra>(_globalscene->add( COLLADA_ELEMENT_INSTANCE_PHYSICS_SCENE ));
        _scene.piscene->setUrl( (string("#") + string(_scene.pscene->getID())).c_str() );
    }

    /// \brief Write link of a kinematic body
    /// \param link Link to write
    /// \param pkinparent Kinbody parent
    /// \param pnodeparent Node parent
    /// \param strModelUri
    /// \param vjoints Vector of joints
    virtual LINKOUTPUT _WriteLink(KinBody::LinkConstPtr plink, daeElementRef pkinparent, domNodeRef pnodeparent, const string& strModelUri, const vector<pair<int, KinBody::JointConstPtr> >& vjoints)
    {
        LINKOUTPUT out;
        string linksid = str(boost::format("link%d")%plink->GetIndex());
        domLinkRef pdomlink = daeSafeCast<domLink>(pkinparent->add(COLLADA_ELEMENT_LINK));
        pdomlink->setName(plink->GetName().c_str());
        pdomlink->setSid(linksid.c_str());

        domNodeRef pnode = daeSafeCast<domNode>(pnodeparent->add(COLLADA_ELEMENT_NODE));
        string nodeid = str(boost::format("v%d.node%d")%plink->GetParent()->GetEnvironmentId()%plink->GetIndex());
        pnode->setId( nodeid.c_str() );
        string nodesid = str(boost::format("node%d")%plink->GetIndex());
        pnode->setSid(nodesid.c_str());
        pnode->setName(plink->GetName().c_str());

        int igeom = 0;
        FOREACHC(itgeom, plink->GetGeometries()) {
            string geomid = str(boost::format("g%d_%s_geom%d")%plink->GetParent()->GetEnvironmentId()%linksid%igeom);
            igeom++;
            domGeometryRef pdomgeom = WriteGeometry(*itgeom, geomid);
            domInstance_geometryRef pinstgeom = daeSafeCast<domInstance_geometry>(pnode->add(COLLADA_ELEMENT_INSTANCE_GEOMETRY));
            pinstgeom->setUrl((string("#")+geomid).c_str());

            domBind_materialRef pmat = daeSafeCast<domBind_material>(pinstgeom->add(COLLADA_ELEMENT_BIND_MATERIAL));
            domBind_material::domTechnique_commonRef pmattec = daeSafeCast<domBind_material::domTechnique_common>(pmat->add(COLLADA_ELEMENT_TECHNIQUE_COMMON));
            domInstance_materialRef pinstmat = daeSafeCast<domInstance_material>(pmattec->add(COLLADA_ELEMENT_INSTANCE_MATERIAL));
            pinstmat->setTarget(xsAnyURI(*pdomgeom, string("#")+geomid+string("_mat")));
            pinstmat->setSymbol("mat0");
        }

        // look for all the child links
        FOREACHC(itjoint, vjoints) {
            KinBody::JointConstPtr pjoint = itjoint->second;
            if( pjoint->GetFirstAttached() != plink && pjoint->GetSecondAttached() != plink ) {
                continue;
            }
            KinBody::LinkPtr pchild = GetChildLink(pjoint);
            if( !pchild || plink == pchild ) {
                continue;
            }

            domLink::domAttachment_fullRef pattfull = daeSafeCast<domLink::domAttachment_full>(pdomlink->add(COLLADA_TYPE_ATTACHMENT_FULL));
            string jointid = str(boost::format("%s/joint%d")%strModelUri%itjoint->first);
            pattfull->setJoint(jointid.c_str());

            LINKOUTPUT childinfo = _WriteLink(pchild, pattfull, pnode, strModelUri, vjoints);
            FOREACH(itused,childinfo.listusedlinks) {
                out.listusedlinks.push_back(make_pair(itused->first,linksid+string("/")+itused->second));
            }
            FOREACH(itprocessed,childinfo.listprocesseddofs) {
                out.listprocesseddofs.push_back(make_pair(itprocessed->first,nodesid+string("/")+itprocessed->second));
            }

            _WriteTransformation(pattfull, pjoint->GetInternalHierarchyLeftTransform());
            _WriteTransformation(childinfo.plink, pjoint->GetInternalHierarchyRightTransform());
            _WriteTransformation(childinfo.pnode, pjoint->GetInternalHierarchyRightTransform());

            // rotate/translate elements
            string jointnodesid = str(boost::format("node_joint%d_axis0")%itjoint->first);
            switch(pjoint->GetType()) {
            case KinBody::Joint::JointRevolute: {
                domRotateRef protate = daeSafeCast<domRotate>(childinfo.pnode->add(COLLADA_ELEMENT_ROTATE,0));
                protate->setSid(jointnodesid.c_str());
                protate->getValue().setCount(4);
                protate->getValue()[0] = pjoint->vAxes[0].x;
                protate->getValue()[1] = pjoint->vAxes[0].y;
                protate->getValue()[2] = pjoint->vAxes[0].z;
                protate->getValue()[3] = 0;
                break;
            }
            case KinBody::Joint::JointPrismatic: {
                domTranslateRef ptrans = daeSafeCast<domTranslate>(childinfo.pnode->add(COLLADA_ELEMENT_TRANSLATE,0));
                ptrans->setSid(jointnodesid.c_str());
                ptrans->getValue().setCount(3);
                ptrans->getValue()[0] = 0;
                ptrans->getValue()[1] = 0;
                ptrans->getValue()[2] = 0;
                break;
            }
            case KinBody::Joint::JointUniversal:
            case KinBody::Joint::JointHinge2:
            default:
                RAVELOG_WARN("unsupported joint type specified %d\n", pjoint->GetType());
                break;
            }

            out.listprocesseddofs.push_back(make_pair(itjoint->first,string(childinfo.pnode->getSid())+string("/")+jointnodesid));
            _WriteTransformation(childinfo.pnode, pjoint->GetInternalHierarchyLeftTransform());
        }

        out.listusedlinks.push_back(make_pair(plink->GetIndex(),linksid));
        out.plink = pdomlink;
        out.pnode = pnode;
        return  out;
    }

    /// \brief Write transformation
    /// \param pelt Element to transform
    /// \param t Transform to write
    void _WriteTransformation(daeElementRef pelt, const Transform& t)
    {
        domRotateRef prot = daeSafeCast<domRotate>(pelt->add(COLLADA_ELEMENT_ROTATE,0));
        domTranslateRef ptrans = daeSafeCast<domTranslate>(pelt->add(COLLADA_ELEMENT_TRANSLATE,0));
        ptrans->getValue().setCount(3);
        ptrans->getValue()[0] = t.trans.x;
        ptrans->getValue()[1] = t.trans.y;
        ptrans->getValue()[2] = t.trans.z;

        prot->getValue().setCount(4);
        // extract axis from quaternion
        Vector vaxisangle = axisAngleFromQuat(t.rot);
        dReal fnorm = RaveSqrt(vaxisangle.lengthsqr3());
        if( fnorm > 0 ) {
            prot->getValue()[0] = vaxisangle.x/fnorm;
            prot->getValue()[1] = vaxisangle.y/fnorm;
            prot->getValue()[2] = vaxisangle.z/fnorm;
            prot->getValue()[3] = fnorm*(180.0/PI);
        }
        else {
            prot->getValue()[0] = 1;
            prot->getValue()[1] = 0;
            prot->getValue()[2] = 0;
            prot->getValue()[3] = 0;
        }
    }

    // binding in instance_kinematics_scene
    void _WriteBindingsInstance_kinematics_scene(domInstance_kinematics_sceneRef ikscene, KinBodyConstPtr pbody, const std::vector<axis_sids>& vaxissids, const std::vector<std::pair<std::string,std::string> >& vkinematicsbindings)
    {
        FOREACHC(it, vkinematicsbindings) {
            domBind_kinematics_modelRef pmodelbind = daeSafeCast<domBind_kinematics_model>(ikscene->add(COLLADA_ELEMENT_BIND_KINEMATICS_MODEL));
            pmodelbind->setNode(it->second.c_str());
            daeSafeCast<domCommon_param>(pmodelbind->add(COLLADA_ELEMENT_PARAM))->setValue(it->first.c_str());
        }
        FOREACHC(it, vaxissids) {
            domBind_joint_axisRef pjointbind = daeSafeCast<domBind_joint_axis>(ikscene->add(COLLADA_ELEMENT_BIND_JOINT_AXIS));
            pjointbind->setTarget(it->jointnodesid.c_str());
            daeSafeCast<domCommon_param>(pjointbind->add(COLLADA_ELEMENT_AXIS)->add(COLLADA_TYPE_PARAM))->setValue(it->axissid.c_str());
            daeSafeCast<domCommon_param>(pjointbind->add(COLLADA_ELEMENT_VALUE)->add(COLLADA_TYPE_PARAM))->setValue(it->valuesid.c_str());
        }
    }

    /// Set vector of four elements
    template <typename T> static void setVector4(T& t, const Vector& v) {
        t.setCount(4);
        t[0] = v.x;
        t[1] = v.y;
        t[2] = v.z;
        t[3] = v.w;
    }

    /// Set vector of three elements
    template <typename T> static void setVector3(T& t, const Vector& v) {
        t.setCount(3);
        t[0] = v.x;
        t[1] = v.y;
        t[2] = v.z;
    }

    virtual void handleError( daeString msg )
    {
        RAVELOG_ERROR("COLLADA error: %s\n", msg);
    }

    virtual void handleWarning( daeString msg )
    {
        RAVELOG_WARN("COLLADA warning: %s\n", msg);
    }

    virtual KinBody::LinkPtr GetChildLink(KinBody::JointConstPtr pjoint) {
        if( !!pjoint->GetFirstAttached() && !!pjoint->GetSecondAttached() ) {
            if( pjoint->GetFirstAttached()->GetParentLink() == pjoint->GetSecondAttached() ) {
                RAVELOG_WARN("returning first link\n");
                return pjoint->GetFirstAttached();
            }
            else if( pjoint->GetSecondAttached()->GetParentLink() == pjoint->GetFirstAttached() )
                return pjoint->GetSecondAttached();
        }
        else if( !!pjoint->GetFirstAttached() ) {
            RAVELOG_WARN("returning first link\n");
            return pjoint->GetFirstAttached();
        }
        else if( !!pjoint->GetSecondAttached() ) {
            return pjoint->GetSecondAttached();
        }
        RAVELOG_WARN(str(boost::format("joint %s cannot find child link\n")%pjoint->GetName()));
        return KinBody::LinkPtr();
    }

    virtual boost::shared_ptr<kinematics_model_output> _GetKinematics_model(KinBodyPtr pbody) {
        FOREACH(it, _listkinbodies) {
            if( it->first->GetXMLFilename() == pbody->GetXMLFilename() && it->first->GetKinematicsGeometryHash() == pbody->GetKinematicsGeometryHash() ) {
                return it->second;
            }
        }
        return boost::shared_ptr<kinematics_model_output>();
    }

    boost::shared_ptr<DAE> _collada;
    domCOLLADA* _dom;
    domCOLLADA::domSceneRef _globalscene;
    domLibrary_visual_scenesRef _visualScenesLib;
    domLibrary_kinematics_scenesRef _kinematicsScenesLib;
    domLibrary_kinematics_modelsRef _kinematicsModelsLib;
    domLibrary_articulated_systemsRef _articulatedSystemsLib;
    domLibrary_physics_scenesRef _physicsScenesLib;
    domLibrary_materialsRef _materialsLib;
    domLibrary_effectsRef _effectsLib;
    domLibrary_geometriesRef _geometriesLib;
    domTechniqueRef _sensorsLib;///< custom sensors library
    SCENE _scene;
    EnvironmentBaseConstPtr _penv;
    list< pair<KinBodyPtr, boost::shared_ptr<kinematics_model_output> > > _listkinbodies;
    //list< pair<RobotBasePtr, boost::shared_ptr<articulated_system_output> > _listrobots;
};

// register for typeof (MSVC only)
#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(ColladaWriter::LINKOUTPUT)
BOOST_TYPEOF_REGISTER_TYPE(ColladaWriter::kinematics_model_output)
BOOST_TYPEOF_REGISTER_TYPE(ColladaWriter::instance_kinematics_model_output)
BOOST_TYPEOF_REGISTER_TYPE(ColladaWriter::articulated_system_output)
#endif

#endif
