// Copyright (C) 2006-2008 Carnegie Mellon University (rdiankov@cs.cmu.edu)
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
#include <assert.h>
#include <cstdio>
#include <cmath>

// include boost for vc++ only (to get typeof working)
#ifdef _MSC_VER
#include <boost/typeof/std/string.hpp>
#include <boost/typeof/std/vector.hpp>
#include <boost/typeof/std/list.hpp>
#include <boost/typeof/std/map.hpp>
#include <boost/typeof/std/set.hpp>
#include <boost/typeof/std/string.hpp>

#define FOREACH(it, v) for(BOOST_TYPEOF(v)::iterator it = (v).begin(); it != (v).end(); (it)++)
#define FOREACH_NOINC(it, v) for(BOOST_TYPEOF(v)::iterator it = (v).begin(); it != (v).end(); )

#define FOREACHC(it, v) for(BOOST_TYPEOF(v)::const_iterator it = (v).begin(); it != (v).end(); (it)++)
#define FOREACHC_NOINC(it, v) for(BOOST_TYPEOF(v)::const_iterator it = (v).begin(); it != (v).end(); )
#define RAVE_REGISTER_BOOST

#else
#include <string>
#include <vector>
#include <list>
#include <map>
#include <set>
#include <string>

#define FOREACH(it, v) for(typeof((v).begin()) it = (v).begin(); it != (v).end(); (it)++)
#define FOREACH_NOINC(it, v) for(typeof((v).begin()) it = (v).begin(); it != (v).end(); )

#define FOREACHC FOREACH
#define FOREACHC_NOINC FOREACH_NOINC

#endif

#include <stdint.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>

#include <boost/shared_ptr.hpp>

class ColladaWriter;

#include "../ravep.h"

using namespace OpenRAVE;
using namespace std;

#ifdef OPENRAVE_COLLADA_SUPPORT

#include "dae.h"
#include "dae/daeErrorHandler.h"
#include "dom/domCOLLADA.h"
#include "dae/domAny.h"
#include "dom/domConstants.h"
#include "dom/domTriangles.h"
#include "dae/daeDocument.h"
#include "dom/domTypes.h"
#include "dom/domElements.h"

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


    ColladaWriter(const EnvironmentBase* penv) : _dom(NULL), _penv(penv)
    {
        daeErrorHandler::setErrorHandler(this);

        RAVELOG_VERBOSEA("init COLLADA reader version: %s, namespace: %s\n", COLLADA_VERSION, COLLADA_NAMESPACE);
        _collada.reset(new DAE);
        _collada->setIOPlugin( NULL );
		_collada->setDatabase( NULL );
        
		const char* documentName = "openrave_snapshot";

		daeDocument *doc = NULL;
		daeInt error = _collada->getDatabase()->insertDocument(documentName, &doc ); // also creates a collada root
		if ( error != DAE_OK || doc == NULL ) {
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
            _scene = daeSafeCast<domCOLLADA::domScene>( _dom->createAndPlace( COLLADA_ELEMENT_SCENE ) );
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
        _kinematicsScenesLib = daeSafeCast<domLibrary_kinematics_scenes>(_dom->createAndPlace (COLLADA_ELEMENT_LIBRARY_KINEMATICS_SCENES));
        _kinematicsScenesLib->setId("kscenes");
        _physicsScenesLib = daeSafeCast<domLibrary_physics_scenes>(_dom->createAndPlace (COLLADA_ELEMENT_LIBRARY_PHYSICS_SCENES));
        _physicsScenesLib->setId("pscenes");
    }
    virtual ~ColladaWriter()
    {
        _collada.reset();
        DAE::cleanup();
    }

    virtual SCENE CreateScene()
    {
        SCENE s;
		s.vscene = daeSafeCast<domVisual_scene>(_visualScenesLib->createAndPlace (COLLADA_ELEMENT_VISUAL_SCENE));
        s.vscene->setId("vscene");
		s.vscene->setName("OpenRAVE Visual Scene");

		s.kscene = daeSafeCast<domKinematics_scene>(_kinematicsScenesLib->createAndPlace (COLLADA_ELEMENT_KINEMATICS_SCENE));
        s.kscene->setId("kscene");
		s.kscene->setName("OpenRAVE Kinematics Scene");

		s.pscene = daeSafeCast<domPhysics_scene>(_physicsScenesLib->createAndPlace (COLLADA_ELEMENT_PHYSICS_SCENE));
        s.pscene->setId("pscene");
		s.pscene->setName("OpenRAVE Physics Scene");

        s.viscene = daeSafeCast<domInstance_with_extra>( _scene->createAndPlace( COLLADA_ELEMENT_INSTANCE_VISUAL_SCENE ) );

        s.viscene->setUrl( (string("#") + string(s.vscene->getID())).c_str() );
        
        s.kiscene = daeSafeCast<domInstance_kinematics_scene>( _scene->createAndPlace( COLLADA_ELEMENT_INSTANCE_KINEMATICS_SCENE ) );
        s.kiscene->setUrl( (string("#") + string(s.kscene->getID())).c_str() );

        s.piscene = daeSafeCast<domInstance_with_extra>( _scene->createAndPlace( COLLADA_ELEMENT_INSTANCE_PHYSICS_SCENE ) );
        s.piscene->setUrl( (string("#") + string(s.pscene->getID())).c_str() );
        return s;
    }

    virtual bool Write(EnvironmentBase* penv)
    {
        SCENE scene = CreateScene();

		domPhysics_scene::domTechnique_commonRef common = daeSafeCast<domPhysics_scene::domTechnique_common>(scene.pscene->createAndPlace (COLLADA_ELEMENT_TECHNIQUE_COMMON));
		domTargetable_float3Ref g = daeSafeCast<domTargetable_float3>(common->createAndPlace (COLLADA_ELEMENT_GRAVITY));
		Vector vgravity = penv->GetPhysicsEngine()->GetGravity();
		g->getValue().set3 (vgravity.x, vgravity.y, vgravity.z);

        FOREACHC(itbody, penv->GetBodies()) {
            if( !Write(*itbody, scene) ) {
                RAVELOG_ERRORA("failed to write body %S\n", (*itbody)->GetName());
                continue;
            }
        }

        return true;
    }

    virtual bool Write(KinBody* pbody)
    {
        SCENE scene = CreateScene();
        return Write(pbody, scene);
    }
    
    virtual bool Write(KinBody* pbody, SCENE& scene)
    {
        KinBody::KinBodyStateSaver saver(pbody);
        vector<dReal> vjointvalues, vzero(pbody->GetDOF());
        pbody->GetJointValues(vjointvalues);
        if( vzero.size() > 0 )
            pbody->SetJointValues(NULL, NULL, &vzero[0]);

        domNodeRef pnoderoot = daeSafeCast<domNode>(scene.vscene->createAndPlace(COLLADA_ELEMENT_NODE));
        string bodyid = string("v")+toString(pbody->GetNetworkId());
        pnoderoot->setId(bodyid.c_str());
        pnoderoot->setName(_stdwcstombs(pbody->GetName()).c_str());

        domKinematics_modelRef kmodel = daeSafeCast<domKinematics_model>(_kinematicsModelsLib->createAndPlace(COLLADA_ELEMENT_KINEMATICS_MODEL));
        kmodel->setId((string("k")+toString(pbody->GetNetworkId())).c_str());
        kmodel->setName(_stdwcstombs(pbody->GetName()).c_str());

        string strModelId = kmodel->getID();
        string strInstModelSid = string("inst_") + string(kmodel->getID());

        vector<boost::shared_ptr<KinBody::Joint> > vjoints;
        vjoints.reserve(pbody->GetJoints().size()+pbody->_vecPassiveJoints.size());
        FOREACH(itj, pbody->GetJoints() ) {
            KinBody::Joint* pj = new KinBody::Joint(pbody);
            *pj = **itj;
            vjoints.push_back(boost::shared_ptr<KinBody::Joint>(pj));
        }
        int dof = pbody->GetDOF();
        FOREACHC(itj, pbody->_vecPassiveJoints) {
            KinBody::Joint* pj = new KinBody::Joint(pbody);
            *pj = **itj;
            vjoints.push_back(boost::shared_ptr<KinBody::Joint>(pj));
            vjoints.back()->jointindex += pbody->GetJoints().size();
            vjoints.back()->dofindex = dof;
            dof += vjoints.back()->GetDOF();
        }

        domInstance_kinematics_modelRef kimodel = daeSafeCast<domInstance_kinematics_model>(scene.kscene->createAndPlace(COLLADA_ELEMENT_INSTANCE_KINEMATICS_MODEL));
        { // set the instance kinematics model and default initial joint values, also create the bindings with scene
            kimodel->setSid(strInstModelSid.c_str());
            kimodel->setUrl((string("#")+strModelId).c_str());
            string sidinstmodelbase = string(scene.kscene->getID()) + string(".") + strInstModelSid;
            string uriinstmodelbase = string(scene.kscene->getID()) + string("/") + strInstModelSid;
            domKinematics_newparamRef pmodelparam = daeSafeCast<domKinematics_newparam>(kimodel->createAndPlace(COLLADA_ELEMENT_NEWPARAM));
            pmodelparam->setSid(sidinstmodelbase.c_str());
            domKinematics_newparam::domSIDREFRef sidref = daeSafeCast<domKinematics_newparam::domSIDREF>(pmodelparam->createAndPlace(COLLADA_ELEMENT_SIDREF));
            sidref->setValue(uriinstmodelbase.c_str());

            domBind_kinematics_modelRef pmodelbind = daeSafeCast<domBind_kinematics_model>(scene.kiscene->createAndPlace(COLLADA_ELEMENT_BIND_KINEMATICS_MODEL));
            pmodelbind->setNode((bodyid+string(".node0")).c_str());
            daeSafeCast<domCommon_param>(pmodelbind->createAndPlace(COLLADA_ELEMENT_PARAM))->setValue(sidinstmodelbase.c_str());

            FOREACH(itjoint, vjoints) {
                string jointname = string("joint") + toString((*itjoint)->GetJointIndex());
                KinBody::Link* pchildlink = GetChildLink(*itjoint, vjoints);
                if( pchildlink == NULL )
                    continue;

                for(int idof = 0; idof < (*itjoint)->GetDOF(); ++idof) {
                    string axisname = string("axis") + toString(idof);
                    string sidname = sidinstmodelbase + string(".") + jointname + string(".") + axisname;
                    string uriname = uriinstmodelbase + string("/") + jointname + string("/") + axisname;

                    // binding
                    domKinematics_newparamRef paxisparam = daeSafeCast<domKinematics_newparam>(kimodel->createAndPlace(COLLADA_ELEMENT_NEWPARAM));
                    paxisparam->setSid(sidname.c_str());
                    domKinematics_newparam::domSIDREFRef sidref = daeSafeCast<domKinematics_newparam::domSIDREF>(paxisparam->createAndPlace(COLLADA_ELEMENT_SIDREF));
                    sidref->setValue(uriname.c_str());
                    // value
                    domKinematics_newparamRef pvalueparam = daeSafeCast<domKinematics_newparam>(kimodel->createAndPlace(COLLADA_ELEMENT_NEWPARAM));
                    pvalueparam->setSid((sidname+string("_value")).c_str());
                    domKinematics_newparam::domFloatRef valueref = daeSafeCast<domKinematics_newparam::domFloat>(pvalueparam->createAndPlace(COLLADA_ELEMENT_FLOAT));
                    valueref->getValue() = vjointvalues[(*itjoint)->GetDOFIndex()+idof];

                    // binding
                    domBind_joint_axisRef pjointbind = daeSafeCast<domBind_joint_axis>(scene.kiscene->createAndPlace(COLLADA_ELEMENT_BIND_JOINT_AXIS));
                    pjointbind->setTarget((bodyid+string(".node")+toString(pchildlink->GetIndex())+string("/node_joint")+toString((*itjoint)->GetJointIndex())+string("_axis")+toString(idof)).c_str());
                    domCommon_sidref_or_paramRef paxisbind = daeSafeCast<domCommon_sidref_or_param>(pjointbind->createAndPlace(COLLADA_ELEMENT_AXIS));
                    daeSafeCast<domCommon_param>(paxisbind->createAndPlace(COLLADA_TYPE_PARAM))->setValue(sidname.c_str());
                    domCommon_float_or_paramRef pvaluebind = daeSafeCast<domCommon_float_or_param>(pjointbind->createAndPlace(COLLADA_ELEMENT_VALUE));
                    daeSafeCast<domCommon_param>(pvaluebind->createAndPlace(COLLADA_TYPE_PARAM))->setValue((sidname+string("_value")).c_str());
                }
            }
        }

        domKinematics_model_techniqueRef ktec = daeSafeCast<domKinematics_model_technique>(kmodel->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));

        // declare all the joints
        vector<domJointRef> vdomjoints(vjoints.size());
        FOREACHC(itjoint, vjoints) {
            domJointRef pjoint = daeSafeCast<domJoint>(ktec->createAndPlace(COLLADA_ELEMENT_JOINT));
            pjoint->setSid( (string("joint")+toString((*itjoint)->GetJointIndex())).c_str() );
            pjoint->setName(_stdwcstombs((*itjoint)->GetName()).c_str());
            
            vector<dReal> lmin((*itjoint)->GetDOF()), lmax((*itjoint)->GetDOF());
            (*itjoint)->GetLimits(&lmin[0], &lmax[0]);

            vector<domAxis_constraintRef> vaxes((*itjoint)->GetDOF());
            for(int ia = 0; ia < (*itjoint)->GetDOF(); ++ia) {
                switch((*itjoint)->GetType()) {
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
                    continue;

                vaxes[ia]->setSid((string("axis")+toString(ia)).c_str());
                domAxisRef paxis = daeSafeCast<domAxis>(vaxes[ia]->createAndPlace(COLLADA_ELEMENT_AXIS));
                paxis->getValue().setCount(3);
                paxis->getValue()[0] = (*itjoint)->vAxes[ia].x;
                paxis->getValue()[1] = (*itjoint)->vAxes[ia].y;
                paxis->getValue()[2] = (*itjoint)->vAxes[ia].z;
                domJoint_limitsRef plimits = daeSafeCast<domJoint_limits>(vaxes[ia]->createAndPlace(COLLADA_TYPE_LIMITS));
                daeSafeCast<domMinmax>(plimits->createAndPlace(COLLADA_ELEMENT_MIN))->getValue() = lmin[ia];
                daeSafeCast<domMinmax>(plimits->createAndPlace(COLLADA_ELEMENT_MAX))->getValue() = lmax[ia];
            }
            
            vdomjoints[(*itjoint)->GetJointIndex()] = pjoint;
        }

        list<int> listunusedlinks;
        for(int ilink = 0; ilink < (int)pbody->GetLinks().size(); ++ilink)
            listunusedlinks.push_back(ilink);

        while(listunusedlinks.size()>0) {
            LINKOUTPUT childinfo = WriteLink(pbody->GetLinks()[listunusedlinks.front()], ktec, pnoderoot, strModelId, vjoints);
            Transform t = pbody->GetLinks()[listunusedlinks.front()]->GetTransform();
            AddTransformation(childinfo.plink, t, false);
            AddTransformation(childinfo.pnode, t, false);

            
            FOREACHC(itused, childinfo.listusedlinks)
                listunusedlinks.erase(find(listunusedlinks.begin(),listunusedlinks.end(),*itused));
        }

        // process all mimic joints
        FOREACH(itjoint, vjoints) {
            if( (*itjoint)->GetMimicJointIndex() < 0 )
                continue;
            
            domFormulaRef pf = daeSafeCast<domFormula>(ktec->createAndPlace(COLLADA_ELEMENT_FORMULA));
            pf->setSid((string("joint")+toString((*itjoint)->GetJointIndex())+string(".formula")).c_str());
            domCommon_float_or_paramRef ptarget = daeSafeCast<domCommon_float_or_param>(pf->createAndPlace(COLLADA_ELEMENT_TARGET));
            daeSafeCast<domCommon_param>(ptarget->createAndPlace(COLLADA_TYPE_PARAM))->setValue((strModelId+string("/joint")+toString((*itjoint)->GetJointIndex())).c_str());

            domFormula_techniqueRef pftec = daeSafeCast<domFormula_technique>(pf->createAndPlace(COLLADA_ELEMENT_TECHNIQUE_COMMON));
            // create a const0*joint+const1 formula
            // <math:math>
            //   <math:apply>
            //     <math:minus/>
            //     <math:apply>
            //       <math:times>
            //       <math:ci>const0</math:ci>
            //       <math:csymbol encoding="COLLADA">joint</math:csymbol>
            //     </math:apply>
            //     <math:ci>-const1</math:ci>
            //   </math:apply>
            // </math:math>
            daeElementRef pmath_math = pftec->createAndPlace("math:math");
            daeElementRef pmath_apply = pmath_math->createAndPlace("math:apply");
            {
                daeElementRef pmath_minus = pmath_apply->createAndPlace("math:minus");
                daeElementRef pmath_apply = pmath_minus->createAndPlace("math:apply");
                {
                    daeElementRef pmath_times = pmath_apply->createAndPlace("math:times");
                    daeElementRef pmath_const0 = pmath_apply->createAndPlace("math:ci");
                    pmath_const0->setCharData(toString((*itjoint)->GetMimicCoeffs()[0]));
                    daeElementRef pmath_symb = pmath_apply->createAndPlace("math:csymbol");
                    pmath_symb->setAttribute("encoding","COLLADA");
                    pmath_symb->setCharData(strModelId+string("/joint")+toString((*itjoint)->GetMimicJointIndex()));
                }
                daeElementRef pmath_const1 = pmath_minus->createAndPlace("math:ci");
                pmath_const1->setCharData(toString(-(*itjoint)->GetMimicCoeffs()[1]));
            }
        }

        return true;
    }

    virtual bool Write(RobotBase* probot)
    {
        return true;
    }

    virtual LINKOUTPUT WriteLink(const KinBody::Link* plink, daeElementRef pkinparent, domNodeRef pnodeparent, const string& strModelUri, const vector<boost::shared_ptr<KinBody::Joint> >& vjoints)
    {
        LINKOUTPUT out;
        string linkid = string("link")+toString(plink->GetIndex());
        domLinkRef pdomlink = daeSafeCast<domLink>(pkinparent->createAndPlace(COLLADA_ELEMENT_LINK));
        pdomlink->setName(_stdwcstombs(plink->GetName()).c_str());
        pdomlink->setSid(linkid.c_str());

        domNodeRef pnode = daeSafeCast<domNode>(pnodeparent->createAndPlace(COLLADA_ELEMENT_NODE));
        pnode->setId( (string("v")+toString(plink->GetParent()->GetNetworkId())+string(".node")+toString(plink->GetIndex())).c_str() );
        pnode->setSid( (string("node")+toString(plink->GetIndex())).c_str());
        pnode->setName(_stdwcstombs(plink->GetName()).c_str());

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
            KinBody::Link* pchild = GetChildLink(*itjoint, vjoints);
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
            switch((*itjoint)->GetType()) {
            case KinBody::Joint::JointRevolute: {
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
        return out;
    }

    virtual domGeometryRef WriteGeometry(const KinBody::Link::GEOMPROPERTIES& geom, const string& parentid)
    {
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
                    for(size_t ind = 0; ind < mesh.vertices.size(); ++ind) {
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

    virtual domEffectRef WriteEffect(const Vector& vambient, const Vector& vdiffuse)
    {
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

    void AddTransformation(daeElementRef pelt, Transform t, bool bAtEnd = true)
    {
        domTranslateRef ptrans;
        domRotateRef prot;
        if( bAtEnd ) {
            ptrans = daeSafeCast<domTranslate>(pelt->createAndPlace(COLLADA_ELEMENT_TRANSLATE));
            prot = daeSafeCast<domRotate>(pelt->createAndPlace(COLLADA_ELEMENT_ROTATE));
        }
        else {
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
        if( fnorm > 0 ) {
            prot->getValue()[0] = t.rot.y/fnorm;
            prot->getValue()[1] = t.rot.z/fnorm;
            prot->getValue()[2] = t.rot.w/fnorm;
            prot->getValue()[3] = atan2(fnorm, t.rot.x)*360.0f/PI;
        }
        else {
            prot->getValue()[0] = 1;
            prot->getValue()[1] = 0;
            prot->getValue()[2] = 0;
            prot->getValue()[3] = 0;
        }
    }

    template <typename T>
    void setVector4(T& t, const Vector& v)
    {
        t.setCount(4);
        t[0] = v.x;
        t[1] = v.y;
        t[2] = v.z;
        t[3] = v.w;
    }

    template <typename T>
    void setVector3(T& t, const Vector& v)
    {
        t.setCount(3);
        t[0] = v.x;
        t[1] = v.y;
        t[2] = v.z;
    }

    virtual bool Save(const char* filename)
    {
        return _collada->saveAs(filename);
    }

    virtual void handleError( daeString msg )
    {
        RAVELOG_ERRORA("COLLADA error: %s\n", msg);
    }

	virtual void handleWarning( daeString msg )
    {
        RAVELOG_WARNA("COLLADA warning: %s\n", msg);
    }

    virtual KinBody::Link* GetChildLink(boost::shared_ptr<KinBody::Joint> pjoint, const vector<boost::shared_ptr<KinBody::Joint> >& vjoints)
    {
        const KinBody* pbody = pjoint->GetParent();
        KinBody::Link* pchildlink = NULL;
        int jointindex = pjoint->GetMimicJointIndex() < 0 ? pjoint->GetJointIndex() : pjoint->GetMimicJointIndex();
        if( pjoint->GetFirstAttached() != NULL && pbody->DoesAffect(jointindex, pjoint->GetFirstAttached()->GetIndex()) )
            pchildlink = pjoint->GetFirstAttached();
        if( pjoint->GetSecondAttached() != NULL && pbody->DoesAffect(jointindex, pjoint->GetSecondAttached()->GetIndex()) ) {
            assert( pjoint->GetMimicJointIndex() >= 0 || pchildlink == NULL );
            bool bSetSecond = true;
            if( pchildlink != NULL ) {
                // in case both are affected, choose link closest to base
                // by checking which parent joints contain the link
                FOREACHC(itjoint, vjoints) {
                    if( (*itjoint)->GetMimicJointIndex() < 0 ) {
                        if( (*itjoint)->GetFirstAttached() == pjoint->GetSecondAttached() || 
                            (*itjoint)->GetSecondAttached() == pjoint->GetSecondAttached() ) {
                            bSetSecond = false;
                            break;
                        }
                    }
                }
            }

            if( bSetSecond )
                pchildlink = pjoint->GetSecondAttached();
        }

        if( pchildlink == NULL )
            RAVELOG_ERRORA("joint %S attached to invalid links\n", pjoint->GetName());

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
    domCOLLADA::domSceneRef _scene;
    domLibrary_visual_scenesRef _visualScenesLib;
    domLibrary_kinematics_scenesRef _kinematicsScenesLib;
    domLibrary_kinematics_modelsRef _kinematicsModelsLib;
    domLibrary_physics_scenesRef _physicsScenesLib;
    domLibrary_materialsRef _materialsLib;
    domLibrary_effectsRef _effectsLib;
    domLibrary_geometriesRef _geometriesLib;
    const EnvironmentBase* _penv;
};

bool RaveWriteColladaFile(EnvironmentBase* penv, const char* filename)
{
    ColladaWriter writer(penv);
    if( !writer.Write(penv) )
        return false;
    return writer.Save(filename);
}

bool RaveWriteColladaFile(KinBody* pbody, const char* filename)
{
    ColladaWriter writer(pbody->GetEnv());
    if( !writer.Write(pbody) )
        return false;
    return writer.Save(filename);
}

bool RaveWriteColladaFile(RobotBase* probot, const char* filename)
{
    ColladaWriter writer(probot->GetEnv());
    if( !writer.Write(probot) )
        return false;
    return writer.Save(filename);
}

#else

bool RaveWriteColladaFile(EnvironmentBase* penv, const char* filename)
{
    RAVELOG_ERRORA("collada files not supported\n");
    return false;
}

bool RaveWriteColladaFile(KinBody* pbody, const char* filename)
{
    RAVELOG_ERRORA("collada files not supported\n");
    return false;
}

bool RaveWriteColladaFile(RobotBase* probot, const char* filename)
{
    RAVELOG_ERRORA("collada files not supported\n");
    return false;
}

#endif
