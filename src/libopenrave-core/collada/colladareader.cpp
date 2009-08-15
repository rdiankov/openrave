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

class ColladaReader;

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

class ColladaReader : public daeErrorHandler
{
    struct KINEMATICSBINDING
    {
        KINEMATICSBINDING(daeElementRef pvisualtrans, domAxis_constraintRef pkinematicaxis, dReal fjointvalue) : pvisualtrans(pvisualtrans), pkinematicaxis(pkinematicaxis), fjointvalue(fjointvalue) {
            assert( pkinematicaxis != NULL );
            pvisualnode = NULL;
            daeElement* pae = pvisualtrans->getParentElement();
            while(pae != NULL) {
                pvisualnode = daeSafeCast<domNode>(pae);
                if( pvisualnode != NULL )
                    break;
                pae = pae->getParentElement();
            }

            if( pvisualnode == NULL )
                RAVELOG_WARNA("couldn't find parent node of element id %s, sid %s\n", pkinematicaxis->getID(), pkinematicaxis->getSid());
        }
        daeElementRef pvisualtrans;
        domAxis_constraintRef pkinematicaxis;
        dReal fjointvalue;
        domNodeRef pvisualnode;
    };

    struct USERDATA
    {
        USERDATA() {}
        USERDATA(dReal scale) : scale(scale) {}
        dReal scale;
    };

public:
    ColladaReader(EnvironmentBase* penv) : _dom(NULL), _penv(penv)
    {
        daeErrorHandler::setErrorHandler(this);
    }
    virtual ~ColladaReader()
    {
        _collada.reset();
        DAE::cleanup();
    }

    bool Init(const string& filename)
    {
        RAVELOG_VERBOSEA("init COLLADA reader version: %s, namespace: %s, filename: %s\n", COLLADA_VERSION, COLLADA_NAMESPACE, filename.c_str());
        _collada.reset(new DAE);
        _dom = _collada->open(filename);
        if( !_dom )
            return false;

        size_t maxchildren = countChildren(_dom);
        _vuserdata.resize(0); _vuserdata.reserve(maxchildren);
        processUserData(_dom,1);
        RAVELOG_VERBOSEA("processed children: %d/%d\n", _vuserdata.size(), maxchildren);
        return true;
    }
    bool Init(const char* pdata, int len)
    {
        assert(0);
        return false;
    }

    size_t countChildren(daeElement* pelt)
    {
        size_t c = 1;
        for(size_t i = 0; i < pelt->getChildren().getCount(); ++i)
            c += countChildren(pelt->getChildren()[i]);
        return c;
    }

    void processUserData(daeElement* pelt, dReal scale)
    {
        // getChild could be optimized since asset tag is supposed to appear as the first element
        domAssetRef passet = daeSafeCast<domAsset>(pelt->getChild("asset"));
        if( passet != NULL && passet->getUnit() != NULL )
            scale = passet->getUnit()->getMeter();

        _vuserdata.push_back(USERDATA(scale));
        pelt->setUserData(&_vuserdata.back());

        for(size_t i = 0; i < pelt->getChildren().getCount(); ++i) {
            if( pelt->getChildren()[i] != passet )
                processUserData(pelt->getChildren()[i], scale);
        }
    }

    bool Extract(EnvironmentBase* penv)
    {
        domCOLLADA::domSceneRef allscene = _dom->getScene();
        assert(allscene != NULL);
        vector<domKinematics_newparam*> vnewparams;

        for (size_t iscene = 0; iscene < allscene->getInstance_kinematics_scene_array().getCount(); iscene++) {
            domInstance_kinematics_sceneRef kiscene = allscene->getInstance_kinematics_scene_array()[iscene];
            domKinematics_sceneRef kscene = daeSafeCast<domKinematics_scene>(kiscene->getUrl().getElement().cast());
            if( !kscene )
                continue;
            for(size_t imodel = 0; imodel < kiscene->getBind_kinematics_model_array().getCount(); imodel++) {
                domBind_kinematics_modelRef kbindmodel = kiscene->getBind_kinematics_model_array()[imodel];
                if( kbindmodel->getNode() == NULL ) {
                    RAVELOG_WARNA("do not support kinematics models without references to nodes\n");
                    continue;
                }

                domNodeRef pnode = daeSafeCast<domNode>(daeSidRef(kbindmodel->getNode(), kbindmodel).resolve().elt);
                if( pnode == NULL || pnode->typeID() != domNode::ID() ) {
                    RAVELOG_WARNA("bind_kinematics_model does not reference valid node %s\n", kbindmodel->getNode());
                    continue;
                }

                domInstance_kinematics_modelRef kimodel = resolve<domInstance_kinematics_model>(kbindmodel, kscene->getInstance_kinematics_model_array());
                if( kimodel == NULL ) {
                    RAVELOG_WARNA("bind_kinematics_model does not reference valid kinematics\n");
                    continue;
                }

                domKinematics_modelRef kmodel = daeSafeCast<domKinematics_model>(kimodel->getUrl().getElement().cast());
                if( !kmodel ) {
                    RAVELOG_WARNA("bind_kinematics_model does not reference valid kinematics\n");
                    return false;
                }

                // get the joint indices
                vector<KINEMATICSBINDING> vbindings;
                for(size_t ijoint = 0; ijoint < kiscene->getBind_joint_axis_array().getCount(); ++ijoint) {
                    domBind_joint_axisRef bindjoint = kiscene->getBind_joint_axis_array()[ijoint];

                    daeElementRef pjtarget = daeSidRef(bindjoint->getTarget(), bindjoint).resolve().elt;

                    if( pjtarget == NULL ) {
                        RAVELOG_WARNA("target node %s not found\n", bindjoint->getTarget());
                        continue;
                    }

                    domAxis_constraintRef pjointaxis = resolve<domAxis_constraint>(bindjoint->getAxis(), kscene->getInstance_kinematics_model_array());
                    if( pjointaxis == NULL ) {
                        RAVELOG_WARNA("joint axis not found\n");
                        continue;
                    }

                    domFloat fdefjointvalue = resolveFloat(bindjoint->getValue(), kscene->getInstance_kinematics_model_array());
                    vbindings.push_back(KINEMATICSBINDING(pjtarget,pjointaxis,fdefjointvalue));
                }

                domPhysics_modelRef pmodel = NULL;
                KinBody* pbody = NULL;
                if( !Extract(&pbody, kmodel, pmodel, pnode, vbindings) ) {
                    RAVELOG_WARNA("failed to load kinbody from kin instance %s\n", kimodel->getID());
                    continue;
                }

                _penv->AddKinBody(pbody);
            }
        }
        
        return true;
    }

    bool Extract(RobotBase** pprobot)
    {
        return true;
    }

    bool Extract(KinBody** ppbody)
    {
        return true;
    }

    // create an openrave body
    bool Extract(KinBody** ppkinbody, domKinematics_modelRef kmodel, domPhysics_modelRef pmodel, domNodeRef pnode, const vector<KINEMATICSBINDING>& vbindings)
    {
        if( *ppkinbody == NULL )
            *ppkinbody = _penv->CreateKinBody();
        KinBody* pkinbody = *ppkinbody;
        pkinbody->SetName(kmodel->getName());

        vector<domJointRef> vdomjoints;
        domKinematics_model_techniqueRef ktec = kmodel->getTechnique_common();
        for(size_t ijoint = 0; ijoint < ktec->getJoint_array().getCount(); ++ijoint)
            vdomjoints.push_back(ktec->getJoint_array()[ijoint]);
        for(size_t ijoint = 0; ijoint < ktec->getInstance_joint_array().getCount(); ++ijoint) {
            domJointRef pelt = daeSafeCast<domJoint>(ktec->getInstance_joint_array()[ijoint]->getUrl().getElement());
            if( !pelt )
                RAVELOG_WARNA("failed to get joint from instance\n");
            else
                vdomjoints.push_back(pelt);
        }
                
        for(size_t ilink = 0; ilink < ktec->getLink_array().getCount(); ++ilink) {
            domLinkRef plink = ktec->getLink_array()[ilink];
            ExtractLink(pkinbody, plink, pnode, Transform(), vdomjoints, vbindings);
        }

        for(size_t iform = 0; iform < ktec->getFormula_array().getCount(); ++iform) {
            domFormulaRef pf = ktec->getFormula_array()[iform];
            for(size_t ichild = 0; ichild < pf->getTechnique_common()->getChildren().getCount(); ++ichild) {
                daeElementRef pelt = pf->getTechnique_common()->getChildren()[ichild];
                RAVELOG_WARNA("collada formulas not supported\n");
            }
        }

        //pkinbody->_vForcedAdjacentLinks.push_back(entry);
        return true;
    }

    KinBody::Link* ExtractLink(KinBody* pkinbody, const domLinkRef pdomlink, const domNodeRef pdomnode, Transform tParentLink, const vector<domJointRef>& vdomjoints, const vector<KINEMATICSBINDING>& vbindings)
    {
        KinBody::Link* plink = new KinBody::Link(pkinbody);
        plink->name = _ravembstowcs(pdomlink->getName());
        plink->bStatic = false;
        plink->index = (int)pkinbody->_veclinks.size();
        pkinbody->_veclinks.push_back(plink);
        RAVELOG_VERBOSEA("adding link %S to kinbody %S\n", plink->GetName(), pkinbody->GetName());
        
        Transform tlink = getFullTransform(pdomlink);
        plink->_t = tParentLink * tlink;  // use the kinematics coordinate system for each link

        // get the geometry
        for(size_t igeom = 0; igeom < pdomnode->getInstance_geometry_array().getCount(); ++igeom) {
            domInstance_geometryRef domigeom = pdomnode->getInstance_geometry_array()[igeom];
            domGeometryRef domgeom = daeSafeCast<domGeometry>(domigeom->getUrl().getElement());
            if( !domgeom ) {
                RAVELOG_WARNA("link %S does not have valid geometry\n", plink->GetName());
                continue;
            }

            map<string,domMaterialRef> mapmaterials;
            if( !!domigeom->getBind_material() && !!domigeom->getBind_material()->getTechnique_common() ) {
                const domInstance_material_Array& matarray = domigeom->getBind_material()->getTechnique_common()->getInstance_material_array();
                for(size_t imat = 0; imat < matarray.getCount(); ++imat) {
                    domMaterialRef pmat = daeSafeCast<domMaterial>(matarray[imat]->getTarget().getElement());
                    if( !!pmat )
                        mapmaterials[matarray[imat]->getSymbol()] = pmat;
                }
            }
            ExtractGeometry(domgeom, mapmaterials, plink);
        }

        TransformMatrix tmnodegeom = (TransformMatrix)plink->_t.inverse() * getNodeParentTransform(pdomnode) * getFullTransform(pdomnode);
        Transform tnodegeom; Vector vscale;
        decompose(tmnodegeom, tnodegeom, vscale);

        FOREACHC(itgeom, plink->_listGeomProperties) {
            itgeom->_t = tnodegeom;
            switch(itgeom->GetType()) {
            case KinBody::Link::GEOMPROPERTIES::GeomBox:
                itgeom->vGeomData *= vscale;
                break;
            case KinBody::Link::GEOMPROPERTIES::GeomSphere:
                itgeom->vGeomData *= max(vscale.z,max(vscale.x,vscale.y));
                break;
            case KinBody::Link::GEOMPROPERTIES::GeomCylinder:
                itgeom->vGeomData.x *= max(vscale.x,vscale.y);
                itgeom->vGeomData.y *= vscale.z;
                break;
            case KinBody::Link::GEOMPROPERTIES::GeomTrimesh:
                itgeom->collisionmesh.ApplyTransform(tmnodegeom);
                itgeom->_t = Transform(); // reset back to identity
                break;
            default:
                RAVELOG_WARNA("unknown geometry type: %d\n", itgeom->GetType());
            }

            KinBody::Link::TRIMESH trimesh = itgeom->GetCollisionMesh();
            trimesh.ApplyTransform(itgeom->_t);
            plink->collision.Append(trimesh);
        }

        for(size_t iatt = 0; iatt < pdomlink->getAttachment_full_array().getCount(); ++iatt) {
            domLink::domAttachment_fullRef pattfull = pdomlink->getAttachment_full_array()[iatt];

            // get link kinematics transformation
            TransformMatrix tatt = getFullTransform(pattfull);

            // process attached links
            daeElement* peltjoint = daeSidRef(pattfull->getJoint(), pattfull).resolve().elt;
            domJointRef pdomjoint = daeSafeCast<domJoint>(peltjoint);

            if( !pdomjoint ) {
                domInstance_jointRef pdomijoint = daeSafeCast<domInstance_joint>(peltjoint);
                if( !!pdomijoint )
                    pdomjoint = daeSafeCast<domJoint>(pdomijoint->getUrl().getElement().cast());
            }
   
            if( !pdomjoint || pdomjoint->typeID() != domJoint::ID() ) {
                RAVELOG_WARNA("could not find attached joint %s!\n", pattfull->getJoint());
                return false;
            }

            // get direct child link
            if( !pattfull->getLink() ) {
                RAVELOG_WARNA("joint %s needs to be attached to a valid link\n", pdomjoint->getID());
                continue;
            }

            // find the correct node in vbindings
            daeTArray<domAxis_constraintRef> vdomaxes = pdomjoint->getChildrenByType<domAxis_constraint>();
            domNodeRef pchildnode = NULL;
            daeElementRef paxisnode = NULL;
            for(size_t ibind = 0; ibind < vbindings.size() && !pchildnode && !paxisnode; ++ibind) {
                domAxis_constraintRef paxisfound = NULL;
                for(size_t ic = 0; ic < vdomaxes.getCount(); ++ic) {
                    if( vdomaxes[ic] == vbindings[ibind].pkinematicaxis ) {
                        pchildnode = vbindings[ibind].pvisualnode;
                        paxisnode = vbindings[ibind].pvisualtrans;
                        break;
                    }
                }
            }

            if( !pchildnode ) {
                RAVELOG_INFOA("failed to find associating node for joint %s\n", pdomjoint->getID());
                continue;
            }

            // create the joints before creating the child links
            dReal fJointWeight = 1;
            vector<KinBody::Joint*> vjoints(vdomaxes.getCount());
            for(size_t ic = 0; ic < vdomaxes.getCount(); ++ic) {
                KinBody::Joint* pjoint = new KinBody::Joint(pkinbody);
                pjoint->bodies[0] = plink;
                pjoint->bodies[1] = NULL;
                pjoint->name = _ravembstowcs(pdomjoint->getName());
                pjoint->jointindex = (int)pkinbody->_vecjoints.size();
                pjoint->dofindex = (int)pkinbody->_vecJointWeights.size();
                pkinbody->_vecJointIndices.push_back((int)pkinbody->_vecJointWeights.size());
                pkinbody->_vecjoints.push_back(pjoint);
                pkinbody->_vecJointWeights.push_back(fJointWeight);
                vjoints[ic] = pjoint;
            }

            KinBody::Link* pchildlink = ExtractLink(pkinbody, pattfull->getLink(), pchildnode, plink->_t*tatt, vdomjoints, vbindings);
            if( pchildlink == NULL )
                continue;

            int numjoints = 0;
            for(size_t ic = 0; ic < vdomaxes.getCount(); ++ic) {
                domAxis_constraintRef pdomaxis = vdomaxes[ic];

                if( pchildlink == NULL ) {
                    // create dummy child link
                    // multiple axes can be easily done with "empty links"
                    RAVELOG_WARNA("openrave does not support collada joints with > 1 degrees\n");

                    wstringstream ss; ss << plink->name;
                    ss << L"_dummy" << numjoints;
                    pchildlink = new KinBody::Link(pkinbody);
                    pchildlink->name = ss.str().c_str();
                    pchildlink->bStatic = false;
                    pchildlink->index = (int)pkinbody->_veclinks.size();
                    pkinbody->_veclinks.push_back(pchildlink);
                }                

                KinBody::Joint* pjoint = vjoints[ic];
                pjoint->bodies[1] = pchildlink;

                // forward kinematics
                if( strcmp(pdomaxis->getElementName(), "revolute") == 0 )
                    pjoint->type = KinBody::Joint::JointRevolute;
                else if( strcmp(pdomaxis->getElementName(), "prismatic") == 0 )
                    pjoint->type = KinBody::Joint::JointPrismatic;

                pjoint->vAxes[0] = Vector(pdomaxis->getAxis()->getValue()[0], pdomaxis->getAxis()->getValue()[1], pdomaxis->getAxis()->getValue()[2]).normalize3();
                pjoint->vanchor = Vector(0,0,0);
                
                int numbad = 0;
                pjoint->offset = 0; // to overcome -pi to pi boundary
                for(int i = 0; i < pjoint->GetDOF(); ++i) {
                    if( !pdomaxis->getLimits() ) {
                        
                        if( pjoint->type == KinBody::Joint::JointRevolute ) {
                            pjoint->_vlowerlimit.push_back(-PI);
                            pjoint->_vupperlimit.push_back(PI);
                        }
                        else {
                            pjoint->_vlowerlimit.push_back(-100000);
                            pjoint->_vupperlimit.push_back(100000);
                        }
                    }
                    else {
                        dReal fscale = (pjoint->type == KinBody::Joint::JointRevolute)?(PI/180.0f):1.0f;
                        pjoint->_vlowerlimit.push_back(pdomaxis->getLimits()->getMin()->getValue()*fscale);
                        pjoint->_vupperlimit.push_back(pdomaxis->getLimits()->getMax()->getValue()*fscale);

                        if( pjoint->type == KinBody::Joint::JointRevolute ) {
                            if( pjoint->_vlowerlimit.back() < -PI || pjoint->_vupperlimit.back() > PI ) {
                                pjoint->offset += 0.5f * (pjoint->_vlowerlimit[i] + pjoint->_vupperlimit[i]);
                                ++numbad;
                            }
                        }
                    }
                }
                if( numbad > 0 ) {
                    pjoint->offset *= 1.0f / (dReal)numbad;
                    RAVELOG_VERBOSEA("joint %S offset is %f\n", pjoint->GetName(), (float)pjoint->offset);
                }
                
                // get node transforms before and after references axis
//                TransformMatrix tchildnodeleft, tchildnoderight;
//                if( !!pchildnode ) {
//                    tchildnodeleft = pchildlink->_t.inverse() * getNodeParentTransform(pchildnode);
//                    size_t ic = 0;
//                    for(; ic < pchildnode->getContents().getCount(); ++ic) {
//                        if( pchildnode->getContents()[ic].cast() == paxisnode.cast() )
//                            break;
//                        tchildnodeleft = tchildnodeleft * getTransform(pchildnode->getContents()[ic]);
//                    }
//
//                    ic++; // skip current node
//                    for(;ic < pchildnode->getContents().getCount(); ++ic) {
//                        tchildnoderight = tchildnoderight * getTransform(pchildnode->getContents()[ic]);
//                    }
//                }

//                FOREACH(itgeom, pchildlink->_listGeomProperties) {
//                    itgeom->_t = tchildnoderight.inverse() * itgeom->_t;
//                }

                // get kinematics transforms
                pjoint->tLeft = tatt;// * tchildnodeleft;
                pjoint->tRight = getFullTransform(pattfull->getLink());//*tchildnoderight;
                
                if( pjoint->type == KinBody::Joint::JointRevolute ) {
                    pjoint->tLeft = pjoint->tLeft * Transform().rotfromaxisangle(pjoint->vAxes[0], -pjoint->offset);
                }

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
            
                pchildlink = NULL;
                ++numjoints;
            }
        }

        //pdomlink->getAttachment_start_array();
        //pdomlink->getAttachment_end_array();

        return plink;
    }

    void FillGeometryColor(const domMaterialRef pmat, KinBody::Link::GEOMPROPERTIES& geom)
    {
        if( !!pmat && !!pmat->getInstance_effect() ) {
            domEffectRef peffect = daeSafeCast<domEffect>(pmat->getInstance_effect()->getUrl().getElement().cast());
            if( !!peffect ) {
                domProfile_common::domTechnique::domPhongRef pphong = daeSafeCast<domProfile_common::domTechnique::domPhong>(peffect->getDescendant(daeElement::matchType(domProfile_common::domTechnique::domPhong::ID())));
                if( !!pphong ) {
                    if( !!pphong->getAmbient() && !!pphong->getAmbient()->getColor() )
                        geom.ambientColor = getVector4(pphong->getAmbient()->getColor()->getValue());
                    if( !!pphong->getDiffuse() && !!pphong->getDiffuse()->getColor() )
                        geom.diffuseColor = getVector4(pphong->getDiffuse()->getColor()->getValue());
                }
            }
        }
    }

    bool ExtractGeometry(const domTrianglesRef triRef, const domVerticesRef vertsRef, const map<string,domMaterialRef>& mapmaterials, KinBody::Link* plink)
    {
        plink->_listGeomProperties.push_back(KinBody::Link::GEOMPROPERTIES());
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

        for (unsigned int w=0;w<triRef->getInput_array().getCount();w++) {
            int offset = triRef->getInput_array()[w]->getOffset();
            daeString str = triRef->getInput_array()[w]->getSemantic();
            if (!strcmp(str,"VERTEX")) {
                indexOffsetRef = triRef->getInput_array()[w];
                vertexoffset = offset;
            }
            if (offset > triangleIndexStride) {
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
                if( !node )
                    continue;
                dReal fUnitScale = GetUnitScale(node);
                const domFloat_arrayRef flArray = node->getFloat_array();
                if (!!flArray) {
                    const domList_of_floats& listFloats = flArray->getValue();
                    int k=vertexoffset;
                    int vertexStride = 3;//instead of hardcoded stride, should use the 'accessor'

                    if( trimesh.indices.capacity() < trimesh.indices.size()+triRef->getCount() )
                        trimesh.indices.reserve(trimesh.indices.size()+triRef->getCount());
                    if( trimesh.vertices.capacity() < trimesh.vertices.size()+triRef->getCount() )
                        trimesh.vertices.reserve(trimesh.vertices.size()+triRef->getCount());
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

    bool ExtractGeometry(const domTrifansRef triRef, const domVerticesRef vertsRef, const map<string,domMaterialRef>& mapmaterials, KinBody::Link* plink)
    {
        plink->_listGeomProperties.push_back(KinBody::Link::GEOMPROPERTIES());
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

        for (unsigned int w=0;w<triRef->getInput_array().getCount();w++) {
            int offset = triRef->getInput_array()[w]->getOffset();
            daeString str = triRef->getInput_array()[w]->getSemantic();
            if (!strcmp(str,"VERTEX")) {
                indexOffsetRef = triRef->getInput_array()[w];
                vertexoffset = offset;
            }
            if (offset > triangleIndexStride) {
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
                    if( !node )
                        continue;
                    dReal fUnitScale = GetUnitScale(node);
                    const domFloat_arrayRef flArray = node->getFloat_array();
                    if (!!flArray) {
                        const domList_of_floats& listFloats = flArray->getValue();
                        int k=vertexoffset;
                        int vertexStride = 3;//instead of hardcoded stride, should use the 'accessor'

                        if( trimesh.indices.capacity() < trimesh.indices.size()+triRef->getCount() )
                            trimesh.indices.reserve(trimesh.indices.size()+triRef->getCount());
                        if( trimesh.vertices.capacity() < trimesh.vertices.size()+triRef->getCount() )
                            trimesh.vertices.reserve(trimesh.vertices.size()+triRef->getCount());

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

    bool ExtractGeometry(const domTristripsRef triRef, const domVerticesRef vertsRef, const map<string,domMaterialRef>& mapmaterials, KinBody::Link* plink)
    {
        plink->_listGeomProperties.push_back(KinBody::Link::GEOMPROPERTIES());
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

        for (unsigned int w=0;w<triRef->getInput_array().getCount();w++) {
            int offset = triRef->getInput_array()[w]->getOffset();
            daeString str = triRef->getInput_array()[w]->getSemantic();
            if (!strcmp(str,"VERTEX")) {
                indexOffsetRef = triRef->getInput_array()[w];
                vertexoffset = offset;
            }
            if (offset > triangleIndexStride) {
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
                    if( !node )
                        continue;
                    dReal fUnitScale = GetUnitScale(node);
                    const domFloat_arrayRef flArray = node->getFloat_array();
                    if (!!flArray) {
                        const domList_of_floats& listFloats = flArray->getValue();
                        int k=vertexoffset;
                        int vertexStride = 3;//instead of hardcoded stride, should use the 'accessor'

                        if( trimesh.indices.capacity() < trimesh.indices.size()+triRef->getCount() )
                            trimesh.indices.reserve(trimesh.indices.size()+triRef->getCount());
                        if( trimesh.vertices.capacity() < trimesh.vertices.size()+triRef->getCount() )
                            trimesh.vertices.reserve(trimesh.vertices.size()+triRef->getCount());

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

    bool ExtractGeometry(const domGeometryRef geom, const map<string,domMaterialRef>& mapmaterials, KinBody::Link* plink)
    {
        vector<Vector> vconvexhull;
        if (!!geom && geom->getMesh()) {
            const domMeshRef meshRef = geom->getMesh();

            for (size_t tg = 0;tg<meshRef->getTriangles_array().getCount();tg++) {
                ExtractGeometry(meshRef->getTriangles_array()[tg], meshRef->getVertices(), mapmaterials, plink);
            }
            for (size_t tg = 0;tg<meshRef->getTrifans_array().getCount();tg++) {
                ExtractGeometry(meshRef->getTrifans_array()[tg], meshRef->getVertices(), mapmaterials, plink);
            }
            for (size_t tg = 0;tg<meshRef->getTristrips_array().getCount();tg++) {
                ExtractGeometry(meshRef->getTristrips_array()[tg], meshRef->getVertices(), mapmaterials, plink);
            }
            if( meshRef->getPolylist_array().getCount() > 0 )
                RAVELOG_WARNA("openrave does not support collada polylists\n");
            if( meshRef->getPolygons_array().getCount() > 0 )
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

        if (!!geom && geom->getConvex_mesh()) {
            {
                const domConvex_meshRef convexRef = geom->getConvex_mesh();
                daeElementRef otherElemRef = convexRef->getConvex_hull_of().getElement();
                if ( otherElemRef != NULL ) {
                    domGeometryRef linkedGeom = *(domGeometryRef*)&otherElemRef;
                    printf( "otherLinked\n");
                }
                else {
                    printf("convexMesh polyCount = %d\n",(int)convexRef->getPolygons_array().getCount());
                    printf("convexMesh triCount = %d\n",(int)convexRef->getTriangles_array().getCount());
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

                    for (size_t  i = 0; i < libgeom->getGeometry_array().getCount(); i++) {
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
                                        if( !node )
                                            continue;
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
                        if( !node )
                            continue;
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

            if( vconvexhull.size() > 0 ) {
                plink->_listGeomProperties.push_back(KinBody::Link::GEOMPROPERTIES());
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

    template <typename T, typename U>
    daeSmartRef<T> resolve(domCommon_sidref_or_paramRef paddr, const U& paramArray)
    {
        daeElement* pref = paddr;
        domSidref sidref = NULL;

        if( paddr->getSIDREF() != NULL )
            sidref = paddr->getSIDREF()->getValue();
        else if( paddr->getParam() != NULL ) { // parameter of kinematics
            // search children parameters of kscene whose sid's match
            for(size_t iikm = 0; iikm < paramArray.getCount(); ++iikm) {
                for(size_t iparam = 0; iparam < paramArray[iikm]->getNewparam_array().getCount(); ++iparam) {
                    // take the SIDREF of those parameters to get the real kinematics model
                    if( strcmp(paramArray[iikm]->getNewparam_array()[iparam]->getSid(), paddr->getParam()->getValue()) == 0 ) {
                        if( paramArray[iikm]->getNewparam_array()[iparam]->getSIDREF() != NULL ) {
                            sidref = paramArray[iikm]->getNewparam_array()[iparam]->getSIDREF()->getValue();
                            pref = paramArray[iikm]->getNewparam_array()[iparam];
                            break;
                        }
                    }
                }

                if( sidref != NULL )
                    break;
            }
        }

        if( sidref == NULL ) {
            RAVELOG_WARNA("failed to find instance kinematics sidref\n");
            return NULL;
        }

        // SID path to kinematics
        daeSmartRef<T> ptarget = daeSafeCast<T>(daeSidRef(sidref, pref).resolve().elt);
        if( ptarget == NULL ) {
            RAVELOG_WARNA("failed to resolve %s from %s\n", sidref, paddr->getID());
            return NULL;
        }

        if( ptarget->typeID() != T::ID() ) {
            RAVELOG_WARNA("unexpected resolved type (%s) \n", ptarget->getTypeName());
            return NULL;
        }

        return ptarget;
    }

    template <typename U>
    domFloat resolveFloat(domCommon_float_or_paramRef paddr, const U& paramArray)
    {
        if( paddr->getFloat() != NULL )
            return paddr->getFloat()->getValue();

        if( paddr->getParam() == NULL ) {
            RAVELOG_WARNA("joint value not specified, setting to 0\n");
            return 0;
        }

        for(size_t iikm = 0; iikm < paramArray.getCount(); ++iikm) {
            for(size_t iparam = 0; iparam < paramArray[iikm]->getNewparam_array().getCount(); ++iparam) {
                domKinematics_newparamRef pnewparam = paramArray[iikm]->getNewparam_array()[iparam];
                if( strcmp(pnewparam->getSid(), paddr->getParam()->getValue()) == 0 ) {
                    if( pnewparam->getFloat() != NULL )
                        return pnewparam->getFloat()->getValue();
                    else if( pnewparam->getSIDREF() != NULL ) {
                        domKinematics_newparam::domFloatRef ptarget = daeSafeCast<domKinematics_newparam::domFloat>(daeSidRef(pnewparam->getSIDREF()->getValue(), pnewparam).resolve().elt);
                        if( ptarget == NULL ) {
                            RAVELOG_WARNA("failed to resolve %s from %s\n", pnewparam->getSIDREF()->getValue(), paddr->getID());
                            continue;
                        }

                        if( ptarget->getElementType() != COLLADA_TYPE::FLOAT ) {
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

    TransformMatrix getTransform(daeElementRef pelt)
    {
        TransformMatrix t;
        domRotateRef protate = daeSafeCast<domRotate>(pelt);
        if( !!protate ) {            
            t.rotfromaxisangle(Vector(protate->getValue()[0],protate->getValue()[1],protate->getValue()[2]), (dReal)(protate->getValue()[3]*(PI/180.0)));
            return t;
        }

        domTranslateRef ptrans = daeSafeCast<domTranslate>(pelt);
        if( !!ptrans ) {
            t.trans = Vector(ptrans->getValue()[0], ptrans->getValue()[1], ptrans->getValue()[2]);
            t.trans *= GetUnitScale(pelt);
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
            
            Vector dir = -(lookat - campos);
            dReal len = RaveSqrt(dir.lengthsqr3());

            if( len > 1e-6 )
                dir *= 1/len;
            else
                dir = Vector(0,0,1);

            Vector up = camup - dir * dot3(dir,camup);
            len = up.lengthsqr3();
            if( len < 1e-8 ) {
                up = Vector(0,1,0);
                up -= dir * dot3(dir,up);
                len = up.lengthsqr3();
                if( len < 1e-8 ) {
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
        if( !!pskew ) {
            RAVELOG_ERRORA("skew transform not implemented\n");
        }

        return t;
    }

    template <typename T>
    TransformMatrix getNodeParentTransform(const T pelt)
    {
        domNodeRef pnode = daeSafeCast<domNode>(pelt->getParent());
        if( !pnode )
            return TransformMatrix();
        return getNodeParentTransform(pnode) * getFullTransform(pnode);
    }

    template <typename T>
    TransformMatrix getFullTransform(const T pelt)
    {
        TransformMatrix t;
        for(size_t i = 0; i < pelt->getContents().getCount(); ++i)
            t = t * getTransform(pelt->getContents()[i]);
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

    virtual void handleError( daeString msg ) {
        RAVELOG_ERRORA("COLLADA error: %s\n", msg);
    }

	virtual void handleWarning( daeString msg ) {
        RAVELOG_WARNA("COLLADA warning: %s\n", msg);
    }

    inline dReal GetUnitScale(daeElement* pelt) {
        return ((USERDATA*)pelt->getUserData())->scale;
    }

private:

    bool computeConvexHull(const vector<Vector>& verts, KinBody::Link::TRIMESH& trimesh)
    {
        RAVELOG_ERRORA("convex hulls not supported\n");
        return false;
//        if( verts.size() <= 3 )
//            return false;
//
//        int dim = 3;  	              // dimension of points
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
//            facetT *facet;	          // set by FORALLfacets 
//            FORALLfacets { // 'qh facet_list' contains the convex hull
//                vconvexplanes.push_back(Vector(facet->normal[0], facet->normal[1], facet->normal[2], facet->offset));
//            }
//
//            bSuccess = true;
//        }
//
//        qh_freeqhull(!qh_ALL);
//        int curlong, totlong;	  // memory remaining after qh_memfreeshort 
//        qh_memfreeshort (&curlong, &totlong);
//        if (curlong || totlong)
//            ROS_ERROR("qhull internal warning (main): did not free %d bytes of long memory (%d pieces)", totlong, curlong);
//     
//        fclose(errfile);
//        return bSuccess;
    }

    boost::shared_ptr<DAE> _collada;
    domCOLLADA* _dom;
    EnvironmentBase* _penv;
    vector<USERDATA> _vuserdata; // all userdata
};

bool RaveParseColladaFile(EnvironmentBase* penv, const char* filename)
{
    ColladaReader reader(penv);
    if( !reader.Init(filename) )
        return false;
    return reader.Extract(penv);
}

bool RaveParseColladaFile(EnvironmentBase* penv, KinBody** pbody, const char* filename)
{
    ColladaReader reader(penv);
    if( !reader.Init(filename) )
        return false;
    return reader.Extract(pbody);
}

bool RaveParseColladaFile(EnvironmentBase* penv, RobotBase** probot, const char* filename)
{
    ColladaReader reader(penv);
    if( !reader.Init(filename) )
        return false;
    return reader.Extract(probot);
}

bool RaveParseColladaData(EnvironmentBase* penv, const char* pdata, int len)
{
    ColladaReader reader(penv);
    if( !reader.Init(pdata, len) )
        return false;
    return reader.Extract(penv);
}

bool RaveParseColladaData(EnvironmentBase* penv, KinBody** pbody, const char* pdata, int len)
{
    ColladaReader reader(penv);
    if( !reader.Init(pdata, len) )
        return false;
    return reader.Extract(pbody);
}

bool RaveParseColladaData(EnvironmentBase* penv, RobotBase** probot, const char* pdata, int len)
{
    ColladaReader reader(penv);
    if( !reader.Init(pdata, len) )
        return false;
    return reader.Extract(probot);
}

#else

bool RaveParseColladaFile(EnvironmentBase* penv, const char* filename)
{
    RAVELOG_ERRORA("collada files not supported\n");
    return false;
}

bool RaveParseColladaFile(EnvironmentBase* penv, KinBody** pbody, const char* filename)
{
    RAVELOG_ERRORA("collada files not supported\n");
    return false;
}

bool RaveParseColladaFile(EnvironmentBase* penv, RobotBase** probot, const char* filename)
{
    RAVELOG_ERRORA("collada files not supported\n");
    return false;
}

bool RaveParseColladaData(EnvironmentBase* penv, const char* pdata, int len)
{
    RAVELOG_ERRORA("collada files not supported\n");
    return false;
}

bool RaveParseColladaData(EnvironmentBase* penv, KinBody** pbody, const char* pdata, int len)
{
    RAVELOG_ERRORA("collada files not supported\n");
    return false;
}

bool RaveParseColladaData(EnvironmentBase* penv, RobotBase** probot, const char* pdata, int len)
{
    RAVELOG_ERRORA("collada files not supported\n");
    return false;
}

#endif
