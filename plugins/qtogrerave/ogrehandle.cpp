#include "ogrehandle.h"

#include <OGRE/Hlms/Pbs/OgreHlmsPbs.h>
#include <OGRE/Hlms/Pbs/OgreHlmsPbsDatablock.h>
#include <OGRE/OgreItem.h>
#include <OGRE/OgreMesh2.h>
#include <OGRE/OgreMeshManager2.h>
#include <OGRE/OgreSubMesh2.h>

namespace qtogrerave {

OgreNodeHandle::OgreNodeHandle(Ogre::Root *root, Ogre::SceneNode *parentNode, OpenRAVE::KinBodyPtr pbody)
{
    _root = root;
    _rootNode = parentNode->createChildSceneNode();
    const OpenRAVE::Transform &transfBody = pbody->GetTransform();
    SetOgreNodeTransform(_rootNode, transfBody);

    const OpenRAVE::Transform invTransfBody = transfBody.inverse();

    Ogre::HlmsManager *hlmsManager = root->getHlmsManager();
    Ogre::HlmsPbs *hlmsPbs = static_cast<Ogre::HlmsPbs*>( hlmsManager->getHlms(Ogre::HLMS_PBS) );

    for (const OpenRAVE::KinBody::LinkPtr &pLink: pbody->GetLinks()) {
        Ogre::SceneNode *linkNode = _rootNode->createChildSceneNode();
        SetOgreNodeTransform(linkNode, invTransfBody * pLink->GetTransform());

        for (const OpenRAVE::KinBody::Link::GeometryPtr &pGeom: pLink->GetGeometries()) {
            Ogre::SceneNode *geomNode = linkNode->createChildSceneNode();
            SetOgreNodeTransform(geomNode, pGeom->GetTransform());

            // TODO: Delete this datablock!!!!!!!!!!!!!!!!!!!!!!!!!!!
            Ogre::String datablockName = pbody->GetName() + pLink->GetName() + pGeom->GetName() + std::to_string(std::time(nullptr));
            Ogre::HlmsPbsDatablock *datablock = static_cast<Ogre::HlmsPbsDatablock*>(hlmsPbs->getDatablock(datablockName));
            if (!datablock) {
                datablock = static_cast<Ogre::HlmsPbsDatablock*>(
                    hlmsPbs->createDatablock(datablockName, datablockName,
                                             Ogre::HlmsMacroblock(),
                                             Ogre::HlmsBlendblock(),
                                             Ogre::HlmsParamVec())
                );
            }
            _materialNames.push_back(datablockName);

            const OpenRAVE::RaveVector<float>& diffuse = pGeom->GetDiffuseColor();
            datablock->setDiffuse(Ogre::Vector3(diffuse.x, diffuse.y, diffuse.z));
            const OpenRAVE::RaveVector<float>& ambient = pGeom->GetAmbientColor();
            datablock->setEmissive(Ogre::Vector3(ambient.x, ambient.y, ambient.z));

            // TODO: Set datablock
            switch(pGeom->GetType()) {
            //  Geometry is defined like a Sphere
            case OpenRAVE::GT_Sphere: {
                // TODO: Check if the sphere is an unit sphere
                // TODO: Ditch v1
                Ogre::v1::Entity* sphere = geomNode->getCreator()->createEntity(Ogre::SceneManager::PT_SPHERE);
                const float radius = pGeom->GetSphereRadius();
                sphere->setDatablock(datablock);
                geomNode->setScale(Ogre::Vector3(radius, radius, radius));
                geomNode->attachObject(sphere);
                break;
            }
            //  Geometry is defined like a Box
            case OpenRAVE::GT_Box: {
                // TODO: Ditch v1
                Ogre::v1::Entity* box = geomNode->getCreator()->createEntity(Ogre::SceneManager::PT_CUBE);
                const OpenRAVE::Vector &extents = pGeom->GetBoxExtents();
                box->setDatablock(datablock);
                geomNode->setScale(Ogre::Vector3(extents.x, extents.y, extents.z)); // <--------- is this extents?
                geomNode->attachObject(box);
                break;
            }
            //  Geometry is defined like a Cylinder
            case OpenRAVE::GT_Cylinder: {
                #if 0
                // make SoCylinder point towards z, not y
                osg::Cylinder* cy = new osg::Cylinder();
                cy->setRadius(orgeom->GetCylinderRadius());
                cy->setHeight(orgeom->GetCylinderHeight());
                osg::ref_ptr<osg::Geode> geode = new osg::Geode;
                osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(cy);
                geode->addDrawable(sd.get());
                pgeometrydata->addChild(geode.get());
                #endif
                break;
            }
            //  Extract geometry from collision Mesh
            case OpenRAVE::GT_Container:
            case OpenRAVE::GT_TriMesh: {
                const OpenRAVE::TriMesh& oremesh = pGeom->GetCollisionMesh();
                Ogre::RenderSystem *renderSystem = root->getRenderSystem();
                Ogre::VaoManager *vaoManager = renderSystem->getVaoManager();
                Ogre::Vector3 min, max;
                const size_t nPoints = oremesh.vertices.size();
                float* vpoints = FormatPoints(oremesh.vertices, min, max);
                Ogre::VertexBufferPacked* vertexBuffer = CreatePointsBuffer(vaoManager, nPoints, vpoints);

                // TODO: Use try except block
                Ogre::IndexBufferPacked *indexBuffer = vaoManager->createIndexBuffer(Ogre::IndexBufferPacked::IT_32BIT,
                                                                                     oremesh.indices.size(), Ogre::BT_IMMUTABLE, // TODO: Really immutable?
                                                                                     (void*) oremesh.indices.data(),
                                                                                     false); // OpenRAVE is resposnsible for managing the data

                Ogre::VertexArrayObject* vao = vaoManager->createVertexArrayObject(
                    {vertexBuffer},
                    indexBuffer, // Do not need index buffer
                    Ogre::OT_TRIANGLE_LIST);

                Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(datablockName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
                Ogre::SubMesh* submesh = mesh->createSubMesh();
                submesh->mVao[Ogre::VpNormal].push_back(vao);
                submesh->mVao[Ogre::VpShadow].push_back(vao);

                //Set the bounds to get frustum culling and LOD to work correctly
                Ogre::Aabb aabb = Ogre::Aabb::newFromExtents(min, max);
                mesh->_setBounds(aabb, false);
                mesh->_setBoundingSphereRadius(aabb.getRadius());
        
                Ogre::SceneManager *sceneManager = geomNode->getCreator();
                Ogre::Item *item = sceneManager->createItem(mesh);
                geomNode->attachObject(item);
                break;
            }
            default:
                break;
            }
        }

    }
}

OgreNodeHandle::~OgreNodeHandle() {
    if (_rootNode) {
        // TODO: Thow about the children?
        _rootNode->getParentSceneNode()->removeAndDestroyChild(_rootNode);
    }
    Ogre::HlmsManager *hlmsManager = _root->getHlmsManager();
    Ogre::HlmsPbs *hlmsPbs = static_cast<Ogre::HlmsPbs*>( hlmsManager->getHlms(Ogre::HLMS_PBS) );

    for (const Ogre::String &materialName: _materialNames) {
        hlmsPbs->destroyDatablock(materialName);
    }
}

} // namespace qtogrerave