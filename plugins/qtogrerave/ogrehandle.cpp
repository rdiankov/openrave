#include "ogrehandle.h"

#include <OGRE/Hlms/Pbs/OgreHlmsPbs.h>
#include <OGRE/Hlms/Pbs/OgreHlmsPbsDatablock.h>
#include <OGRE/OgreItem.h>
#include <OGRE/OgreMesh2.h>
#include <OGRE/OgreMeshManager2.h>
#include <OGRE/OgreSubMesh2.h>

namespace qtogrerave {

OgreNodeHandle::OgreNodeHandle(Ogre::Root *root, Ogre::SceneNode *parentNode, OpenRAVE::KinBody &body)
{
    _root = root;
    _node = parentNode->createChildSceneNode();
    _node->setName(body.GetName());

    // if (!body.IsEnabled()) {
    //     return;
    // }

    const OpenRAVE::Transform &transfBody = body.GetTransform();
    SetOgreNodeTransform(_node, transfBody);

    const OpenRAVE::Transform invTransfBody = transfBody.inverse();

    Ogre::HlmsManager *hlmsManager = root->getHlmsManager();
    Ogre::HlmsPbs *hlmsPbs = static_cast<Ogre::HlmsPbs*>( hlmsManager->getHlms(Ogre::HLMS_PBS) );

    for (const OpenRAVE::KinBody::LinkPtr &pLink: body.GetLinks()) {
        Ogre::SceneNode *linkNode = _node->createChildSceneNode();
        linkNode->setName(pLink->GetName());
        SetOgreNodeTransform(linkNode, invTransfBody * pLink->GetTransform());

        for (const OpenRAVE::KinBody::Link::GeometryPtr &pGeom: pLink->GetGeometries()) {
            Ogre::SceneNode *geomNode = linkNode->createChildSceneNode();

            std::string geomName = pGeom->GetName();

            geomNode->setName(body.GetName() + ":" + pLink->GetName() + ":" + geomName);

            SetOgreNodeTransform(geomNode, pGeom->GetTransform());

            Ogre::String datablockName = body.GetName() + ":" + pLink->GetName() + ":" +
                (geomName.empty() ? std::to_string(reinterpret_cast<uintptr_t>(pGeom.get())) :
                                    geomName);

            // TODO: Delete this datablock!!!!!!!!!!!!!!!!!!!!!!!!!!!
            geomNode->setName(datablockName);
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
            const float transparency = pGeom->GetTransparency();
            if (transparency > 0.0f) {
                datablock->setTransparency(1.0f - transparency);
            }

            // TODO: Set datablock
            switch(pGeom->GetType()) {
            //  Geometry is defined like a Sphere
            case OpenRAVE::GT_Sphere: {
                // TODO: Check if the sphere is an unit sphere
                // TODO: Ditch v1
                Ogre::v1::Entity* sphere = geomNode->getCreator()->createEntity(Ogre::SceneManager::PT_SPHERE);
                const float radius = pGeom->GetSphereRadius();
                sphere->setDatablock(datablock);
                geomNode->setScale(Ogre::Vector3(radius, radius, radius) / 50.0f);
                geomNode->attachObject(sphere);
                break;
            }
            //  Geometry is defined like a Box
            case OpenRAVE::GT_Box: {
                // TODO: Ditch v1
                Ogre::v1::Entity* box = geomNode->getCreator()->createEntity(Ogre::SceneManager::PT_CUBE);
                const OpenRAVE::Vector &extents = pGeom->GetBoxExtents();
                box->setDatablock(datablock);
                // Yes, magic number. See OgrePrefabFactory.cpp
                geomNode->setScale(Ogre::Vector3(extents.x, extents.y, extents.z) / 50.0f); // <--------- is this extents?
                geomNode->attachObject(box);
                break;
            }
            //  Geometry is defined like a Cylinder
            case OpenRAVE::GT_Cylinder:
            case OpenRAVE::GT_Container:
            case OpenRAVE::GT_TriMesh: {
                pGeom->InitCollisionMesh();
                const OpenRAVE::TriMesh& oremesh = pGeom->GetCollisionMesh();
                Ogre::RenderSystem *renderSystem = root->getRenderSystem();
                Ogre::VaoManager *vaoManager = renderSystem->getVaoManager();
                Ogre::Vector3 min, max;
                const size_t nPoints = oremesh.vertices.size();
                const size_t nIndices = oremesh.indices.size();

                if (nPoints == 0 || nIndices == 0) {
                    continue;
                }

                // No need to try/except here because the indices are managed by OpenRAVE
                Ogre::IndexBufferPacked *indexBuffer = vaoManager->createIndexBuffer(Ogre::IndexBufferPacked::IT_32BIT,
                                                                                     nIndices, Ogre::BT_IMMUTABLE, // TODO: Really immutable?
                                                                                     (void*) oremesh.indices.data(),
                                                                                     false); // OpenRAVE is resposnsible for managing the data
                Ogre::VertexBufferPacked* packedBuffer = CreateBufferPacked(vaoManager, oremesh.vertices, oremesh.indices, min, max);

                Ogre::VertexArrayObject* vao = vaoManager->createVertexArrayObject(
                    {packedBuffer},
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
    if (_node) {
        // TODO: What about _node's children?
        _node->getParentSceneNode()->removeAndDestroyChild(_node);
    }
    Ogre::HlmsManager *hlmsManager = _root->getHlmsManager();
    Ogre::HlmsPbs *hlmsPbs = static_cast<Ogre::HlmsPbs*>( hlmsManager->getHlms(Ogre::HLMS_PBS) );

    for (const Ogre::String &materialName: _materialNames) {
        hlmsPbs->destroyDatablock(materialName);
    }

    // TODO: How to delete the vertex buffer?
}

void OgreNodeHandle::Update(const OpenRAVE::KinBody &body)
{
    // TODO: Use dirty bit to track geometry update?

    const OpenRAVE::Transform &transfBody = body.GetTransform();
    SetOgreNodeTransform(_node, transfBody);
    const OpenRAVE::Transform invTransfBody = transfBody.inverse();

    if (body.GetLinks().size() != _node->numChildren()) {
        RAVELOG_WARN("TODO: Update links");
    }

    Ogre::Node::NodeVecIterator itlinknode = _node->getChildIterator();
    for (const OpenRAVE::KinBody::LinkPtr &pLink: body.GetLinks()) {
        // Assume the OpenRAVE / Ogre nodes are in the same order
        Ogre::Node* linkNode = itlinknode.getNext();
        SetOgreNodeTransform(linkNode, invTransfBody * pLink->GetTransform());

        if (pLink->GetGeometries().size() != linkNode->numChildren()) {
            RAVELOG_WARN("TODO: Update links");
        }
        Ogre::Node::NodeVecIterator itgeomnode = linkNode->getChildIterator();
        // Assume the OpenRAVE / Ogre nodes are in the same order
        for (const OpenRAVE::KinBody::Link::GeometryPtr &pGeom: pLink->GetGeometries()) {
            Ogre::Node* geomNode = itgeomnode.getNext();
            SetOgreNodeTransform(geomNode, pGeom->GetTransform());
        }
    }
}

} // namespace qtogrerave