#include "ogrehandle.h"

#include <OGRE/Hlms/Pbs/OgreHlmsPbs.h>
#include <OGRE/Hlms/Pbs/OgreHlmsPbsDatablock.h>

namespace qtogrerave {

OgreNodeHandle::OgreNodeHandle(Ogre::Root *root, Ogre::SceneNode *parentNode, OpenRAVE::KinBodyPtr pbody)
{
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
            Ogre::HlmsPbsDatablock *datablock = static_cast<Ogre::HlmsPbsDatablock*>(
                hlmsPbs->createDatablock(datablockName, datablockName,
                                         Ogre::HlmsMacroblock(),
                                         Ogre::HlmsBlendblock(),
                                         Ogre::HlmsParamVec()));

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
            	#if 0
                // make triangleMesh
                osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;

                //geom->setColorBinding(osg::Geometry::BIND_OVERALL); // need to call geom->setColorArray first

                const TriMesh& mesh = orgeom->GetCollisionMesh();
                osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
                vertices->reserveArray(mesh.vertices.size());
                for(size_t i = 0; i < mesh.vertices.size(); ++i) {
                    RaveVector<float> v = mesh.vertices[i];
                    vertices->push_back(osg::Vec3(v.x, v.y, v.z));
                }
                geom->setVertexArray(vertices.get());


                osg::DrawElementsUInt* geom_prim = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, mesh.indices.size());
                for(size_t i = 0; i < mesh.indices.size(); ++i) {
                    (*geom_prim)[i] = mesh.indices[i];
                }
                geom->addPrimitiveSet(geom_prim);

                osgUtil::SmoothingVisitor::smooth(*geom); // compute vertex normals
                osg::ref_ptr<osg::Geode> geode = new osg::Geode;
                geode->addDrawable(geom);	
                pgeometrydata->addChild(geode);
                #endif
                break;
            }
            default:
                break;
            }
		}

	}
}

} // namespace qtogrerave