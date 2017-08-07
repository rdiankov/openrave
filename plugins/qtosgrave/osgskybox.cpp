//
// Created by mujin on 6/15/17.
//

#include "osgskybox.h"

class MoveEarthySkyWithEyePointTransform : public osg::Transform {
public:
    /** Get the transformation matrix which moves from local coords to world coords.*/
    virtual bool computeLocalToWorldMatrix(osg::Matrix& matrix, osg::NodeVisitor* nv) const
    {
        osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(nv);
        if (cv) {
            osg::Vec3 eyePointLocal = cv->getEyeLocal();
            matrix.preMultTranslate(eyePointLocal);
        }
        return true;
    }

    /** Get the transformation matrix which moves from world coords to local coords.*/
    virtual bool computeWorldToLocalMatrix(osg::Matrix& matrix, osg::NodeVisitor* nv) const
    {
        osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(nv);
        if (cv) {
            osg::Vec3 eyePointLocal = cv->getEyeLocal();
            matrix.postMultTranslate(-eyePointLocal);
        }
        return true;
    }
};

struct TexMatCallback : public osg::NodeCallback {
public:

    TexMatCallback(osg::TexMat& tm)
            :
            _texMat(tm)
    {
    }

    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
    {
        osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(nv);
        if (cv) {
            const osg::Matrix& MV = *(cv->getModelViewMatrix());
            const osg::Matrix R = osg::Matrix::rotate(osg::DegreesToRadians(112.0f), 0.0f, 0.0f, 1.0f)*
                    osg::Matrix::rotate(osg::DegreesToRadians(90.0f), 1.0f, 0.0f, 0.0f);

            osg::Quat q = MV.getRotate();
            const osg::Matrix C = osg::Matrix::rotate(q.inverse());

            _texMat.setMatrix(C*R);
        }

        traverse(node, nv);
    }

    osg::TexMat& _texMat;
};

Skybox::Skybox()
{
    osg::StateSet* stateset = new osg::StateSet();

    osg::TexEnv* te = new osg::TexEnv;
    te->setMode(osg::TexEnv::REPLACE);
    stateset->setTextureAttributeAndModes(0, te, osg::StateAttribute::ON);

    osg::TexGen* tg = new osg::TexGen;
    tg->setMode(osg::TexGen::NORMAL_MAP);
    stateset->setTextureAttributeAndModes(0, tg, osg::StateAttribute::ON);

    osg::TexMat* tm = new osg::TexMat;
    stateset->setTextureAttribute(0, tm);

    stateset->setTextureAttributeAndModes(0, _skymap, osg::StateAttribute::ON);

    stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    stateset->setMode(GL_CULL_FACE, osg::StateAttribute::OFF);

    // clear the depth to the far plane.
    osg::Depth* depth = new osg::Depth;
    depth->setFunction(osg::Depth::ALWAYS);
    depth->setRange(1.0, 1.0);
    stateset->setAttributeAndModes(depth, osg::StateAttribute::ON);

    stateset->setRenderBinDetails(-1, "RenderBin");

    osg::Drawable* drawable = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0f, 0.0f, 0.0f), 1));

    osg::Geode* geode = new osg::Geode;
    geode->setCullingActive(false);
    geode->setStateSet(stateset);
    geode->addDrawable(drawable);

    osg::Transform* transform = new MoveEarthySkyWithEyePointTransform;
    transform->setCullingActive(false);
    transform->addChild(geode);

//  clearNode->setRequiresClear(false);
    this->setCullCallback(new TexMatCallback(*tm));
    this->addChild(transform);
}

void Skybox::setTextureCubeMap(const std::string& posx,
        const std::string& negx,
        const std::string& posy,
        const std::string& negy,
        const std::string& posz,
        const std::string& negz)
{
    if (_skymap == 0) {
        _skymap = new osg::TextureCubeMap;
    }

    osg::ref_ptr<osg::Image> imagePosX = osgDB::readRefImageFile(posx);
    osg::ref_ptr<osg::Image> imageNegX = osgDB::readRefImageFile(negx);
    osg::ref_ptr<osg::Image> imagePosY = osgDB::readRefImageFile(posy);
    osg::ref_ptr<osg::Image> imageNegY = osgDB::readRefImageFile(negy);
    osg::ref_ptr<osg::Image> imagePosZ = osgDB::readRefImageFile(posz);
    osg::ref_ptr<osg::Image> imageNegZ = osgDB::readRefImageFile(negz);

    if (imagePosX && imageNegX && imagePosY && imageNegY && imagePosZ && imageNegZ) {
        _skymap->setImage(osg::TextureCubeMap::POSITIVE_X, imagePosX);
        _skymap->setImage(osg::TextureCubeMap::NEGATIVE_X, imageNegX);
        _skymap->setImage(osg::TextureCubeMap::POSITIVE_Y, imagePosY);
        _skymap->setImage(osg::TextureCubeMap::NEGATIVE_Y, imageNegY);
        _skymap->setImage(osg::TextureCubeMap::POSITIVE_Z, imagePosZ);
        _skymap->setImage(osg::TextureCubeMap::NEGATIVE_Z, imageNegZ);

        _skymap->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
        _skymap->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
        _skymap->setWrap(osg::Texture::WRAP_R, osg::Texture::CLAMP_TO_EDGE);

        _skymap->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
        _skymap->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);

        osg::StateSet* stateSet = this->getOrCreateStateSet();
        stateSet->setTextureAttributeAndModes(0, _skymap, osg::StateAttribute::ON);
    }
}

void Skybox::show(bool visible)
{
    _bVisible = visible;
    this->setNodeMask(_bVisible ? 0x1 : 0x0);
}

Skybox::~Skybox()
{

}

