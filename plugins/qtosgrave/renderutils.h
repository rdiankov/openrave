
#ifndef OPENRAVE_QTOSG_RENDERUTILS_H
#define OPENRAVE_QTOSG_RENDERUTILS_H

#include <osg/Geode>
#include <osg/Group>
#include <osg/Camera>
#include <osg/Shader>
#include <osg/Geometry>
#include <osg/Texture2D>
#include <osg/TextureRectangle>
#include <osgDB/ReadFile>
#include <osg/PolygonMode>

class RenderUtils {
public:
    static void SetShaderProgramOnStateSet(osg::ref_ptr<osg::StateSet> onwerStateSet, const std::string& vertShaderString, const std::string& fragShaderString)
    {
        osg::ref_ptr<osg::Program> program = new osg::Program;
        osg::Shader* vertShader = new osg::Shader(osg::Shader::VERTEX, vertShaderString);
        program->addShader(vertShader);

        osg::Shader* fragShader = new osg::Shader(osg::Shader::FRAGMENT, fragShaderString);
        program->addShader(fragShader);

        onwerStateSet->setAttributeAndModes(
            program.get(),
            osg::StateAttribute::ON);
    }

    static osg::Texture2D *CreateTexture(const std::string &fileName)
    {
        osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;
        texture->setImage(osgDB::readRefImageFile(fileName));
        texture->setWrap(osg::Texture2D::WRAP_S, osg::Texture2D::REPEAT);
        texture->setWrap(osg::Texture2D::WRAP_T, osg::Texture2D::REPEAT);
        texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
        texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
        texture->setMaxAnisotropy(16.0f);
        return texture.release();
    }

    static osg::Texture2D *CreateFloatTextureRectangle(int width, int height)
    {
        osg::ref_ptr<osg::Texture2D> tex2D = new osg::Texture2D();
        //tex2D->setNumSamples(4);
        tex2D->setTextureSize(width, height);
        tex2D->setSourceFormat(GL_RGBA);
        tex2D->setInternalFormat(GL_RGBA32F_ARB);
        tex2D->setSourceType(GL_FLOAT);
        tex2D->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR);
        tex2D->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR);
        return tex2D.release();
    }

    static osg::Texture2D *CreateDepthFloatTextureRectangle(int width, int height)
    {
        osg::ref_ptr<osg::Texture2D> tex2D = new osg::Texture2D();
        //tex2D->setNumSamples(4);
        tex2D->setTextureSize(width, height);
        tex2D->setSourceFormat(GL_DEPTH_COMPONENT);
        tex2D->setInternalFormat(GL_DEPTH_COMPONENT24);
        tex2D->setSourceType(GL_UNSIGNED_INT);
        return tex2D.release();
    }

    static osg::Geode *CreateScreenQuad(float width, float height, float scale = 1, osg::Vec3 corner = osg::Vec3())
    {
        osg::Geometry* geom = osg::createTexturedQuadGeometry(
            corner,
            osg::Vec3(width, 0, 0),
            osg::Vec3(0, height, 0),
            0,
            0,
            scale,
            scale);
        osg::ref_ptr<osg::Geode> quad = new osg::Geode;
        quad->addDrawable(geom);
        int values = osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED;
        quad->getOrCreateStateSet()->setAttribute(
            new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK,
                                osg::PolygonMode::FILL),
            values);
        quad->getOrCreateStateSet()->setMode(GL_LIGHTING, values);
        return quad.release();
    }

    static std::vector<osg::ref_ptr<osg::Texture2D>> SetupRenderToTextureCamera(osg::ref_ptr<osg::Camera> camera, int width, int height, int numColorAttachments=1)
    {
        std::vector<osg::ref_ptr<osg::Texture2D>> result;
        camera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
        camera->setClearColor(osg::Vec4(0,0,0,0));
        camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
        camera->setRenderOrder(osg::Camera::PRE_RENDER);
        camera->setViewport(0, 0, 1024, 768);
        camera->setViewMatrix(osg::Matrix::identity());
        for(int i = 0; i < numColorAttachments; ++i) {
            osg::Texture2D* tex = RenderUtils::CreateFloatTextureRectangle(width, height);
            camera->attach(osg::Camera::BufferComponent(osg::Camera::COLOR_BUFFER0+i), tex);
            result.push_back(tex);
        }
        osg::ref_ptr<osg::Texture2D> depthTexture = RenderUtils::CreateDepthFloatTextureRectangle(1024, 768);
        camera->attach(osg::Camera::DEPTH_BUFFER, depthTexture.get());

        return result;
    }

    static osg::Camera *CreateHUDCamera()
    {
        osg::ref_ptr<osg::Camera> camera = new osg::Camera;
        camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
        camera->setClearMask(GL_DEPTH_BUFFER_BIT);
        camera->setAllowEventFocus(false);
        camera->setProjectionMatrix(osg::Matrix::identity());
        camera->setViewMatrix(osg::Matrix::identity());
        camera->setProjectionMatrix(osg::Matrix::identity());
        camera->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        return camera.release();
    }

    static osg::ref_ptr<osg::Camera> CreateTextureDisplayQuadCamera(const osg::Vec3 &pos, osg::ref_ptr<osg::StateSet> quadStateSet, float scale=1, float width = 2,float height = 2)
    {
        osg::ref_ptr<osg::Camera> hc = CreateHUDCamera();
        osg::Geode* quad = CreateScreenQuad(width, height, scale, pos);
        quad->setStateSet(quadStateSet.get());
        hc->addChild(quad);
        return hc;
    }
};

#endif