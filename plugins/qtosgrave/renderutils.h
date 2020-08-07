
#ifndef OPENRAVE_QTOSG_RENDERUTILS_H
#define OPENRAVE_QTOSG_RENDERUTILS_H

#include <osg/Geode>
#include <osg/Group>
#include <osg/Camera>
#include <osg/Shader>
#include <osg/Geometry>
#include <osg/Texture2D>
#include <osg/PolygonMode>
#include <osg/TextureRectangle>
#include <osg/Texture2DMultisample>

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
            osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
    }

    static osg::Texture2DMultisample *createFloatTextureRectangle(int width, int height)
    {
        osg::ref_ptr<osg::Texture2DMultisample> tex2D = new osg::Texture2DMultisample;
        tex2D->setTextureSize(width, height);
        tex2D->setInternalFormat(GL_RGBA16F_ARB);
        tex2D->setSourceFormat(GL_RGBA);
        tex2D->setSourceType(GL_FLOAT);
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

    static osg::Camera* CreateRenderToTextureCamera(osg::Camera::BufferComponent buffer, osg::Texture *tex, bool isAbsolute)
    {
        osg::ref_ptr<osg::Camera> camera = new osg::Camera;
        SetupRenderToTextureCamera(camera, buffer, tex, isAbsolute);
        return camera.release();
    }

    static void SetupRenderToTextureCamera(osg::ref_ptr<osg::Camera> camera, osg::Camera::BufferComponent buffer, osg::Texture *tex, bool isAbsolute)
    {
        camera->setClearColor(osg::Vec4());
        camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
        camera->setRenderOrder(osg::Camera::PRE_RENDER);
        if (tex)
        {
            tex->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR);
            tex->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR);
            camera->setViewport(0, 0, tex->getTextureWidth(), tex->getTextureHeight());
            camera->attach(buffer, tex);
        }
        if (isAbsolute)
        {
            camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
            camera->setProjectionMatrix(osg::Matrix::ortho2D(0.0, 1.0, 0.0, 1.0));
            camera->setViewMatrix(osg::Matrix::identity());
            camera->addChild(CreateScreenQuad(1.0f, 1.0f));
        }
    }

    static osg::Camera *CreateHUDCamera(double left = 0, double right = 1, double bottom = 0, double top = 1)
    {
        osg::ref_ptr<osg::Camera> camera = new osg::Camera;
        camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
        camera->setClearMask(GL_DEPTH_BUFFER_BIT);
        camera->setRenderOrder(osg::Camera::POST_RENDER);
        camera->setAllowEventFocus(false);
        camera->setProjectionMatrix(osg::Matrix::ortho2D(left, right, bottom, top));
        camera->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        return camera.release();
    }

    static osg::ref_ptr<osg::Camera> CreateTextureDisplayQuad( const osg::Vec3 &pos, osg::StateAttribute *tex, float scale, float width = 1,float height = 1)
    {
        osg::ref_ptr<osg::Camera> hc = CreateHUDCamera();
        hc->addChild(CreateScreenQuad(width, height, scale, pos));
        hc->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex);
        return hc;
    }
};

#endif