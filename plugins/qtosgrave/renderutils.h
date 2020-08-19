
#ifndef OPENRAVE_QTOSG_RENDERUTILS_H
#define OPENRAVE_QTOSG_RENDERUTILS_H

#include <osg/Geode>
#include <osg/Group>
#include <osg/Camera>
#include <osg/Shader>
#include <osg/Geometry>
#include <osg/Texture2D>
#include <osgDB/ReadFile>
#include <osg/PolygonMode>
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
            osg::StateAttribute::ON);
    }

    static void SetShaderProgramFileOnStateSet(osg::ref_ptr<osg::StateSet> onwerStateSet, const std::string& vertShaderFilePath, const std::string& fragShaderFilePath )
    {
        osg::ref_ptr<osg::Program> program = new osg::Program;
        program->addShader(osgDB::readShaderFile(vertShaderFilePath));
        program->addShader(osgDB::readShaderFile(fragShaderFilePath));

        onwerStateSet->setAttributeAndModes(
            program.get(),
            osg::StateAttribute::ON);
    }

    static osg::Texture *CreateFloatTextureRectangle(int width, int height, bool useMultiSamples=false)
    {
        osg::ref_ptr<osg::Texture> tex = nullptr;
        if(useMultiSamples) {
            osg::Texture2DMultisample* texMS = new osg::Texture2DMultisample(4, false);
            texMS->setTextureSize(width, height);
            tex = texMS;
        }
        else {
            osg::Texture2D* tex2D = new osg::Texture2D();
            tex2D->setTextureSize(width, height);
            tex = tex2D;
        }
        tex->setSourceFormat(GL_RGBA);
        tex->setInternalFormat(GL_RGBA32F_ARB);
        tex->setSourceType(GL_FLOAT);
        tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
        tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
        return tex.release();
    }

    static osg::Texture *CreateDepthFloatTextureRectangle(int width, int height, bool useMultiSamples=false)
    {
        osg::ref_ptr<osg::Texture> tex = nullptr;
        if(useMultiSamples) {
            osg::Texture2DMultisample* texMS = new osg::Texture2DMultisample(4,false);
            texMS->setTextureSize(width, height);
            tex = texMS;
        }
        else {
            osg::Texture2D* tex2D = new osg::Texture2D();
            tex2D->setTextureSize(width, height);
            tex = tex2D;
        }
        tex->setSourceFormat(GL_DEPTH_COMPONENT);
        tex->setInternalFormat(GL_DEPTH_COMPONENT24);
        tex->setSourceType(GL_UNSIGNED_INT);
        return tex.release();
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

    static std::vector<osg::ref_ptr<osg::Texture>> SetupRenderToTextureCamera(osg::ref_ptr<osg::Camera> camera, int width, int height, int numColorAttachments=1, bool useMultiSamples=false)
    {
        std::vector<osg::ref_ptr<osg::Texture>> result;
        camera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
        camera->setClearColor(osg::Vec4(0,0,0,0));
        camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
        camera->setRenderOrder(osg::Camera::PRE_RENDER);
        camera->setViewport(0, 0, 1024, 768);
        camera->setViewMatrix(osg::Matrix::identity());
        for(int i = 0; i < numColorAttachments; ++i) {
            osg::Texture* tex = RenderUtils::CreateFloatTextureRectangle(width, height, useMultiSamples);
            camera->attach(osg::Camera::BufferComponent(osg::Camera::COLOR_BUFFER0+i), tex, 0, 0, false, 4, 4);
            result.push_back(tex);
        }
        osg::ref_ptr<osg::Texture> depthTexture = RenderUtils::CreateDepthFloatTextureRectangle(width, height, useMultiSamples);
        camera->attach(osg::Camera::DEPTH_BUFFER, depthTexture.get(), 0, 0, false, 4, 4);

        return result;
    }

    static osg::Camera *CreateFBOTextureDisplayHUDViewPort(osg::Texture* texture, const osg::Vec2f& corner, const osg::Vec2f& size, int fboWidth, int fboHeight)
    {
        osg::ref_ptr<osg::Camera> hc = CreateHUDCamera();
        osg::Geode* quad = CreateScreenQuad(size.x(), size.y(), 1, osg::Vec3(corner.x(), corner.y(), 0));
        osg::StateSet* stateSet = quad->getOrCreateStateSet();
        stateSet->setTextureAttributeAndModes(0, texture);
        RenderUtils::SetShaderProgramFileOnStateSet(stateSet, "/data/shaders/simpleTexture.vert", "/data/shaders/simpleTexture.frag");
        stateSet->addUniform(new osg::Uniform("textureSize", osg::Vec2f(fboWidth, fboHeight)));
        osg::Vec2f scale = corner * 0.5 + osg::Vec2f(0.5, 0.5);
        stateSet->addUniform(new osg::Uniform("offSet", scale));
        hc->addChild(quad);
        return hc.release();
    }

    static osg::Camera* CreateHUDCamera()
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