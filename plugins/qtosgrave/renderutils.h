
#ifndef OPENRAVE_QTOSG_RENDERUTILS_H
#define OPENRAVE_QTOSG_RENDERUTILS_H

#include <osg/Camera>
#include <osg/Group>
#include <osg/Shader>
#include <osg/Texture2D>
#include <osg/TextureRectangle>
#include <osg/Texture2DMultisample>

class RenderUtils {
public:
    osg::ref_ptr<osg::StateSet> setShaderProgram(osg::ref_ptr<osg::Camera> pass,
                                            const std::string& vertShaderString,
                                            const std::string& fragShaderString)
    {
        osg::ref_ptr<osg::Program> program = new osg::Program;
        osg::Shader* vertShader = new osg::Shader(osg::Sahder::VERTEX, vertShaderString);
        program->addShader(vertShader);

        osg::Shader* fragShader = new osg::Shader(osg::Sahder::FRAGMENT, fragShaderString);
        program->addShader(fragShader);

        osg::ref_ptr<osg::StateSet> ss = pass->getOrCreateStateSet();
        ss->setAttributeAndModes(
            program.get(),
            osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
        return ss;
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

    static osg::Geode *createScreenQuad(float width,
                                float height,
                                float scale = 1,
                                osg::Vec3 corner = osg::Vec3())
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

    static osg::Camera *createHUDCamera(double left = 0,
                                double right = 1,
                                double bottom = 0,
                                double top = 1)
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

    static osg::ref_ptr<osg::Camera> createTextureDisplayQuad(
        const osg::Vec3 &pos,
        osg::StateAttribute *tex,
        float scale,
        float width = 1,
        float height = 1)
    {
        osg::ref_ptr<osg::Camera> hc = createHUDCamera();
        hc->addChild(createScreenQuad(width, height, scale, pos));
        hc->getOrCreateStateSet()->setTextureAttributeAndModes(0, tex);
        return hc;
    }
};

#endif