
#ifndef OPENRAVE_QTOSG_RENDERUTILS_H
#define OPENRAVE_QTOSG_RENDERUTILS_H

#include <osg/Geode>
#include <osg/Depth>
#include <osg/Group>
#include <osg/Camera>
#include <osg/Shader>
#include <osg/Geometry>
#include <osg/Texture2D>
#include <osg/BlendFunc>
#include <osgDB/ReadFile>
#include <osg/PolygonMode>
#include <osg/Texture2DMultisample>

#include "osgpick.h"

#include <string>

///data/shaders/simpleTexture.vert
const std::string simpleTexture_vert =
    "#version 120\n"
    "\n"
    "varying vec2 clipPos;\n"
    "void main()\n"
    "{\n"
    "    // Vertex position in main camera Screen space.\n"
    "    clipPos = gl_Vertex.xy*0.5 + vec2(0.5);\n"
    "    gl_Position = gl_Vertex;\n"
    "};\n";

///data/shaders/simpleTexture.frag
const std::string simpleTexture_frag =
    "#version 120\n"
    "\n"
    "uniform vec2 textureSize;\n"
    "uniform vec2 offSet;\n"
    "varying vec2 clipPos;\n"
    "\n"
    "uniform sampler2D colorTexture0;\n"
    "\n"
    "vec4 accessTexel(sampler2D tex, vec2 tc) {\n"
    "    return texture2D(tex, tc);\n"
    "}\n"
    "\n"
    "void main()\n"
    "    {\n"
    "    vec2 texCoord = clipPos - offSet;\n"
    "    gl_FragColor = accessTexel(colorTexture0, texCoord);\n"
    "};\n"
"\n";

class RenderUtils {
public:
    struct FBOData {
        std::vector<osg::ref_ptr<osg::Texture>> colorTextures;
        osg::ref_ptr<osg::Texture> depthTexture;
    };
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
        tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR);
        tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
        tex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
        tex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
        tex->setSourceFormat(GL_RGBA);
        tex->setInternalFormat(GL_RGBA32F_ARB);
        tex->setSourceType(GL_FLOAT);
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
        tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
        tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::NEAREST);
        tex->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_BORDER);
        tex->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_BORDER);
        tex->setSourceFormat(GL_DEPTH_COMPONENT);
        tex->setInternalFormat(GL_DEPTH_COMPONENT24);
        tex->setSourceType(GL_FLOAT);

        return tex.release();
    }

    static osg::Geode *CreateScreenQuad(float width, float height, float scale = 1, osg::Vec3 corner = osg::Vec3(), osg::ref_ptr<osg::StateSet> stateSet=nullptr)
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
        if(stateSet) {
            quad->setStateSet(stateSet.get());
        }
        quad->addDrawable(geom);
        int values = osg::StateAttribute::OFF | osg::StateAttribute::PROTECTED;
        quad->getOrCreateStateSet()->setAttribute(
            new osg::PolygonMode(osg::PolygonMode::FRONT_AND_BACK,
                                osg::PolygonMode::FILL),
            values);
        quad->getOrCreateStateSet()->setMode(GL_LIGHTING, values);
        osg::Depth* depth = new osg::Depth;
        depth->setWriteMask( false );
        quad->getOrCreateStateSet()->setAttributeAndModes( depth, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE );
        quad->getOrCreateStateSet()->setAttributeAndModes(new osg::BlendFunc(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA ));
	    quad->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
        quad->setNodeMask(~OSG_IS_PICKABLE_MASK);
        return quad.release();
    }

    static void SetupRenderToTextureCamera(osg::ref_ptr<osg::Camera> camera, int width, int height, FBOData& result, osg::Texture* reusedDepthTexture = nullptr, int numColorAttachments=1, bool useMultiSamples=false, const std::vector<osg::ref_ptr<osg::Texture>>& inheritedColorBuffers = std::vector<osg::ref_ptr<osg::Texture>>())
    {
        camera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
        camera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
        camera->setClearColor(osg::Vec4(0, 0, 0, 0));
        camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
        camera->setRenderOrder(osg::Camera::PRE_RENDER);
        camera->setViewport(0, 0, width, height);
        camera->setViewMatrix(osg::Matrix::identity());
        for(int i = 0; i < numColorAttachments; ++i) {
            osg::ref_ptr<osg::Texture> tex = nullptr;
            if(i < (int)inheritedColorBuffers.size()) {
                tex = inheritedColorBuffers[i].get();
            }
            else {
                tex = RenderUtils::CreateFloatTextureRectangle(width, height);
            }

            if(useMultiSamples) {
                camera->attach(osg::Camera::BufferComponent(osg::Camera::COLOR_BUFFER0+i), tex.get(), 0, 0, true, 4, 4);
            }
            else {
                camera->attach(osg::Camera::BufferComponent(osg::Camera::COLOR_BUFFER0+i), tex.get());
            }
            result.colorTextures.push_back(tex);
        }
        if(reusedDepthTexture == nullptr) {
            return;
        }
        result.depthTexture = reusedDepthTexture;
        camera->attach(osg::Camera::DEPTH_BUFFER, result.depthTexture.get());
    }

    static osg::Camera *CreateFBOTextureDisplayHUDViewPort(osg::Texture* texture, const osg::Vec2f& corner, const osg::Vec2f& size, int fboWidth, int fboHeight)
    {
        osg::ref_ptr<osg::Camera> hc = CreateHUDCamera();
        osg::Geode* quad = CreateScreenQuad(size.x(), size.y(), 1, osg::Vec3(corner.x(), corner.y(), 0));
        osg::StateSet* stateSet = quad->getOrCreateStateSet();
        stateSet->setTextureAttributeAndModes(0, texture);
        RenderUtils::SetShaderProgramOnStateSet(stateSet, simpleTexture_vert, simpleTexture_frag);
        stateSet->addUniform(new osg::Uniform("textureSize", osg::Vec2f(fboWidth, fboHeight)));
        osg::Vec2f scale = corner * 0.5 + osg::Vec2f(0.5, 0.5);
        stateSet->addUniform(new osg::Uniform("offSet", scale));
        hc->addChild(quad);
        return hc.release();
    }

    static osg::Camera* CreateHUDCamera()
    {
        osg::ref_ptr<osg::Camera> camera = new osg::Camera;
        camera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
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
        osg::Geode* quad = CreateScreenQuad(width, height, scale, pos, quadStateSet);
        hc->addChild(quad);
        return hc;
    }
};

#endif