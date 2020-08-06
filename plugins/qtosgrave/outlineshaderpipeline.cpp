
#include "renderutils.h"
#include "outlineshaderpipeline.h"

namespace {
    const std::string normalColorFragShaderStr =
                "#version 150 core\n"
                "\n"
                "in vec3 normal;\n"
                "uniform vec4 maincolor;\n"
                "out vec4 fragColor;\n"
                "\n"
                "in vec3 position;\n"
                "\n"
                "void main()\n"
                "{\n"
                "    fragColor = vec4(normal, 1);\n"
                "}\n";

    const std::string normalColorVertShaderStr =
                "#version 150 core\n"
                "\n"
                "\n"
                "in vec3 vertexPosition;\n"
                "in vec3 vertexNormal;\n"
                "\n"
                "out vec3 normal;\n"
                "out vec3 position;\n"
                "\n"
                "uniform mat4 modelView;\n"
                "uniform mat3 modelViewNormal;\n"
                "uniform mat4 mvp;\n"
                "\n"
                "void main()\n"
                "{\n"
                "    normal = normalize(modelViewNormal * vertexNormal);\n"
                "    position = vertexPosition;\n"
                "    // Calculate vertex position in clip coordinates\n"
                "    gl_Position = mvp * vec4(vertexPosition, 1.0);\n"
                "}\n";

    const std::string outlineFragShaderStr =
                "#version 150\n"
                "\n"
                "uniform sampler2DMS color;\n"
                "uniform vec2 winSize;\n"
                "\n"
                "uniform float sobelEdgeLength;\n"
                "\n"
                "out vec4 fragColor;\n"
                "\n"
                "vec4 accessTexel(sampler2DMS tex, ivec2 tc) {\n"
                "    vec4 c = texelFetch(tex, tc, 0) + texelFetch(tex, tc, 1) + texelFetch(tex, tc, 2) + texelFetch(tex, tc, 3);\n"
                "    return c / 4.0;\n"
                "}\n"
                "\n"
                "\n"
                "float intensityFromColor(vec4 color) {\n"
                "    //return  log(color.r + 1) + log(color.g + 1) + log(color.b + 1);\n"
                "    return 0.2126 * color.r + 0.7152 * color.g + 0.0722 * color.b;\n"
                "}\n"
                "\n"
                "void make_kernel(inout vec4 n[9], sampler2DMS tex, ivec2 coord)\n"
                "{\n"
                "\tfloat w = 1.0;\n"
                "\tfloat h = 1.0;\n"
                "\n"
                "\tn[0] = accessTexel(tex, coord + ivec2( -w, -h));\n"
                "\tn[1] = accessTexel(tex, coord + ivec2(0.0, -h));\n"
                "\tn[2] = accessTexel(tex, coord + ivec2(  w, -h));\n"
                "\tn[3] = accessTexel(tex, coord + ivec2( -w, 0.0));\n"
                "\tn[4] = accessTexel(tex, coord);\n"
                "\tn[5] = accessTexel(tex, coord + ivec2(  w, 0.0));\n"
                "\tn[6] = accessTexel(tex, coord + ivec2( -w, h));\n"
                "\tn[7] = accessTexel(tex, coord + ivec2(0.0, h));\n"
                "\tn[8] = accessTexel(tex, coord + ivec2(  w, h));\n"
                "}\n"
                "\n"
                "vec4 applyBlur(sampler2DMS tex, ivec2 coord) {\n"
                "    vec4 n[9];\n"
                "    make_kernel(n, tex, coord);\n"
                "\n"
                "    vec4 sum = (1.0 * n[0] + 2.0 * n[1] + 1.0 * n[2] +\n"
                "                2.0 * n[3] + 4.0 * n[4] + 2.0 * n[5] +\n"
                "                1.0 * n[6] + 2.0 * n[7] + 1.0 * n[8]) / 16.0;\n"
                "\n"
                "    return sum;\n"
                "}\n"
                "\n"
                "float gradientIntensity(float stepSize, ivec2 coord ) {\n"
                "    float h = 1;\n"
                "\n"
                "    vec3 xm = (accessTexel(color, coord + ivec2( -h, 0 ))).rgb;\n"
                "    vec3 xp = (accessTexel(color, coord + ivec2( h, 0 ))).rgb;\n"
                "    vec3 ym = (accessTexel(color, coord + ivec2( 0.0, -h ))).rgb;\n"
                "    vec3 yp = (accessTexel(color, coord + ivec2( 0.0, h ))).rgb;\n"
                "\n"
                "    vec3 dx = (xp - xm) / (2 * h);\n"
                "    vec3 dy = (yp - ym) / (2 * h);\n"
                "\n"
                "    return length(dx) + length(dy);\n"
                "}\n"
                "\n"
                "void main()\n"
                "{\n"
                "    float intensity = gradientIntensity(1, ivec2(gl_FragCoord.x, gl_FragCoord.y));\n"
                "    fragColor = vec4(0,1,0,intensity);\n"
                "}\n";

        const std::string outlineVertShaderStr =
                "#version 150\n"
                "\n"
                "in vec3 vertexPosition;\n"
                "void main()\n"
                "{\n"
                "    // assume position in eye space"
                "    gl_Position = vec4(vertexPosition, 1.0);\n"
                "}\n";
}

OutlineShaderPipeline OutlineShaderPipeline::CreateOutlineRenderingScene(osg::ref_ptr<osg::Camera> originalSceneCamera, osg::ref_ptr<osg::Node> originalSceneRoot, int viewportWidth, int viewportHeight)
{
        OutlineShaderPipeline pipeline;
        // First pass will render the same scene using a special shader that render objects with different colors
        // different from background, so to prepare for outline edge detection post processing shader
        osg::ref_ptr<osg::StateSet> firstPassStateSet = new osg::StateSet;
        RenderUtils::SetShaderProgramOnStateSet(firstPassStateSet.get(), normalColorVertShaderStr, normalColorFragShaderStr);
        pipeline.firstPassGroup = new osg::Group();
        //firstPassGroup->setStateSet(firstPassStateSet);

        // clone main camera settings so to render same scene
        pipeline.firstPassCamera = new osg::Camera(*originalSceneCamera, osg::CopyOp::DEEP_COPY_ALL);
        pipeline.firstPassGroup->addChild(pipeline.firstPassCamera.get());
        pipeline.firstPassCamera->addChild(originalSceneRoot);

        return pipeline;
        // osg::Texture* renderToTexture = createFloatTextureRectangle(viewportWidth, viewportHeight);
        // osg::ref_ptr<osg::Camera> renderPassCamera =
        // createTextureDisplayQuad(osg::Vec3(-1, -1, 0),
        //                          p.pass2Normals,
        //                          p.textureSize);
}

void OutlineShaderPipeline::Resize(int width, int height)
{
        firstPassCamera->setViewport(0,0,width,height);
}
