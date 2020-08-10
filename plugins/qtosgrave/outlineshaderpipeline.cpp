
#include "renderutils.h"
#include "outlineshaderpipeline.h"

#include <osg/Depth>
#include <osg/CullFace>
#include <osg/BlendFunc>

	namespace {
	const std::string simpleTextureVert = 
			"#version 120\n"
			"void main()\n"
			"{\n"
			"	// Vertex position in main camera Screen space.\n"
			"	gl_Position = gl_Vertex;\n"
			"}\n";

	const std::string simpleTextureFrag =
			"#version 120\n"
			//"#extension GL_ARB_texture_multisample : enable \n"

			"\n"
			"uniform sampler2D diffuseTexture;\n"
			"\n"
			"\n"
			"float gradientIntensity(float stepSize, vec2 coord ) {\n"
			"    float h = 1;\n"
			"\n"
			"    vec3 xm = (texture2D(diffuseTexture, coord + vec2( -h/1024, 0 ))).rgb;\n"
			"    vec3 xp = (texture2D(diffuseTexture, coord + vec2( h/1024, 0 ))).rgb;\n"
			"    vec3 ym = (texture2D(diffuseTexture, coord + vec2( 0.0, -h/768 ))).rgb;\n"
			"    vec3 yp = (texture2D(diffuseTexture, coord + vec2( 0.0, h/768 ))).rgb;\n"
			"\n"
			"    vec3 dx = (xp - xm) / (2 * h);\n"
			"    vec3 dy = (yp - ym) / (2 * h);\n"
			"\n"
			"    return length(dx) + length(dy);\n"
			"}\n"
			"\n"
			// "vec4 accessTexel(sampler2DMS tex, ivec2 tc) {\n"
			// "    vec4 c = texelFetch(tex, tc, 0) + texelFetch(tex, tc, 1) + texelFetch(tex, tc, 2) + texelFetch(tex, tc, 3);\n"
			// "    return c / 4.0;\n"
			// "}\n"
			"void main()\n"
			"{\n"
		    //"    gl_FragColor = vec4(gl_FragCoord.x/1024,gl_FragCoord.y/768,0,1);\n"
			//"    gl_FragColor = vec4(gl_FragCoord.x/1024, gl_FragCoord.y/768,0,1);\n"
			//"    gl_FragColor = texture2D(diffuseTexture,  vec2(gl_FragCoord.x/1024, gl_FragCoord.y/768));\n"
			"    float intensity = gradientIntensity(1, vec2(gl_FragCoord.x/1024, gl_FragCoord.y/768));\n"
			"	gl_FragColor = vec4(1,0,0, sqrt(intensity));\n"
			"}\n";


	const std::string normalColorFragShaderStr =
			"#version 120\n"
			"\n"
			"varying vec3 normal;\n"
			
			"\n"
			"varying vec3 position;\n"
			//"out vec4 color;\n"
			"\n"
			"void main()\n"
			"{\n"
			"    gl_FragColor = vec4(normal + position, 1);\n"
			"}\n";

	const std::string normalColorVertShaderStr =
			"#version 120\n"
			"\n"
			// "in vec4 osg_Vertex;\n"
			// "in vec3 osg_Normal;\n"
			// "in vec4 osg_Color;\n"
			"\n"
			"varying vec3 normal;\n"
			"varying vec3 position;\n"
			"\n"
			"\n"
			"void main()\n"
			"{\n"
			"    normal = normalize(gl_Normal);\n"
			"    position = gl_Vertex.xyz;\n"
			"    // Calculate vertex position in clip coordinates\n"
			"    gl_Position = gl_ModelViewProjectionMatrix * vec4(gl_Vertex.xyz, 1);\n"
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

OutlineShaderPipeline::OutlineShaderPipeline()
{
	// empty
}

void OutlineShaderPipeline::InitializeOutlinePipelineState(osg::ref_ptr<osg::Camera> originalSceneCamera, osg::ref_ptr<osg::StateSet> inheritedStateSet, osg::ref_ptr<osg::Node> originalSceneRoot, int viewportWidth, int viewportHeight)
{
	// First pass will render the same scene using a special shader that render objects with different colors
	// different from background, so to prepare for outline edge detection post processing shader
	osg::ref_ptr<osg::StateSet> firstPassStateSet = new osg::StateSet((*inheritedStateSet.get()), osg::CopyOp::DEEP_COPY_ALL);
	RenderUtils::SetShaderProgramOnStateSet(firstPassStateSet.get(), normalColorVertShaderStr, normalColorFragShaderStr);
	_firstPassState.firstPassGroup = new osg::Group();
	_firstPassState.firstPassGroup->setStateSet(firstPassStateSet);
	_firstPassState.firstPassGroup->addChild(originalSceneRoot);

	// clone main camera settings so to render same scene
	_firstPassState.firstPassCamera = new osg::Camera();
	_firstPassState.firstPassRenderTexture = RenderUtils::CreateFloatTextureRectangle(viewportWidth, viewportHeight);
	RenderUtils::SetupRenderToTextureCamera(_firstPassState.firstPassCamera, osg::Camera::COLOR_BUFFER, _firstPassState.firstPassRenderTexture.get());
	_firstPassState.firstPassCamera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	_firstPassState.firstPassCamera->setClearColor(osg::Vec4(0.95, 0, 0, 1.0));
	_firstPassState.firstPassCamera->setViewMatrix(osg::Matrix::identity());
	_firstPassState.firstPassCamera->setProjectionMatrix(osg::Matrix::identity());
	// add outline camera as child of original scene camera so we can iherit transform and render same scene in the first pass (except we will render to texture)
	originalSceneCamera->addChild(_firstPassState.firstPassCamera.get());
	_firstPassState.firstPassCamera->addChild(_firstPassState.firstPassGroup.get());

	osg::ref_ptr<osg::StateSet> secondPassStateSet = new osg::StateSet();
	RenderUtils::SetShaderProgramOnStateSet(secondPassStateSet.get(), simpleTextureVert, simpleTextureFrag);
	_secondPassState.secondPassCamera = RenderUtils::CreateTextureDisplayQuadCamera(osg::Vec3(-1.0, -1.0, 0), secondPassStateSet);
	secondPassStateSet->setTextureAttributeAndModes(0, _firstPassState.firstPassRenderTexture.get(), osg::StateAttribute::ON);
    secondPassStateSet->addUniform(new osg::Uniform("diffuseTexture", 0));
	secondPassStateSet->setMode(GL_BLEND, osg::StateAttribute::ON);
	secondPassStateSet->setAttributeAndModes(new osg::BlendFunc(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA ));
	osg::Depth* depth = new osg::Depth;
	depth->setWriteMask( false );
	secondPassStateSet->setAttributeAndModes( depth, osg::StateAttribute::ON);
}
