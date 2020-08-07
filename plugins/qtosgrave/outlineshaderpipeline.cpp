
#include "renderutils.h"
#include "outlineshaderpipeline.h"
#include <iostream>

	namespace {
	const std::string normalColorFragShaderStr =
			"#version 300 es\n"
			"precision mediump float;                  \n"
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
			"#version 300 es\n"
			"precision mediump float;                  \n"
			"\n"
			"in vec3 vertexPosition;\n"
			"in vec3 vertexNormal;\n"
			"\n"
			"out vec3 normal;\n"
			"out vec3 position;\n"
			"\n"
			"uniform mat4 osg_ModelViewProjectionMatrix;\n"
			"uniform mat3 osg_NormalMatrix;\n"
			"\n"
			"void main()\n"
			"{\n"
			"    normal = normalize(osg_NormalMatrix * vertexNormal);\n"
			"    position = vertexPosition;\n"
			"    // Calculate vertex position in clip coordinates\n"
			"    gl_Position = osg_ModelViewProjectionMatrix * vec4(vertexPosition, 1.0);\n"
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

void OutlineShaderPipeline::InitializeOutlinePipelineState(osg::ref_ptr<osg::Camera> originalSceneCamera, osg::ref_ptr<osg::Node> originalSceneRoot, int viewportWidth, int viewportHeight)
{
	// First pass will render the same scene using a special shader that render objects with different colors
	// different from background, so to prepare for outline edge detection post processing shader
	osg::ref_ptr<osg::StateSet> firstPassStateSet = new osg::StateSet;
	RenderUtils::SetShaderProgramOnStateSet(firstPassStateSet.get(), normalColorVertShaderStr, normalColorFragShaderStr);
	_firstPassState.firstPassGroup = new osg::Group();
	_firstPassState.firstPassGroup->addChild(originalSceneRoot);
	_firstPassState.firstPassGroup->setStateSet(firstPassStateSet);

	// clone main camera settings so to render same scene
	_firstPassState.firstPassCamera = new osg::Camera();
	_firstPassState.firstPassCamera->setReferenceFrame(osg::Camera::RELATIVE_RF);
	// render after main camera
	_firstPassState.firstPassCamera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	_firstPassState.firstPassCamera->setClearColor(osg::Vec4(0.95, 0, 0, 1.0));
	_firstPassState.firstPassCamera->setViewMatrix(osg::Matrix::identity());
	_firstPassState.firstPassCamera->setComputeNearFarMode(osg::Camera::DO_NOT_COMPUTE_NEAR_FAR);

	// add outline camera as child of original scene camera so we can iherit transform and render same scene in the first pass (except we will render to texture)
	originalSceneCamera->addChild(_firstPassState.firstPassCamera);
	_firstPassState.firstPassCamera->addChild(_firstPassState.firstPassGroup);

	// osg::Texture* renderToTexture = createFloatTextureRectangle(viewportWidth, viewportHeight);
	// osg::ref_ptr<osg::Camera> renderPassCamera =
	// createTextureDisplayQuad(osg::Vec3(-1, -1, 0),
	//                          p.pass2Normals,
	//                          p.textureSize);
}

void printOSGMAtrix(const osg::Matrixd& m) {
	for(int i = 0 ; i < 4; ++i) {
			std::cout << m(i,0) << ", " << m(i,1) << m(i,2) << ", " << m(i,3) << std::endl;
	}
}

void OutlineShaderPipeline::UpdateOutlineCameraFromOriginal(osg::ref_ptr<osg::Camera> originalSceneCamera, int width, int height)
{
	if(!_firstPassState.firstPassGroup || !_firstPassState.firstPassCamera) {
			return;
	}
	_firstPassState.firstPassCamera->setViewport(0,0,width,height);

	// copy projection matrix from original scene camera to our first pass camera, so to render same view
	// (ps: dont need to copy viewMatrix because our outline first pass camera is set to be child of scene original camera, so it will follow rigid body transformations)
	osg::Matrixd originalSceneCameraProjection = originalSceneCamera->getProjectionMatrix();
	_firstPassState.firstPassCamera->setProjectionMatrix(originalSceneCameraProjection);
}
