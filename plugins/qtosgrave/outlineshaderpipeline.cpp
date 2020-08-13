
#include "renderutils.h"
#include "outlineshaderpipeline.h"

#include <osg/Depth>
#include <osg/CullFace>
#include <osg/BlendFunc>

// state control

// predefined state control preprocessor variables
// to change programable state such as highligh color, see OutlineShaderPipeline constructor
#define ENABLE_SHOW_HIGHLIGHT_BEHIND_OBJECTS 1 //< if on, will show selection highlight over other objects
#define FADE_OUTLINE_WITH_FRAGMENT_DEPTH 1 // will fade out the edges proportionally to view distance. This prevent edges from becoming proportinally too big compared to the distance scene
#define INVERT_BLUR_AND_EDGE_DETECTION_PASS_ORDER 0 // < normal pipeline order is first edge detection, then blur to soften edges. Inverse order result in slightly different rendering

// debug preprocessor variables
#define SHOW_BLUR_PASS_ONLY 0
#define SHOW_COLORID_SCENE_ONLY 0
#define SHOW_EDGE_DETECTION_PASS_ONLY 0
#define BYPASS_BLUR_RENDER_PASS 0

namespace {
	const std::string outlineVertStr =
			"#version 120\n"
			"void main()\n"
			"{\n"
			"	// Vertex position in main camera Screen space.\n"
			"	gl_Position = gl_Vertex;\n"
			"}\n";

	const std::string outlineFragStr =
			"#version 120\n"
			"#extension GL_ARB_texture_multisample : enable \n"
			"uniform vec3 outlineColor;"
			"uniform vec3 selectionColor;"
			"uniform float highlightIntensity;"

			"\n"
			"uniform sampler2DMS diffuseTexture;\n"
			"\n"
			"vec4 accessTexel(sampler2DMS tex, ivec2 tc) {\n"
			"    vec4 c = texelFetch(tex, tc, 0) + texelFetch(tex, tc, 1) + texelFetch(tex, tc, 2) + texelFetch(tex, tc, 3);\n"
			"    return c / 4.0;\n"
			"}\n"
			"void getNeighbors(inout vec4 n[4], ivec2 coord)\n"
			"{\n"
			" // n values are stored from - to +, first x then y \n"
			"    float h = 1;\n"
			"\n"
			"    n[0] = (accessTexel(diffuseTexture, coord + ivec2( -h, 0 )));\n"
			"    n[1] = (accessTexel(diffuseTexture, coord + ivec2( h, 0 )));\n"
			"    n[2] = (accessTexel(diffuseTexture, coord + ivec2( 0.0, -h )));\n"
			"    n[3] = (accessTexel(diffuseTexture, coord + ivec2( 0.0, h )));\n"
			"}\n"
			""
			"float gradientIntensity(in vec4 n[4]) {\n"
			"    float h = 1;\n"
			"\n"
			"    vec4 xm = n[0].rgba;\n"
			"    vec4 xp = n[1].rgba;\n"
			"    vec4 ym = n[2].rgba;\n"
			"    vec4 yp = n[3].rgba;\n"
			"\n"
			"    vec4 dx = (xp - xm) / (2 * h);\n"
			"    vec4 dy = (yp - ym) / (2 * h);\n"
			"\n"
			"    return length(dx) + length(dy);\n"
			"}\n"
			"\n"

			"void main()\n"
			"{\n"
			" 	 vec4 samples[4];"
			"    getNeighbors(samples, ivec2(gl_FragCoord.x, gl_FragCoord.y));\n"
			"    float alphaIntensity = length(vec2((samples[0].a - samples[1].a)/2, (samples[2].a - samples[3].a)/2));\n"
			"    float intensity = gradientIntensity(samples);\n"
			"    bool selected = alphaIntensity > 0 || accessTexel(diffuseTexture, ivec2(gl_FragCoord.x, gl_FragCoord.y)).a > 0;\n"
#if			SHOW_BLUR_PASS_ONLY || SHOW_EDGE_DETECTION_PASS_ONLY
			"    gl_FragColor = vec4(intensity, intensity, intensity, 1);\n"
#else
			"    if(selected) {"
			"  		  gl_FragColor = vec4(selectionColor.xyz, intensity * highlightIntensity*0.5 + 0.25);\n" // sum a constant offset in alpha so to create the filling highlight effect
			"         return;\n"
			"    }\n"
			"    intensity = intensity * intensity;\n"
		    "    gl_FragColor = vec4(outlineColor, intensity);\n"
#endif
			"}\n";

	const std::string blurVertStr =
			"#version 120\n"
			"void main()\n"
			"{\n"
			"	// Vertex position in main camera Screen space.\n"
			"	gl_Position = gl_Vertex;\n"
			"}\n";

	const std::string blurFragStr =
			"#version 120\n"
			"#extension GL_ARB_texture_multisample : enable \n"

			"\n"
			"uniform sampler2DMS diffuseTexture;\n"
			"\n"
			"vec4 accessTexel(sampler2DMS tex, ivec2 tc) {\n"
			"    vec4 c = texelFetch(tex, tc, 0) + texelFetch(tex, tc, 1) + texelFetch(tex, tc, 2) + texelFetch(tex, tc, 3);\n"
			"    return c / 4.0;\n"
			"}\n"
			"void getNeighbors(inout vec4 n[9], ivec2 coord)\n"
			"{\n"
			" // n values are stored from - to +, first x then y \n"
			"    float h = 1;\n"
			"    float w = 1;\n"
				"\tn[0] = accessTexel(diffuseTexture, coord + ivec2( -w, -h));\n"
				"\tn[1] = accessTexel(diffuseTexture, coord + ivec2(0.0, -h));\n"
				"\tn[2] = accessTexel(diffuseTexture, coord + ivec2(  w, -h));\n"
				"\tn[3] = accessTexel(diffuseTexture, coord + ivec2( -w, 0.0));\n"
				"\tn[4] = accessTexel(diffuseTexture, coord);\n"
				"\tn[5] = accessTexel(diffuseTexture, coord + ivec2(  w, 0.0));\n"
				"\tn[6] = accessTexel(diffuseTexture, coord + ivec2( -w, h));\n"
				"\tn[7] = accessTexel(diffuseTexture, coord + ivec2(0.0, h));\n"
				"\tn[8] = accessTexel(diffuseTexture, coord + ivec2(  w, h));\n"
			"}\n"
			""
			"vec4 applyBlur(ivec2 coord) {\n"
			"    vec4 n[9];\n"
			"    getNeighbors(n, coord);\n"
			"\n"
			"    vec4 sum = (1.0 * n[0] + 2.0 * n[1] + 1.0 * n[2] +\n"
			"                2.0 * n[3] + 4.0 * n[4] + 2.0 * n[5] +\n"
			"                1.0 * n[6] + 2.0 * n[7] + 1.0 * n[8]) / 16.0;\n"
			"\n"
			"    return sum;\n"
			"}\n"
			"\n"


			"void main()\n"
			"{\n"
#if         BYPASS_BLUR_RENDER_PASS
			"    vec4 blur = accessTexel(diffuseTexture, ivec2(gl_FragCoord.x, gl_FragCoord.y));\n"
#else
			"    vec4 blur = applyBlur(ivec2(gl_FragCoord.x, gl_FragCoord.y));\n"
#endif
			"    gl_FragColor = blur;\n"
			"}\n";


	const std::string preRenderFragShaderStr =
			"#version 120\n"
			"\n"
			"varying vec3 normal;\n"


			"\n"
			"varying vec3 position;\n"
			"varying vec4 color;\n"
			"uniform vec3 uniqueColorId;"
			"uniform int isSelected;"

			"\n"
			"void main()\n"
			"{\n"
			" float depthVal = min(2, 4*gl_FragCoord.w);\n"
#if 	ENABLE_SHOW_HIGHLIGHT_BEHIND_OBJECTS
			"  if(isSelected == 1) {\n"
			"    gl_FragDepth = gl_FragCoord.z / 50;\n"
			"  }   \n"
			"else {\n"
			"  gl_FragDepth = gl_FragCoord.z;\n"
			"}\n"
#endif
			"\n"
			"float depthMult = 1.0;\n"
#if         FADE_OUTLINE_WITH_FRAGMENT_DEPTH
			"depthMult = min(2, depthVal * 3);\n"
#endif
			" gl_FragColor = vec4(normalize(uniqueColorId*0.5 + normalize(normal)) * depthMult, isSelected);\n"
			"}\n";

	const std::string preRenderVertShaderStr =
			"#version 120\n"
			"\n"
			"varying vec3 normal;\n"
			"varying vec3 position;\n"
			"varying vec4 color;\n"
			"\n"
			"\n"
			"void main()\n"
			"{\n"
			"    color = gl_Color;\n"
			"    normal = (gl_Normal);\n"
			"    position = gl_Vertex.xyz;\n"
			"    // Calculate vertex position in clip coordinates\n"
			"    gl_Position = gl_ModelViewProjectionMatrix * vec4(gl_Vertex.xyz, 1);\n"
			"}\n";
}

void RenderPassState::HandleResize(int width, int height)
{
	if(camera) {
		camera->setRenderingCache(0);
		camera->setViewport(0, 0, width, height);
	}
}

OutlineShaderPipeline::OutlineShaderPipeline()
{
	_outlineColor = osg::Vec3(0, 0, 0);
	_selectedObjectHighlightIntensity = 1.0f; //< from 0 to +inf, a multiplier on the highligh intensity, 0 means off, 1 means normal (e.g: 2 means double intensity)
	_selectedObjectHighlightColor = osg::Vec3(0, 0.95, 0); //< color of the selected object highlight
}

OutlineShaderPipeline::~OutlineShaderPipeline()
{
	for(RenderPassState* state : _renderPassStates) {
		delete state;
	}
}

// First pass is render the scene using special shaders to enhance edges
inline RenderPassState* OutlineShaderPipeline::createFirstRenderPass(osg::ref_ptr<osg::Camera> mainSceneCamera, osg::ref_ptr<osg::Node> mainSceneRoot, int maxFBOBufferWidth, int maxFBOBufferHeight)
{
	// First pass will render the same scene using a special shader that render objects with different colors
	// different from background, so to prepare for outline edge detection post processing shader
	RenderPassState* renderPassState = new RenderPassState();
	osg::ref_ptr<osg::StateSet> firstPassStateSet = new osg::StateSet();
	RenderUtils::SetShaderProgramOnStateSet(firstPassStateSet.get(), preRenderVertShaderStr, preRenderFragShaderStr);
	osg::ref_ptr<osg::Group> firstPassGroup = new osg::Group();
	firstPassGroup->setStateSet(firstPassStateSet);
	firstPassGroup->addChild(mainSceneRoot);

	// clone main camera settings so to render same scene
	renderPassState->camera = new osg::Camera();
	renderPassState->colorFboTexture = RenderUtils::CreateFloatTextureRectangle(maxFBOBufferWidth, maxFBOBufferHeight);
#	if !SHOW_COLORID_SCENE_ONLY
	RenderUtils::SetupRenderToTextureCamera(renderPassState->camera, osg::Camera::COLOR_BUFFER, renderPassState->colorFboTexture.get());
#endif
	renderPassState->camera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	renderPassState->camera->setClearColor(osg::Vec4(0, 0, 0, 0));
	renderPassState->camera->setViewMatrix(osg::Matrix::identity());
	renderPassState->camera->setProjectionMatrix(osg::Matrix::identity());
	// add outline camera as child of original scene camera so we can iherit transform and render same scene in the first pass (except we will render to texture)
	mainSceneCamera->addChild(renderPassState->camera.get());
	renderPassState->camera->addChild(firstPassGroup.get());
	firstPassStateSet->addUniform(new osg::Uniform("uniqueColorId", osg::Vec3(0, 0, 0)));
	firstPassStateSet->addUniform(new osg::Uniform("isSelected", 0));
	// disable blend because we use alpha channel for encoding other information
	firstPassStateSet->setMode(GL_BLEND, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	return renderPassState;
}

RenderPassState* OutlineShaderPipeline::createSecondRenderPass(RenderPassState* firstPassState, int maxFBOBufferWidth, int maxFBOBufferHeight)
{
	RenderPassState* renderPassState = new RenderPassState();
	osg::ref_ptr<osg::StateSet> secondPassStateSet = new osg::StateSet();
#if INVERT_BLUR_AND_EDGE_DETECTION_PASS_ORDER
	RenderUtils::SetShaderProgramOnStateSet(secondPassStateSet.get(), outlineVertStr, outlineFragStr);
#else
	RenderUtils::SetShaderProgramOnStateSet(secondPassStateSet.get(), blurVertStr, blurFragStr);
#endif

	renderPassState->camera = RenderUtils::CreateTextureDisplayQuadCamera(osg::Vec3(-1.0, -1.0, 0), secondPassStateSet);
	renderPassState->colorFboTexture = RenderUtils::CreateFloatTextureRectangle(maxFBOBufferWidth, maxFBOBufferHeight);
	RenderUtils::SetupRenderToTextureCamera(renderPassState->camera, osg::Camera::COLOR_BUFFER, renderPassState->colorFboTexture.get());

	// add outline camera as child of original scene camera so we can iherit transform and render same scene in the first pass (except we will render to texture)
	secondPassStateSet->setTextureAttributeAndModes(0, firstPassState->colorFboTexture.get(), osg::StateAttribute::ON);
    secondPassStateSet->addUniform(new osg::Uniform("diffuseTexture", 0));
	secondPassStateSet->addUniform(new osg::Uniform("outlineColor", _outlineColor));
	secondPassStateSet->addUniform(new osg::Uniform("selectionColor", _selectedObjectHighlightColor));
	secondPassStateSet->addUniform(new osg::Uniform("isSelected", 0));

	return renderPassState;
}

 RenderPassState* OutlineShaderPipeline::createThirdRenderPass(RenderPassState* secondPassState)
{
	RenderPassState* renderPassState = new RenderPassState();
	osg::ref_ptr<osg::StateSet> thirdPassStateSet = new osg::StateSet();
#if INVERT_BLUR_AND_EDGE_DETECTION_PASS_ORDER
	RenderUtils::SetShaderProgramOnStateSet(thirdPassStateSet.get(), blurVertStr, blurFragStr);
#else
	RenderUtils::SetShaderProgramOnStateSet(thirdPassStateSet.get(), outlineVertStr, outlineFragStr);
#endif
	renderPassState->camera = RenderUtils::CreateTextureDisplayQuadCamera(osg::Vec3(-1.0, -1.0, 0), thirdPassStateSet);
	thirdPassStateSet->setTextureAttributeAndModes(0, secondPassState->colorFboTexture.get(), osg::StateAttribute::ON);
    thirdPassStateSet->addUniform(new osg::Uniform("diffuseTexture", 0));
	thirdPassStateSet->setMode(GL_BLEND, osg::StateAttribute::ON);
	thirdPassStateSet->setAttributeAndModes(new osg::BlendFunc(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA ));
	osg::Depth* depth = new osg::Depth;
	depth->setWriteMask( false );
	thirdPassStateSet->setAttributeAndModes(depth, osg::StateAttribute::ON);
	thirdPassStateSet->addUniform(new osg::Uniform("outlineColor", _outlineColor));
	thirdPassStateSet->addUniform(new osg::Uniform("selectionColor", _selectedObjectHighlightColor));
	thirdPassStateSet->addUniform(new osg::Uniform("isSelected", 0));
	thirdPassStateSet->addUniform(new osg::Uniform("highlightIntensity", _selectedObjectHighlightIntensity));

	return renderPassState;
}

osg::ref_ptr<osg::Group> OutlineShaderPipeline::CreateOutlineSceneFromOriginalScene(osg::ref_ptr<osg::Camera> mainSceneCamera, 
	osg::ref_ptr<osg::Node> mainSceneRoot, int maxFBOBufferWidth, int maxFBOBufferHeight)
{
	_renderPassStates.push_back(createFirstRenderPass(mainSceneCamera, mainSceneRoot, maxFBOBufferWidth, maxFBOBufferHeight));
	_renderPassStates.push_back(createSecondRenderPass(_renderPassStates[0], maxFBOBufferWidth, maxFBOBufferHeight));
	_renderPassStates.push_back(createThirdRenderPass(_renderPassStates[1]));

	osg::ref_ptr<osg::Group> passesGroup = new osg::Group();
	passesGroup->addChild(_renderPassStates[0]->camera.get());
#	if !SHOW_COLORID_SCENE_ONLY
	passesGroup->addChild(_renderPassStates[1]->camera.get());
	passesGroup->addChild(mainSceneRoot);
	passesGroup->addChild(_renderPassStates[2]->camera.get());
#endif
	return passesGroup.release();

}
void OutlineShaderPipeline::HandleResize(int width, int height)
{
	for(RenderPassState* state : _renderPassStates) {
		state->HandleResize(width, height);
	}
}
