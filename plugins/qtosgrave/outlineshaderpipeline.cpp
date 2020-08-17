
#include "renderutils.h"
#include "outlineshaderpipeline.h"
#include <sstream>

#include <osg/Depth>
#include <osg/CullFace>
#include <osg/BlendFunc>

// state control

// predefined state control preprocessor variables
// to change programable state such as highligh color, see OutlineShaderPipeline constructor
#define ENABLE_SHOW_HIGHLIGHT_BEHIND_OBJECTS 0 //< if on, will show selection highlight over other objects
#define FADE_OUTLINE_WITH_FRAGMENT_DEPTH 1 // will fade out the edges proportionally to view distance. This prevent edges from becoming proportinally too big compared to the distance scene


// debug preprocessor variables
#define SHOW_PRERENDER_SCENE_ONLY 0
#define SHOW_EDGE_DETECTION_PASS_ONLY 0


namespace {
	const std::string outlineVertStr =
			"#version 120\n"
			"varying vec2 clipPos;"
			"void main()\n"
			"{\n"
			"	// Vertex position in main camera Screen space.\n"
			"   clipPos = gl_Vertex.xy;\n"
			"	gl_Position = gl_Vertex;\n"
			"}\n";

	const std::string outlineFragStr =
			"#version 120\n"
			"#extension GL_ARB_texture_multisample : enable \n"
			"varying vec2 clipPos;"
			"uniform vec3 outlineColor;"
			"uniform vec3 selectionColor;"
			"uniform float highlightIntensity;"
			"uniform float depthThreshold;"
			"uniform float normalThreshold;"
			"uniform float depthNormalThreshold;"
			"uniform float depthNormalThresholdScale;"

			"\n"
			"uniform sampler2DMS colorTexture0;\n"
			"\n"
			"vec4 accessTexel(sampler2DMS tex, ivec2 tc) {\n"
			"    vec4 c = texelFetch(tex, tc, 0) + texelFetch(tex, tc, 1) + texelFetch(tex, tc, 2) + texelFetch(tex, tc, 3);\n"
			"    return c / 4.0;\n"
			"}\n"
			"void getNeighbors(inout vec4 n[9], ivec2 coord)\n"
			"{\n"
			" // n values are stored from - to +, first x then y \n"
			" float h = 1;\n"
			" float w = 1;\n"
			"\n"

    		"		n[0] = accessTexel(colorTexture0, coord + ivec2( -w, -h));"
    		"		n[1] = accessTexel(colorTexture0, coord + ivec2(0.0, -h));"
    		"		n[2] = accessTexel(colorTexture0, coord + ivec2(  w, -h));"
    		"		n[3] = accessTexel(colorTexture0, coord + ivec2( -w, 0.0));"
    		"		n[4] = accessTexel(colorTexture0, coord);"
    		"		n[5] = accessTexel(colorTexture0, coord + ivec2(  w, 0.0));"
    		"		n[6] = accessTexel(colorTexture0, coord + ivec2( -w, h));"
    		"		n[7] = accessTexel(colorTexture0, coord + ivec2(0.0, h));"
    		"		n[8] = accessTexel(colorTexture0, coord + ivec2(  w, h));"
			"}\n"

			"void getNeighbors4(inout vec4 n[4], ivec2 coord)\n"
			"{\n"
			" // n values are stored from - to +, first x then y \n"
			" float halfScaleFloor = 0;\n"
			" float halfScaleCeil = 1;\n"
			"\n"
			"    n[0] = (accessTexel(colorTexture0, coord + ivec2( -halfScaleFloor, -halfScaleFloor )));\n" // bottomLeftUV
			"    n[1] = (accessTexel(colorTexture0, coord + ivec2( halfScaleCeil, halfScaleCeil )));\n" // topRightUV
			"    n[2] = (accessTexel(colorTexture0, coord + ivec2( halfScaleCeil, -halfScaleFloor )));\n" // bottomRightUV
			"    n[3] = (accessTexel(colorTexture0, coord + ivec2( -halfScaleFloor, halfScaleCeil )));\n" // topLeftUV
			"}\n"
			""
			"float edgeNormal(in vec4 n[4], float threshold) {\n"
			"\n"
			"    vec3 n0 = n[0].rgb;\n"
			"    vec3 n1 = n[1].rgb;\n"
			"    vec3 n2 = n[2].rgb;\n"
			"    vec3 n3 = n[3].rgb;\n"
			"\n"
			"    vec3 d0 = (n1 - n0);\n"
			"    vec3 d1 = (n3 - n2);\n"
			"\n"
			"    float edge = sqrt(dot(d0, d0) + dot(d1, d1));\n"
			"    return edge;\n"
			"}\n"

			"\n"

			"float sobelIntensity(in vec4 n[9]) {\n"
			"	vec4 sobel_edge_h = n[2] + (2.0*n[5]) + n[8] - (n[0] + (2.0*n[3]) + n[6]);\n"
			"	vec4 sobel_edge_v = n[0] + (2.0*n[1]) + n[2] - (n[6] + (2.0*n[7]) + n[8]);\n"
			"	vec4 sobel = sqrt((sobel_edge_h * sobel_edge_h) + (sobel_edge_v * sobel_edge_v));\n"
			"	return length(sobel.rgb);\n"
			"}\n"

			"float sobelSlphaIntensity(in vec4 n[9]) {\n"
			"	float sobel_edge_h = n[2].a + (2.0*n[5].a) + n[8].a - (n[0].a + (2.0*n[3].a) + n[6].a);\n"
			"	float sobel_edge_v = n[0].a + (2.0*n[1].a) + n[2].a - (n[6].a + (2.0*n[7].a) + n[8].a);\n"
			"	float sobel = sqrt((sobel_edge_h * sobel_edge_h) + (sobel_edge_v * sobel_edge_v));\n"
			"	return sobel;\n"
			"}\n"

			"void main()\n"
			"{\n"
			" 	 vec4 samples[4];"
			" 	 vec4 texelCenter = accessTexel(colorTexture0, ivec2(gl_FragCoord.x, gl_FragCoord.y));\n"
			" 	 getNeighbors4(samples, ivec2(gl_FragCoord.x, gl_FragCoord.y));\n"
			" 	 float depthCenterValue = texelCenter.a;\n"
			"    bool selected = false;\n"
			"\n"
			"    vec3 normal = normalize(vec3(texelCenter.x, texelCenter.y, texelCenter.z));" // we store normals in 0,1 range, need to unpack to -1,1 range
			 "   float viewDot = 1 - dot(normalize(normal), normalize(vec3(-clipPos.xy,1)));\n"
			 "   float normalThreshold01 = clamp((viewDot - depthNormalThreshold) / (1 - depthNormalThreshold), 0.0, 1.0);\n"
			 "   float normalThreshold = normalThreshold01 * depthNormalThresholdScale + 1;\n"
			 "   float eNormal = edgeNormal(samples, 0);\n"
			 //"   eNormal = eNormal > 0.8 ? 1.0 : 0.0;\n"
			 "   float edgeDepth = 20*(sqrt((samples[1].a - samples[0].a) * (samples[1].a - samples[0].a)) + ((samples[3].a - samples[2].a)*(samples[3].a - samples[2].a)));//sobelSlphaIntensity(samples);//\n"
			 "   float depthThreshold = depthThreshold * depthCenterValue * normalThreshold;\n"
			 "   edgeDepth = edgeDepth > depthThreshold ? 1.0 : 0.0;\n"
			 "   float edge = max(edgeDepth, eNormal);\n"
#if			SHOW_EDGE_DETECTION_PASS_ONLY
			"    gl_FragColor = vec4(edgeDepth, edgeDepth, edgeDepth, 1.0);\n"
		    "    return;\n"
#else
			"    if(selected) {"
			"  		  gl_FragColor = vec4(selectionColor.xyz, edge * highlightIntensity*0.5 + 0.08);\n" // sum a constant offset in alpha so to create the filling highlight effect
			"         return;\n"
			"    }\n"
		    "    gl_FragColor = vec4(outlineColor, edge);\n"
			//"    gl_FragColor = vec4(edge, edge, edge, 1.0);\n"
#endif
			"}\n";

	const std::string preRenderFragShaderStr =
			"#version 120\n"
			"\n"
			"varying vec3 normal;\n"


			"\n"
			"varying vec3 position;\n"
			"varying vec4 color;\n"
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
			"float depth = gl_FragCoord.w;"
			" gl_FragColor = vec4(normalize(normal), depth);\n"
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
			"    normal = (gl_ModelViewMatrix * vec4(gl_Normal, 0)).xyz;\n"
			"    vec4 inPos = vec4(gl_Vertex.xyz, 1);"
			"    position = (gl_ModelViewMatrix * inPos).xyz;\n"
			"    // Calculate vertex position in clip coordinates\n"
			"    gl_Position = gl_ModelViewProjectionMatrix * inPos;\n"
			"}\n";

	const std::string fxaaRenderFragShaderStr =
		"#version 120\n"
		"#extension GL_ARB_texture_multisample : enable \n"
		"\n"
		"uniform vec2 viewportSize;\n"
		"uniform sampler2DMS colorTexture0;\n"
		"\n"
		"\n"
		"varying vec2 v_rgbNW;\n"
		"varying vec2 v_rgbNE;\n"
		"varying vec2 v_rgbSW;\n"
		"varying vec2 v_rgbSE;\n"
		"varying vec2 v_rgbM;\n"
		"varying vec2 texCoord;\n"
		"\n"
		"\n"
		"#ifndef FXAA_REDUCE_MIN\n"
		"    #define FXAA_REDUCE_MIN   (1.0/ 128.0)\n"
		"#endif\n"
		"#ifndef FXAA_REDUCE_MUL\n"
		"    #define FXAA_REDUCE_MUL   (1.0 / 8.0)\n"
		"#endif\n"
		"#ifndef FXAA_SPAN_MAX\n"
		"    #define FXAA_SPAN_MAX     8.0\n"
		"#endif\n"
		"\n"
		"vec4 accessTexel(sampler2DMS tex, vec2 coord) {\n"
		"	ivec2 tc = ivec2(floor(textureSize(tex) * coord));\n"
		"	vec4 c = texelFetch(tex, tc, 0) + texelFetch(tex, tc, 1) + texelFetch(tex, tc, 2) + texelFetch(tex, tc, 3);\n"
    	"	return c / 4.0;\n"
		//"   return texelFetch(tex, tc, 0);"
		"}\n"
		"\n"
		"//optimized version for mobile, where dependent \n"
		"//texture reads can be a bottleneck\n"
		"vec4 fxaa(sampler2DMS tex, vec2 fragCoord, vec2 resolution,\n"
		"            vec2 v_rgbNW, vec2 v_rgbNE, \n"
		"            vec2 v_rgbSW, vec2 v_rgbSE, \n"
		"            vec2 v_rgbM) {\n"
		"    vec4 color;\n"
		"    mediump vec2 inverseVP = vec2(1.0 / resolution.x, 1.0 / resolution.y);\n"
		"    vec3 rgbNW = accessTexel(tex, v_rgbNW).xyz;\n"
		"    vec3 rgbNE = accessTexel(tex, v_rgbNE).xyz;\n"
		"    vec3 rgbSW = accessTexel(tex, v_rgbSW).xyz;\n"
		"    vec3 rgbSE = accessTexel(tex, v_rgbSE).xyz;\n"
		"    vec4 texColor = accessTexel(tex, v_rgbM);\n"
		"    vec3 rgbM  = texColor.xyz;\n"
		"    vec3 luma = vec3(0.299, 0.587, 0.114);\n"
		"    float lumaNW = dot(rgbNW, luma);\n"
		"    float lumaNE = dot(rgbNE, luma);\n"
		"    float lumaSW = dot(rgbSW, luma);\n"
		"    float lumaSE = dot(rgbSE, luma);\n"
		"    float lumaM  = dot(rgbM,  luma);\n"
		"    float lumaMin = min(lumaM, min(min(lumaNW, lumaNE), min(lumaSW, lumaSE)));\n"
		"    float lumaMax = max(lumaM, max(max(lumaNW, lumaNE), max(lumaSW, lumaSE)));\n"
		"    \n"
		"    mediump vec2 dir;\n"
		"    dir.x = -((lumaNW + lumaNE) - (lumaSW + lumaSE));\n"
		"    dir.y =  ((lumaNW + lumaSW) - (lumaNE + lumaSE));\n"
		"    \n"
		"    float dirReduce = max((lumaNW + lumaNE + lumaSW + lumaSE) *\n"
		"                          (0.25 * FXAA_REDUCE_MUL), FXAA_REDUCE_MIN);\n"
		"    \n"
		"    float rcpDirMin = 1.0 / (min(abs(dir.x), abs(dir.y)) + dirReduce);\n"
		"    dir = min(vec2(FXAA_SPAN_MAX, FXAA_SPAN_MAX),\n"
		"              max(vec2(-FXAA_SPAN_MAX, -FXAA_SPAN_MAX),\n"
		"              dir * rcpDirMin)) * inverseVP;\n"
		"    \n"
		"    vec3 rgbA = 0.5 * (\n"
		"        accessTexel(tex, fragCoord * inverseVP + dir * (1.0 / 3.0 - 0.5)).xyz +\n"
		"        accessTexel(tex, fragCoord * inverseVP + dir * (2.0 / 3.0 - 0.5)).xyz);\n"
		"    vec3 rgbB = rgbA * 0.5 + 0.25 * (\n"
		"        accessTexel(tex, fragCoord * inverseVP + dir * -0.5).xyz +\n"
		"        accessTexel(tex, fragCoord * inverseVP + dir * 0.5).xyz);\n"
		"\n"
		"    float lumaB = dot(rgbB, luma);\n"
		"    if ((lumaB < lumaMin) || (lumaB > lumaMax))\n"
		"        color = vec4(rgbA, texColor.a);\n"
		"    else\n"
		"        color = vec4(rgbB, texColor.a);\n"
		"    return color;\n"
		"}\n"
		"\n"
		"void main()\n"
		"{\n"
		"   vec2 fragCoord = vec2(gl_FragCoord.x, gl_FragCoord.y) / viewportSize;\n"
		//"   gl_FragColor = fxaa(colorTexture0, fragCoord * viewportSize, viewportSize, v_rgbNW, v_rgbNE, v_rgbSW, v_rgbSE, v_rgbM);\n"
		"   gl_FragColor = accessTexel(colorTexture0,  fragCoord);\n"
		//"  gl_FragColor = vec4(fragCoord.xy, 0, 1);\n"
		"}\n";

	const std::string fxaaRenderVertShaderStr =
		"#version 120\n"
		"#extension GL_ARB_texture_multisample : enable \n"
		"\n"
		"uniform mat4 modelMatrix;\n"
		"uniform vec2 viewportSize;\n"
		"uniform sampler2DMS colorTexture0;\n"
		"\n"
		"varying vec2 v_rgbNW;\n"
		"varying vec2 v_rgbNE;\n"
		"varying vec2 v_rgbSW;\n"
		"varying vec2 v_rgbSE;\n"
		"varying vec2 v_rgbM;\n"
		"varying vec2 texCoord;\n"
		"\n"
		"\n"
		"void texcoords(vec2 fragCoord, vec2 resolution,\n"
		"\t\t\tout vec2 v_rgbNW, out vec2 v_rgbNE,\n"
		"\t\t\tout vec2 v_rgbSW, out vec2 v_rgbSE,\n"
		"\t\t\tout vec2 v_rgbM) {\n"
		"\tvec2 inverseVP = 1.0 / resolution.xy;\n"
		"\tv_rgbNW = (fragCoord + vec2(-1.0, -1.0)) * inverseVP;\n"
		"\tv_rgbNE = (fragCoord + vec2(1.0, -1.0)) * inverseVP;\n"
		"\tv_rgbSW = (fragCoord + vec2(-1.0, 1.0)) * inverseVP;\n"
		"\tv_rgbSE = (fragCoord + vec2(1.0, 1.0)) * inverseVP;\n"
		"\tv_rgbM = vec2(fragCoord * inverseVP);\n"
		"}\n"
		"void main()\n"
		"{\n"
		"    vec4 viewPos = vec4(gl_Vertex.xyz, 1.0);\n"
		"    texCoord = (viewPos.xy*0.5 + vec2(0.5));\n"
		"    texcoords(texCoord * viewportSize, viewportSize, v_rgbNW, v_rgbNE, v_rgbSW, v_rgbSE, v_rgbM);\n"
		"    gl_Position = viewPos;\n"
		"}\n";

}

void RenderPassState::HandleResize(int width, int height)
{
	if(camera) {
		camera->setRenderingCache(0);
		camera->setViewport(0, 0, width, height);
		state->addUniform(new osg::Uniform("viewportSize", osg::Vec2f(width, height)));
		for(unsigned int i = 0; i < colorFboTextures.size(); ++i) {
			if(colorFboTextures[i]) {
				colorFboTextures[i]->setTextureSize(width, height);
			}
		}
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

RenderPassState* OutlineShaderPipeline::createSceneToTexturePass(osg::ref_ptr<osg::Camera> mainSceneCamera, osg::ref_ptr<osg::Node> mainSceneRoot, int numColorAttachments, const std::string& vshader, const std::string& fshader)
{
	// First pass will render the same scene using a special shader that render objects with different colors
	// different from background, so to prepare for outline edge detection post processing shader
	RenderPassState* renderPassState = new RenderPassState();
	renderPassState->state = new osg::StateSet();
	RenderUtils::SetShaderProgramOnStateSet(renderPassState->state.get(), vshader, fshader);
	osg::ref_ptr<osg::Group> firstPassGroup = new osg::Group();
	firstPassGroup->setStateSet(renderPassState->state.get());
	firstPassGroup->addChild(mainSceneRoot);

	// clone main camera settings so to render same scene
	renderPassState->camera = new osg::Camera();
	renderPassState->colorFboTextures = RenderUtils::SetupRenderToTextureCamera(renderPassState->camera);

	renderPassState->camera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	renderPassState->camera->setClearColor(osg::Vec4(0, 0, 0, 0));
	renderPassState->camera->setViewMatrix(osg::Matrix::identity());
	renderPassState->camera->setProjectionMatrix(osg::Matrix::identity());
	// add outline camera as child of original scene camera so we can iherit transform and render same scene in the first pass (except we will render to texture)
	mainSceneCamera->addChild(renderPassState->camera.get());
	renderPassState->camera->addChild(firstPassGroup.get());
	renderPassState->state->addUniform(new osg::Uniform("isSelected", 0));
	// disable blend because we use alpha channel for encoding other information
	renderPassState->state->setMode(GL_BLEND, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	return renderPassState;
}

RenderPassState* OutlineShaderPipeline::createTextureToTexturePass(RenderPassState* inputPass, int numColorAttachments, const std::string& vshader, const std::string& fshader)
{
	RenderPassState* renderPassState = new RenderPassState();
	renderPassState->state = new osg::StateSet();
	RenderUtils::SetShaderProgramOnStateSet(renderPassState->state.get(), vshader, fshader);

	renderPassState->camera = RenderUtils::CreateTextureDisplayQuadCamera(osg::Vec3(-1.0, -1.0, 0), renderPassState->state.get());

	std::vector<osg::ref_ptr<osg::Texture2DMultisample>> inputTextures = inputPass->colorFboTextures;
	for(unsigned int i = 0; i < inputTextures.size(); ++i) {
		std::ostringstream bufferNumStr;
		bufferNumStr << "colorTexture";
		bufferNumStr << i;
		renderPassState->state->setTextureAttributeAndModes(0, inputTextures[i].get(), osg::StateAttribute::ON);
		renderPassState->state->addUniform(new osg::Uniform(bufferNumStr.str().c_str(), i));

	}

	renderPassState->colorFboTextures = RenderUtils::SetupRenderToTextureCamera(renderPassState->camera);

	// add outline camera as child of original scene camera so we can iherit transform and render same scene in the first pass (except we will render to texture)
	renderPassState->state->addUniform(new osg::Uniform("outlineColor", _outlineColor));
	renderPassState->state->addUniform(new osg::Uniform("selectionColor", _selectedObjectHighlightColor));
	renderPassState->state->addUniform(new osg::Uniform("isSelected", 0));
	renderPassState->state->addUniform(new osg::Uniform("depthThreshold", 1.5f));
	renderPassState->state->addUniform(new osg::Uniform("depthNormalThreshold", 0.5f));
	renderPassState->state->addUniform(new osg::Uniform("depthNormalThresholdScale", 7.0f));
	renderPassState->state->addUniform(new osg::Uniform("normalThreshold", 0.8f));
	return renderPassState;
}

RenderPassState* OutlineShaderPipeline::createTextureToColorBufferPass(RenderPassState* inputPass, int numColorAttachments, const std::string& vshader, const std::string& fshader)
{
	RenderPassState* renderPassState = new RenderPassState();
	renderPassState->state = new osg::StateSet();
	RenderUtils::SetShaderProgramOnStateSet(renderPassState->state.get(), outlineVertStr, outlineFragStr);
	renderPassState->camera = RenderUtils::CreateTextureDisplayQuadCamera(osg::Vec3(-1.0, -1.0, 0), renderPassState->state.get());
	std::vector<osg::ref_ptr<osg::Texture2DMultisample>> inputTextures = inputPass->colorFboTextures;
	for(unsigned int i = 0; i < inputTextures.size(); ++i) {
		std::ostringstream bufferNumStr;
		bufferNumStr << "colorTexture";
		bufferNumStr << i;
		renderPassState->state->setTextureAttributeAndModes(0, inputTextures[i].get(), osg::StateAttribute::ON);
		renderPassState->state->addUniform(new osg::Uniform(bufferNumStr.str().c_str(), i));

	}

    renderPassState->state->setMode(GL_BLEND, osg::StateAttribute::ON);
	renderPassState->state->setAttributeAndModes(new osg::BlendFunc(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA ));
	osg::Depth* depth = new osg::Depth;
	depth->setWriteMask( false );
	renderPassState->state->setAttributeAndModes(depth, osg::StateAttribute::ON);
	renderPassState->state->addUniform(new osg::Uniform("outlineColor", _outlineColor));
	renderPassState->state->addUniform(new osg::Uniform("selectionColor", _selectedObjectHighlightColor));
	renderPassState->state->addUniform(new osg::Uniform("isSelected", 0));
	renderPassState->state->addUniform(new osg::Uniform("highlightIntensity", _selectedObjectHighlightIntensity));
	renderPassState->state->addUniform(new osg::Uniform("depthThreshold", 0.2f));
	renderPassState->state->addUniform(new osg::Uniform("depthNormalThreshold", 0.5f));
	renderPassState->state->addUniform(new osg::Uniform("depthNormalThresholdScale", 7.0f));
	renderPassState->state->addUniform(new osg::Uniform("normalThreshold", 0.4f));

	return renderPassState;
}


// // First pass is render the scene using special shaders to enhance edges
// inline RenderPassState* OutlineShaderPipeline::createFirstRenderPass(osg::ref_ptr<osg::Camera> mainSceneCamera, osg::ref_ptr<osg::Node> mainSceneRoot, int maxFBOBufferWidth, int maxFBOBufferHeight)
// {
// 	// First pass will render the same scene using a special shader that render objects with different colors
// 	// different from background, so to prepare for outline edge detection post processing shader
// 	RenderPassState* renderPassState = new RenderPassState();
// 	renderPassState->state = new osg::StateSet();
// 	RenderUtils::SetShaderProgramOnStateSet(renderPassState->state.get(), preRenderVertShaderStr, preRenderFragShaderStr);
// 	osg::ref_ptr<osg::Group> firstPassGroup = new osg::Group();
// 	firstPassGroup->setStateSet(renderPassState->state.get());
// 	firstPassGroup->addChild(mainSceneRoot);

// 	// clone main camera settings so to render same scene
// 	renderPassState->camera = new osg::Camera();
// 	renderPassState->colorFboTexture = RenderUtils::CreateFloatTextureRectangle(maxFBOBufferWidth, maxFBOBufferHeight);
// #	if !SHOW_PRERENDER_SCENE_ONLY
// 	RenderUtils::SetupRenderToTextureCamera(renderPassState->camera, osg::Camera::COLOR_BUFFER, renderPassState->colorFboTexture.get());
// #endif
// 	renderPassState->camera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
// 	renderPassState->camera->setClearColor(osg::Vec4(0, 0, 0, 0));
// 	renderPassState->camera->setViewMatrix(osg::Matrix::identity());
// 	renderPassState->camera->setProjectionMatrix(osg::Matrix::identity());
// 	// add outline camera as child of original scene camera so we can iherit transform and render same scene in the first pass (except we will render to texture)
// 	mainSceneCamera->addChild(renderPassState->camera.get());
// 	renderPassState->camera->addChild(firstPassGroup.get());
// 	renderPassState->state->addUniform(new osg::Uniform("isSelected", 0));
// 	// disable blend because we use alpha channel for encoding other information
// 	renderPassState->state->setMode(GL_BLEND, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
// 	return renderPassState;
// }

// RenderPassState* OutlineShaderPipeline::createSecondRenderPass(RenderPassState* firstPassState, int maxFBOBufferWidth, int maxFBOBufferHeight)
// {
// 	RenderPassState* renderPassState = new RenderPassState();
// 	renderPassState->state = new osg::StateSet();
// 	RenderUtils::SetShaderProgramOnStateSet(renderPassState->state.get(), outlineVertStr, outlineFragStr);

// 	renderPassState->camera = RenderUtils::CreateTextureDisplayQuadCamera(osg::Vec3(-1.0, -1.0, 0), renderPassState->state.get());
// 	renderPassState->colorFboTexture = RenderUtils::CreateFloatTextureRectangle(maxFBOBufferWidth, maxFBOBufferHeight);
// 	RenderUtils::SetupRenderToTextureCamera(renderPassState->camera, osg::Camera::COLOR_BUFFER, renderPassState->colorFboTexture.get());

// 	// add outline camera as child of original scene camera so we can iherit transform and render same scene in the first pass (except we will render to texture)
// 	renderPassState->state->setTextureAttributeAndModes(0, firstPassState->colorFboTexture.get(), osg::StateAttribute::ON);
//     renderPassState->state->addUniform(new osg::Uniform("colorTexture0", 0));
// 	renderPassState->state->addUniform(new osg::Uniform("outlineColor", _outlineColor));
// 	renderPassState->state->addUniform(new osg::Uniform("selectionColor", _selectedObjectHighlightColor));
// 	renderPassState->state->addUniform(new osg::Uniform("isSelected", 0));
// 	renderPassState->state->addUniform(new osg::Uniform("depthThreshold", 1.5f));
// 	renderPassState->state->addUniform(new osg::Uniform("depthNormalThreshold", 0.5f));
// 	renderPassState->state->addUniform(new osg::Uniform("depthNormalThresholdScale", 7.0f));
// 	renderPassState->state->addUniform(new osg::Uniform("normalThreshold", 0.8f));

// 	return renderPassState;
// }

// RenderPassState* OutlineShaderPipeline::createThirdRenderPass(RenderPassState* secondPassState, int maxFBOBufferWidth, int maxFBOBufferHeight)
// {
// 	RenderPassState* renderPassState = new RenderPassState();
// 	renderPassState->state = new osg::StateSet();
// 	RenderUtils::SetShaderProgramOnStateSet(renderPassState->state.get(), outlineVertStr, outlineFragStr);
// 	renderPassState->camera = RenderUtils::CreateTextureDisplayQuadCamera(osg::Vec3(-1.0, -1.0, 0), renderPassState->state.get());
// 	renderPassState->state->setTextureAttributeAndModes(0, secondPassState->colorFboTexture.get(), osg::StateAttribute::ON);
//     renderPassState->state->addUniform(new osg::Uniform("colorTexture0", 0));
// 	renderPassState->state->setMode(GL_BLEND, osg::StateAttribute::ON);
// 	renderPassState->state->setAttributeAndModes(new osg::BlendFunc(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA ));
// 	osg::Depth* depth = new osg::Depth;
// 	depth->setWriteMask( false );
// 	renderPassState->state->setAttributeAndModes(depth, osg::StateAttribute::ON);
// 	renderPassState->state->addUniform(new osg::Uniform("outlineColor", _outlineColor));
// 	renderPassState->state->addUniform(new osg::Uniform("selectionColor", _selectedObjectHighlightColor));
// 	renderPassState->state->addUniform(new osg::Uniform("isSelected", 0));
// 	renderPassState->state->addUniform(new osg::Uniform("highlightIntensity", _selectedObjectHighlightIntensity));
// 	renderPassState->state->addUniform(new osg::Uniform("depthThreshold", 0.2f));
// 	renderPassState->state->addUniform(new osg::Uniform("depthNormalThreshold", 0.5f));
// 	renderPassState->state->addUniform(new osg::Uniform("depthNormalThresholdScale", 7.0f));
// 	renderPassState->state->addUniform(new osg::Uniform("normalThreshold", 0.4f));

// 	return renderPassState;
// }

osg::ref_ptr<osg::Group> OutlineShaderPipeline::CreateOutlineSceneFromOriginalScene(osg::ref_ptr<osg::Camera> mainSceneCamera, 
	osg::ref_ptr<osg::Node> mainSceneRoot, int maxFBOBufferWidth, int maxFBOBufferHeight)
{
	setMaxFBOSize(maxFBOBufferWidth, maxFBOBufferHeight);
	RenderPassState* normalAndDepthMapPass = createSceneToTexturePass(mainSceneCamera, mainSceneRoot, 1, preRenderVertShaderStr, preRenderFragShaderStr);
	RenderPassState* outlinePass = createTextureToColorBufferPass(normalAndDepthMapPass, 1, outlineVertStr, outlineFragStr);

	_renderPassStates.push_back(normalAndDepthMapPass);
	_renderPassStates.push_back(outlinePass);


	osg::ref_ptr<osg::Group> passesGroup = new osg::Group();
	passesGroup->addChild(_renderPassStates[0]->camera.get());
#	if !SHOW_PRERENDER_SCENE_ONLY
	passesGroup->addChild(mainSceneRoot);
	passesGroup->addChild(_renderPassStates[1]->camera.get());
	//passesGroup->addChild(_renderPassStates[2]->camera.get());
#endif
	return passesGroup.release();

}
void OutlineShaderPipeline::HandleResize(int width, int height)
{
	for(RenderPassState* state : _renderPassStates) {
		state->HandleResize(width, height);
	}
}
