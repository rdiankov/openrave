
#include "renderutils.h"
#include "outlineshaderpipeline.h"
#include <sstream>

#include <QObject>
#include <osg/Depth>
#include <osg/CullFace>
#include <osg/BlendFunc>
#include <QFileSystemWatcher>

class RenderStatePassShaderReloader {

public:
	RenderStatePassShaderReloader(RenderPassState* state)
	{
		_shaderFileWatcher.addPath(QString(state->vertShaderFile.c_str()));
		_shaderFileWatcher.addPath(QString(state->fragShaderFile.c_str()));

		QObject::connect(&_shaderFileWatcher, &QFileSystemWatcher::fileChanged, [=](const QString& file){
            state->ReloadShaders();
			std::cout << "file " << file.toStdString() << " has changed " << std::endl;
        });
	}

private:

	QFileSystemWatcher _shaderFileWatcher;

};

// state control
namespace {
	const float g_NormalThreshold = 0.8f;
}

RenderPassState::RenderPassState()
{
	state = new osg::StateSet();
	autoReloadShaders = false;
	_shaderReloader = nullptr;
}

void RenderPassState::HandleResize(int width, int height)
{
	if(camera) {
		camera->setRenderingCache(0);
		camera->setViewport(0, 0, width, height);
	}
}

void RenderPassState::SetShaderFiles(const std::string& vertShader, const std::string& fragShader, bool autoReload)
{
	vertShaderFile = vertShader;
	fragShaderFile = fragShader;
	RenderUtils::SetShaderProgramFileOnStateSet(state, vertShader, fragShader);
	autoReloadShaders = autoReload;
	if(autoReload) {
		_shaderReloader = new RenderStatePassShaderReloader(this);
	}
}
void RenderPassState::ReloadShaders()
{
	RenderUtils::SetShaderProgramFileOnStateSet(state, vertShaderFile, fragShaderFile);
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

RenderPassState* OutlineShaderPipeline::CreateSceneToTexturePass(osg::ref_ptr<osg::Camera> mainSceneCamera,
	osg::ref_ptr<osg::Node> mainSceneRoot, int numColorAttachments, const std::string& vshader, const std::string& fshader, bool useMultiSamples)
{
	// First pass will render the same scene using a special shader that render objects with different colors
	// different from background, so to prepare for outline edge detection post processing shader
	RenderPassState* renderPassState = new RenderPassState();
	//RenderUtils::SetShaderProgramOnStateSet(renderPassState->state.get(), vshader, fshader);
	renderPassState->SetShaderFiles(vshader, fshader, true);
	osg::ref_ptr<osg::Group> firstPassGroup = new osg::Group();
	firstPassGroup->setStateSet(renderPassState->state.get());
	firstPassGroup->addChild(mainSceneRoot);

	// clone main camera settings so to render same scene
	renderPassState->camera = new osg::Camera();
	renderPassState->colorFboTextures = RenderUtils::SetupRenderToTextureCamera(renderPassState->camera, _maxFBOBufferWidth, _maxFBOBufferHeight, numColorAttachments, useMultiSamples);

	renderPassState->camera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
	renderPassState->camera->setClearColor(osg::Vec4(0, 0, 0, 0));
	renderPassState->camera->setViewMatrix(osg::Matrix::identity());
	renderPassState->camera->setProjectionMatrix(osg::Matrix::identity());
	// add outline camera as child of original scene camera so we can iherit transform and render same scene in the first pass (except we will render to texture)
	mainSceneCamera->addChild(renderPassState->camera.get());
	renderPassState->camera->addChild(firstPassGroup.get());
	renderPassState->state->addUniform(new osg::Uniform("isSelected", 0));
	renderPassState->state->addUniform(new osg::Uniform("textureSize", osg::Vec2f(_maxFBOBufferWidth, _maxFBOBufferHeight)));
	renderPassState->state->addUniform(new osg::Uniform("osg_MaterialDiffuseColor", osg::Vec4f(0,0,0,1)));
	renderPassState->state->addUniform(new osg::Uniform("osg_MaterialAmbientColor", osg::Vec4f(0,0,0,1)));
	renderPassState->state->setRenderBinDetails(0, "RenderBin", osg::StateSet::OVERRIDE_RENDERBIN_DETAILS);
	// disable blend because we use alpha channel for encoding other information
	renderPassState->state->setMode(GL_BLEND, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	return renderPassState;
}

RenderPassState* OutlineShaderPipeline::CreateTextureToTexturePass(RenderPassState* inputPass, int numColorAttachments, const std::string& vshader, const std::string& fshader)
{
	RenderPassState* renderPassState = new RenderPassState();
	renderPassState->camera = RenderUtils::CreateTextureDisplayQuadCamera(osg::Vec3(-1.0, -1.0, 0), renderPassState->state.get());

	std::vector<osg::ref_ptr<osg::Texture>> inputTextures = inputPass->colorFboTextures;
	for(unsigned int i = 0; i < inputTextures.size(); ++i) {
		std::ostringstream bufferNumStr;
		bufferNumStr << "colorTexture";
		bufferNumStr << i;
		renderPassState->state->setTextureAttributeAndModes((int)i, inputTextures[i].get(), osg::StateAttribute::ON);
		renderPassState->state->addUniform(new osg::Uniform(bufferNumStr.str().c_str(), (int)i));
	}

	//RenderUtils::SetShaderProgramOnStateSet(renderPassState->state.get(), vshader, fshader);
	renderPassState->SetShaderFiles(vshader, fshader, true);
	renderPassState->colorFboTextures = RenderUtils::SetupRenderToTextureCamera(renderPassState->camera, _maxFBOBufferWidth, _maxFBOBufferHeight, numColorAttachments);

	// add outline camera as child of original scene camera so we can iherit transform and render same scene in the first pass (except we will render to texture)
	renderPassState->state->addUniform(new osg::Uniform("outlineColor", _outlineColor));
	renderPassState->state->addUniform(new osg::Uniform("selectionColor", _selectedObjectHighlightColor));
	renderPassState->state->addUniform(new osg::Uniform("isSelected", 0));
	renderPassState->state->addUniform(new osg::Uniform("depthThreshold", 1.5f));
	renderPassState->state->addUniform(new osg::Uniform("depthNormalThreshold", 0.5f));
	renderPassState->state->addUniform(new osg::Uniform("depthNormalThresholdScale", 7.0f));
	renderPassState->state->addUniform(new osg::Uniform("normalThreshold", g_NormalThreshold));
	renderPassState->state->addUniform(new osg::Uniform("textureSize", osg::Vec2f(_maxFBOBufferWidth, _maxFBOBufferHeight)));
	renderPassState->state->addUniform(new osg::Uniform("osg_MaterialDiffuseColor", osg::Vec4f(0,0,0,1)));
	renderPassState->state->addUniform(new osg::Uniform("osg_MaterialAmbientColor", osg::Vec4f(0,0,0,1)));
	return renderPassState;
}

RenderPassState* OutlineShaderPipeline::CreateTextureToColorBufferPass(RenderPassState* inputPass, int numColorAttachments, const std::string& vshader, const std::string& fshader)
{
	RenderPassState* renderPassState = new RenderPassState();
	//RenderUtils::SetShaderProgramOnStateSet(renderPassState->state.get(), vshader, fshader);
	renderPassState->SetShaderFiles(vshader, fshader, true);
	renderPassState->camera = RenderUtils::CreateTextureDisplayQuadCamera(osg::Vec3(-1.0, -1.0, 0), renderPassState->state.get());
	renderPassState->camera->setClearColor(osg::Vec4(1, 1, 1, 1));
	std::vector<osg::ref_ptr<osg::Texture>> inputTextures = inputPass->colorFboTextures;
	for(unsigned int i = 0; i < inputTextures.size(); ++i) {
		std::ostringstream bufferNumStr;
		bufferNumStr << "colorTexture";
		bufferNumStr << i;
		renderPassState->state->setTextureAttributeAndModes((int)i, inputTextures[i].get(), osg::StateAttribute::ON);
		renderPassState->state->addUniform(new osg::Uniform(bufferNumStr.str().c_str(), (int)i));
	}

    renderPassState->state->setMode(GL_BLEND, osg::StateAttribute::ON);
	renderPassState->state->setAttributeAndModes(new osg::BlendFunc(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA ));
	osg::Depth* depth = new osg::Depth;
	depth->setWriteMask( false );
	renderPassState->state->setAttributeAndModes(depth, osg::StateAttribute::ON);
	renderPassState->state->addUniform(new osg::Uniform("outlineColor", _outlineColor));
	renderPassState->state->addUniform(new osg::Uniform("selectionColor", _selectedObjectHighlightColor));
	renderPassState->state->addUniform(new osg::Uniform("isSelected", 0));
	renderPassState->state->addUniform(new osg::Uniform("depthThreshold", 1.5f));
	renderPassState->state->addUniform(new osg::Uniform("depthNormalThreshold", 0.5f));
	renderPassState->state->addUniform(new osg::Uniform("depthNormalThresholdScale", 7.0f));
	renderPassState->state->addUniform(new osg::Uniform("normalThreshold", g_NormalThreshold));
	renderPassState->state->addUniform(new osg::Uniform("textureSize", osg::Vec2f(_maxFBOBufferWidth, _maxFBOBufferHeight)));
	renderPassState->state->addUniform(new osg::Uniform("osg_MaterialDiffuseColor", osg::Vec4f(0,0,0,1)));

	return renderPassState;
}

osg::ref_ptr<osg::Group> OutlineShaderPipeline::CreateOutlineSceneFromOriginalScene(osg::ref_ptr<osg::Camera> mainSceneCamera,
	osg::ref_ptr<osg::Node> mainSceneRoot, int maxFBOBufferWidth, int maxFBOBufferHeight)
{
	setMaxFBOSize(maxFBOBufferWidth, maxFBOBufferHeight);
	static const std::string preRenderVertShaderPath = "/data/shaders/prerender.vert";
	static const std::string preRenderFragShaderPath = "/data/shaders/prerender.frag";
	static const std::string outlineVertPath = "/data/shaders/outline.vert";
	static const std::string outlineFragPath = "/data/shaders/outline.frag";
	static const std::string fxaaVertPath = "/data/shaders/fxaa.vert";
	static const std::string fxaaFragPath = "/data/shaders/fxaa.frag";
	RenderPassState* normalAndDepthMapPass = CreateSceneToTexturePass(mainSceneCamera, mainSceneRoot, 3, preRenderVertShaderPath, preRenderFragShaderPath, true);
	RenderPassState* outlinePass = CreateTextureToTexturePass(normalAndDepthMapPass, 1, outlineVertPath, outlineFragPath);
	RenderPassState* fxaaPass = CreateTextureToColorBufferPass(outlinePass, 1, fxaaVertPath, fxaaFragPath);

	// osg::Camera* normalHud = RenderUtils::CreateFBOTextureDisplayHUDViewPort(normalAndDepthMapPass->colorFboTextures[0].get(), osg::Vec2f(-1, 0), osg::Vec2f(1,1), maxFBOBufferWidth, maxFBOBufferHeight);
	// osg::Camera* depthHud = RenderUtils::CreateFBOTextureDisplayHUDViewPort(normalAndDepthMapPass->colorFboTextures[1].get(), osg::Vec2f(0,0), osg::Vec2f(1,1), maxFBOBufferWidth, maxFBOBufferHeight);
	// osg::Camera* outlineNoFxaaHud = RenderUtils::CreateFBOTextureDisplayHUDViewPort(outlinePass->colorFboTextures[0].get(), osg::Vec2f(-1,-1), osg::Vec2f(1,1), maxFBOBufferWidth, maxFBOBufferHeight);
	// osg::Camera* outlineFxaaHud = RenderUtils::CreateFBOTextureDisplayHUDViewPort(fxaaPass->colorFboTextures[0].get(), osg::Vec2f(0,-1), osg::Vec2f(1,1), maxFBOBufferWidth, maxFBOBufferHeight);

	_renderPassStates.push_back(normalAndDepthMapPass);
	_renderPassStates.push_back(outlinePass);
	_renderPassStates.push_back(fxaaPass);


	osg::ref_ptr<osg::Group> passesGroup = new osg::Group();
	passesGroup->addChild(_renderPassStates[0]->camera.get());
	//passesGroup->addChild(mainSceneRoot);
	passesGroup->addChild(_renderPassStates[1]->camera.get());
	passesGroup->addChild(_renderPassStates[2]->camera.get());
	// passesGroup->addChild(normalHud);
	// passesGroup->addChild(depthHud);
	// passesGroup->addChild(outlineNoFxaaHud);
	// passesGroup->addChild(outlineFxaaHud);

	return passesGroup.release();

}
void OutlineShaderPipeline::HandleResize(int width, int height)
{
	for(RenderPassState* state : _renderPassStates) {
		state->HandleResize(width, height);
	}
}
