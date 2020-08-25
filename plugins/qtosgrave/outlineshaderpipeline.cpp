
#include "renderutils.h"
#include "outlineshaderpipeline.h"
#include <sstream>

#include <QObject>
#include <osg/Depth>
#include <osg/CullFace>
#include <osg/BlendFunc>
#include <QFileSystemWatcher>

#include "qtosg.h"

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
	osg::ref_ptr<osg::Node> mainSceneRoot, int numColorAttachments, const std::string& vshader, const std::string& fshader, osg::Texture* reusedDepthTexture, bool useMultiSamples)
{
	RenderPassState* renderPassState = new RenderPassState();
	renderPassState->SetShaderFiles(vshader, fshader, true);
	osg::ref_ptr<osg::Group> firstPassGroup = new osg::Group();
	firstPassGroup->setStateSet(renderPassState->state.get());
	firstPassGroup->addChild(mainSceneRoot);

	// clone main camera settings so to render same scene
	renderPassState->camera = new osg::Camera();
	RenderUtils::FBOData fboData;
	RenderUtils::SetupRenderToTextureCamera(renderPassState->camera, _maxFBOBufferWidth, _maxFBOBufferHeight, fboData, reusedDepthTexture, numColorAttachments, useMultiSamples);
	renderPassState->colorFboTextures = fboData.colorTextures;
	renderPassState->depthFboTexture = fboData.depthTexture;

	renderPassState->camera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	renderPassState->camera->setClearColor(osg::Vec4(0, 0, 0, 0));
	renderPassState->camera->setViewMatrix(osg::Matrix::identity());
	renderPassState->camera->setProjectionMatrix(osg::Matrix::identity());
	// add outline camera as child of original scene camera so we can iherit transform and render same scene in the first pass (except we will render to texture)
	mainSceneCamera->addChild(renderPassState->camera.get());
	renderPassState->camera->addChild(firstPassGroup.get());
    renderPassState->state->setMode(GL_BLEND, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	_SetupOutlineShaderUniforms(renderPassState);
	return renderPassState;
}

RenderPassState* OutlineShaderPipeline::CreateTextureToTexturePass(RenderPassState* inputPass, int numColorAttachments, const std::string& vshader, const std::string& fshader, osg::Texture* reusedDepthTexture)
{
	RenderPassState* renderPassState = new RenderPassState();
	renderPassState->camera = RenderUtils::CreateTextureDisplayQuadCamera(osg::Vec3(-1.0, -1.0, 0), renderPassState->state.get());
	renderPassState->camera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
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

	RenderUtils::FBOData fboData;
	RenderUtils::SetupRenderToTextureCamera(renderPassState->camera, _maxFBOBufferWidth, _maxFBOBufferHeight, fboData, reusedDepthTexture, numColorAttachments);
	renderPassState->colorFboTextures = fboData.colorTextures;
	renderPassState->depthFboTexture = fboData.depthTexture;
	_SetupOutlineShaderUniforms(renderPassState);
	return renderPassState;
}

RenderPassState* OutlineShaderPipeline::CreateTextureToColorBufferPass(RenderPassState* inputPass, int numColorAttachments, const std::string& vshader, const std::string& fshader)
{
	RenderPassState* renderPassState = new RenderPassState();
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

	_SetupOutlineShaderUniforms(renderPassState);
	return renderPassState;
}

void OutlineShaderPipeline::_SetupOutlineShaderUniforms(RenderPassState* pass)
{
	pass->state->addUniform(new osg::Uniform("outlineColor", _outlineColor));
	pass->state->addUniform(new osg::Uniform("selectionColor", _selectedObjectHighlightColor));
	pass->state->addUniform(new osg::Uniform("isSelected", 0));
	pass->state->addUniform(new osg::Uniform("outlineEnabled", true));
	pass->state->addUniform(new osg::Uniform("depthThreshold", 1.5f));
	pass->state->addUniform(new osg::Uniform("depthNormalThreshold", 0.5f));
	pass->state->addUniform(new osg::Uniform("depthNormalThresholdScale", 7.0f));
	pass->state->addUniform(new osg::Uniform("normalThreshold", g_NormalThreshold));
	pass->state->addUniform(new osg::Uniform("textureSize", osg::Vec2f(_maxFBOBufferWidth, _maxFBOBufferHeight)));
	pass->state->addUniform(new osg::Uniform("osg_MaterialDiffuseColor", osg::Vec4f(0,0,0,1)));
}

osg::ref_ptr<osg::Group> OutlineShaderPipeline::CreateOutlineSceneFromOriginalScene(osg::ref_ptr<osg::Camera> mainSceneCamera,
	osg::ref_ptr<osg::Node> mainSceneRoot, int maxFBOBufferWidth, int maxFBOBufferHeight)
{
	setMaxFBOSize(maxFBOBufferWidth, maxFBOBufferHeight);
	static const std::string preRenderVertShaderPath = "/data/shaders/prerender.vert";
	static const std::string preRenderFragShaderPath = "/data/shaders/prerender.frag";
	static const std::string outlineVertPath = "/data/shaders/outline.vert";
	static const std::string outlineFragPath = "/data/shaders/outline.frag";
	static const std::string depthTestVertPath = "/data/shaders/depthtest.vert";
	static const std::string depthTestFragPath = "/data/shaders/depthtest.frag";

	mainSceneCamera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
    osg::ref_ptr<osg::Texture> depthBuffer = RenderUtils::CreateDepthFloatTextureRectangle(maxFBOBufferWidth, maxFBOBufferHeight);
	RenderPassState* normalAndDepthMapPass = CreateSceneToTexturePass(mainSceneCamera, mainSceneRoot, 3, preRenderVertShaderPath, preRenderFragShaderPath, nullptr, true);
	normalAndDepthMapPass->camera->setCullMask(~qtosgrave::TRANSPARENT_ITEM_MASK);
	normalAndDepthMapPass->state->setAttributeAndModes(new osg::BlendFunc(osg::BlendFunc::SRC_ALPHA, osg::BlendFunc::ONE_MINUS_SRC_ALPHA ));
	normalAndDepthMapPass->state->setMode(GL_BLEND, osg::StateAttribute::ON);
	normalAndDepthMapPass->state->setRenderBinDetails(0, "DepthSortedBin", osg::StateSet::USE_RENDERBIN_DETAILS);

	RenderPassState* outlinePass = CreateTextureToColorBufferPass(normalAndDepthMapPass, 1, outlineVertPath, outlineFragPath);
	// RenderPassState* normalAndDepthMapPassTransparency = CreateSceneToTexturePass(mainSceneCamera, mainSceneRoot, 3, preRenderVertShaderPath, preRenderFragShaderPath, depthBuffer.get(), true);
	// normalAndDepthMapPassTransparency->camera->setClearMask(GL_COLOR_BUFFER_BIT);
	// normalAndDepthMapPassTransparency->camera->setCullMask(qtosgrave::TRANSPARENT_ITEM_MASK);

	// RenderPassState* outlinePassTransparency = CreateTextureToColorBufferPass(normalAndDepthMapPassTransparency, 1, depthTestVertPath, depthTestFragPath);

	// normalAndDepthMapPassTransparency->state->setTextureAttributeAndModes(5, depthBuffer.get(), osg::StateAttribute::ON);
	// normalAndDepthMapPassTransparency->state->addUniform(new osg::Uniform("depthTexture", 5));
	// outlinePassTransparency->state->setTextureAttributeAndModes(5, depthBuffer.get(), osg::StateAttribute::ON);
	// outlinePassTransparency->state->addUniform(new osg::Uniform("depthTexture", 5));

	_renderPassStates.push_back(normalAndDepthMapPass);
	_renderPassStates.push_back(outlinePass);
	// _renderPassStates.push_back(normalAndDepthMapPassTransparency);
	// _renderPassStates.push_back(outlinePassTransparency);

	osg::ref_ptr<osg::Group> passesGroup = new osg::Group();
	passesGroup->addChild(normalAndDepthMapPass->camera.get());
	// passesGroup->addChild(normalAndDepthMapPassTransparency->camera.get());
	osg::Group* grp = new osg::Group();
	//RenderUtils::SetShaderProgramFileOnStateSet(grp->getOrCreateStateSet(), "/data/shaders/perpixellighting.vert", "/data/shaders/perpixellighting.frag");
	// grp->addChild(mainSceneRoot);
	// passesGroup->addChild(grp);
	//passesGroup->addChild(outlinePassTransparency->camera.get());
	passesGroup->addChild(outlinePass->camera.get());



	return passesGroup.release();

}
void OutlineShaderPipeline::HandleResize(int width, int height)
{
	for(RenderPassState* state : _renderPassStates) {
		state->HandleResize(width, height);
	}
}
