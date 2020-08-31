
#include "renderutils.h"
#include "outlineshaderpipeline.h"
#include <sstream>

#include <QObject>
#include <osg/Depth>
#include <osg/CullFace>
#include <osg/BlendFunc>
#include <QFileSystemWatcher>

#include "qtosg.h"

#define DEBUG_FBO_BUFFERS 0

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
		camera->setViewport(0, 0, width, height);
	}

	// for(unsigned int i = 0; i < colorFboTextures.size(); ++i) {

	// 	if(!colorFboTextures[i]) {
	// 		continue;
	// 	}
	// 	osg::Texture2D* tex2D = dynamic_cast<osg::Texture2D*>(colorFboTextures[i].get());
	// 	if(tex2D) {
	// 		tex2D->setTextureSize(width, height);
	// 		std::cout << "RESIZING! "  << std::endl;
	// 	}
	// 	else {
	// 		dynamic_cast<osg::Texture2DMultisample*>(colorFboTextures[i].get())->setTextureSize(width, height);
	// 	}
	// }
	// state->addUniform(new osg::Uniform("textureSize", osg::Vec2f(width, height)));
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
	osg::ref_ptr<osg::Node> mainSceneRoot, int numColorAttachments, const std::string& vshader, const std::string& fshader, osg::Texture* reusedDepthTexture, bool useMultiSamples, const std::vector<osg::ref_ptr<osg::Texture>>& inheritedColorBuffers)
{
	RenderPassState* renderPassState = new RenderPassState();
	renderPassState->SetShaderFiles(vshader, fshader, true);
	osg::ref_ptr<osg::Group> firstPassGroup = new osg::Group();
	firstPassGroup->setStateSet(renderPassState->state.get());
	firstPassGroup->addChild(mainSceneRoot);

	// clone main camera settings so to render same scene
	renderPassState->camera = new osg::Camera();

	renderPassState->camera->setRenderingCache(0);
	RenderUtils::FBOData fboData;
	RenderUtils::SetupRenderToTextureCamera(renderPassState->camera, _maxFBOBufferWidth, _maxFBOBufferHeight, fboData, reusedDepthTexture, numColorAttachments, useMultiSamples, inheritedColorBuffers);
	renderPassState->colorFboTextures = fboData.colorTextures;
	renderPassState->depthFboTexture = fboData.depthTexture;

	renderPassState->camera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	renderPassState->camera->setClearColor(osg::Vec4(0, 0, 0, 0));
	renderPassState->camera->setViewMatrix(osg::Matrix::identity());
	renderPassState->camera->setProjectionMatrix(osg::Matrix::identity());
	// add outline camera as child of original scene camera so we can iherit transform and render same scene in the first pass (except we will render to texture)
	mainSceneCamera->addChild(renderPassState->camera.get());
	renderPassState->camera->addChild(firstPassGroup.get());
	_SetupOutlineShaderUniforms(renderPassState);
	return renderPassState;
}

RenderPassState* OutlineShaderPipeline::CreateTextureToTexturePass(RenderPassState* inputPass, int numColorAttachments, const std::string& vshader, const std::string& fshader, bool useMultiSamples, osg::Texture* reusedDepthTexture)
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
	RenderUtils::SetupRenderToTextureCamera(renderPassState->camera, _maxFBOBufferWidth, _maxFBOBufferHeight, fboData, reusedDepthTexture, numColorAttachments, useMultiSamples);
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
	renderPassState->camera->setClearColor(osg::Vec4(0, 0, 0, 0));
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
	pass->state->addUniform(new osg::Uniform("isSelected", 0));
	pass->state->addUniform(new osg::Uniform("outlineEnabled", false));
	pass->state->addUniform(new osg::Uniform("outlineColor", _outlineColor));
	pass->state->addUniform(new osg::Uniform("selectionColor", _selectedObjectHighlightColor));
	pass->state->addUniform(new osg::Uniform("osg_MaterialDiffuseColor", osg::Vec4f(0,0,0,1)));
	pass->state->addUniform(new osg::Uniform("textureSize", osg::Vec2f(_maxFBOBufferWidth, _maxFBOBufferHeight)));
}

osg::ref_ptr<osg::Group> OutlineShaderPipeline::CreateOutlineSceneFromOriginalScene(osg::ref_ptr<osg::Camera> mainSceneCamera,
	osg::ref_ptr<osg::Node> mainSceneRoot, int maxFBOBufferWidth, int maxFBOBufferHeight)
{
	setMaxFBOSize(1024, 1024);
	static const std::string preRenderVertShaderPath = "/data/shaders/prerender.vert";
	static const std::string preRenderFragShaderPath = "/data/shaders/prerender.frag";
	static const std::string preRenderTransparentVertShaderPath = "/data/shaders/prerender_transparent.vert";
	static const std::string preRenderTransparentFragShaderPath = "/data/shaders/prerender_transparent.frag";
	static const std::string outlineVertPath = "/data/shaders/outline.vert";
	static const std::string outlineFragPath = "/data/shaders/outline.frag";

    osg::ref_ptr<osg::Texture> depthBuffer = nullptr;
	mainSceneCamera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
	RenderPassState* normalAndDepthMapPass = CreateSceneToTexturePass(mainSceneCamera, mainSceneRoot, 1, preRenderVertShaderPath, preRenderFragShaderPath, depthBuffer.get(), true);
	normalAndDepthMapPass->camera->setCullMask(~qtosgrave::TRANSPARENT_ITEM_MASK);

#if DEBUG_FBO_BUFFERS
	RenderPassState* outlinePass = CreateTextureToTexturePass(normalAndDepthMapPass, 1, outlineVertPath, outlineFragPath);
#else
	RenderPassState* outlinePass = CreateTextureToColorBufferPass(normalAndDepthMapPass, 1, outlineVertPath, outlineFragPath);
#endif
	RenderPassState* normalAndDepthMapPassTransparency = CreateSceneToTexturePass(mainSceneCamera, mainSceneRoot, 1, preRenderTransparentVertShaderPath, preRenderTransparentFragShaderPath, depthBuffer.get(), true);
	normalAndDepthMapPassTransparency->camera->setCullMask(qtosgrave::TRANSPARENT_ITEM_MASK);

#if DEBUG_FBO_BUFFERS
	RenderPassState* outlinePassTransparency = CreateTextureToTexturePass(normalAndDepthMapPassTransparency, 1, outlineVertPath, outlineFragPath);
#else
	RenderPassState* outlinePassTransparency = CreateTextureToColorBufferPass(normalAndDepthMapPassTransparency, 1, outlineVertPath, outlineFragPath);
#endif
	// export opaque pipiline resulting depth texture to pre-render shader of transparent objects to perform z test
	normalAndDepthMapPassTransparency->state->setTextureAttributeAndModes(5, normalAndDepthMapPass->colorFboTextures[0].get(), osg::StateAttribute::ON);
	normalAndDepthMapPassTransparency->state->addUniform(new osg::Uniform("depthTexture", 5));
	_renderPassStates.push_back(normalAndDepthMapPass);
	_renderPassStates.push_back(outlinePass);
	_renderPassStates.push_back(normalAndDepthMapPassTransparency);
	_renderPassStates.push_back(outlinePassTransparency);

#if DEBUG_FBO_BUFFERS
	osg::Camera* fboInputDepthBuffer = RenderUtils::CreateFBOTextureDisplayHUDViewPort(depthBuffer.get(), osg::Vec2f(-1, 0), osg::Vec2f(1,1), _maxFBOBufferWidth, _maxFBOBufferHeight);
	osg::Camera* transparentObjectsPreRenderNormals = RenderUtils::CreateFBOTextureDisplayHUDViewPort(normalAndDepthMapPassTransparency->colorFboTextures[0].get(), osg::Vec2f(0, 0), osg::Vec2f(1,1), _maxFBOBufferWidth, _maxFBOBufferHeight);
	osg::Camera* opaqueObjectsEdges = RenderUtils::CreateFBOTextureDisplayHUDViewPort(outlinePass->colorFboTextures[0].get(), osg::Vec2f(-1, -1), osg::Vec2f(1,1), _maxFBOBufferWidth, _maxFBOBufferHeight);
	osg::Camera* transparentObjectsEdges = RenderUtils::CreateFBOTextureDisplayHUDViewPort(outlinePassTransparency->colorFboTextures[0].get(), osg::Vec2f(0, -1), osg::Vec2f(1,1), _maxFBOBufferWidth, _maxFBOBufferHeight);
#endif

	osg::ref_ptr<osg::Group> passesGroup = new osg::Group();
	passesGroup->addChild(normalAndDepthMapPass->camera.get());
	passesGroup->addChild(normalAndDepthMapPassTransparency->camera.get());


#if DEBUG_FBO_BUFFERS
	passesGroup->addChild(fboInputDepthBuffer);
	passesGroup->addChild(outlinePass->camera.get());
	passesGroup->addChild(outlinePassTransparency->camera.get());
	passesGroup->addChild(transparentObjectsPreRenderNormals);
	passesGroup->addChild(opaqueObjectsEdges);
	passesGroup->addChild(transparentObjectsEdges);
#else
	osg::Group* grp = new osg::Group();
	RenderUtils::SetShaderProgramFileOnStateSet(grp->getOrCreateStateSet(), "/data/shaders/perpixellighting.vert", "/data/shaders/perpixellighting.frag");
	grp->getOrCreateStateSet()->addUniform(new osg::Uniform("osg_LightEnabled", true));
	grp->addChild(mainSceneRoot);
	passesGroup->addChild(grp);
	passesGroup->addChild(outlinePassTransparency->camera.get());
	passesGroup->addChild(outlinePass->camera.get());
#endif

	return passesGroup.release();

}
void OutlineShaderPipeline::HandleResize(int width, int height)
{
	for(RenderPassState* state : _renderPassStates) {
		state->HandleResize(width, height);
	}
}
