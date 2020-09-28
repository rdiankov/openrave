
#include <osg/Node>
#include <osg/Group>
#include <osg/Uniform>
#include <osg/Texture>

#include <osg/Switch>
#include <vector>

#ifndef OPENRAVE_QTOSG_OUTLINESHADERPIPELINE_H
#define OPENRAVE_QTOSG_OUTLINESHADERPIPELINE_H

class RenderStatePassShaderReloader;

struct RenderPassState {
    RenderPassState();
    void HandleResize(int width, int height);
    void SetShaderFiles(const std::string& vertShader, const std::string& fragShader, bool autoReload);
    void ReloadShaders();
    osg::ref_ptr<osg::Camera> camera;
    osg::ref_ptr<osg::StateSet> state;
    std::vector<osg::ref_ptr<osg::Texture>> colorFboTextures;
    osg::ref_ptr<osg::Texture> depthFboTexture;
    std::string vertShaderFile;
    std::string fragShaderFile;
    bool autoReloadShaders;

private:
    RenderStatePassShaderReloader* _shaderReloader;
};

class OutlineShaderPipeline {
public:
    OutlineShaderPipeline();
    virtual ~OutlineShaderPipeline();

    // if enabled, will disable advanced shaders for outline rendering and use fallback (toon shading) that is more compatible with old controllers
    // default is: disabled (false)
    void SetCompatibilityModeEnabled(bool value);

    /// \brief This function creates a outline scene pipeline with two passes to render a regular scene with outline edges
    osg::ref_ptr<osg::Group> CreateOutlineSceneFromOriginalScene(osg::ref_ptr<osg::Camera> mainSceneCamera, osg::ref_ptr<osg::Node> mainSceneRoot, int maxFBOBufferWidth, int maxFBOBufferHeight, bool useMultiSamples=false);
    void HandleResize(int width, int height);
    RenderPassState* CreateSceneToTexturePass(osg::ref_ptr<osg::Camera> mainSceneCamera, osg::ref_ptr<osg::Node> mainSceneRoot,
        int numColorAttachments, const std::string& vshader, const std::string& fshader, osg::Texture* reusedDepthTexture = nullptr, bool useMultiSamples=false, const std::vector<osg::ref_ptr<osg::Texture>>& inheritedColorBuffers = std::vector<osg::ref_ptr<osg::Texture>>());
    RenderPassState* CreateTextureToTexturePass(RenderPassState* inputPass, int numColorAttachments, const std::string& vshader, const std::string& fshader, bool useMultiSamples=false, osg::Texture* reusedDepthTexture = nullptr);
    RenderPassState* CreateTextureToColorBufferPass(RenderPassState* inputPass, int numColorAttachments, const std::string& vshader, const std::string& fshader);


protected:
    void _SetupOutlineShaderUniforms(RenderPassState* pass);
    // sets the initial size fort FBO buffers. They will be resized if HandleResize is called.
    void  SetInitialFBOTextureSize(int maxFBOBufferWidth, int maxFBOBufferHeight) { _maxFBOBufferWidth = maxFBOBufferWidth; _maxFBOBufferHeight = maxFBOBufferHeight; }

public:
	osg::Vec3 _outlineColor;
    int _maxFBOBufferWidth;
    int _maxFBOBufferHeight;
	osg::Vec3 _selectedObjectHighlightColor; //< color of the selected object highlight
    float _selectedObjectHighlightIntensity; //< from 0 to +inf, a multiplier on the highligh intensity, 0 means off, 1 means normal (e.g: 2 means double intensity)
    osg::ref_ptr<osg::Switch> _compatibilityModeSwitch;
    std::vector<RenderPassState*> _renderPassStates;
    bool _compatibilityMode;
};

#endif