
#include <osg/Node>
#include <osg/Group>
#include <osg/Uniform>
#include <osg/Texture2D>

#include <vector>

#ifndef OPENRAVE_QTOSG_OUTLINESHADERPIPELINE_H
#define OPENRAVE_QTOSG_OUTLINESHADERPIPELINE_H

struct RenderPassState {
    void HandleResize(int width, int height);
    osg::ref_ptr<osg::Camera> camera;
    osg::ref_ptr<osg::StateSet> state;
    std::vector<osg::ref_ptr<osg::Texture2D>> colorFboTextures;
};

class OutlineShaderPipeline {
public:
    OutlineShaderPipeline();
    virtual ~OutlineShaderPipeline();

    /// \brief This function creates a outline scene pipeline with two passes to render a regular scene with outline edges
    osg::ref_ptr<osg::Group> CreateOutlineSceneFromOriginalScene(osg::ref_ptr<osg::Camera> mainSceneCamera, osg::ref_ptr<osg::Node> mainSceneRoot, int maxFBOBufferWidth, int maxFBOBufferHeight);
    void HandleResize(int width, int height);
    RenderPassState* createSceneToTexturePass(osg::ref_ptr<osg::Camera> mainSceneCamera, osg::ref_ptr<osg::Node> mainSceneRoot, int numColorAttachments, const std::string& vshader, const std::string& fshader);
    RenderPassState* createTextureToTexturePass(RenderPassState* inputPass, int numColorAttachments, const std::string& vshader, const std::string& fshader);
    RenderPassState* createTextureToColorBufferPass(RenderPassState* inputPass, int numColorAttachments, const std::string& vshader, const std::string& fshader);


protected:
    void  setMaxFBOSize(int maxFBOBufferWidth, int maxFBOBufferHeight) { _maxFBOBufferWidth = maxFBOBufferWidth; _maxFBOBufferHeight = maxFBOBufferHeight; }

public:
	osg::Vec3 _outlineColor;
    int _maxFBOBufferWidth;
    int _maxFBOBufferHeight;
	osg::Vec3 _selectedObjectHighlightColor; //< color of the selected object highlight
    float _selectedObjectHighlightIntensity; //< from 0 to +inf, a multiplier on the highligh intensity, 0 means off, 1 means normal (e.g: 2 means double intensity)
    std::vector<RenderPassState*> _renderPassStates;
};

#endif