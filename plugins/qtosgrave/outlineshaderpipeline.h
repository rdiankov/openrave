
#include <osg/Node>
#include <osg/Group>
#include <osg/Uniform>
#include <osg/Texture2DMultisample>

#ifndef OPENRAVE_QTOSG_OUTLINESHADERPIPELINE_H
#define OPENRAVE_QTOSG_OUTLINESHADERPIPELINE_H

struct RenderPassState {
    void HandleResize(int width, int height);
    osg::ref_ptr<osg::Camera> camera;
    osg::ref_ptr<osg::Texture2DMultisample> colorFboTexture;
};

class OutlineShaderPipeline {
public:
    OutlineShaderPipeline();
    virtual ~OutlineShaderPipeline();

    /// \brief This function creates a outline scene pipeline with two passes to render a regular scene with outline edges
    osg::ref_ptr<osg::Group> CreateOutlineSceneFromOriginalScene(osg::ref_ptr<osg::Camera> mainSceneCamera, osg::ref_ptr<osg::Node> mainSceneRoot, int maxFBOBufferWidth, int maxFBOBufferHeight);
    void HandleResize(int width, int height);

protected:
    RenderPassState* createFirstRenderPass(osg::ref_ptr<osg::Camera> mainSceneCamera, osg::ref_ptr<osg::Node> mainSceneRoot, int maxFBOBufferWidth, int maxFBOBufferHeight);
    RenderPassState* createSecondRenderPass(RenderPassState* firstPassState, int maxFBOBufferWidth, int maxFBOBufferHeight);
    RenderPassState* createThirdRenderPass(RenderPassState* secondPassState);

public:
	osg::Vec3 _outlineColor;
	osg::Vec3 _selectedObjectHighlightColor; //< color of the selected object highlight
    float _selectedObjectHighlightIntensity; //< from 0 to +inf, a multiplier on the highligh intensity, 0 means off, 1 means normal (e.g: 2 means double intensity)
    std::vector<RenderPassState*> _renderPassStates;
};

#endif