
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

public:
    std::vector<RenderPassState*> _renderPassStates;
};

#endif