
#include <osg/Node>
#include <osg/Group>
#include <osg/Uniform>
#include <osg/TextureRectangle>

#ifndef OPENRAVE_QTOSG_OUTLINESHADERPIPELINE_H
#define OPENRAVE_QTOSG_OUTLINESHADERPIPELINE_H

struct OutlineFirstPassState {
    osg::ref_ptr<osg::Group> firstPassGroup;
    osg::ref_ptr<osg::Camera> firstPassCamera;
    osg::ref_ptr<osg::TextureRectangle> firstPassRenderTexture;
};

struct OutlineSecondPassState {
    osg::ref_ptr<osg::Camera> secondPassCamera;
};

class OutlineShaderPipeline {
public:
    OutlineShaderPipeline();
    virtual ~OutlineShaderPipeline() {;}

    /// \brief This function creates a outline scene pipeline with two passes to render a regular scene with outline edges
    void InitializeOutlinePipelineState(osg::ref_ptr<osg::Camera> originalSceneCamera, osg::ref_ptr<osg::StateSet> inheritedStateSet, osg::ref_ptr<osg::Node> originalSceneRoot, int viewportWidth, int viewportHeight);

public:
    OutlineFirstPassState _firstPassState;
    OutlineSecondPassState _secondPassState;
};

#endif