#pragma once

#include <osg/Group>
#include <osg/BlendFunc>
#include <osg/Program>

#include <osg/StateSet>
#include <osgFX/Registry>
#include <osgFX/Technique>

namespace osgtt {

class RenderingTechnique: public osgFX::Technique {
protected:
    osg::ref_ptr<osg::StateSet> _state;
public:
    RenderingTechnique(osg::ref_ptr<osg::StateSet> state);

    virtual void define_passes();
    virtual void getRequiredExtensions(std::vector<std::string>& extensions) const;
};

class OutlineEffect : public osgFX::Effect {
public:
    OutlineEffect();
    OutlineEffect(osg::ref_ptr<osg::StateSet> state);
    OutlineEffect(const OutlineEffect& copy, const osg::CopyOp& copyop = osg::CopyOp::SHALLOW_COPY);

    const char* effectName() const;
    const char* effectAuthor() const;
    const char* effectDescription() const;

    protected:
        osg::ref_ptr<osg::StateSet> _state;
        bool define_techniques();

};

// A TransparencyGroup is an osg::Group-like node (it uses the Group interface but
// behaves differently internally) that provides a simple API for applying a technique
// for transparency on its child subgraph. Any children you add to this particular
// Group will be managed internally so that whatever technique is currently in effect
// can function properly.
class TransparencyGroup: public osg::Group {
public:
    // These are the supported "kinds" of transparency we can apply to our subgraph.
    enum TransparencyMode {
        // This is the simplest, fastest, most innaccurate form; it is the OSG
        // default method, and simply sorts by depth.
        DEPTH_SORTED_BIN,
        // This mode is not depth sorted. "Transparent objects are rendered using
        // multiplicative alpha blending, in a second rendering pass with depth buffer
        // updates disabled." - http://doc.coin3d.org/Coin/classSoGLRenderAction.html
        DELAYED_BLEND,
        // Default is no method set.
        NO_TRANSPARENCY
    };

    TransparencyGroup();
    TransparencyGroup(const TransparencyGroup&, const osg::CopyOp& co = osg::CopyOp::SHALLOW_COPY);

    META_Node(osgtt, TransparencyGroup);

    virtual bool addChild(osg::Node* child, bool transparent, bool twoSided);
    virtual bool insertChild(unsigned int index, osg::Node* child);
    virtual bool removeChildren(unsigned int pos, unsigned int numChildrenToRemove);
    virtual bool replaceChild(osg::Node* origChild, osg::Node* newChild);
    virtual bool setChild(unsigned int i, osg::Node* child);
    void setStateForNode(osg::Node* child, bool transparent, bool twoSided);

    TransparencyMode getTransparencyMode() const
	{
        return _mode;
    }

    void setTransparencyMode(TransparencyMode mode);

protected:
    TransparencyMode             _mode;
    osg::ref_ptr<osg::BlendFunc> _blendFunc;

private:
    osg::ref_ptr<osg::Group> _scene;
    osg::ref_ptr<osg::StateSet> _transparentState, _transparentStateDoubleSided, _opaqueState, _opaqueStateDoubleSided;
    osg::ref_ptr<osg::Program> _program;

    osg::ref_ptr<OutlineEffect> _transparentOutline, _transparentOutlineDoubleSided, _opaqueOutline, _opaqueOutlineDoubleSided;

    osgtt::OutlineEffect* getEffect(bool transparent, bool twoSided);
};

}