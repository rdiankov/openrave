#include "transparencygroup.h"

#include <GL/gl.h>
#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/Node>
#include <osg/Program>
#include <osg/StateSet>
#include <osg/ref_ptr>

#include <osg/PolygonOffset>
#include <osg/VertexProgram>
#include <osg/PolygonMode>
#include <osg/CullFace>
#include <osg/Image>
#include <osg/TexEnv>
#include <osg/LineWidth>
#include <osg/Material>
#include <osg/Shader>
#include <osgFX/Technique>

osg::ref_ptr<osg::Program> getProgram(const char * vert_source, const char * frag_source)
{
    osg::ref_ptr<osg::Program> program = new osg::Program;
    program->addShader( new osg::Shader( osg::Shader::VERTEX, vert_source ) );
    program->addShader( new osg::Shader( osg::Shader::FRAGMENT, frag_source ) );
    return program;
}

osg::ref_ptr<osg::Program> getOutlinePassProgram()
{
    const char * vert_source =
        "void main( void )"
        "{"
        "    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;"
        "}";

    const char * frag_source =
        "void main( void )"
        "{"
        "    gl_FragColor = vec4(0, 0, 0, 1);"
        "}";

    return getProgram(vert_source, frag_source);
}

osg::ref_ptr<osg::Program> getMainPassProgram()
{
    const char * vert_source =
        "varying float LightIntensity;"
        "varying vec3 EyeSpaceNormal;"
        "varying vec3 LocalSpaceNormal;"
        "void main( void )"
        "{"
        "    vec3 LightDirection = normalize(vec3(0, 0, 1));"
        "    vec3 eye_space_normal = normalize(gl_NormalMatrix * gl_Normal);"
        "    LightIntensity = max(0.0, dot(LightDirection, eye_space_normal));"
        "    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;"
        "    EyeSpaceNormal = eye_space_normal;"
        "}";

    const char * frag_source =
        "varying float LightIntensity;"
        "varying vec3 EyeSpaceNormal;"
        "varying vec3 LocalSpaceNormal;"
        "void main( void )"
        "{"
        "    float a = 0.5;"
        "    float MinimumLight = 0.9;"
        "    gl_FragColor = vec4(0, 0, 0, 0);"
        "    gl_FragColor.xyz += (MinimumLight + (1.0 - MinimumLight) * LightIntensity) * gl_FrontMaterial.diffuse + gl_FrontMaterial.ambient.xyz;"
        "    gl_FragColor.a = a;"
        "}";

    return getProgram(vert_source, frag_source);
}

osg::ref_ptr<osg::StateSet> getOutlinePassStateSet()
{
    osg::ref_ptr<osg::StateSet> ss = new osg::StateSet;
    osg::ref_ptr<osg::PolygonMode> polymode = new osg::PolygonMode;
    polymode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);

    ss->setAttributeAndModes(polymode.get(), osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);

    osg::ref_ptr<osg::CullFace> cf = new osg::CullFace;
    cf->setMode(osg::CullFace::FRONT);
    ss->setAttributeAndModes(cf.get(), osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);

    osg::LineWidth *_wf_lw = new osg::LineWidth(0.5);
    ss->setAttributeAndModes(_wf_lw, osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);
    osg::Material *_wf_mat = new osg::Material();
    _wf_mat->setColorMode(osg::Material::OFF);
    _wf_mat->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 0.5));
    _wf_mat->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 0.5));
    _wf_mat->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 0.5));
    ss->setAttributeAndModes(_wf_mat, osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);

    ss->setMode(GL_LIGHTING, osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
    ss->setTextureMode(0, GL_TEXTURE_1D, osg::StateAttribute::OVERRIDE|osg::StateAttribute::OFF);
    ss->setTextureMode(0, GL_TEXTURE_2D, osg::StateAttribute::OVERRIDE|osg::StateAttribute::OFF);

    osg::ref_ptr<osg::Program> program = getOutlinePassProgram();
    ss->setAttributeAndModes(program.get(), osg::StateAttribute::ON);
    return ss;
}

osgtt::RenderingTechnique::RenderingTechnique(osg::ref_ptr<osg::StateSet> state) : Technique()
{
    _state = state;
}

void osgtt::RenderingTechnique::getRequiredExtensions(std::vector<std::string>& extensions) const
{
    extensions.push_back( "GL_ARB_shader_objects" );
    extensions.push_back( "GL_ARB_vertex_shader" );
    extensions.push_back( "GL_ARB_fragment_shader" );
}

void osgtt::RenderingTechnique::define_passes()
{
    addPass(getOutlinePassStateSet().get());
    addPass(_state.get());
}

osgtt::OutlineEffect::OutlineEffect(osg::ref_ptr<osg::StateSet> state)
{
    _state = state;
    define_techniques();
}

osg::ref_ptr<osgtt::RenderingTechnique> makeTechnique(osg::ref_ptr<osg::StateSet> colorPass)
{
    osgtt::RenderingTechnique *technique = new osgtt::RenderingTechnique(colorPass);
    return technique;
}

bool osgtt::OutlineEffect::define_techniques()
{
    addTechnique(makeTechnique(_state));
    return true;
}

osg::ref_ptr<osgtt::OutlineEffect> makeEffect(osg::ref_ptr<osg::StateSet> colorPass)
{
    osgtt::OutlineEffect *effect = new osgtt::OutlineEffect(colorPass);
    return effect;
}

const char* osgtt::OutlineEffect::effectName() const
{
    return "OutlineEffect";
}

const char* osgtt::OutlineEffect::effectAuthor() const
{
    return "not applicable";
}

const char* osgtt::OutlineEffect::effectDescription() const
{
    return "not applicable";
}

namespace osgtt {

TransparencyGroup::TransparencyGroup():
_mode(NO_TRANSPARENCY) {
    _program          = getMainPassProgram();
    _scene            = new osg::Group();
    _blendFunc        = new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    _transparentState = new osg::StateSet();
    _transparentStateDoubleSided = new osg::StateSet();
    _opaqueState      = new osg::StateSet();
    _opaqueStateDoubleSided      = new osg::StateSet();

    _transparentState->setMode( GL_CULL_FACE, osg::StateAttribute::ON ); // single-sided transparent faces should cull
    _transparentStateDoubleSided->setMode( GL_CULL_FACE, osg::StateAttribute::OFF ); // need to see (lit) back faces in transparency
    _opaqueState->setMode( GL_CULL_FACE, osg::StateAttribute::ON ); // opaque back faces should cull
    _opaqueStateDoubleSided->setMode( GL_CULL_FACE, osg::StateAttribute::OFF ); // opaque doublesided should not cull
}

TransparencyGroup::TransparencyGroup(const TransparencyGroup& tg, const osg::CopyOp& co):
osg::Group    (tg, co),
_mode         (tg._mode),
_scene        (tg._scene)
{
}

bool TransparencyGroup::addChild(osg::Node* child, bool transparent, bool twoSided)
{
    // setStateForNode(child, transparent, twoSided);
    return getEffect(transparent, twoSided)->addChild(child);
    // return _scene->addChild(child);
}

void TransparencyGroup::setStateForNode(osg::Node* child, bool transparent, bool twoSided)
{
    if(transparent) {
        if(twoSided) {
            child->setStateSet(_transparentStateDoubleSided);
        }
        else {
            child->setStateSet(_transparentState);
        }

    }
    else {
        if(twoSided) {
            child->setStateSet(_opaqueStateDoubleSided);
        }
        else {
            child->setStateSet(_opaqueState);
        }
    }
}

osgtt::OutlineEffect* TransparencyGroup::getEffect(bool transparent, bool twoSided)
{
    if(transparent) {
        if(twoSided) {
            return _transparentOutlineDoubleSided;
        }
        else {
            return _transparentOutline;
        }

    }
    else {
        if(twoSided) {
            return _opaqueOutlineDoubleSided;
        }
        else {
            return _opaqueOutline;
        }
    }
}

bool TransparencyGroup::insertChild(unsigned int index, osg::Node* child)
{
    return _scene->insertChild(index, child);
}

bool TransparencyGroup::removeChildren(unsigned int pos, unsigned int numChildrenToRemove)
{
    return _scene->removeChildren(pos, numChildrenToRemove);
}

bool TransparencyGroup::replaceChild(osg::Node* origChild, osg::Node* newChild)
{
    return _scene->replaceChild(origChild, newChild);
}

bool TransparencyGroup::setChild(unsigned int i, osg::Node* child)
{
    return _scene->setChild(i, child);
}

void TransparencyGroup::setTransparencyMode(TransparencyMode mode)
{
    if(_mode == mode) {
        return;
    }

    // NOTE: As we setup the TransparencyMode settings, _mode will still contain the
    // PREVIOUS TransparencyMode, while the mode argument will contain the desired mode.

    // First, remove all of the previous children, whatever they are.
    Group::removeChildren(0, getNumChildren());
    Node::dirtyBound(); // just in case

    // In this mode, we'll just add our proxied scene object and use OSG's default
    // transparency/alpha/blending/whatever.
    osg::ref_ptr<osg::Depth> depth = new osg::Depth;
    _transparentState->setAttributeAndModes(_program.get(), osg::StateAttribute::ON);
    _transparentStateDoubleSided->setAttributeAndModes(_program.get(), osg::StateAttribute::ON);
    if(mode == DEPTH_SORTED_BIN) {
        depth->setWriteMask(true);
        _transparentState->setAttributeAndModes(depth.get(), osg::StateAttribute::ON);
        _transparentState->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
        _transparentState->setAttributeAndModes(_blendFunc.get(), osg::StateAttribute::ON);

        _transparentStateDoubleSided->setAttributeAndModes(depth.get(), osg::StateAttribute::ON);
        _transparentStateDoubleSided->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
        _transparentStateDoubleSided->setAttributeAndModes(_blendFunc.get(), osg::StateAttribute::ON);

        Group::addChild(_scene.get());
    }
    // In this mode, we render transparent objects last, but without depth sorting or writing
    else if(mode == DELAYED_BLEND) {
        depth->setWriteMask(false);
        _transparentState->setAttributeAndModes(depth.get(), osg::StateAttribute::ON);

        _transparentState->setRenderingHint(osg::StateSet::DEFAULT_BIN);
        _transparentState->setRenderBinDetails(12, "RenderBin");
        _transparentState->setAttributeAndModes(_blendFunc.get(), osg::StateAttribute::ON);

        _transparentStateDoubleSided->setAttributeAndModes(depth.get(), osg::StateAttribute::ON);
        
        _transparentStateDoubleSided->setRenderingHint(osg::StateSet::DEFAULT_BIN);
        _transparentStateDoubleSided->setRenderBinDetails(12, "RenderBin");
        _transparentStateDoubleSided->setAttributeAndModes(_blendFunc.get(), osg::StateAttribute::ON);

        Group::addChild(_scene.get());
    }
    else { // NO_TRANSPARENCY
        depth->setWriteMask(true);
        _transparentState->setAttributeAndModes(depth.get(), osg::StateAttribute::ON);

        _transparentState->setRenderingHint(osg::StateSet::DEFAULT_BIN);
        _transparentState->setAttributeAndModes(_blendFunc.get(), osg::StateAttribute::OFF);

        _transparentStateDoubleSided->setAttributeAndModes(depth.get(), osg::StateAttribute::ON);
        _transparentStateDoubleSided->setRenderingHint(osg::StateSet::DEFAULT_BIN);
        _transparentStateDoubleSided->setAttributeAndModes(_blendFunc.get(), osg::StateAttribute::OFF);

        Group::addChild(_scene.get());
    }

    _transparentOutline = makeEffect(_transparentState);
    _transparentOutlineDoubleSided = makeEffect(_transparentStateDoubleSided);
    _opaqueOutline = makeEffect(_opaqueState);
    _opaqueOutlineDoubleSided = makeEffect(_opaqueStateDoubleSided);

    _scene->addChild(_transparentOutline);
    _scene->addChild(_transparentOutlineDoubleSided);
    _scene->addChild(_opaqueOutline);
    _scene->addChild(_opaqueOutlineDoubleSided);
    _mode = mode;
}

}
