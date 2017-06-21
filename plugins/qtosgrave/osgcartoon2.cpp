// -*- coding: utf-8 -*-
// openrave: tweaked openscenegraph cartoon shader

#include "osgcartoon2.h"

#include <osgFX/Registry>

#include <osg/PolygonOffset>
#include <osg/Texture1D>
#include <osg/VertexProgram>
#include <osg/PolygonMode>
#include <osg/CullFace>
#include <osg/Image>
#include <osg/TexEnv>
#include <osg/LineWidth>
#include <osg/Material>
#include <osg/Program>
#include <osg/Shader>


#include <sstream>

namespace qtosgrave {

using namespace osgFX;

static osg::Image* create_sharp_lighting_map(int levels = 4, int texture_size = 256)
{
    osg::ref_ptr<osg::Image> image = new osg::Image;
    image->setImage(texture_size, 1, 1, 4, GL_RGBA, GL_UNSIGNED_BYTE, new unsigned char[4*texture_size], osg::Image::USE_NEW_DELETE);
    for (int i=0; i<texture_size; ++i) {
        float c = i/static_cast<float>(texture_size);
        c = (1+static_cast<int>(sqrtf(c) * (levels))) / static_cast<float>(levels+1);
        *(image->data(i, 0)+0) = static_cast<unsigned char>(c*255);
        *(image->data(i, 0)+1) = static_cast<unsigned char>(c*255);
        *(image->data(i, 0)+2) = static_cast<unsigned char>(c*255);
        *(image->data(i, 0)+3) = 255;
    }
    return image.release();
}

// register a prototype for this effect
//Registry::Proxy proxy(new OpenRAVECartoon2); ???

///////////////////////////////////////////////////////////////////////////
// A port of Marco Jez's "cartoon.cg" to the OpenGL Shading Language
// by Mike Weiblen 2003-10-03,
//
// This shader is simplified due to limitations in the OGLSL implementation
// in the current 3Dlabs driver.  As the OGLSL implementation improves,
// need to revisit and enhance this shader.
class OGLSL_TechniqueBla : public Technique {
public:
    OGLSL_TechniqueBla(osg::Material* wf_mat, osg::LineWidth *wf_lw, int lightnum, osg::Camera *camera)
        : Technique(), _wf_mat(wf_mat), _wf_lw(wf_lw), _lightnum(lightnum), _camera(camera) {
    }

    void getRequiredExtensions(std::vector<std::string>& extensions) const
    {
        extensions.push_back( "GL_ARB_shader_objects" );
        extensions.push_back( "GL_ARB_vertex_shader" );
        extensions.push_back( "GL_ARB_fragment_shader" );
    }

protected:

    void define_passes()
    {
        // implement pass #1 (solid surfaces)
        {
            std::ostringstream vert_source;
            vert_source <<
                "varying vec3 eye_space_normal;\n"
                "varying vec3 LightDirection;\n"
                "void main( void )\n"
                "{\n"
                "    vec4 LightPosition = gl_LightSource["<<_lightnum<<"].position;\n"
                "    if (LightPosition[3]!=0.0) { \n"
                "        vec4 eye_space_position = gl_ModelViewMatrix * gl_Vertex;\n"
                "        LightDirection = LightPosition.xyz-eye_space_position.xyz;\n"
                "    } else {\n"
                "        LightDirection = LightPosition.xyz;\n"
                "    }\n"
                "    LightDirection = normalize(LightDirection);\n"
                "    eye_space_normal = normalize(gl_NormalMatrix * gl_Normal);\n"
                "    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;\n"
                "}\n";

            const char * frag_source =
                "uniform sampler1D CartoonTexUnit;"
                "varying vec3 eye_space_normal;"
                "varying vec3 LightDirection;\n"
                "void main( void )"
                "{"
                "    float CartoonTexCoord = max(0.0, dot(LightDirection, eye_space_normal));\n"
                "    gl_FragColor = texture1D( CartoonTexUnit, CartoonTexCoord ) * gl_FrontMaterial.diffuse;"
                "    gl_FragColor.xyz += gl_FrontMaterial.ambient.xyz;"
                "}";

            osg::ref_ptr<osg::StateSet> ss = new osg::StateSet;

            // osg::ref_ptr<osg::PolygonOffset> polyoffset = new osg::PolygonOffset;
            // polyoffset->setFactor(1.0f);
            // polyoffset->setUnits(1.0f);
            // ss->setAttributeAndModes(polyoffset.get(), osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);

            osg::ref_ptr<osg::Program> program = new osg::Program;
            program->addShader( new osg::Shader( osg::Shader::VERTEX, vert_source.str() ) );
            program->addShader( new osg::Shader( osg::Shader::FRAGMENT, frag_source ) );

            ss->addUniform( new osg::Uniform("CartoonTexUnit", 0));
            ss->setAttributeAndModes( program.get(), osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);


            ss->setTextureMode(0, GL_TEXTURE_2D, osg::StateAttribute::OVERRIDE | osg::StateAttribute::OFF);

            osg::ref_ptr<osg::Texture1D> texture = new osg::Texture1D;
            texture->setImage(create_sharp_lighting_map());
            texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
            texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::NEAREST);
            ss->setTextureAttributeAndModes(0, texture.get(), osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);

            osg::ref_ptr<osg::TexEnv> texenv = new osg::TexEnv;
            texenv->setMode(osg::TexEnv::MODULATE);
            ss->setTextureAttributeAndModes(0, texenv.get(), osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);

            addPass(ss.get());
        }

        // implement pass #2 (outlines)
        /*{
            osg::ref_ptr<osg::StateSet> ss = new osg::StateSet;
            osg::ref_ptr<osg::PolygonMode> polymode = new osg::PolygonMode;
            polymode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
            ss->setAttributeAndModes(polymode.get(), osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);

            osg::ref_ptr<osg::CullFace> cf = new osg::CullFace;
            cf->setMode(osg::CullFace::FRONT);
            ss->setAttributeAndModes(cf.get(), osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);

            ss->setAttributeAndModes(_wf_lw.get(), osg::StateAttribute::OVERRIDE | osg::StateAttribute::ON);

            _wf_mat->setColorMode(osg::Material::OFF);
            _wf_mat->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1));
            _wf_mat->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1));
            _wf_mat->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1));

            // set by outline colour so no need to set here.
            //_wf_mat->setEmission(osg::Material::FRONT_AND_BACK, osg::Vec4(0, 0, 0, 1));

            ss->setAttributeAndModes(_wf_mat.get(), osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);

            ss->setMode(GL_LIGHTING, osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
            ss->setTextureMode(0, GL_TEXTURE_1D, osg::StateAttribute::OVERRIDE|osg::StateAttribute::OFF);
            ss->setTextureMode(0, GL_TEXTURE_2D, osg::StateAttribute::OVERRIDE|osg::StateAttribute::OFF);

            addPass(ss.get());

        }*/
    }

private:
    osg::ref_ptr<osg::Material> _wf_mat;
    osg::ref_ptr<osg::LineWidth> _wf_lw;
    int _lightnum;
    osg::Camera *_camera;
};

///////////////////////////////////////////////////////////////////////////

OpenRAVECartoon2::OpenRAVECartoon2(osg::Camera *camera)
    :    Effect(),
    _wf_mat(new osg::Material),
    _wf_lw(new osg::LineWidth(2.0f)),
    _lightnum(0),
    _camera(camera)
{
    setOutlineColor(osg::Vec4(0, 0, 0, 1));
}

OpenRAVECartoon2::OpenRAVECartoon2(const OpenRAVECartoon2& copy, const osg::CopyOp& copyop)
    :    Effect(copy, copyop),
    _wf_mat(static_cast<osg::Material* >(copyop(copy._wf_mat.get()))),
    _wf_lw(static_cast<osg::LineWidth *>(copyop(copy._wf_lw.get()))),
    _lightnum(copy._lightnum),
    _camera(copy._camera)
{
}

bool OpenRAVECartoon2::define_techniques()
{
    addTechnique(new OGLSL_TechniqueBla(_wf_mat.get(), _wf_lw.get(), _lightnum, _camera));
    return true;
}

//  http://www.mutantstargoat.com/bekos/wordpress/2011/06/12/deferred-rendering-with-openscenegraph/
osg::Camera* OpenRAVECartoon2::CreateCameraFor3DTransparencyPass(
    osg::Texture2D* rttDepth,
    osg::Texture2D* rttAccum,
    osg::Texture2D* rttRevealage)
{
    osg::Camera* camera = new osg::Camera();
    //  Set the camera to render before the main camera.
    camera->setRenderOrder(osg::Camera::PRE_RENDER); // <---------------------------- set order?
    //  Tell the camera to use OpenGL frame buffer object where supported.
    camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
    //  Attack RTT textures to Camera
    camera->attach(osg::Camera::DEPTH_BUFFER,  rttDepth);
    camera->attach(osg::Camera::COLOR_BUFFER0, rttAccum);
    camera->attach(osg::Camera::COLOR_BUFFER1, rttRevealage);
    //  Set up the background color and clear mask.
    camera->setClearColor(osg::Vec4(0.0f,0.0f,0.0f,1.0f)); // <---------------- per RT clear
    camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

/*    //  Set viewport
    camera->setViewport(0,0,512,512);
    //  Set up projection.
    camera->setProjectionMatrixAsPerspective(   45.0, 1.0, 10.0, 100.0);
    //  Set view
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    camera->setViewMatrixAsLookAt(osg::Vec3(0.0f,-30.0f,0.0f),osg::Vec3(0,0,0),osg::Vec3(0.0f,0.0f,1.0f));
    
    //  Camera hints
    camera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);*/

    return camera;
}
}

osg::Texture2D* OpenRAVECartoon2::CreateAccumRTFor3DTransparencyPass(uint32_t width, uint32_t height)
{
    osg::Texture2D* texture = new osg::Texture2D();
    texture->setTextureSize(width, height);
    texture->setInternalFormat(GL_RGBA32F_ARB); // Use 16?
    texture->setSourceFormat(GL_RGBA); // <---- Needed?
    texture->setWrap(osg::Texture2D::WRAP_S,osg::Texture2D::CLAMP_TO_EDGE);
    texture->setWrap(osg::Texture2D::WRAP_T,osg::Texture2D::CLAMP_TO_EDGE);
    texture->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
    texture->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
    return texture;
}

osg::Texture2D* CreateRevealageRTFor3DTransparencyPass(uint32_t width, uint32_t height)
{
    osg::Texture2D* texture = new osg::Texture2D();
    texture->setTextureSize(width, height);
    texture->setInternalFormat(GL_R8); // Use 16?
    texture->setSourceFormat(GL_RED); // <---- Needed?
    texture->setWrap(osg::Texture2D::WRAP_S,osg::Texture2D::CLAMP_TO_EDGE);
    texture->setWrap(osg::Texture2D::WRAP_T,osg::Texture2D::CLAMP_TO_EDGE);
    texture->setFilter(osg::Texture2D::MIN_FILTER,osg::Texture2D::LINEAR);
    texture->setFilter(osg::Texture2D::MAG_FILTER,osg::Texture2D::LINEAR);
    return texture;
}

} // end namespace qtosgrave
