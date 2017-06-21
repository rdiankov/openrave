// -*- coding: utf-8 -*-
/* -*-c++-*- OpenSceneGraph - Copyright (C) 1998-2006 Robert Osfield
 *
 * This library is open source and may be redistributed and/or modified under
 * the terms of the OpenSceneGraph Public License (OSGPL) version 0.0 or
 * (at your option) any later version.  The full license is in LICENSE file
 * included with this distribution, and on the openscenegraph.org website.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * OpenSceneGraph Public License for more details.
 */
//osgFX - Copyright (C) 2003 Marco Jez
// openrave: tweaked openscenegraph cartoon shader

#ifndef OPENRAVE_OSGFX_CARTOON2_
#define OPENRAVE_OSGFX_CARTOON2_

#include <osgFX/Export>
#include <osgFX/Effect>

#include <osg/Material>
#include <osg/LineWidth>
#include <osg/Texture2D>

namespace qtosgrave {

/**
   This effect implements a technique called 'Cel-Shading' to produce a
   cartoon-style (non photorealistic) rendering. Two passes are required:
   the first one draws solid surfaces, the second one draws the outlines.
   A vertex program is used to setup texture coordinates for a sharp lighting
   texture on unit 0 which is generated on-the-fly.
   This effect requires the ARB_vertex_program extension.
 */
class OpenRAVECartoon2 : public osgFX::Effect {
public:
    OpenRAVECartoon2() {} // <--------------------------hack
    OpenRAVECartoon2(osg::Camera *camera);
    OpenRAVECartoon2(const OpenRAVECartoon2& copy, const osg::CopyOp& copyop = osg::CopyOp::SHALLOW_COPY);

    osg::Camera* CreateCameraFor3DTransparencyPass(
        osg::Texture2D* rttDepth,
        osg::Texture2D* rttAccum,
        osg::Texture2D* rttRevealage);
    osg::Texture2D* CreateAccumRTFor3DTransparencyPass(uint32_t width, uint32_t height);
    osg::Texture2D* CreateRevealageRTFor3DTransparencyPass(uint32_t width, uint32_t height);

    // effect class informations
    META_Effect(
        osgFX,
        OpenRAVECartoon2,

        "OpenRAVECartoon2",

        "This effect implements a technique called 'Cel-Shading' to produce a "
        "cartoon-style (non photorealistic) rendering. Two passes are required: "
        "the first one draws solid surfaces, the second one draws the outlines. "
        "A vertex program is used to setup texture coordinates for a sharp lighting "
        "texture on unit 0 which is generated on-the-fly.\n"
        "This effect requires the ARB_vertex_program extension "
        "or OpenGL Shading Language.",

        "Marco Jez; OGLSL port by Mike Weiblen");

    /** get the outline color */
    inline const osg::Vec4& getOutlineColor() const;

    /** set the outline color */
    inline void setOutlineColor(const osg::Vec4& color);

    /** get the outline line width */
    inline float getOutlineLineWidth() const;

    /** set the outline line width */
    inline void setOutlineLineWidth(float w);

    /** get the OpenGL light number */
    inline int getLightNumber() const;

    /** set the OpenGL light number that will be used in lighting computations */
    inline void setLightNumber(int n);

protected:
    virtual ~OpenRAVECartoon2() {
    }
    OpenRAVECartoon2& operator=(const OpenRAVECartoon2&) {
        return *this;
    }

    bool define_techniques();

private:
    osg::ref_ptr<osg::Material> _wf_mat;
    osg::ref_ptr<osg::LineWidth> _wf_lw;
    int _lightnum;
    // Use raw pointer for not. Does not take ownership
    // TODO: Fix this before merging
    osg::Camera* _camera;
};

// INLINE METHODS

inline const osg::Vec4& OpenRAVECartoon2::getOutlineColor() const
{
    return _wf_mat->getEmission(osg::Material::FRONT_AND_BACK);
}

inline void OpenRAVECartoon2::setOutlineColor(const osg::Vec4& color)
{
    _wf_mat->setEmission(osg::Material::FRONT_AND_BACK, color);
}

inline float OpenRAVECartoon2::getOutlineLineWidth() const
{
    return _wf_lw->getWidth();
}

inline void OpenRAVECartoon2::setOutlineLineWidth(float w)
{
    _wf_lw->setWidth(w);
}

inline int OpenRAVECartoon2::getLightNumber() const
{
    return _lightnum;
}

inline void OpenRAVECartoon2::setLightNumber(int n)
{
    _lightnum = n;
    dirtyTechniques();
}

} // end namespace qtosgrave

#endif
