//
// Created by mujin on 6/15/17.
//

#ifndef OPENRAVE_OSGSKYBOX_H
#define OPENRAVE_OSGSKYBOX_H

#include <osg/Depth>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/TextureCubeMap>
#include <osg/Transform>
#include <osg/TexMat>
#include <osg/TexGen>
#include <osg/TexEnv>
#include <osg/TextureCubeMap>
#include <osg/ref_ptr>
#include <osgUtil/SmoothingVisitor>
#include <osgUtil/CullVisitor>
#include <osgDB/ReadFile>

class Skybox : public osg::ClearNode
{
 public:
  Skybox();

  void setTextureCubeMap(const std::string &posx,
                         const std::string &negx,
                         const std::string &posy,
                         const std::string &negy,
                         const std::string &posz,
                         const std::string &negz);

  void show(bool visible);

 protected:
  bool _bVisible;
  osg::ref_ptr<osg::TextureCubeMap> _skymap;
  virtual ~Skybox();
};

#endif //OPENRAVE_OSGSKYBOX_H
