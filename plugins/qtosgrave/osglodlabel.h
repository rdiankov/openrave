#ifndef OPENRAVE_OSGLODLABEL_H
#define OPENRAVE_OSGLODLABEL_H

#include "qtosg.h"
#include <osgText/Text>
#include <osg/LOD>

namespace qtosgrave {
	/// \brief OSG text label that scales by camera distance and also disappears if far away enough
	class OSGLODLabel : public osg::LOD
	{
	public:
	    OSGLODLabel(const std::string& label, const RaveVector<float>& color=RaveVector<float>(0,0,0,1), float height=0.05, osg::ref_ptr<osgText::Font> font=0);
	    ~OSGLODLabel();
	    void traverse(osg::NodeVisitor& nv);
        static void SetFont(osgText::Font* font);
    private:
        /// \brief fallback font for LOD label if set
        static osg::ref_ptr<osgText::Font> OSG_FONT;
	};
}

#endif
