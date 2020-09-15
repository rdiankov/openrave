#ifndef OPENRAVE_OSGLODLABEL_H
#define OPENRAVE_OSGLODLABEL_H

#include "qtosg.h"

namespace qtosgrave {
	/// \brief OSG text label that scales by camera distance and also disappears if far away enough
	class OSGLODLabel : public osg::LOD
	{
	public:
	    OSGLODLabel(const std::string& label);
	    ~OSGLODLabel();
	    void traverse(osg::NodeVisitor& nv);
	};
}

#endif