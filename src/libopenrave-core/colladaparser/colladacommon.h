// -*- coding: utf-8 -*-
// Copyright (C) 2012-2013 Rosen Diankov <rosen.diankov@gmail.com>
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
/** \file colladacommon.h
    \brief Common external definitions for the collada reader/writer
 */
#ifndef OPENRAVE_COLLADA_COMMON_H
#define OPENRAVE_COLLADA_COMMON_H

#include "../ravep.h"

#ifdef OPENRAVE_COLLADA_SUPPORT

#define COLLADA_DOM_NAMESPACE // collada-dom 2.4
namespace ColladaDOM150 {} // declare in case earlier versions are used

#include <dae.h>
#include <dae/daeErrorHandler.h>
#include <dae/domAny.h>
#include <dae/daeDocument.h>
#include <1.5/dom/domCOLLADA.h>
#include <1.5/dom/domConstants.h>
#include <1.5/dom/domTriangles.h>
#include <1.5/dom/domTypes.h>
#include <1.5/dom/domElements.h>
#include <boost/lexical_cast.hpp>

namespace OpenRAVE
{

class ColladaReader;
class ColladaWriter;

/// \brief holds all information required to describe the collada references to the loaded body
class ColladaXMLReadable : public XMLReadable
{
public:
    static std::string GetXMLIdStatic() {
        return string("__collada__");
    }

    /// \brief sid bindings for kinematics, physics, and visual
    struct LinkBinding
    {
        LinkBinding() : index(-1) {
        }
        LinkBinding(const std::string& kmodel, const std::string& pmodel, const std::string& vmodel) : kmodel(kmodel), pmodel(pmodel), vmodel(vmodel), index(-1) {
        }
        std::string kmodel, pmodel, vmodel;
        int index; ///< for _bindingLinkSIDs, it is the index inside the _bindingModelURIs vector
    };

    /// \brief sid bindings for kinematics, physics, and visual
    struct AxisBinding
    {
        AxisBinding() {
        }
        AxisBinding(const std::string& kmodelaxissidref, const std::string& nodesid, const std::string& jointsidref) : kmodelaxissidref(kmodelaxissidref), nodesid(nodesid), jointsidref(jointsidref) {
        }
        std::string kmodelaxissidref;
        std::string nodesid;
        std::string jointsidref; ///< the sidref of the joint kmodelid/jointsid
    };

    /// \brief sid bindings for kinematics, physics, and visual
    struct ModelBinding
    {
        ModelBinding() {
        }
        ModelBinding(const std::string& kmodel, const std::string& pmodel, const std::string& vmodel) : kmodel(kmodel), pmodel(pmodel), vmodel(vmodel) {
        }
        std::string kmodel; ///< kmodel is a SIDREF to the instance_kinematics_model that will later be used in bind_kinematics_model
        std::string pmodel;
        std::string vmodel; ///< vmodel is the node url without any translations/rotations
        std::string ikmodelsidref;
    };

    ColladaXMLReadable() : XMLReadable(GetXMLIdStatic()) {
    }

    std::list< std::pair<std::string, bool> > _articulated_systemURIs; ///< pairs of (urls, isexternal) of the articulated_system, ordered in the same way as they are read. The first is the top-most level
    std::vector<ModelBinding> _bindingModelURIs;
    std::vector<AxisBinding> _bindingAxesSIDs; ///< same order as the body DOF
    std::list<AxisBinding> _bindingPassiveAxesSIDs; ///< same order as body->GetPassiveJoints()
    std::vector<LinkBinding> _bindingLinkSIDs; ///< link bindings, SID for link, rigidbody, but URL for vmodel (node). same order as link indices
};

typedef boost::shared_ptr<ColladaXMLReadable> ColladaXMLReadablePtr;

/** Have to maintain a global DAE pointer that is only destroyed on OpenRAVE destruction. The reasons are:

   1. destroying DAE unconditionally calls xmlCleanupParser (libxml2)
   2. libxml2 versions before 2.9.0 have a bug that fail to be re-initialiezd after xmlCleanupParser is called

 */
/// \brief get the global DAE.
///
/// \param clear is true, will attempt to reset any resolvers to defaults
boost::shared_ptr<DAE> GetGlobalDAE(bool resetdefaults=true);
/// \brief reset the global DAE
void SetGlobalDAE(boost::shared_ptr<DAE>);
boost::mutex& GetGlobalDAEMutex();


// register for typeof (MSVC only)
#ifdef RAVE_REGISTER_BOOST
#include BOOST_TYPEOF_INCREMENT_REGISTRATION_GROUP()
BOOST_TYPEOF_REGISTER_TYPE(ColladaXMLReadable::Binding)
#endif


bool RaveParseColladaURI(EnvironmentBasePtr penv, const std::string& uri,const AttributesList& atts);
bool RaveParseColladaURI(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& uri, const AttributesList& atts);
bool RaveParseColladaURI(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& uri, const AttributesList& atts);
bool RaveParseColladaFile(EnvironmentBasePtr penv, const std::string& filename,const AttributesList& atts);
bool RaveParseColladaFile(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& filename,const AttributesList& atts);
bool RaveParseColladaFile(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& filename,const AttributesList& atts);
bool RaveParseColladaData(EnvironmentBasePtr penv, const std::string& data,const AttributesList& atts);
bool RaveParseColladaData(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& data,const AttributesList& atts);
bool RaveParseColladaData(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& data,const AttributesList& atts);

void RaveWriteColladaFile(EnvironmentBasePtr penv, const std::string& filename,const AttributesList& atts);
void RaveWriteColladaFile(KinBodyPtr pbody, const std::string& filename,const AttributesList& atts);
void RaveWriteColladaFile(const std::list<KinBodyPtr>& listbodies, const std::string& filename,const AttributesList& atts);

} // end OpenRAVE namespace

#else // OPENRAVE_COLLADA_SUPPORT

namespace OpenRAVE
{

inline bool RaveParseColladaURI(EnvironmentBasePtr penv, const std::string& uri,const AttributesList& atts)
{
    RAVELOG_ERROR("collada files not supported\n");
    return false;
}

inline bool RaveParseColladaURI(EnvironmentBasePtr penv, KinBodyPtr& pbody, const string& uri, const AttributesList& atts)
{
    RAVELOG_ERROR("collada files not supported\n");
    return false;
}

inline bool RaveParseColladaURI(EnvironmentBasePtr penv, RobotBasePtr& probot, const string& uri, const AttributesList& atts)
{
    RAVELOG_ERROR("collada files not supported\n");
    return false;
}

inline bool RaveParseColladaFile(EnvironmentBasePtr penv, const string& filename,const AttributesList& atts)
{
    RAVELOG_ERROR("collada files not supported\n");
    return false;
}

inline bool RaveParseColladaFile(EnvironmentBasePtr penv, KinBodyPtr& pbody, const string& filename,const AttributesList& atts)
{
    RAVELOG_ERROR("collada files not supported\n");
    return false;
}

inline bool RaveParseColladaFile(EnvironmentBasePtr penv, RobotBasePtr& probot, const string& filename,const AttributesList& atts)
{
    RAVELOG_ERROR("collada files not supported\n");
    return false;
}

inline bool RaveParseColladaData(EnvironmentBasePtr penv, const string& pdata,const AttributesList& atts)
{
    RAVELOG_ERROR("collada files not supported\n");
    return false;
}

inline bool RaveParseColladaData(EnvironmentBasePtr penv, KinBodyPtr& pbody, const string& pdata,const AttributesList& atts)
{
    RAVELOG_ERROR("collada files not supported\n");
    return false;
}

inline bool RaveParseColladaData(EnvironmentBasePtr penv, RobotBasePtr& probot, const string& pdata,const AttributesList& atts)
{
    RAVELOG_ERROR("collada files not supported\n");
    return false;
}


inline void RaveWriteColladaFile(EnvironmentBasePtr penv, const std::string& filename,const AttributesList& atts)
{
    RAVELOG_ERROR("collada files not supported\n");
}

inline void RaveWriteColladaFile(KinBodyPtr pbody, const std::string& filename,const AttributesList& atts)
{
    RAVELOG_ERROR("collada files not supported\n");
}

inline void RaveWriteColladaFile(const std::list<KinBodyPtr>& listbodies, const std::string& filename,const AttributesList& atts)
{
    RAVELOG_ERROR("collada files not supported\n");
}

} // end OpenRAVE namespace

#endif // OPENRAVE_COLLADA_SUPPORT


#endif
