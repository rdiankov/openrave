// -*- coding: utf-8 -*-
// Copyright (C) 2011-2012 Rosen Diankov <rosen.diankov@gmail.com>
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
#include "../ravep.h"

#include "XFileHelper.h"
#include "XFileParser.h"

#include <boost/lexical_cast.hpp>

#ifdef HAVE_BOOST_FILESYSTEM
#include <boost/filesystem/operations.hpp>
#endif

class XFileReader
{
    class JointData : public SerializableData
    {
public:
        virtual void Serialize(std::ostream& O, int options=0) const {
            O << _mapjoints.size() << endl;
            FOREACHC(it,_mapjoints) {
                O << it->first << " = " << it->second << endl;
            }
        }

        virtual void Deserialize(std::istream& I) {
            _mapjoints.clear();
            int num = 0;
            I >> num;
            for(int i = 0; i < num; ++i) {
                std::string name,equals,jointindex;
                I >> name >> equals >> jointindex;
                _mapjoints[name] = boost::lexical_cast<int>(jointindex);
            }
        }

        std::map<std::string,int > _mapjoints; ///< original joint index
    };

public:
    XFileReader(EnvironmentBasePtr penv) : _penv(penv) {
    }

    void ReadFile(KinBodyPtr pbody, const std::string& filename, const AttributesList& atts)
    {
        std::ifstream f(filename.c_str());
        if( !f ) {
            throw OPENRAVE_EXCEPTION_FORMAT("failed to read %s filename",filename,ORE_InvalidArguments);
        }
        f.seekg(0,ios::end);
        string filedata; filedata.resize(f.tellg());
        f.seekg(0,ios::beg);
        f.read(&filedata[0], filedata.size());
        Read(pbody,filedata,atts);
        pbody->__struri = filename;
#if defined(HAVE_BOOST_FILESYSTEM) && BOOST_VERSION >= 103600 // stem() was introduced in 1.36
        boost::filesystem::path bfpath(filename);
#if defined(BOOST_FILESYSTEM_VERSION) && BOOST_FILESYSTEM_VERSION >= 3
        pbody->SetName(bfpath.stem().string());
#else
        pbody->SetName(bfpath.stem());
#endif
#endif

    }

    void ReadFile(RobotBasePtr probot, const std::string& filename, const AttributesList& atts)
    {
        std::ifstream f(filename.c_str());
        if( !f ) {
            throw OPENRAVE_EXCEPTION_FORMAT("failed to read %s filename",filename,ORE_InvalidArguments);
        }
        f.seekg(0,ios::end);
        string filedata; filedata.resize(f.tellg());
        f.seekg(0,ios::beg);
        f.read(&filedata[0], filedata.size());
        Read(probot,filedata,atts);
        probot->__struri = filename;
#if defined(HAVE_BOOST_FILESYSTEM) && BOOST_VERSION >= 103600 // stem() was introduced in 1.36
        boost::filesystem::path bfpath(filename);
#if defined(BOOST_FILESYSTEM_VERSION) && BOOST_FILESYSTEM_VERSION >= 3
        probot->SetName(bfpath.stem().string());
#else
        probot->SetName(bfpath.stem());
#endif
#endif
    }

    void Read(KinBodyPtr pbody, const std::string& data,const AttributesList& atts)
    {
        _ProcessAtts(atts, pbody);
        Assimp::XFileParserOpenRAVE parser(data.c_str());
        _Read(pbody,parser.GetImportedData());
        if( pbody->GetName().size() == 0 ) {
            pbody->SetName("body");
        }
    }

    void Read(RobotBasePtr probot, const std::string& data,const AttributesList& atts)
    {
        _ProcessAtts(atts,probot);
        Assimp::XFileParserOpenRAVE parser(data.c_str());
        _Read(probot,parser.GetImportedData());
        if( probot->GetName().size() == 0 ) {
            probot->SetName("robot");
        }
        // add manipulators
        FOREACH(itmanip,_listendeffectors) {
            RobotBase::ManipulatorInfo manipinfo;
            manipinfo._name = itmanip->first->_name;
            manipinfo._sEffectorLinkName = itmanip->first->GetName();
            manipinfo._sBaseLinkName = probot->GetLinks().at(0)->GetName();
            manipinfo._tLocalTool = itmanip->second;
            manipinfo._vdirection=Vector(1,0,0);
            probot->_vecManipulators.push_back(RobotBase::ManipulatorPtr(new RobotBase::Manipulator(probot,manipinfo)));
        }
    }

protected:
    void _ProcessAtts(const AttributesList& atts, KinBodyPtr pbody)
    {
        _listendeffectors.clear();
        _vScaleGeometry = Vector(1,1,1);
        _bFlipYZ = false;
        _bSkipGeometry = false;
        _prefix = "";
        FOREACHC(itatt,atts) {
            if( itatt->first == "skipgeometry" ) {
                _bSkipGeometry = _stricmp(itatt->second.c_str(), "true") == 0 || itatt->second=="1";
            }
            else if( itatt->first == "scalegeometry" ) {
                stringstream ss(itatt->second);
                Vector v(1,1,1);
                ss >> v.x >> v.y >> v.z;
                if( !ss ) {
                    v.z = v.y = v.x;
                }
                _vScaleGeometry *= v;
            }
            else if( itatt->first == "prefix" ) {
                _prefix = itatt->second;
            }
            else if( itatt->first == "flipyz" ) {
                _bFlipYZ = _stricmp(itatt->second.c_str(), "true") == 0 || itatt->second=="1";
            }
            else if( itatt->first == "name" ) {
                pbody->SetName(itatt->second);
            }
        }
    }

    void _Read(KinBodyPtr pbody, const Assimp::XFile::Scene* scene)
    {
        BOOST_ASSERT(!!scene);
        Transform t;
        KinBody::LinkPtr parent;
        if( pbody->GetLinks().size() > 0 ) {
            parent = pbody->GetLinks()[0];
            t = parent->GetTransform();
        }
        _Read(pbody, parent, scene->mRootNode, t, 0);

        // remove any NULL joints or mimic properties from the main joints...?
        int ijoint = 0;
        vector<KinBody::JointPtr> vecjoints; vecjoints.reserve(pbody->_vecjoints.size());
        vecjoints.swap(pbody->_vecjoints);
        boost::shared_ptr<JointData> jointdata(new JointData());
        FOREACH(itjoint,vecjoints) {
            if( !!*itjoint ) {
                if( !!(*itjoint)->_vmimic[0] ) {
                    RAVELOG_WARN(str(boost::format("joint %s had mimic set!\n")%(*itjoint)->GetName()));
                    (*itjoint)->_vmimic[0].reset();
                }
                jointdata->_mapjoints[(*itjoint)->GetName()] = ijoint;
                pbody->_vecjoints.push_back(*itjoint);
            }
            ++ijoint;
        }
        pbody->SetUserData("xfile", UserDataPtr(jointdata));
    }

    void _Read(KinBodyPtr pbody, KinBody::LinkPtr plink, const Assimp::XFile::Node* node, const Transform &transparent, int level)
    {
        BOOST_ASSERT(!!node);
        Transform tnode = transparent * ExtractTransform(node->mTrafoMatrix);

        RAVELOG_VERBOSE(str(boost::format("node=%s, parent=%s, children=%d, meshes=%d, pivot=%d")%node->mName%(!node->mParent ? string() : node->mParent->mName)%node->mChildren.size()%node->mMeshes.size()%(!!node->mFramePivot)));

        Transform tflipyz;
        if( _bFlipYZ ) {
            tflipyz.rot = quatFromAxisAngle(Vector(PI/2,0,0));
        }

        if( !!node->mFramePivot ) {
            Transform tpivot = tnode*ExtractTransform(node->mFramePivot->mPivotMatrix);
            KinBody::JointPtr pjoint(new KinBody::Joint(pbody));
            // support mimic joints, so have to look at mJointIndex!
            if( node->mFramePivot->mType == 1 ) {
                pjoint->_type = KinBody::JointRevolute;
                pjoint->_vlowerlimit[0] = -PI;
                pjoint->_vupperlimit[0] = PI;
            }
            else if( node->mFramePivot->mType == 2 ) {
                pjoint->_type = KinBody::JointPrismatic;
                pjoint->_vlowerlimit[0] = -10000*_vScaleGeometry.x;
                pjoint->_vupperlimit[0] = 10000*_vScaleGeometry.x;
            }
            else if( node->mFramePivot->mType == 5 ) {
                RAVELOG_WARN(str(boost::format("frame %s is some type of geometry scaling joint?\n")%node->mName));
                pjoint.reset();
                //pjoint->_type = KinBody::JointPrismatic;
                //pjoint->_vlowerlimit[0] = -10000;
                //pjoint->_vupperlimit[0] = 10000;
            }
            else {
                if( node->mFramePivot->mType != 0 ) {
                    RAVELOG_WARN(str(boost::format("unknown joint %s type %d")%node->mName%node->mFramePivot->mType));
                }
                pjoint.reset();
            }

            KinBody::LinkPtr pchildlink;
            if( !!pjoint || !plink || level == 0 ) {
                pchildlink.reset(new KinBody::Link(pbody));
                pchildlink->_name = _prefix+node->mName;
                pchildlink->_t = tflipyz*tpivot*tflipyz.inverse();
                pchildlink->_bStatic = false;
                pchildlink->_bIsEnabled = true;
                pchildlink->_index = pbody->_veclinks.size();
                pbody->_veclinks.push_back(pchildlink);
            }

            if( !!pjoint ) {
                if( node->mFramePivot->mAttribute & 2 ) {
                    // usually this is set for the first and last joints of the file?
                }
                if( node->mFramePivot->mAttribute & 4 ) {
                    // end effector?
                    _listendeffectors.push_back(make_pair(pchildlink,pchildlink->_t.inverse()*tflipyz*tpivot*tflipyz.inverse()));
                }

                pjoint->_name = _prefix+node->mFramePivot->mName;
                pjoint->_bIsCircular[0] = false;
                std::vector<Vector> vaxes(1);
                Transform t = plink->_t.inverse()*tflipyz*tpivot;
                Vector vmotiondirection = Vector(node->mFramePivot->mMotionDirection.x, node->mFramePivot->mMotionDirection.y, node->mFramePivot->mMotionDirection.z);
                vaxes[0] = t.rotate(vmotiondirection);
                if( _bFlipYZ ) {
                    // flip z here makes things right....
                    if( node->mFramePivot->mType == 1 ) {
                        vaxes[0].z *= -1;
                    }
                }
                std::vector<dReal> vcurrentvalues;
                pjoint->_ComputeInternalInformation(plink,pchildlink,t.trans,vaxes,vcurrentvalues);
                if( node->mFramePivot->mJointIndex > pbody->_vecjoints.size() ) {
                    pbody->_vecjoints.resize(node->mFramePivot->mJointIndex);
                }

                string orgjointname = str(boost::format("%sj%d")%_prefix%node->mFramePivot->mJointIndex);
                // prioritize joints with orgjointname when putting into _vecjoints. The only reason to do this is to maintain consistency between expected joint values.

                if( orgjointname != pjoint->_name ) {
                    //KinBody::JointPtr porgjoint = pbody->_vecjoints.at(node->mFramePivot->mJointIndex-1);
                    // joint already exists, so must be mimic?
                    dReal fmult = RaveSqrt(vmotiondirection.lengthsqr3());
                    pjoint->_vmimic[0].reset(new KinBody::Mimic());
                    pjoint->_vmimic[0]->_equations[0] = str(boost::format("%s*%f")%orgjointname%fmult);
                    pjoint->_vmimic[0]->_equations[1] = str(boost::format("|%s %f")%orgjointname%fmult);
                }
                else {
                    // add the joint (make sure motion direction is unit)
                    if( RaveFabs(vaxes[0].lengthsqr3()-1) > 0.0001 ) {
                        RAVELOG_WARN(str(boost::format("joint %s motion axis is not unit: %f %f %f\n")%pjoint->_name%vmotiondirection.x%vmotiondirection.y%vmotiondirection.z));
                    }
                }

                if( !pbody->_vecjoints.at(node->mFramePivot->mJointIndex-1) ) {
                    pbody->_vecjoints.at(node->mFramePivot->mJointIndex-1) = pjoint;
                }
                else {
                    pbody->_vPassiveJoints.push_back(pjoint);
                    if( orgjointname == pjoint->_name ) {
                        // swap with official joint (can come later in the hierarchy)
                        swap(pbody->_vecjoints.at(node->mFramePivot->mJointIndex-1), pbody->_vPassiveJoints.back());
                    }
                }
            }

            if( !!pchildlink ) {
                plink = pchildlink;
            }
        }

        FOREACH(it,node->mMeshes) {

            if( !plink ) {
                // link is expected and one doesn't exist, so create it
                plink.reset(new KinBody::Link(pbody));
                plink->_name = _prefix+node->mName;
                plink->_bStatic = false;
                plink->_bIsEnabled = true;
                plink->_index = pbody->_veclinks.size();
                pbody->_veclinks.push_back(plink);
            }

            Assimp::XFile::Mesh* pmesh = *it;
            KinBody::GeometryInfo g;
            g._t = plink->_t.inverse() * tflipyz * tnode;
            g._type = GT_TriMesh;
            g._meshcollision.vertices.resize(pmesh->mPositions.size());
            for(size_t i = 0; i < pmesh->mPositions.size(); ++i) {
                g._meshcollision.vertices[i] = Vector(pmesh->mPositions[i].x*_vScaleGeometry.x,pmesh->mPositions[i].y*_vScaleGeometry.y,pmesh->mPositions[i].z*_vScaleGeometry.z);
            }
            size_t numindices = 0;
            for(size_t iface = 0; iface < pmesh->mPosFaces.size(); ++iface) {
                numindices += 3*(pmesh->mPosFaces[iface].mIndices.size()-2);
            }
            g._meshcollision.indices.resize(numindices);
            std::vector<int>::iterator itindex = g._meshcollision.indices.begin();
            for(size_t iface = 0; iface < pmesh->mPosFaces.size(); ++iface) {
                for(size_t i = 2; i < pmesh->mPosFaces[iface].mIndices.size(); ++i) {
                    *itindex++ = pmesh->mPosFaces[iface].mIndices.at(0);
                    *itindex++ = pmesh->mPosFaces[iface].mIndices.at(1);
                    *itindex++ = pmesh->mPosFaces[iface].mIndices.at(i);
                }
            }

            size_t matindex = 0;
            if( pmesh->mFaceMaterials.size() > 0 ) {
                matindex = pmesh->mFaceMaterials.at(0);
            }
            if( matindex < pmesh->mMaterials.size() ) {
                const Assimp::XFile::Material& mtrl = pmesh->mMaterials.at(matindex);
                g._vDiffuseColor = Vector(mtrl.mDiffuse.r, mtrl.mDiffuse.g, mtrl.mDiffuse.b, mtrl.mDiffuse.a);
                g._vAmbientColor = Vector(mtrl.mEmissive.r, mtrl.mEmissive.g, mtrl.mEmissive.b, 1);
            }
            plink->_vGeometries.push_back(KinBody::Link::GeometryPtr(new KinBody::Link::Geometry(plink,g)));
        }

        FOREACH(it,node->mChildren) {
            _Read(pbody, plink, *it,tnode,level+1);
        }
    }

    Transform ExtractTransform(const aiMatrix4x4 &aimatrix)
    {
        TransformMatrix tmnode;
        tmnode.m[0] = aimatrix.a1; tmnode.m[1] = aimatrix.a2; tmnode.m[2] = aimatrix.a3; tmnode.trans[0] = aimatrix.a4*_vScaleGeometry.x;
        tmnode.m[4] = aimatrix.b1; tmnode.m[5] = aimatrix.b2; tmnode.m[6] = aimatrix.b3; tmnode.trans[1] = aimatrix.b4*_vScaleGeometry.y;
        tmnode.m[8] = aimatrix.c1; tmnode.m[9] = aimatrix.c2; tmnode.m[10] = aimatrix.c3; tmnode.trans[2] = aimatrix.c4*_vScaleGeometry.z;
        return tmnode;
    }

    EnvironmentBasePtr _penv;
    std::string _prefix;
    Vector _vScaleGeometry;
    bool _bFlipYZ;
    std::list< pair<KinBody::LinkPtr, Transform> > _listendeffectors;
    bool _bSkipGeometry;
};

bool RaveParseXFile(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& filename,const AttributesList &atts)
{
    if( !ppbody ) {
        ppbody = RaveCreateKinBody(penv,"");
    }
    XFileReader reader(penv);
    string filedata = RaveFindLocalFile(filename);
    if( filedata.size() == 0 ) {
        return false;
    }
    reader.ReadFile(ppbody,filedata,atts);
    return true;
}

bool RaveParseXFile(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& filename,const AttributesList &atts)
{
    if( !pprobot ) {
        pprobot = RaveCreateRobot(penv,"GenericRobot");
    }
    XFileReader reader(penv);
    string filedata = RaveFindLocalFile(filename);
    if( filedata.size() == 0 ) {
        return false;
    }
    reader.ReadFile(pprobot,filedata,atts);
    return true;
}

bool RaveParseXData(EnvironmentBasePtr penv, KinBodyPtr& ppbody, const std::string& data,const AttributesList &atts)
{
    if( !ppbody ) {
        ppbody = RaveCreateKinBody(penv,"");
    }
    XFileReader reader(penv);
    reader.Read(ppbody,data,atts);
    return true;
}

bool RaveParseXData(EnvironmentBasePtr penv, RobotBasePtr& pprobot, const std::string& data,const AttributesList &atts)
{
    if( !pprobot ) {
        pprobot = RaveCreateRobot(penv,"GenericRobot");
    }
    XFileReader reader(penv);
    reader.Read(pprobot,data,atts);
    return true;
}
