// Copyright (C) 2006-2010 Rosen Diankov (rdiankov@cs.cmu.edu)
// -*- coding: utf-8 -*-
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
#define NO_IMPORT_ARRAY
#include "openravepy_int.h"

namespace openravepy {

std::string openravepyCompilerVersion()
{
    stringstream ss;
#if defined(_MSC_VER)
    ss << "msvc " << _MSC_VER;
#elif defined(__GNUC__)
    ss << "gcc " << __GNUC__ << "." << __GNUC_MINOR__ << "." << __GNUC_PATCHLEVEL__;
#elif defined(__MINGW32_VERSION)
    ss << "mingw " << __MINGW32_VERSION;
#endif
    return ss.str();
}

void raveLog(const string& s, DebugLevel level)
{
    if( s.size() > 0 ) {
        RavePrintfA(s,level);
        if( s[s.size()-1] != '\n') {
            RavePrintfA("\n",level);
        }
    }
}

void raveLogFatal(const string& s)
{
    raveLog(s,Level_Verbose);
}
void raveLogError(const string& s)
{
    raveLog(s,Level_Error);
}
void raveLogWarn(const string& s)
{
    raveLog(s,Level_Warn);
}
void raveLogInfo(const string& s)
{
    raveLog(s,Level_Info);
}
void raveLogDebug(const string& s)
{
    raveLog(s,Level_Debug);
}
void raveLogVerbose(const string& s)
{
    raveLog(s,Level_Verbose);
}

object RaveGetPluginInfo()
{
    boost::python::list plugins;
    std::list< std::pair<std::string, PLUGININFO> > listplugins;
    OpenRAVE::RaveGetPluginInfo(listplugins);
    FOREACH(itplugin, listplugins) {
        plugins.append(boost::python::make_tuple(itplugin->first,object(boost::shared_ptr<PyPluginInfo>(new PyPluginInfo(itplugin->second)))));
    }
    return plugins;
}

object RaveGetLoadedInterfaces()
{
    std::map<InterfaceType, std::vector<std::string> > interfacenames;
    OpenRAVE::RaveGetLoadedInterfaces(interfacenames);
    boost::python::list ointerfacenames;
    FOREACHC(it, interfacenames) {
        boost::python::list names;
        FOREACHC(itname,it->second) {
            names.append(*itname);
        }
        ointerfacenames.append(boost::python::make_tuple(it->first,names));
    }
    return ointerfacenames;
}

object quatFromAxisAngle1(object oaxis)
{
    return toPyVector4(quatFromAxisAngle(ExtractVector3(oaxis)));
}

object quatFromAxisAngle2(object oaxis, dReal angle)
{
    return toPyVector4(quatFromAxisAngle(ExtractVector3(oaxis),angle));
}

object quatFromRotationMatrix(object R)
{
    TransformMatrix t;
    t.rotfrommat(extract<dReal>(R[0][0]), extract<dReal>(R[0][1]), extract<dReal>(R[0][2]),
                 extract<dReal>(R[1][0]), extract<dReal>(R[1][1]), extract<dReal>(R[1][2]),
                 extract<dReal>(R[2][0]), extract<dReal>(R[2][1]), extract<dReal>(R[2][2]));
    return toPyVector4(quatFromMatrix(t));
}

object quatSlerp(object q1, object q2, dReal t)
{
    return toPyVector4(quatSlerp(ExtractVector4(q1),ExtractVector4(q2),t));
}

object axisAngleFromRotationMatrix(object R)
{
    TransformMatrix t;
    t.rotfrommat(extract<dReal>(R[0][0]), extract<dReal>(R[0][1]), extract<dReal>(R[0][2]),
                 extract<dReal>(R[1][0]), extract<dReal>(R[1][1]), extract<dReal>(R[1][2]),
                 extract<dReal>(R[2][0]), extract<dReal>(R[2][1]), extract<dReal>(R[2][2]));
    return toPyVector3(axisAngleFromMatrix(t));
}

object axisAngleFromQuat(object oquat)
{
    return toPyVector3(axisAngleFromQuat(ExtractVector4(oquat)));
}

object rotationMatrixFromQuat(object oquat)
{
    return toPyArrayRotation(matrixFromQuat(ExtractVector4(oquat)));
}

object rotationMatrixFromQArray(object qarray)
{
    boost::python::list orots;
    int N = len(qarray);
    for(int i = 0; i < N; ++i)
        orots.append(rotationMatrixFromQuat(qarray[i]));
    return orots;
}

object matrixFromQuat(object oquat)
{
    return toPyArray(matrixFromQuat(ExtractVector4(oquat)));
}

object rotationMatrixFromAxisAngle1(object oaxis)
{
    return toPyArrayRotation(matrixFromAxisAngle(ExtractVector3(oaxis)));
}

object rotationMatrixFromAxisAngle2(object oaxis, dReal angle)
{
    return toPyArrayRotation(matrixFromAxisAngle(ExtractVector3(oaxis),angle));
}

object matrixFromAxisAngle1(object oaxis)
{
    return toPyArray(matrixFromAxisAngle(ExtractVector3(oaxis)));
}

object matrixFromAxisAngle2(object oaxis, dReal angle)
{
    return toPyArray(matrixFromAxisAngle(ExtractVector3(oaxis),angle));
}

object matrixFromPose(object opose)
{
    return toPyArray(TransformMatrix(ExtractTransformType<dReal>(opose)));
}

object matrixFromPoses(object oposes)
{
    boost::python::list omatrices;
    int N = len(oposes);
    for(int i = 0; i < N; ++i)
        omatrices.append(matrixFromPose(oposes[i]));
    return omatrices;
}

object poseFromMatrix(object o)
{
    TransformMatrix t;
    for(int i = 0; i < 3; ++i) {
        t.m[4*i+0] = extract<dReal>(o[i][0]);
        t.m[4*i+1] = extract<dReal>(o[i][1]);
        t.m[4*i+2] = extract<dReal>(o[i][2]);
        t.trans[i] = extract<dReal>(o[i][3]);
    }
    return toPyArray(Transform(t));
}

object poseFromMatrices(object otransforms)
{
    int N = len(otransforms);
    if( N == 0 )
        return static_cast<numeric::array>(handle<>());

    npy_intp dims[] = {N,7};
    PyObject *pyvalues = PyArray_SimpleNew(2,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
    dReal* pvalues = (dReal*)PyArray_DATA(pyvalues);
    TransformMatrix tm;
    for(int j = 0; j < N; ++j) {
        object o = otransforms[j];
        for(int i = 0; i < 3; ++i) {
            tm.m[4*i+0] = extract<dReal>(o[i][0]);
            tm.m[4*i+1] = extract<dReal>(o[i][1]);
            tm.m[4*i+2] = extract<dReal>(o[i][2]);
            tm.trans[i] = extract<dReal>(o[i][3]);
        }
        Transform tpose(tm);
        pvalues[0] = tpose.rot.x; pvalues[1] = tpose.rot.y; pvalues[2] = tpose.rot.z; pvalues[3] = tpose.rot.w;
        pvalues[4] = tpose.trans.x; pvalues[5] = tpose.trans.y; pvalues[6] = tpose.trans.z;
        pvalues += 7;
    }
    return static_cast<numeric::array>(handle<>(pyvalues));
}

object invertPoses(object o)
{
    int N = len(o);
    if( N == 0 )
        return numeric::array(boost::python::list());

    npy_intp dims[] = {N,7};
    PyObject *pytrans = PyArray_SimpleNew(2,dims, sizeof(dReal)==8?PyArray_DOUBLE:PyArray_FLOAT);
    dReal* ptrans = (dReal*)PyArray_DATA(pytrans);
    for(int i = 0; i < N; ++i, ptrans += 7) {
        object oinputtrans = o[i];
        Transform t = Transform(Vector(extract<dReal>(oinputtrans[0]),extract<dReal>(oinputtrans[1]),extract<dReal>(oinputtrans[2]),extract<dReal>(oinputtrans[3])),
                                Vector(extract<dReal>(oinputtrans[4]),extract<dReal>(oinputtrans[5]),extract<dReal>(oinputtrans[6]))).inverse();
        ptrans[0] = t.rot.x; ptrans[1] = t.rot.y; ptrans[2] = t.rot.z; ptrans[3] = t.rot.w;
        ptrans[4] = t.trans.x; ptrans[5] = t.trans.y; ptrans[6] = t.trans.z;
    }
    return static_cast<numeric::array>(handle<>(pytrans));
}

object quatRotateDirection(object source, object target)
{
    return toPyVector4(quatRotateDirection(ExtractVector3(source), ExtractVector3(target)));
}

object quatMult(object oquat1, object oquat2)
{
    return toPyVector4(quatMultiply(ExtractVector4(oquat1),ExtractVector4(oquat2)));
}

object poseMult(object opose1, object opose2)
{
    return toPyArray(ExtractTransformType<dReal>(opose1)*ExtractTransformType<dReal>(opose2));
}

object transformLookat(object olookat, object ocamerapos, object ocameraup)
{
    return toPyArray(transformLookat(ExtractVector3(olookat),ExtractVector3(ocamerapos),ExtractVector3(ocameraup)));
}

string matrixSerialization(object o)
{
    stringstream ss; ss << ExtractTransformMatrix(o);
    return ss.str();
}

string poseSerialization(object o)
{
    stringstream ss; ss << ExtractTransform(o);
    return ss.str();
}

BOOST_PYTHON_FUNCTION_OVERLOADS(RaveInitialize_overloads, RaveInitialize, 0, 2)

void init_openravepy_global()
{
    def("RaveSetDebugLevel",OpenRAVE::RaveSetDebugLevel,args("level"), DOXY_FN1(RaveSetDebugLevel));
    def("RaveGetDebugLevel",OpenRAVE::RaveGetDebugLevel,DOXY_FN1(RaveGetDebugLevel));
    def("RaveGetHomeDirectory",OpenRAVE::RaveGetHomeDirectory,DOXY_FN1(RaveGetHomeDirectory));
    def("RaveLogFatal",openravepy::raveLogFatal,args("log"),"Send a fatal log to the openrave system");
    def("RaveLogError",openravepy::raveLogError,args("log"),"Send an error log to the openrave system");
    def("RaveLogWarn",openravepy::raveLogWarn,args("log"),"Send a warn log to the openrave system");
    def("RaveLogInfo",openravepy::raveLogInfo,args("log"),"Send an info log to the openrave system");
    def("RaveLogDebug",openravepy::raveLogDebug,args("log"),"Send a debug log to the openrave system");
    def("RaveLogVerbose",openravepy::raveLogVerbose,args("log"),"Send a verbose log to the openrave system");
    def("RaveLog",openravepy::raveLog,args("log","level"),"Send a log to the openrave system with excplicit level");
    def("RaveInitialize",RaveInitialize,RaveInitialize_overloads(args("load_all_plugins","level"),DOXY_FN1(RaveInitialize)));
    def("RaveDestroy",RaveDestroy,DOXY_FN1(RaveDestroy));
    def("RaveGetPluginInfo",openravepy::RaveGetPluginInfo,DOXY_FN1(RaveGetPluginInfo));
    def("RaveGetLoadedInterfaces",openravepy::RaveGetLoadedInterfaces,DOXY_FN1(raveGetLoadedInterfaces));
    def("RaveReloadPlugins",OpenRAVE::RaveReloadPlugins,DOXY_FN1(RaveReloadPlugins));
    def("RaveLoadPlugin",OpenRAVE::RaveLoadPlugin,args("filename"),DOXY_FN1(RaveLoadPlugins));
    def("RaveHasInterface",OpenRAVE::RaveHasInterface,args("type","name"),DOXY_FN1(RaveHasInterface));

    def("raveSetDebugLevel",OpenRAVE::RaveSetDebugLevel,args("level"), DOXY_FN1(RaveSetDebugLevel));
    def("raveGetDebugLevel",OpenRAVE::RaveGetDebugLevel,DOXY_FN1(RaveGetDebugLevel));
    def("raveLogFatal",openravepy::raveLogFatal,args("log"),"Send a fatal log to the openrave system");
    def("raveLogError",openravepy::raveLogError,args("log"),"Send an error log to the openrave system");
    def("raveLogWarn",openravepy::raveLogWarn,args("log"),"Send a warn log to the openrave system");
    def("raveLogInfo",openravepy::raveLogInfo,args("log"),"Send an info log to the openrave system");
    def("raveLogDebug",openravepy::raveLogDebug,args("log"),"Send a debug log to the openrave system");
    def("raveLogVerbose",openravepy::raveLogVerbose,args("log"),"Send a verbose log to the openrave system");
    def("raveLog",openravepy::raveLog,args("log","level"),"Send a log to the openrave system with excplicit level");

    def("quatFromAxisAngle",openravepy::quatFromAxisAngle1, args("axisangle"), DOXY_FN1(quatFromAxisAngle "const RaveVector"));
    def("quatFromAxisAngle",openravepy::quatFromAxisAngle2, args("axis","angle"), DOXY_FN1(quatFromAxisAngle "const RaveVector; T"));
    def("quatFromRotationMatrix",openravepy::quatFromRotationMatrix, args("rotation"), DOXY_FN1(quatFromMatrix "const RaveTransform"));
    def("quatSlerp",openravepy::quatSlerp, args("quat0","quat1","t"), DOXY_FN1(quatSlerp "const RaveVector; const RaveVector; T"));
    def("axisAngleFromRotationMatrix",openravepy::axisAngleFromRotationMatrix, args("rotation"), DOXY_FN1(axisAngleFromMatrix "const RaveTransformMatrix"));
    def("axisAngleFromQuat",openravepy::axisAngleFromQuat, args("quat"), DOXY_FN1(axisAngleFromQuat "const RaveVector"));
    def("rotationMatrixFromQuat",openravepy::rotationMatrixFromQuat, args("quat"), DOXY_FN1(matrixFromQuat "const RaveVector"));
    def("rotationMatrixFromQArray",openravepy::rotationMatrixFromQArray,args("quatarray"),"Converts an array of quaternions to a list of 3x3 rotation matrices.\n\n:param quatarray: nx4 array\n");
    def("matrixFromQuat",openravepy::matrixFromQuat, args("quat"), "Converts a quaternion to a 4x4 affine matrix.\n\n:param quat: 4 values\n");
    def("rotationMatrixFromAxisAngle",openravepy::rotationMatrixFromAxisAngle1, args("axisangle"), DOXY_FN1(matrixFromAxisAngle "const RaveVector"));
    def("rotationMatrixFromAxisAngle",openravepy::rotationMatrixFromAxisAngle2, args("axis","angle"), DOXY_FN1(matrixFromAxisAngle "const RaveVector, T"));
    def("matrixFromAxisAngle",openravepy::matrixFromAxisAngle1, args("axisangle"), DOXY_FN1(matrixFromAxisAngle "const RaveVector"));
    def("matrixFromAxisAngle",openravepy::matrixFromAxisAngle2, args("axis","angle"), DOXY_FN1(matrixFromAxisAngle "const RaveVector, T"));
    def("matrixFromPose",openravepy::matrixFromPose, args("pose"), "Converts a 7 element quaterion+translation transform to a 4x4 matrix.\n\n:param pose: 7 values\n");
    def("matrixFromPoses",openravepy::matrixFromPoses, args("poses"), "Converts a Nx7 element quaterion+translation array to a 4x4 matrices.\n\n:param poses: nx7 array\n");
    def("poseFromMatrix",openravepy::poseFromMatrix, args("transform"), "Converts a 4x4 matrix to a 7 element quaternion+translation representation.\n\n:param transform: 3x4 or 4x4 affine matrix\n");
    def("poseFromMatrices",openravepy::poseFromMatrices, args("transforms"), "Converts an array/list of 4x4 matrices to a Nx7 array where each row is quaternion+translation representation.\n\n:param transforms: list of 3x4 or 4x4 affine matrices\n");
    def("invertPoses",openravepy::invertPoses,args("poses"), "Inverts a Nx7 array of poses where first 4 columns are the quaternion and last 3 are the translation components.\n\n:param poses: nx7 array");
    def("quatRotateDirection",openravepy::quatRotateDirection,args("sourcedir,targetdir"), DOXY_FN1(quatRotateDirection));
    def("quatMult",openravepy::quatMult,args("quat0","quat1"),DOXY_FN1(quatMultiply));
    def("quatMultiply",openravepy::quatMult,args("quat0","quat1"),DOXY_FN1(quatMultiply));
    def("poseMult",openravepy::poseMult,args("pose1","pose2"),"multiplies two poses.\n\n:param pose1: 7 values\n\n:param pose2: 7 values\n");
    def("transformLookat",openravepy::transformLookat,args("lookat","camerapos","cameraup"),"Returns a camera matrix that looks along a ray with a desired up vector.\n\n:param lookat: unit axis, 3 values\n\n:param camerapos: 3 values\n\n:param cameraup: unit axis, 3 values\n");
    def("matrixSerialization",openravepy::matrixSerialization,args("transform"),"Serializes a transformation into a string representing a 3x4 matrix.\n\n:param transform: 3x4 or 4x4 array\n");
    def("poseSerialization",openravepy::poseSerialization, args("pose"), "Serializes a transformation into a string representing a quaternion with translation.\n\n:param pose: 7 values\n");
    def("openravepyCompilerVersion",openravepy::openravepyCompilerVersion,"Returns the compiler version that openravepy_int was compiled with");
}

} // end namespace openravepy
