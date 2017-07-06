// -*- coding: utf-8 -*-
// Copyright (C) 2013 Rosen Diankov <rosen.diankov@gmail.com>
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
/** \file openrave_c.h
    \brief  Defines C public headers
 */
#ifndef OPENRAVE_C_H
#define OPENRAVE_C_H

#include <openrave/config.h>

// Now we use the generic helper definitions above to define OPENRAVE_C_API and OPENRAVE_C_LOCAL.
// OPENRAVE_C_API is used for the public API symbols. It either DLL imports or DLL exports (or does nothing for static build)
// OPENRAVE_C_LOCAL is used for non-api symbols.
#if defined(OPENRAVE_C_DLL) // defined if OpenRAVE is compiled as a DLL
  #ifdef OPENRAVE_C_DLL_EXPORTS // defined if we are building the OpenRAVE DLL (instead of using it)
    #define OPENRAVE_C_API OPENRAVE_HELPER_DLL_EXPORT
  #else
    #define OPENRAVE_C_API OPENRAVE_HELPER_DLL_IMPORT
  #endif // OPENRAVE_DLL_EXPORTS
  #define OPENRAVE_C_LOCAL OPENRAVE_HELPER_DLL_LOCAL
#else // OPENRAVE_DLL is not defined: this means OpenRAVE is a static lib.
  #define OPENRAVE_C_API
  #define OPENRAVE_C_LOCAL
#endif // OPENRAVE_DLL


#if OPENRAVE_PRECISION // 1 if double precision
typedef double OpenRAVEReal;
#define g_fEpsilon 1e-15
#else
typedef float OpenRAVEReal;
#define g_fEpsilon 2e-7f
#endif

#ifdef __cplusplus
extern "C" {
#endif

enum DebugLevel {
    Level_Fatal=0,
    Level_Error=1,
    Level_Warn=2,
    Level_Info=3,
    Level_Debug=4,
    Level_Verbose=5,
    Level_OutputMask=0xf,
    Level_VerifyPlans=0x80000000, ///< if set, should verify every plan returned. the verification is left up to the planners or the modules calling the planners. See \ref planningutils::ValidateTrajectory
};

/// \brief Calls \ref RaveSetDebugLevel
OPENRAVE_C_API void ORCSetDebugLevel(int level);

/// \brief Calls \ref RaveInitialize
OPENRAVE_C_API void ORCInitialize(int bLoadAllPlugins, int level);

/// \brief Calls \ref RaveDestroy
OPENRAVE_C_API void ORCDestroy();

/// \brief Calls \ref RaveCreateKinBody
OPENRAVE_C_API void* ORCCreateKinBody(void* env, const char* name);

/// \brief Creates a new \ref OpenRAVE::TriMesh object and copies the vertex/index data.
///
/// Have to call ORCTriMeshDestroy afterwards.
/// \param vertex a numvertices*3 real array
/// \param index a numtriangles*3 int array for every triangle
OPENRAVE_C_API void* ORCCreateTriMesh(OpenRAVEReal* vertices, int numvertices, OpenRAVEReal* indices, int numtriangles);

/// \brief frees the trimesh data returned from \ref ORCTriMeshCreate
OPENRAVE_C_API void ORCTriMeshDestroy(void* trimesh);

/// \name \ref EnvironmentBase methods
//@{

/// \brief Calls \ref EnvironmentBase::Destroy
OPENRAVE_C_API void ORCEnvironmentDestroy(void* env);

OPENRAVE_C_API int ORCEnvironmentLoad(void* env, const char* filename);

/// \brief Calls \ref EnvironmentBase::GetKinBody
OPENRAVE_C_API void* ORCEnvironmentGetKinBody(void* env, const char* name);

/// \brief Calls \ref EnvironmentBase::GetBodies
///
/// Have to release each of the robot pointers with \ref ORCInterfaceRelease.
/// If NULL, the function will not fill the bodies list only return the number of bodies available.
/// \brief env pointer to environment
/// \brief bodies a pre-allocated list of body pointers to fill
/// \return number of bodies
OPENRAVE_C_API int ORCEnvironmentGetBodies(void* env, void** bodies);

/// \brief Calls \ref EnvironmentBase::GetRobots
///
/// Have to release each of the robot pointers with \ref ORCInterfaceRelease.
/// If NULL, the function will not fill the robots list only return the number of robots available.
/// \brief env pointer to environment
/// \brief robots a pre-allocated list of robot pointers to fill
/// \return number of robots
OPENRAVE_C_API int ORCEnvironmentGetRobots(void* env, void** robots);

/// \brief Calls \ref EnvironmentBase::Add
///
/// \param pinterface can be body, robot, sensor, etc (any type derived from InterfaceBase)
OPENRAVE_C_API void ORCEnvironmentAdd(void* env, void* pinterface);

/// \brief Calls \ref EnvironmentBase::AddModule
OPENRAVE_C_API int ORCEnvironmentAddModule(void* env, void* module, const char* args);

/// \brief Calls \ref EnvironmentBase::Remove
OPENRAVE_C_API void ORCEnvironmentRemove(void* env, void* pinterface);

/// \brief Calls \ref EnvironmentBase::GetSimulationTime
OPENRAVE_C_API unsigned long long ORCEnvironmentGetSimulationTime(void* env);

/// \brief Return the global environment mutex used to protect environment information access in multi-threaded environments.
///
/// Accessing environment body information and adding/removing bodies
/// or changing any type of scene property should have the environment lock acquired. Once the environment
/// is locked, the user is guaranteed that nnothing will change in the environment.
OPENRAVE_C_API void ORCEnvironmentLock(void* env);

/// \brief unlock an already locked mutex
OPENRAVE_C_API void ORCEnvironmentUnlock(void* env);

/// \brief Starts a viewer thread for the current environment
OPENRAVE_C_API int ORCEnvironmentSetViewer(void* env, const char* viewername);

//@}

/// \name \ref InterfaceBase methods
//@{

/// \brief release the interfaces like robots, modules, etc.
OPENRAVE_C_API void ORCInterfaceRelease(void* pinterface);

/// \brief Calls \ref InterfaceBase::SendCommand
///
/// If command failed, will return NULL string.
/// \return The output string. It has to be released with free() by the user.
OPENRAVE_C_API char* ORCInterfaceSendCommand(void* pinterface, const char* command);

//@}

/// \name \ref KinBody methods
//@{

/// \brief Calls \ref KinBody::GetName
OPENRAVE_C_API const char* ORCBodyGetName(void* body);

/// \brief Calls \ref KinBody::SetName
OPENRAVE_C_API void ORCBodySetName(void* body, const char* name);

/// \brief Calls \ref KinBody::GetDOF
OPENRAVE_C_API int ORCBodyGetDOF(void* body);

/// \brief Calls \ref KinBody::GetDOFValues
///
/// \param[out] values fills the already initialized array of DOF values
OPENRAVE_C_API void ORCBodyGetDOFValues(void* body, OpenRAVEReal* values);

/// \brief Calls \ref KinBody::SetDOFValues
///
/// \param[in] values uses this array to set the DOF values.
OPENRAVE_C_API void ORCBodySetDOFValues(void* body, const OpenRAVEReal* values);

/// \brief Calls \ref KinBody::GetLinks
///
/// Have to release each of the link pointers with \ref ORCBodyLinkRelease.
/// If NULL, the function will not fill the links list only return the number of links available.
/// \brief env pointer to environment
/// \brief links a pre-allocated list of link pointers to fill
/// \return number of links
OPENRAVE_C_API int ORCBodyGetLinks(void* body, void** links);

/// \brief Calls \ref KinBody::SetTransform
///
/// \param[in] pose 7 values of the quaternion (4) and translation (3) of the world pose of the transform.
OPENRAVE_C_API void ORCBodySetTransform(void* body, const OpenRAVEReal* pose);

/// \brief Calls \ref KinBody::SetTransform
///
/// \param[in] matrix column-order, row-major 3x4 matrix of the body world transform
OPENRAVE_C_API void ORCBodySetTransformMatrix(void* body, const OpenRAVEReal* matrix);

/// \brief Calls \ref KinBody::GetTransform
///
/// \param[out] pose 7 values of the quaternion (4) and translation (3) of the world pose of the transform.
OPENRAVE_C_API void ORCBodyGetTransform(void* body, OpenRAVEReal* pose);

/// \brief Calls \ref KinBody::GetTransform
///
/// \param[out] matrix column-order, row-major 3x4 matrix of the body world transform
OPENRAVE_C_API void ORCBodyGetTransformMatrix(void* body, OpenRAVEReal* matrix);

/// \brief Calls \ref KinBody::InitFromTrimesh
///
/// \param trimesh returned from ORCTriMeshCreate()
OPENRAVE_C_API int ORCBodyInitFromTrimesh(void* body, void* trimesh, int visible);

//@}

/// \name \ref KinBody::Link methods
//@{

/// \brief Calls \ref KinBody::Link::GetGeometries
///
/// Have to release each of the link pointers with \ref ORCBodyGeometryRelease.
/// If NULL, the function will not fill the geometries list only return the number of geometries available.
/// \brief env pointer to environment
/// \brief geometries a pre-allocated list of link pointers to fill
/// \return number of geometries
OPENRAVE_C_API int ORCBodyLinkGetGeometries(void* link, void** geometries);

/// \brief release the body's link
OPENRAVE_C_API void ORCBodyLinkRelease(void* link);

//@}

/// \name \ref KinBody::Link::Geometry methods
//@{

/// \brief Calls \ref KinBody::Link::Geometry::SetDiffuseColor
OPENRAVE_C_API void ORCBodyGeometrySetDiffuseColor(void* geometry, float red, float green, float blue);

//@}

OPENRAVE_C_API const char* ORCRobotGetName(void* robot);

/// \brief Calls \ref RaveCreateModule
///
/// Have to release the module pointer with \ref ORCModuleRelease
OPENRAVE_C_API void* ORCModuleCreate(void* env, const char* modulename);

#ifdef __cplusplus
}
#endif

#endif
