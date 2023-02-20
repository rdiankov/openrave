if( UNIX )
  add_definitions("-DLINUX")
endif()

check_include_file_cxx("ext/hash_map" HAVE_EXT_HASH_MAP_H)
if( HAVE_EXT_HASH_MAP_H )
  add_definitions("-DUSE_EXT_HASH_MAP")
endif()

add_library(convexdecomposition STATIC
  NvConcavityVolume.cpp      NvFloatMath.cpp             NvRayCast.cpp           NvSplitMesh.cpp  NvThreadConfig.cpp  wavefront.cpp
  NvConvexDecomposition.cpp  NvMeshIslandGeneration.cpp  NvRemoveTjunctions.cpp  NvStanHull.cpp
  NvConcavityVolume.h      NvHashMap.h               NvRemoveTjunctions.h  NvStanHull.h      wavefront.h
  NvConvexDecomposition.h  NvMeshIslandGeneration.h  NvSimpleTypes.h       NvThreadConfig.h
  NvFloatMath.h            NvRayCast.h               NvSplitMesh.h         NvUserMemAlloc.h
  NvFloatMath.inl )
