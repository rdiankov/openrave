/*
 * Copyright 2006 Sony Computer Entertainment Inc.
 *
 * Licensed under the MIT Open Source License, for details please see license.txt or the website
 * http://www.opensource.org/licenses/mit-license.php
 *
 */
#ifndef __DOM__
#define __DOM__

class DAE;
class daeMetaElement;

#ifdef COLLADA_DOM_SUPPORT150
namespace ColladaDOM150 {
extern DLLSPEC daeString COLLADA_VERSION;
extern DLLSPEC daeString COLLADA_NAMESPACE;

// Register all types
DLLSPEC void registerDomTypes(DAE& dae);

// Register all elements
DLLSPEC daeMetaElement* registerDomElements(DAE& dae);

DLLSPEC daeInt colladaTypeCount();

}
#endif

#ifdef COLLADA_DOM_SUPPORT141
namespace ColladaDOM141 {
extern DLLSPEC daeString COLLADA_VERSION;
extern DLLSPEC daeString COLLADA_NAMESPACE;

// Register all types
DLLSPEC void registerDomTypes(DAE& dae);

// Register all elements
DLLSPEC daeMetaElement* registerDomElements(DAE& dae);

DLLSPEC daeInt colladaTypeCount();

}
#endif

#endif // __DOM_INTERFACE__
