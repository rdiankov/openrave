/*
 * Copyright 2006 Sony Computer Entertainment Inc.
 *
 * Licensed under the MIT Open Source License, for details please see license.txt or the website
 * http://www.opensource.org/licenses/mit-license.php
 *
 */

// \file daeDom.h holds all the bindings between dae and different versions of the doms. allows dae functions to use the current loaded dom seamlessly.
// specversion the collada specification to load into memory. For example: "1.4.1" or "1.5.0". If NULL, then the highest version found will be loaded.
#ifndef __DAE_DOM__
#define __DAE_DOM__

class daeMetaElement;
class DAE;

daeMetaElement* initializeDomMeta(DAE& dae, const char* specversion=NULL);

daeInt GetColladaTypeCount(const char* specversion=NULL);

daeString GetCOLLADA_VERSION(const char* specversion=NULL);

daeString GetCOLLADA_NAMESPACE(const char* specversion=NULL);

daeMetaElement * registerElementAny(DAE& dae);

void copyElementAny(daeElementRef dstAny, daeElement* srcAny);

daeInt getDomAnyID(DAE& dae);
daeInt getDomSourceID(DAE& dae);
daeInt getDomCOLLADAID(const char* specversion = NULL);

daeString COLLADA_ELEMENT_TECHNIQUE_COMMON(DAE& dae);
daeString COLLADA_ELEMENT_TECHNIQUE(DAE& dae);

daeDoubleArray* getDomSourceFloatArray(daeElement* elt);

#endif //__DAE_DOM__
