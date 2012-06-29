/*
 * Copyright 2006 Sony Computer Entertainment Inc.
 *
 * Licensed under the MIT Open Source License, for details please see license.txt or the website
 * http://www.opensource.org/licenses/mit-license.php
 *
 */
#include <dae.h>
#include <dae/daeDom.h>
#include <dae/daeMetaElement.h>
#include <dom.h>
#ifdef COLLADA_DOM_SUPPORT150
#include <1.5/dom/domAny.h>
#include <1.5/dom/domSource.h>
#include <1.5/dom/domCOLLADA.h>
#include <1.5/dom/domConstants.h>
#endif
#ifdef COLLADA_DOM_SUPPORT141
#include <1.4/dom/domAny.h>
#include <1.4/dom/domSource.h>
#include <1.4/dom/domCOLLADA.h>
#include <1.4/dom/domConstants.h>
#endif

daeMetaElement* initializeDomMeta(DAE& dae, const char* specversion)
{
#ifdef COLLADA_DOM_SUPPORT150
    if( specversion == NULL || strcmp(specversion,"1.5.0") == 0 ) {
        ColladaDOM150::registerDomTypes(dae);
        return ColladaDOM150::registerDomElements(dae);
    }
#endif
#ifdef COLLADA_DOM_SUPPORT141
    if( specversion == NULL || strcmp(specversion,"1.4.1") == 0 ) {
        ColladaDOM141::registerDomTypes(dae);
        return ColladaDOM141::registerDomElements(dae);
    }
#endif
    return NULL;
}

daeInt GetColladaTypeCount(const char* specversion)
{
#ifdef COLLADA_DOM_SUPPORT150
    if( specversion == NULL || strcmp(specversion,"1.5.0") == 0 ) {
        return ColladaDOM150::colladaTypeCount();
    }
#endif
#ifdef COLLADA_DOM_SUPPORT141
    if( specversion == NULL || strcmp(specversion,"1.4.1") == 0 ) {
        return ColladaDOM141::colladaTypeCount();
    }
#endif
    return 0;
}

daeString GetCOLLADA_VERSION(const char* specversion)
{
#ifdef COLLADA_DOM_SUPPORT150
    if( specversion == NULL || strcmp(specversion,"1.5.0") == 0 ) {
        return ColladaDOM150::COLLADA_VERSION;
    }
#endif
#ifdef COLLADA_DOM_SUPPORT141
    if( specversion == NULL || strcmp(specversion,"1.4.1") == 0 ) {
        return ColladaDOM141::COLLADA_VERSION;
    }
#endif
    return "";
}

daeString GetCOLLADA_NAMESPACE(const char* specversion)
{
#ifdef COLLADA_DOM_SUPPORT150
    if( specversion == NULL || strcmp(specversion,"1.5.0") == 0 ) {
        return ColladaDOM150::COLLADA_NAMESPACE;
    }
#endif
#ifdef COLLADA_DOM_SUPPORT141
    if( specversion == NULL || strcmp(specversion,"1.4.1") == 0 ) {
        return ColladaDOM141::COLLADA_NAMESPACE;
    }
#endif
    return "";
}

daeMetaElement * registerElementAny(DAE& dae)
{
#ifdef COLLADA_DOM_SUPPORT150
    if( strcmp(dae.getDomVersion(),"1.5.0") == 0 ) {
        return ColladaDOM150::domAny::registerElement(dae);
    }
#endif
#ifdef COLLADA_DOM_SUPPORT141
    if( strcmp(dae.getDomVersion(),"1.4.1") == 0 ) {
        return ColladaDOM141::domAny::registerElement(dae);
    }
#endif
    return NULL;
}

daeInt getDomAnyID(DAE& dae)
{
#ifdef COLLADA_DOM_SUPPORT150
    if( strcmp(dae.getDomVersion(),"1.5.0") == 0 ) {
        return ColladaDOM150::domAny::ID();
    }
#endif
#ifdef COLLADA_DOM_SUPPORT141
    if( strcmp(dae.getDomVersion(),"1.4.1") == 0 ) {
        return ColladaDOM141::domAny::ID();
    }
#endif
    return NULL;
}

daeInt getDomSourceID(DAE& dae)
{
#ifdef COLLADA_DOM_SUPPORT150
    if( strcmp(dae.getDomVersion(),"1.5.0") == 0 ) {
        return ColladaDOM150::domSource::ID();
    }
#endif
#ifdef COLLADA_DOM_SUPPORT141
    if( strcmp(dae.getDomVersion(),"1.4.1") == 0 ) {
        return ColladaDOM141::domSource::ID();
    }
#endif
    return NULL;
}

daeInt getDomCOLLADAID(const char* specversion)
{
#ifdef COLLADA_DOM_SUPPORT150
    if( specversion == NULL || strcmp(specversion,"1.5.0") == 0 ) {
        return ColladaDOM150::domCOLLADA::ID();
    }
#endif
#ifdef COLLADA_DOM_SUPPORT141
    if( specversion == NULL || strcmp(specversion,"1.4.1") == 0 ) {
        return ColladaDOM141::domCOLLADA::ID();
    }
#endif
    return NULL;
}

void copyElementAny(daeElementRef dstAny, daeElement* srcAny)
{
#ifdef COLLADA_DOM_SUPPORT150
    if( strcmp(srcAny->getDAE()->getDomVersion(),"1.5.0") == 0 ) {
        ColladaDOM150::domAny* thisAny = (ColladaDOM150::domAny*)srcAny;
        ColladaDOM150::domAny* retAny = (ColladaDOM150::domAny*)dstAny.cast();
        for (daeUInt i = 0; i < (daeUInt)thisAny->getAttributeCount(); i++)
            retAny->setAttribute(thisAny->getAttributeName(i), thisAny->getAttributeValue(i));
        retAny->setValue(thisAny->getValue());
    }
#endif
#ifdef COLLADA_DOM_SUPPORT141
    if( strcmp(srcAny->getDAE()->getDomVersion(),"1.4.1") == 0 ) {
        ColladaDOM141::domAny* thisAny = (ColladaDOM141::domAny*)srcAny;
        ColladaDOM141::domAny* retAny = (ColladaDOM141::domAny*)dstAny.cast();
        for (daeUInt i = 0; i < (daeUInt)thisAny->getAttributeCount(); i++)
            retAny->setAttribute(thisAny->getAttributeName(i), thisAny->getAttributeValue(i));
        retAny->setValue(thisAny->getValue());
    }
#endif
}

daeString COLLADA_ELEMENT_TECHNIQUE_COMMON(DAE& dae)
{
#ifdef COLLADA_DOM_SUPPORT150
    if( strcmp(dae.getDomVersion(),"1.5.0") == 0 ) {
        return ColladaDOM150::COLLADA_ELEMENT_TECHNIQUE_COMMON;
    }
#endif
#ifdef COLLADA_DOM_SUPPORT141
    if( strcmp(dae.getDomVersion(),"1.4.1") == 0 ) {
        return ColladaDOM141::COLLADA_ELEMENT_TECHNIQUE_COMMON;
    }
#endif
    return daeString();
}

daeString COLLADA_ELEMENT_TECHNIQUE(DAE& dae)
{
#ifdef COLLADA_DOM_SUPPORT150
    if( strcmp(dae.getDomVersion(),"1.5.0") == 0 ) {
        return ColladaDOM150::COLLADA_ELEMENT_TECHNIQUE;
    }
#endif
#ifdef COLLADA_DOM_SUPPORT141
    if( strcmp(dae.getDomVersion(),"1.4.1") == 0 ) {
        return ColladaDOM141::COLLADA_ELEMENT_TECHNIQUE;
    }
#endif
    return daeString();
}

daeDoubleArray* getDomSourceFloatArray(daeElement* elt)
{
#ifdef COLLADA_DOM_SUPPORT150
    if( strcmp(elt->getDAE()->getDomVersion(),"1.5.0") == 0 ) {
        if( elt->typeID() == ColladaDOM150::domSource::ID() ) {
            ColladaDOM150::domFloat_array* floatArray = ((ColladaDOM150::domSource*)elt)->getFloat_array();
            if( floatArray != NULL ) {
                return (daeDoubleArray*)floatArray->getCharDataObject()->get(floatArray);
            }
        }
        return NULL;
    }
#endif
#ifdef COLLADA_DOM_SUPPORT141
    if( strcmp(elt->getDAE()->getDomVersion(),"1.4.1") == 0 ) {
        if( elt->typeID() == ColladaDOM141::domSource::ID() ) {
            ColladaDOM141::domFloat_array* floatArray = ((ColladaDOM141::domSource*)elt)->getFloat_array();
            if( floatArray != NULL ) {
                return (daeDoubleArray*)floatArray->getCharDataObject()->get(floatArray);
            }
        }
        return NULL;
    }
#endif
    return NULL;
}
