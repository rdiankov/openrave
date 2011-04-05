#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domFx_common_float_or_param.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domFx_common_float_or_param::create(DAE& dae)
{
	domFx_common_float_or_paramRef ref = new domFx_common_float_or_param(dae);
	return ref;
}


daeMetaElement *
domFx_common_float_or_param::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "fx_common_float_or_param" );
	meta->registerClass(domFx_common_float_or_param::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float" );
	mea->setOffset( daeOffsetOf(domFx_common_float_or_param,elemFloat) );
	mea->setElementType( domFx_common_float_or_param::domFloat::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "param" );
	mea->setOffset( daeOffsetOf(domFx_common_float_or_param,elemParam) );
	mea->setElementType( domFx_common_float_or_param::domParam::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domFx_common_float_or_param,_contents));
	meta->addContentsOrder(daeOffsetOf(domFx_common_float_or_param,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domFx_common_float_or_param,_CMData), 1);
	meta->setElementSize(sizeof(domFx_common_float_or_param));
	meta->validate();

	return meta;
}

daeElementRef
domFx_common_float_or_param::domFloat::create(DAE& dae)
{
	domFx_common_float_or_param::domFloatRef ref = new domFx_common_float_or_param::domFloat(dae);
	return ref;
}


daeMetaElement *
domFx_common_float_or_param::domFloat::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float" );
	meta->registerClass(domFx_common_float_or_param::domFloat::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float"));
		ma->setOffset( daeOffsetOf( domFx_common_float_or_param::domFloat , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	//	Add attribute: sid
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( dae.getAtomicTypes().get("Sid"));
		ma->setOffset( daeOffsetOf( domFx_common_float_or_param::domFloat , attrSid ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_common_float_or_param::domFloat));
	meta->validate();

	return meta;
}

daeElementRef
domFx_common_float_or_param::domParam::create(DAE& dae)
{
	domFx_common_float_or_param::domParamRef ref = new domFx_common_float_or_param::domParam(dae);
	return ref;
}


daeMetaElement *
domFx_common_float_or_param::domParam::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "param" );
	meta->registerClass(domFx_common_float_or_param::domParam::create);

	meta->setIsInnerClass( true );

	//	Add attribute: ref
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "ref" );
		ma->setType( dae.getAtomicTypes().get("xsNCName"));
		ma->setOffset( daeOffsetOf( domFx_common_float_or_param::domParam , attrRef ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_common_float_or_param::domParam));
	meta->validate();

	return meta;
}

