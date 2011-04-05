#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domCommon_float_or_param.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domCommon_float_or_param::create(DAE& dae)
{
	domCommon_float_or_paramRef ref = new domCommon_float_or_param(dae);
	return ref;
}


daeMetaElement *
domCommon_float_or_param::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "common_float_or_param" );
	meta->registerClass(domCommon_float_or_param::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "float" );
	mea->setOffset( daeOffsetOf(domCommon_float_or_param,elemFloat) );
	mea->setElementType( domCommon_float_or_param::domFloat::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "param" );
	mea->setOffset( daeOffsetOf(domCommon_float_or_param,elemParam) );
	mea->setElementType( domCommon_param::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domCommon_float_or_param,_contents));
	meta->addContentsOrder(daeOffsetOf(domCommon_float_or_param,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domCommon_float_or_param,_CMData), 1);
	meta->setElementSize(sizeof(domCommon_float_or_param));
	meta->validate();

	return meta;
}

daeElementRef
domCommon_float_or_param::domFloat::create(DAE& dae)
{
	domCommon_float_or_param::domFloatRef ref = new domCommon_float_or_param::domFloat(dae);
	return ref;
}


daeMetaElement *
domCommon_float_or_param::domFloat::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "float" );
	meta->registerClass(domCommon_float_or_param::domFloat::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float"));
		ma->setOffset( daeOffsetOf( domCommon_float_or_param::domFloat , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCommon_float_or_param::domFloat));
	meta->validate();

	return meta;
}

