#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domCommon_int_or_param.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domCommon_int_or_param::create(DAE& dae)
{
	domCommon_int_or_paramRef ref = new domCommon_int_or_param(dae);
	return ref;
}


daeMetaElement *
domCommon_int_or_param::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "common_int_or_param" );
	meta->registerClass(domCommon_int_or_param::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "int" );
	mea->setOffset( daeOffsetOf(domCommon_int_or_param,elemInt) );
	mea->setElementType( domCommon_int_or_param::domInt::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "param" );
	mea->setOffset( daeOffsetOf(domCommon_int_or_param,elemParam) );
	mea->setElementType( domCommon_param::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domCommon_int_or_param,_contents));
	meta->addContentsOrder(daeOffsetOf(domCommon_int_or_param,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domCommon_int_or_param,_CMData), 1);
	meta->setElementSize(sizeof(domCommon_int_or_param));
	meta->validate();

	return meta;
}

daeElementRef
domCommon_int_or_param::domInt::create(DAE& dae)
{
	domCommon_int_or_param::domIntRef ref = new domCommon_int_or_param::domInt(dae);
	return ref;
}


daeMetaElement *
domCommon_int_or_param::domInt::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "int" );
	meta->registerClass(domCommon_int_or_param::domInt::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Int"));
		ma->setOffset( daeOffsetOf( domCommon_int_or_param::domInt , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCommon_int_or_param::domInt));
	meta->validate();

	return meta;
}

