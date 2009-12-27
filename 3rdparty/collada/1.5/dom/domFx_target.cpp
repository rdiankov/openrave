#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domFx_target.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domFx_target::create(DAE& dae)
{
	domFx_targetRef ref = new domFx_target(dae);
	return ref;
}


daeMetaElement *
domFx_target::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "fx_target" );
	meta->registerClass(domFx_target::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 0, 1 );
	mea->setName( "binary" );
	mea->setOffset( daeOffsetOf(domFx_target,elemBinary) );
	mea->setElementType( domFx_target::domBinary::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	

	//	Add attribute: platform
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "platform" );
		ma->setType( dae.getAtomicTypes().get("xsString"));
		ma->setOffset( daeOffsetOf( domFx_target , attrPlatform ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: target
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "target" );
		ma->setType( dae.getAtomicTypes().get("xsString"));
		ma->setOffset( daeOffsetOf( domFx_target , attrTarget ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: options
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "options" );
		ma->setType( dae.getAtomicTypes().get("xsString"));
		ma->setOffset( daeOffsetOf( domFx_target , attrOptions ));
		ma->setContainer( meta );
		ma->setIsRequired( false );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_target));
	meta->validate();

	return meta;
}

daeElementRef
domFx_target::domBinary::create(DAE& dae)
{
	domFx_target::domBinaryRef ref = new domFx_target::domBinary(dae);
	return ref;
}


daeMetaElement *
domFx_target::domBinary::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "binary" );
	meta->registerClass(domFx_target::domBinary::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "ref" );
	mea->setOffset( daeOffsetOf(domFx_target::domBinary,elemRef) );
	mea->setElementType( domFx_target::domBinary::domRef::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "hex" );
	mea->setOffset( daeOffsetOf(domFx_target::domBinary,elemHex) );
	mea->setElementType( domFx_target::domBinary::domHex::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domFx_target::domBinary,_contents));
	meta->addContentsOrder(daeOffsetOf(domFx_target::domBinary,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domFx_target::domBinary,_CMData), 1);
	meta->setElementSize(sizeof(domFx_target::domBinary));
	meta->validate();

	return meta;
}

daeElementRef
domFx_target::domBinary::domRef::create(DAE& dae)
{
	domFx_target::domBinary::domRefRef ref = new domFx_target::domBinary::domRef(dae);
	ref->_value.setContainer( (domFx_target::domBinary::domRef*)ref );
	return ref;
}


daeMetaElement *
domFx_target::domBinary::domRef::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "ref" );
	meta->registerClass(domFx_target::domBinary::domRef::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domFx_target::domBinary::domRef , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_target::domBinary::domRef));
	meta->validate();

	return meta;
}

daeElementRef
domFx_target::domBinary::domHex::create(DAE& dae)
{
	domFx_target::domBinary::domHexRef ref = new domFx_target::domBinary::domHex(dae);
	return ref;
}


daeMetaElement *
domFx_target::domBinary::domHex::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "hex" );
	meta->registerClass(domFx_target::domBinary::domHex::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("List_of_hex_binary"));
		ma->setOffset( daeOffsetOf( domFx_target::domBinary::domHex , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	//	Add attribute: format
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "format" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domFx_target::domBinary::domHex , attrFormat ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_target::domBinary::domHex));
	meta->validate();

	return meta;
}

