#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domImage_source.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domImage_source::create(DAE& dae)
{
	domImage_sourceRef ref = new domImage_source(dae);
	return ref;
}


daeMetaElement *
domImage_source::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "image_source" );
	meta->registerClass(domImage_source::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( meta, cm, 0, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "ref" );
	mea->setOffset( daeOffsetOf(domImage_source,elemRef) );
	mea->setElementType( domImage_source::domRef::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "hex" );
	mea->setOffset( daeOffsetOf(domImage_source,elemHex) );
	mea->setElementType( domImage_source::domHex::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domImage_source,_contents));
	meta->addContentsOrder(daeOffsetOf(domImage_source,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domImage_source,_CMData), 1);
	meta->setElementSize(sizeof(domImage_source));
	meta->validate();

	return meta;
}

daeElementRef
domImage_source::domRef::create(DAE& dae)
{
	domImage_source::domRefRef ref = new domImage_source::domRef(dae);
	ref->_value.setContainer( (domImage_source::domRef*)ref );
	return ref;
}


daeMetaElement *
domImage_source::domRef::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "ref" );
	meta->registerClass(domImage_source::domRef::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domImage_source::domRef , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domImage_source::domRef));
	meta->validate();

	return meta;
}

daeElementRef
domImage_source::domHex::create(DAE& dae)
{
	domImage_source::domHexRef ref = new domImage_source::domHex(dae);
	return ref;
}


daeMetaElement *
domImage_source::domHex::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "hex" );
	meta->registerClass(domImage_source::domHex::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("List_of_hex_binary"));
		ma->setOffset( daeOffsetOf( domImage_source::domHex , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	//	Add attribute: format
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "format" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domImage_source::domHex , attrFormat ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domImage_source::domHex));
	meta->validate();

	return meta;
}

