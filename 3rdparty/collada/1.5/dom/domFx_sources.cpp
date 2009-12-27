#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domFx_sources.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domFx_sources::create(DAE& dae)
{
	domFx_sourcesRef ref = new domFx_sources(dae);
	return ref;
}


daeMetaElement *
domFx_sources::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "fx_sources" );
	meta->registerClass(domFx_sources::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	cm = new daeMetaChoice( meta, cm, 0, 0, 0, -1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "inline" );
	mea->setOffset( daeOffsetOf(domFx_sources,elemInline_array) );
	mea->setElementType( domFx_sources::domInline::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "import" );
	mea->setOffset( daeOffsetOf(domFx_sources,elemImport_array) );
	mea->setElementType( domFx_sources::domImport::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();

	cm->setMaxOrdinal( 3000 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domFx_sources,_contents));
	meta->addContentsOrder(daeOffsetOf(domFx_sources,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domFx_sources,_CMData), 1);
	meta->setElementSize(sizeof(domFx_sources));
	meta->validate();

	return meta;
}

daeElementRef
domFx_sources::domInline::create(DAE& dae)
{
	domFx_sources::domInlineRef ref = new domFx_sources::domInline(dae);
	return ref;
}


daeMetaElement *
domFx_sources::domInline::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "inline" );
	meta->registerClass(domFx_sources::domInline::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsString"));
		ma->setOffset( daeOffsetOf( domFx_sources::domInline , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_sources::domInline));
	meta->validate();

	return meta;
}

daeElementRef
domFx_sources::domImport::create(DAE& dae)
{
	domFx_sources::domImportRef ref = new domFx_sources::domImport(dae);
	return ref;
}


daeMetaElement *
domFx_sources::domImport::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "import" );
	meta->registerClass(domFx_sources::domImport::create);

	meta->setIsInnerClass( true );

	//	Add attribute: ref
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "ref" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domFx_sources::domImport , attrRef ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domFx_sources::domImport));
	meta->validate();

	return meta;
}

