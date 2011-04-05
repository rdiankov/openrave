#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domLink.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domLink::create(DAE& dae)
{
	domLinkRef ref = new domLink(dae);
	return ref;
}


daeMetaElement *
domLink::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "link" );
	meta->registerClass(domLink::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	cm = new daeMetaChoice( meta, cm, 0, 0, 0, -1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "rotate" );
	mea->setOffset( daeOffsetOf(domLink,elemRotate_array) );
	mea->setElementType( domRotate::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "translate" );
	mea->setOffset( daeOffsetOf(domLink,elemTranslate_array) );
	mea->setElementType( domTranslate::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();

	cm = new daeMetaChoice( meta, cm, 1, 3001, 0, -1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "attachment_full" );
	mea->setOffset( daeOffsetOf(domLink,elemAttachment_full_array) );
	mea->setElementType( domLink::domAttachment_full::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "attachment_start" );
	mea->setOffset( daeOffsetOf(domLink,elemAttachment_start_array) );
	mea->setElementType( domLink::domAttachment_start::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "attachment_end" );
	mea->setOffset( daeOffsetOf(domLink,elemAttachment_end_array) );
	mea->setElementType( domLink::domAttachment_end::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();

	cm->setMaxOrdinal( 6001 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domLink,_contents));
	meta->addContentsOrder(daeOffsetOf(domLink,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domLink,_CMData), 2);
	//	Add attribute: sid
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( dae.getAtomicTypes().get("Sid"));
		ma->setOffset( daeOffsetOf( domLink , attrSid ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: name
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domLink , attrName ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domLink));
	meta->validate();

	return meta;
}

daeElementRef
domLink::domAttachment_full::create(DAE& dae)
{
	domLink::domAttachment_fullRef ref = new domLink::domAttachment_full(dae);
	return ref;
}


daeMetaElement *
domLink::domAttachment_full::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "attachment_full" );
	meta->registerClass(domLink::domAttachment_full::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	cm = new daeMetaChoice( meta, cm, 0, 0, 0, -1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "rotate" );
	mea->setOffset( daeOffsetOf(domLink::domAttachment_full,elemRotate_array) );
	mea->setElementType( domRotate::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "translate" );
	mea->setOffset( daeOffsetOf(domLink::domAttachment_full,elemTranslate_array) );
	mea->setElementType( domTranslate::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();

	mea = new daeMetaElementAttribute( meta, cm, 3001, 1, 1 );
	mea->setName( "link" );
	mea->setOffset( daeOffsetOf(domLink::domAttachment_full,elemLink) );
	mea->setElementType( domLink::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 3001 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domLink::domAttachment_full,_contents));
	meta->addContentsOrder(daeOffsetOf(domLink::domAttachment_full,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domLink::domAttachment_full,_CMData), 1);
	//	Add attribute: joint
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "joint" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domLink::domAttachment_full , attrJoint ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domLink::domAttachment_full));
	meta->validate();

	return meta;
}

daeElementRef
domLink::domAttachment_start::create(DAE& dae)
{
	domLink::domAttachment_startRef ref = new domLink::domAttachment_start(dae);
	return ref;
}


daeMetaElement *
domLink::domAttachment_start::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "attachment_start" );
	meta->registerClass(domLink::domAttachment_start::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( meta, cm, 0, 0, 1, -1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "rotate" );
	mea->setOffset( daeOffsetOf(domLink::domAttachment_start,elemRotate_array) );
	mea->setElementType( domRotate::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "translate" );
	mea->setOffset( daeOffsetOf(domLink::domAttachment_start,elemTranslate_array) );
	mea->setElementType( domTranslate::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 3000 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domLink::domAttachment_start,_contents));
	meta->addContentsOrder(daeOffsetOf(domLink::domAttachment_start,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domLink::domAttachment_start,_CMData), 1);
	//	Add attribute: joint
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "joint" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domLink::domAttachment_start , attrJoint ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domLink::domAttachment_start));
	meta->validate();

	return meta;
}

daeElementRef
domLink::domAttachment_end::create(DAE& dae)
{
	domLink::domAttachment_endRef ref = new domLink::domAttachment_end(dae);
	return ref;
}


daeMetaElement *
domLink::domAttachment_end::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "attachment_end" );
	meta->registerClass(domLink::domAttachment_end::create);

	meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( meta, cm, 0, 0, 0, -1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "rotate" );
	mea->setOffset( daeOffsetOf(domLink::domAttachment_end,elemRotate_array) );
	mea->setElementType( domRotate::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "translate" );
	mea->setOffset( daeOffsetOf(domLink::domAttachment_end,elemTranslate_array) );
	mea->setElementType( domTranslate::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 3000 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domLink::domAttachment_end,_contents));
	meta->addContentsOrder(daeOffsetOf(domLink::domAttachment_end,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domLink::domAttachment_end,_CMData), 1);
	//	Add attribute: joint
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "joint" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domLink::domAttachment_end , attrJoint ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domLink::domAttachment_end));
	meta->validate();

	return meta;
}

