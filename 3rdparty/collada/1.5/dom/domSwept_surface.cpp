#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domSwept_surface.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domSwept_surface::create(DAE& dae)
{
	domSwept_surfaceRef ref = new domSwept_surface(dae);
	return ref;
}


daeMetaElement *
domSwept_surface::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "swept_surface" );
	meta->registerClass(domSwept_surface::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "curve" );
	mea->setOffset( daeOffsetOf(domSwept_surface,elemCurve) );
	mea->setElementType( domCurve::registerElement(dae) );
	cm->appendChild( mea );

	cm = new daeMetaChoice( meta, cm, 0, 1, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "direction" );
	mea->setOffset( daeOffsetOf(domSwept_surface,elemDirection) );
	mea->setElementType( domSwept_surface::domDirection::registerElement(dae) );
	cm->appendChild( mea );

	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "origin" );
	mea->setOffset( daeOffsetOf(domSwept_surface,elemOrigin) );
	mea->setElementType( domSwept_surface::domOrigin::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 1, 1 );
	mea->setName( "axis" );
	mea->setOffset( daeOffsetOf(domSwept_surface,elemAxis) );
	mea->setElementType( domSwept_surface::domAxis::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 1 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();

	cm->setMaxOrdinal( 1 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();

	mea = new daeMetaElementArrayAttribute( meta, cm, 2, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domSwept_surface,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 2 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domSwept_surface,_contents));
	meta->addContentsOrder(daeOffsetOf(domSwept_surface,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domSwept_surface,_CMData), 1);
	meta->setElementSize(sizeof(domSwept_surface));
	meta->validate();

	return meta;
}

daeElementRef
domSwept_surface::domDirection::create(DAE& dae)
{
	domSwept_surface::domDirectionRef ref = new domSwept_surface::domDirection(dae);
	return ref;
}


daeMetaElement *
domSwept_surface::domDirection::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "direction" );
	meta->registerClass(domSwept_surface::domDirection::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3"));
		ma->setOffset( daeOffsetOf( domSwept_surface::domDirection , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domSwept_surface::domDirection));
	meta->validate();

	return meta;
}

daeElementRef
domSwept_surface::domOrigin::create(DAE& dae)
{
	domSwept_surface::domOriginRef ref = new domSwept_surface::domOrigin(dae);
	return ref;
}


daeMetaElement *
domSwept_surface::domOrigin::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "origin" );
	meta->registerClass(domSwept_surface::domOrigin::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3"));
		ma->setOffset( daeOffsetOf( domSwept_surface::domOrigin , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domSwept_surface::domOrigin));
	meta->validate();

	return meta;
}

daeElementRef
domSwept_surface::domAxis::create(DAE& dae)
{
	domSwept_surface::domAxisRef ref = new domSwept_surface::domAxis(dae);
	return ref;
}


daeMetaElement *
domSwept_surface::domAxis::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "axis" );
	meta->registerClass(domSwept_surface::domAxis::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3"));
		ma->setOffset( daeOffsetOf( domSwept_surface::domAxis , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domSwept_surface::domAxis));
	meta->validate();

	return meta;
}

