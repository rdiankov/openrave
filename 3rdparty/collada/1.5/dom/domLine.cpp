#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domLine.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domLine::create(DAE& dae)
{
	domLineRef ref = new domLine(dae);
	return ref;
}


daeMetaElement *
domLine::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "line" );
	meta->registerClass(domLine::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "origin" );
	mea->setOffset( daeOffsetOf(domLine,elemOrigin) );
	mea->setElementType( domLine::domOrigin::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 1, 1 );
	mea->setName( "direction" );
	mea->setOffset( daeOffsetOf(domLine,elemDirection) );
	mea->setElementType( domLine::domDirection::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 2, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domLine,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 2 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domLine));
	meta->validate();

	return meta;
}

daeElementRef
domLine::domOrigin::create(DAE& dae)
{
	domLine::domOriginRef ref = new domLine::domOrigin(dae);
	return ref;
}


daeMetaElement *
domLine::domOrigin::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "origin" );
	meta->registerClass(domLine::domOrigin::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3"));
		ma->setOffset( daeOffsetOf( domLine::domOrigin , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domLine::domOrigin));
	meta->validate();

	return meta;
}

daeElementRef
domLine::domDirection::create(DAE& dae)
{
	domLine::domDirectionRef ref = new domLine::domDirection(dae);
	return ref;
}


daeMetaElement *
domLine::domDirection::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "direction" );
	meta->registerClass(domLine::domDirection::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float3"));
		ma->setOffset( daeOffsetOf( domLine::domDirection , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domLine::domDirection));
	meta->validate();

	return meta;
}

