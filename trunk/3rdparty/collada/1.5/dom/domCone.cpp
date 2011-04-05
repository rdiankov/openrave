#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domCone.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domCone::create(DAE& dae)
{
	domConeRef ref = new domCone(dae);
	return ref;
}


daeMetaElement *
domCone::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "cone" );
	meta->registerClass(domCone::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "radius" );
	mea->setOffset( daeOffsetOf(domCone,elemRadius) );
	mea->setElementType( domCone::domRadius::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 1, 1 );
	mea->setName( "angle" );
	mea->setOffset( daeOffsetOf(domCone,elemAngle) );
	mea->setElementType( domCone::domAngle::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 2, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domCone,elemExtra_array) );
	mea->setElementType( domExtra::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 2 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domCone));
	meta->validate();

	return meta;
}

daeElementRef
domCone::domRadius::create(DAE& dae)
{
	domCone::domRadiusRef ref = new domCone::domRadius(dae);
	return ref;
}


daeMetaElement *
domCone::domRadius::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "radius" );
	meta->registerClass(domCone::domRadius::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("Float"));
		ma->setOffset( daeOffsetOf( domCone::domRadius , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCone::domRadius));
	meta->validate();

	return meta;
}

daeElementRef
domCone::domAngle::create(DAE& dae)
{
	domCone::domAngleRef ref = new domCone::domAngle(dae);
	return ref;
}


daeMetaElement *
domCone::domAngle::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "angle" );
	meta->registerClass(domCone::domAngle::create);

	meta->setIsInnerClass( true );
	//	Add attribute: _value
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( dae.getAtomicTypes().get("xsFloat"));
		ma->setOffset( daeOffsetOf( domCone::domAngle , _value ));
		ma->setContainer( meta );
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domCone::domAngle));
	meta->validate();

	return meta;
}

