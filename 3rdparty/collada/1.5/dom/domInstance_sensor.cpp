#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domInstance_sensor.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domInstance_sensor::create(DAE& dae)
{
	domInstance_sensorRef ref = new domInstance_sensor(dae);
	return ref;
}


daeMetaElement *
domInstance_sensor::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "instance_sensor" );
	meta->registerClass(domInstance_sensor::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 0, 1 );
	mea->setName( "translate" );
	mea->setOffset( daeOffsetOf(domInstance_sensor,elemTranslate) );
	mea->setElementType( domTranslate::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 0, 1 );
	mea->setName( "rotate" );
	mea->setOffset( daeOffsetOf(domInstance_sensor,elemRotate) );
	mea->setElementType( domRotate::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 1 );
	meta->setCMRoot( cm );
	//	Add attribute: id
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( dae.getAtomicTypes().get("xsID"));
		ma->setOffset( daeOffsetOf( domInstance_sensor , attrId ));
		ma->setContainer( meta );

		meta->appendAttribute(ma);
	}

	//	Add attribute: name
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domInstance_sensor , attrName ));
		ma->setContainer( meta );

		meta->appendAttribute(ma);
	}

	//	Add attribute: type
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "link" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domInstance_sensor , attrLink ));
		ma->setContainer( meta );

		meta->appendAttribute(ma);
	}

	//	Add attribute: url
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "url" );
		ma->setType( dae.getAtomicTypes().get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domInstance_sensor , attrUrl ));
		ma->setContainer( meta );
		ma->setIsRequired( true );

		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domInstance_sensor));
	meta->validate();

	return meta;
}
