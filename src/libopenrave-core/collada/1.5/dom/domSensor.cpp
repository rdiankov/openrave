#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domSensor.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domSensor::create(DAE& dae)
{
	domSensorRef ref = new domSensor(dae);
	return ref;
}


daeMetaElement *
domSensor::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "sensor" );
	meta->registerClass(domSensor::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	cm = new daeMetaAny( meta, cm, 0, 0, -1 );

	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();

	cm->setMaxOrdinal( 0 );
	meta->setCMRoot( cm );

	meta->setAllowsAny( true );
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domSensor,_contents));
	meta->addContentsOrder(daeOffsetOf(domSensor,_contentsOrder));

	//	Add attribute: id
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( dae.getAtomicTypes().get("xsID"));
		ma->setOffset( daeOffsetOf( domSensor , attrId ));
		ma->setContainer( meta );

		meta->appendAttribute(ma);
	}

	//	Add attribute: name
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domSensor , attrName ));
		ma->setContainer( meta );

		meta->appendAttribute(ma);
	}

	//	Add attribute: type
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "type" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domSensor , attrType ));
		ma->setContainer( meta );

		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domSensor));
	meta->validate();

	return meta;
}

