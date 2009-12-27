#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domAxis_constraint.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domAxis_constraint::create(DAE& dae)
{
	domAxis_constraintRef ref = new domAxis_constraint(dae);
	return ref;
}


daeMetaElement *
domAxis_constraint::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "axis_constraint" );
	meta->registerClass(domAxis_constraint::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "axis" );
	mea->setOffset( daeOffsetOf(domAxis_constraint,elemAxis) );
	mea->setElementType( domAxis::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 0, 1 );
	mea->setName( "limits" );
	mea->setOffset( daeOffsetOf(domAxis_constraint,elemLimits) );
	mea->setElementType( domJoint_limits::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 1 );
	meta->setCMRoot( cm );	

	//	Add attribute: sid
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( dae.getAtomicTypes().get("Sid"));
		ma->setOffset( daeOffsetOf( domAxis_constraint , attrSid ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domAxis_constraint));
	meta->validate();

	return meta;
}

