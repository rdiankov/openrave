#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domJoint_limits.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domJoint_limits::create(DAE& dae)
{
	domJoint_limitsRef ref = new domJoint_limits(dae);
	return ref;
}


daeMetaElement *
domJoint_limits::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "joint_limits" );
	meta->registerClass(domJoint_limits::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 0, 1 );
	mea->setName( "min" );
	mea->setOffset( daeOffsetOf(domJoint_limits,elemMin) );
	mea->setElementType( domMinmax::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 0, 1 );
	mea->setName( "max" );
	mea->setOffset( daeOffsetOf(domJoint_limits,elemMax) );
	mea->setElementType( domMinmax::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 1 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domJoint_limits));
	meta->validate();

	return meta;
}

