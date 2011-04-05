#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domKinematics_limits.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domKinematics_limits::create(DAE& dae)
{
	domKinematics_limitsRef ref = new domKinematics_limits(dae);
	return ref;
}


daeMetaElement *
domKinematics_limits::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "kinematics_limits" );
	meta->registerClass(domKinematics_limits::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "min" );
	mea->setOffset( daeOffsetOf(domKinematics_limits,elemMin) );
	mea->setElementType( domCommon_float_or_param::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 1, 1 );
	mea->setName( "max" );
	mea->setOffset( daeOffsetOf(domKinematics_limits,elemMax) );
	mea->setElementType( domCommon_float_or_param::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 1 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domKinematics_limits));
	meta->validate();

	return meta;
}

