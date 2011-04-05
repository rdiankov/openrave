#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domMotion_technique.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domMotion_technique::create(DAE& dae)
{
	domMotion_techniqueRef ref = new domMotion_technique(dae);
	return ref;
}


daeMetaElement *
domMotion_technique::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "motion_technique" );
	meta->registerClass(domMotion_technique::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 0, -1 );
	mea->setName( "axis_info" );
	mea->setOffset( daeOffsetOf(domMotion_technique,elemAxis_info_array) );
	mea->setElementType( domMotion_axis_info::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 0, 1 );
	mea->setName( "effector_info" );
	mea->setOffset( daeOffsetOf(domMotion_technique,elemEffector_info) );
	mea->setElementType( domMotion_effector_info::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 1 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domMotion_technique));
	meta->validate();

	return meta;
}

