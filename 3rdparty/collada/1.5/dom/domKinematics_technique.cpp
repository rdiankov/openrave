#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domKinematics_technique.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domKinematics_technique::create(DAE& dae)
{
	domKinematics_techniqueRef ref = new domKinematics_technique(dae);
	return ref;
}


daeMetaElement *
domKinematics_technique::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "kinematics_technique" );
	meta->registerClass(domKinematics_technique::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 0, -1 );
	mea->setName( "axis_info" );
	mea->setOffset( daeOffsetOf(domKinematics_technique,elemAxis_info_array) );
	mea->setElementType( domKinematics_axis_info::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 1, 1 );
	mea->setName( "frame_origin" );
	mea->setOffset( daeOffsetOf(domKinematics_technique,elemFrame_origin) );
	mea->setElementType( domKinematics_frame::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 2, 1, 1 );
	mea->setName( "frame_tip" );
	mea->setOffset( daeOffsetOf(domKinematics_technique,elemFrame_tip) );
	mea->setElementType( domKinematics_frame::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 3, 0, 1 );
	mea->setName( "frame_tcp" );
	mea->setOffset( daeOffsetOf(domKinematics_technique,elemFrame_tcp) );
	mea->setElementType( domKinematics_frame::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 4, 0, 1 );
	mea->setName( "frame_object" );
	mea->setOffset( daeOffsetOf(domKinematics_technique,elemFrame_object) );
	mea->setElementType( domKinematics_frame::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 4 );
	meta->setCMRoot( cm );	

	meta->setElementSize(sizeof(domKinematics_technique));
	meta->validate();

	return meta;
}

