#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domMotion_effector_info.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domMotion_effector_info::create(DAE& dae)
{
	domMotion_effector_infoRef ref = new domMotion_effector_info(dae);
	return ref;
}


daeMetaElement *
domMotion_effector_info::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "motion_effector_info" );
	meta->registerClass(domMotion_effector_info::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 0, -1 );
	mea->setName( "bind" );
	mea->setOffset( daeOffsetOf(domMotion_effector_info,elemBind_array) );
	mea->setElementType( domKinematics_bind::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 1, 0, -1 );
	mea->setName( "newparam" );
	mea->setOffset( daeOffsetOf(domMotion_effector_info,elemNewparam_array) );
	mea->setElementType( domKinematics_newparam::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 2, 0, -1 );
	mea->setName( "setparam" );
	mea->setOffset( daeOffsetOf(domMotion_effector_info,elemSetparam_array) );
	mea->setElementType( domKinematics_setparam::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 3, 0, 1 );
	mea->setName( "speed" );
	mea->setOffset( daeOffsetOf(domMotion_effector_info,elemSpeed) );
	mea->setElementType( domCommon_float2_or_param::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 4, 0, 1 );
	mea->setName( "acceleration" );
	mea->setOffset( daeOffsetOf(domMotion_effector_info,elemAcceleration) );
	mea->setElementType( domCommon_float2_or_param::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 5, 0, 1 );
	mea->setName( "deceleration" );
	mea->setOffset( daeOffsetOf(domMotion_effector_info,elemDeceleration) );
	mea->setElementType( domCommon_float2_or_param::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 6, 0, 1 );
	mea->setName( "jerk" );
	mea->setOffset( daeOffsetOf(domMotion_effector_info,elemJerk) );
	mea->setElementType( domCommon_float2_or_param::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 6 );
	meta->setCMRoot( cm );	

	//	Add attribute: sid
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( dae.getAtomicTypes().get("Sid"));
		ma->setOffset( daeOffsetOf( domMotion_effector_info , attrSid ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: name
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domMotion_effector_info , attrName ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domMotion_effector_info));
	meta->validate();

	return meta;
}

