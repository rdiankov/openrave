#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domKinematics_axis_info.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domKinematics_axis_info::create(DAE& dae)
{
	domKinematics_axis_infoRef ref = new domKinematics_axis_info(dae);
	return ref;
}


daeMetaElement *
domKinematics_axis_info::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "kinematics_axis_info" );
	meta->registerClass(domKinematics_axis_info::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 0, -1 );
	mea->setName( "newparam" );
	mea->setOffset( daeOffsetOf(domKinematics_axis_info,elemNewparam_array) );
	mea->setElementType( domKinematics_newparam::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 0, 1 );
	mea->setName( "active" );
	mea->setOffset( daeOffsetOf(domKinematics_axis_info,elemActive) );
	mea->setElementType( domCommon_bool_or_param::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 2, 0, 1 );
	mea->setName( "locked" );
	mea->setOffset( daeOffsetOf(domKinematics_axis_info,elemLocked) );
	mea->setElementType( domCommon_bool_or_param::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 3, 0, -1 );
	mea->setName( "index" );
	mea->setOffset( daeOffsetOf(domKinematics_axis_info,elemIndex_array) );
	mea->setElementType( domKinematics_index::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 4, 0, 1 );
	mea->setName( "limits" );
	mea->setOffset( daeOffsetOf(domKinematics_axis_info,elemLimits) );
	mea->setElementType( domKinematics_limits::registerElement(dae) );
	cm->appendChild( mea );

	cm = new daeMetaChoice( meta, cm, 0, 5, 0, -1 );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "formula" );
	mea->setOffset( daeOffsetOf(domKinematics_axis_info,elemFormula_array) );
	mea->setElementType( domFormula::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementArrayAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "instance_formula" );
	mea->setOffset( daeOffsetOf(domKinematics_axis_info,elemInstance_formula_array) );
	mea->setElementType( domInstance_formula::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();

	cm->setMaxOrdinal( 3005 );
	meta->setCMRoot( cm );	
	// Ordered list of sub-elements
	meta->addContents(daeOffsetOf(domKinematics_axis_info,_contents));
	meta->addContentsOrder(daeOffsetOf(domKinematics_axis_info,_contentsOrder));

	meta->addCMDataArray(daeOffsetOf(domKinematics_axis_info,_CMData), 1);
	//	Add attribute: sid
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( dae.getAtomicTypes().get("Sid"));
		ma->setOffset( daeOffsetOf( domKinematics_axis_info , attrSid ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: name
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domKinematics_axis_info , attrName ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	//	Add attribute: axis
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "axis" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domKinematics_axis_info , attrAxis ));
		ma->setContainer( meta );
		ma->setIsRequired( true );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domKinematics_axis_info));
	meta->validate();

	return meta;
}

