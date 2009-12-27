#include <dae.h>
#include <dae/daeDom.h>
#include <dom/domBind_joint_axis.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domBind_joint_axis::create(DAE& dae)
{
	domBind_joint_axisRef ref = new domBind_joint_axis(dae);
	return ref;
}


daeMetaElement *
domBind_joint_axis::registerElement(DAE& dae)
{
	daeMetaElement* meta = dae.getMeta(ID());
	if ( meta != NULL ) return meta;

	meta = new daeMetaElement(dae);
	dae.setMeta(ID(), *meta);
	meta->setName( "bind_joint_axis" );
	meta->registerClass(domBind_joint_axis::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( meta, cm, 0, 1, 1 );
	mea->setName( "axis" );
	mea->setOffset( daeOffsetOf(domBind_joint_axis,elemAxis) );
	mea->setElementType( domCommon_sidref_or_param::registerElement(dae) );
	cm->appendChild( mea );

	mea = new daeMetaElementAttribute( meta, cm, 1, 1, 1 );
	mea->setName( "value" );
	mea->setOffset( daeOffsetOf(domBind_joint_axis,elemValue) );
	mea->setElementType( domCommon_float_or_param::registerElement(dae) );
	cm->appendChild( mea );

	cm->setMaxOrdinal( 1 );
	meta->setCMRoot( cm );	

	//	Add attribute: target
	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "target" );
		ma->setType( dae.getAtomicTypes().get("xsToken"));
		ma->setOffset( daeOffsetOf( domBind_joint_axis , attrTarget ));
		ma->setContainer( meta );
	
		meta->appendAttribute(ma);
	}

	meta->setElementSize(sizeof(domBind_joint_axis));
	meta->validate();

	return meta;
}

